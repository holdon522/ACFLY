#include "M32_PosCtrl.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "StorageSystem.hpp"
#include "Missions.hpp"
#include "NavCmdProcess.hpp"
#include "InFlightCmdProcess.hpp"
#include "drv_PWMOut.hpp"
#include "AuxFuncs.hpp"
#include "ctrl_Main.hpp"
#include "Filters_LP.hpp"

M32_PosCtrl::M32_PosCtrl():Mode_Base( "PosCtrl", 32 )
{
	
}

void M32_PosCtrl::get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode )
{	//获取飞行模式
	if( get_Position_MSStatus() != MS_Ready )
	{
		*mode = AFunc_AltHold;
		return;
	}
	
	if( rc.available )
	{	//接收机可用更新模式按钮状态		
		uint8_t MF_mode = *mode;
		
		uint8_t new_btn_zones[4];
		new_btn_zones[0] = get_RcButtonZone( rc.data[4], btn_zones[0] );
		new_btn_zones[1] = get_RcButtonZone( rc.data[5], btn_zones[1] );
		new_btn_zones[2] = get_RcButtonZone( rc.data[6], btn_zones[2] );
		new_btn_zones[3] = get_RcButtonZone( rc.data[7], btn_zones[3] );
		
		if( new_btn_zones[0]<=5 && new_btn_zones[0]!=btn_zones[0] )
		{	//模式按钮改变更改模式
			MF_mode = cfg.Bt1AFunc1[8*new_btn_zones[0]];			
		}
		
		/*判断执行任务*/
			if( cfg.MissionBt[0]>=2 && cfg.MissionBt[0]<=4 )
			{	//按钮按下执行任务
				if( rc.available_channels >= cfg.MissionBt[0]+4 )
				{
					uint8_t new_btn_zone = new_btn_zones[cfg.MissionBt[0]-1];
					uint8_t old_btn_zone = btn_zones[cfg.MissionBt[0]-1];
					if( new_btn_zone != old_btn_zone )
					{	//按钮状态发生变化
						if( new_btn_zone>=4 )
							MF_mode = AFunc_Mission;
						else if( old_btn_zone<=5 )
							MF_mode = 0;
					}
				}
			}
			else if( cfg.MissionBt[0]>=12 && cfg.MissionBt[0]<=14 )
			{	//按钮变化执行任务
				if( rc.available_channels >= cfg.MissionBt[0]-10+4 )
				{
					//获取按钮状态
					uint8_t new_btn_zone = new_btn_zones[cfg.MissionBt[0]-11];
					uint8_t old_btn_zone = btn_zones[cfg.MissionBt[0]-11];
					if( old_btn_zone<=5 && new_btn_zone!=old_btn_zone )
					{	//按钮状态发生变化
						if( *mode != AFunc_Mission )
							MF_mode = AFunc_Mission;
						else
							MF_mode = 0;
					}
				}
			}
		/*判断执行任务*/
		
		/*判断返航*/
			if( cfg.RTLBt[0]>=2 && cfg.RTLBt[0]<=4 )
			{	//按钮按下返航
				if( rc.available_channels >= cfg.RTLBt[0]+4 )
				{
					uint8_t new_btn_zone = new_btn_zones[cfg.RTLBt[0]-1];
					uint8_t old_btn_zone = btn_zones[cfg.RTLBt[0]-1];	
					if( new_btn_zone!=old_btn_zone )
					{	//按钮状态发生变化	
						if( new_btn_zone>=4 )
							MF_mode = AFunc_RTL;
						else if( old_btn_zone<=5 )
							MF_mode = 0;
					}
				}
			}
			else if( cfg.RTLBt[0]>=12 && cfg.RTLBt[0]<=14 )
			{	//按钮变化返航
				if( rc.available_channels >= cfg.RTLBt[0]-10+4 )
				{
					//获取按钮状态
					uint8_t new_btn_zone = new_btn_zones[cfg.RTLBt[0]-11];
					uint8_t old_btn_zone = btn_zones[cfg.RTLBt[0]-11];
					if( old_btn_zone<=5 && new_btn_zone!=old_btn_zone )
					{	//按钮状态发生变化
						if( *mode != AFunc_RTL )
							MF_mode = AFunc_RTL;
						else
							MF_mode = 0;
					}
				}
			}
		/*判断返航*/
		if( MF_mode == 0 )
			MF_mode = cfg.Bt1AFunc1[8*new_btn_zones[0]];
		
		*mode = (AFunc)MF_mode;
		
		btn_zones[0] = new_btn_zones[0];
		btn_zones[1] = new_btn_zones[1];
		btn_zones[2] = new_btn_zones[2];
		btn_zones[3] = new_btn_zones[3];
	}
	else
	{
		*mode = AFunc_RTL;
	}
}

ModeResult M32_PosCtrl::main_func( void* param1, uint32_t param2 )
{
	set_mav_mode_arm();
	
	double freq = 50;
	double h = 1.0/freq;
	setLedMode(LEDMode_Flying1);
	Altitude_Control_Enable();
	Position_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	uint32_t RollOverProtectCounter = 0;
	
	//读取模式配置
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	
	//读取初始航线信息
	CurrentWpInf currentWpInf;
	ReadParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0 );
	
	if( MFunc_cfg.RstWp0[0] != 0 )
		//需要解锁初始化航点为0
		setCurrentMission(0);

	//Loiter模式
	#define LoiterAccFilterFreq 3
	vector3<double> LoiterAcc;
	vector3<double> lastLoiterTVel;
	Filter_LP_IIR_1 LoiterAccFilters[3];
	LoiterAccFilters[0].set_cutoff_frequency(freq, LoiterAccFilterFreq);
	LoiterAccFilters[1].set_cutoff_frequency(freq, LoiterAccFilterFreq);
	LoiterAccFilters[2].set_cutoff_frequency(freq, LoiterAccFilterFreq);
	
	//任务模式
	bool mode_switched = true;
	#define change_Mode(x) {*resMode=cMode=x; mode_switched = true;}
	uint16_t current_mission_ind;
	MissionInf current_mission_inf;
	uint8_t ModeButtonZone = 255;
	uint8_t MissionButtonZone = 255;
	uint8_t RTLButtonZone = 255;
	uint8_t rcv_Mode = 255;
	TIME rcvTIME(false);
	AFunc cMode = AFunc_PosHold;
	AFunc* resMode;
	if(param1)
	{
		resMode = (AFunc*)param1;
		cMode = *resMode;
	}
	else
		resMode = &cMode;
	
	//任务状态机
	NavCmdInf navInf; 
	init_NavCmdInf(&navInf);
	//指令执行是否完成（手动模式）
	bool ManualModeNavCmdInprogress = false;
	ModeMsg ManualModeNavCmd;
	//是否需要更新当前任务存储
	bool mission_updated = false;
	
	//任务模式飞到上次坐标点状态
	uint8_t MissionMode_BackToLastWp = 0;
	
	//是否处理inFlightCmd
	#define DealInFlightCmd 0
	//下一个任务递增量（中间的InFlightCmd个数）
	#define MissionInc 1
	//定距拍照当前距离倍数
	#define CamTriggDistMult 2
	
	//Offboard模式临时变量
	TIME Offboard_ThrTIME(false);
	TIME Offboard_YawTIME(false);
	TIME Offboard_PitRolTIME(false);
	
	//初始化相机触发距离
	CamTriggDist = 0;
	double camTriggDist = CamTriggDist;
		
	//初始化Aux处理
	init_process_AuxFuncs();
		
	while(1)
	{
		os_delay(h);
		
		//获取接收机
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//处理Aux通道
		process_AuxFuncs(&rc);
		
		//获取消息
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		uint8_t msg_handled = 0;	//1-返回acceped 2-返回denied
		
		bool inFlight;
		get_is_inFlight(&inFlight);
		
		if( msg_available && msg.cmd==MAV_CMD_COMPONENT_ARM_DISARM )
		{	//地面站加锁
			if( msg.params[0] == 0 )
			{
				if( (msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK )
				{
					Attitude_Control_Disable();
					set_mav_mode_disarm();
					os_delay(1.0);
					
					uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
					const Port* port = get_CommuPort( port_index );
					if( port->write )
					{
						mavlink_message_t msg_sd;
						if( mavlink_lock_chan( port_index, 0.01 ) )
						{
							mavlink_msg_command_ack_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								port_index ,
								&msg_sd,
								msg.cmd,	//command
								MAV_RESULT_ACCEPTED ,	//result
								100 ,	//progress
								0 ,	//param2
								msg.sd_sysid ,	//target system
								msg.sd_compid //target component
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(port_index);
						}
					}
				}
				//保存当前航点信息
				if( mission_updated )
					UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
				return MR_OK;
			}
			msg_handled = 1;
		}
		if( get_CrashedState() )
		{	//侧翻加锁
			Attitude_Control_Disable();
			//保存当前航点信息
			if( mission_updated )
				UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
			return MR_Err;
		}
		

		uint8_t reqMode = cMode;
		if( msg_available && msg.cmd==176 )
		{	//指令更改模式
			if( (int)msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED )
			{	//mavlink定义模式
				uint32_t main_mode = msg.params[1];
				uint32_t sub_mode = msg.params[2];
				if( main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL )
				{	//指令进入手动
					reqMode = AFunc_PosHold;
					msg_handled = 1;
				}
				else if( main_mode==PX4_CUSTOM_MAIN_MODE_ALTCTL )
				{	//指令进入手动
					reqMode = AFunc_AltHold;
					msg_handled = 1;
				}
				else if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
				{	//指令进入任务模式
					reqMode = AFunc_Mission;
					msg_handled = 1;
				}
				else if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_RTL )
				{	//指令进入返航模式
					reqMode = AFunc_RTL;
					msg_handled = 1;
				}
				else if( main_mode==PX4_CUSTOM_MAIN_MODE_OFFBOARD )
				{	//指令进入OFFBOARD模式
					reqMode = AFunc_Offboard;
					msg_handled = 1;
				}
			}
		}
		
		
		if( rc.available )
		{	//接收机可用
			
			uint8_t new_ModeButtonZone = get_RcButtonZone( rc.data[4], ModeButtonZone );
			if( ModeButtonZone<=5 && new_ModeButtonZone!=ModeButtonZone )
			{	//模式按钮改变更改模式
				reqMode = MFunc_cfg.Bt1AFunc1[8*new_ModeButtonZone];			
			}
			ModeButtonZone = new_ModeButtonZone;
			
			//使用遥控器更新飞行模式
			bool sticks_in_neutral = 
				in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
			if( !sticks_in_neutral )
			{	//摇杆没回中不允许自动操作
				if( is_AFunc_auto(cMode) )
				{
					if( is_AFunc_auto(MFunc_cfg.Bt1AFunc1[8*ModeButtonZone]) == false )
						reqMode = MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];
					else
						reqMode = AFunc_PosHold;
					MissionButtonZone = RTLButtonZone = 255;
					rcv_Mode = cMode;
					rcvTIME = TIME::now();
				}
				if( rcv_Mode != 255 )
				{	//手动操作时间过长退出恢复
					if( rcvTIME.get_pass_time() > 4.5 )
						rcv_Mode = 255;
				}
			}
			else
			{	//摇杆回中可执行自动操作	
				
				/*回舵恢复模式*/
					bool rcv = false;
					if( rcv_Mode != 255 )
					{
						reqMode = rcv_Mode;
						rcv = true;
						rcv_Mode = 255;
					}
				/*回舵恢复模式*/
				
				/*判断执行任务*/
					if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
					{	//按钮按下执行任务
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]+4 )
						{			
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-1+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );									
							if( new_MissionButtonZone!=MissionButtonZone && !rcv )
							{	//按钮状态发生变化
								if( new_MissionButtonZone>=4 )
									reqMode = AFunc_Mission;
								else if( MissionButtonZone<=5 )
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
					else if( MFunc_cfg.MissionBt[0]>=12 && MFunc_cfg.MissionBt[0]<=14 )
					{	//按钮变化执行任务
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]-10+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-11+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );
							if( MissionButtonZone<=5 && new_MissionButtonZone!=MissionButtonZone && !rcv )
							{	//按钮状态发生变化
								if( cMode != AFunc_Mission )
									reqMode = AFunc_Mission;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
				/*判断执行任务*/
				
				/*判断返航*/
					if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
					{	//按钮按下返航
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-1+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );		
							if( new_RTLButtonZone!=RTLButtonZone && !rcv )
							{	//按钮状态发生变化	
								if( new_RTLButtonZone>=4 )
									reqMode = AFunc_RTL;
								else if( RTLButtonZone<=5 )
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
					else if( MFunc_cfg.RTLBt[0]>=12 && MFunc_cfg.RTLBt[0]<=14 )
					{	//按钮变化返航
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]-10+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-11+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );	
							if( RTLButtonZone<=5 && new_RTLButtonZone!=RTLButtonZone && !rcv )
							{	//按钮状态发生变化
								if( cMode != AFunc_RTL )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
				/*判断返航*/
					
				if( reqMode == 0 )
				{	//有按钮松开重新检测按钮位置
					
					/*判断执行任务*/
						if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
						{	//按钮按下执行任务
							if( MissionButtonZone>=4 )
								reqMode = AFunc_Mission;
						}
					/*判断执行任务*/
						
					/*判断返航*/
						if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
						{	//按钮按下返航
							if( RTLButtonZone>=4 )
								reqMode = AFunc_RTL;
						}
					/*判断返航*/
						
					if( reqMode == 0 )
						reqMode = MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];
				}
			}
		}
		else
		{	//接收机不可用重置遥控状态
			ModeButtonZone = MissionButtonZone = RTLButtonZone = 255;
			//如果不是自动模式则切换到返航模式
			if( is_AFunc_auto(cMode)==false )
				reqMode = AFunc_RTL;
		}
		
		if( is_AFunc_auto(reqMode) || is_AFunc_auto(cMode) )
		{	//进出自动模式置位mode_swithced
			if( cMode != reqMode )
			{
				cMode = (AFunc)reqMode;
				mode_switched = true;
			}
		}
		else
			cMode = (AFunc)reqMode;
		#define swManualMode if( is_AFunc_auto(MFunc_cfg.Bt1AFunc1[8*ModeButtonZone]) == false )\
														*resMode = cMode = (AFunc)MFunc_cfg.Bt1AFunc1[8*ModeButtonZone];\
													else\
														*resMode = cMode = AFunc_PosHold;\
		
		if( cMode!=AFunc_Mission || is_MSafeCtrl() )
		{	//不在任务模式时保存任务
			if( mission_updated )
			{
				mission_updated = false;
				//保存当前航点信息
				UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
			}
		}
		
		if( cMode==AFunc_RTL )
		{	//进入安全模式返航
RTL:
			enter_MSafe(true);
			/*判断退出模式*/
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			/*判断退出模式*/
		}
		else if( is_MSafeCtrl() )
		{
			cMode = AFunc_PosHold;
			Position_Control_set_XYLock();
			Position_Control_set_ZLock();
			/*判断退出模式*/
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					return MR_OK;
				}
			/*判断退出模式*/
		}
		else if( cMode==AFunc_TakeOff )
		{	//起飞模式
			if( mode_switched )
			{
				mode_switched = false;
				if( inFlight ){
					swManualMode}
				else
				{
					Position_Control_Enable();
					bool pos_ena;
					is_Position_Control_Enabled(&pos_ena);
					if( pos_ena )
					{
						Position_Control_set_XYLock();
						Position_Control_Takeoff_HeightGlobal(param2);
					}
					else{
						swManualMode}
				}
			}
			else
			{
				Position_Control_Enable();
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				if( pos_ena == false )
				{	//位置控制器无法打开返回手动模式
					swManualMode
					goto Manual_Mode;
				}
				
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_AUTO,
					PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF );
				
				Position_Control_set_XYLock();
				Position_ControlMode alt_mode;
				get_Altitude_ControlMode(&alt_mode);
				if( alt_mode == Position_ControlMode_Position )
				{
					swManualMode
				}
			}			
		}
		else if( cMode==AFunc_Mission )
		{	//任务模式
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//位置控制器无法打开返回手动模式
				swManualMode
				goto Manual_Mode;
			}
			
			if( mode_switched )
			{	//刚切换到任务模式
				//首先刹车等待								
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				Attitude_Control_set_YawLock();
				
				camTriggDist = CamTriggDist = 0;
				
				//等待刹车完成
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//刹车完成
					++navInf.counter2;
					//等待1秒再进入任务飞行
					if( navInf.counter2 >= 1*freq )
					{
						mode_switched = false;					
						if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
						{	//载入下一航点成功
							//初始化任务信息
							init_NavCmdInf(&navInf);
							//设置相机触发距离
							camTriggDist = CamTriggDist;
							if( current_mission_ind == currentWpInf.CurrentWp[0] )
								//恢复航线飞行
								MissionMode_BackToLastWp = 0;
							else
								//当前和记录的航点不同不恢复飞行
								MissionMode_BackToLastWp = 10;
						}
						else
						{	//获取不到航点信息
							//先试着把航点设置为首个
							setCurrentMission(0);
							//复位当前航点信息
							currentWpInf.CurrentWp[0] = 0;
							currentWpInf.line_x = 0;
							currentWpInf.line_y = 0;
							currentWpInf.line_z = 0;
							currentWpInf.line_fs = -1;
							currentWpInf.CamTrigDist = 0;
							MissionMode_BackToLastWp = 10;
							if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
							{	//载入下一航点成功
								//初始化任务信息
								init_NavCmdInf(&navInf);
								//设置相机触发距离
								camTriggDist = CamTriggDist;
							}
							else
							{	//无航点信息返回手动模式
								swManualMode
								goto Manual_Mode;
							}
						}
					}
				}
				else
					navInf.counter2 = 0;
			}
			else if( MissionMode_BackToLastWp != 10 )
			{	//首先飞到上次航线飞行位置
				
				//设定mavlink模式
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_AUTO,
					PX4_CUSTOM_SUB_MODE_AUTO_MISSION );
				
				double AB_length = safe_sqrt( sq(currentWpInf.line_x) + sq(currentWpInf.line_y) + sq(currentWpInf.line_z) );
				if( current_mission_inf.cmd!=MAV_CMD_NAV_WAYPOINT || currentWpInf.CurrentWp[0]==0 || currentWpInf.line_fs<0 )
				{	//不需要恢复直接进入任务飞行							
					MissionMode_BackToLastWp = 10;
				}
				else
				{	//需要恢复到上次飞行位置
					switch( MissionMode_BackToLastWp )
					{
						case 0:
						{	//没起飞先起飞
							if(inFlight)
							{
								camTriggDist = CamTriggDist = currentWpInf.CamTrigDist;
								++MissionMode_BackToLastWp;
							}
							else
							{
								Position_Control_set_XYLock();
								Position_Control_set_TargetVelocityZ(50);
							}
							break;
						}
						case 1:
						{	//高度调整
							//锁定xy
							Position_Control_set_XYLock();							
							//求Z偏移距离
							double z_offset = 0;
							if( AB_length > 0.1 )
							{
								double inv_AB_length = 1.0 / AB_length;
								double ufs = AB_length - currentWpInf.line_fs;
								if( ufs < 0 )
									ufs = 0;
								z_offset = ufs * currentWpInf.line_z*inv_AB_length;
							}
							//执行结果
							bool res = false;
							switch(current_mission_inf.frame)
							{
								case MAV_FRAME_GLOBAL_INT:
								case MAV_FRAME_GLOBAL:
								{
									res = Position_Control_set_TargetPositionZGlobal( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT:
								{
									res = Position_Control_set_TargetPositionZRA( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								case MAV_FRAME_GLOBAL_TERRAIN_ALT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
								{
									res = Position_Control_set_TargetPositionZRA( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								
								case MAV_FRAME_LOCAL_NED:
									res = Position_Control_set_TargetPositionZ( -(current_mission_inf.params[6]*100 + z_offset), 0 );
									break;
								
								case MAV_FRAME_LOCAL_ENU:
									res = Position_Control_set_TargetPositionZ( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								
								case MAV_FRAME_BODY_NED:
								case MAV_FRAME_BODY_FRD:
								case MAV_FRAME_BODY_OFFSET_NED:
								case MAV_FRAME_LOCAL_OFFSET_NED:
									res = Position_Control_set_TargetPositionZRelative( -(current_mission_inf.params[6]*100 + z_offset), 0 );
									break;
								
								
								case MAV_FRAME_BODY_FLU:
								{
									res = Position_Control_set_TargetPositionZRelative( current_mission_inf.params[6]*100 + z_offset, 0 );
									break;
								}
								
								default:
									MissionMode_BackToLastWp = 10;
							}
							if(res)
								++MissionMode_BackToLastWp;
							else
								MissionMode_BackToLastWp = 10;
							break;
						}
						
						case 2:
						{	//等待高度调整完成
							//锁定xy
							Position_Control_set_XYLock();
							Position_ControlMode alt_mode;
							get_Altitude_ControlMode(&alt_mode);
							if( alt_mode == Position_ControlMode_Position )
								++MissionMode_BackToLastWp;
							break;
						}
						
						case 3:
						{	//旋转偏航
														
							//锁定XYZ
							Position_Control_set_XYLock();
							Position_Control_set_ZLock();
							//求AB向量长度倒数
							double offset_x=0, offset_y=0;
							if( AB_length > 0.1 )
							{
								double inv_AB_length = 1.0 / AB_length;
								double ufs = AB_length - currentWpInf.line_fs;
								if( ufs < 0 )
									ufs = 0;
								offset_x = ufs * currentWpInf.line_x*inv_AB_length;
								offset_y = ufs * currentWpInf.line_y*inv_AB_length;
							}
							
							double LA, LB;
							switch(current_mission_inf.frame)
							{
								case MAV_FRAME_GLOBAL:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT:
								case MAV_FRAME_GLOBAL_INT:
								case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT:
								case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
								{	//全球定位
									if( current_mission_inf.params[4]<-90 ||  current_mission_inf.params[4]> 90 
										|| current_mission_inf.params[5]<-180 || current_mission_inf.params[5]>180 )
									{	//经纬度为不正确不转偏航
										MissionMode_BackToLastWp += 2;
										goto ModeLoopFin;
									}
									
									//获取最优全球定位传感器信息
									PosSensorHealthInf2 global_inf;
									if( get_OptimalGlobal_XY( &global_inf ) == false )
									{
										MissionMode_BackToLastWp = 10;
										goto ModeLoopFin;
									}
									//获取指定经纬度平面坐标
									double x, y;
									map_projection_project( &global_inf.mp, current_mission_inf.params[4], current_mission_inf.params[5], &x, &y );
									x -= global_inf.HOffset.x;
									y -= global_inf.HOffset.y;
									x += offset_x;
									y += offset_y;
									LA = y - global_inf.PositionENU.y;
									LB = x - global_inf.PositionENU.x;
									break;
								}
								
								case MAV_FRAME_LOCAL_NED:
								{
									vector3<double> position;
									get_Position_Ctrl(&position);
									double x, y;
									x = current_mission_inf.params[5]*100;
									y = current_mission_inf.params[4]*100;
									x += offset_x;
									y += offset_y;
									LA = y - position.y;
									LB = x - position.x;
									break;
								}
								
								case MAV_FRAME_LOCAL_ENU:
								{
									vector3<double> position;
									get_Position_Ctrl(&position);
									double x, y;
									x = current_mission_inf.params[4]*100;
									y = current_mission_inf.params[5]*100;
									x += offset_x;
									y += offset_y;
									LA = y - position.y;
									LB = x - position.x;
									break;
								}
								
								default:
									MissionMode_BackToLastWp = 10;
									goto ModeLoopFin;
							}
							
							if( sq(LA) + sq(LB) > 5 )
								Attitude_Control_set_Target_Yaw( atan2(LA,LB) );
							++MissionMode_BackToLastWp;
								
							break;
						}
						
						case 4:
						{	//等待偏航旋转开始航点飞行
							Position_Control_set_XYLock();
							Position_Control_set_ZLock();
							double yawTrackErr;
							Attitude_Control_get_YawTrackErr(&yawTrackErr);
							if( yawTrackErr < 0.01 )
							{						
								//求AB向量长度倒数
								double offset_x=0, offset_y=0;
								if( AB_length > 0.1 )
								{
									double inv_AB_length = 1.0 / AB_length;
									double ufs = AB_length - currentWpInf.line_fs;
									if( ufs < 0 )
										ufs = 0;
									offset_x = ufs * currentWpInf.line_x*inv_AB_length;
									offset_y = ufs * currentWpInf.line_y*inv_AB_length;
								}
								double Tx, Ty;
								switch(current_mission_inf.frame)
								{
									case MAV_FRAME_GLOBAL:
									case MAV_FRAME_GLOBAL_RELATIVE_ALT:
									case MAV_FRAME_GLOBAL_INT:
									case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
									case MAV_FRAME_GLOBAL_TERRAIN_ALT:
									case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
									{	//全球定位
										if( current_mission_inf.params[4]<-90 ||  current_mission_inf.params[4]> 90 
											|| current_mission_inf.params[5]<-180 || current_mission_inf.params[5]>180 )
										{	//经纬度为不正确退出
											MissionMode_BackToLastWp = 10;
											goto ModeLoopFin;
										}
										
										//获取最优全球定位传感器信息
										PosSensorHealthInf2 global_inf;
										if( get_OptimalGlobal_XY( &global_inf ) == false )
										{
											MissionMode_BackToLastWp = 10;
											goto ModeLoopFin;
										}
										//获取指定经纬度平面坐标
										double x, y;
										map_projection_project( &global_inf.mp, current_mission_inf.params[4], current_mission_inf.params[5], &x, &y );
										x -= global_inf.HOffset.x;
										y -= global_inf.HOffset.y;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									case MAV_FRAME_LOCAL_NED:
									{
										double x, y;
										x = current_mission_inf.params[5]*100;
										y = current_mission_inf.params[4]*100;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									case MAV_FRAME_LOCAL_ENU:
									{
										double x, y;
										x = current_mission_inf.params[4]*100;
										y = current_mission_inf.params[5]*100;
										x += offset_x;
										y += offset_y;
										Tx = x;	Ty = y;
										break;
									}
									
									default:
										MissionMode_BackToLastWp = 10;
										goto ModeLoopFin;
								}
								
								bool res = Position_Control_set_TargetPositionXY( Tx, Ty, 0 );
								if(res)
									++MissionMode_BackToLastWp;
								else
									MissionMode_BackToLastWp = 10;
							}
							break;
						}
						
						case 5:
						{	//等待航线飞行完成
							Position_Control_set_ZLock();
							Position_ControlMode pos_mode;
							get_Position_ControlMode(&pos_mode);
							if( pos_mode == Position_ControlMode_Position )
							{	//已成功移动到上次航线位置
								MissionMode_BackToLastWp = 10;
							}
							break;
						}
						
					}
				}
			}
			else
			{	//任务飞行
				
				mission_updated = true;
				
				//设定mavlink模式
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_AUTO,
					PX4_CUSTOM_SUB_MODE_AUTO_MISSION );
				
				int16_t res = -100;
				if( Process_InflightCmd( current_mission_inf.cmd, current_mission_inf.params ) == false )
				{
					res = Process_NavCmd(
						current_mission_inf.cmd,
						freq, 
						current_mission_inf.frame,
						current_mission_inf.params,
						&navInf
					);
				}
				
				if( NavCmdRs_SuccessOrFault(res) )
				{	//错误或执行完成
					
					//航线结束拍照
					if( NavCmdRs_Success(res) )
					{
						if( camTriggDist > 0 )
						{
							for( uint8_t i = 0; i < 10; ++i )
							{
								Position_Control_set_XYLock();
								Position_Control_set_ZLock();
								Attitude_Control_set_YawLock();
								os_delay(0.05);
							}
							InflightCmd_CamTakePhoto();
						}
					}
					
					//不自动执行返回手动模式
					if( current_mission_inf.autocontinue == 0 )
					{
						swManualMode
					}
					
					if( res < 0 )
					{	//切换到下一模式
						MissionInf chk_inf;
						uint16_t chk_ind;
						if( ReadCurrentMission(&chk_inf, &chk_ind) )
						{	//读取当前任务信息比较						
							if( chk_ind==current_mission_ind && memcmp( &chk_inf, &current_mission_inf, sizeof(MissionInf) ) == 0 )
							{	//如果相同才切换下一个任务
								if( setCurrentMission( getCurrentMissionInd() + navInf.usr_temp[MissionInc] + 1 ) == false )
								{	//无航点信息返回手动模式
									setCurrentMission( 0 );
									swManualMode
									//复位当前航点信息
									currentWpInf.CurrentWp[0] = 0;
									currentWpInf.line_x = 0;
									currentWpInf.line_y = 0;
									currentWpInf.line_z = 0;
									currentWpInf.line_fs = -1;
									currentWpInf.CamTrigDist = 0;
									if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
									{
										bool inFlight;
										get_is_inFlight(&inFlight);
										if( inFlight==false )
										{	//降落完成加锁
											Attitude_Control_Disable();
											UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
											return MR_OK;
										}
									}
								}
								else
								{
									if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
									{	//载入下一航点成功
										//初始化任务信息
										init_NavCmdInf(&navInf);
										//设置相机触发距离
										camTriggDist = CamTriggDist;
										//更新任务状态
										if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
										{
											vector3<double> AB;
											NavCmd16_WAYPOINT_GetAB( current_mission_inf.frame, current_mission_inf.params, &AB );
											currentWpInf.CurrentWp[0] = current_mission_ind;
											currentWpInf.line_x = AB.x;
											currentWpInf.line_y = AB.y;
											currentWpInf.line_z = AB.z;
											currentWpInf.line_fs = 0;
											currentWpInf.CamTrigDist = camTriggDist;
										}
										else
										{
											currentWpInf.CurrentWp[0] = current_mission_ind;
											currentWpInf.line_x = 0;
											currentWpInf.line_y = 0;
											currentWpInf.line_z = 0;
											currentWpInf.line_fs = 0;
											currentWpInf.CamTrigDist = camTriggDist;
										}
									}
									else
									{	//无航点信息返回手动模式
										setCurrentMission( 0 );
										swManualMode						
										//复位当前航点信息
										currentWpInf.CurrentWp[0] = 0;
										currentWpInf.line_x = 0;
										currentWpInf.line_y = 0;
										currentWpInf.line_z = 0;
										currentWpInf.line_fs = -1;
										currentWpInf.CamTrigDist = 0;
										if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
										{
											bool inFlight;
											get_is_inFlight(&inFlight);
											if( inFlight==false )
											{	//降落完成加锁
												Attitude_Control_Disable();
												UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
												return MR_OK;
											}
										}
									}
								}
							}
							else
							{	//航点信息不相同不切换下一任务
								//使用新获取的任务信息
								current_mission_inf = chk_inf;
								current_mission_ind = chk_ind;
								//初始化任务信息
								init_NavCmdInf(&navInf);
								//更新任务状态																	
								if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
								{
									vector3<double> AB;
									NavCmd16_WAYPOINT_GetAB( current_mission_inf.frame, current_mission_inf.params, &AB );
									currentWpInf.CurrentWp[0] = current_mission_ind;
									currentWpInf.line_x = AB.x;
									currentWpInf.line_y = AB.y;
									currentWpInf.line_z = AB.z;
									currentWpInf.line_fs = 0;
									currentWpInf.CamTrigDist = camTriggDist;
								}
								else
								{
									currentWpInf.CurrentWp[0] = current_mission_ind;
									currentWpInf.line_x = 0;
									currentWpInf.line_y = 0;
									currentWpInf.line_z = 0;
									currentWpInf.line_fs = 0;
									currentWpInf.CamTrigDist = camTriggDist;
								}
							}
						}
						else
						{	//无航点信息返回手动模式
							setCurrentMission( 0 );
							swManualMode
							//复位当前航点信息
							currentWpInf.CurrentWp[0] = 0;
							currentWpInf.line_x = 0;
							currentWpInf.line_y = 0;
							currentWpInf.line_z = 0;
							currentWpInf.line_fs = -1;
							currentWpInf.CamTrigDist = 0;
							if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
							{	//降落完成加锁
								bool inFlight;
								get_is_inFlight(&inFlight);
								if( inFlight==false )
								{
									Attitude_Control_Disable();
									UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
									return MR_OK;
								}
							}
						}
					}
					else
					{	//切换到指定模式
						if( setCurrentMission( res ) == false )
						{	//切换失败返回手动模式
							setCurrentMission( 0 );
							swManualMode
							//复位当前航点信息
							currentWpInf.CurrentWp[0] = 0;
							currentWpInf.line_x = 0;
							currentWpInf.line_y = 0;
							currentWpInf.line_z = 0;
							currentWpInf.line_fs = -1;
							currentWpInf.CamTrigDist = 0;
						}
						if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
						{	//载入下一航点成功
							//初始化任务信息
							init_NavCmdInf(&navInf);
							//设置相机触发距离
							camTriggDist = CamTriggDist;
							//更新任务状态														
							if( current_mission_inf.cmd == MAV_CMD_NAV_WAYPOINT )
							{
								vector3<double> AB;
								NavCmd16_WAYPOINT_GetAB( current_mission_inf.frame, current_mission_inf.params, &AB );
								currentWpInf.CurrentWp[0] = current_mission_ind;
								currentWpInf.line_x = AB.x;
								currentWpInf.line_y = AB.y;
								currentWpInf.line_z = AB.z;
								currentWpInf.line_fs = 0;
								currentWpInf.CamTrigDist = camTriggDist;
							}
							else
							{
								currentWpInf.CurrentWp[0] = current_mission_ind;
								currentWpInf.line_x = 0;
								currentWpInf.line_y = 0;
								currentWpInf.line_z = 0;
								currentWpInf.line_fs = 0;
								currentWpInf.CamTrigDist = camTriggDist;
							}
						}
						else
						{	//无航点信息返回手动模式
							setCurrentMission( 0 );
							swManualMode							
							//复位当前航点信息
							currentWpInf.CurrentWp[0] = 0;
							currentWpInf.line_x = 0;
							currentWpInf.line_y = 0;
							currentWpInf.line_z = 0;
							currentWpInf.line_fs = -1;
							currentWpInf.CamTrigDist = 0;
							if( current_mission_inf.cmd==MAV_CMD_NAV_RETURN_TO_LAUNCH || current_mission_inf.cmd==MAV_CMD_NAV_LAND )
							{
								bool inFlight;
								get_is_inFlight(&inFlight);
								if( inFlight==false )
								{	//降落完成加锁
									Attitude_Control_Disable();
									UpdateParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0, sizeof(CurrentWpInf)/8 );
									return MR_OK;
								}
							}
						}					
					}
				}
				else
				{	//任务执行中			
					if( NavCmdRs_InProgress_CanExInFlightCmd(res) )
					{	//可执行InFlightCmd
						
						if( navInf.usr_temp[DealInFlightCmd] == 0 )
						{	//还未执行inFlightCmd
							//执行所有inFlightCmd
							MissionInf inFlightMs_inf;
							while(1)
							{
								if( ReadMission( getCurrentMissionInd() + navInf.usr_temp[MissionInc] + 1, &inFlightMs_inf ) )
								{
									if( Process_InflightCmd( inFlightMs_inf.cmd, inFlightMs_inf.params ) )
										navInf.usr_temp[MissionInc] += 1;
									else
										break;
								}
								else
									break;
							}
						}
						navInf.usr_temp[DealInFlightCmd] = 1;
						
						//储存飞行状态
						vector3<double> line_AB;
						double flightDistance = -1;
						if( Position_Control_get_LineFlightABDistance( &line_AB, &flightDistance ) )
						{
							currentWpInf.CurrentWp[0] = current_mission_ind;
							currentWpInf.line_x = line_AB.x;
							currentWpInf.line_y = line_AB.y;
							currentWpInf.line_z = line_AB.z;
							currentWpInf.line_fs = flightDistance;
							currentWpInf.CamTrigDist = camTriggDist;
						}
						
						//定距拍照
						if( camTriggDist > 0 )
						{
							Position_Control_get_LineFlightDistance(&flightDistance);
							if( flightDistance >= 0 )
							{
								int mult = (int)(flightDistance / camTriggDist) + 1;
								if( mult > navInf.usr_temp[CamTriggDistMult] )
								{
									InflightCmd_CamTakePhoto();
									navInf.usr_temp[CamTriggDistMult] = mult;
								}
							}
						}
					}
				}
			}
		}
		else if( cMode==AFunc_Offboard )
		{
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//位置控制器无法打开返回手动模式
				swManualMode
				goto Manual_Mode;
			}
			
			if(mode_switched)
			{	//刚进入模式初始化变量
				mode_switched = false;
				ManualModeNavCmdInprogress = false;
				Attitude_Control_set_YawLock();
			}
			
			//设定mavlink模式			
			set_mav_mode( 
				MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
				PX4_CUSTOM_MAIN_MODE_OFFBOARD,
				0 );
			
			//执行指令
			if(msg_available)
			{
				//执行inflight cmd
				if( Process_InflightCmd( msg.cmd, msg.params ) == false )
				if( check_NavCmd( msg.cmd, freq, default_NavCmd_frame, msg.params ) )
				{	//指令可被执行
					init_NavCmdInf(&navInf);
					ManualModeNavCmdInprogress = true;
					ManualModeNavCmd = msg;
					msg_handled = 1;
				}
				
				//执行set_position_target指令
				if( msg.cmd == 1084 )
				{	//SET_POSITION_TARGET_LOCAL_NED
					uint16_t type_mask = msg.params[0];
					//thr
					if( (type_mask&POSITION_TARGET_TYPEMASK_Z_IGNORE)==0 )
					{	//pva
						double posz=msg.params[3]*100;
						double velz=0;
						double accz=0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_VZ_IGNORE)==0 )
						{
							velz = msg.params[6]*100;
							if( (type_mask&POSITION_TARGET_TYPEMASK_AZ_IGNORE)==0 )
								accz = msg.params[9]*100;
						}
						switch( msg.frame )
						{
							case MAV_FRAME_LOCAL_NED:
							{
								Position_Control_set_TargetPosVelAccZ_OffBoard( -posz, -velz, -accz );
								break;
							}
							default:
							case MAV_FRAME_LOCAL_ENU:
							{
								Position_Control_set_TargetPosVelAccZ_OffBoard( posz, velz, accz );
								break;
							}
							case MAV_FRAME_BODY_FLU:
							{
								Position_Control_set_TargetPosRelativeVelAccZ_OffBoard( posz, velz, accz );
								break;
							}
							case MAV_FRAME_BODY_FRD:
							case MAV_FRAME_BODY_NED:
							case MAV_FRAME_BODY_OFFSET_NED:
							{
								Position_Control_set_TargetPosRelativeVelAccZ_OffBoard( -posz, -velz, -accz );
								break;
							}
						}
						Offboard_ThrTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_VZ_IGNORE)==0 )
					{	//va
						double velz=0;
						double accz=0;
						velz = msg.params[6]*100;
						if( (type_mask&POSITION_TARGET_TYPEMASK_AZ_IGNORE)==0 )
							accz = msg.params[9]*100;
						switch( msg.frame )
						{
							case MAV_FRAME_BODY_FRD:
							case MAV_FRAME_BODY_NED:
							case MAV_FRAME_BODY_OFFSET_NED:
							case MAV_FRAME_LOCAL_NED:
							{
								Position_Control_set_TargetVelAccZ_OffBoard( -velz, -accz );
								break;
							}
							default:
							case MAV_FRAME_BODY_FLU:
							case MAV_FRAME_LOCAL_ENU:
							{
								Position_Control_set_TargetVelAccZ_OffBoard( velz, accz );
								break;
							}
						}
						Offboard_ThrTIME = TIME::now();
					}
					//yaw
					if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_IGNORE)==0 )
					{	//pva
						double Tyaw = 0.5*Pi - msg.params[10];
						double Trate = 0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)==0 )
							Trate = -msg.params[11];
						Attitude_Control_set_Target_Yaw_Offboard( Tyaw, Trate );
						Offboard_YawTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)==0 )
					{
						double Trate = -msg.params[11];
						Attitude_Control_set_Target_YawRate( Trate );
						Offboard_YawTIME = TIME::now();
					}
					//pit rol
					if( (type_mask&POSITION_TARGET_TYPEMASK_X_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_Y_IGNORE)==0 )
					{	//pva
						double posx=msg.params[1]*100;	double posy=msg.params[2]*100;
						double velx=0, vely=0;
						double accx=0, accy=0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)==0 )
						{
							velx = msg.params[4]*100;	vely = msg.params[5]*100;
							if( (type_mask&POSITION_TARGET_TYPEMASK_AX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_AY_IGNORE)==0 )
								accx = msg.params[7]*100;	accy = msg.params[8]*100;
						}
						switch( msg.frame )
						{
							case MAV_FRAME_LOCAL_NED:
							{
								Position_Control_set_TargetPosVelAccXY_OffBoard( posy, posx, vely, velx, accy, accx );
								break;
							}
							default:
							case MAV_FRAME_LOCAL_ENU:
							{
								Position_Control_set_TargetPosVelAccXY_OffBoard( posx, posy, velx, vely, accx, accy );
								break;
							}
							case MAV_FRAME_BODY_FLU:
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
								double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
								double velx_enu = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
								double vely_enu = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
								double accx_enu = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
								double accy_enu = BodyHeading2ENU_y( accx , accy , sin_Yaw , cos_Yaw );
								Position_Control_set_TargetPosRelativeVelAccXY_OffBoard( posx_enu, posy_enu, velx_enu, vely_enu, accx_enu, accy_enu );
								break;
							}
							case MAV_FRAME_BODY_FRD:
							case MAV_FRAME_BODY_NED:
							case MAV_FRAME_BODY_OFFSET_NED:
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
								double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
								double velx_enu = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
								double vely_enu = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
								double accx_enu = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
								double accy_enu = BodyHeading2ENU_y( accx , accy , sin_Yaw , cos_Yaw );
								Position_Control_set_TargetPosRelativeVelAccXY_OffBoard( posx_enu, -posy_enu, velx_enu, -vely_enu, accx_enu, -accy_enu );
								break;
							}
						}
						Offboard_PitRolTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)==0 )
					{	//va
						double velx=0, vely=0;
						double accx=0, accy=0;
						velx = msg.params[4]*100;	vely = msg.params[5]*100;
						if( (type_mask&POSITION_TARGET_TYPEMASK_AX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_AY_IGNORE)==0 )
							accx = msg.params[7]*100;	accy = msg.params[8]*100;
						switch( msg.frame )
						{
							case MAV_FRAME_LOCAL_NED:
							{
								Position_Control_set_TargetVelAccXY_OffBoard( vely, velx, accy, accx );
								break;
							}
							default:
							case MAV_FRAME_LOCAL_ENU:
							{
								Position_Control_set_TargetVelAccXY_OffBoard( velx, vely, accx, accy );
								break;
							}
							case MAV_FRAME_BODY_FLU:
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								double velx_enu = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
								double vely_enu = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
								double accx_enu = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
								double accy_enu = BodyHeading2ENU_y( accx , accy , sin_Yaw , cos_Yaw );
								Position_Control_set_TargetVelAccXY_OffBoard( velx_enu, vely_enu, accx_enu, accy_enu );
								break;
							}
							case MAV_FRAME_BODY_FRD:
							case MAV_FRAME_BODY_NED:
							case MAV_FRAME_BODY_OFFSET_NED:
							{
								double yaw;	double yaw_declination;
								get_YawDeclination(&yaw_declination);
								Attitude_Control_get_TargetTrackYaw(&yaw);
								yaw += yaw_declination;
								double sin_Yaw, cos_Yaw;
								fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
								double velx_enu = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
								double vely_enu = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
								double accx_enu = BodyHeading2ENU_x( accx , accy , sin_Yaw , cos_Yaw );
								double accy_enu = BodyHeading2ENU_y( accx , accy , sin_Yaw , cos_Yaw );
								Position_Control_set_TargetVelAccXY_OffBoard( velx_enu, -vely_enu, accx_enu, -accy_enu );
								break;
							}
						}
						Offboard_PitRolTIME = TIME::now();
					}
					
					//发送ACK
					uint8_t port_id = msg.cmd_type&0xf;
					const Port* port = get_CommuPort(port_id);
					if( port->write )
					{
						if( mavlink_lock_chan(port_id,0.01) )
						{
							mavlink_message_t msg_sd;
							
							mavlink_msg_position_target_local_ned_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								port_id , 	//chan
								&msg_sd ,
								TIME::get_System_Run_Time()*1e3,
								msg.frame,
								msg.params[0],
								msg.params[1],
								msg.params[2],
								msg.params[3],
								msg.params[4],
								msg.params[5],
								msg.params[6],
								msg.params[7],
								msg.params[8],
								msg.params[9],
								msg.params[10],
								msg.params[11]
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, -1);
							mavlink_unlock_chan(port_id);
						}
					}
				}
				else if( msg.cmd == 1086 )
				{	//SET_POSITION_TARGET_GLOBAL_INT
					uint16_t type_mask = msg.params[0];
					//thr
					if( (type_mask&POSITION_TARGET_TYPEMASK_Z_IGNORE)==0 )
					{	//pva
						double posz=msg.params[3]*100;
						double velz=0;
						double accz=0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_VZ_IGNORE)==0 )
						{
							velz = msg.params[6]*100;
							if( (type_mask&POSITION_TARGET_TYPEMASK_AZ_IGNORE)==0 )
								accz = msg.params[9]*100;
						}
						switch( msg.frame )
						{
							case MAV_FRAME_GLOBAL_INT:
							case MAV_FRAME_GLOBAL:
							{
								Position_Control_set_TargetPosGlobalVelAccZ_OffBoard( -posz, -velz, -accz );
								break;
							}
							default:
							case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
							case MAV_FRAME_GLOBAL_RELATIVE_ALT:
							case MAV_FRAME_GLOBAL_TERRAIN_ALT:
							case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
							{
								Position_Control_set_TargetPosZRAVelAccZ_OffBoard( -posz, -velz, -accz );
								break;
							}
						}
						Offboard_ThrTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_VZ_IGNORE)==0 )
					{	//va
						double velz=0;
						double accz=0;
						velz = msg.params[6]*100;
						if( (type_mask&POSITION_TARGET_TYPEMASK_AZ_IGNORE)==0 )
							accz = msg.params[9]*100;
						Position_Control_set_TargetVelAccZ_OffBoard( -velz, -accz );
						Offboard_ThrTIME = TIME::now();
					}
					//yaw
					if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_IGNORE)==0 )
					{	//pva
						double Tyaw = 0.5*Pi - msg.params[10];
						double Trate = 0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)==0 )
							Trate = -msg.params[11];
						Attitude_Control_set_Target_Yaw_Offboard( Tyaw, Trate );
						Offboard_YawTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)==0 )
					{
						double Trate = -msg.params[11];
						Attitude_Control_set_Target_YawRate( Trate );
						Offboard_YawTIME = TIME::now();
					}
					//pit rol
					if( (type_mask&POSITION_TARGET_TYPEMASK_X_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_Y_IGNORE)==0 )
					{	//pva
						double posx=msg.params[1]*100;	double posy=msg.params[2]*100;
						double velx=0, vely=0;
						double accx=0, accy=0;
						if( (type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)==0 )
						{
							velx = msg.params[4]*100;	vely = msg.params[5]*100;
							if( (type_mask&POSITION_TARGET_TYPEMASK_AX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_AY_IGNORE)==0 )
								accx = msg.params[7]*100;	accy = msg.params[8]*100;
						}
						Position_Control_set_TargetPosVelAccXY_OffBoard( posy, posx, vely, velx, accy, accx );
						Offboard_PitRolTIME = TIME::now();
					}
					else if( (type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)==0 )
					{	//va
						double velx=0, vely=0;
						double accx=0, accy=0;
						velx = msg.params[4]*100;	vely = msg.params[5]*100;
						if( (type_mask&POSITION_TARGET_TYPEMASK_AX_IGNORE)==0 && (type_mask&POSITION_TARGET_TYPEMASK_AY_IGNORE)==0 )
							accx = msg.params[7]*100;	accy = msg.params[8]*100;
						Position_Control_set_TargetVelAccXY_OffBoard( vely, velx, accy, accx );
						Offboard_PitRolTIME = TIME::now();
					}
					
					//发送ACK
					uint8_t port_id = msg.cmd_type&0xf;
					const Port* port = get_CommuPort(port_id);
					if( port->write )
					{
						if( mavlink_lock_chan(port_id,0.01) )
						{
							mavlink_message_t msg_sd;
							
							mavlink_msg_position_target_local_ned_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								port_id , 	//chan
								&msg_sd ,
								TIME::get_System_Run_Time()*1e3,
								msg.frame,
								msg.params[0],
								msg.params[1],
								msg.params[2],
								msg.params[3],
								msg.params[4],
								msg.params[5],
								msg.params[6],
								msg.params[7],
								msg.params[8],
								msg.params[9],
								msg.params[10],
								msg.params[11]
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, -1);
							mavlink_unlock_chan(port_id);
						}
					}
				}
			}
			
			if( rc.available )
			{				
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//降落后自动加锁
					if( inFlight==false && rc.data[0]<30 )
					{
						if( ++exit_mode_counter >= 500 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_counter = exit_mode_counter_rs;
					//手势强制加锁
					if( rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90 )
					{
						if( ++exit_mode_Gcounter >= 50 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_Gcounter = 0;
				/*判断退出模式*/
					
				//设定mavlink模式				
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_OFFBOARD,
					0 );

				//判断摇杆是否回中
				bool thr_in_neutral = in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] );
				bool yaw_in_neutral = in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] );
				bool pitrol_in_neutral = 
					in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
				
				//油门控制 摇杆在中位且一段时间无控制才锁定
				if( thr_in_neutral )
				{
					if( Offboard_ThrTIME.get_pass_time() > 1.5 )
						Position_Control_set_ZLock();
				}
				else
				{
					double thr_stick = remove_deadband( rc.data[0] - 50.0 , (double)MFunc_cfg.NeutralZone[0] );
					if( thr_stick > 0 )
						thr_stick *= get_maxVelUp() / 50;
					else
						thr_stick *= get_maxVelDown() / 50;
					Position_Control_set_TargetVelocityZ(thr_stick);
					Offboard_ThrTIME.set_invalid();
				}
				
				//偏航控制 摇杆在中位且一段时间无控制才锁定
				if( yaw_in_neutral )
				{
					if( Offboard_YawTIME.get_pass_time() > 1.5 )
						Attitude_Control_set_YawLock();
				}
				else
				{
					double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
					Offboard_YawTIME.set_invalid();
				}
				
				//横滚俯仰控制 摇杆在中位且一段时间无控制才锁定
				if( pitrol_in_neutral )
				{
					if( Offboard_PitRolTIME.get_pass_time() > 1.5 )
						Position_Control_set_XYLock();
				}
				else
				{
					double RPCtrlScale = atan2( get_maxAccXY() / 50.0, GravityAcc );
					double XYCtrlScale = get_maxVelXY() / 50.0;						
					double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
					double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
					vector3<double> velocityFLU;
					get_VelocityFLU_Ctrl(&velocityFLU);
					double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
					double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
					constrain_vector( vel_stick_err_roll, vel_stick_err_pitch, 50 );
					Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
						pitch_sitck_d * XYCtrlScale ,\
						-roll_sitck_d * XYCtrlScale , \
						fabs( vel_stick_err_roll  )*RPCtrlScale, \
						fabs( vel_stick_err_pitch )*RPCtrlScale \
					);
					Offboard_PitRolTIME.set_invalid();
				}
			}
			else if( Offboard_ThrTIME.get_pass_time()>2 && Offboard_PitRolTIME.get_pass_time()>2 )
			{	//无遥控信号且一段时间无信号
				//进入安全模式
				change_Mode(AFunc_RTL)
				goto RTL;
			}
		}
		else
		{	//手动飞行模式（包含定高定点控制）
			Manual_Mode:
						
			if(mode_switched)
			{	//刚进入手动模式初始化变量
				mode_switched = false;
				ManualModeNavCmdInprogress = false;
				Attitude_Control_set_YawLock();
			}
			
			if( rc.available )
			{				
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//降落后自动加锁
					if( inFlight==false && rc.data[0]<30 )
					{
						if( ++exit_mode_counter >= 500 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_counter = exit_mode_counter_rs;
					//手势强制加锁
					if( rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90 )
					{
						if( ++exit_mode_Gcounter >= 50 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_Gcounter = 0;
				/*判断退出模式*/
					
				//切换定高定点
				if( cMode==AFunc_AltHold )
					Position_Control_Disable();
				else
					Position_Control_Enable();				
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				
				if(pos_ena)
					setLedMode(LEDMode_Flying2);
				else
					setLedMode(LEDMode_Flying1);
				
				//设定mavlink模式
				if( pos_ena )						
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_POSCTL,
						0 );
				else
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_ALTCTL,
						0 );

				
				//判断摇杆是否回中
				bool sticks_in_neutral = 
					in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
				
				//非loiter模式复位loiter速度
				if( !pos_ena || cMode!=AFunc_Loiter ) {
					lastLoiterTVel.x = lastLoiterTVel.y = 0;
					LoiterAcc.x = LoiterAcc.y = 0;
					LoiterAccFilters[0].reset(0);
					LoiterAccFilters[1].reset(0);
				}
				
				if( pos_ena && cMode==AFunc_Loiter )
				{
					//油门杆控制垂直速度
					if( in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Position_Control_set_ZLock();
					else
					{
						double thr_stick = remove_deadband( rc.data[0] - 50.0 , (double)MFunc_cfg.NeutralZone[0] );
						if( thr_stick > 0 )
							thr_stick *= get_maxVelUp() / 50;
						else
							thr_stick *= get_maxVelDown() / 50;
						Position_Control_set_TargetVelocityZ(thr_stick);
					}
					
					//偏航杆在中间锁偏航
					//不在中间控制偏航速度
					double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
					if( in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Attitude_Control_set_YawLock();
					else
						Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );						
					
					//XY期望速度
					double RPCtrlScale = atan2( get_maxAccXY() / 50.0, GravityAcc );
					double XYCtrlScale = get_maxVelXY() / 50.0;						
					double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
					double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
					vector3<double> velocityFLU;
					get_VelocityFLU_Ctrl(&velocityFLU);
					double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
					double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
					constrain_vector( vel_stick_err_roll, vel_stick_err_pitch, 50 );
					vector2<double> TvelFlu;
					TvelFlu.x = pitch_sitck_d * XYCtrlScale;
					TvelFlu.y = -roll_sitck_d * XYCtrlScale;
					
					//获取ENU系期望速度
					double yaw;	double yaw_declination;
					get_YawDeclination(&yaw_declination);
					Attitude_Control_get_TargetTrackYaw(&yaw);
					yaw += yaw_declination;
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					vector2<double> TvelEnu;
					TvelEnu.x = BodyHeading2ENU_x( TvelFlu.x , TvelFlu.y , sin_Yaw , cos_Yaw );
					TvelEnu.y = BodyHeading2ENU_y( TvelFlu.x , TvelFlu.y , sin_Yaw , cos_Yaw );

					//限制期望速度
					vector3<double> velocityEnu;
					get_VelocityENU(&velocityEnu);
					vector2<double> velEnu( velocityEnu.x, velocityEnu.y );
					vector2<double> velErr = TvelEnu - velEnu;
					double velErrDis = velErr.get_square();
					double maxVelErrDis = get_maxAccXY() / get_P2();
					vector2<double> TvelEnuC;
					if( velErrDis > sq(maxVelErrDis) ) {
						velErrDis = safe_sqrt(velErrDis);
						velErr *= maxVelErrDis/velErrDis;
						TvelEnuC = velErr + velEnu;
					} else {
						TvelEnuC = TvelEnu;
					}
					
					//期望加速度滤波
					double TAccEnu_x = (TvelEnuC.x - lastLoiterTVel.x)*freq;
					double TAccEnu_y = (TvelEnuC.y - lastLoiterTVel.y)*freq;
					lastLoiterTVel.x = TvelEnuC.x;
					lastLoiterTVel.y = TvelEnuC.y;
					LoiterAccFilters[0].run(TAccEnu_x);
					LoiterAccFilters[1].run(TAccEnu_y);
					TAccEnu_x = constrain( LoiterAccFilters[0].get_result(), (double)get_maxAccXY() );
					TAccEnu_y = constrain( LoiterAccFilters[1].get_result(), (double)get_maxAccXY() );
					
					Position_Control_set_TargetPosRelativeVelAccXY_OffBoard( 
						TvelEnuC.x*h, TvelEnuC.y*h, 
						TvelEnu.x*get_VelXYFF(), TvelEnu.y*get_VelXYFF(), 
						0 , 0
					);
				}
				else if( sticks_in_neutral && pos_ena && cMode==AFunc_PosHold )
				{	//摇杆在中间且在定点模式下允许执行命令
					if(msg_available)
					{
						if( Process_InflightCmd( msg.cmd, msg.params ) == false )
						if( check_NavCmd( msg.cmd, freq, default_NavCmd_frame, msg.params ) )
						{	//指令可被执行
							init_NavCmdInf(&navInf);
							ManualModeNavCmdInprogress = true;
							ManualModeNavCmd = msg;
							msg_handled = 1;
						}
					}
					
					if( ManualModeNavCmdInprogress )
					{	//需要执行NavCmd
						int16_t res = -100;
						res = Process_NavCmd( ManualModeNavCmd.cmd, freq, default_NavCmd_frame, ManualModeNavCmd.params, &navInf );
						if( NavCmdRs_SuccessOrFault(res) )
						{	//NavCmd执行完成
							ManualModeNavCmdInprogress = false;
						}
					}
					else
					{
						Position_Control_set_ZLock();
						Position_Control_set_XYLock();
						Attitude_Control_set_YawLock();
					}
				}
				else
				{	//摇杆不在中间手动飞行
								
					ManualModeNavCmdInprogress = false;
					
					//油门杆控制垂直速度
					if( in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Position_Control_set_ZLock();
					else
					{
						double thr_stick = remove_deadband( rc.data[0] - 50.0 , (double)MFunc_cfg.NeutralZone[0] );
						if( thr_stick > 0 )
							thr_stick *= get_maxVelUp() / 50;
						else
							thr_stick *= get_maxVelDown() / 50;
						Position_Control_set_TargetVelocityZ(thr_stick);
					}
					
					if( pos_ena )
					{
						
						if( cMode==AFunc_ManualCircle )
						{
							double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
							double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
							double yaw_sitck_d = remove_deadband( rc.data[1] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
							double h = 1.0/freq;
							Position_Control_do_ManualCircleRelative( 5*roll_sitck_d*h, -3*pitch_sitck_d*h, 3*yaw_sitck_d*h );
						}
						else
						{
							//俯仰横滚杆控水平速度
							if( in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] ) && in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) )							
								Position_Control_set_XYLock();
							else
							{
								double RPCtrlScale = atan2( get_maxAccXY() / 50.0, GravityAcc );
								double XYCtrlScale = get_maxVelXY() / 50.0;						
								double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
								double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, (double)MFunc_cfg.NeutralZone[0] );
								vector3<double> velocityFLU;
								get_VelocityFLU_Ctrl(&velocityFLU);
								double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
								double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
								constrain_vector( vel_stick_err_roll, vel_stick_err_pitch, 50 );
								Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
									pitch_sitck_d * XYCtrlScale ,\
									-roll_sitck_d * XYCtrlScale , \
									fabs( vel_stick_err_roll  )*RPCtrlScale, \
									fabs( vel_stick_err_pitch )*RPCtrlScale \
								);
							}
						}
					}
					else
					{
						//补偿风力扰动
						vector3<double> WindDisturbance;
						get_WindDisturbance( &WindDisturbance );
						Quaternion attitude;
						get_Attitude_quat(&attitude);
						double yaw = attitude.getYaw();		
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
						double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
						
						//俯仰横滚杆控俯仰横滚
						double RPCtrlScale = degree2rad( get_maxLean() / 50.0 );
//						Attitude_Control_set_Target_RollPitch( 
//							( rc.data[3] - 50 )*RPCtrlScale - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
//							( rc.data[2] - 50 )*RPCtrlScale - atan2( WindDisturbance_Bodyheading_x , GravityAcc ) 
//						);
						Attitude_Control_set_Target_RollPitch( 
							( rc.data[3] - 50 )*RPCtrlScale,
							( rc.data[2] - 50 )*RPCtrlScale
						);
					}
					
					//偏航杆在中间锁偏航
					//不在中间控制偏航速度
					double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
					if( in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) )
						Attitude_Control_set_YawLock();
					else
						Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );							
				}
			}
			else
			{
				//无遥控信号进入安全模式
				change_Mode(AFunc_RTL)
				goto RTL;
			}
		}
		
ModeLoopFin:
		/*返回消息处理结果*/
			if( msg_available )
			{
				uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
				const Port* port = get_CommuPort( port_index );
				if( port->write )
				{
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( port_index, 0.01 ) )
					{
						mavlink_msg_command_ack_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							port_index ,
							&msg_sd,
							msg.cmd,	//command
							msg_handled==1 ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED ,	//result
							100 ,	//progress
							0 ,	//param2
							msg.sd_sysid ,	//target system
							msg.sd_compid //target component
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(port_index);
					}
				}
			}
		/*返回消息处理结果*/
	}
	return MR_OK;
}
