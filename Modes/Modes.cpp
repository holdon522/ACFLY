#include "Basic.hpp"
#include "Modes.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "Receiver.hpp"
#include "Commulink.hpp"
#include "MeasurementSystem.hpp"
#include "StorageSystem.hpp"
#include "Parameters.hpp"
#include "queue.h"
#include "mavlink.h"
#include "drv_PWMOut.hpp"
#include "AuxFuncs.hpp"
#include "ControlSystem.hpp"

#include "M10_RCCalib.hpp"
#include "M11_TempCalib.hpp"
#include "M12_AccCalib.hpp"
#include "M13_MagCalib.hpp"
#include "M14_HorizonCalib.hpp"

#include "M30_AttCtrl.hpp"
#include "M32_PosCtrl.hpp"
#include "M35_Auto1.hpp"

Mode_Base* modes[80] = {0};
static QueueHandle_t message_queue = xQueueCreate( 20, sizeof(ModeMsg) );
bool SendMsgToMode( ModeMsg msg, double TIMEOUT )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xQueueSend( message_queue, &msg, TIMEOUT_Ticks ) == pdTRUE )
		return true;
	else
		return false;
}
bool ModeReceiveMsg( ModeMsg* msg, double TIMEOUT )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xQueueReceive( message_queue, msg, TIMEOUT_Ticks ) == pdTRUE )
		return true;
	else
		return false;
}
static bool changeMode( uint16_t mode_index, void* param1, uint32_t param2, ModeResult* result,
												bool* msg_available, bool* msg_handled, ModeMsg* msg )
{
	/*先返回消息处理结果*/
		if( *msg_available )
		{
			uint8_t port_index = msg->cmd_type & CMD_TYPE_PORT_MASK;
			const Port* port = get_CommuPort( port_index );
			if( (msg->cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK && port->write )
			{
				mavlink_message_t msg_sd;
				if( mavlink_lock_chan( port_index, 0.01 ) )
				{
					mavlink_msg_command_ack_pack_chan( 
						get_CommulinkSysId() ,	//system id
						get_CommulinkCompId() ,	//component id
						port_index ,
						&msg_sd,
						msg->cmd,	//command
						*msg_handled ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED ,	//result
						0 ,	//progress
						0 ,	//param2
						msg->sd_sysid ,	//target system
						msg->sd_compid //target component
					);
					mavlink_msg_to_send_buffer(port->write, 
																		 port->lock,
																		 port->unlock,
																		 &msg_sd, 0, 0.01);
					mavlink_unlock_chan(port_index);
				}
			}
			
			*msg_available = false;
		}
	/*先返回消息处理结果*/
	
	if( modes[mode_index] != 0 )
	{
		xQueueReset(message_queue);
		sendLedSignal(LEDSignal_Start1);
		if( result != 0 )
			*result = modes[mode_index]->main_func( param1, param2 );
		else
			modes[mode_index]->main_func( param1, param2 );
		xQueueReset(message_queue);
		setLedMode(LEDMode_Normal1);
		return true;
	}
	return false;
}

static inline void send_VoltAbnormal( TIME& VoltageErrSendTime )
{
	sendLedSignal(LEDSignal_Err1);
	if( VoltageErrSendTime.get_pass_time() > 2 )
	{
		VoltageErrSendTime = TIME::now();
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{
			const Port* port = get_CommuPort(i);
			if( port && port->write )
			{	
				mavlink_message_t msg_sd;
				if(mavlink_lock_chan( i, 0.01 )){
					mavlink_msg_statustext_pack_chan( 
					get_CommulinkSysId() ,	//system id
					get_CommulinkCompId() ,	//component id
					i ,	//chan
					&msg_sd,
					MAV_SEVERITY_ERROR,
					"拒绝解锁! 检查电池电压!",
					0,0
					);				
					mavlink_msg_to_send_buffer(port->write, 
						 port->lock,
						 port->unlock,
						 &msg_sd, 0, 0.01);
					mavlink_unlock_chan(i);	
				}
			}
		}
	}
}
	

static void Modes_Server(void* pvParameters)
{
	//等待驱动初始化完成
	while( getInitializationCompleted() == false )
		os_delay(0.1);
	setLedManualCtrl( 0, 0, 30, true, 800 );
	os_delay(0.2);
	setLedMode(LEDMode_Processing1);
	
	//等待姿态解算系统准备完成
	while( get_Attitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	setLedManualCtrl( 0, 0, 30, true, 1000 );
	os_delay(0.2);
	setLedMode(LEDMode_Processing1);
	
	//等待位置解算系统准备完成
	while( get_Altitude_MSStatus() != MS_Ready )
		os_delay(0.1);
	setLedManualCtrl( 0, 0, 30, true, 1200 );
	os_delay(0.2);
	setLedMode(LEDMode_Processing1);
	sendLedSignal(LEDSignal_Start2);

	//初始化Aux处理
	init_process_AuxFuncs();
	
	//进入地面模式
	xQueueReset(message_queue);
	set_mav_mode( 
		MAV_MODE_STABILIZE_DISARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
		PX4_CUSTOM_MAIN_MODE_STABILIZED,
		0 );
	setLedMode(LEDMode_Normal1);
	uint16_t pre_enter_mode_counter = 0;
	uint8_t last_pre_enter_mode = 0;
	
	//状态
	AFunc MF_mode;
	uint8_t btn_zones[4];
	btn_zones[0] = btn_zones[1] = btn_zones[2] = btn_zones[3] = 255;
	uint8_t ModeButtonZone = 255;
	
	TIME VoltageErrSendTime(false);
	bool BatteryNormal = true;
	
	#define reset_States last_pre_enter_mode = 0;\
												btn_zones[0] = btn_zones[1] = btn_zones[2] = btn_zones[3] = 255;\
												ModeButtonZone = 255
												
	
	while(1)
	{
		os_delay(0.02);
		
		//关闭控制器
		Attitude_Control_Disable();
		set_mav_state(MAV_STATE_STANDBY);
		
		/*电压检测*/		
			float BatVoltage = get_MainBatteryVoltage_filted();
			if( BatVoltage < 7 )
			{	//没接电压检测
				BatteryNormal = true;
			}
			else
			{
				float STVoltage[2];
				if( ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 ) == PR_OK )
				{	//电压大于或小于标准电压的30%不允许解锁
					if( STVoltage[0]>7 && (BatVoltage>STVoltage[0]*1.3f || BatVoltage<STVoltage[0]*0.7f) )
						BatteryNormal = false;	
					else
						BatteryNormal = true;				
				}
			}
		
		/*电压检测*/		
			
		//获取接收机
		Receiver rc;
		getReceiver( &rc, 0, 0.02);

		if( rc.available )
		{	//接收机可用更新模式按钮状态
			uint8_t new_ModeButtonZone = get_RcButtonZone( rc.data[4], ModeButtonZone );
			if( ModeButtonZone<=5 && new_ModeButtonZone!=ModeButtonZone )
			{	//模式按钮改变重置锁定状态
				
			}
			ModeButtonZone = new_ModeButtonZone;
		}
		else
		{	//接收机不可用重置遥控状态
			ModeButtonZone = 255;
		}
		
		//处理Aux通道
		process_AuxFuncs(&rc);
		
		//获取消息
		bool msg_available;
		bool msg_handled = false;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		
		//读取模式配置
		ModeFuncCfg MFunc_cfg;
		ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
		
		//获取定位状态
		if( get_Position_MSStatus() == MS_Ready )
			setLedMode(LEDMode_Normal2);
		else
			setLedMode(LEDMode_Normal1);
		
		/*模式回传显示*/
			uint8_t p_mode = 0;
			if( ModeButtonZone==0 )
				p_mode = MFunc_cfg.Bt1PAFunc1[0];
			else if( ModeButtonZone==1 )
				p_mode = MFunc_cfg.Bt1PAFunc2[0];
			else if( ModeButtonZone==2 )
				p_mode = MFunc_cfg.Bt1PAFunc3[0];
			else if( ModeButtonZone==3 )
				p_mode = MFunc_cfg.Bt1PAFunc4[0];
			else if( ModeButtonZone==4 )
				p_mode = MFunc_cfg.Bt1PAFunc5[0];
			else
				p_mode = MFunc_cfg.Bt1PAFunc6[0];
			
			if( modes[p_mode] != 0 )
			{				
				modes[p_mode]->get_MavlinkMode( MFunc_cfg, rc, btn_zones, &MF_mode );
				
				uint16_t mav_mode, mav_main_mode, mav_sub_mode;
				if( MF_mode==AFunc_Stabilize )
				{
					mav_mode = MAV_MODE_STABILIZE_DISARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
					mav_main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
					mav_sub_mode = 0;
				}
				else if( MF_mode==AFunc_PosHold )
				{
					mav_mode = MAV_MODE_STABILIZE_DISARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
					mav_main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
					mav_sub_mode = 0;
				}
				else if( MF_mode==AFunc_AltHold )
				{
					mav_mode = MAV_MODE_STABILIZE_DISARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
					mav_main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
					mav_sub_mode = 0;
				}
				else if( MF_mode==AFunc_Mission )
				{
					mav_mode = MAV_MODE_STABILIZE_DISARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
					mav_main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
					mav_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
				}
				else if( MF_mode==AFunc_RTL )
				{
					mav_mode = MAV_MODE_STABILIZE_DISARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
					mav_main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
					mav_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
				}
				else if( MF_mode==AFunc_Offboard )
				{
					mav_mode = MAV_MODE_STABILIZE_DISARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					mav_main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD,
					mav_sub_mode = 0;
				}
				
				set_mav_mode( 
					mav_mode,
					mav_main_mode,
					mav_sub_mode );
			}
		/*模式回传显示*/
		
		if( rc.available )
		{
			uint8_t pre_enter_mode = 0;
			
			if( (rc.data[0] < 10.0f) && (rc.data[1] < 10.0f) && (rc.data[2] < 10.0f) && (rc.data[3] < 10.0f) )
				pre_enter_mode = 12;	//加速度校准
			else if( (rc.data[0] < 10.0f) && (rc.data[1] < 10.0f) && (rc.data[2] > 90.0f) && (rc.data[3] < 10.0f) )
				pre_enter_mode = 13;	//磁力计校准
			else if( (rc.data[0] < 10.0f) && (rc.data[1] > 90.0f) && (rc.data[2] > 45.0f) && (rc.data[2] < 55.0f) && (rc.data[3] > 90.0f) )
				pre_enter_mode = 11;	//温度系数校准
			else if( (rc.data[0] < 10.0f) && (rc.data[1] > 90.0f) && (rc.data[2] < 10.0f) && (rc.data[3] < 10.0f) )
			{
				if( BatteryNormal )
				{
					if( ModeButtonZone==0  )
						pre_enter_mode = MFunc_cfg.Bt1PAFunc1[0];
					else if( ModeButtonZone==1 )
						pre_enter_mode = MFunc_cfg.Bt1PAFunc2[0];
					else if( ModeButtonZone==2 )
						pre_enter_mode = MFunc_cfg.Bt1PAFunc3[0];
					else if( ModeButtonZone==3 )
						pre_enter_mode = MFunc_cfg.Bt1PAFunc4[0];
					else if( ModeButtonZone==4 )
						pre_enter_mode = MFunc_cfg.Bt1PAFunc5[0];
					else
						pre_enter_mode = MFunc_cfg.Bt1PAFunc6[0];
				}
				else
				{
					pre_enter_mode = 0;
					send_VoltAbnormal(VoltageErrSendTime);
				}
			}
			
			//计数进入模式
			if( pre_enter_mode==0 || pre_enter_mode!=last_pre_enter_mode )
				pre_enter_mode_counter = 0;
			else
			{
				if( ++pre_enter_mode_counter >= 50 )
				{
					if( modes[pre_enter_mode] != 0 )
					{
						changeMode( pre_enter_mode, &MF_mode, 0, 0,
												&msg_available, &msg_handled, &msg );
						reset_States;
						continue;
					}
				}				
			}
			last_pre_enter_mode = pre_enter_mode;
		}
		
		//处理消息
		if( msg_available )
		{
			switch( msg.cmd )
			{
				case 241:
				{	//进入校准
					if( msg.params[0]==1 )
					{	//陀螺校准
						changeMode( 11, &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
						reset_States;
					}
					else if( msg.params[1]==1 )
					{	//罗盘校准
						changeMode( 13, &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
						reset_States;
					}
					else if( msg.params[4]==1 )
					{	//加速度校准
						changeMode( 12, &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
						reset_States;
					}
					else if( msg.params[4]==1 )
					{	//加速度校准
						changeMode( 12, &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
						reset_States;
					}
					else if( msg.params[4]==2 )
					{	//水平校准
						changeMode( 14, &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
						reset_States;
					}
					else if( msg.params[3]==1 )
					{	//遥控器校准
						changeMode( 10, &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
						reset_States;
					}
					break;
				}
				case 176:
				{	//do set mode
					if( msg.params[0] == 0 )
					{
						if( BatteryNormal || msg.params[1]<30 )
						{
							changeMode( msg.params[1], &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
							reset_States;
						}
						else
							send_VoltAbnormal(VoltageErrSendTime);
					}
					else if( (int)msg.params[0] & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED )
					{	//mavlink定义模式
						uint32_t main_mode = msg.params[1];
						uint32_t sub_mode = msg.params[2];
						if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_MISSION )
						{	//自动模式
							msg_handled = true;
							MF_mode = AFunc_Mission;
							if( (int)msg.params[0] & MAV_MODE_FLAG_SAFETY_ARMED )
							{
								if( BatteryNormal )
								{
									changeMode( 32, &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
									reset_States;
								}
								else
									send_VoltAbnormal(VoltageErrSendTime);
							}
						}
						else if( main_mode==PX4_CUSTOM_MAIN_MODE_POSCTL  )
						{	//定点模式
							msg_handled = true;
							MF_mode = AFunc_PosHold;
						}
						else if( main_mode==PX4_CUSTOM_MAIN_MODE_ALTCTL  )
						{	//定高模式
							msg_handled = true;
							MF_mode = AFunc_AltHold;
						}
						if( (main_mode==PX4_CUSTOM_MAIN_MODE_AUTO || main_mode==0) && sub_mode==PX4_CUSTOM_SUB_MODE_AUTO_RTL )
						{	//返航
							msg_handled = true;
							MF_mode = AFunc_RTL;
						}
						else if( main_mode==PX4_CUSTOM_MAIN_MODE_OFFBOARD )
						{	//OFFBOARD
							msg_handled = true;
							MF_mode = AFunc_Offboard;
						}
					}
					break;
				}
				
				case 22:
				{	//takeoff起飞
					if( BatteryNormal )
					{
						msg_handled = true;
						MF_mode = AFunc_TakeOff;
						changeMode( 32, &MF_mode, msg.params[6]*100, 0,
												&msg_available, &msg_handled, &msg );
						reset_States;
					}
					else
						send_VoltAbnormal(VoltageErrSendTime);
					break;
				}
				
				case MAV_CMD_COMPONENT_ARM_DISARM:
				{	//解锁
					if( msg.params[0] == 1 )
					{
						if( BatteryNormal )
						{
							msg_handled = true;
							changeMode( 32, &MF_mode, 0, 0,
													&msg_available, &msg_handled, &msg );
							reset_States;
						}
						else
							send_VoltAbnormal(VoltageErrSendTime);
					}
					break;
				}
				
			}
		}
		
		/*返回消息处理结果*/
			if( msg_available )
			{
				uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
				const Port* port = get_CommuPort( port_index );
				if( (msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK && port->write )
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
							0 ,	//progress
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
}

void ModeRegister( Mode_Base* mode, uint8_t id )
{
	if( modes[id] == 0 )
		modes[id] = mode;
}

void init_Modes()
{
	//注册模式
	new M10_RCCalib();
	new M11_TempCalib();
	new M12_AccCalib();
	new M13_MagCalib();
	new M14_HorizonCalib();
	
	new M30_AttCtrl();
	new M32_PosCtrl();
	new M35_Auto1();
	
	//注册参数
	ModeFuncCfg initial_cfg;
	//按钮1解锁前功能（模式序号）
	initial_cfg.Bt1PAFunc1[0] = 32;
	initial_cfg.Bt1PAFunc2[0] = 32;
	initial_cfg.Bt1PAFunc3[0] = 32;
	initial_cfg.Bt1PAFunc4[0] = 32;
	initial_cfg.Bt1PAFunc5[0] = 32;
	initial_cfg.Bt1PAFunc6[0] = 32;
	//按钮1解锁后功能
	initial_cfg.Bt1AFunc1[0] = 1;
	initial_cfg.Bt1AFunc2[0] = 1;
	initial_cfg.Bt1AFunc3[0] = 22;
	initial_cfg.Bt1AFunc4[0] = 22;
	initial_cfg.Bt1AFunc5[0] = 2;
	initial_cfg.Bt1AFunc6[0] = 2;
	//任务执行按钮
	initial_cfg.MissionBt[0] = 13;
	//返航按钮
	initial_cfg.RTLBt[0] = 12;
	//安全按钮
	initial_cfg.SafeBt[0] = 0;
	//中位死区
	initial_cfg.NeutralZone[0] = 5.0;
	//位置速度响应曲线系数
	initial_cfg.PosVelAlpha[0] = 1.6;
	//姿态响应曲线系数
	initial_cfg.AttAlpha[0] = 1.5;
	//重置航点
	initial_cfg.RstWp0[0] = 0;
	
	MAV_PARAM_TYPE param_types[] = {
		//按钮1解锁前功能（模式序号）
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		//按钮1解锁后功能
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		MAV_PARAM_TYPE_UINT8 ,
		//任务执行按钮
		MAV_PARAM_TYPE_UINT8 ,
		//返航按钮
		MAV_PARAM_TYPE_UINT8 ,
		//安全按钮
		MAV_PARAM_TYPE_UINT8 ,
		//中位死区
		MAV_PARAM_TYPE_REAL32 ,
		//位置速度响应曲线系数
		MAV_PARAM_TYPE_REAL32 ,
		//姿态响应曲线系数
		MAV_PARAM_TYPE_REAL32 ,
		//解锁使用0号航点
		MAV_PARAM_TYPE_UINT8 ,
	};
	SName param_names[] = {
		//按钮1解锁前功能（模式序号）
		"MFunc_Bt1PAF1" ,
		"MFunc_Bt1PAF2" ,
		"MFunc_Bt1PAF3" ,
		"MFunc_Bt1PAF4" ,
		"MFunc_Bt1PAF5" ,
		"MFunc_Bt1PAF6" ,
		//按钮1解锁后功能
		"MFunc_Bt1AF1" ,
		"MFunc_Bt1AF2" ,
		"MFunc_Bt1AF3" ,
		"MFunc_Bt1AF4" ,
		"MFunc_Bt1AF5" ,
		"MFunc_Bt1AF6" ,
		//任务执行按钮
		"MFunc_MissionBt" ,
		//返航按钮
		"MFunc_RTLBt" ,
		//安全按钮
		"MFunc_SafeBt" ,
		//中位死区
		"MFunc_NeutralZ" ,
		//位置速度响应曲线系数
		"MFunc_PVAlpha" ,
		//姿态响应曲线系数
		"MFunc_AttAlpha" ,
		//解锁使用0号航点
		"MFunc_RstWp0"
	};
	ParamGroupRegister( "MFunc", 2, sizeof(ModeFuncCfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
	
	//注册当前航点信息
	CurrentWpInf initial_CurrentWpInf;
	initial_CurrentWpInf.CurrentWp[0] = 0;
	initial_CurrentWpInf.line_x = 0;
	initial_CurrentWpInf.line_y = 0;
	initial_CurrentWpInf.line_z = 0;
	initial_CurrentWpInf.line_fs = -1;
	ParamGroupRegister( "CurrentWp", 1, sizeof(CurrentWpInf)/8, 0, 0, (uint64_t*)&initial_CurrentWpInf );
	
	init_AuxFuncs();
	
	xTaskCreate( Modes_Server, "Modes", 4096, NULL, SysPriority_UserTask, NULL);
}