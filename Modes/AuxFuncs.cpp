#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "drv_PWMOut.hpp"
#include "Commulink.hpp"
#include "StorageSystem.hpp"
#include "MeasurementSystem.hpp"
#include "ControlSystem.hpp"
#include "event_groups.h"
#include "semphr.h"
//保存之前通道值用于触�?
static double last_Channel_values[16];
//云台是否自动控制
static float GimbalCtrl_LockedAtt[16];

//拍照次数
static uint16_t PhotoCnt = 0;
//相片序号
static uint16_t PhotoIndex = 1;

//发送完成标�?
static EventGroupHandle_t IO_events = xEventGroupCreate();

/*热靴触发*/
	static SemaphoreHandle_t PosLogMutex = xSemaphoreCreateMutex();
	static bool SD_Pos_Record()
	{
		//获取时间
		RTC_TimeStruct RTC_Time;
		RTC_Time = Get_RTC_Time();
		//获取速度
		vector3<double> vel;
		get_VelocityENU_Ctrl(&vel);	
		
		//获取姿�?
		Quaternion airframe_quat;
		get_Attitude_quat(&airframe_quat);
		airframe_quat.Enu2Ned();	
		
		static int8_t global_pos_ind = -1;
		PosSensorHealthInf3 posInf;
		if( global_pos_ind < 0 )
		{	//第一次获取最优位置传感器
			if( get_OptimalGlobal_XYZ(&posInf) )
				global_pos_ind = posInf.sensor_ind;
		}
		else
			get_PosSensorHealth_XYZ( &posInf, global_pos_ind );
		
		double lat = 0;
		double lon = 0;
		double alt = 0;
		double accN = 999999;
		double accE = 999999;
		double accD = 999999;
		uint8_t fix_type = 0;
		uint16_t week = 0;
		double TOW = 0;
		
		if( global_pos_ind >= 0 )
		{
			Position_Sensor gps_sensor;
			if( GetPositionSensor( global_pos_ind, &gps_sensor ) )
			{
				if( gps_sensor.data.available && gps_sensor.data.sensor_type==Position_Sensor_Type_GlobalPositioning )
				{
					//计算经纬�?
					map_projection_reproject( &posInf.mp, 
						posInf.PositionENU.x+posInf.HOffset.x, 
						posInf.PositionENU.y+posInf.HOffset.y,
						&lat, &lon );
					//高度
					alt = posInf.PositionENU.z + posInf.HOffset.z;
					alt *= 0.01;
					//精度
					accN = gps_sensor.inf.addition_inf[4]*0.01;
					accE = gps_sensor.inf.addition_inf[4]*0.01;
					accD = gps_sensor.inf.addition_inf[5]*0.01;
					//week
					week = gps_sensor.inf.addition_inf[2];
					//TOW
					TOW = gps_sensor.inf.addition_inf[3];
					//fix
					if( gps_sensor.inf.addition_inf[1]==1 )
						fix_type=0;
					else if( gps_sensor.inf.addition_inf[1]==5 )
						fix_type=34;
					else if( gps_sensor.inf.addition_inf[1]==6 )
						fix_type=50;
					else
						fix_type=16;
				}
			}
		}		

		xSemaphoreTake( PosLogMutex, portMAX_DELAY );
		
			uint16_t photo_ind = PhotoIndex;
			char pos_txt_buf[200];
			int n = sprintf(
				pos_txt_buf,
				"%4d\t%6.6f\t[%4d]\t%4d,N\t%4d,E\t%4d,V \t%11.8f,Lat \t%11.8f,Lon \t%6.3f,Ellh \t%9.6f,    %9.6f,    %9.6f,    %2d,Q\r\n",
				PhotoIndex++,
				TOW,
				week,
				0,
				0,
				0,
				lat,
				lon,
				alt,
				accN,
				accE,
				accD,	
				fix_type
			);			
			
		xSemaphoreGive(PosLogMutex);
		
		if(!SDLog_Txt1( pos_txt_buf, n ))
			return false;
		
		//发送拍照信息到地面�?
		double Altitude_Local=0;
		vector3<double> Position;
		get_Position_Ctrl(&Position);
		double homeZ;
		double heightAboveGround = 0;
		if( getHomeLocalZ( &homeZ, 0, 0.01 ) )
			heightAboveGround = Position.z - homeZ;
		
		mavlink_message_t msg_sd;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{	//遍历所有端�?	
			if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
			{
				mavlink_msg_camera_feedback_pack_chan(
					get_CommulinkSysId() ,	// system id
					get_CommulinkCompId() ,	// component id
					i ,
					&msg_sd,	  
					TIME::get_System_Run_Time() * 1e3 , // boot ms					
					1, //target_system  System ID
					global_pos_ind, // camera_id
					photo_ind, //Image index
					lat*1e7, // lat [degE7]
					lon*1e7, // lon [degE7]
					alt,  // alt_msl [m] Altitude (MSL).
					heightAboveGround*0.01, //alt_rel [m] Altitude (Relative to HOME location).
					0, //Camera Roll
					0, //Camera Pitch 
					0, //Camera Yaw
					30, //foc_len [mm] Focal Length
					0,  //0:Shooting photos, not video
					photo_ind	//Completed image captures
				);
				const Port* port = get_CommuPort(i);
				if(port->write){
					mavlink_msg_to_send_buffer(port->write, 
																	 port->lock,
																	 port->unlock,
																	 &msg_sd, 0, 2/configTICK_RATE_HZ);
				}
				mavlink_unlock_chan(i);	
			}	
		}
		return true;
	}

/*热靴触发*/

bool setAuxPWM( float PWMus, uint8_t ind )
{
	if( getInitializationCompleted() == false )
		return false;
	if( PWMus<0 || PWMus>100000 )
		return false;
	
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
	uint16_t aux_cfg = ((uint16_t*)&aux_configs)[ind*4];
	if( aux_cfg>=0 && aux_cfg<=16 )
	{
		GimbalCtrl_LockedAtt[ind] = PWMus;
		return true;
	}
	return false;
}
	
void init_process_AuxFuncs()
{
	Receiver rc;
	if( getReceiver(&rc) )
	{
		//复位保存之前通道
		for( uint8_t i = 0; i < rc.raw_available_channels; ++i )
			last_Channel_values[i] = rc.raw_data[i];
		for( uint8_t i = rc.raw_available_channels; i < 16; ++i )
			last_Channel_values[i] = -200;
	}
	else
	{
		//复位保存之前通道
		for( uint8_t i = 0; i < 16; ++i )
			last_Channel_values[i] = -200;
	}
	//复位运动自动控制标志
	for( uint8_t i = 0; i < 16; ++i )
		GimbalCtrl_LockedAtt[i] = 20000;
}

static SemaphoreHandle_t CamMutex = xSemaphoreCreateRecursiveMutex();
bool AuxCamTakePhoto()
{
	if( getInitializationCompleted() == false )
		return false;
	uint8_t cam_chans = 0;
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
	if( xSemaphoreTakeRecursive( CamMutex, 0.1*configTICK_RATE_HZ ) )
	{
		uint8_t MainMotorCount = get_MainMotorCount();		
		
		//拉低（拉高）相机PWM
		for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
		{
			uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
			if( aux_cfg>=25 && aux_cfg<=48 )
			{
				Aux_PWM_Out( aux_configs.Aux_CamOnPwm[0]*0.1-100, i );
				++cam_chans;
			}
		}
		
		//无相机返�?
		if(cam_chans==0)
		{
			xSemaphoreGiveRecursive(CamMutex);
			return false;
		}
		
		//拉高（拉低）相机PWM
		os_delay(aux_configs.Aux_CamShTime[0]);
		for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
		{
			uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
			if( aux_cfg>=25 && aux_cfg<=40 )
				Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
		}
		
		xSemaphoreTake( PosLogMutex, portMAX_DELAY );
			++PhotoCnt;
		xSemaphoreGive(PosLogMutex);
		
		mavlink_message_t msg_sd;
		for( uint8_t i = 0 ; i < MAVLINK_COMM_NUM_BUFFERS ; ++i )
		{	//遍历所有端�?	
			if( mavlink_lock_chan( i, 2/configTICK_RATE_HZ ) )
			{
				mavlink_msg_camera_status_pack_chan(
					get_CommulinkSysId() ,	// system id
					get_CommulinkCompId() ,	// component id
					i ,
					&msg_sd,	  
					TIME::get_System_Run_Time() * 1e6 , // boot us	
					255,	//target_system
					1, //cam index
					PhotoCnt, //image index
					1	,	//event id
					0, // p1
					0, // p2
					0, // p3
					0 //p4
				);
				const Port* port = get_CommuPort(i);
				if(port->write){
					mavlink_msg_to_send_buffer(port->write, 
																	 port->lock,
																	 port->unlock,
																	 &msg_sd, 0, 2/configTICK_RATE_HZ);
				}
				mavlink_unlock_chan(i);	
			}	
		}
		SD_Pos_Record();
		
		xSemaphoreGiveRecursive(CamMutex);
		return true;
	}
	else
		return false;
}

void process_AuxFuncs(const Receiver* rc)
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );

	uint8_t MainMotorCount = get_MainMotorCount();
	if( xSemaphoreTakeRecursive( CamMutex, 0 ) )	
	{
		for( uint8_t i = MainMotorCount; i < PWMChannelsCount; ++i )
		{
			uint16_t aux_cfg = ((uint16_t*)&aux_configs)[i*4];
			float aux_param1 = ((float*)&aux_configs.Aux1Param1)[i*2];
			float aux_param2 = ((float*)&aux_configs.Aux1Param2)[i*2];
			if( aux_cfg==0 )
			{	//映射遥控器通道
				if( GimbalCtrl_LockedAtt[i] < 100000 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-1000)*0.1, i );
			}
			if( aux_cfg>=1 && aux_cfg<=16 )
			{	//映射遥控器通道
				if( GimbalCtrl_LockedAtt[i] < 100000 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-1000)*0.1, i );
				uint8_t ref_chan = aux_cfg - 1;
				if( rc->connected && rc->raw_available_channels>ref_chan )
				{
					if( fabs(GimbalCtrl_LockedAtt[i])<100000 && last_Channel_values[i]>-100 )
					{	//Aux自动控制
						//通道变化大于阈值才调整Aux
						if( fabs(rc->raw_data[ref_chan]-last_Channel_values[i])>10 )
						{
							Aux_PWM_Out( (rc->raw_data[ref_chan]-50.0)*aux_param1+50+aux_param2*0.1, i );
							last_Channel_values[i] = rc->raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 200000;
						}
					}
					else
					{	//Aux手动控制
						Aux_PWM_Out( (rc->raw_data[ref_chan]-50.0)*aux_param1+50+aux_param2*0.1, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
					}
				}
				else
					last_Channel_values[i] = -200;
			}
			else if( aux_cfg>=501 && aux_cfg<=516 )
			{	//映射遥控器通道（开关量）
				uint8_t ref_chan = aux_cfg - 501;
				if( rc->connected && rc->raw_available_channels>ref_chan )
				{
					if( rc->raw_data[ref_chan] > 50 )
					{
						if( aux_param1 > 0 )
							Aux_PWM_Out( 6000, i );
						else
							Aux_PWM_Out( 0, i );
					}
					else
					{
						if( aux_param1 > 0 )
							Aux_PWM_Out( 0, i );
						else
							Aux_PWM_Out( 6000, i );
					}
				}
			}
			else if( aux_cfg>=25 && aux_cfg<=48 )
			{	//用遥控器对应通道进行相机快门触发（raw_data＿
				uint8_t ref_chan = aux_cfg - 25;
				if( rc->connected && rc->raw_available_channels>ref_chan )
				{
					if( last_Channel_values[i]>-100 )
					{
						if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 15 )
						{	//触发相机
							if( last_Channel_values[i] > -100 )							
							{
								AuxCamTakePhoto();					
							}
							last_Channel_values[i] = rc->raw_data[ref_chan];
						}
						else
							Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
					}
					else
					{	//旧通道数据不可用
						Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
					}
				}
				else
				{	//无遥控器
					Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
					last_Channel_values[i] = -200;
				}
			}
			else if( aux_cfg>=49 && aux_cfg<=72 )
			{	//用遥控器对应通道进行无刷云台俯仰控制（raw_data＿
				double angle90 = (aux_configs.Aux_BsYTPit90[0]-1000)*0.1;
				double angle0 = (aux_configs.Aux_BsYTPit0[0]-1000)*0.1;
				double scale = (angle90 - angle0)/90.0;
				if( GimbalCtrl_LockedAtt[i] < 200 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-0)*scale + angle0, i );
				
				uint8_t ref_chan = aux_cfg - 49;
				if( rc->connected && rc->raw_available_channels>ref_chan )
				{
					if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
					{	//云台自动控制
						//通道变化大于阈值才调整云台
						if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10 )
						{
							float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
							angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
							Aux_PWM_Out( (angle-0)*scale + angle0, i );
							last_Channel_values[i] = rc->raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 200000;
						}
					}
					else
					{	//云台手动控制
						float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
					}
				}
				else
				{	//无遥控信号
					last_Channel_values[i] = -200;
					if( GimbalCtrl_LockedAtt[i] > 200 )
					{	//非自动控制且无遥控信叿
						//锁定0角度
						float angle = 0;
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
					}
				}
			}
			else if( aux_cfg>=73 && aux_cfg<=96 )
			{	//用遥控器对应通道进行舵机云台俯仰控制（raw_data＿
				double angle90 = (aux_configs.Aux_StYTPit90[0]-1000)*0.1;
				double angle0 = (aux_configs.Aux_StYTPit0[0]-1000)*0.1;
				double scale = (angle90 - angle0)/90.0;
				Quaternion quat;
				get_Airframe_quat(&quat);
				double pitch = rad2degree(quat.getPitch());
				if( GimbalCtrl_LockedAtt[i] < 200 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-pitch-0)*scale + angle0, i );
				
				uint8_t ref_chan = aux_cfg - 73;
				if( rc->connected && rc->raw_available_channels>ref_chan )
				{
					if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
					{	//云台自动控制
						//通道变化大于阈值才调整云台
						if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10 )
						{
							float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 							
							angle -= pitch;
							angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
							Aux_PWM_Out( (angle-0)*scale + angle0, i );
							last_Channel_values[i] = rc->raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 20000;
						}
					}
					else
					{	//云台手动控制
						float angle = (rc->raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
						angle -= pitch;
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
					}
				}
				else 
				{	//无遥控信号
					last_Channel_values[i] = -200;
					if( GimbalCtrl_LockedAtt[i] > 200 )
					{	//非自动控制且无遥控信叿
						//锁定0角度
						float angle = 0;
						Aux_PWM_Out( (angle-pitch-0)*scale + angle0, i );
					}
				}
			}
			else if( aux_cfg>=97 && aux_cfg<=120 )
			{	//用遥控器对应通道进行舵机云台横滚控制（raw_data＿
				double angleN45 = (aux_configs.Aux_StYTRolN45[0]-1000)*0.1;
				double angleP45 = (aux_configs.Aux_StYTRolP45[0]-1000)*0.1;
				double angle0 = (angleN45 + angleP45) / 2;
				double scale = (angleP45 - angle0)/45.0;
				Quaternion quat;
				get_Airframe_quat(&quat);
				double roll = rad2degree(quat.getRoll());
				if( GimbalCtrl_LockedAtt[i] < 200 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-roll-0)*scale + angleN45, i );
				
				uint8_t ref_chan = aux_cfg - 97;
				if( rc->connected && rc->raw_available_channels>ref_chan )
				{
					if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
					{	//云台自动控制
						//通道变化大于阈值才调整云台
						if( fabs(rc->raw_data[ref_chan] - last_Channel_values[i]) > 10 )
						{
							float angle = (rc->raw_data[ref_chan]-50.0)*(45.0/50.0)*aux_param1 + aux_param2; 
							angle -= roll;
							angle = constrain( angle, aux_configs.Aux_YTRollMax[0] );
							Aux_PWM_Out( (angle-0)*scale + angle0, i );
							last_Channel_values[i] = rc->raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 20000;
						}
					}
					else
					{	//云台手动控制
						float angle = (rc->raw_data[ref_chan]-50.0)*(45.0/50.0)*aux_param1 + aux_param2; 
						angle -= roll;
						angle = constrain( angle, aux_configs.Aux_YTRollMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = rc->raw_data[ref_chan];
					}
				}
				else
				{	//无遥控器信号
					if( GimbalCtrl_LockedAtt[i] > 200 )
					{	//非自动控制且无遥控信叿
						//锁定0角度
						float angle = 0;
						Aux_PWM_Out( (angle-roll-0)*scale + angle0, i );
					}
				}
			}
			
			
			
			else if( aux_cfg>=1001 && aux_cfg<=1016 )
			{	//映射虚拟摇杆通道
				Receiver jrc;
				getJoyStick(&jrc,0);
				
				if( GimbalCtrl_LockedAtt[i] < 100000 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-1000)*0.1, i );
				uint8_t ref_chan = aux_cfg - 1001;
				if( jrc.connected && jrc.raw_available_channels>ref_chan )
				{
					if( fabs(GimbalCtrl_LockedAtt[i])<100000 && last_Channel_values[i]>-100 )
					{	//Aux自动控制
						//通道变化大于阈值才调整Aux
						if( fabs(jrc.raw_data[ref_chan]-last_Channel_values[i])>10 )
						{
							Aux_PWM_Out( (jrc.raw_data[ref_chan]-50.0)*aux_param1+50+aux_param2*0.1, i );
							last_Channel_values[i] = jrc.raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 200000;
						}
					}
					else
					{	//Aux手动控制
						Aux_PWM_Out( (jrc.raw_data[ref_chan]-50.0)*aux_param1+50+aux_param2*0.1, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
					}
				}
				else
					last_Channel_values[i] = -200;
			}
			else if( aux_cfg>=1025 && aux_cfg<=1048 )
			{	//用虚拟摇杆对应通道进行相机快门触发（raw_data＿
				Receiver jrc;
				getJoyStick(&jrc,0);

				uint8_t ref_chan = aux_cfg - 1025;
				if( jrc.connected && jrc.raw_available_channels>ref_chan )
				{
					if( last_Channel_values[i]>-100 )
					{
						if( fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 15 )
						{	//触发相机
							if( last_Channel_values[i] > -100 )							
							{
								AuxCamTakePhoto();					
							}
							last_Channel_values[i] = jrc.raw_data[ref_chan];
						}
						else
							Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
					}
					else
					{	//旧通道数据不可用
						Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
					}
				}
				else
				{	//无遥控器
					Aux_PWM_Out( aux_configs.Aux_CamOffPwm[0]*0.1-100, i );
					last_Channel_values[i] = -200;
				}
			}
			else if( aux_cfg>=1049 && aux_cfg<=1072 )
			{	//用虚拟摇杆对应通道进行无刷云台俯仰控制（raw_data＿
				Receiver jrc;
				getJoyStick(&jrc,0);
				
				double angle90 = (aux_configs.Aux_BsYTPit90[0]-1000)*0.1;
				double angle0 = (aux_configs.Aux_BsYTPit0[0]-1000)*0.1;
				double scale = (angle90 - angle0)/90.0;
				if( GimbalCtrl_LockedAtt[i] < 200 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-0)*scale + angle0, i );
				
				uint8_t ref_chan = aux_cfg - 1049;
				if( jrc.connected && jrc.raw_available_channels>ref_chan )
				{
					if( fabs(GimbalCtrl_LockedAtt[i]) < 200 && last_Channel_values[i]>-100 )
					{	//云台自动控制
						//通道变化大于阈值才调整云台
						if( fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10 )
						{
							float angle = (jrc.raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
							angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
							Aux_PWM_Out( (angle-0)*scale + angle0, i );
							last_Channel_values[i] = jrc.raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 200000;
						}
					}
					else
					{	//云台手动控制
						float angle = (jrc.raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
					}
				}
				else
				{	//无遥控信号
					last_Channel_values[i] = -200;
					if( GimbalCtrl_LockedAtt[i] > 200 )
					{	//非自动控制且无遥控信叿
						//锁定0角度
						float angle = 0;
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
					}
				}
			}
			else if( aux_cfg>=1073 && aux_cfg<=1096 )
			{	//用虚拟摇杆对应通道进行舵机云台俯仰控制（raw_data＿
				Receiver jrc;
				getJoyStick(&jrc,0);

				double angle90 = (aux_configs.Aux_StYTPit90[0]-1000)*0.1;
				double angle0 = (aux_configs.Aux_StYTPit0[0]-1000)*0.1;
				double scale = (angle90 - angle0)/90.0;
				Quaternion quat;
				get_Airframe_quat(&quat);
				double pitch = rad2degree(quat.getPitch());
				if( GimbalCtrl_LockedAtt[i] < 200 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-pitch-0)*scale + angle0, i );
				
				uint8_t ref_chan = aux_cfg - 1073;
				if( jrc.connected && jrc.raw_available_channels>ref_chan )
				{
					if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
					{	//云台自动控制
						//通道变化大于阈值才调整云台
						if( fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10 )
						{
							float angle = (jrc.raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 							
							angle -= pitch;
							angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
							Aux_PWM_Out( (angle-0)*scale + angle0, i );
							last_Channel_values[i] = jrc.raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 20000;
						}
					}
					else
					{	//云台手动控制
						float angle = (jrc.raw_data[ref_chan]-50.0)*(60.0/50.0)*aux_param1 + 45+aux_param2; 
						angle -= pitch;
						angle = constrain( angle, aux_configs.Aux_YTPitMin[0], aux_configs.Aux_YTPitMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
					}
				}
				else 
				{	//无遥控信号
					last_Channel_values[i] = -200;
					if( GimbalCtrl_LockedAtt[i] > 200 )
					{	//非自动控制且无遥控信叿
						//锁定0角度
						float angle = 0;
						Aux_PWM_Out( (angle-pitch-0)*scale + angle0, i );
					}
				}
			}
			else if( aux_cfg>=1097 && aux_cfg<=1120 )
			{	//用虚拟摇杆对应通道进行舵机云台横滚控制（raw_data＿
				Receiver jrc;
				getJoyStick(&jrc,0);
				
				double angleN45 = (aux_configs.Aux_StYTRolN45[0]-1000)*0.1;
				double angleP45 = (aux_configs.Aux_StYTRolP45[0]-1000)*0.1;
				double angle0 = (angleN45 + angleP45) / 2;
				double scale = (angleP45 - angle0)/45.0;
				Quaternion quat;
				get_Airframe_quat(&quat);
				double roll = rad2degree(quat.getRoll());
				if( GimbalCtrl_LockedAtt[i] < 200 )
					Aux_PWM_Out( (GimbalCtrl_LockedAtt[i]-roll-0)*scale + angleN45, i );
				
				uint8_t ref_chan = aux_cfg - 97;
				if( jrc.connected && jrc.raw_available_channels>ref_chan )
				{
					if( fabs(GimbalCtrl_LockedAtt[i])<200 && last_Channel_values[i]>-100 )
					{	//云台自动控制
						//通道变化大于阈值才调整云台
						if( fabs(jrc.raw_data[ref_chan] - last_Channel_values[i]) > 10 )
						{
							float angle = (jrc.raw_data[ref_chan]-50.0)*(45.0/50.0)*aux_param1 + aux_param2; 
							angle -= roll;
							angle = constrain( angle, aux_configs.Aux_YTRollMax[0] );
							Aux_PWM_Out( (angle-0)*scale + angle0, i );
							last_Channel_values[i] = jrc.raw_data[ref_chan];
							GimbalCtrl_LockedAtt[i] = 20000;
						}
					}
					else
					{	//云台手动控制
						float angle = (jrc.raw_data[ref_chan]-50.0)*(45.0/50.0)*aux_param1 + aux_param2; 
						angle -= roll;
						angle = constrain( angle, aux_configs.Aux_YTRollMax[0] );
						Aux_PWM_Out( (angle-0)*scale + angle0, i );
						last_Channel_values[i] = jrc.raw_data[ref_chan];
					}
				}
				else
				{	//无遥控器信号
					if( GimbalCtrl_LockedAtt[i] > 200 )
					{	//非自动控制且无遥控信叿
						//锁定0角度
						float angle = 0;
						Aux_PWM_Out( (angle-roll-0)*scale + angle0, i );
					}
				}
			}
		}
		xSemaphoreGiveRecursive(CamMutex);
	}
}


bool AuxGimbalSetAngle( double angle )
{
	AuxFuncsConfig aux_configs;
	ReadParamGroup( "AuxCfg", (uint64_t*)&aux_configs, 0 );
	
	uint8_t MainMotorsCount = get_MainMotorCount();
	uint8_t AuxChannelsCount = get_AuxChannelCount();
	for( uint8_t i = MainMotorsCount; i < MainMotorsCount+AuxChannelsCount; ++i )
	{
		uint8_t aux_cfg = ((uint8_t*)&aux_configs)[i*8];
		if( aux_cfg>=49 && aux_cfg<=72 )
		{	//无刷云台俯仰�?
			angle = constrain( angle, (double)aux_configs.Aux_YTPitMin[0], (double)aux_configs.Aux_YTPitMax[0] );
			GimbalCtrl_LockedAtt[i] = angle;
		}
		else if( aux_cfg>=73 && aux_cfg<=96 )
		{	//无刷云台俯仰�?
			angle = constrain( angle, (double)aux_configs.Aux_YTPitMin[0], (double)aux_configs.Aux_YTPitMax[0] );
			GimbalCtrl_LockedAtt[i] = angle;
		}
		else if( aux_cfg>=97 && aux_cfg<=120 )
		{	//无刷云台俯仰�?
			angle = constrain( angle, (double)aux_configs.Aux_YTRollMax[0] );
			GimbalCtrl_LockedAtt[i] = angle;
		}
	}
	return true;
}

void init_AuxFuncs()
{
	//注册通信参数	
	AuxFuncsConfig initial_cfg;
	initial_cfg.Aux1Func[0] = 0;
	initial_cfg.Aux2Func[0] = 0;
	initial_cfg.Aux3Func[0] = 0;
	initial_cfg.Aux4Func[0] = 0;
	initial_cfg.Aux5Func[0] = 0;
	initial_cfg.Aux6Func[0] = 0;
	initial_cfg.Aux7Func[0] = 0;
	initial_cfg.Aux8Func[0] = 0;
	initial_cfg.Aux1Param1[0] = 1;
	initial_cfg.Aux2Param1[0] = 1;
	initial_cfg.Aux3Param1[0] = 1;
	initial_cfg.Aux4Param1[0] = 1;
	initial_cfg.Aux5Param1[0] = 1;
	initial_cfg.Aux6Param1[0] = 1;
	initial_cfg.Aux7Param1[0] = 1;
	initial_cfg.Aux8Param1[0] = 1;
	initial_cfg.Aux1Param2[0] = 0;
	initial_cfg.Aux2Param2[0] = 0;
	initial_cfg.Aux3Param2[0] = 0;
	initial_cfg.Aux4Param2[0] = 0;
	initial_cfg.Aux5Param2[0] = 0;
	initial_cfg.Aux6Param2[0] = 0;
	initial_cfg.Aux7Param2[0] = 0;
	initial_cfg.Aux8Param2[0] = 0;
	initial_cfg.Aux_CamOnPwm[0] = 2000;
	initial_cfg.Aux_CamOffPwm[0] = 1000;
	initial_cfg.Aux_CamShTime[0] = 0.1;
	initial_cfg.Aux_BsYTPit0[0] = 1000;
	initial_cfg.Aux_BsYTPit90[0] = 2000;
	initial_cfg.Aux_StYTPit0[0] = 1250;
	initial_cfg.Aux_StYTPit90[0] = 1750;
	initial_cfg.Aux_StYTRolN45[0] = 1000;
	initial_cfg.Aux_StYTRolP45[0] = 2000;
	initial_cfg.Aux_YTPitMin[0] = -20;
	initial_cfg.Aux_YTPitMax[0] = 120;
	initial_cfg.Aux_YTRollMax[0] = 45;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_UINT16 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
	};
	SName param_names[] = {
		"Aux_1Func" ,
		"Aux_2Func" ,
		"Aux_3Func" ,
		"Aux_4Func" ,
		"Aux_5Func" ,
		"Aux_6Func" ,
		"Aux_7Func" ,
		"Aux_8Func" ,
		"Aux_1Param1" ,
		"Aux_2Param1" ,
		"Aux_3Param1" ,
		"Aux_4Param1" ,
		"Aux_5Param1" ,
		"Aux_6Param1" ,
		"Aux_7Param1" ,
		"Aux_8Param1" ,
		"Aux_1Param2" ,
		"Aux_2Param2" ,
		"Aux_3Param2" ,
		"Aux_4Param2" ,
		"Aux_5Param2" ,
		"Aux_6Param2" ,
		"Aux_7Param2" ,
		"Aux_8Param2" ,
		"Aux_CamOnPwm" ,
		"Aux_CamOffPwm" ,
		"Aux_CamShTime" ,
		"Aux_BsYTPit0" ,
		"Aux_BsYTPit90" ,
		"Aux_StYTPit0" ,
		"Aux_StYTPit90" ,
		"Aux_StYTRol-45" ,
		"Aux_StYTRol+45" ,
		"Aux_StPitMin" ,
		"Aux_StPitMax" ,
		"Aux_StRolMax" ,
	};
	ParamGroupRegister( "AuxCfg", 1, sizeof(initial_cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
}