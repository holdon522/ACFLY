#include "ctrl_Attitude.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "Parameters.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "TD4.hpp"
#include "ESO_AngularRate.hpp"
#include "ESO_h.hpp"
#include "Filters_LP.hpp"
#include "drv_PWMOut.hpp"
#include "smooth_kp.hpp"
#include "TD3_3D.hpp"
#include "vector2.hpp"
#include "Commulink.hpp"

#include "StorageSystem.hpp"

/*参数*/
	//控制参数
	struct PosCtrlCfg
	{
		//默认XY航线速度
		float AutoVXY[2];
		//默认Z向上航线速度
		float AutoVZUp[2];
		//默认Z向下航线速度
		float AutoVZDown[2];
		//默认XYZ航线速度
		float AutoVXYZ[2];
		
		//高度前馈滤波器
		float Z_TD4P1[2];
		float Z_TD4P2[2];
		float Z_TD4P3[2];
		float Z_TD4P4[2];
		
		//位置反馈增益
		float P1[2];
		//速度反馈增益
		float P2[2];
		//加速度反馈增益
		float P3[2];
		//水平速度控制速度反馈增益
		float P2_VelXY[2];
		//水平速度前馈
		float VelXYFF[2];
		//高度加速度反馈滤波器
		float ZSense[2];
		
		//最大水平速度
		float maxVelXY[2];
		//最大水平加速度
		float maxAccXY[2];
		//最大水平加加速度
		float maxJerkXY[2];
		//自动模式最大水平速度
		float maxAutoVelXY[2];
		//自动模式最大水平加速度
		float maxAutoAccXY[2];
		//自动模式最大水平加加速度
		float maxAutoJerkXY[2];
		
		//最大向上速度
		float maxVelUp[2];
		//最大向下速度
		float maxVelDown[2];
		//最大向上加速度
		float maxAccUp[2];
		//最大向下加速度
		float maxAccDown[2];
		//最大向上加加速度
		float maxJerkUp[2];
		//最大向下加加速度
		float maxJerkDown[2];
		//自动模式最大向上速度
		float maxAutoVelUp[2];
		//自动模式最大向下速度
		float maxAutoVelDown[2];
		//自动模式最大向上加速度
		float maxAutoAccUp[2];
		//自动模式最大向下加速度
		float maxAutoAccDown[2];
		//自动模式最大向上加加速度
		float maxAutoJerkUp[2];
		//自动模式最大向下加加速度
		float maxAutoJerkDown[2];
		
		//最大刹车加速度比例
		float maxBreakAccPc[2];
		//最大刹车加加速度
		float maxBreakJerk[2];

		//到达目标点范围
		float WPRange[2];
		//降落速度
		float LandVel[2];
		//协调转弯速度
		float SMVel[2];
	}__PACKED;
	static PosCtrlCfg cfg;
/*参数*/	

/*参数接口*/
	float get_maxVelUp()
	{
		return cfg.maxVelUp[0];
	}
	float get_maxVelDown()
	{
		return cfg.maxVelDown[0];
	}
	float get_maxVelXY()
	{
		return cfg.maxVelXY[0];
	}
	float get_maxAccXY()
	{
		return cfg.maxAccXY[0];
	}
	float get_LandVel()
	{
		return cfg.LandVel[0];
	}
	float get_WPRange()
	{
		return cfg.WPRange[0];
	}
	float get_P1()
	{
		return cfg.P1[0];
	}
	float get_P2()
	{
		return cfg.P2[0];
	}
	float get_VelXYFF()
	{
		return cfg.VelXYFF[0];
	}
/*参数接口*/
	
/*控制接口*/
	static bool Altitude_Control_Enabled = false;
	static bool Position_Control_Enabled = false;
	
	//位置控制模式
	static Position_ControlMode Altitude_ControlMode = Position_ControlMode_Position;
	static Position_ControlMode HorizontalPosition_ControlMode = Position_ControlMode_Position;
	
	//期望TD4滤波器
	static TD4_SL Target_tracker[3];
	static TD3_2DSL Target_tracker_xy;
	//期望速度低通滤波器
	static Filter_Butter4_LP TargetVelocityFilter[3];
	//垂直加速度误差TD4滤波器
	static TD3_Lite AccZ_Err_filter;
	
	/*目标模式
		OffBoard中：
			3-位置+速度+加速度控制
			2-速度+加速度控制
	*/
	static uint8_t OffBoardXY_target_mode = 0;
	static uint8_t OffBoardZ_target_mode = 0;
	static vector3<double> target_position;
	static vector3<double> target_velocity;
	static vector3<double> target_acc;
	static vector3<double> target_position_SM;
	static double target_SM_vel = 0;
	static double VelCtrlMaxRoll = -1 , VelCtrlMaxPitch = -1;
	static double VelCtrlMaxAcc = -1;
	
	//获取模式及期望位置速度
	bool get_TargetPosInf( Position_ControlMode* pos_mode, Position_ControlMode* alt_mode, 
													vector3<double>* t_pos, vector3<double>* t_vel,
													double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if(pos_mode)
				*pos_mode = HorizontalPosition_ControlMode;
			if(alt_mode)
				*alt_mode = Altitude_ControlMode;
			if(t_pos)
				*t_pos = target_position;
			if(t_vel)
				*t_vel = target_velocity;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	/*高度*/
		bool is_Altitude_Control_Enabled( bool* ena, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				*ena = Altitude_Control_Enabled;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Altitude_Control_Enable( double TIMEOUT )
		{
			if( get_Altitude_MSStatus() != MS_Ready )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == true )
				{	//控制器已打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				Attitude_Control_Enable();
				Altitude_ControlMode = Position_ControlMode_Locking;
				Altitude_Control_Enabled = true;
				
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				//读参数
				if( ReadParamGroup( "PosCtrl", (uint64_t*)&cfg, 0, TIMEOUT ) != PR_OK )
				{
					UnlockCtrl();
					return false;
				}				
				Position_Control_set_ZAutoSpeed( cfg.AutoVZUp[0], cfg.AutoVZDown[0] );
				Target_tracker[2].P1 = cfg.Z_TD4P1[0];
				Target_tracker[2].P2 = cfg.Z_TD4P2[0];
				Target_tracker[2].P3 = cfg.Z_TD4P3[0];
				Target_tracker[2].P4 = cfg.Z_TD4P4[0];
				
				Position_Control_reset_ZAutoSpeed();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Altitude_Control_Disable( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( ForceMSafeCtrl && !isMSafe )
				{	//屏蔽用户控制
					UnlockCtrl();
					return false;
				}
				Position_Control_Disable();
				Altitude_ControlMode = Position_ControlMode_Null;
				Altitude_Control_Enabled = false;					
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//获取飞行模式
		bool get_Altitude_ControlMode( Position_ControlMode* mode, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && (Is_2DAutoMode(Altitude_ControlMode) || Is_3DAutoMode(Altitude_ControlMode)) )
				{	//用户控制访问且为自动模式时更新控制时间
					last_ZCtrlTime = TIME::now();
				}
				*mode = Altitude_ControlMode;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//设定Z自动飞行速度
		static double AutoVelZUp = 200;
		static double AutoVelZDown = 200;
		bool Position_Control_get_ZAutoSpeed( double* SpUp, double* SpDown, double TIMEOUT )
		{		
			if( LockCtrl(TIMEOUT) )
			{
				*SpUp = AutoVelZUp;
				*SpDown = AutoVelZDown;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_reset_ZAutoSpeed( double TIMEOUT )
		{
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				AutoVelZUp = cfg.AutoVZUp[0];
				AutoVelZDown = cfg.AutoVZDown[0];
				
				//飞行速度限幅
				if( AutoVelZUp > cfg.maxAutoVelUp[0] )
					AutoVelZUp = cfg.maxAutoVelUp[0];

				if( AutoVelZDown > cfg.maxAutoVelDown[0] )
					AutoVelZDown = cfg.maxAutoVelDown[0];
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_ZAutoSpeed( double SpUp, double SpDown, double TIMEOUT )
		{
			if( !isvalid(SpUp) || !isvalid(SpDown) )
				return false;
			
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				if( SpUp > 0 )
					AutoVelZUp = SpUp;
				if( SpDown > 0 )
					AutoVelZDown = SpDown;
				
				//飞行速度限幅
				if( AutoVelZUp > cfg.maxAutoVelUp[0] )
					AutoVelZUp = cfg.maxAutoVelUp[0];

				if( AutoVelZDown > cfg.maxAutoVelDown[0] )
					AutoVelZDown = cfg.maxAutoVelDown[0];
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//移动Z目标位置
		bool Position_Control_move_TargetPositionZRelative( double posz, double TIMEOUT )
		{
			if( !isvalid(posz) )
				return false;
			//必须为位置或者自动模式
			if( Altitude_ControlMode!=Position_ControlMode_Position &&
					Is_2DAutoMode(Altitude_ControlMode)==false &&
					Is_3DAutoMode(Altitude_ControlMode)==false )
				return false;
			
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe )
			{	//用户控制访问更新控制时间
				last_ZCtrlTime = TIME::now();
			}
			target_position.z += posz;
			
			return true;
		}
		
		bool Position_Control_set_TargetPositionZ( double posz, double vel, double TIMEOUT )
		{
			if( !isvalid(posz) || !isvalid(vel) )
				return false;
			
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				target_position.z = posz;
				if( Is_2DAutoMode(Altitude_ControlMode)==false && Altitude_ControlMode!=Position_ControlMode_Position )
					Target_tracker[2].x1 = pos.z;
				
				//设定飞行速度
				//飞行速度限幅
				if( target_position.z > pos.z )
				{
					if( vel > 10 )
						AutoVelZUp = vel;
					else if( vel < 0 )
						AutoVelZUp = cfg.AutoVZUp[0];
					
					if( AutoVelZUp > cfg.maxAutoVelUp[0] )
						AutoVelZUp = cfg.maxAutoVelUp[0];
				}
				else
				{
					if( vel > 10 )
						AutoVelZDown = vel;
					else if( vel < 0 )
						AutoVelZDown = cfg.AutoVZDown[0];
					
					if( AutoVelZDown > cfg.maxAutoVelDown[0] )
						AutoVelZDown = cfg.maxAutoVelDown[0];
				}
				
				//切换模式
				Altitude_ControlMode = Position_ControlMode_RouteLine;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetPositionZGlobal( double posz, double vel, double TIMEOUT )
		{
			if( !isvalid(posz) || !isvalid(vel) )
				return false;
			
			//获取最优全球定位传感器信息
			PosSensorHealthInf1 global_inf;
			if( get_OptimalGlobal_Z( &global_inf ) == false )
				return false;
			posz -= global_inf.HOffset;
			return Position_Control_set_TargetPositionZ( posz, vel, TIMEOUT );
		}
		bool Position_Control_set_TargetPositionZRelative( double posz, double vel, double TIMEOUT )
		{
			if( !isvalid(posz) || !isvalid(vel) )
				return false;
			vector3<double> pos;
			if( get_Position_Ctrl( &pos, TIMEOUT ) == false )
				return false;
			return Position_Control_set_TargetPositionZ( pos.z + posz, vel, TIMEOUT );
		}
		bool Position_Control_set_TargetPositionZRA( double posz, double vel, double TIMEOUT )
		{
			//获取起飞位置Z坐标
			double homeZ;
			getHomeLocalZ(&homeZ);
			return Position_Control_set_TargetPositionZ( homeZ + posz, vel, TIMEOUT );
		}
		
		bool Position_Control_set_TargetVelocityZ( double velz, double TIMEOUT )
		{
			if( !isvalid(velz) )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				target_velocity.z = velz;
				Altitude_ControlMode = Position_ControlMode_Velocity;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_ZLock( double TIMEOUT )
		{
			vector3<double> pos;
			if( get_Position_Ctrl( &pos, TIMEOUT ) == false )
				return false;
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( Altitude_ControlMode != Position_ControlMode_Position )
					Altitude_ControlMode = Position_ControlMode_Locking;
				Target_tracker[2].x1 = pos.z;
				
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		//起飞到当前高度上方的height高度
		static double TakeoffHeight;
		bool Position_Control_Takeoff_HeightRelative( double height, double TIMEOUT )
		{
			if( height < 10 )
					return false;
			bool inFlight;	get_is_inFlight(&inFlight);
			if( inFlight == true )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Altitude_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_ZCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				Altitude_ControlMode = Position_ControlMode_Takeoff;
				TakeoffHeight = height;
			
				//更新控制时间
				if(!isMSafe)
					last_ZCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Takeoff_HeightGlobal( double height, double TIMEOUT )
		{
			//获取最优全球定位传感器信息
			PosSensorHealthInf1 global_inf;
			if( get_OptimalGlobal_Z( &global_inf ) == false )
				return false;
			height -= global_inf.HOffset;
			height -= global_inf.PositionENU.z;
			return Position_Control_Takeoff_HeightRelative( height, TIMEOUT );
		}
		bool Position_Control_Takeoff_Height( double height, double TIMEOUT )
		{
			vector3<double> pos;
			get_Position_Ctrl(&pos);
			height = height - pos.z;
			return Position_Control_Takeoff_HeightRelative( height, TIMEOUT );
		}
	/*高度*/
		
	/*水平位置*/
		static double pos_vel = -1;
		static double XYLock_maxAcc = -1;
		static double XYLock_CAcc = 0;
		static vector2<double> Expected_VelXY;
		bool is_Position_Control_Enabled( bool* ena, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				*ena = Position_Control_Enabled;
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Enable( double TIMEOUT )
		{
			if( get_Position_MSStatus() != MS_Ready )
				return false;
			
			vector3<double> vel;
			get_VelocityENU_Ctrl(&vel);
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == true )
				{	//控制器已打开
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				Altitude_Control_Enable();
				if( Altitude_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				Expected_VelXY.x = vel.x;
				Expected_VelXY.y = vel.y;
				HorizontalPosition_ControlMode = Position_ControlMode_Locking;
				Position_Control_Enabled = true;
				
				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				//读参数	
				Target_tracker_xy.P1 = 10;
				Target_tracker_xy.P2 = 10;
				Target_tracker_xy.P3 = 20;
				Target_tracker_xy.r2 = cfg.maxVelXY[0];
				Target_tracker_xy.r3 = cfg.maxAccXY[0];
				Position_Control_reset_XYAutoSpeed();
				Position_Control_reset_XYZAutoSpeed();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_Disable( double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( ForceMSafeCtrl && !isMSafe )
				{	//屏蔽用户控制
					UnlockCtrl();
					return false;
				}
				HorizontalPosition_ControlMode = Position_ControlMode_Null;
				Position_Control_Enabled = false;		
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool get_Position_ControlMode( Position_ControlMode* mode, double TIMEOUT )
		{
			if( LockCtrl(TIMEOUT) )
			{
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && (Is_2DAutoMode(HorizontalPosition_ControlMode) || Is_3DAutoMode(HorizontalPosition_ControlMode)) )
				{	//用户控制访问且为自动模式时更新控制时间
					last_XYCtrlTime = TIME::now();
				}
				*mode = HorizontalPosition_ControlMode;
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool Position_Control_set_TargetVelocityXY_AngleLimit( double velx, double vely, double maxAngle, double TIMEOUT )
		{
			if( !isvalid(velx) || !isvalid(vely) || !isvalid(maxAngle) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				constrain_vector( velx, vely, (double)cfg.maxVelXY[0] );
				target_velocity.x = velx;
				target_velocity.y = vely;	
				HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
				VelCtrlMaxRoll = maxAngle;
				if( VelCtrlMaxRoll>=0 && VelCtrlMaxRoll<0.05 )
					VelCtrlMaxRoll = 0.05;
				VelCtrlMaxPitch = -1;

				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
			
				UnlockCtrl();
				return true;
			}
			return false;
		}
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( double velx, double vely, double maxRoll, double maxPitch, double TIMEOUT )
		{
			if( !isvalid(velx) || !isvalid(vely) || !isvalid(maxPitch) || !isvalid(maxRoll) )
				return false;
			
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{	//控制器未打开
					UnlockCtrl();
					return false;
				}	
				
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				
				constrain_vector( velx, vely, (double)cfg.maxVelXY[0] );
				
				double yaw;	double yaw_declination;
				get_YawDeclination(&yaw_declination);
				Attitude_Control_get_TargetTrackYaw(&yaw);
				yaw += yaw_declination;
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double velx_ENU = BodyHeading2ENU_x( velx , vely , sin_Yaw , cos_Yaw );
				double vely_ENU = BodyHeading2ENU_y( velx , vely , sin_Yaw , cos_Yaw );
				
				target_velocity.x = velx_ENU;
				target_velocity.y = vely_ENU;	
				HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
				VelCtrlMaxRoll = maxRoll;
				if( VelCtrlMaxRoll>=0 && VelCtrlMaxRoll<0.05 )
					VelCtrlMaxRoll = 0.05;
				VelCtrlMaxPitch = maxPitch;
				if( VelCtrlMaxPitch>=0 && VelCtrlMaxPitch < 0.05 )
					VelCtrlMaxPitch = 0.05;		

				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
			
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		bool Position_Control_set_XYLock( double maxAcc, double TIMEOUT )
		{
			if( !isvalid(maxAcc) )
				return false;
			
			vector3<double> vel;
			get_VelocityENU_Ctrl(&vel);
			if( LockCtrl(TIMEOUT) )
			{
				if( Position_Control_Enabled == false )
				{
					UnlockCtrl();
					return false;
				}
				bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
				if( !isMSafe && ForceMSafeCtrl )
				{	//屏蔽用户控制
					last_XYCtrlTime = TIME::now();
					UnlockCtrl();
					return false;
				}
				if( HorizontalPosition_ControlMode != Position_ControlMode_Position )
				{
					XYLock_maxAcc = maxAcc;
					if( HorizontalPosition_ControlMode != Position_ControlMode_Locking )
					{
						XYLock_CAcc = 0;
						Expected_VelXY.x = vel.x;
						Expected_VelXY.y = vel.y;
						HorizontalPosition_ControlMode = Position_ControlMode_Locking;
					}
				}
				
				//更新控制时间
				if(!isMSafe)
					last_XYCtrlTime = TIME::now();
				
				UnlockCtrl();
				return true;
			}
			return false;
		}
		
		/*OffBoard模式*/
			/*XY*/
				bool Position_Control_set_TargetPosVelAccXY_OffBoard( double posx, double posy, double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(posx) || !isvalid(posy) ||
							!isvalid(velx) || !isvalid(vely) ||
							!isvalid(accx) || !isvalid(accy) )
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_XYCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}
						
						//设定目标位置
						target_position.x = posx;
						target_position.y = posy;
						
						//设定目标速度
						target_velocity.x = velx;
						target_velocity.y = vely;
						
						//设定目标加速度
						target_acc.x = accx;
						target_acc.y = accy;
						
						HorizontalPosition_ControlMode = Position_ControlMode_OffBoard;
						OffBoardXY_target_mode = 3;
						
						//更新控制时间
						if(!isMSafe)
							last_XYCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
				bool Position_Control_set_TargetGlobalPosVelAccXY_OffBoard( double Lat, double Lon, double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(Lat) || !isvalid(Lon) ||
							!isvalid(velx) || !isvalid(vely) ||
							!isvalid(accx) || !isvalid(accy) )
						return false;
					
					//获取最优全球定位传感器信息
					PosSensorHealthInf3 global_inf;
					if( get_OptimalGlobal_XYZ( &global_inf ) == false )
						return false;
					//获取指定经纬度平面坐标
					double x, y;
					map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
					x -= global_inf.HOffset.x;
					y -= global_inf.HOffset.y;
					return Position_Control_set_TargetPosVelAccXY_OffBoard( x, y, velx, vely, accx, accy, TIMEOUT );
				}
				bool Position_Control_set_TargetPosRelativeVelAccXY_OffBoard( double posx, double posy, double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(posx) || !isvalid(posy) ||
							!isvalid(velx) || !isvalid(vely) ||
							!isvalid(accx) || !isvalid(accy) )
						return false;
					
					if( LockCtrl(TIMEOUT) )
					{
						vector3<double> position;
						if( HorizontalPosition_ControlMode==Position_ControlMode_Position || HorizontalPosition_ControlMode==Position_ControlMode_OffBoard )
							position = target_position;
						else
							get_Position_Ctrl(&position);
						bool res = Position_Control_set_TargetPosVelAccXY_OffBoard( position.x + posx, position.y + posy, velx, vely, accx, accy, TIMEOUT );
						
						UnlockCtrl();
						return res;
					}
					return false;
				}
				
				bool Position_Control_set_TargetVelAccXY_OffBoard( double velx, double vely, double accx, double accy, double TIMEOUT )
				{
					if( !isvalid(velx) || !isvalid(vely) ||
							!isvalid(accx) || !isvalid(accy) )
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_XYCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}

						//设定目标速度
						target_velocity.x = velx;
						target_velocity.y = vely;
						
						//设定目标加速度
						target_acc.x = accx;
						target_acc.y = accy;
						
						HorizontalPosition_ControlMode = Position_ControlMode_OffBoard;
						OffBoardXY_target_mode = 2;
						
						//更新控制时间
						if(!isMSafe)
							last_XYCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
			/*XY*/
				
			/*Z*/
				bool Position_Control_set_TargetPosVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(posz) ||
							!isvalid(velz) ||
							!isvalid(accz))
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_ZCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}
						
						//设定目标位置
						target_position.z = posz;
						
						//设定目标速度
						target_velocity.z = velz;
						
						//设定目标加速度
						target_acc.z = accz;
						
						Altitude_ControlMode = Position_ControlMode_OffBoard;
						OffBoardZ_target_mode = 3;
						
						//更新控制时间
						if(!isMSafe)
							last_ZCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
				bool Position_Control_set_TargetPosGlobalVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(posz) || !isvalid(velz) || !isvalid(accz) )
						return false;
					
					//获取最优全球定位传感器信息
					PosSensorHealthInf1 global_inf;
					if( get_OptimalGlobal_Z( &global_inf ) == false )
						return false;
					posz -= global_inf.HOffset;
					return Position_Control_set_TargetPosVelAccZ_OffBoard( posz, velz, accz, TIMEOUT );
				}
				bool Position_Control_set_TargetPosRelativeVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(posz) || !isvalid(velz) || !isvalid(accz) )
						return false;
					
					vector3<double> pos;
					if( get_Position_Ctrl( &pos, TIMEOUT ) == false )
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Altitude_ControlMode==Position_ControlMode_Position || Altitude_ControlMode==Position_ControlMode_OffBoard )
							pos.z = target_position.z;
						bool res = Position_Control_set_TargetPosVelAccZ_OffBoard( pos.z + posz, velz, accz, TIMEOUT );
						
						UnlockCtrl();
						return res;
					}
					return false;
				}
				bool Position_Control_set_TargetPosZRAVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT )
				{
					//获取起飞位置Z坐标
					double homeZ;
					getHomeLocalZ(&homeZ);
					return Position_Control_set_TargetPosVelAccZ_OffBoard( homeZ + posz, velz, accz, TIMEOUT );
				}
				
				bool Position_Control_set_TargetVelAccZ_OffBoard( double velz, double accz, double TIMEOUT )
				{
					if( !isvalid(velz) ||
							!isvalid(accz))
						return false;
					if( LockCtrl(TIMEOUT) )
					{
						if( Position_Control_Enabled == false )
						{
							UnlockCtrl();
							return false;
						}
						bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
						if( !isMSafe && ForceMSafeCtrl )
						{	//屏蔽用户控制
							last_ZCtrlTime = TIME::now();
							UnlockCtrl();
							return false;
						}
						
						//设定目标速度
						target_velocity.z = velz;
						
						//设定目标加速度
						target_acc.z = accz;
						
						Altitude_ControlMode = Position_ControlMode_OffBoard;
						OffBoardZ_target_mode = 2;
						
						//更新控制时间
						if(!isMSafe)
							last_ZCtrlTime = TIME::now();
						
						UnlockCtrl();
						return true;
					}
					return false;
				}
			/*Z*/
		/*OffBoard模式*/
		
		/*手动绕圈模式*/
			static double ManualCircleVel = 0;
			static double ManualCircleR = 500;
			static vector2<double> ManualCircleOrigin;
			bool Position_Control_do_ManualCircleRelative( double dCircleVel, double dCircleR, double dCircleO, double TIMEOUT )
			{
				if( !isvalid(dCircleVel) || !isvalid(dCircleR) || !isvalid(dCircleO) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{	//控制器未打开
						UnlockCtrl();
						return false;
					}	
					
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					if( HorizontalPosition_ControlMode!=Position_ControlMode_Position && 
							HorizontalPosition_ControlMode!=Position_ControlMode_ManualCircle )				
					{	//未锁定位置先锁定
						UnlockCtrl();
						Position_Control_set_XYLock();
						return false;
					}		
					
					Quaternion attitude;
					if( get_AirframeY_quat( &attitude, TIMEOUT ) == false )
					{
						UnlockCtrl();
						return false;
					}
					
					vector3<double> position;
					if( get_Position_Ctrl( &position, TIMEOUT ) == false )
					{
						UnlockCtrl();
						return false;
					}
					
					if( HorizontalPosition_ControlMode!=Position_ControlMode_ManualCircle )
					{	//初始进入模式
						ManualCircleVel = 0;
						ManualCircleR = 500;
						
						double yaw = attitude.getYaw();
						double sin_Yaw, cos_Yaw;
						fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
						ManualCircleOrigin.x = position.x + BodyHeading2ENU_x( ManualCircleR , 0 , sin_Yaw , cos_Yaw );
						ManualCircleOrigin.y = position.y + BodyHeading2ENU_y( ManualCircleR , 0 , sin_Yaw , cos_Yaw );
						
						HorizontalPosition_ControlMode = Position_ControlMode_ManualCircle;
					}									
					
					if( !is_zero(dCircleO) && ManualCircleR+dCircleO>=20 )
					{	//移动圆心
						vector2<double> RVec = vector2<double>(position.x,position.y) - ManualCircleOrigin;
						double RVec_length = safe_sqrt(RVec.get_square());					
						vector2<double> RVec_normed;
						if( RVec_length > 0.1 )
							RVec_normed = RVec * (1.0/RVec_length);
						ManualCircleOrigin -= RVec_normed*dCircleO;
						ManualCircleR += dCircleO;
					}
					//限制最大速度
					ManualCircleVel += dCircleVel;
					ManualCircleR += dCircleR;
					if( ManualCircleR < 20 )
						ManualCircleR = 20;
					double max_vel = safe_sqrt(0.7*cfg.maxAccXY[0]*ManualCircleR);
					if( max_vel > cfg.maxVelXY[0] )
						max_vel = cfg.maxVelXY[0];
					if( fabs(ManualCircleVel) > max_vel )
						ManualCircleVel = sign(ManualCircleVel)*max_vel;

					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
				
					UnlockCtrl();
					return true;
				}
				return false;
			}
		/*手动绕圈模式*/
		
		
		static double AutoVelXY = 500;
		static double AutoVelXYZ = 500;
		/*直线飞行*/
			//直线方程系数
			//A=(x1,y1,z1)=target_position目标点
			//B=(x2,y2,z2)=起点
			static vector3<double> route_line_A_B;	//(B-A)
			static double route_line_m;	// 1/(B-A)^2
			static double line_track_desired = 0;
			static double line_track_desired_vel = 0;
			static double line_track_desired_maxv = 500;
			static double line_track_desired_maxacc = 300;
			static uint16_t line_track_delay_counter = 0;
			#define reset_line_track_state(maxv,maxacc,desired_vel) { line_track_delay_counter = 0;\
																																line_track_desired = 0;\
																																line_track_desired_vel = desired_vel;\
																																line_track_desired_maxv = maxv;\
																																line_track_desired_maxacc = maxacc; }
			
			//直线飞行障碍限制
			static double max_line_track_desired = -1;
			static double max_line_track_desired_vel = -1;
			bool Position_Control_set_RouteLineAvoidanceRelative( double pos, double vel, double TIMEOUT )
			{
				if( !isvalid(pos) || !isvalid(vel) )
					return false;
				if( HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine3D && HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine )
					return false;

				vector3<double> position;
				get_Position_Ctrl(&position);
				if( LockCtrl(TIMEOUT) )
				{
					if( pos < 0 )
						max_line_track_desired = -1;
					else {
						
						//计算垂足
						vector3<double> A_C = position - target_position;
						double k = (A_C * route_line_A_B) * route_line_m;
						vector3<double> foot_point = (route_line_A_B * k) + target_position;
						
						//计算偏差
						vector3<double> B = target_position + route_line_A_B;
						vector3<double> dis = foot_point - B;
						double distance = safe_sqrt(dis.get_square());
						
						//移动目标点
						max_line_track_desired = distance + pos;
					}
					
					if( vel < 0 )
						max_line_track_desired_vel = -1;
					else {
						max_line_track_desired_vel = vel;
					}
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
		
			bool Position_Control_set_TargetPositionXY( double posx, double posy, double vel, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(vel) )
					return false;
				vector3<double> cVel;
				get_VelocityENU_Ctrl(&cVel);
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					vector3<double> position;
					if( HorizontalPosition_ControlMode==Position_ControlMode_Position )
						position = target_position;
					else
						get_Position_Ctrl(&position);
					target_position.x = posx;
					target_position.y = posy;
					
					//calculate vector B-A
					route_line_A_B = position - target_position;
					route_line_A_B.z = 0;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//设置速度
						if( vel > 10 )
							AutoVelXY = vel;
						else if( vel < 0 )
							AutoVelXY = cfg.AutoVXY[0];
						
						//速度限幅
						if( AutoVelXY > cfg.maxAutoVelXY[0] )
							AutoVelXY = cfg.maxAutoVelXY[0];
						
						double vel_desired = safe_sqrt( cVel.get_square() ) * 0.5;
						reset_line_track_state(AutoVelXY,cfg.maxAutoAccXY[0],vel_desired)
						//切换模式
						HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
					}
					else
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYZ( double posx, double posy, double posz, double vel, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(posz) || !isvalid(vel) )
					return false;
				vector3<double> cVel;
				get_VelocityENU_Ctrl(&cVel);
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					vector3<double> position;
					if( HorizontalPosition_ControlMode==Position_ControlMode_Position && Altitude_ControlMode==Position_ControlMode_Position )
						position = target_position;
					else
						get_Position_Ctrl(&position);
					target_position.x = posx;
					target_position.y = posy;
					target_position.z = posz;
					
					//calculate vector B-A
					route_line_A_B = position - target_position;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//设置速度
						if( vel > 10 )
							AutoVelXYZ = vel;
						else if( vel < 0 )
							AutoVelXYZ = cfg.AutoVXYZ[0];
						
						/*速度限幅*/
							double xyz_length = safe_sqrt( route_line_A_B.get_square() );
							if( xyz_length > 1 )
							{
								double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
								
								double inv_xyz = 1.0 / xyz_length;
								double xy_scale = xy_length * inv_xyz;
								double z_scale = fabs(route_line_A_B.z) * inv_xyz;
								
								double scale = 1.0;
								if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
									scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
								if( route_line_A_B.z < 0 )
								{	//向上飞
									if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
									{
										double new_scale = cfg.maxAutoVelUp[0] / (AutoVelXYZ*z_scale);
										if( new_scale < scale )
											scale = new_scale;
									}
								}
								else
								{	//向下飞
									if( AutoVelXYZ*z_scale > cfg.maxAutoVelDown[0] )
									{
										double new_scale = cfg.maxAutoVelDown[0] / (AutoVelXYZ*z_scale);
										if( new_scale < scale )
											scale = new_scale;
									}
								}
								
								AutoVelXYZ *= scale;
							}
							else
							{
								if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
									AutoVelXYZ = cfg.maxAutoVelXY[0];
							}
						/*速度限幅*/
						
						double vel_desired = safe_sqrt( cVel.get_square() ) * 0.5;
						reset_line_track_state(AutoVelXYZ,cfg.maxAutoAccXY[0],vel_desired)
						//切换模式
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
					}
					else
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_set_TargetPositionXYRelative( double posx, double posy, double vel, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(vel) )
					return false;
				
				vector3<double> cVel;
				get_VelocityENU_Ctrl(&cVel);
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					vector3<double> position;
					if( HorizontalPosition_ControlMode==Position_ControlMode_Position )
						position = target_position;
					else
						get_Position_Ctrl(&position);
					target_position.x = position.x + posx;
					target_position.y = position.y + posy;
					
					//calculate vector B-A
					route_line_A_B = position - target_position;
					route_line_A_B.z = 0;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//设置速度
						if( vel > 10 )
							AutoVelXY = vel;
						else if( vel < 0 )
							AutoVelXY = cfg.AutoVXY[0];
						
						//速度限幅
						if( AutoVelXY > cfg.maxAutoVelXY[0] )
							AutoVelXY = cfg.maxAutoVelXY[0];
						
						double vel_desired = safe_sqrt( cVel.get_square() ) * 0.5;
						reset_line_track_state(AutoVelXY,cfg.maxAutoAccXY[0],vel_desired)
						//切换模式
						HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
					}
					else
						HorizontalPosition_ControlMode = Position_ControlMode_Position;
				
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_TargetPositionXYZRelative( double posx, double posy, double posz, double vel, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				vector3<double> cVel;
				get_VelocityENU_Ctrl(&cVel);
				if( LockCtrl(TIMEOUT) )
				{
					if( Position_Control_Enabled == false )
					{
						UnlockCtrl();
						return false;
					}
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}
					
					vector3<double> position;
					if( HorizontalPosition_ControlMode==Position_ControlMode_Position && Altitude_ControlMode==Position_ControlMode_Position )
						position = target_position;
					else
						get_Position_Ctrl(&position);				
					target_position.x = position.x + posx;
					target_position.y = position.y + posy;
					target_position.z = position.z + posz;
					
					//calculate vector B-A
					route_line_A_B = position - target_position;
					double route_line_A_B_sq = route_line_A_B.get_square();
					if( route_line_A_B_sq > 0.1 )
					{
						//calculate 1/(B-A)^2
						route_line_m = 1.0 / route_line_A_B_sq;
						
						//设置速度
						if( vel > 10 )
							AutoVelXYZ = vel;
						else if( vel < 0 )
							AutoVelXYZ = cfg.AutoVXYZ[0];
						
						/*速度限幅*/
							double xyz_length = safe_sqrt( route_line_A_B.get_square() );
							if( xyz_length > 1 )
							{
								double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
								
								double inv_xyz = 1.0 / xyz_length;
								double xy_scale = xy_length * inv_xyz;
								double z_scale = fabs(route_line_A_B.z) * inv_xyz;
								
								double scale = 1.0;
								if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
									scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
								if( route_line_A_B.z < 0 )
								{	//向上飞
									if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
									{
										double new_scale = cfg.maxAutoVelUp[0] / (AutoVelXYZ*z_scale);
										if( new_scale < scale )
											scale = new_scale;
									}
								}
								else
								{	//向下飞
									if( AutoVelXYZ*z_scale > cfg.maxAutoVelDown[0] )
									{
										double new_scale = cfg.maxAutoVelDown[0] / (AutoVelXYZ*z_scale);
										if( new_scale < scale )
											scale = new_scale;
									}
								}
								
								AutoVelXYZ *= scale;
							}
							else
							{
								if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
									AutoVelXYZ = cfg.maxAutoVelXY[0];
							}
						/*速度限幅*/
						
						//切换模式
						double vel_desired = safe_sqrt( cVel.get_square() ) * 0.5;
						reset_line_track_state(AutoVelXYZ,cfg.maxAutoAccXY[0],vel_desired)
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_RouteLine3D;
					}
					else
						HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
				
					//更新控制时间
					if(!isMSafe)
						last_XYCtrlTime = last_ZCtrlTime = TIME::now();
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			//移动XY目标位置
			bool Position_Control_move_TargetPositionXYRelative( double posx, double posy, double TIMEOUT )
			{
				if( !isvalid(posx) || !isvalid(posy) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{				
					//必须为位置或者自动模式
					if( HorizontalPosition_ControlMode!=Position_ControlMode_Position &&
							Is_2DAutoMode(HorizontalPosition_ControlMode)==false &&
							Is_3DAutoMode(HorizontalPosition_ControlMode)==false )
					{
						UnlockCtrl();
						return false;
					}
					
					bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
					if( !isMSafe && ForceMSafeCtrl )
					{	//屏蔽用户控制
						last_XYCtrlTime = TIME::now();
						UnlockCtrl();
						return false;
					}	
					
					target_position.x += posx;
					target_position.y += posy;
					
					if( !isMSafe )
					{	//用户控制访问更新控制时间
						last_XYCtrlTime = TIME::now();
					}
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_move_TargetPositionXYRelativeBodyheading( double posx, double posy, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_move_TargetPositionXYRelative( posx_enu, posy_enu, TIMEOUT );
			}
			
			bool Position_Control_set_TargetPositionXYRelativeBodyheading( double posx, double posy, double vel, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_set_TargetPositionXYRelative( posx_enu, posy_enu, vel, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRelativeBodyheading( double posx, double posy, double posz, double vel, double TIMEOUT )
			{
				Quaternion att;
				get_Attitude_quat( &att );
				double yaw = att.getYaw();
				double sin_Yaw, cos_Yaw;
				fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
				double posx_enu = BodyHeading2ENU_x( posx , posy , sin_Yaw , cos_Yaw );
				double posy_enu = BodyHeading2ENU_y( posx , posy , sin_Yaw , cos_Yaw );
				return Position_Control_set_TargetPositionXYZRelative( posx_enu, posy_enu, posz, vel, TIMEOUT );
			}
			
			bool Position_Control_set_TargetPositionXY_LatLon( double Lat, double Lon, double vel, double TIMEOUT )
			{
				if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(vel) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				return Position_Control_set_TargetPositionXY( x, y, vel, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZ_LatLon( double Lat, double Lon, double posz, double vel, double TIMEOUT )
			{
				if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf3 global_inf;
				if( get_OptimalGlobal_XYZ( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				posz -= global_inf.HOffset.z;
				return Position_Control_set_TargetPositionXYZ( x, y, posz, vel, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRA_LatLon( double Lat, double Lon, double posz, double vel, double TIMEOUT )
			{
				if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				//获取起飞位置Z坐标
				double homeZ;
				getHomeLocalZ(&homeZ);
				return Position_Control_set_TargetPositionXYZ( x, y, homeZ + posz, vel, TIMEOUT );
			}
			bool Position_Control_set_TargetPositionXYZRelative_LatLon( double Lat, double Lon, double posz, double vel, double TIMEOUT )
			{
				if( !isvalid(Lat) || !isvalid(Lon) || !isvalid(posz) || !isvalid(vel) )
					return false;
				
				//获取最优全球定位传感器信息
				PosSensorHealthInf2 global_inf;
				if( get_OptimalGlobal_XY( &global_inf ) == false )
					return false;
				//获取指定经纬度平面坐标
				double x, y, z;
				map_projection_project( &global_inf.mp, Lat, Lon, &x, &y );
				x -= global_inf.HOffset.x;
				y -= global_inf.HOffset.y;
				vector3<double> position;
				if( HorizontalPosition_ControlMode==Position_ControlMode_Position )
					position = target_position;
				else
					get_Position_Ctrl(&position);
				z = position.z + posz;
				return Position_Control_set_TargetPositionXYZ( x, y, z, vel, TIMEOUT );
			}
			
			bool Position_Control_get_LineFlightDistance( double* distance, double TIMEOUT )
			{
				if( distance == 0 )
					return false;
				if( HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine3D && HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine )
					return false;
				
				vector3<double> position;
				get_Position_Ctrl(&position);
				if( LockCtrl(TIMEOUT) )
				{
					//计算偏差
					vector3<double> B = target_position + route_line_A_B;
					vector3<double> B_C = position - B;
					if( route_line_A_B.get_square() > 0.1 )
					{
						vector3<double> route_n = route_line_A_B * (-1.0/safe_sqrt(route_line_A_B.get_square()));
						*distance = route_n * B_C;
					}
					else
						*distance = safe_sqrt( B_C.get_square() );
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_get_LineFlightABDistance( vector3<double>* AB, double* distance, double TIMEOUT )
			{
				if( AB==0 && distance==0 )
					return false;
				if( HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine3D && HorizontalPosition_ControlMode!=Position_ControlMode_RouteLine )
					return false;
				
				vector3<double> position;
				get_Position_Ctrl(&position);
				if( LockCtrl(TIMEOUT) )
				{
					if( AB != 0 )
						*AB = route_line_A_B;
					if( distance != 0 )
					{
						//计算垂足
						vector3<double> A_C = position - target_position;
						double k = (A_C * route_line_A_B) * route_line_m;
						vector3<double> foot_point = (route_line_A_B * k) + target_position;
						
						//计算偏差
						vector3<double> B = target_position + route_line_A_B;
						vector3<double> dis = foot_point - B;
						*distance = safe_sqrt(dis.get_square());
					}
					UnlockCtrl();
					return true;
				}
				return false;
			}
		/*直线飞行*/
			
		/*设定自动飞行速度*/
			bool Position_Control_get_XYAutoSpeed( double* AtVelXY, double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					*AtVelXY = AutoVelXY;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_reset_XYAutoSpeed( double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXY = cfg.AutoVXY[0];
					if( AutoVelXY > cfg.maxAutoVelXY[0] )
						AutoVelXY = cfg.maxAutoVelXY[0];
					line_track_desired_maxv = AutoVelXY;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_XYAutoSpeed( double AtVelXY, double TIMEOUT )
			{
				if( !isvalid(AtVelXY) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXY = AtVelXY;
					if( AutoVelXY > cfg.maxAutoVelXY[0] )
						AutoVelXY = cfg.maxAutoVelXY[0];
					line_track_desired_maxv = AutoVelXY;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			
			bool Position_Control_get_XYZAutoSpeed( double* AtVelXYZ, double TIMEOUT )
			{
				if( LockCtrl(TIMEOUT) )
				{
					*AtVelXYZ = AutoVelXYZ;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_reset_XYZAutoSpeed( double TIMEOUT )
			{		
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXYZ = cfg.AutoVXYZ[0];
					
					//速度限幅
					double xyz_length = safe_sqrt( route_line_A_B.get_square() );
					if( xyz_length > 1 )
					{
						double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
						
						double inv_xyz = 1.0 / xyz_length;
						double xy_scale = xy_length * inv_xyz;
						double z_scale = fabs(route_line_A_B.z) * inv_xyz;
						
						double scale = 1.0;
						if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
							scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
						if( route_line_A_B.z < 0 )
						{	//向上飞
							if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
							{
								double new_scale = cfg.maxAutoVelUp[0] / (AutoVelXYZ*z_scale);
								if( new_scale < scale )
									scale = new_scale;
							}
						}
						else
						{	//向下飞
							if( AutoVelXYZ*z_scale > cfg.maxAutoVelDown[0] )
							{
								double new_scale = cfg.maxAutoVelDown[0] / (AutoVelXYZ*z_scale);
								if( new_scale < scale )
									scale = new_scale;
							}
						}
						
						AutoVelXYZ *= scale;
					}
					else
					{
						if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
							AutoVelXYZ = cfg.maxAutoVelXY[0];
					}
					line_track_desired_maxv = AutoVelXYZ;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
			bool Position_Control_set_XYZAutoSpeed( double AtVelXYZ, double TIMEOUT )
			{
				if( !isvalid(AtVelXYZ) )
					return false;
				
				if( LockCtrl(TIMEOUT) )
				{
					AutoVelXYZ = AtVelXYZ;
					
					//速度限幅
					double xyz_length = safe_sqrt( route_line_A_B.get_square() );
					if( xyz_length > 1 )
					{
						double xy_length = safe_sqrt( route_line_A_B.x*route_line_A_B.x + route_line_A_B.y*route_line_A_B.y );
						
						double inv_xyz = 1.0 / xyz_length;
						double xy_scale = xy_length * inv_xyz;
						double z_scale = fabs(route_line_A_B.z) * inv_xyz;
						
						double scale = 1.0;
						if( AutoVelXYZ*xy_scale > cfg.maxAutoVelXY[0] )
							scale = cfg.maxAutoVelXY[0] / (AutoVelXYZ*xy_scale);
						if( route_line_A_B.z < 0 )
						{	//向上飞
							if( AutoVelXYZ*z_scale > cfg.maxAutoVelUp[0] )
							{
								double new_scale = cfg.maxAutoVelUp[0] / (AutoVelXYZ*z_scale);
								if( new_scale < scale )
									scale = new_scale;
							}
						}
						else
						{	//向下飞
							if( AutoVelXYZ*z_scale > cfg.maxAutoVelDown[0] )
							{
								double new_scale = cfg.maxAutoVelDown[0] / (AutoVelXYZ*z_scale);
								if( new_scale < scale )
									scale = new_scale;
							}
						}
						
						AutoVelXYZ *= scale;
					}
					else
					{
						if( AutoVelXYZ > cfg.maxAutoVelXY[0] )
							AutoVelXYZ = cfg.maxAutoVelXY[0];
					}
					line_track_desired_maxv = AutoVelXYZ;
					
					UnlockCtrl();
					return true;
				}
				return false;
			}
		/*设定自动飞行速度*/
	/*水平位置*/
/*控制接口*/
		
/*滤波器*/
	static Filter_Butter4_LP ThrOut_Filters[3];
/*滤波器*/
			
void ctrl_Position()
{	
	bool Attitude_Control_Enabled;	is_Attitude_Control_Enabled(&Attitude_Control_Enabled);
	if( Attitude_Control_Enabled == false )
	{
		Altitude_Control_Enabled = false;
		Position_Control_Enabled = false;
		return;
	}
	
	double h = 1.0/CtrlRateHz;
	
	double e_1_n;
	double e_1;
	double e_2_n;
	double e_2;
	
	bool inFlight;	get_is_inFlight(&inFlight);
	vector3<double> Position;	get_Position_Ctrl(&Position);
	vector3<double> VelocityENU;	get_VelocityENU_Ctrl(&VelocityENU);
	vector3<double> AccelerationENU;	get_AccelerationENU_Ctrl(&AccelerationENU);
	double AccelerationENU_ESO_Z;
	get_es_AccZ(&AccelerationENU_ESO_Z);
	
	//位置速度滤波
	double Ps = cfg.P1[0];
	double Pv = cfg.P2[0];
	double Pa = cfg.P3[0];
	#define T21P 0.5
	
	static vector3<double> TAcc;
	vector3<double> TargetVelocity;
	vector3<double> TargetVelocity_1;
	vector3<double> TargetVelocity_2;
	
	//XY或Z其中一个为非3D模式则退出3D模式
	if( Is_3DAutoMode(HorizontalPosition_ControlMode) && Is_3DAutoMode(Altitude_ControlMode)==false )
		HorizontalPosition_ControlMode = Position_ControlMode_Locking;
	else if( Is_3DAutoMode(HorizontalPosition_ControlMode)==false && Is_3DAutoMode(Altitude_ControlMode) )
		Altitude_ControlMode = Position_ControlMode_Locking;
	
	//计算移动补偿后的target_position
	double SMlength = safe_sqrt( target_position_SM.get_square() );
	if( 0.5*target_SM_vel*target_SM_vel/line_track_desired_maxacc >= SMlength )
	{	//需要减速
		target_SM_vel -= line_track_desired_maxacc*h;
		if( target_SM_vel < 0 )
			target_SM_vel = 0;
	}
	else
	{	//前馈加速到最高速度
		if( target_SM_vel < cfg.SMVel[0] )
			target_SM_vel += line_track_desired_maxacc*h;
		if( target_SM_vel > cfg.SMVel[0] )
			target_SM_vel = cfg.SMVel[0];
	}
	double dSM_vel = target_SM_vel*h;
	if( SMlength > dSM_vel )
		target_position_SM -= target_position_SM * ( dSM_vel/SMlength);
	else
	{
		target_position_SM.zero();
		target_SM_vel = 0;
	}
	vector3<double> target_position_ds = target_position + target_position_SM;
	
	if( Position_Control_Enabled )
	{	//水平位置控制
		if( get_Position_MSStatus() != MS_Ready )
		{
			Position_Control_Enabled = false;
			goto PosCtrl_Finish;
		}
		
		switch( HorizontalPosition_ControlMode )
		{
			case Position_ControlMode_ManualCircle:
			{	//手动绕圈模式
				if( inFlight )
				{
					//计算半径方向偏差
					vector2<double> RVec = vector2<double>(Position.x,Position.y) - ManualCircleOrigin;
					double RVec_length = safe_sqrt(RVec.get_square());					
					vector2<double> RVec_normed;
					if( RVec_length > 0.1 )
						RVec_normed = RVec * (1.0/RVec_length);
					double RErrVec_lengthErr = ManualCircleR - RVec_length;
					vector2<double> RErrVec = RVec_normed * RErrVec_lengthErr;
					
					//计算e1导数
					vector2<double> e1_1( VelocityENU.x, VelocityENU.y );
					double e1r_1 = - (e1_1 * RVec_normed);
					vector2<double> e1_2( TAcc.x, TAcc.y );
					double e1r_2 = - (e1_2 * RVec_normed);
					vector2<double> circleVel = e1_1 + RVec_normed*e1r_1;
					/*半径方向*/
						vector3<double> esAngularRate;
						get_EsAngularRate(&esAngularRate);
						smooth_kp_d2 d1r = smooth_kp_2( RErrVec_lengthErr, e1r_1, e1r_2 , Ps, AutoVelXY+100 );
						vector2<double> T2r = RVec_normed*d1r.d0;
						double CircleAcc = 0;
						if( RVec_length > 0.1 )
							CircleAcc = circleVel.get_square() * (1.0/RVec_length);
						vector2<double> T2r_1 = RVec_normed*d1r.d1 - RVec_normed*CircleAcc;
						vector2<double> T2r_2 = RVec_normed*d1r.d2;
					/*半径方向*/
						
					//计算绕圆周速度
					vector2<double> TargetCircleVel(-RVec_normed.y,RVec_normed.x);
					TargetCircleVel *= ManualCircleVel;
						
					//控制偏航
					Quaternion quat;
					get_AirframeY_quat(&quat);
					double currentYaw = quat.getYaw();
					double yaw_err = atan2(-RVec.y,-RVec.x) - currentYaw;
					yaw_err = Mod( yaw_err, 2*Pi );
					if(yaw_err > Pi)
						yaw_err -= 2*Pi;
					while(yaw_err < -Pi)
						yaw_err += 2*Pi;
					double target_yaw_rate = 0;
					if( RVec_length > 0.1 )
						target_yaw_rate = (RVec_normed.x*VelocityENU.y - RVec_normed.y*VelocityENU.x) * (1.0/RVec_length);
					Attitude_Control_set_Target_YawRate( yaw_err*1.0 + target_yaw_rate );
						
					TargetVelocity.x = T2r.x+TargetCircleVel.x;	TargetVelocity.y = T2r.y+TargetCircleVel.y;
					TargetVelocity_1.x = T2r_1.x;	TargetVelocity_1.y = T2r_1.y;
					TargetVelocity_2.x = T2r_2.x;	TargetVelocity_2.y = T2r_2.y;
				}
				else
				{	//没起飞前在绕圈模式
					//回到position模式
					HorizontalPosition_ControlMode = Position_ControlMode_Position;
				}
				break;
			}
			
			case Position_ControlMode_Position:
			{	//悬停模式
				if( inFlight )
				{
					vector2<double> e1;
					e1.x = target_position_ds.x - Position.x;
					e1.y = target_position_ds.y - Position.y;
					vector2<double> e1_1;
					e1_1.x = - VelocityENU.x;
					e1_1.y = - VelocityENU.y;
					vector2<double> e1_2;
					e1_2.x = - TAcc.x;
					e1_2.y = - TAcc.y;
					double e1_length = safe_sqrt(e1.get_square());
					e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
					if( !is_zero(e1_length) )
						e_1 = e_1_n / e1_length;
					else
						e_1 = 0;
					e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
					if( !is_zero(e1_length*e1_length) )
						e_2 = e_2_n / (e1_length*e1_length);
					else
						e_2 = 0;
					
					double _pos_vel = 200;
					if( e1_length < 100 )
						pos_vel = -1;
					if( pos_vel > 0 )
						_pos_vel = pos_vel;					
					smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, _pos_vel );
					vector2<double> T2;
					vector2<double> T2_1;
					vector2<double> T2_2;
					if( !is_zero(e1_length*e1_length*e1_length) )
					{
						vector2<double> n = e1 * (1.0/e1_length);
						vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
						vector2<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
						T2 = n*d1.d0;
						T2_1 = n*d1.d1 + n_1*d1.d0;
						T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
					}
					TargetVelocity.x = T2.x;	TargetVelocity.y = T2.y;
					TargetVelocity_1.x = T21P*T2_1.x;	TargetVelocity_1.y = T21P*T2_1.y;
					TargetVelocity_2.x = T2_2.x;	TargetVelocity_2.y = T2_2.y;
				}
				else
				{	//没起飞前在位置控制模式
					//重置期望位置
					target_position.x = Position.x;
					target_position.y = Position.y;
					target_position_SM.zero();
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}		
			case Position_ControlMode_Velocity:
			{	//速度控制模式
				if( !inFlight )
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				else
				{
					TargetVelocity.x = target_velocity.x;
					TargetVelocity.y = target_velocity.y;
					Pv = cfg.P2_VelXY[0];
				}
				break;
			}
			
			case Position_ControlMode_OffBoard:
			{	//OffBoard模式
				if( inFlight )
				{
					if( OffBoardXY_target_mode == 3 )
					{	//控制位置+速度+加速度
						vector2<double> e1;
						e1.x = target_position_ds.x - Position.x;
						e1.y = target_position_ds.y - Position.y;
						vector2<double> e1_1;
						e1_1.x = target_velocity.x - VelocityENU.x;
						e1_1.y = target_velocity.y - VelocityENU.y;
						vector2<double> e1_2;
						e1_2.x = target_acc.x - TAcc.x;
						e1_2.y = target_acc.y - TAcc.y;
						double e1_length = safe_sqrt(e1.get_square());
						e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
						if( !is_zero(e1_length) )
							e_1 = e_1_n / e1_length;
						else
							e_1 = 0;
						e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
						if( !is_zero(e1_length*e1_length) )
							e_2 = e_2_n / (e1_length*e1_length);
						else
							e_2 = 0;
						
						double _pos_vel = cfg.maxVelXY[0];
//						if( e1_length < 100 )
//							pos_vel = -1;
//						if( pos_vel > 0 )
//							_pos_vel = pos_vel;					
						smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, _pos_vel );
						vector2<double> T2;
						vector2<double> T2_1;
						vector2<double> T2_2;
						if( !is_zero(e1_length*e1_length*e1_length) )
						{
							vector2<double> n = e1 * (1.0/e1_length);
							vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
							vector2<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
							T2 = n*d1.d0;
							T2_1 = n*d1.d1 + n_1*d1.d0;
							T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
						}
						TargetVelocity.x = T2.x + target_velocity.x;	TargetVelocity.y = T2.y + target_velocity.y;
						TargetVelocity_1.x = T21P*T2_1.x + target_acc.x;	TargetVelocity_1.y = T21P*T2_1.y + target_acc.y;
						TargetVelocity_2.x = T2_2.x;	TargetVelocity_2.y = T2_2.y;
					}
					else
					{	//速度+加速度
						TargetVelocity.x = target_velocity.x;	TargetVelocity.y = target_velocity.y;
						TargetVelocity_1.x = T21P*target_acc.x;	TargetVelocity_1.y = T21P*target_acc.y;
					}
				}
				else
				{	//没起飞前在位置控制模式
					//重置期望位置
					target_position.x = Position.x;
					target_position.y = Position.y;
					target_position_SM.zero();
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}		
			
//			case Position_ControlMode_VelocityTrack:
//			{
//				if( inFlight )
//				{
//					double max_acc = VelCtrlMaxAcc;
//					if( VelCtrlMaxAcc <= 0 )
//						max_acc = cfg.maxAccXY[0];
//					if( target_velocity.x > TargetVelocity.x )
//					{
//						TargetVelocity.x += max_acc*h;
//						TargetVelocity_1.x = max_acc;
//					}
//					
//					vector2<double> e1;
//					e1.x = target_position.x - Position.x;
//					e1.y = target_position.y - Position.y;
//					vector2<double> e1_1;
//					e1_1.x = - VelocityENU.x;
//					e1_1.y = - VelocityENU.y;
//					vector2<double> e1_2;
//					e1_2.x = - TAcc.x;
//					e1_2.y = - TAcc.y;
//					double e1_length = safe_sqrt(e1.get_square());
//					e_1_n = e1.x*e1_1.x + e1.y*e1_1.y;
//					if( !is_zero(e1_length) )
//						e_1 = e_1_n / e1_length;
//					else
//						e_1 = 0;
//					e_2_n = ( e1.x*e1_2.x + e1.y*e1_2.y + e1_1.x*e1_1.x + e1_1.y*e1_1.y )*e1_length - e_1*e_1_n;
//					if( !is_zero(e1_length*e1_length) )
//						e_2 = e_2_n / (e1_length*e1_length);
//					else
//						e_2 = 0;
//					
//					smooth_kp_d2 d1 = smooth_kp_2( e1_length, e_1, e_2 , Ps, 200 );
//					vector2<double> T2;
//					vector2<double> T2_1;
//					vector2<double> T2_2;
//					if( !is_zero(e1_length*e1_length*e1_length) )
//					{
//						vector2<double> n = e1 * (1.0/e1_length);
//						vector2<double> n_1 = (e1_1*e1_length - e1*e_1) / (e1_length*e1_length);
//						vector2<double> n_2 = ( (e1_2*e1_length-e1*e_2)*e1_length - (e1_1*e1_length-e1*e_1)*(2*e_1) ) / (e1_length*e1_length*e1_length);
//						T2 = n*d1.d0;
//						T2_1 = n*d1.d1 + n_1*d1.d0;
//						T2_2 = n*d1.d2 + n_1*(2*d1.d1) + n_2*d1.d0;
//					}
//					TargetVelocity.x = T2.x;	TargetVelocity.y = T2.y;
//					TargetVelocity_1.x = T2_1.x;	TargetVelocity_1.y = T2_1.y;
//					TargetVelocity_2.x = T2_2.x;	TargetVelocity_2.y = T2_2.y;
//				}
//				else
//				{
//					//没起飞前在位置控制模式
//					//重置期望位置
//					target_position.x = Position.x;
//					target_position.y = Position.y;
//					Attitude_Control_set_Target_RollPitch( 0, 0 );
//					goto PosCtrl_Finish;
//				}
//				break;
//			}		
			
			case Position_ControlMode_RouteLine:
			{
				if( inFlight )
				{						
					//计算垂足	
					vector2<double> A( target_position_ds.x, target_position_ds.y );					
					vector2<double> C( Position.x, Position.y );
					vector2<double> A_C = C - A;
					vector2<double> A_B( route_line_A_B.x, route_line_A_B.y );
					double k = (A_C * A_B) * route_line_m;
					vector2<double> foot_point = (A_B * k) + A;
					
					//计算跟踪A点位置
					vector2<double> A_desired = A;
					double A_B_length = safe_sqrt(A_B.get_square());		
					double line_track_desired_acc = 0;
					double comp_desired_vel = line_track_desired_vel;		
					if( A_B_length > 0.1 )
					{
						double AvDestL = A_B_length;
						if( max_line_track_desired>0 && max_line_track_desired<A_B_length ){
							//限制距离
							AvDestL = max_line_track_desired;
						}
						if( 0.5*line_track_desired_vel*line_track_desired_vel/line_track_desired_maxacc >= AvDestL - line_track_desired )
						{	//需要减速
							line_track_desired_vel -= line_track_desired_maxacc*h;
							if( line_track_desired_vel < 0 )
								line_track_desired_vel = 0;
							else
								line_track_desired_acc = -line_track_desired_maxacc;
						}
						else
						{	//前馈加速到最高速度
							double max_v = line_track_desired_maxv;
							if( cfg.WPRange[0] > 0 )
							{
								max_v = 0.8*(A_B_length - line_track_desired - cfg.WPRange[0]);
								if( max_v < cfg.SMVel[0] )
									max_v = cfg.SMVel[0];
								if( target_position_SM.get_square() > 1*1 )
									max_v = cfg.SMVel[0];
								if( max_v > line_track_desired_maxv )
									max_v = line_track_desired_maxv;
							}
							if( line_track_desired_vel < max_v )
								line_track_desired_vel += line_track_desired_maxacc*h;
							if( line_track_desired_vel > max_v )
								line_track_desired_vel = max_v;
							else
								line_track_desired_acc = +line_track_desired_maxacc;
						}
						//根绝障碍限制目标速度
						if( max_line_track_desired_vel>0 && line_track_desired_vel>max_line_track_desired_vel )
							line_track_desired_vel = max_line_track_desired_vel;
						//以目标速度移动目标位置
						line_track_desired += line_track_desired_vel*h;
						//根据障碍限制目标位置
						if( max_line_track_desired>0 && line_track_desired>max_line_track_desired )
							line_track_desired = max_line_track_desired;
						//限制目标位置
						if( line_track_desired > A_B_length )
							line_track_desired = A_B_length;
						
						comp_desired_vel = line_track_desired_vel;
						if( cfg.WPRange[0] > 0 )
						{	//提前到点
							//修正期望速度
							double s_rm = A_B_length - line_track_desired - cfg.WPRange[0];
							double mv = 0;
							if( s_rm > 0 )
								mv = safe_sqrt(2*line_track_desired_maxacc*s_rm);
							
							if( comp_desired_vel > mv )
								comp_desired_vel = mv;
						}
						
						vector2<double> B = A + A_B;
						A_desired = B - A_B*(line_track_desired/A_B_length);
						pos_vel = cfg.WPRange[0] * 1.0;
						if( pos_vel < 200 )
							pos_vel = 200;
					}
					
					//计算偏差
					vector2<double> e1r = A_desired - foot_point;
					vector2<double> e1d = foot_point - C;
					double e1r_length = safe_sqrt(e1r.get_square());
					double e1d_length = safe_sqrt(e1d.get_square());
					
					//计算route方向单位向量
					vector2<double> route_n;
					if( e1r_length > 0.001 )
						route_n = e1r * (1.0/e1r_length);
					
					//计算d方向单位向量
					vector2<double> d_n;
					if( e1d_length > 0.001 )
						d_n = e1d * (1.0/e1d_length);
					
					double route_sign = 1.0;
					if( route_n*vector2<double>(route_line_A_B.x,route_line_A_B.y) > 0 )
						route_sign = -1.0;
					
					//计算e1导数
					vector2<double> e1_1( VelocityENU.x, VelocityENU.y );
					double e1r_1 = route_sign*comp_desired_vel - (e1_1 * route_n);
					double e1d_1 = -(e1_1 * d_n);
					//e1二阶导
					vector2<double> e1_2( TAcc.x, TAcc.y );
					double e1r_2 = route_sign*line_track_desired_acc - (e1_2 * route_n);
					double e1d_2 = -(e1_2 * d_n);
					
					if( pos_vel < 200 )
						pos_vel = 200;
					/*route方向*/
						smooth_kp_d2 d1r = smooth_kp_2( e1r_length, e1r_1, e1r_2 , Ps, AutoVelXY+100 );
						vector2<double> T2r = route_n * d1r.d0;
						vector2<double> T2r_1 = route_n * d1r.d1;
						vector2<double> T2r_2 = route_n * d1r.d2;
					/*route方向*/
					
					/*d方向*/
						smooth_kp_d2 d1d = smooth_kp_2( e1d_length, e1d_1, e1d_2 , Ps, pos_vel );
						vector2<double> T2d = d_n * d1d.d0;
						vector2<double> T2d_1 = d_n * d1d.d1;
						vector2<double> T2d_2 = d_n * d1d.d2;
					/*d方向*/
						
					TargetVelocity.x = T2r.x+T2d.x;	TargetVelocity.y = T2r.y+T2d.y;
					TargetVelocity_1.x = T21P*T2r_1.x+T2d_1.x;	TargetVelocity_1.y = T21P*T2r_1.y+T2d_1.y;
					TargetVelocity_2.x = T2r_2.x+T2d_2.x;	TargetVelocity_2.y = T2r_2.y+T2d_2.y;
						
					//判断到点
					double wp_range = 0;
					if( cfg.WPRange[0] > 0 )
						wp_range = cfg.WPRange[0];
					if( wp_range < 0.1 )
						wp_range = 0.1;
					if( A_B_length - line_track_desired <= wp_range ) 
					{
						if( A_B_length > 0.1 )
						{	//增加缓慢移动量
							double SM_length = A_B_length - line_track_desired;
							vector2<double> SM_inc = A_B * (SM_length/A_B_length);
							target_position_SM.x += SM_inc.x;
							target_position_SM.y += SM_inc.y;
							target_SM_vel += line_track_desired_vel;
						}
						
						//计算到点延时时间
						double wp_delay = 0;
						if( cfg.WPRange[0] < 0 )
							wp_delay = -cfg.WPRange[0];
						
						//到点延时
						++line_track_delay_counter;
						if( line_track_delay_counter >= wp_delay*CtrlRateHz )
							HorizontalPosition_ControlMode = Position_ControlMode_Position;
					}
					else
						line_track_delay_counter = 0;
				}
				else
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			
			case Position_ControlMode_RouteLine3D:
			{
				if( inFlight )
				{
					//计算垂足
					vector3<double> A_C = Position - target_position_ds;
					double k = (A_C * route_line_A_B) * route_line_m;
					vector3<double> foot_point = (route_line_A_B * k) + target_position_ds;
					
					//计算跟踪A点位置
					vector3<double> A_desired = target_position_ds;
					double A_B_length = safe_sqrt(route_line_A_B.get_square());	
					double line_track_desired_acc = 0;		
					double comp_desired_vel = line_track_desired_vel;					
					if( A_B_length > 0.1 )
					{
						if( 0.5*line_track_desired_vel*line_track_desired_vel/line_track_desired_maxacc >= A_B_length - line_track_desired )
						{	//需要减速
							line_track_desired_vel -= line_track_desired_maxacc*h;
							if( line_track_desired_vel < 0 )
								line_track_desired_vel = 0;
							else
								line_track_desired_acc = -line_track_desired_maxacc;
						}
						else
						{	//前馈加速到最高速度
							double max_v = line_track_desired_maxv;
							if( cfg.WPRange[0] > 0 )
							{
								max_v = 0.8*(A_B_length - line_track_desired - cfg.WPRange[0]);
								if( max_v < cfg.SMVel[0] )
									max_v = cfg.SMVel[0];
								if( target_position_SM.get_square() > 1*1 )
									max_v = cfg.SMVel[0];
								if( max_v > line_track_desired_maxv )
									max_v = line_track_desired_maxv;
							}
							if( line_track_desired_vel < max_v )
								line_track_desired_vel += line_track_desired_maxacc*h;
							if( line_track_desired_vel > max_v )
								line_track_desired_vel = max_v;
							else
								line_track_desired_acc = +line_track_desired_maxacc;
						}
						line_track_desired += line_track_desired_vel*h;
						if( line_track_desired > A_B_length )
							line_track_desired = A_B_length;

						comp_desired_vel = line_track_desired_vel;
						if( cfg.WPRange[0] > 0 )
						{	//提前到点
							//修正期望速度
							double s_rm = A_B_length - line_track_desired - cfg.WPRange[0];
							double mv = 0;
							if( s_rm > 0 )
								mv = safe_sqrt(2*line_track_desired_maxacc*s_rm);
							
							if( comp_desired_vel > mv )
								comp_desired_vel = mv;
						}
						
						vector3<double> B = target_position_ds + route_line_A_B;
						A_desired = B - route_line_A_B*(line_track_desired/A_B_length);
						pos_vel = cfg.WPRange[0] * 1.0;
						if( pos_vel < 200 )
							pos_vel = 200;
					}
					
					//计算偏差
					vector3<double> e1r = A_desired - foot_point;
					vector3<double> e1d = foot_point - Position;
					double e1r_length = safe_sqrt(e1r.get_square());
					double e1d_length = safe_sqrt(e1d.get_square());
					
					//计算route方向单位向量
					vector3<double> route_n;
					if( e1r_length > 0.001 )
						route_n = e1r * (1.0/e1r_length);

					double route_sign = 1.0;
					if( route_n*route_line_A_B > 0 )
						route_sign = -1.0;
					
					//计算e1导数
					double e1r_1_length = -(VelocityENU * route_n);	
					vector3<double> e1r_1 = route_n * e1r_1_length;					
					vector3<double> e1d_1 = -(VelocityENU + e1r_1);
					e1r_1_length += route_sign*comp_desired_vel;
					//e1二阶导
					vector3<double> e1_2( TAcc.x, TAcc.y, AccelerationENU_ESO_Z );
					double e1r_2_length = -(e1_2 * route_n);
					vector3<double> e1r_2 = route_n * e1r_2_length;					
					vector3<double> e1d_2 = -(e1_2 + e1r_2);
					e1r_2_length += route_sign*line_track_desired_acc;
					
					if( pos_vel < 200 )
						pos_vel = 200;
					/*route方向*/
						smooth_kp_d2 d1r = smooth_kp_2( e1r_length, e1r_1_length, e1r_2_length , Ps, AutoVelXYZ+100 );
						vector3<double> T2r = route_n * d1r.d0;
						vector3<double> T2r_1 = route_n * d1r.d1;
						vector3<double> T2r_2 = route_n * d1r.d2;
					
						T2r_1.x *= T21P;
						T2r_1.y *= T21P;
					/*route方向*/

					/*d方向*/
						e_1_n = e1d.x*e1d_1.x + e1d.y*e1d_1.y + e1d.z*e1d_1.z;
						if( !is_zero(e1d_length) )
							e_1 = e_1_n / e1d_length;
						else
							e_1 = 0;
						e_2_n = ( e1d.x*e1d_2.x + e1d.y*e1d_2.y + e1d.z*e1d_2.z + e1d_1.x*e1d_1.x + e1d_1.y*e1d_1.y + e1d_1.z*e1d_1.z )*e1d_length - e_1*e_1_n;
						if( !is_zero(e1d_length*e1d_length) )
							e_2 = e_2_n / (e1d_length*e1d_length);
						else
							e_2 = 0;
						smooth_kp_d2 d1d = smooth_kp_2( e1d_length, e_1, e_2 , Ps, pos_vel );
						vector3<double> T2d;
						vector3<double> T2d_1;
						vector3<double> T2d_2;
						if( !is_zero(e1d_length*e1d_length*e1d_length) )
						{
							vector3<double> n = e1d * (1.0/e1d_length);
							vector3<double> n_1 = (e1d_1*e1d_length - e1d*e_1) / (e1d_length*e1d_length);
							vector3<double> n_2 = ( (e1d_2*e1d_length-e1d*e_2)*e1d_length - (e1d_1*e1d_length-e1d*e_1)*(2*e_1) ) / (e1d_length*e1d_length*e1d_length);
							T2d = n*d1d.d0;
							T2d_1 = n*d1d.d1 + n_1*d1d.d0;
							T2d_2 = n*d1d.d2 + n_1*(2*d1d.d1) + n_2*d1d.d0;
						}
						
						T2d_1.x *= T21P;
						T2d_1.y *= T21P;
					/*d方向*/
						
					TargetVelocity = T2r + T2d;
					TargetVelocity_1 = T2r_1 + T2d_1;
					TargetVelocity_2 = T2r_2 + T2d_2;
						
					//判断到点
					pos_vel = AutoVelXYZ;
					double wp_range = 0;
					if( cfg.WPRange[0] > 0 )
						wp_range = cfg.WPRange[0];
					if( wp_range < 0.1 )
						wp_range = 0.1;
					if( A_B_length - line_track_desired <= wp_range ) 
					{
						if( A_B_length > 0.1 )
						{	//增加缓慢移动量
							double SM_length = A_B_length - line_track_desired;
							vector3<double> SM_inc = route_line_A_B * (SM_length/A_B_length);
							target_position_SM += SM_inc;
							target_SM_vel += line_track_desired_vel;
						}
						
						//计算到点延时时间
						double wp_delay = 0;
						if( cfg.WPRange[0] < 0 )
							wp_delay = -cfg.WPRange[0];
						
						//到点延时
						++line_track_delay_counter;
						if( line_track_delay_counter >= wp_delay*CtrlRateHz )
							HorizontalPosition_ControlMode = Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
						line_track_delay_counter = 0;
				}
				else
				{
					//没起飞时重置期望速度
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
			
			case Position_ControlMode_Locking:
			default:
			{	//刹车锁位置
				static uint16_t lock_counter = 0;
				target_position_SM.zero();
				if( inFlight )
				{
					double ExpectVelLength = safe_sqrt(Expected_VelXY.get_square());
					if( ExpectVelLength > XYLock_CAcc*h )
					{
						lock_counter = 0;
						double inv_ExpectVelLength = 1.0 / ExpectVelLength;
						vector2<double> dec = Expected_VelXY*(inv_ExpectVelLength * XYLock_CAcc*h);
						Expected_VelXY -= dec;
						TargetVelocity.x = Expected_VelXY.x;
						TargetVelocity.y = Expected_VelXY.y;
					}
					else
					{
						TargetVelocity.x = 0;
						TargetVelocity.y = 0;						
						if( ++lock_counter >= CtrlRateHz*1.2 )
						{
							lock_counter = 0;
							TargetVelocity.zero();
							target_position.x = Position.x + 1.0/Pv*VelocityENU.x;
							target_position.y = Position.y + 1.0/Pv*VelocityENU.y;
							HorizontalPosition_ControlMode = Position_ControlMode_Position;
						}
					}
				}
				else
				{
					lock_counter = 0;
					target_position.x = Position.x;
					target_position.y = Position.y;
					HorizontalPosition_ControlMode = Position_ControlMode_Position;
					Attitude_Control_set_Target_RollPitch( 0, 0 );
					goto PosCtrl_Finish;
				}
				break;
			}
		}	
		
		//计算期望加速度
		vector2<double> e2;
		e2.x = TargetVelocity.x - VelocityENU.x;
		e2.y = TargetVelocity.y - VelocityENU.y;
		vector2<double> e2_1;
		e2_1.x = TargetVelocity_1.x - TAcc.x;
		e2_1.y = TargetVelocity_1.y - TAcc.y;
		double e2_length = safe_sqrt(e2.get_square());
		e_1_n = e2.x*e2_1.x + e2.y*e2_1.y;
		if( !is_zero(e2_length) )
			e_1 = e_1_n / e2_length;
		else
			e_1 = 0;
		smooth_kp_d1 d2;
		if( HorizontalPosition_ControlMode==Position_ControlMode_Locking )
		{
			XYLock_CAcc += h * cfg.maxBreakJerk[0];
			if( XYLock_maxAcc>50 )
				XYLock_CAcc = constrain( XYLock_CAcc, 10.0, XYLock_maxAcc );
			else
				XYLock_CAcc = constrain( XYLock_CAcc, 10.0, (double)cfg.maxAccXY[0] );
			d2 = smooth_kp_1( e2_length, e_1 , Pv, XYLock_CAcc );
		}
		else
			d2 = smooth_kp_1( e2_length, e_1 , Pv, cfg.maxAccXY[0] );
		vector2<double> T3;
		vector2<double> T3_1;
		if( !is_zero(e2_length*e2_length) )
		{
			vector2<double> n = e2 * (1.0/e2_length);
			vector2<double> n_1 = (e2_1*e2_length - e2*e_1) / (e2_length*e2_length);
			T3 = n*d2.d0;
			T3_1 = n*d2.d1 + n_1*d2.d0;
		}
		T3 += vector2<double>( TargetVelocity_1.x, TargetVelocity_1.y );
		T3_1 += vector2<double>( TargetVelocity_2.x, TargetVelocity_2.y );
		
		vector2<double> e3;
		e3.x = T3.x - TAcc.x;
		e3.y = T3.y - TAcc.y;
		double e3_length = safe_sqrt(e3.get_square());
		double d3;
		if( Is_AutoMode(HorizontalPosition_ControlMode) )
			d3 = smooth_kp_0( e3_length , Pa, cfg.maxAutoJerkXY[0] );
		else
			d3 = smooth_kp_0( e3_length , Pa, cfg.maxJerkXY[0] );
		vector2<double> T4;
		if( !is_zero(e3_length) )
		{
			vector2<double> n = e3 * (1.0/e3_length);
			T4 = n*d3;
		}
		if( Is_AutoMode(HorizontalPosition_ControlMode) )
			T4.constrain( cfg.maxAutoJerkXY[0] );
		else
			T4.constrain( cfg.maxJerkXY[0] );
		T4 += T3_1;
		
		TAcc.x += T4.x*h;
		TAcc.y += T4.y*h;	
		
		//去除风力扰动
		vector3<double> WindDisturbance;
		get_WindDisturbance( &WindDisturbance );
		vector2<double> target_acceleration;
//		target_acceleration.x = TAcc.x - WindDisturbance.x;
//		target_acceleration.y = TAcc.y - WindDisturbance.y;
		target_acceleration.x = T3.x - WindDisturbance.x;
		target_acceleration.y = T3.y - WindDisturbance.y;
		
		//旋转至Bodyheading
		Quaternion attitude;
		get_Attitude_quat(&attitude);
		double yaw = attitude.getYaw();		
		double sin_Yaw, cos_Yaw;
		fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
		double target_acceleration_x_bodyheading = ENU2BodyHeading_x( target_acceleration.x , target_acceleration.y , sin_Yaw , cos_Yaw );
		double target_acceleration_y_bodyheading = ENU2BodyHeading_y( target_acceleration.x , target_acceleration.y , sin_Yaw , cos_Yaw );
//		target_acceleration_x_bodyheading = ThrOut_Filters[0].run(target_acceleration_x_bodyheading);
//		target_acceleration_y_bodyheading = ThrOut_Filters[1].run(target_acceleration_y_bodyheading);
		
		//计算风力补偿角度
		double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
		double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
		//计算角度
		double AccUp = GravityAcc + AccelerationENU_ESO_Z;
		double AntiDisturbancePitch = atan2( -WindDisturbance_Bodyheading_x , AccUp );
		double AntiDisturbanceRoll = atan2( WindDisturbance_Bodyheading_y , AccUp );
		
		if( inFlight )
		{
			double logbuf[4];
			logbuf[0] = WindDisturbance_Bodyheading_x;
			logbuf[1] = TargetVelocity.x;
			logbuf[2] = VelocityENU.x;
			logbuf[3] = target_acceleration.x;
			SDLog_Msg_DebugVect( "sp", logbuf, 4 );
		}
		
		//计算目标角度
		double target_Roll = atan2( -target_acceleration_y_bodyheading , AccUp );
		double target_Pitch = atan2( target_acceleration_x_bodyheading , AccUp );
		if( HorizontalPosition_ControlMode==Position_ControlMode_Velocity )
		{	//角度限幅
			if( VelCtrlMaxRoll>0 && VelCtrlMaxPitch>0 )
			{
				target_Roll = constrain( target_Roll , AntiDisturbanceRoll - VelCtrlMaxRoll, AntiDisturbanceRoll + VelCtrlMaxRoll );
				target_Pitch = constrain( target_Pitch , AntiDisturbancePitch - VelCtrlMaxPitch, AntiDisturbancePitch + VelCtrlMaxPitch );
			}
			else if( VelCtrlMaxRoll>0 )
			{
				vector2<double> Tangle( target_Roll - AntiDisturbanceRoll, target_Pitch - AntiDisturbancePitch );
				Tangle.constrain(VelCtrlMaxRoll);
				target_Roll = AntiDisturbanceRoll + Tangle.x;
				target_Pitch = AntiDisturbancePitch + Tangle.y;
			}
		}
		
		//设定目标角度
		Attitude_Control_set_Target_RollPitch( target_Roll, target_Pitch );
		
		//获取真实目标角度修正TAcc
		Attitude_Control_get_Target_RollPitch( &target_Roll, &target_Pitch );
		target_acceleration_x_bodyheading = tan(target_Pitch)*GravityAcc;
		target_acceleration_y_bodyheading = -tan(target_Roll)*GravityAcc;
		target_acceleration.x = BodyHeading2ENU_x( target_acceleration_x_bodyheading, target_acceleration_y_bodyheading , sin_Yaw, cos_Yaw );
		target_acceleration.y = BodyHeading2ENU_y( target_acceleration_x_bodyheading, target_acceleration_y_bodyheading , sin_Yaw, cos_Yaw );
		TAcc.x = target_acceleration.x + WindDisturbance.x;
		TAcc.y = target_acceleration.y + WindDisturbance.y;
	}//水平位置控制
	else
	{
		ThrOut_Filters[0].reset(0);
		ThrOut_Filters[1].reset(0);
	}
	
PosCtrl_Finish:	
	if( Altitude_Control_Enabled )
	{//高度控制
			
		//设置控制量限幅
		Target_tracker[2].r2p = cfg.maxVelUp[0];
		Target_tracker[2].r2n = cfg.maxVelDown[0];
		Target_tracker[2].r3p = cfg.maxAccUp[0];
		Target_tracker[2].r3n = cfg.maxAccDown[0];
		Target_tracker[2].r4p = cfg.maxJerkUp[0];
		Target_tracker[2].r4n = cfg.maxJerkDown[0];
		
		if( !Is_3DAutoMode(Altitude_ControlMode) )
		{
			switch( Altitude_ControlMode )
			{
				case Position_ControlMode_Position:
				{	//控制位置
					if( inFlight )
					{
						Target_tracker[2].r2p = 0.3*cfg.maxVelUp[0];
						Target_tracker[2].r2n = 0.3*cfg.maxVelDown[0];
						Target_tracker[2].track4( target_position_ds.z , 1.0 / CtrlRateHz );
					}
					else
					{
						//没起飞前在位置控制模式
						//不要起飞
						Target_tracker[2].reset();
						target_position.z = Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( 0 );
						goto AltCtrl_Finish;
					}
					break;
				}
				case Position_ControlMode_OffBoard:
				{	//控制位置
					bool OffBoardUp = false;
					if( OffBoardZ_target_mode==3 )
					{						
						if( target_position_ds.z>=Position.z+10 && target_velocity.z>=0 && target_acc.z>=0 )
							OffBoardUp = true;
					}
					else
					{
						if( target_velocity.z>=10 && target_acc.z>=0 )
							OffBoardUp = true;
					}
					if( inFlight || OffBoardUp )
					{
						if( OffBoardZ_target_mode == 3 )
						{	//位置+速度+加速度
							Target_tracker[2].r2p = 0.3*cfg.maxVelUp[0];
							Target_tracker[2].r2n = 0.3*cfg.maxVelDown[0];
							Target_tracker[2].track4( target_position_ds.z,target_velocity.z,target_acc.z,0,0, 1.0/CtrlRateHz );
						}
						else
						{	//速度+加速度
							double TVel;
							if( target_velocity.z > cfg.maxVelUp[0] )
								TVel = cfg.maxVelUp[0];
							else if( target_velocity.z < -cfg.maxVelDown[0] )
								TVel = -cfg.maxVelDown[0];
							else
								TVel = target_velocity.z;
							Target_tracker[2].track3( TVel,target_acc.z,0,0 , 1.0/CtrlRateHz );
						}
					}
					else
					{	//没起飞前在位置控制模式
						//不要起飞
						Target_tracker[2].reset();
						target_position.z = Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( 0 );
						goto AltCtrl_Finish;
					}
					break;
				}
				case Position_ControlMode_Velocity:
				{	//控制速度
					target_position_SM.zero();
					if( inFlight )
					{
						double TVel;
						if( target_velocity.z > cfg.maxVelUp[0] )
							TVel = cfg.maxVelUp[0];
						else if( target_velocity.z < -cfg.maxVelDown[0] )
							TVel = -cfg.maxVelDown[0];
						else
							TVel = target_velocity.z;
						Target_tracker[2].track3( TVel , 1.0 / CtrlRateHz );
					}
					else
					{
						//没起飞且期望速度为负
						//不要起飞
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;						
						if( target_velocity.z <= 0 )
						{
							Attitude_Control_set_Throttle( 0 );
							goto AltCtrl_Finish;
						}
					}
					break;
				}
				
				case Position_ControlMode_Takeoff:
				{	//起飞
					target_position_SM.zero();
					//设置控制量最大值
					Target_tracker[2].r3p = cfg.maxAutoAccUp[0];
					Target_tracker[2].r3n = cfg.maxAutoAccDown[0];
					Target_tracker[2].r4p = cfg.maxAutoJerkUp[0];
					Target_tracker[2].r4n = cfg.maxAutoJerkDown[0];
					
					if( inFlight )
					{
						//已起飞
						//控制达到目标高度
						double homeZ;
						getHomeLocalZ(&homeZ);
						if( Position.z - homeZ < 100 )
							Target_tracker[2].r2n = Target_tracker[2].r2p = 50;
						else
							Target_tracker[2].r2n = Target_tracker[2].r2p = AutoVelZUp;
						Target_tracker[2].track4( target_position.z , 1.0 / CtrlRateHz );
						if( fabs( Target_tracker[2].x1 - target_position.z ) < 0.7 && \
								fabs( target_position.z - Position.z ) < 50 && \
								in_symmetry_range( Target_tracker[2].x2 , 0.7 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.7 )	)
							Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
					{
						//未起飞
						//等待起飞
						target_position.z = Position.z + TakeoffHeight;
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
					}
					break;
				}
				case Position_ControlMode_RouteLine:
				{	//飞到指定高度
					
					//设置控制量最大值
					Target_tracker[2].r3p = cfg.maxAutoAccUp[0];
					Target_tracker[2].r3n = cfg.maxAutoAccDown[0];
					Target_tracker[2].r4p = cfg.maxAutoJerkUp[0];
					Target_tracker[2].r4n = cfg.maxAutoJerkDown[0];
					
					if( inFlight )
					{
						//已起飞
						//控制达到目标高度
						Target_tracker[2].r2n = AutoVelZUp;
						Target_tracker[2].r2p = AutoVelZDown;
						Target_tracker[2].track4( target_position_ds.z , 1.0f / CtrlRateHz );
						if( fabs( Target_tracker[2].x1 - target_position_ds.z ) < 0.7 && \
								fabs( target_position_ds.z - Position.z ) < 50 && \
								in_symmetry_range( Target_tracker[2].x2 , 0.7 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.7 )	)
							Altitude_ControlMode = Position_ControlMode_Position;
					}
					else
					{
						//未起飞
						//不要起飞
						Target_tracker[2].reset();
						Target_tracker[2].x1 = Position.z;
						Attitude_Control_set_Throttle( 0 );
						goto AltCtrl_Finish;
					}
					break;
				}
				
				case Position_ControlMode_Locking:
				default:
				{	//锁位置（减速到0然后锁住高度）
					target_position_SM.zero();
					if( inFlight )
					{
						Target_tracker[2].track3( 0 , 1.0 / CtrlRateHz );
						if( in_symmetry_range( VelocityENU.z , 10.0 ) && \
								in_symmetry_range( Target_tracker[2].x2 , 0.3 ) && \
								in_symmetry_range( Target_tracker[2].x3 , 0.3 ) && \
								in_symmetry_range( Target_tracker[2].x4 , 0.3 )	)
						{
							target_position.z = Target_tracker[2].x1 = Position.z;
							Altitude_ControlMode = Position_ControlMode_Position;
						}
					}
					else
					{
						Altitude_ControlMode = Position_ControlMode_Position;
						Attitude_Control_set_Throttle( 0 );
						goto AltCtrl_Finish;
					}
					break;
				}
			}
		}
		
		if( inFlight )
		{
			//计算期望速度
			double target_velocity_z;
			//期望垂直速度的导数
			double Tvz_1, Tvz_2;
			if( Is_3DAutoMode(Altitude_ControlMode) )
			{
				target_velocity_z = TargetVelocity.z;
				Tvz_1 = TargetVelocity_1.z;
				Tvz_2 = TargetVelocity_2.z;
				Target_tracker[2].reset();
				Target_tracker[2].x1 = target_position.z;
			}
			else
			{
				if( Target_tracker[2].get_tracking_mode() == 4 )
				{
					double max_fb_vel = ( Target_tracker[2].x1 - Position.z ) > 0 ? cfg.maxAutoVelUp[0] : cfg.maxAutoVelDown[0];
					smooth_kp_d2 TvFb = smooth_kp_2(
							Target_tracker[2].x1 - Position.z,
							Target_tracker[2].x2 - VelocityENU.z, 
							Target_tracker[2].x3 - AccelerationENU_ESO_Z, 
							Ps, max_fb_vel );
					target_velocity_z = TvFb.d0 + Target_tracker[2].x2;
					Tvz_1 = TvFb.d1 + Target_tracker[2].x3;
					Tvz_2 = TvFb.d2 + Target_tracker[2].x4;
				}
				else
				{
					target_velocity_z = Target_tracker[2].x2;
					Tvz_1 = Target_tracker[2].x3;
					Tvz_2 = Target_tracker[2].x4;
				}
			}
			
			//计算期望加速度
			double max_fb_acc = ( target_velocity_z - VelocityENU.z ) > 0 ? cfg.maxAutoAccUp[0] : cfg.maxAutoAccDown[0];
			smooth_kp_d1 TaFb = smooth_kp_1(
					target_velocity_z - VelocityENU.z,
					Tvz_1 - AccelerationENU_ESO_Z, 
					2, max_fb_acc );
			double target_acceleration_z = TaFb.d0 + Tvz_1;
			double target_acceleration_z_1 = TaFb.d1 + Tvz_2;
			
			//长期速度误差时加速修正
			double velErr = target_velocity_z - VelocityENU.z;
			static float velErr_lastT = 0;
			if( velErr>0 && velErr_lastT>=0 )
				velErr_lastT += h;
			else if( velErr<0 && velErr_lastT<=0 )
				velErr_lastT -= h;
			else
				velErr_lastT = 0;
			float targetAccZ_Inc = constrain( 80 * velErr_lastT*velErr_lastT*velErr_lastT, -500.0f, 500.0f );
			target_acceleration_z += targetAccZ_Inc;
			
			//target_acceleration_z = TargetVelocityFilter[2].run( target_acceleration_z );
			//加速度误差
			double acceleration_z_error = target_acceleration_z - AccelerationENU.z;
			double AccZErrP = cfg.ZSense[0]*10;
			acceleration_z_error = AccZ_Err_filter.track3( acceleration_z_error, h, AccZErrP,AccZErrP,AccZErrP );
			AccZ_Err_filter.x1 -= h*( Pa*acceleration_z_error );
			AccZ_Err_filter.x2 -= Pa*acceleration_z_error;
			
			//获取倾角cosin
			Quaternion quat;
			get_Airframe_quat(&quat);
			double lean_cosin = quat.get_lean_angle_cosin();
			
			//获取悬停油门 - 电机起转油门
			double hover_throttle;
			get_hover_throttle(&hover_throttle);
			
			//计算输出油门
			double force, T, b;
			get_throttle_force(&force);
			get_ESO_height_T(&T);
			get_throttle_b(&b);
			if( force < 1 )
				force = 1;
			double throttle = ( force + T * ( Pa*acceleration_z_error + target_acceleration_z_1 ) )/b;
			//double throttle = ( target_acceleration_z + Pa*T*acceleration_z_error )/b + hover_throttle;
			//倾角补偿
			if( lean_cosin > 0.1 )				
				throttle /= lean_cosin;
			else	//倾角太大
				throttle = 50;
			
			if( inFlight )
			{
				double logbuf[10];
				logbuf[0] = throttle;
				logbuf[1] = hover_throttle;
				logbuf[2] = force;
				logbuf[3] = target_acceleration_z;
				logbuf[4] = AccelerationENU_ESO_Z;
				SDLog_Msg_DebugVect( "thr", logbuf, 5 );
			}
			
			//油门限幅
			if( throttle > 100 )
				throttle = 100;
			if( inFlight )
			{
				if( throttle < 0 )
					throttle = 0;
			}
			
			//输出
			Attitude_Control_set_Throttle( throttle );
		}
		else
		{
			//没起飞
			//均匀增加油门起飞
			double throttle;
			get_OutputThrottle(&throttle);
			ThrOut_Filters[2].reset(throttle);
			AccZ_Err_filter.reset();
			Attitude_Control_set_Throttle( throttle + h * 35 );
		}
		
	}//高度控制
AltCtrl_Finish:
	return;
}

void init_Ctrl_Position()
{
	//初始化期望TD4滤波器
	Target_tracker[0] = TD4_SL( 15 , 15 , 15 , 15 );
	Target_tracker[1] = TD4_SL( 15 , 15 , 15 , 15 );
	Target_tracker[2] = TD4_SL( 1 , 2 , 50 , 60 );	
	Target_tracker[2].r3p = 900;
	Target_tracker[2].r3n = 600;
	
	//初始化期望速度低通滤波器
	TargetVelocityFilter[0].set_cutoff_frequency( CtrlRateHz , 10 );
	TargetVelocityFilter[1].set_cutoff_frequency( CtrlRateHz , 10 );
	TargetVelocityFilter[2].set_cutoff_frequency( CtrlRateHz , 10 );
	
	AccZ_Err_filter.reset();
	
	/*初始化参数*/
		PosCtrlCfg initial_cfg;
		//默认XY航线速度
		initial_cfg.AutoVXY[0] = 1000;
		//默认Z向上航线速度
		initial_cfg.AutoVZUp[0] = 450;
		//默认Z向下航线速度
		initial_cfg.AutoVZDown[0] = 350;
		//默认XYZ航线速度
		initial_cfg.AutoVXYZ[0] = 1000;	
		
		//高度前馈滤波器
		initial_cfg.Z_TD4P1[0] = 1;
		initial_cfg.Z_TD4P2[0] = 5;
		initial_cfg.Z_TD4P3[0] = 50;
		initial_cfg.Z_TD4P4[0] = 60;
		
		//位置反馈增益
		initial_cfg.P1[0] = 1;
		//速度反馈增益
		initial_cfg.P2[0] = 2;
		//加速度反馈增益
		initial_cfg.P3[0] = 10;
		//水平速度控制速度反馈增益
		initial_cfg.P2_VelXY[0] = 4;
		//水平速度前馈
		initial_cfg.VelXYFF[0] = 0.3;
		//垂直加速度反馈滤波器
		initial_cfg.ZSense[0] = 8;
		
		//最大水平速度
		initial_cfg.maxVelXY[0] = 1500;
		//最大水平加速度
		initial_cfg.maxAccXY[0] = 600;
		//最大水平加加速度
		initial_cfg.maxJerkXY[0] = 3000;
		//自动模式最大水平速度
		initial_cfg.maxAutoVelXY[0] = 1200;
		//自动模式最大水平加速度
		initial_cfg.maxAutoAccXY[0] = 200;
		//自动模式最大水平加加速度
		initial_cfg.maxAutoJerkXY[0] = 600;
		
		//最大向上速度
		initial_cfg.maxVelUp[0] = 500;
		//最大向下速度
		initial_cfg.maxVelDown[0] = 400;
		//最大向上加速度
		initial_cfg.maxAccUp[0] = 800;
		//最大向下加速度
		initial_cfg.maxAccDown[0] = 600;
		//最大向上加加速度
		initial_cfg.maxJerkUp[0] = 5000;
		//最大向下加加速度
		initial_cfg.maxJerkDown[0] = 3000;		
		//自动模式最大向上速度
		initial_cfg.maxAutoVelUp[0] = 500;
		//自动模式最大向下速度
		initial_cfg.maxAutoVelDown[0] = 350;
		//自动模式最大向上加速度
		initial_cfg.maxAutoAccUp[0] = 250;
		//自动模式最大向下加速度
		initial_cfg.maxAutoAccDown[0] = 200;
		//自动模式最大向上加加速度
		initial_cfg.maxAutoJerkUp[0] = 800;
		//自动模式最大向下加加速度
		initial_cfg.maxAutoJerkDown[0] = 800;
		
		//最大刹车加速度比例
		initial_cfg.maxBreakAccPc[0] = 1.0f;
		//最大刹车加加速度
		initial_cfg.maxBreakJerk[0] = 2000;
		
		//到达目标点范围
		initial_cfg.WPRange[0] = -1;
		//降落速度
		initial_cfg.LandVel[0] = 50;
		//协调转弯速度
		initial_cfg.SMVel[0] = 250;
	/*初始化参数*/

	SName param_names[] = {
		//默认XY航线速度
		"PC_AutoVXY" ,	
		//默认Z向上航线速度
		"PC_AutoVZUp" ,	
		//默认Z向下航线速度
		"PC_AutoVZDn" ,	
		//默认XYZ航线速度
		"PC_AutoVXYZ" ,	
		
		//高度前馈滤波器
		"PC_Z_TD4P1" ,
		"PC_Z_TD4P2" ,
		"PC_Z_TD4P3" ,
		"PC_Z_TD4P4" ,
		
		//位置反馈增益
		"PC_P1" ,
		//速度反馈增益
		"PC_P2" ,
		//加速度反馈增益
		"PC_P3" ,
		//水平速度控制速度反馈增益
		"PC_P2VelXY" ,
		//水平速度前馈
		"PC_VelXYFF" ,
		//垂直加速度反馈滤波器
		"PC_ZSense" ,
		
		//最大水平速度
		"PC_maxVelXY" ,
		//最大水平加速度
		"PC_maxAccXY" ,
		//最大水平加加速度
		"PC_maxJerkXY" ,
		//自动模式最大水平速度
		"PC_maxAutoVelXY" ,
		//自动模式最大水平加速度
		"PC_maxAutoAccXY" ,
		//自动模式最大水平加加速度
		"PC_maxAutoJerkXY" ,
		
		//最大向上速度
		"PC_maxVelUp" ,
		//最大向下速度
		"PC_maxVelDn" ,
		//最大向上加速度
		"PC_maxAccUp" ,
		//最大向下加速度
		"PC_maxAccDn" ,
		//最大向上加加速度
		"PC_maxJerkUp" ,
		//最大向下加加速度
		"PC_maxJerkDn" ,
		//自动模式最大向上速度
		"PC_maxAutoVelUp" ,
		//自动模式最大向下速度
		"PC_maxAutoVelDn" ,
		//自动模式最大向上加速度
		"PC_maxAutoAccUp" ,
		//自动模式最大向下加速度
		"PC_maxAutoAccDn" ,
		//自动模式最大向上加加速度
		"PC_maxAutoJerkUp" ,
		//自动模式最大向下加加速度
		"PC_maxAutoJerkDn" ,
		
		//最大刹车加速度比例
		"PC_maxBreakAccPc",
		//最大刹车加加速度
		"PC_maxBreakJerk",
		
		//到达目标点范围
		"PC_WPRange" ,
		//降落速度
		"PC_LandVel" ,
		//协调转弯速度
		"PC_SMVel" ,
	};

	MAV_PARAM_TYPE param_types[] = {
		//默认XY航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//默认Z向上航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//默认Z向下航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//默认XYZ航线速度
		MAV_PARAM_TYPE_REAL32 ,	
		//高度前馈滤波器
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		MAV_PARAM_TYPE_REAL32 ,
		
		//位置反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		//速度反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		//加速度反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		//水平速度控制速度反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		//水平速度前馈
		MAV_PARAM_TYPE_REAL32 ,
		//水平速度控制加速度反馈增益
		MAV_PARAM_TYPE_REAL32 ,
		
		//最大水平速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大水平加速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大水平加加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大水平速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大水平加速度
		MAV_PARAM_TYPE_REAL32 ,
		//垂直加速度反馈滤波器
		MAV_PARAM_TYPE_REAL32 ,
		
		//最大向上速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向下速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向上加速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向下加速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向上加加速度
		MAV_PARAM_TYPE_REAL32 ,
		//最大向下加加速度
		MAV_PARAM_TYPE_REAL32 ,	
		//自动模式最大向上速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向下速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向上加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向下加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向上加加速度
		MAV_PARAM_TYPE_REAL32 ,
		//自动模式最大向下加加速度
		MAV_PARAM_TYPE_REAL32 ,
		
		//最大刹车加速度比例
		MAV_PARAM_TYPE_REAL32,
		//最大刹车加加速度
		MAV_PARAM_TYPE_REAL32,
		
		//到达目标点范围
		MAV_PARAM_TYPE_REAL32 ,
		//降落速度
		MAV_PARAM_TYPE_REAL32 ,
		//协调转弯速度
		MAV_PARAM_TYPE_REAL32 ,
	};

	ThrOut_Filters[0].set_cutoff_frequency( CtrlRateHz, 20 );
	ThrOut_Filters[1].set_cutoff_frequency( CtrlRateHz, 20 );
	ThrOut_Filters[2].set_cutoff_frequency( CtrlRateHz, 20 );
	
	ParamGroupRegister( "PosCtrl", 7, sizeof(cfg)/8, param_types, param_names, (uint64_t*)&initial_cfg );
}