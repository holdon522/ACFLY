#include "ctrl_Attitude.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "Parameters.hpp"
#include "MeasurementSystem.hpp"
#include "TD4.hpp"
#include "TD3_3D.hpp"
#include "ESO_AngularRate.hpp"
#include "ESO_h.hpp"
#include "Filters_LP.hpp"
#include "drv_PWMOut.hpp"
#include "Receiver.hpp"
#include "drv_ADC.hpp"

#include "StorageSystem.hpp"

/*参数*/
	//控制方式定义
	struct UAVCtrlM
	{
		//初始化
		void (*Init)();
		//解锁前动作
		void (*PreArmControl)( Receiver rc );
		//初始化动作
		void (*InitControl)( bool DbgSafe );
		//动力分配
		void (*MotorControl)( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe );
		//eso类型 0-二阶 1-一阶(Heli)
		int8_t RP_eso_type;	int8_t Y_eso_type;
		//控制器类型 0-二阶推力导数控制 1-期望加速度P减扰动 2-期望加速度PD减扰动
		int8_t RP_ctrl_type;	int8_t Y_ctrl_type;
		//是否在初始化时执行控制程序
		bool ctrl_on_init;
		//是否在离地（起飞）前不添加扰动
		bool RP_NoDisturbanceOG;
	};
	static bool CtrlM_available =false;
	static UAVCtrlM CtrlM;
	extern const UAVCtrlM* UAVCtrlMs[];

	//控制参数
	struct AttCtrlCfg
	{
		uint8_t UAVType[8];	//机型类型
		float STThrottle[2];	//起转油门
		float NonlinearFactor[2];	//电机非线性参数
		float FullThrRatio[2];	//满油门比例
		uint8_t STMode[8];	//启动模式
		float STDelay[2];	//启动延时
		float T[2];	//惯性时间T
		float T2[2];	//惯性时间T2
		float b[6];	//RPY增益b
		float TD4_P1[6];	//RPY前馈TD4滤波器P1
		float TD4_P2[6];	//RPY前馈TD4滤波器P2
		float TD4_P3[6];	//RPY前馈TD4滤波器P3
		float TD4_P4[6];	//RPY前馈TD4滤波器P4
		float P1[6];	//反馈增益P1
		float P2[6];	//反馈增益P2
		float P3[6];	//反馈增益P3
		float P4[6];	//反馈增益P4
		float beta[2];	//ESO beta
		float beta2[2];	//ESO beta
		float beta_h[2];	//ESO_h beta
		float beta_hAcc[2];	//ESO_hAcc beta
		float maxLean[2];	//最大倾斜角
		float maxRPSp[2];	//最大Pitch Roll速度
		float maxRPAcc[2]; //最大Pitch Roll加速度
		float maxYSp[2];	//最大偏航速度
		float maxYAcc[2];	//最大偏航加速度
		float YawPri[2];	//偏航优先级（最大允许为偏航下降的油门量）
		
		float st_Freq[2];	//舵机PWM频率
		float st1min[2];	//舵机1最小值
		float st1max[2];	//舵机1最大值
		float st2min[2];	//舵机2最小值
		float st2max[2];	//舵机2最大值
		float st3min[2];	//舵机3最小值
		float st3max[2];	//舵机3最大值
		float st4min[2];	//舵机4最小值
		float st4max[2];	//舵机4最大值
		float st5min[2];	//舵机5最小值
		float st5max[2];	//舵机5最大值
		float st6min[2];	//舵机6最小值
		float st6max[2];	//舵机6最大值
		float st7min[2];	//舵机1最小值
		float st7max[2];	//舵机1最大值
		float st8min[2];	//舵机2最小值
		float st8max[2];	//舵机2最大值
	}__PACKED;

	//参数
	static AttCtrlCfg cfg;
	
	//电池掉压估计
	struct VoltKpLS 
	{
		public:
			double sumTV;
			double sumT;
			double sumV;
			double sumT2;
			uint32_t n;
			double max_thr, min_thr;
		
			inline void reset() { n = sumTV = sumT = sumV = sumT2 = 0; }
			inline void doIntegral( double thr, double volt )
			{
				if( n==0 ) {
					min_thr = max_thr = thr;
				} else {
					if( thr<min_thr )
						min_thr = thr;
					else if( thr>max_thr )
						max_thr = thr;
				}
				sumT += thr;
				sumV += volt;
				sumTV += thr*volt;
				sumT2 += sq(thr);
				++n;
			}
			inline double calcKp()
			{	
				if( n < 20 )
					return 0;
				double res = n*sumT2 - sq(sumT);
				if( fabs(res) < 0.001 )
					return 0;
				res = (n*sumTV - sumT*sumV) / res;
				if( res > -0.00001 )
					return 0;
				return res;
			}
	};
	static VoltKpLS thrVoltKpLS;
	static VoltKpLS curVoltKpLS;
	static double thrVoltKp = 0;
	static double curVoltKp = 0;
/*参数*/
	
/*内部接口*/
	float get_STThrottle()
	{
		return cfg.STThrottle[0];
	}
	float get_maxLean()
	{
		return cfg.maxLean[0];
	}
	float get_maxYawSpeed()
	{
		return cfg.maxYSp[0];
	}
	float getVoltKp()
	{
		return thrVoltKp;
	}
/*内部接口*/
	
/*起飞地点*/
	static bool HomeLatLonAvailable;
	static bool HomeAvailable;
	static vector2<double> HomeLatLon;
	static vector2<double> HomePoint;
	static double HomeLocalZ = 0;
	static double HomeYaw = 0;
	bool getHomeLocalZ( double* home, double* home_yaw, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if(home)
				*home = HomeLocalZ;
			if(home_yaw)
				*home_yaw = HomeYaw;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool getHomePoint( vector2<double>* home, double* home_yaw, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			bool available = HomeAvailable;
			if(available)
			{
				if(home)
					*home = HomePoint;
			}
			if(home_yaw)
				*home_yaw = HomeYaw;
			UnlockCtrl();
			if( available )
				return true;
			else
				return false;
		}
		return false;
	}
	bool getHomeLatLon( vector2<double>* home, double* home_yaw, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			bool available = HomeLatLonAvailable;
			if(available)
			{
				if(home)
					*home = HomeLatLon;
			}
			if(home_yaw)
				*home_yaw = HomeYaw;
			UnlockCtrl();
			if( available )
				return true;
			else
				return false;
		}
		return false;
	}
/*起飞地点*/
	
/*状态观测器*/
	//姿态ESO
	static ESO_AngularRate ESO[3];
	static ESO_AngularRateHeli ESOHeli[3];
	static ESO_AngularRate_Base* CtrlM_ESO[3];
	//高度ESO
	static double throttle_u = 0;
	static double outputThrottle = 0;
	static ESO_h ESO_height;
	static double hover_throttle = 0;
	static double WindDisturbance_x = 0;
	static double WindDisturbance_y = 0;
	static bool inFlight = false;
	static Filter_Butter4_LP AccZ_filter;
	static TD4_Lite WindDisturbance_filter[2];
	#define WindDisturbance_Filter_P 2.0
	//侧翻检测
	static bool DebugSafeEna = false;
	static uint32_t crash_counter = 0;
	static double AC_angle_error = 0;
	static double AC_rate_error = 0;
	static bool crashed = false;
	static inline void update_output_throttle( double throttle, double h )
	{
		Quaternion quat;
		get_AirframeY_quat( &quat, 0.1 );
		double lean_cosin = quat.get_lean_angle_cosin();
		
		//加速度滤波
		vector3<double> AccENU;
		get_AccelerationENU_Ctrl(&AccENU);
		double AccZ = AccZ_filter.run(AccENU.z);
		
		//观测悬停油门
		outputThrottle = throttle;
		double r_throttle = throttle;
		if( r_throttle < 0 )
			r_throttle = 0;
		if( lean_cosin < 0.1f )
				lean_cosin = 0.1f;
		r_throttle *= lean_cosin;		
		throttle_u = r_throttle;
		hover_throttle = ESO_height.get_hover_throttle();
		
		//更新电池内阻估计
		if( inFlight==false && throttle>2 ) {
			double volt = Get_MainBaterry_Voltage();
			thrVoltKpLS.doIntegral(throttle, volt);
		} else {
			thrVoltKpLS.reset();
			curVoltKpLS.reset();
		}
		
		//更新飞行状态
		static uint16_t onGround_counter = 0;
		if( inFlight == false )
		{
			onGround_counter = 0;
			if( AccZ > 20 && throttle > 2.5 ) {
				thrVoltKp = thrVoltKpLS.calcKp();
				inFlight = true;
			}
		}
		else
		{
			if( ( (hover_throttle<2) ) && lean_cosin>0 && fabs(AccZ)<15 )
			{
				if( ++onGround_counter >= 1.0 * CtrlRateHz )
					inFlight = false;
			}
			else
				onGround_counter = 0;
		}
		
		//侧翻保护
		if( inFlight && DebugSafeEna==false && AC_angle_error>degree2rad(30.0) && AC_rate_error>0.3 && throttle>0 )
		{
			if( ++crash_counter >= CtrlRateHz*2 )
			{
				crash_counter = CtrlRateHz*2;
				crashed = true;
			}
		}
		else
		{
			crashed = false;
			crash_counter = 0;
		}
		
		//观测水平分力
		if( inFlight )
		{
			vector3<double> active_force_xy_vec = quat.rotate_axis_z();
			if( lean_cosin < 0.3f )
				lean_cosin = 0.3f;
			active_force_xy_vec = active_force_xy_vec *( ( AccENU.z + GravityAcc ) / lean_cosin );
			vector3<double> WindDisturbance_xy;
			WindDisturbance_xy.x = AccENU.x - active_force_xy_vec.x;
			WindDisturbance_xy.y = AccENU.y - active_force_xy_vec.y;
			
			WindDisturbance_x = WindDisturbance_filter[0].track4( 
				WindDisturbance_xy.x, h, WindDisturbance_Filter_P,WindDisturbance_Filter_P,WindDisturbance_Filter_P,WindDisturbance_Filter_P );
			WindDisturbance_y = WindDisturbance_filter[1].track4( 
				WindDisturbance_xy.y, h, WindDisturbance_Filter_P,WindDisturbance_Filter_P,WindDisturbance_Filter_P,WindDisturbance_Filter_P );
//			double lp_factor = 2 * Pi * (1.0/CtrlRateHz) * 1.0;
//			WindDisturbance_x += lp_factor * ( WindDisturbance_xy.x - WindDisturbance_x );
//			WindDisturbance_y += lp_factor * ( WindDisturbance_xy.y - WindDisturbance_y );
		}
		else
		{
			WindDisturbance_filter[0].reset();
			WindDisturbance_filter[1].reset();
			WindDisturbance_x = WindDisturbance_y = 0;
		}
		
		//更新Home点位置
		if( inFlight == false )
		{
			vector3<double> position;
			get_Position_Ctrl(&position);
			HomeLocalZ = position.z;
			
			PosSensorHealthInf2 posInf;
			if( get_Health_XY(&posInf) )
			{
				HomeAvailable = true;
				HomePoint.x = posInf.PositionENU.x;
				HomePoint.y = posInf.PositionENU.y;
			}
			else
				HomeAvailable = false;
			
			if( get_OptimalGlobal_XY(&posInf) )
			{
				HomeLatLonAvailable = true;
				map_projection_reproject( &posInf.mp, 
					posInf.PositionENU.x+posInf.HOffset.x, 
					posInf.PositionENU.y+posInf.HOffset.y,
					&HomeLatLon.x, &HomeLatLon.y );
			}
			
			HomeYaw = quat.getYaw();
		}
		else if( get_Position_MSStatus()!= MS_Ready )
			HomeAvailable = false;
	}
	
	static double Roll_u = 0;
	static double Pitch_u = 0;
	static double Yaw_u = 0;
	void update_ESO_1()
	{
		//更新角速度观测器
		vector3<double> angular_rate;
		get_AngularRate_Ctrl(&angular_rate);
		ESO[0].run(angular_rate.x);
		ESO[1].run(angular_rate.y);
		ESO[2].run(angular_rate.z);
		ESOHeli[0].run(angular_rate.x);
		ESOHeli[1].run(angular_rate.y);
		ESOHeli[2].run(angular_rate.z);
		
		vector3<double> acc;
		get_AccelerationENU_Ctrl(&acc);
		ESO_height.run( acc.z );
	}	
	void update_ESO_2()
	{
		ESO[0].update_u(Roll_u);
		ESO[1].update_u(Pitch_u);
		ESO[2].update_u(Yaw_u);
		ESOHeli[0].update_u(Roll_u);
		ESOHeli[1].update_u(Pitch_u);
		ESOHeli[2].update_u(Yaw_u);
		ESO_height.update_u(throttle_u);
	}
/*状态观测器*/	

/*观测器接口*/
	bool get_hover_throttle( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = hover_throttle;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_throttle_force( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = ESO_height.get_force();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_es_AccZ( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = ESO_height.get_EsAcc();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_throttle_b( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = ESO_height.get_b();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_ESO_height_T( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = ESO_height.get_T();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_is_inFlight( bool* result, double TIMEOUT )
	{
		*result = inFlight;
		return true;
	}
	bool get_WindDisturbance( vector3<double>* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = vector3<double>( WindDisturbance_x, WindDisturbance_y, 0 );;
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	
	bool get_EsAngularRate( vector3<double>* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = vector3<double>( ESO[0].get_EsAngularRate(), ESO[1].get_EsAngularRate(), ESO[2].get_EsAngularRate() );
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool get_EsAngularAcc( vector3<double>* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = vector3<double>( ESO[0].get_EsAngularAcceleration(), ESO[1].get_EsAngularAcceleration(), ESO[2].get_EsAngularAcceleration() );
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool get_CrashedState(){ return crashed; }
/*观测器接口*/
	
/*控制接口*/
	//初始化计数
	static int32_t StartCounter = 0;
	#define Ctrl_initing ( StartCounter < 1e6 )
	#define set_Ctrl_inited ( StartCounter = 1e9 )
	//保护方式
	static uint8_t SafeBt = 0;
	
	//期望TD4滤波器
	static TD3_2DSL Target_tracker_RP;
	static TD4_SL Target_trackerY;
	
	//姿态控制模式
	static bool Attitude_Control_Enabled = false;
	static Attitude_ControlMode RollPitch_ControlMode = Attitude_ControlMode_Angle;
	static Attitude_ControlMode Yaw_ControlMode = Attitude_ControlMode_Angle;

	//输出滤波器
	static double outRoll_filted = 0;
	static double outPitch_filted = 0;
	static double outYaw_filted = 0;
	
	static double throttle = 0;
	static double target_Roll;
	static double target_Pitch;
	static double target_Yaw;
	static vector3<double> target_AngularRate;
	
	bool is_Attitude_Control_Enabled( bool* enabled, double TIMEOUT )
	{
		*enabled = Attitude_Control_Enabled;
		return true;
	}
	bool Attitude_Control_Enable( double TIMEOUT )
	{
		if( get_Attitude_MSStatus() != MS_Ready )
			return false;
		
		Quaternion quat;
		if( get_Airframe_quat( &quat, TIMEOUT ) == false )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == true )
			{	//控制器已打开
				UnlockCtrl();
				return false;
			}
			
			//读参数
			if( ReadParamGroup( "AttCtrl", (uint64_t*)&cfg, 0, TIMEOUT ) != PR_OK )
			{
				UnlockCtrl();
				return false;
			}
			uint8_t safe_bt[8];	
			if( ReadParam("MFunc_SafeBt", 0, 0, (uint64_t*)safe_bt, 0 ) != PR_OK )
				SafeBt = 0;
			else
				SafeBt = safe_bt[0];
			
			UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
			if( mt_count.MTCount==0 || UAVCtrlMs[cfg.UAVType[0]]==0 )
			{
				UnlockCtrl();
				return false;
			}
			set_MainMotorCount( mt_count.MTCount, mt_count.STCount, cfg.st_Freq[0] );
			CtrlM = *UAVCtrlMs[cfg.UAVType[0]];
			CtrlM_available = true;
			StartCounter = 0;
			
			/*初始化*/				
				//读取电池电压
				float BatVoltage = get_MainBatteryVoltage_filted();
				//读取基准电压
				float STVoltage[2] = {0};
				ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 );
				//计算增益修正系数
				double b_scale = 1.0;
				if( STVoltage[0]>7 && BatVoltage>7 )
					b_scale = BatVoltage / STVoltage[0];
				
				//初始化高度ESO
				ESO_height.init( cfg.T[0], cfg.beta_h[0], cfg.beta_hAcc[0], CtrlRateHz*CtrlRateDiv );
			
				//初始化期望TD4滤波器
				Target_tracker_RP.P1=cfg.TD4_P1[0];
				Target_tracker_RP.P2=cfg.TD4_P2[0];
				Target_tracker_RP.P3=cfg.TD4_P3[0];
				Target_tracker_RP.r2=degree2rad(cfg.maxRPSp[0]);
				Target_tracker_RP.r3=degree2rad(cfg.maxRPAcc[0]);
				Target_tracker_RP.r4=degree2rad(100000.0);		
				
				Target_trackerY.P1=cfg.TD4_P1[4];
				Target_trackerY.P2=cfg.TD4_P2[4];
				Target_trackerY.P3=cfg.TD4_P3[4];
				Target_trackerY.P4=cfg.TD4_P4[4];
				Target_trackerY.r2n=Target_trackerY.r2p=degree2rad(cfg.maxYSp[0]);
				Target_trackerY.r3n=Target_trackerY.r3p=degree2rad(cfg.maxYAcc[0]);
				
				CtrlM.Init();
				switch( CtrlM.RP_eso_type )
				{
					case 0:
					{	//二阶
						CtrlM_ESO[0] = &ESO[0];
						CtrlM_ESO[1] = &ESO[1];
						break;
					}
					case 1:
					{	//二阶
						CtrlM_ESO[0] = &ESOHeli[0];
						CtrlM_ESO[1] = &ESOHeli[1];
						break;
					}
				}
				switch( CtrlM.Y_eso_type )
				{
					case 0:
					{	//二阶
						CtrlM_ESO[2] = &ESO[2];
						break;
					}
					case 1:
					{	//二阶
						CtrlM_ESO[2] = &ESOHeli[2];
						break;
					}
				}
			/*初始化*/
			
			
			Attitude_Control_Enabled = true;
			target_Yaw = quat.getYaw();
			target_Roll = target_Pitch = 0;
			RollPitch_ControlMode = Attitude_ControlMode_Angle;
			Yaw_ControlMode = Attitude_ControlMode_Angle;
			
			//更新控制时间
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if(!isMSafe)
				last_ZCtrlTime = last_XYCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_Disable( double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			Altitude_Control_Disable();
			Position_Control_Disable();
			Attitude_Control_Enabled = false;			
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool get_Target_Throttle( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = throttle;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_OutputThrottle( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = outputThrottle;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Throttle( double thr, double TIMEOUT )
	{
		if( !isvalid(thr) )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			bool alt_enabled;
			is_Altitude_Control_Enabled(&alt_enabled);
			bool isAtCtrl = (xTaskGetCurrentTaskHandle()==ControlTaskHandle);
			if( alt_enabled && !isAtCtrl )
			{	//位置控制器起作用时不允许用户控制
				UnlockCtrl();
				return false;
			}
			
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe && alt_enabled==false && ForceMSafeCtrl )
			{	//屏蔽用户控制
				last_ZCtrlTime = TIME::now();
				UnlockCtrl();
				return false;
			}
			
			throttle = thr;
			
			//更新控制时间			
			if(!isMSafe && alt_enabled==false)
				last_ZCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool Attitude_Control_get_Target_RollPitch( double* Roll, double* Pitch, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			*Roll = target_Roll;
			*Pitch = target_Pitch;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_RollPitch( double Roll, double Pitch, double TIMEOUT )
	{
		if( !isvalid(Roll) || !isvalid(Pitch) )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			bool pos_enabled;
			is_Position_Control_Enabled(&pos_enabled);
			bool isAtCtrl = (xTaskGetCurrentTaskHandle()==ControlTaskHandle);
			if( pos_enabled && !isAtCtrl )
			{	//位置控制器起作用时不允许用户控制
				UnlockCtrl();
				return false;
			}
			
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe && pos_enabled==false && ForceMSafeCtrl )
			{	//屏蔽用户控制
				last_XYCtrlTime = TIME::now();
				UnlockCtrl();
				return false;
			}
			
			double angle = safe_sqrt( Roll*Roll + Pitch*Pitch );
			if( angle > degree2rad(cfg.maxLean[0]) )
			{
				double scale = degree2rad(cfg.maxLean[0]) / angle;
				Roll *= scale;
				Pitch *= scale;
			}		
			target_Roll = Roll;
			target_Pitch = Pitch;
			RollPitch_ControlMode = Attitude_ControlMode_Angle;
			
			//更新控制时间
			if(!isMSafe && pos_enabled==false)
				last_XYCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool Attitude_Control_get_TargetYaw( double* TargetYaw, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			*TargetYaw = target_Yaw;
			
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool Attitude_Control_get_TargetTrackYaw( double* TargetYaw, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			*TargetYaw = Target_trackerY.x1;
			
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool Attitude_Control_get_YawTrackErr( double* YawErr, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			*YawErr = fabs(Target_trackerY.x1 - target_Yaw);
			
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool Attitude_Control_set_Target_Yaw( double Yaw, double TIMEOUT )
	{		
		if( !isvalid(Yaw) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;	
		
		Quaternion quat, quatY;
		get_Airframe_quat(&quat);
		get_AirframeY_quat(&quatY);
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			Position_ControlMode pos_mode;
			get_Position_ControlMode(&pos_mode);
			bool isAtCtrl = (xTaskGetCurrentTaskHandle()==ControlTaskHandle);
			if( Is_YawAutoMode(pos_mode) && !isAtCtrl )
			{	//位置控制器起作用时不允许用户控制
				UnlockCtrl();
				return false;
			}
			
			if( Yaw_ControlMode != Attitude_ControlMode_Angle )
			{
				Target_trackerY.x1 = quat.getYaw();
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			
			double yaw_err = Mod( Yaw - quatY.getYaw(), 2*Pi );
			if(yaw_err > Pi)
				yaw_err -= 2*Pi;
			while(yaw_err < -Pi)
				yaw_err += 2*Pi;
			target_Yaw = Target_trackerY.x1 + yaw_err;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_YawRelative( double Yaw, double TIMEOUT )
	{		
		if( !isvalid(Yaw) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		
		Quaternion quat;
		get_Airframe_quat(&quat);
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			Position_ControlMode pos_mode;
			get_Position_ControlMode(&pos_mode);
			bool isAtCtrl = (xTaskGetCurrentTaskHandle()==ControlTaskHandle);
			if( Is_YawAutoMode(pos_mode) && !isAtCtrl )
			{	//位置控制器起作用时不允许用户控制
				UnlockCtrl();
				return false;
			}
			
			double currentYaw = quat.getYaw();
			if( Yaw_ControlMode != Attitude_ControlMode_Angle )
			{
				Target_trackerY.x1 = currentYaw;
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			target_Yaw = Target_trackerY.x1 + Yaw;
		
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_Yaw_Offboard( double Yaw, double YawRate, double TIMEOUT )
	{		
		if( !isvalid(Yaw) || !isvalid(YawRate) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;	
		
		Quaternion quat, quatY;
		get_Airframe_quat(&quat);
		get_AirframeY_quat(&quatY);
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			Position_ControlMode pos_mode;
			get_Position_ControlMode(&pos_mode);
			bool isAtCtrl = (xTaskGetCurrentTaskHandle()==ControlTaskHandle);
			if( Is_YawAutoMode(pos_mode) && !isAtCtrl )
			{	//位置控制器起作用时不允许用户控制
				UnlockCtrl();
				return false;
			}
			
			if( Yaw_ControlMode!=Attitude_ControlMode_Angle && Yaw_ControlMode!=Attitude_ControlMode_OffBoard )
				Target_trackerY.x1 = quat.getYaw();
						
			double yaw_err = Mod( Yaw - quatY.getYaw(), 2*Pi );
			if(yaw_err > Pi)
				yaw_err -= 2*Pi;
			while(yaw_err < -Pi)
				yaw_err += 2*Pi;
			target_Yaw = Target_trackerY.x1 + yaw_err;
			target_AngularRate.z = YawRate;
			Yaw_ControlMode = Attitude_ControlMode_OffBoard;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_YawRelative_Offboard( double Yaw, double YawRate, double TIMEOUT )
	{		
		if( !isvalid(Yaw) || !isvalid(YawRate) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;	
		
		Quaternion quat, quatY;
		get_Airframe_quat(&quat);
		get_AirframeY_quat(&quatY);
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			Position_ControlMode pos_mode;
			get_Position_ControlMode(&pos_mode);
			bool isAtCtrl = (xTaskGetCurrentTaskHandle()==ControlTaskHandle);
			if( Is_YawAutoMode(pos_mode) && !isAtCtrl )
			{	//位置控制器起作用时不允许用户控制
				UnlockCtrl();
				return false;
			}
			
			double currentYaw = quat.getYaw();
			if( Yaw_ControlMode!=Attitude_ControlMode_Angle && Yaw_ControlMode!=Attitude_ControlMode_OffBoard )
			{
				Target_trackerY.x1 = currentYaw;
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			target_Yaw = Target_trackerY.x1 + Yaw;
			target_AngularRate.z = YawRate;
			Yaw_ControlMode = Attitude_ControlMode_OffBoard;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_YawRate( double YawRate, double TIMEOUT )
	{
		if( !isvalid(YawRate) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			Position_ControlMode pos_mode;
			get_Position_ControlMode(&pos_mode);
			bool isAtCtrl = (xTaskGetCurrentTaskHandle()==ControlTaskHandle);
			if( Is_YawAutoMode(pos_mode) && !isAtCtrl )
			{	//位置控制器起作用时不允许用户控制
				UnlockCtrl();
				return false;
			}
			
			target_AngularRate.z = YawRate;
			Yaw_ControlMode = Attitude_ControlMode_AngularRate;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_YawLock( double TIMEOUT )
	{
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			Position_ControlMode pos_mode;
			get_Position_ControlMode(&pos_mode);
			bool isAtCtrl = (xTaskGetCurrentTaskHandle()==ControlTaskHandle);
			if( Is_YawAutoMode(pos_mode) && !isAtCtrl )
			{	//位置控制器起作用时不允许用户控制
				UnlockCtrl();
				return false;
			}
			
			if( Yaw_ControlMode == Attitude_ControlMode_AngularRate )
				Yaw_ControlMode = Attitude_ControlMode_Locking;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
/*控制接口*/
	
/*机型控制方式*/
	
	//PWM映射
	static inline void ST_PWMMap( double pwm_out[], uint8_t start, uint8_t STmotors )
	{
		float* st_mins = (float*)&cfg.st1min[0];
		float* st_maxs = (float*)&cfg.st1max[0];
		for( uint8_t i = start; i < start+STmotors; ++i )
		{
			float st_min = st_mins[i*4];
			float st_max = st_maxs[i*4];
			float scale = st_max - st_min;
			pwm_out[i] = pwm_out[i]/100.0*scale + st_min;
			pwm_out[i] = (pwm_out[i] - 1000)/1000.0*100;
		}
	}
	
	//电机非线性输出 线性修正
	static inline void throttle_nonlinear_compensation( uint8_t mt_count, double out[12] )
	{
		double output_minimum_throttle = cfg.STThrottle[0];
		double output_range = 100.0f - output_minimum_throttle;
		double inv_output_range = 1.0 / output_range;
		
		//a：非线性因子(0-1)
		//m：最大油门比例(0.6-1)
		
		//设油门-力曲线方程为：
		//F = kx^2 + (1-a)x ( 0<=x<=m F最大值为1 )
		//x = m时：km^2 + (1-a)m = 1
		//得k = ( 1 - (1-a)m ) / m^2
		//a_1 = a - 1
		//Hk  = 1 / 2k
		//K4  = 4* k
		//解方程组：kx^2 + (1-a)x = out
		//得到的x即为线性化后的输出
		double _lift_max = cfg.FullThrRatio[0];
		double a_1 = cfg.NonlinearFactor[0] - 1;
		double k = ( 1 + a_1*_lift_max ) / (_lift_max*_lift_max);
		double Hk = 1.0f / (2*k);
		double K4 = 4 * k;
			
		for( uint8_t i = 0; i < mt_count; ++i )
		{
			if( out[i] > output_minimum_throttle - 0.1f )
			{
				out[i] -= output_minimum_throttle;
				out[i] *= inv_output_range;
				if( out[i] < 0 )
					out[i] = 0;
				out[i] = Hk*( a_1 + safe_sqrt( a_1*a_1 + K4*out[i] ) );
				out[i] *= output_range;
				out[i] += output_minimum_throttle;			
			}
			else
				out[i] = 0;
		}
	}
	
	/*多旋翼*/
	
		static inline double MultiRotor_percent_to_throttle( double percent , double output_minimum_throttle, double output_range )
		{
			return percent/100.0*output_range + output_minimum_throttle;
		}
		static inline double MultiRotor_throttle_to_percent( double throttle , double output_minimum_throttle, double output_range )
		{
			return (throttle - output_minimum_throttle) / output_range*100;
		}
	
		static void MultiRotor_Init()
		{
			//初始化姿态ESO
			ESO[0].init( cfg.T[0], cfg.b[0], cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESO[1].init( cfg.T[0], cfg.b[2], cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESO[2].init( 1.0/(CtrlRateHz*CtrlRateDiv), cfg.b[4], cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
		}
		
		static void MultiRotor_PreArmControl( Receiver rc )
		{
			MainMotor_PullDownAll();
		}
			
		static void MultiRotor_InitControl( bool DbgSafe )
		{
			if( DbgSafe )
			{
				MainMotor_PullDownAll();
				StartCounter = 0;
			}
			
			//启动初始化
			if( StartCounter < 1e6 )
			{	//还未进行启动初始化
				if( StartCounter >= 0 )
				{	//启动动作
					switch(cfg.STMode[0])
					{	//顺序起转电机
						case 1:
						{	
							UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
							if( StartCounter < 0.3*mt_count.MTCount * CtrlRateHz )
							{
								double pwm_out[8] = {0};
								for( uint8_t i = 0; i < mt_count.MTCount; ++i )
								{
									if( StartCounter > 0.3*i * CtrlRateHz )
										pwm_out[i] = cfg.STThrottle[0];
									else
										break;
								}
								MainMotor_PWM_Out( pwm_out );
								++StartCounter;
							}
							else
								StartCounter = -1;
							break;
						}
						
						default:
						{
							double pwm_out[8] = {0};
							UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
							for( uint8_t i = 0; i < mt_count.MTCount; ++i )
								pwm_out[i] = cfg.STThrottle[0];
							MainMotor_PWM_Out( pwm_out );
							StartCounter = -1;
							break;
						}
					}
				}
				else
				{	//启动延时
					if( (-StartCounter) > cfg.STDelay[0]*CtrlRateHz )
						StartCounter = 1e9;
					else
						--StartCounter;
				}
				return;
			}
		}
	
		static void MultiRotor_MotorControl( uint8_t mt_count, double output_throttle, double outRoll, double outPitch, double outYaw, double rp_out[], double yaw_out[], bool DbgSafe )
		{
			//读取电池电压
			float BatVoltage = get_MainBatteryVoltage_filted();
			//计算增益修正系数
			float STVoltage[2] = {0};
			ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 );
			//计算增益修正系数
			double bat_b_scale = 1.0;
			if( STVoltage[0]>7 && BatVoltage>7 )
				bat_b_scale = constrain( BatVoltage / STVoltage[0], 0.35f, 3.0f );
			
			//海拔推力补偿
			double baro_b_scale = 1.0;
			extern float internal_barometer_temperature;
			double tempK = internal_barometer_temperature + C_TO_KELVIN;
			if( tempK > 100 )
			{
				extern float internal_barometer_pressure;
				extern float internal_barometer_altitude;	
				double eas2tas_squared = SSL_AIR_DENSITY / (internal_barometer_pressure / (ISA_GAS_CONSTANT * tempK) );
				if( eas2tas_squared > 0.7 )
					baro_b_scale = 1.0 / eas2tas_squared;
			}
			
			//根据电池电压调整控制对象增益
			double b_scale = bat_b_scale * baro_b_scale;
			ESO[0].b = cfg.b[0] * b_scale;
			ESO[1].b = cfg.b[2] * b_scale;
			ESO[2].b = cfg.b[4] * b_scale;
			
			double rotor_output[12];
			double output_minimum_throttle = cfg.STThrottle[0];	
			
			if( output_throttle<0 || DbgSafe )
			{
				MainMotor_PullDownAll();
				update_output_throttle( 0 , 1.0/CtrlRateHz );
				return;
			}		
			
			double output_range = 100.0 - output_minimum_throttle;
			double output_midpoint = output_range / 2;
			
			//油门百分比转换为实际油门量
			output_throttle = MultiRotor_percent_to_throttle( output_throttle, output_minimum_throttle, output_range );
			
			/*pitch roll 输出限幅*/
				//如果需要的pitch roll输出超出当前油门能提供的输出范围
				//调整油门获得尽量满足pitch roll输出
				double output_max = fabs(rp_out[0]);
				for( uint8_t i = 1 ; i < mt_count ; ++i )
				{
					double abs_out = fabs(rp_out[i]);
					if( abs_out > output_max ) 
						output_max = abs_out;
				}
				
				double max_allow_output = 100.0f - output_throttle;
				double min_allow_output = output_minimum_throttle - output_throttle;			
				double allow_ouput_range;
				if( max_allow_output < -min_allow_output )
				{	//降低油门确保姿态输出
					allow_ouput_range = max_allow_output;
					if( output_max > allow_ouput_range )
					{	//需要降低油门
						if( output_max > output_midpoint )
						{	//输出超过最大输出范围
							//将油门调整为50%确保可以进行最大输出
							output_throttle = output_midpoint + output_minimum_throttle;
							allow_ouput_range = output_midpoint;
						}
						else
						{	//降低油门到所需值
							output_throttle = 100.0f - output_max;
							allow_ouput_range = output_max;
						}
					}
				}
				else
				{	//抬高油门保证姿态输出
					allow_ouput_range = -min_allow_output;
					if( output_max > allow_ouput_range )
					{	//需要抬高油门
						
						//求最高允许的油门值（不能大于悬停油门）
						double hover_throttle_force = MultiRotor_percent_to_throttle( hover_throttle, output_minimum_throttle, output_range ) - output_minimum_throttle;
						double max_allowed_output_range = hover_throttle_force*0.85;
						if( output_midpoint < max_allowed_output_range )
							max_allowed_output_range = output_midpoint;				
						if( max_allowed_output_range < output_throttle - output_minimum_throttle )
							max_allowed_output_range = output_throttle - output_minimum_throttle;
						double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
						
						if( output_max > max_allowed_output_range )
						{	//输出范围大于最大允许范围
							//抬高油门至最大允许范围
							output_throttle = max_allowed_throttle;
							allow_ouput_range = max_allowed_output_range;
						}
						else
						{	//抬高油门到所需值
							output_throttle = output_minimum_throttle + output_max;
							allow_ouput_range = output_max;
						}
					}
				}
				
				//输出限幅修正
				if( output_max > allow_ouput_range )
				{	//需要修正输出
					double scale  = allow_ouput_range / output_max;
					for( uint8_t i = 0 ; i < mt_count ; ++i )
						rotor_output[i] = rp_out[i] * scale;
					Roll_u = outRoll * scale;
					Pitch_u = outPitch * scale;
				}		
				else
				{
					for( uint8_t i = 0 ; i < mt_count ; ++i )
						rotor_output[i] = rp_out[i];
					Roll_u = outRoll;
					Pitch_u = outPitch;
				}			
			/*pitch roll 输出限幅*/
			
			/*yaw output 输出限幅*/
				//抬升油门保住偏航输出
				
				//计算偏航输出到上下界距离
				double yaw_out_up=100, yaw_out_dn=100;
				for( uint8_t i = 0 ; i < mt_count ; ++i )
				{
					double current_rotor_output = output_throttle + rotor_output[i];
					max_allow_output = 100.0f - current_rotor_output;
					min_allow_output = current_rotor_output - output_minimum_throttle;
								
					if( yaw_out[i] > 0 )
					{	//更新距离上界距离
						double out_up = max_allow_output - yaw_out[i];
						if( out_up < yaw_out_up )
							yaw_out_up = out_up;
					}
					else
					{	//更新距离下界距离
						double out_dn = min_allow_output - -yaw_out[i];
						if( out_dn < yaw_out_dn )
							yaw_out_dn = out_dn;
					}
				}
				
				if( yaw_out_dn < 0 )
				{	//偏航输出超出下界
					//抬升油门保住偏航输出
					
					//求最高允许的油门值（不能大于悬停油门）
					double hover_throttle_force = MultiRotor_percent_to_throttle( hover_throttle, output_minimum_throttle, output_range ) - output_minimum_throttle;
					double max_allowed_output_range = hover_throttle_force*0.85;
					if( output_midpoint < max_allowed_output_range )
						max_allowed_output_range = output_midpoint;				
					if( max_allowed_output_range < output_throttle - output_minimum_throttle )
						max_allowed_output_range = output_throttle - output_minimum_throttle;
					double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
					
					//期望抬升的油门量
					double req_up = -yaw_out_dn;
					//抬升油门量不能使输出超出上界
					if( req_up > yaw_out_up )
						req_up = yaw_out_up;
					if( req_up < 0 )
						req_up = 0;
					
					//抬升油门
					output_throttle += req_up;
					if( output_throttle > max_allowed_throttle )
						output_throttle = max_allowed_throttle;
				}
				else if( cfg.YawPri[0]>0.1 && yaw_out_up<0 )
				{	//降低油门确保姿态输出
					
					//求最多允许降低的油门值
					double max_dn = 0;
					max_dn = cfg.YawPri[0];
					//期望降低的油门量
					double req_dn = -yaw_out_up;
					//降低油门量不能使输出超出下界
					if( req_dn > max_dn )
						req_dn = max_dn;
					if( req_dn > yaw_out_dn )
						req_dn = yaw_out_dn;
					if( req_dn < 0 )
						req_dn = 0;
					
					//降低油门
					output_throttle -= req_dn;
				}

				/*yaw输出限幅计算*/
					double yaw_scale = 1.0;		
					for( uint8_t i = 0 ; i < mt_count ; ++i )
					{
						double current_rotor_output = output_throttle + rotor_output[i];
						max_allow_output = 100.0f - current_rotor_output;
						min_allow_output = output_minimum_throttle - current_rotor_output;
						if( yaw_out[i] > max_allow_output + 0.01f )
						{
							double new_yaw_scale = max_allow_output / yaw_out[i];
							if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
						}
						else if( yaw_out[i] < min_allow_output - 0.01f )
						{
							double new_yaw_scale = min_allow_output / yaw_out[i];
							if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
						}
					}
				/*yaw输出限幅计算*/
				
				//lower yaw output to ensure attitude control and alt control
				if( yaw_scale < 0 )
					yaw_scale = 0;
				outYaw *= yaw_scale;
				Yaw_u = outYaw;
			/*yaw output 输出限幅*/
				
			//更新油门油门观测
			double throttle_percent = (output_throttle - output_minimum_throttle) / output_range*100;
			update_output_throttle( throttle_percent , 1.0/CtrlRateHz );
			//补偿非线性输出
			for( uint8_t i = 0 ; i < mt_count ; ++i )
				rotor_output[i] += output_throttle + yaw_scale*yaw_out[i];
			throttle_nonlinear_compensation( mt_count, rotor_output );
			MainMotor_PWM_Out( rotor_output );
		}
	
		static void MultiRotor4X_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			double rp_out[4];	
			rp_out[0] = -outPitch+outRoll;
			rp_out[1] = +outPitch+outRoll;		
			rp_out[2] = +outPitch-outRoll;
			rp_out[3] = -outPitch-outRoll;
			double yaw_out[4];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			MultiRotor_MotorControl( 4, output_throttle, outRoll , outPitch , outYaw, rp_out, yaw_out, DbgSafe );
		}
		static const UAVCtrlM MultiRotor4X_CtrlM = {
			MultiRotor_Init,
			MultiRotor_PreArmControl,
			MultiRotor_InitControl,
			MultiRotor4X_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数 偏航控加速度
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
			
		static void MultiRotor4C_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			double rp_out[4];			
			rp_out[0] = -outPitch;
			rp_out[1] = +outRoll;		
			rp_out[2] = +outPitch;
			rp_out[3] = -outRoll;
			double yaw_out[4];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			MultiRotor_MotorControl( 4, output_throttle, outRoll , outPitch , outYaw, rp_out, yaw_out, DbgSafe );
		}
		static const UAVCtrlM MultiRotor4C_CtrlM = {
			MultiRotor_Init,
			MultiRotor_PreArmControl,
			MultiRotor_InitControl,
			MultiRotor4C_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数 偏航控加速度
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
		
		static void MultiRotor6X_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			double rp_out[6];
			double RollS = outRoll * 1.1547005383792515290182975610039f;
			double half_outRoll = 0.5f * RollS;			
			rp_out[0] = -outPitch+half_outRoll;
			rp_out[1] = RollS;
			rp_out[2] = +outPitch+half_outRoll;
			rp_out[3] = +outPitch-half_outRoll;
			rp_out[4] = -RollS;
			rp_out[5] = -outPitch-half_outRoll;
			double yaw_out[6];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = -outYaw;
			yaw_out[5] = +outYaw;
			MultiRotor_MotorControl( 6, output_throttle, outRoll , outPitch , outYaw, rp_out, yaw_out, DbgSafe );
		}
		static const UAVCtrlM MultiRotor6X_CtrlM = {
			MultiRotor_Init,
			MultiRotor_PreArmControl,
			MultiRotor_InitControl,
			MultiRotor6X_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数 偏航控加速度
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
		
		static void MultiRotor6C_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			double rp_out[6];
			double PitchS = outPitch * 1.1547005383792515290182975610039f;
			double half_outPitch = 0.5f * PitchS;			
			rp_out[0] = -PitchS;
			rp_out[1] = -half_outPitch+outRoll;
			rp_out[2] = +half_outPitch+outRoll;
			rp_out[3] = +PitchS;
			rp_out[4] = +half_outPitch-outRoll;
			rp_out[5] = -half_outPitch-outRoll;
			double yaw_out[6];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = -outYaw;
			yaw_out[5] = +outYaw;
			MultiRotor_MotorControl( 6, output_throttle, outRoll , outPitch , outYaw, rp_out, yaw_out, DbgSafe );
		}
		static const UAVCtrlM MultiRotor6C_CtrlM = {
			MultiRotor_Init,
			MultiRotor_PreArmControl,
			MultiRotor_InitControl,
			MultiRotor6C_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数 偏航控加速度
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
		
		static void MultiRotor62X_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			double RollS = outRoll * 1.1547005383792515290182975610039f;
			double half_outRoll = 0.5f * RollS;
			double rp_out[12];   
			rp_out[0] = -outPitch+half_outRoll;
			rp_out[1] = RollS;
			rp_out[2] = +outPitch+half_outRoll;
			rp_out[3] = +outPitch-half_outRoll;
			rp_out[4] = -RollS;
			rp_out[5] = -outPitch-half_outRoll;
			rp_out[6] = -outPitch+half_outRoll;
			rp_out[7] = RollS;
			rp_out[8] = +outPitch+half_outRoll;
			rp_out[9] = +outPitch-half_outRoll;
			rp_out[10] = -RollS;
			rp_out[11] = -outPitch-half_outRoll;
			double yaw_out[12];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = -outYaw;
			yaw_out[5] = +outYaw;
			yaw_out[6] = +outYaw;
			yaw_out[7] = -outYaw;
			yaw_out[8] = +outYaw;
			yaw_out[9] = -outYaw;
			yaw_out[10] = +outYaw;
			yaw_out[11] = -outYaw;
			MultiRotor_MotorControl( 12, output_throttle, outRoll , outPitch , outYaw, rp_out, yaw_out, DbgSafe );
		}
		static const UAVCtrlM MultiRotor62X_CtrlM = {
			MultiRotor_Init,
			MultiRotor_PreArmControl,
			MultiRotor_InitControl,
			MultiRotor62X_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数 偏航控加速度
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
		
		static void MultiRotor8X_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			double rp_out[8];		
			rp_out[0] = -outPitch+outRoll;
			rp_out[1] = -outPitch+outRoll;
			rp_out[2] = +outPitch+outRoll;
			rp_out[3] = +outPitch+outRoll;
			rp_out[4] = +outPitch-outRoll;
			rp_out[5] = +outPitch-outRoll;
			rp_out[6] = -outPitch-outRoll;
			rp_out[7] = -outPitch-outRoll;
			double yaw_out[8];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = -outYaw;
			yaw_out[5] = +outYaw;
			yaw_out[6] = -outYaw;
			yaw_out[7] = +outYaw;
			MultiRotor_MotorControl( 8, output_throttle, outRoll , outPitch , outYaw, rp_out, yaw_out, DbgSafe );
		}
		static const UAVCtrlM MultiRotor8X_CtrlM = {
			MultiRotor_Init,
			MultiRotor_PreArmControl,
			MultiRotor_InitControl,
			MultiRotor8X_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数 偏航控加速度
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
		
		static void MultiRotor6S1_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			double rp_out[6];			
			double RollS = outRoll * 1.732;
			double half_outRoll = 0.5f * RollS;
			rp_out[0] = -outPitch+half_outRoll;
			rp_out[1] = RollS;
			rp_out[2] = +outPitch+half_outRoll;
			rp_out[3] = +outPitch-half_outRoll;
			rp_out[4] = -RollS;
			rp_out[5] = -outPitch-half_outRoll;
			double yaw_out[6];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = -outYaw;
			yaw_out[5] = +outYaw;
			MultiRotor_MotorControl( 6, output_throttle, outRoll , outPitch , outYaw, rp_out, yaw_out, DbgSafe );
		}
		static const UAVCtrlM MultiRotor6S1_CtrlM = {
			MultiRotor_Init,
			MultiRotor_PreArmControl,
			MultiRotor_InitControl,
			MultiRotor6S1_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数 偏航控加速度
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
		
		static void MultiRotor42X_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			double rp_out[8];   
			rp_out[0] = -outPitch+outRoll;
			rp_out[1] = +outPitch+outRoll;  
			rp_out[2] = +outPitch-outRoll;
			rp_out[3] = -outPitch-outRoll;
			rp_out[4] = -outPitch+outRoll;
			rp_out[5] = +outPitch+outRoll;  
			rp_out[6] = +outPitch-outRoll;
			rp_out[7] = -outPitch-outRoll;
			double yaw_out[8];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = +outYaw;
			yaw_out[5] = -outYaw;
			yaw_out[6] = +outYaw;
			yaw_out[7] = -outYaw;
			MultiRotor_MotorControl( 8, output_throttle, outRoll , outPitch , outYaw, rp_out, yaw_out, DbgSafe );
		}
		static const UAVCtrlM MultiRotor42X_CtrlM = {
			MultiRotor_Init,
			MultiRotor_PreArmControl,
			MultiRotor_InitControl,
			MultiRotor42X_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数 偏航控加速度
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
	/*多旋翼*/
		
	/*三轴*/
		static void TriRotor_Init()
		{
			//初始化姿态ESO
			ESO[0].init( cfg.T[0], cfg.b[0], cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESO[1].init( cfg.T[0], cfg.b[2], cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESO[2].init( cfg.T2[0], cfg.b[4], cfg.beta2[0], cfg.beta2[0], CtrlRateHz*CtrlRateDiv );
		}
		
		static void TriRotor_PreArmControl( Receiver rc )
		{
			double pwmout[6];
			pwmout[0] = pwmout[1] = pwmout[2] = pwmout[3] = 0;
			pwmout[4] = pwmout[5] = 100-rc.data[1];
			ST_PWMMap( pwmout, 4, 2 );
			MainMotor_PWM_Out( pwmout );
			MainMotor_PullDownAll();
		}
			
		static void TriRotor_InitControl( bool DbgSafe )
		{
			if( DbgSafe )
			{
				MainMotor_PullDownAll();
				StartCounter = 0;
			}
			
			//启动初始化
			if( StartCounter < 1e6 )
			{	//还未进行启动初始化
				if( StartCounter >= 0 )
				{	//启动动作
					switch(cfg.STMode[0])
					{	//顺序起转电机
						case 1:
						{	
							if( StartCounter < 0.3*3 * CtrlRateHz )
							{
								double pwm_out[8] = {0};
								for( uint8_t i = 0; i < 3; ++i )
								{
									if( StartCounter > 0.3*i * CtrlRateHz )
										pwm_out[i] = cfg.STThrottle[0];
									else
										break;
								}
								MainMotor_PWM_Out( pwm_out );
								++StartCounter;
							}
							else
								StartCounter = -1;
							break;
						}
						
						default:
						{
							double pwm_out[8] = {0};
							for( uint8_t i = 0; i < 3; ++i )
								pwm_out[i] = cfg.STThrottle[0];
							MainMotor_PWM_Out( pwm_out );
							StartCounter = -1;
							break;
						}
					}
				}
				else
				{	//启动延时
					if( (-StartCounter) > cfg.STDelay[0]*CtrlRateHz )
						StartCounter = 1e9;
					else
						--StartCounter;
				}
				return;
			}
		}
	
		static void TriRotorX_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			//读取电池电压
			float BatVoltage = get_MainBatteryVoltage_filted();
			//计算增益修正系数
			float STVoltage[2] = {0};
			ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 );
			//计算增益修正系数
			double bat_b_scale = 1.0;
			if( STVoltage[0]>7 && BatVoltage>7 )
				bat_b_scale = constrain( BatVoltage / STVoltage[0], 0.35f, 3.0f );
			
			//海拔推力补偿
			double baro_b_scale = 1.0;
			extern float internal_barometer_temperature;
			double tempK = internal_barometer_temperature + C_TO_KELVIN;
			if( tempK > 100 )
			{
				extern float internal_barometer_pressure;
				extern float internal_barometer_altitude;	
				double eas2tas_squared = SSL_AIR_DENSITY / (internal_barometer_pressure / (ISA_GAS_CONSTANT * tempK) );
				if( eas2tas_squared > 0.7 )
					baro_b_scale = 1.0 / eas2tas_squared;
			}
			
			//根据电池电压调整控制对象增益
			double b_scale = bat_b_scale * baro_b_scale;
			ESO[0].b = cfg.b[0] * b_scale;
			ESO[1].b = cfg.b[2] * b_scale;
			ESO[2].b = cfg.b[4] * b_scale;
			
			double rp_out[3];   
			rp_out[0] = -0.5*outPitch+outRoll;
			rp_out[1] = +outPitch;  
			rp_out[2] = -0.5*outPitch-outRoll;
			
			double rotor_output[12];
			double output_minimum_throttle = cfg.STThrottle[0];	
			
			if( output_throttle<0 || DbgSafe )
			{
				MainMotor_PullDownAll();
				update_output_throttle( 0 , 1.0/CtrlRateHz );
				return;
			}		
			
			double output_range = 100.0 - output_minimum_throttle;
			double output_midpoint = output_range / 2;
			
			//油门百分比转换为实际油门量
			output_throttle = MultiRotor_percent_to_throttle( output_throttle, output_minimum_throttle, output_range );
			
			/*pitch roll 输出限幅*/
				//如果需要的pitch roll输出超出当前油门能提供的输出范围
				//调整油门获得尽量满足pitch roll输出
				double output_max = fabs(rp_out[0]);
				for( uint8_t i = 1 ; i < 3 ; ++i )
				{
					double abs_out = fabs(rp_out[i]);
					if( abs_out > output_max ) 
						output_max = abs_out;
				}
				
				double max_allow_output = 100.0f - output_throttle;
				double min_allow_output = output_minimum_throttle - output_throttle;			
				double allow_ouput_range;
				if( max_allow_output < -min_allow_output )
				{	//降低油门确保姿态输出
					allow_ouput_range = max_allow_output;
					if( output_max > allow_ouput_range )
					{	//需要降低油门
						if( output_max > output_midpoint )
						{	//输出超过最大输出范围
							//将油门调整为50%确保可以进行最大输出
							output_throttle = output_midpoint + output_minimum_throttle;
							allow_ouput_range = output_midpoint;
						}
						else
						{	//降低油门到所需值
							output_throttle = 100.0f - output_max;
							allow_ouput_range = output_max;
						}
					}
				}
				else
				{	//抬高油门保证姿态输出
					allow_ouput_range = -min_allow_output;
					if( output_max > allow_ouput_range )
					{	//需要抬高油门
						
						//求最高允许的油门值（不能大于悬停油门）
						double hover_throttle_force = MultiRotor_percent_to_throttle( hover_throttle, output_minimum_throttle, output_range ) - output_minimum_throttle;
						double max_allowed_output_range = hover_throttle_force*0.85;
						if( output_midpoint < max_allowed_output_range )
							max_allowed_output_range = output_midpoint;				
						if( max_allowed_output_range < output_throttle - output_minimum_throttle )
							max_allowed_output_range = output_throttle - output_minimum_throttle;
						double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
						
						if( output_max > max_allowed_output_range )
						{	//输出范围大于最大允许范围
							//抬高油门至最大允许范围
							output_throttle = max_allowed_throttle;
							allow_ouput_range = max_allowed_output_range;
						}
						else
						{	//抬高油门到所需值
							output_throttle = output_minimum_throttle + output_max;
							allow_ouput_range = output_max;
						}
					}
				}
				
				//输出限幅修正
				if( output_max > allow_ouput_range )
				{	//需要修正输出
					double scale  = allow_ouput_range / output_max;
					for( uint8_t i = 0 ; i < 3 ; ++i )
						rotor_output[i] = rp_out[i] * scale;
					Roll_u = outRoll * scale;
					Pitch_u = outPitch * scale;
				}		
				else
				{
					for( uint8_t i = 0 ; i < 3 ; ++i )
						rotor_output[i] = rp_out[i];
					Roll_u = outRoll;
					Pitch_u = outPitch;
				}			
			/*pitch roll 输出限幅*/
			
			rotor_output[3] = 0;
			/*yaw输出限幅*/
				double absYaw = fabs(outYaw);
				if( absYaw > 50 )
					outYaw *= 50.0 / absYaw;
				rotor_output[4] = rotor_output[5] = outYaw + 50;
				Yaw_u = outYaw;
			/*yaw输出限幅*/
				
			//更新油门油门观测
			double throttle_percent = (output_throttle - output_minimum_throttle) / output_range*100;
			update_output_throttle( throttle_percent , 1.0/CtrlRateHz );
			//补偿非线性输出
			for( uint8_t i = 0 ; i < 3 ; ++i )
				rotor_output[i] += output_throttle;
			throttle_nonlinear_compensation( 3, rotor_output );
			ST_PWMMap( rotor_output, 4, 2 );
			MainMotor_PWM_Out( rotor_output );
		}
		
		static const UAVCtrlM TriRotorX_CtrlM = {
			TriRotor_Init,
			TriRotor_PreArmControl,
			TriRotor_InitControl,
			TriRotorX_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 0,	//横滚俯仰控推力导数
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
	/*三轴*/
	
	/*直升机*/
		#define Heli_RPb_scale 0.04
		static inline double Heli131_Map( double output_throttle, double outRoll, double outPitch, double outYaw, 
			double out[], double* outRoll_u, double* outPitch_u, double* outYaw_u )
		{
			float output_minimum_throttle = 0;
			double output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
			
			if( output_throttle < 0.1f )
			{
				out[0] = out[1] = out[2] = 0;
			}		
			else
			{	//横滚俯仰动力分配
				out[0] = +outRoll - 0.5*outPitch;
				out[1] = +outPitch;
				out[2] = -outRoll - 0.5*outPitch;
				
				//如果需要的pitch roll输出超出当前油门能提供的输出范围
				//调整油门获得尽量满足pitch roll输出
				double output_max = fabs(out[0]);
				for( uint8_t i = 0 ; i < 3 ; ++i )
				{
					double abs_out = fabs(out[i]);
					if( abs_out > output_max ) 
						output_max = abs_out;
				}
				
				double max_allow_output = 100.0f - output_throttle;
				double min_allow_output = output_minimum_throttle - output_throttle;			
				double allow_ouput_range;
				if( max_allow_output < -min_allow_output )
				{	//降低油门确保姿态输出
					allow_ouput_range = max_allow_output;
					if( output_max > allow_ouput_range )
					{	//需要降低油门
						if( output_max > output_midpoint )
						{	//输出超过最大输出范围
							//将油门调整为50%确保可以进行最大输出
							output_throttle = output_midpoint + output_minimum_throttle;
							allow_ouput_range = output_midpoint;
						}
						else
						{	//降低油门到所需值
							output_throttle = 100.0f - output_max;
							allow_ouput_range = output_max;
						}
					}
				}
				else
				{	//抬高油门保证姿态输出
					allow_ouput_range = -min_allow_output;
					if( output_max > allow_ouput_range )
					{	//需要抬高油门
						
						//求最高允许的油门值（不能大于悬停油门）
						double hover_throttle_force = hover_throttle - output_minimum_throttle;
						double max_allowed_output_range = hover_throttle_force*0.85;
						if( output_midpoint < max_allowed_output_range )
							max_allowed_output_range = output_midpoint;				
						if( max_allowed_output_range < output_throttle - output_minimum_throttle )
							max_allowed_output_range = output_throttle - output_minimum_throttle;
						double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
						
						if( output_max > max_allowed_output_range )
						{	//输出范围大于最大允许范围
							//抬高油门至最大允许范围
							output_throttle = max_allowed_throttle;
							allow_ouput_range = max_allowed_output_range;
						}
						else
						{	//抬高油门到所需值
							output_throttle = output_minimum_throttle + output_max;
							allow_ouput_range = output_max;
						}
					}
				}
				
				//输出限幅修正
				if( output_max > allow_ouput_range )
				{	//需要修正输出
					double scale  = allow_ouput_range / output_max;
					for( uint8_t i = 0 ; i < 3 ; ++i )
						out[i] *= scale;
					if( outRoll_u )
						*outRoll_u = outRoll * scale;
					if( outPitch_u )
						*outPitch_u = outPitch * scale;
				}		
				else
				{
					if( outRoll_u )
						*outRoll_u = outRoll;
					if( outPitch_u )
						*outPitch_u = outPitch;
				}
				
				//加上油门
				for( uint8_t i = 0 ; i < 3 ; ++i )
					out[i] += output_throttle;
			}
			
			/*yaw output 输出限幅*/
				//抬升油门保住偏航输出
				
				/*yaw输出限幅计算*/
					double yaw_scale = 1.0;		
					double abs_yaw = fabs(outYaw);
					if( abs_yaw > 50 )
						yaw_scale = 50 / abs_yaw;
				/*yaw输出限幅计算*/
				
				//lower yaw output to ensure attitude control and alt control
				if( yaw_scale < 0 )
					yaw_scale = 0;
				outYaw *= yaw_scale;
				if( outYaw_u )
					*outYaw_u = outYaw;
				out[3] = outYaw+50;
			/*yaw output 输出限幅*/
				
			//更新油门油门观测
			return output_throttle;
		}
		
		static void Heli131_Init()
		{
			//初始化姿态ESO
			ESOHeli[0].init( cfg.T[0], cfg.b[0]*Heli_RPb_scale, cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESOHeli[1].init( cfg.T[0], cfg.b[2]*Heli_RPb_scale, cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESO[2].init( cfg.T2[0], cfg.b[4], cfg.beta2[0], cfg.beta2[0], CtrlRateHz*CtrlRateDiv );
		}
		
		static void Heli131_PreArmControl( Receiver rc )
		{
			double pwmout[6];
			//电机关闭
			pwmout[0] = pwmout[1] = 0;
			if( rc.available )
				Heli131_Map( rc.data[0], rc.data[3]-50 , rc.data[2]-50 , 50-rc.data[1], &pwmout[2], 0,0,0 );
			else
				Heli131_Map( 0, 0 , 0 , 0, &pwmout[2], 0,0,0 );
			ST_PWMMap( pwmout, 2, 4 );
			MainMotor_PWM_Out( pwmout );
			MainMotor_PullDownAll();
		}
			
		static void Heli131_InitControl( bool DbgSafe )
		{
			if( DbgSafe )
			{
				MainMotor_PullDownAll();
				StartCounter = 0;
			}
	
			++StartCounter;
			if( StartCounter > cfg.STDelay[0]*CtrlRateHz )
				set_Ctrl_inited;
		}
	
		static void Heli131_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			if( DbgSafe )
			{
				double pwmout[6];
				//电机关闭
				pwmout[0] = pwmout[1] = 0;
				Receiver rc;
				getReceiver(&rc);
				if( rc.available )
					Heli131_Map( rc.data[0], rc.data[3]-50 , rc.data[2]-50 , 50-rc.data[1], &pwmout[2], 0,0,0 );
				else
					Heli131_Map( 0, 0 , 0 , 0, &pwmout[2], 0,0,0 );
				ST_PWMMap( pwmout, 2, 4 );
				MainMotor_PWM_Out( pwmout );
				update_output_throttle( 0 , 1.0/CtrlRateHz );
				return;
			}
			
			//读取电池电压
			float BatVoltage = get_MainBatteryVoltage_filted();
			//计算增益修正系数
			float STVoltage[2] = {0};
			ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 );
			//计算增益修正系数
			double bat_b_scale = 1.0;
			if( STVoltage[0]>7 && BatVoltage>7 )
				bat_b_scale = constrain( BatVoltage / STVoltage[0], 0.35f, 3.0f );
			
			//海拔推力补偿
			double baro_b_scale = 1.0;
			extern float internal_barometer_temperature;
			double tempK = internal_barometer_temperature + C_TO_KELVIN;
			if( tempK > 100 )
			{
				extern float internal_barometer_pressure;
				extern float internal_barometer_altitude;	
				double eas2tas_squared = SSL_AIR_DENSITY / (internal_barometer_pressure / (ISA_GAS_CONSTANT * tempK) );
				if( eas2tas_squared > 0.7 )
					baro_b_scale = 1.0 / eas2tas_squared;
			}
			
			//根据电池电压调整控制对象增益
			double b_scale = bat_b_scale * baro_b_scale;
			ESOHeli[0].b = cfg.b[0] * Heli_RPb_scale*b_scale;
			ESOHeli[1].b = cfg.b[2] * Heli_RPb_scale*b_scale;
			ESO[2].b = cfg.b[4] * b_scale;
			
			double pwmout[6];
			//求电机输出
			float fly_pwm = (cfg.st1min[0] - 1000) * 0.1f;
			if( Ctrl_initing )
			{
				output_throttle = 0;
				pwmout[0] = pwmout[1] = ( (float)StartCounter / (cfg.STDelay[0]*CtrlRateHz) ) * (fly_pwm-cfg.STThrottle[0]) + cfg.STThrottle[0];
			}
			else
				pwmout[0] = pwmout[1] = fly_pwm;
			
			//计算舵机输出
			output_throttle = Heli131_Map( output_throttle, outRoll , outPitch , outYaw, &pwmout[2], 
				&Roll_u, &Pitch_u, &Yaw_u );
			ST_PWMMap( pwmout, 2, 4 );
			MainMotor_PWM_Out( pwmout );
			update_output_throttle( output_throttle , 1.0/CtrlRateHz );
		}
		
		static const UAVCtrlM Heli131_CtrlM = {
			Heli131_Init,
			Heli131_PreArmControl,
			Heli131_InitControl,
			Heli131_MotorControl,
			1, 0,	//横滚俯仰二阶eso
			5, 0,	//横滚俯仰控推力导数
			true,	//初始化时进行控制
			true,	//起飞前不添加扰动
		};
	/*直升机*/
		
	/*共轴双桨 M2S2*/
		static void CoaxialM2S2_Init()
		{
			//初始化姿态ESO
//			ESOHeli[0].init( cfg.T[0], cfg.b[0]*Heli_RPb_scale, cfg.beta[0], CtrlRateHz*CtrlRateDiv );
//			ESOHeli[1].init( cfg.T[0], cfg.b[2]*Heli_RPb_scale, cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESO[0].init( cfg.T[0], cfg.b[0], cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESO[1].init( cfg.T[0], cfg.b[2], cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
			ESO[2].init( 1.0/(CtrlRateHz*CtrlRateDiv), cfg.b[4], cfg.beta2[0], cfg.beta2[0], CtrlRateHz*CtrlRateDiv );
		}
		
		static inline void CoaxialM2S2_Map( double outRoll, double outPitch, double pwmout[4] )
		{
			
		}
		
		static void CoaxialM2S2_PreArmControl( Receiver rc )
		{
			double pwmout[4];
			pwmout[0] = pwmout[1] = 0;
			pwmout[2] = rc.data[3];
			pwmout[3] = rc.data[2];
			ST_PWMMap( pwmout, 2, 2 );
			MainMotor_PWM_Out( pwmout );
			MainMotor_PullDownAll();
		}
			
		static void CoaxialM2S2_InitControl( bool DbgSafe )
		{
			if( DbgSafe )
			{
				MainMotor_PullDownAll();
				StartCounter = 0;
			}
			
			//启动初始化
			if( StartCounter < 1e6 )
			{	//还未进行启动初始化
				if( StartCounter >= 0 )
				{	//启动动作
					switch(cfg.STMode[0])
					{	//顺序起转电机
						case 1:
						{	
							if( StartCounter < 0.3*2 * CtrlRateHz )
							{
								double pwm_out[8] = {0};
								pwm_out[2] = pwm_out[3] = 50;
								for( uint8_t i = 0; i < 2; ++i )
								{
									if( StartCounter > 0.3*i * CtrlRateHz )
										pwm_out[i] = cfg.STThrottle[0];
									else
										break;
								}
								ST_PWMMap( pwm_out, 2, 2 );
								MainMotor_PWM_Out( pwm_out );
								++StartCounter;
							}
							else
								StartCounter = -1;
							break;
						}
						
						default:
						{
							double pwm_out[8] = {0};
							pwm_out[2] = pwm_out[3] = 50;
							for( uint8_t i = 0; i < 2; ++i )
								pwm_out[i] = cfg.STThrottle[0];
							ST_PWMMap( pwm_out, 2, 2 );
							MainMotor_PWM_Out( pwm_out );
							StartCounter = -1;
							break;
						}
					}
				}
				else
				{	//启动延时
					if( (-StartCounter) > cfg.STDelay[0]*CtrlRateHz )
						StartCounter = 1e9;
					else
						--StartCounter;
				}
				return;
			}
		}
	
		static void CoaxialM2S2_MotorControl( double output_throttle, double outRoll, double outPitch, double outYaw, bool DbgSafe )
		{
			if( DbgSafe )
			{
				double pwmout[4];
				pwmout[0] = pwmout[1] = 0;
				pwmout[2] = 50;
				pwmout[3] = 50;
				ST_PWMMap( pwmout, 2, 2 );
				MainMotor_PWM_Out( pwmout );
				MainMotor_PullDownAll();
				update_output_throttle( 0 , 1.0/CtrlRateHz );
				return;
			}
			
			//读取电池电压
			float BatVoltage = get_MainBatteryVoltage_filted();
			//计算增益修正系数
			float STVoltage[2] = {0};
			ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 );
			//计算增益修正系数
			double bat_b_scale = 1.0;
			if( STVoltage[0]>7 && BatVoltage>7 )
				bat_b_scale = constrain( BatVoltage / STVoltage[0], 0.35f, 3.0f );
			
			//海拔推力补偿
			double baro_b_scale = 1.0;
			extern float internal_barometer_temperature;
			double tempK = internal_barometer_temperature + C_TO_KELVIN;
			if( tempK > 100 )
			{
				extern float internal_barometer_pressure;
				extern float internal_barometer_altitude;	
				double eas2tas_squared = SSL_AIR_DENSITY / (internal_barometer_pressure / (ISA_GAS_CONSTANT * tempK) );
				if( eas2tas_squared > 0.7 )
					baro_b_scale = 1.0 / eas2tas_squared;
			}
			
			//根据电池电压调整控制对象增益
			double b_scale = bat_b_scale * baro_b_scale;
//			ESOHeli[0].b = cfg.b[0] * Heli_RPb_scale*b_scale;
//			ESOHeli[1].b = cfg.b[2] * Heli_RPb_scale*b_scale;
			ESO[0].b = cfg.b[0] * b_scale;
			ESO[1].b = cfg.b[2] * b_scale;
			ESO[2].b = cfg.b[4] * b_scale;
			
			double rotor_output[12];
			double output_minimum_throttle = cfg.STThrottle[0];	
			
			if( output_throttle<0 || DbgSafe )
			{
				MainMotor_PullDownAll();
				update_output_throttle( 0 , 1.0/CtrlRateHz );
				return;
			}		
			
			double output_range = 100.0 - output_minimum_throttle;
			double output_midpoint = output_range / 2;
			
			//油门百分比转换为实际油门量
			output_throttle = MultiRotor_percent_to_throttle( output_throttle, output_minimum_throttle, output_range );
			
			/*pitch roll 输出限幅*/
				double absRoll = fabs(outRoll);
				if( absRoll > 50 )
					outRoll *= 50.0 / absRoll;
				rotor_output[2] = outRoll + 50;
				Roll_u = outRoll;
				
				double absPitch = fabs(outPitch);
				if( absPitch > 50 )
					outPitch *= 50.0 / absPitch;
				rotor_output[3] = outPitch + 50;
				Pitch_u = outPitch;
			/*pitch roll 输出限幅*/
			
			/*yaw输出限幅*/
				double allow_output_up = 100 - output_throttle;
				double allow_output_dn = output_throttle - output_minimum_throttle;
				double allow_output = ( allow_output_up < allow_output_dn) ? allow_output_up : allow_output_dn;
				if( fabs(outYaw) > allow_output )
				{
					double scale = allow_output / fabs(outYaw);
					outYaw *= scale;
				}
				Yaw_u = outYaw;
			/*yaw输出限幅*/
				
			//更新油门油门观测
			double throttle_percent = (output_throttle - output_minimum_throttle) / output_range*100;
			update_output_throttle( throttle_percent , 1.0/CtrlRateHz );
			//补偿非线性输出
			rotor_output[0] = output_throttle - outYaw;
			rotor_output[1] = output_throttle + outYaw;
			throttle_nonlinear_compensation( 2, rotor_output );
			ST_PWMMap( rotor_output, 2, 2 );
			MainMotor_PWM_Out( rotor_output );
		}
		
		static const UAVCtrlM CoaxialM2S2_CtrlM = {
			CoaxialM2S2_Init,
			CoaxialM2S2_PreArmControl,
			CoaxialM2S2_InitControl,
			CoaxialM2S2_MotorControl,
			0, 0,	//横滚俯仰二阶eso
			0, 1,	//横滚俯仰控推力导数
			false,	//初始化时不进行控制
			true,	//起飞前不添加扰动
		};
	/*共轴双桨 M2S2*/
		
	const UAVCtrlM* UAVCtrlMs[] = {
		/*000-*/	0	,
		/*001-*/	0	,
		/*002-*/	0	,
		/*003-*/	0	,
		/*004-*/	0	,
		/*005-*/	0	,
		/*006-*/	0	,
		/*007-*/	0	,
		/*008-*/	0	,
		/*009-*/	0	,
		/*010-*/	&MultiRotor4X_CtrlM	,
		/*011-*/	&MultiRotor6X_CtrlM	,
		/*012-*/	&MultiRotor8X_CtrlM	,
		/*013-*/	0	,
		/*014-*/	0	,
		/*015-*/	&MultiRotor4C_CtrlM	,
		/*016-*/	&MultiRotor6C_CtrlM	,
		/*017-*/	0	,
		/*018-*/	0	,
		/*019-*/	0	,
		/*020-*/	&MultiRotor42X_CtrlM	,
		/*021-*/	&MultiRotor62X_CtrlM	,
		/*022-*/	0	,
		/*023-*/	0	,
		/*024-*/	0	,
		/*025-*/	0	,
		/*026-*/	0	,
		/*027-*/	0	,
		/*028-*/	0	,
		/*029-*/	0	,
		/*030-*/	0	,
		/*031-*/	0	,
		/*032-*/	&MultiRotor6S1_CtrlM	,
		/*033-*/	0	,
		/*034-*/	0	,
		/*035-*/	0	,
		/*036-*/	0	,
		/*037-*/	0	,
		/*038-*/	0	,
		/*039-*/	0	,
		/*040-*/	0	,
		/*041-*/	0	,
		/*042-*/	0	,
		/*043-*/	0	,
		/*044-*/	0	,
		/*045-*/	0	,
		/*046-*/	0	,
		/*047-*/	0	,
		/*048-*/	0	,
		/*049-*/	0	,
		/*050-*/	0	,
		/*051-*/	0	,
		/*052-*/	0	,
		/*053-*/	0	,
		/*054-*/	0	,
		/*055-*/	0	,
		/*056-*/	0	,
		/*057-*/	0	,
		/*058-*/	0	,
		/*059-*/	0	,
		/*060-*/	0	,
		/*061-*/	0	,
		/*062-*/	0	,
		/*063-*/	0	,
		/*064-*/	0	,
		/*065-*/	0	,
		/*066-*/	0	,
		/*067-*/	0	,
		/*068-*/	0	,
		/*069-*/	0	,
		/*070-*/	0	,
		/*071-*/	0	,
		/*072-*/	0	,
		/*073-*/	0	,
		/*074-*/	0	,
		/*075-*/	0	,
		/*076-*/	0	,
		/*077-*/	0	,
		/*078-*/	0	,
		/*079-*/	0	,
		/*080-*/	&TriRotorX_CtrlM	,
		/*081-*/	0	,
		/*082-*/	0	,
		/*083-*/	0	,
		/*084-*/	0	,
		/*085-*/	0	,
		/*086-*/	0	,
		/*087-*/	0	,
		/*088-*/	0	,
		/*089-*/	0	,
		/*090-*/	0	,
		/*091-*/	0	,
		/*092-*/	0	,
		/*093-*/	0	,
		/*094-*/	0	,
		/*095-*/	0	,
		/*096-*/	0	,
		/*097-*/	0	,
		/*098-*/	0	,
		/*099-*/	0	,
		/*100-*/	&Heli131_CtrlM	,
		/*101-*/	0	,
		/*102-*/	0	,
		/*103-*/	0	,
		/*104-*/	0	,
		/*105-*/	0	,
		/*106-*/	0	,
		/*107-*/	0	,
		/*108-*/	0	,
		/*109-*/	0	,
		/*110-*/	0	,
		/*111-*/	0	,
		/*112-*/	0	,
		/*113-*/	0	,
		/*114-*/	0	,
		/*115-*/	0	,
		/*116-*/	0	,
		/*117-*/	0	,
		/*118-*/	0	,
		/*119-*/	0	,
		/*120-*/	0	,
		/*121-*/	0	,
		/*122-*/	0	,
		/*123-*/	0	,
		/*124-*/	0	,
		/*125-*/	0	,
		/*126-*/	0	,
		/*127-*/	0	,
		/*128-*/	0	,
		/*129-*/	0	,
		/*130-*/	0	,
		/*131-*/	0	,
		/*132-*/	0	,
		/*133-*/	0	,
		/*134-*/	0	,
		/*135-*/	0	,
		/*136-*/	0	,
		/*137-*/	0	,
		/*138-*/	0	,
		/*139-*/	0	,
		/*140-*/	0	,
		/*141-*/	0	,
		/*142-*/	0	,
		/*143-*/	0	,
		/*144-*/	0	,
		/*145-*/	0	,
		/*146-*/	0	,
		/*147-*/	0	,
		/*148-*/	0	,
		/*149-*/	0	,
		/*150-*/	0	,
		/*151-*/	0	,
		/*152-*/	0	,
		/*153-*/	0	,
		/*154-*/	0	,
		/*155-*/	0	,
		/*156-*/	0	,
		/*157-*/	0	,
		/*158-*/	0	,
		/*159-*/	0	,
		/*160-*/	&CoaxialM2S2_CtrlM	,
		/*161-*/	0	,
		/*162-*/	0	,
		/*163-*/	0	,
		/*164-*/	0	,
		/*165-*/	0	,
		/*166-*/	0	,
		/*167-*/	0	,
		/*168-*/	0	,
		/*169-*/	0	,
		/*170-*/	0	,
		/*171-*/	0	,
		/*172-*/	0	,
		/*173-*/	0	,
		/*174-*/	0	,
		/*175-*/	0	,
		/*176-*/	0	,
		/*177-*/	0	,
		/*178-*/	0	,
		/*179-*/	0	,
		/*180-*/	0	,
		/*181-*/	0	,
		/*182-*/	0	,
		/*183-*/	0	,
		/*184-*/	0	,
		/*185-*/	0	,
		/*186-*/	0	,
		/*187-*/	0	,
		/*188-*/	0	,
		/*189-*/	0	,
		/*190-*/	0	,
		/*191-*/	0	,
		/*192-*/	0	,
		/*193-*/	0	,
		/*194-*/	0	,
		/*195-*/	0	,
		/*196-*/	0	,
		/*197-*/	0	,
		/*198-*/	0	,
		/*199-*/	0	,
		/*200-*/	0	,
	};
		
/*机型控制方式*/
	
void ctrl_Attitude()
{	
	double h = 1.0 / CtrlRateHz;
	
	Receiver rc;
	getReceiver(&rc);
	
	bool DbgSafe = false;
	if( SafeBt )
	{	//Debug保护
		if( SafeBt==10 )
		{
			if( (rc.available && rc.data[0] < cfg.STThrottle[0] - 0.1) && (rc.data[1] < 10) )
				DbgSafe = true;
		}
		else if( SafeBt>=2 && SafeBt<=4 )
		{
			if( rc.available && rc.available_channels>=4+SafeBt && rc.data[4+SafeBt-1]>90 )
				DbgSafe = true;
		}
	}
	DebugSafeEna = DbgSafe;
	
	if( Attitude_Control_Enabled == false )
	{	//控制器未打开
		
		//定时同步参数
		static uint16_t ParamUpdateCounter = 65500;
		if( getInitializationCompleted() )
		{
			if( ParamUpdateCounter >= 3*CtrlRateHz )
			{
				ParamUpdateCounter = 0;
				ReadParamGroup( "AttCtrl", (uint64_t*)&cfg, 0, 0 );
				
				UAV_MTCount mt_count = UAV_MainMotorCount(cfg.UAVType[0]);
				if( mt_count.MTCount==0 || UAVCtrlMs[cfg.UAVType[0]]==0 )
				{
					CtrlM_available = false;
					set_MainMotorCount( 0, 0 );
				}
				else
				{
					set_MainMotorCount( mt_count.MTCount, mt_count.STCount, cfg.st_Freq[0] );
					CtrlM = *UAVCtrlMs[cfg.UAVType[0]];
					CtrlM_available = true;
				}
			}
			else
				++ParamUpdateCounter;
		}
		
		Roll_u = Pitch_u = Yaw_u = 0;
		update_output_throttle( 0 , h );
		if( CtrlM_available )
			CtrlM.PreArmControl(rc);
		return;
	}	
	
	//启动初始化
	if( Ctrl_initing )
	{	//还未进行启动初始化
		CtrlM.InitControl(DbgSafe);
		if( CtrlM.ctrl_on_init == false )
			return;
	}
	
	Quaternion AirframeQuat;
	get_Airframe_quat( &AirframeQuat, 0.1 );
	
	//获取控制参数
	double Ps = cfg.P1[0];
	double PsY = cfg.P1[4];
	vector3<double> P2( cfg.P2[0], cfg.P2[2], cfg.P2[4] );
	vector3<double> P3( cfg.P3[0], cfg.P3[2], cfg.P3[4] );
	
	//目标Roll Pitch四元数
	Quaternion target_quat_PR;			
	//目标角速度
	vector3<double> target_angular_velocity;

	//获取当前四元数的Pitch Roll分量四元数
	double Yaw = AirframeQuat.getYaw();
	double half_sinYaw, half_cosYaw;
	fast_sin_cos( 0.5*Yaw, &half_sinYaw, &half_cosYaw );
	Quaternion YawQuat(
		half_cosYaw ,
		0 ,
		0 ,
		half_sinYaw
	);
	YawQuat.conjugate();
	Quaternion_Ef current_quat_PR = Quaternion_Ef( YawQuat*AirframeQuat );
	
	//计算旋转矩阵
	current_quat_PR.conjugate();				
	double Rotation_Matrix[3][3];	//反向旋转
	current_quat_PR.get_rotation_matrix(Rotation_Matrix);
	current_quat_PR.conjugate();	
	double Rotation_Matrix_P[3][3]; //正向旋转
	current_quat_PR.get_rotation_matrix(Rotation_Matrix_P);
	
	//运行扩张状态观测器得到估计角速度、角加速度
	
	vector3<double> AngularRateCtrl;
	get_AngularRate_Ctrl( &AngularRateCtrl, 0.1 );
	vector3<double> angular_rate_ESO;
	vector3<double> angular_acceleration_ESO;
	//使用ESO估计角速度、角加速度
	angular_rate_ESO.set_vector(
		CtrlM_ESO[0]->get_EsAngularRate(),
		CtrlM_ESO[1]->get_EsAngularRate(),
		CtrlM_ESO[2]->get_EsAngularRate() );
	angular_acceleration_ESO.set_vector(
		CtrlM_ESO[0]->get_EsAngularAcceleration(),
		CtrlM_ESO[1]->get_EsAngularAcceleration(),
		CtrlM_ESO[2]->get_EsAngularAcceleration() );	
	
	//计算ENU坐标系下的角速度、角加速度
	vector3<double> angular_rate_ENU;
	angular_rate_ENU.x = Rotation_Matrix_P[0][0]*angular_rate_ESO.x + Rotation_Matrix_P[0][1]*angular_rate_ESO.y + Rotation_Matrix_P[0][2]*angular_rate_ESO.z;
	angular_rate_ENU.y = Rotation_Matrix_P[1][0]*angular_rate_ESO.x + Rotation_Matrix_P[1][1]*angular_rate_ESO.y + Rotation_Matrix_P[1][2]*angular_rate_ESO.z;
	angular_rate_ENU.z = Rotation_Matrix_P[2][0]*angular_rate_ESO.x + Rotation_Matrix_P[2][1]*angular_rate_ESO.y + Rotation_Matrix_P[2][2]*angular_rate_ESO.z;
	vector3<double> angular_acceleration_ENU;
	angular_acceleration_ENU.x = Rotation_Matrix_P[0][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[0][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[0][2]*angular_acceleration_ESO.z;
	angular_acceleration_ENU.y = Rotation_Matrix_P[1][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[1][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[1][2]*angular_acceleration_ESO.z;
	angular_acceleration_ENU.z = Rotation_Matrix_P[2][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[2][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[2][2]*angular_acceleration_ESO.z;
	
	//由Roll Pitch控制模式
	//计算Roll Pitch目标角速度（ENU系）
	vector3<double> target_angular_rate_RP;
	switch( RollPitch_ControlMode )
	{
		default:
		case Attitude_ControlMode_Angle:
		{
			//TD4滤目标角度
			Target_tracker_RP.track3( vector2<double>(target_Roll,target_Pitch), 1.0 / CtrlRateHz );
			
			//使用目标角度构造目标四元数
			//calculate target quat Q1
			//      front
			//       x
			//       ^
			//       |
			// y < --O
			double half_sinR, half_cosR;
			fast_sin_cos( 0.5*Target_tracker_RP.x1.x, &half_sinR, &half_cosR );
			double half_sinP, half_cosP;
			fast_sin_cos( 0.5*Target_tracker_RP.x1.y, &half_sinP, &half_cosP );
			target_quat_PR = Quaternion( 
				half_cosR*half_cosP ,
				half_cosP*half_sinR ,
				half_cosR*half_sinP ,
				-half_sinR*half_sinP
			);
			
			//计算误差四元数Q
			//Q*Q1=Qt  Q1为当前机体四元数，Qt为目标四元数
			//Q=Qt*inv(Q1)
			Quaternion current_quat_conj = current_quat_PR;	current_quat_conj.conjugate();
			vector3<double> PR_rotation = ( target_quat_PR * current_quat_conj ).get_Rotation_vec();
			vector3<double> feed_foward_ratePR = { Target_tracker_RP.x2.x, Target_tracker_RP.x2.y , 0 };
			target_angular_rate_RP = ( PR_rotation * Ps );
			target_angular_rate_RP.constrain(cfg.maxRPSp[0]);
			target_angular_rate_RP += feed_foward_ratePR;
			
			AC_angle_error = safe_sqrt( PR_rotation.get_square() );
			break;
		}
	}
	
	double target_angular_rate_Y;
	switch(Yaw_ControlMode)
	{
		case Attitude_ControlMode_Angle:
		{
			if(inFlight)
			{
				//TD4滤目标角度
				Target_trackerY.r2n = Target_trackerY.r2p = degree2rad(cfg.maxYSp[0]*0.7);
				Target_trackerY.track4( target_Yaw , 1.0f / CtrlRateHz );
				
				//角度误差化为-180 - +180
				double angle_error = Target_trackerY.x1 - Yaw;
				while( angle_error < -Pi )
					angle_error+=2*Pi;
				while( angle_error > Pi )
					angle_error-=2*Pi;

				//求目标角速度
				target_angular_rate_Y = angle_error * Ps + Target_trackerY.x2;
				target_angular_rate_Y = constrain( target_angular_rate_Y , 2.5 );
			}
			else
			{				
				Target_trackerY.reset();
				Target_trackerY.x1 = target_Yaw = Yaw;
				target_angular_rate_Y = 0;
			}
			break;
		}
		case Attitude_ControlMode_AngularRate:
		{
			if(inFlight)
			{
				Target_trackerY.r2n=Target_trackerY.r2p=degree2rad(cfg.maxYSp[0]);
				Target_trackerY.track3( target_AngularRate.z , 1.0 / CtrlRateHz );
				target_angular_rate_Y = Target_trackerY.x2;
			}
			else
			{				
				Target_trackerY.reset();
				Target_trackerY.x1 = target_Yaw = Yaw;
				target_angular_rate_Y = 0;
			}
			break;
		}
		case Attitude_ControlMode_OffBoard:
		{
			if(inFlight)
			{
				//TD4滤目标角度
				Target_trackerY.r2n = Target_trackerY.r2p = degree2rad(cfg.maxYSp[0]*0.7);
				Target_trackerY.track4( target_Yaw,target_AngularRate.z,0,0,0, 1.0f/CtrlRateHz );
				
				//角度误差化为-180 - +180
				double angle_error = Target_trackerY.x1 - Yaw;
				while( angle_error < -Pi )
					angle_error+=2*Pi;
				while( angle_error > Pi )
					angle_error-=2*Pi;

				//求目标角速度
				target_angular_rate_Y = angle_error * Ps + Target_trackerY.x2;
				target_angular_rate_Y = constrain( target_angular_rate_Y , 2.5 );
			}
			else
			{				
				Target_trackerY.reset();
				Target_trackerY.x1 = target_Yaw = Yaw;
				target_angular_rate_Y = 0;
			}
		}
		case Attitude_ControlMode_Locking:
		{
			if(inFlight)
			{
				Target_trackerY.track3( 0 , 1.0 / CtrlRateHz );
				target_angular_rate_Y = Target_trackerY.x2;
				if( in_symmetry_range( target_angular_rate_Y , 0.001 ) && in_symmetry_range( angular_rate_ENU.z , 0.05 ) )
				{							
					Target_trackerY.x1 = target_Yaw = Yaw;
					Yaw_ControlMode = Attitude_ControlMode_Angle;
				}
			}
			else
			{			
				Target_trackerY.reset();
				Target_trackerY.x1 = target_Yaw = Yaw;
				target_angular_rate_Y = 0;
			}
			break;
		}
	}
	
	//计算前馈量
		double YawAngleP =  ( Target_trackerY.get_tracking_mode() == 4 ) ? ( Ps ) : 0;
		vector3<double> Tv1_ENU = { Ps*( Target_tracker_RP.x2.x - angular_rate_ENU.x ) + Target_tracker_RP.x3.x ,
															Ps*( Target_tracker_RP.x2.y - angular_rate_ENU.y ) + Target_tracker_RP.x3.y ,
															YawAngleP*( Target_trackerY.x2 - angular_rate_ENU.z ) + Target_trackerY.x3 };
		vector3<double> Tv2_ENU = { Ps*( Target_tracker_RP.x3.x - angular_acceleration_ENU.x ) + Target_tracker_RP.T4.x ,
															Ps*( Target_tracker_RP.x3.y - angular_acceleration_ENU.y ) + Target_tracker_RP.T4.y,
															YawAngleP*( Target_trackerY.x3 - angular_acceleration_ENU.z ) + Target_trackerY.x4 };
		
		vector3<double> Tv1;
		Tv1.x = Rotation_Matrix[0][0]*Tv1_ENU.x + Rotation_Matrix[0][1]*Tv1_ENU.y + Rotation_Matrix[0][2]*Tv1_ENU.z;
		Tv1.y = Rotation_Matrix[1][0]*Tv1_ENU.x + Rotation_Matrix[1][1]*Tv1_ENU.y + Rotation_Matrix[1][2]*Tv1_ENU.z;
		Tv1.z = Rotation_Matrix[2][0]*Tv1_ENU.x + Rotation_Matrix[2][1]*Tv1_ENU.y + Rotation_Matrix[2][2]*Tv1_ENU.z;
		vector3<double> Tv2;
		Tv2.x = Rotation_Matrix[0][0]*Tv2_ENU.x + Rotation_Matrix[0][1]*Tv2_ENU.y + Rotation_Matrix[0][2]*Tv2_ENU.z;
		Tv2.y = Rotation_Matrix[1][0]*Tv2_ENU.x + Rotation_Matrix[1][1]*Tv2_ENU.y + Rotation_Matrix[1][2]*Tv2_ENU.z;
		Tv2.z = Rotation_Matrix[2][0]*Tv2_ENU.x + Rotation_Matrix[2][1]*Tv2_ENU.y + Rotation_Matrix[2][2]*Tv2_ENU.z;
		vector3<double> Ta1 = { P2.x*( Tv1.x - angular_acceleration_ESO.x ) + Tv2.x ,
														P2.y*( Tv1.y - angular_acceleration_ESO.y ) + Tv2.y ,
														P2.z*( Tv1.z - angular_acceleration_ESO.z ) + Tv2.z };
	//计算前馈量
													
	//把目标速度从Bodyheading旋转到机体
		vector3<double> target_angular_rate_ENU;
		target_angular_rate_ENU.x = target_angular_rate_RP.x;
		target_angular_rate_ENU.y = target_angular_rate_RP.y;
		target_angular_rate_ENU.z = target_angular_rate_RP.z + target_angular_rate_Y;

		vector3<double> target_angular_rate_body;
		target_angular_rate_body.x = Rotation_Matrix[0][0]*target_angular_rate_ENU.x + Rotation_Matrix[0][1]*target_angular_rate_ENU.y + Rotation_Matrix[0][2]*target_angular_rate_ENU.z;
		target_angular_rate_body.y = Rotation_Matrix[1][0]*target_angular_rate_ENU.x + Rotation_Matrix[1][1]*target_angular_rate_ENU.y + Rotation_Matrix[1][2]*target_angular_rate_ENU.z;
		target_angular_rate_body.z = Rotation_Matrix[2][0]*target_angular_rate_ENU.x + Rotation_Matrix[2][1]*target_angular_rate_ENU.y + Rotation_Matrix[2][2]*target_angular_rate_ENU.z;
	//把目标速度从Bodyheading旋转到机体
													
	//计算目标角加速度
		vector3<double> angular_rate_error = target_angular_rate_body - angular_rate_ESO;
		AC_rate_error = safe_sqrt( angular_rate_error.get_square() );
		vector3<double> target_angular_acceleration = target_angular_rate_body - angular_rate_ESO;
		target_angular_acceleration.x *= P2.x;
		target_angular_acceleration.y *= P2.y;
		target_angular_acceleration.z *= P2.z;
		target_angular_acceleration = target_angular_acceleration + Tv1;
	//计算目标角加速度
													
	//计算角加速度误差
	vector3<double> angular_acceleration_error = target_angular_acceleration - angular_acceleration_ESO;

	//获取扰动
	vector3<double> disturbance(
		CtrlM_ESO[0]->get_EsDisturbance() ,
		CtrlM_ESO[1]->get_EsDisturbance() ,
		CtrlM_ESO[2]->get_EsDisturbance()
	);

	double outRoll;double outPitch;double outYaw;
	if( inFlight || (CtrlM.RP_NoDisturbanceOG==false) )
	{
		switch( CtrlM.RP_ctrl_type )
		{
			default:
			case 0:
			{	//控制推力导数
				outRoll = 	( CtrlM_ESO[0]->get_EsMainPower() + CtrlM_ESO[0]->get_T() * ( angular_acceleration_error.x * P3.x + Ta1.x /*- disturbance_x*/ ) )/CtrlM_ESO[0]->get_b();
				outPitch =	( CtrlM_ESO[1]->get_EsMainPower() + CtrlM_ESO[1]->get_T() * ( angular_acceleration_error.y * P3.y + Ta1.y /*- disturbance_y*/ ) )/CtrlM_ESO[1]->get_b();
				break;
			}
			case 1:
			{	//期望角加速度P减扰动
				outRoll = 	( ( target_angular_acceleration.x - disturbance.x ) )/CtrlM_ESO[0]->get_b();
				outPitch =	( ( target_angular_acceleration.y - disturbance.y ) )/CtrlM_ESO[1]->get_b();
				break;
			}
			case 2:
			{	//期望角加速度PD减扰动
				outRoll = 	( ( target_angular_acceleration.x + P3.x*CtrlM_ESO[0]->get_T()*angular_acceleration_error.x - disturbance.x ) )/CtrlM_ESO[0]->get_b();
				outPitch =	( ( target_angular_acceleration.y + P3.x*CtrlM_ESO[1]->get_T()*angular_acceleration_error.y - disturbance.y ) )/CtrlM_ESO[1]->get_b();
				break;
			}
			case 5:
			{	//期望角速度P
				outRoll = 	( target_angular_rate_body.x - disturbance.x ) / CtrlM_ESO[0]->get_b();
				outPitch =	( target_angular_rate_body.y - disturbance.y ) / CtrlM_ESO[1]->get_b();
				break;
			}
		}
		
		switch( CtrlM.Y_ctrl_type )
		{
			default:
			case 0:
			{	//控制推力导数
				outYaw = ( CtrlM_ESO[2]->get_EsMainPower() + CtrlM_ESO[2]->get_T() * ( angular_acceleration_error.z * P3.z + Ta1.z ) ) / CtrlM_ESO[2]->get_b();
				break;
			}
			case 1:
			{	//期望角加速度P减扰动
				outYaw = ( target_angular_acceleration.z - disturbance.z ) / CtrlM_ESO[2]->get_b();
				break;
			}
			case 2:
			{	//期望角加速度PD减扰动
				outYaw = ( target_angular_acceleration.z + P3.z*CtrlM_ESO[2]->get_T()*angular_acceleration_error.z - disturbance.z ) / CtrlM_ESO[2]->get_b();
				break;
			}
			case 5:
			{	//期望角速度P
				outYaw = 	( target_angular_rate_body.z - disturbance.z ) / CtrlM_ESO[2]->get_b();
				break;
			}
		}
	}
	else
	{	//起飞前
		switch( CtrlM.RP_ctrl_type )
		{
			default:
			case 0:
			{	//控制推力导数
				outRoll = 	( ( target_angular_acceleration.x ) )/CtrlM_ESO[0]->get_b();
				outPitch =	( ( target_angular_acceleration.y ) )/CtrlM_ESO[1]->get_b();
				break;
			}
			case 1:
			{	//期望角加速度P减扰动
				outRoll = 	( ( target_angular_acceleration.x ) )/CtrlM_ESO[0]->get_b();
				outPitch =	( ( target_angular_acceleration.y ) )/CtrlM_ESO[1]->get_b();
				break;
			}
			case 2:
			{	//期望角加速度PD减扰动
				outRoll = 	( ( target_angular_acceleration.x + P3.x*CtrlM_ESO[0]->get_T()*angular_acceleration_error.x ) )/CtrlM_ESO[0]->get_b();
				outPitch =	( ( target_angular_acceleration.y + P3.x*CtrlM_ESO[1]->get_T()*angular_acceleration_error.y ) )/CtrlM_ESO[1]->get_b();
				break;
			}
			case 5:
			{	//期望角速度P
				outRoll = 	( target_angular_rate_body.x ) / CtrlM_ESO[0]->get_b();
				outPitch =	( target_angular_rate_body.y ) / CtrlM_ESO[1]->get_b();
//				outRoll = 	( target_angular_rate_body.x - disturbance.x ) / CtrlM_ESO[0]->get_b();
//				outPitch =	( target_angular_rate_body.y - disturbance.y ) / CtrlM_ESO[1]->get_b();
				break;
			}
		}
		
		switch( CtrlM.Y_ctrl_type )
		{
			default:
			case 0:
			{	//控制推力导数
				outYaw = ( CtrlM_ESO[2]->get_EsMainPower() + CtrlM_ESO[2]->get_T() * ( angular_acceleration_error.z * P3.z + Ta1.z ) ) / CtrlM_ESO[2]->get_b();
				break;
			}
			case 1:
			{	//期望角加速度P减扰动
				outYaw = ( target_angular_acceleration.z - disturbance.z ) / CtrlM_ESO[2]->get_b();
				break;
			}
			case 2:
			{	//期望角加速度PD减扰动
				outYaw = ( target_angular_acceleration.z + P3.z*CtrlM_ESO[2]->get_T()*angular_acceleration_error.z - disturbance.z ) / CtrlM_ESO[2]->get_b();
				break;
			}
			case 5:
			{	//期望角速度P
				outYaw = 	( target_angular_rate_body.z - disturbance.z ) / CtrlM_ESO[2]->get_b();
				break;
			}
		}
	}
	
//	outRoll_filted += 80 * h * ( outRoll - outRoll_filted );
//	outPitch_filted += 80 * h * ( outPitch - outPitch_filted );
//	outYaw_filted += 80 * h * ( outYaw - outYaw_filted );
	
	if( inFlight )
	{
		double logbuf[10];
		logbuf[0] = target_angular_rate_body.x;
		logbuf[1] = AngularRateCtrl.x;
		logbuf[2] = angular_rate_ESO.x;
		logbuf[3] = target_angular_acceleration.x;
		logbuf[4] = angular_acceleration_ESO.x;
		logbuf[5] = CtrlM_ESO[0]->get_EsMainPower();
		logbuf[6] = outRoll;
		logbuf[7] = disturbance.x;
		logbuf[8] = CtrlM_ESO[0]->get_u();
		SDLog_Msg_DebugVect( "att", logbuf, 9 );
		
//		logbuf[0] = target_angular_rate_body.z;
//		logbuf[1] = AngularRateCtrl.z;
//		logbuf[2] = angular_rate_ESO.z;
//		logbuf[3] = target_angular_acceleration.z;
//		logbuf[4] = angular_acceleration_ESO.z;
//		logbuf[5] = ESO[2].get_EsMainPower();
//		logbuf[6] = outYaw;
//		logbuf[7] = disturbance.z;
//		logbuf[8] = ESO[2].u;
//		SDLog_Msg_DebugVect( "yaw", logbuf, 9 );
	}

	CtrlM.MotorControl( throttle, outRoll, outPitch, outYaw, DbgSafe );
}

void init_Ctrl_Attitude()
{
	AccZ_filter.set_cutoff_frequency( CtrlRateHz , 1.5 );
	
	//注册参数
	cfg.UAVType[0] = UAVType_Rotor4_X;
	cfg.STThrottle[0] = 10;
	cfg.NonlinearFactor[0] = 0.45;
	cfg.FullThrRatio[0] = 0.95;
	cfg.STMode[0] = 1;
	cfg.STDelay[0] = 1.5;
	cfg.T[0] = 0.1;
	cfg.T2[0] = 0.1;
	cfg.b[0] = 5.5;	cfg.b[2] = 5.5;	cfg.b[4] = 1.0;
	cfg.TD4_P1[0] = 15;	cfg.TD4_P1[2] = 15;	cfg.TD4_P1[4] = 2;
	cfg.TD4_P2[0] = 15;	cfg.TD4_P2[2] = 15;	cfg.TD4_P2[4] = 5;
	cfg.TD4_P3[0] = 25;	cfg.TD4_P3[2] = 25;	cfg.TD4_P3[4] = 25;
	cfg.TD4_P4[0] = 25;	cfg.TD4_P4[2] = 25;	cfg.TD4_P4[4] = 25;
	cfg.P1[0] = 7;	cfg.P1[2] = 7;	cfg.P1[4] = 2;
	cfg.P2[0] = 10;	cfg.P2[2] = 10;	cfg.P2[4] = 5;
	cfg.P3[0] = 50;	cfg.P3[2] = 50;	cfg.P3[4] = 25;
	cfg.P4[0] = 15;	cfg.P4[2] = 15;	cfg.P4[4] = 15;
	cfg.beta[0] = 12;
	cfg.beta2[0] = 12;
	cfg.beta_h[0] = 4;
	cfg.beta_hAcc[0] = 4;
	cfg.maxLean[0] = 35;
	cfg.maxRPSp[0] = 350;
	cfg.maxRPAcc[0] = 7000;
	cfg.maxYSp[0] = 80;
	cfg.maxYAcc[0] = 1000;
	cfg.YawPri[0] = 0;
	cfg.st_Freq[0] = 333;
	cfg.st1min[0] = 1000;
	cfg.st1max[0] = 2000;
	cfg.st2min[0] = 1000;
	cfg.st2max[0] = 2000;
	cfg.st3min[0] = 1000;
	cfg.st3max[0] = 2000;
	cfg.st4min[0] = 1000;
	cfg.st4max[0] = 2000;
	cfg.st5min[0] = 1000;
	cfg.st5max[0] = 2000;
	cfg.st6min[0] = 1000;
	cfg.st6max[0] = 2000;
	cfg.st7min[0] = 1000;
	cfg.st7max[0] = 2000;
	cfg.st8min[0] = 1000;
	cfg.st8max[0] = 2000;

	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,	//UAV Type
		MAV_PARAM_TYPE_REAL32 ,	//起转油门
		MAV_PARAM_TYPE_REAL32 ,	//非线性参数
		MAV_PARAM_TYPE_REAL32 ,	//满油门比例
		MAV_PARAM_TYPE_UINT8 ,	//启动模式
		MAV_PARAM_TYPE_REAL32 ,	//启动延时
		MAV_PARAM_TYPE_REAL32 ,	//T
		MAV_PARAM_TYPE_REAL32 ,	//T2
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//b[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P1[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P2[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P3[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P4[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P1[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P2[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P3[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P4[3]
		MAV_PARAM_TYPE_REAL32 ,	//beta
		MAV_PARAM_TYPE_REAL32 ,	//beta2
		MAV_PARAM_TYPE_REAL32 ,	//h_beta
		MAV_PARAM_TYPE_REAL32 ,	//hAcc_beta
		MAV_PARAM_TYPE_REAL32 ,	//最大倾斜角
		MAV_PARAM_TYPE_REAL32 ,	//maxRPSp
		MAV_PARAM_TYPE_REAL32 ,	//maxRPAcc
		MAV_PARAM_TYPE_REAL32 ,	//maxYSp
		MAV_PARAM_TYPE_REAL32 ,	//maxYAcc
		MAV_PARAM_TYPE_REAL32 ,	//YawPri
		
		MAV_PARAM_TYPE_REAL32 ,	//舵机PWM频率
		MAV_PARAM_TYPE_REAL32 ,	//舵机1最小值
		MAV_PARAM_TYPE_REAL32 ,	//舵机1最大值
		MAV_PARAM_TYPE_REAL32 ,	//舵机2最小值
		MAV_PARAM_TYPE_REAL32 ,	//舵机2最大值
		MAV_PARAM_TYPE_REAL32 ,	//舵机3最小值
		MAV_PARAM_TYPE_REAL32 ,	//舵机3最大值
		MAV_PARAM_TYPE_REAL32 ,	//舵机4最小值
		MAV_PARAM_TYPE_REAL32 ,	//舵机4最大值
		MAV_PARAM_TYPE_REAL32 ,	//舵机5最小值
		MAV_PARAM_TYPE_REAL32 ,	//舵机5最大值
		MAV_PARAM_TYPE_REAL32 ,	//舵机6最小值
		MAV_PARAM_TYPE_REAL32 ,	//舵机6最大值
		MAV_PARAM_TYPE_REAL32 ,	//舵机7最小值
		MAV_PARAM_TYPE_REAL32 ,	//舵机7最大值
		MAV_PARAM_TYPE_REAL32 ,	//舵机8最小值
		MAV_PARAM_TYPE_REAL32 ,	//舵机8最大值
	};
	SName param_names[] = {
		"AC_UAVType" ,	//UAV Type
		"AC_STThr" ,	//起转油门
		"AC_NonlinF" ,	//非线性参数
		"AC_FullThrR" ,	//满油门比例
		"AC_STMode" ,	//启动模式
		"AC_STDelay" ,//启动延时
		"AC_T" ,	//T
		"AC_T2" ,	//T2
		"AC_Roll_b" ,"AC_Pitch_b" ,"AC_Yaw_b" ,	//b[3]
		"AC_Roll_TD4P1" ,"AC_Pitch_TD4P1" ,"AC_Yaw_TD4P1" ,	//TD4_P1[3]
		"AC_Roll_TD4P2" ,"AC_Pitch_TD4P2" ,"AC_Yaw_TD4P2" ,	//TD4_P2[3]
		"AC_Roll_TD4P3" ,"AC_Pitch_TD4P3" ,"AC_Yaw_TD4P3" ,	//TD4_P3[3]
		"AC_Roll_TD4P4" ,"AC_Pitch_TD4P4" ,"AC_Yaw_TD4P4" ,	//TD4_P4[3]
		"AC_Roll_P1" ,"AC_Pitch_P1" ,"AC_Yaw_P1" ,	//P1[3]
		"AC_Roll_P2" ,"AC_Pitch_P2" ,"AC_Yaw_P2" ,	//P2[3]
		"AC_Roll_P3" ,"AC_Pitch_P3" ,"AC_Yaw_P3" ,	//P3[3]
		"AC_Roll_P4" ,"AC_Pitch_P4" ,"AC_Yaw_P4" ,	//P4[3]
		"AC_Beta" ,	//beta
		"AC_Beta2" ,	//beta
		"AC_hBeta" ,	//h_beta
		"AC_hAccBeta" ,	//h_beta
		"AC_maxLean" ,	//最大倾斜角
		"AC_maxRPSp" ,	//最大Pitch Roll速度
		"AC_maxRPAcc" ,	//最大Pitch Roll加速度
		"AC_maxYSp" ,	//最大偏航速度
		"AC_maxYAcc" ,	//最大偏航加速度
		"AC_YawPri" ,	//偏航优先级（最大允许为偏航下降的油门量）
		
		"AC_STFreq" ,	//舵机PWM频率
		"AC_ST1Min" ,	//舵机1最小值
		"AC_ST1Max" ,	//舵机1最大值
		"AC_ST2Min" ,	//舵机2最小值
		"AC_ST2Max" ,	//舵机2最大值
		"AC_ST3Min" ,	//舵机3最小值
		"AC_ST3Max" ,	//舵机3最大值
		"AC_ST4Min" ,	//舵机4最小值
		"AC_ST4Max" ,	//舵机4最大值
		"AC_ST5Min" ,	//舵机5最小值
		"AC_ST5Max" ,	//舵机5最大值
		"AC_ST6Min" ,	//舵机6最小值
		"AC_ST6Max" ,	//舵机6最大值
		"AC_ST7Min" ,	//舵机7最小值
		"AC_ST7Max" ,	//舵机7最大值
		"AC_ST8Min" ,	//舵机8最小值
		"AC_ST8Max" ,	//舵机8最大值
	};
	ParamGroupRegister( "AttCtrl", 3, sizeof(cfg)/8, param_types, param_names, (uint64_t*)&cfg );
}