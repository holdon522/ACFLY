#pragma once

#include "Basic.hpp"
#include "vector2.hpp"
#include "vector3.hpp"

//飞行器类型
enum UAVType
{
	/*10-99多旋翼*/
		UAVType_Rotor4_X = 10 ,	//四旋翼X型
		UAVType_Rotor6_X = 11 ,	//六旋翼X型
		UAVType_Rotor8_X = 12 ,	//八旋翼X型
	
		UAVType_Rotor4_C = 15 ,	//四旋翼十字型
		UAVType_Rotor6_C = 16 ,	//六旋翼十字型
	
		UAVType_Rotor42_X = 20 ,	//四旋翼Double X字型
	
		UAVType_Rotor6_S1 = 32 ,	//六旋翼异构
		UAVType_Rotor3_X = 80 ,	//三旋翼（尾舵在后）
	/*10-99多旋翼*/
	
	/*100-150直升机*/
		UAVType_Heli131 = 100 ,	//直升机 1主旋翼+3舵机（左后右）+1尾舵
		UAVType_CoaxialM2S2 = 160 ,	//共轴双桨 2主电机+2舵机 舵机1横滚舵机2俯仰
	/*100-180直升机*/
};

struct UAV_MTCount
{
	uint8_t MTCount;
	uint8_t STCount;
};

inline UAV_MTCount UAV_MainMotorCount( uint8_t type )
{
	switch( type )
	{
		case UAVType_Rotor4_X:
		{
			UAV_MTCount res;
			res.MTCount = 4;
			res.STCount = 0;
			return res;
		}
		case UAVType_Rotor4_C:
		{
			UAV_MTCount res;
			res.MTCount = 4;
			res.STCount = 0;
			return res;
		}
		
		case UAVType_Rotor6_X:
		{
			UAV_MTCount res;
			res.MTCount = 6;
			res.STCount = 0;
			return res;
		}
		
		case UAVType_Rotor6_C:
		{
			UAV_MTCount res;
			res.MTCount = 6;
			res.STCount = 0;
			return res;
		}
		
		case UAVType_Rotor8_X:
		{
			UAV_MTCount res;
			res.MTCount = 8;
			res.STCount = 0;
			return res;
		}

		case UAVType_Rotor6_S1:
		{
			UAV_MTCount res;
			res.MTCount = 6;
			res.STCount = 0;
			return res;
		}
		
		case UAVType_Rotor42_X:
		{
			UAV_MTCount res;
			res.MTCount = 8;
			res.STCount = 0;
			return res;
		}
		

		
		case UAVType_Rotor3_X:
		{
			UAV_MTCount res;
			res.MTCount = 6;
			res.STCount = 2;
			return res;
		}
		
		case UAVType_Heli131:
		{
			UAV_MTCount res;
			res.MTCount = 6;
			res.STCount = 4;
			return res;
		}
		
		case UAVType_CoaxialM2S2:
		{
			UAV_MTCount res;
			res.MTCount = 4;
			res.STCount = 2;
			return res;
		}
		
		default:
		{
			UAV_MTCount res;
			res.MTCount = 0;
			res.STCount = 0;
			return res;
		}
	}
}

/*观测接口*/
	//获取悬停油门
	bool get_hover_throttle( double* result, double TIMEOUT = -1 );
	//获取当前油门主动力对应的加速度
	bool get_throttle_force( double* result, double TIMEOUT = -1 );
	//获取高度观测器估计的垂直加速度
	bool get_es_AccZ( double* result, double TIMEOUT = -1 );
	//获取高度观测器惯性时间T
	bool get_ESO_height_T( double* result, double TIMEOUT = -1 );
	//获取油门->加速度增益
	bool get_throttle_b( double* result, double TIMEOUT = -1 );
	//获取是否在飞行
	bool get_is_inFlight( bool* result, double TIMEOUT = -1 );
	//获取风力扰动对应的加速度
	bool get_WindDisturbance( vector3<double>* result, double TIMEOUT = -1 );
	
	//获取观测估计角速度
	bool get_EsAngularRate( vector3<double>* result, double TIMEOUT = -1 );
	//获取观测估计角加速度
	bool get_EsAngularAcc( vector3<double>* result, double TIMEOUT = -1 );
	//获取是否侧翻
	bool get_CrashedState();
/*观测接口*/	

/*Home位置*/
	bool getHomeLocalZ( double* home, double* home_yaw=0, double TIMEOUT = -1 );
	bool getHomePoint( vector2<double>* home, double* home_yaw=0, double TIMEOUT = -1 );
	bool getHomeLatLon( vector2<double>* home, double* home_yaw=0, double TIMEOUT = -1 );
/*Home位置*/

/*参数接口*/
	//获取最大倾斜角
	float get_maxLean();
	//获取最大偏航速度
	float get_maxYawSpeed();

	//获取最大上升速度
	float get_maxVelUp();
	//获取最大下降速度
	float get_maxVelDown();
	//获取最大水平速度
	float get_maxVelXY();
	//获取最大水平加速度
	float get_maxAccXY();
	//获取降落速度
	float get_LandVel();
	//获取航点提前量
	float get_WPRange();
	//获取位置环P1
	float get_P1();
	//获取速度环P2
	float get_P2();
	//获取XY速度前馈
	float get_VelXYFF();
/*参数接口*/

/*姿态控制*/
	enum Attitude_ControlMode
	{
		Attitude_ControlMode_Angle ,
		Attitude_ControlMode_AngularRate ,
		Attitude_ControlMode_Locking ,
		Attitude_ControlMode_OffBoard ,
	};
	
	//获取姿态控制器是否打开
	bool is_Attitude_Control_Enabled( bool* enabled, double TIMEOUT = -1 );
	//打开关闭姿态控制器
	bool Attitude_Control_Enable( double TIMEOUT = -1 );
	bool Attitude_Control_Disable( double TIMEOUT = -1 );

	//获取当前期望油门
	bool get_Target_Throttle( double* result, double TIMEOUT = -1 );
	//获取当前实际输出油门
	bool get_OutputThrottle( double* result, double TIMEOUT = -1 );
	//设定油门
	bool Attitude_Control_set_Throttle( double thr, double TIMEOUT = -1 );
	//获取目标Roll Pitch
	bool Attitude_Control_get_Target_RollPitch( double* Roll, double* Pitch, double TIMEOUT = -1 );
	//设定目标Roll Pitch
	bool Attitude_Control_set_Target_RollPitch( double Roll, double Pitch, double TIMEOUT = -1 );

	//获取目标Yaw
	bool Attitude_Control_get_TargetYaw( double* TargetYaw, double TIMEOUT = -1 );
	bool Attitude_Control_get_TargetTrackYaw( double* TargetYaw, double TIMEOUT = -1 );
	//获取yaw跟踪误差（误差为0即yaw旋转完成）
	bool Attitude_Control_get_YawTrackErr( double* YawErr, double TIMEOUT = -1 );
	//设定目标Yaw
	bool Attitude_Control_set_Target_Yaw( double Yaw, double TIMEOUT = -1 );
	bool Attitude_Control_set_Target_YawRelative( double Yaw, double TIMEOUT = -1 );
	//设定目标Yaw Offboard模式
	bool Attitude_Control_set_Target_Yaw_Offboard( double Yaw, double YawRate, double TIMEOUT=-1 );
	bool Attitude_Control_set_Target_YawRelative_Offboard( double Yaw, double YawRate, double TIMEOUT=-1 );
	//设定目标Yaw速度
	bool Attitude_Control_set_Target_YawRate( double YawRate, double TIMEOUT = -1 );
	//锁定Yaw（刹车后锁角度）
	bool Attitude_Control_set_YawLock( double TIMEOUT = -1 );
/*姿态控制*/

/*位置控制*/
	enum Position_ControlMode
	{
		//控制器未打开
		Position_ControlMode_Null = 255 ,
		
		//普通模式	
		//Position_ControlMode_VelocityTrack = 16 ,	//速度控制跟踪模式
		Position_ControlMode_ManualCircle = 15 ,	//2D绕圈模式
		Position_ControlMode_Position = 12 ,	//位置锁定模式
		Position_ControlMode_Velocity = 11 ,	//速度控制模式
		Position_ControlMode_Locking = 10 ,	//刹车后锁位置
		Position_ControlMode_OffBoard = 8 ,	//
		
		//2D自动模式
		Position_ControlMode_Takeoff = 20 ,	//起飞模式
		Position_ControlMode_RouteLine = 22 ,	//巡线模式

		//3D自动模式
		Position_ControlMode_RouteLine3D = 52 ,	//巡线模式
		Position_ControlMode_Circle3D = 55 ,	//巡线模式
	};
	#define Is_2DAutoMode(x) (x>=20 && x<=49)
	#define Is_3DAutoMode(x) (x>=50 && x<=79)
	#define Is_AutoMode(x) (x>=20 && x<=79)
	#define Is_YawAutoMode(x) (x==15)
	
	//获取模式及期望位置速度
	bool get_TargetPosInf( Position_ControlMode* pos_mode, Position_ControlMode* alt_mode, 
													vector3<double>* t_pos, vector3<double>* t_vel,
													double TIMEOUT=-1 );
	
	/*高度*/
		//打开关闭高度控制器
		bool is_Altitude_Control_Enabled( bool* ena, double TIMEOUT = -1 );
		bool Altitude_Control_Enable( double TIMEOUT = -1 );
		bool Altitude_Control_Disable( double TIMEOUT = -1 );
	
		//获取当前高度控制模式
		bool get_Altitude_ControlMode( Position_ControlMode* mode, double TIMEOUT = -1 );
	
		//设定Z自动飞行速度
		bool Position_Control_get_ZAutoSpeed( double* SpUp, double* SpDown, double TIMEOUT = -1 );
		bool Position_Control_reset_ZAutoSpeed( double TIMEOUT = -1 );
		bool Position_Control_set_ZAutoSpeed( double SpUp, double SpDown, double TIMEOUT = -1 );
	
		//移动Z目标位置（仅能在位置或者自动模式中使用）
		bool Position_Control_move_TargetPositionZRelative( double posz, double TIMEOUT = -1 );
	
		//设定目标高度
		bool Position_Control_set_TargetPositionZ( double posz, double vel = -1, double TIMEOUT = -1 );
		//设定目标高度(相对当前上升或下降）
		bool Position_Control_set_TargetPositionZRelative( double posz, double vel = -1, double TIMEOUT = -1 );
		//设定目标高度（指定海拔高度）
		bool Position_Control_set_TargetPositionZGlobal( double posz, double vel = -1, double TIMEOUT = -1 );
		//设定目标高度（相对起飞点）
		bool Position_Control_set_TargetPositionZRA( double posz, double vel = -1, double TIMEOUT = -1 );
		
		//设定目标垂直速度
		bool Position_Control_set_TargetVelocityZ( double velz, double TIMEOUT = -1 );
		//刹车后锁高度
		bool Position_Control_set_ZLock( double TIMEOUT = -1 );		
	
		//起飞到当前高度上方的height高度
		bool Position_Control_Takeoff_HeightRelative( double height, double TIMEOUT = -1 );
		//起飞到指定海拔高度
		bool Position_Control_Takeoff_HeightGlobal( double height, double TIMEOUT = -1 );
		//起飞到指定高度（Local坐标）
		bool Position_Control_Takeoff_Height( double height, double TIMEOUT = -1 );
	/*高度*/
	
	/*水平位置*/
		//打开关闭水平位置控制
		bool is_Position_Control_Enabled( bool* ena, double TIMEOUT = -1 );
		bool Position_Control_Enable( double TIMEOUT = -1 );
		bool Position_Control_Disable( double TIMEOUT = -1 );
		
		//获取当前水平位置控制模式
		bool get_Position_ControlMode( Position_ControlMode* mode, double TIMEOUT = -1 );
	
		//设定自动飞行速度
		bool Position_Control_get_XYAutoSpeed( double* AtVelXY, double TIMEOUT = -1 );
		bool Position_Control_reset_XYAutoSpeed( double TIMEOUT = -1 );
		bool Position_Control_set_XYAutoSpeed( double AtVelXY, double TIMEOUT = -1 );
		
		bool Position_Control_get_XYZAutoSpeed( double* AtVelXYZ, double TIMEOUT = -1 );
		bool Position_Control_reset_XYZAutoSpeed( double TIMEOUT = -1 );
		bool Position_Control_set_XYZAutoSpeed( double AtVelXYZ, double TIMEOUT = -1 );
	
		bool Position_Control_do_ManualCircleRelative( double dCircleVel, double dCircleR, double dCircleO, double TIMEOUT = -1 );
	
		/*飞直线*/
			//飞到目标水平位置
			bool Position_Control_set_TargetPositionXY( double posx, double posy, double vel = -1, double TIMEOUT = -1 );
			bool Position_Control_set_TargetPositionXYZ( double posx, double posy, double posz, double vel = -1, double TIMEOUT = -1 );
			//飞到目标水平位置（相对当前坐标）
			bool Position_Control_set_TargetPositionXYRelative( double posx, double posy, double vel = -1, double TIMEOUT = -1 );
			bool Position_Control_set_TargetPositionXYZRelative( double posx, double posy, double posz, double vel = -1, double TIMEOUT = -1 );
			//飞到目标水平位置（相对当前坐标且偏移在Bodyheading系下）
			bool Position_Control_set_TargetPositionXYRelativeBodyheading( double posx, double posy, double vel = -1, double TIMEOUT = -1 );
			bool Position_Control_set_TargetPositionXYZRelativeBodyheading( double posx, double posy, double posz, double vel = -1, double TIMEOUT = -1 );
			//根据经纬度（根据最优全球定位传感器）设定目标水平位置
			bool Position_Control_set_TargetPositionXY_LatLon( double Lat, double Lon, double vel = -1, double TIMEOUT = -1 );
			//根据经纬度和海拔高度（根据最优全球定位传感器）设定目标位置（三维直线）
			bool Position_Control_set_TargetPositionXYZ_LatLon( double Lat, double Lon, double posz, double vel = -1, double TIMEOUT = -1 );
			//根据经纬度（根据最优全球定位传感器）设定目标位置（三维直线）
			//posz为相对当前高度增加的高度
			bool Position_Control_set_TargetPositionXYZRelative_LatLon( double Lat, double Lon, double posz, double vel = -1, double TIMEOUT = -1 );
			//根据经纬度（根据最优全球定位传感器）设定目标位置（三维直线）
			//posz为距离起飞点Z坐标的高度
			bool Position_Control_set_TargetPositionXYZRA_LatLon( double Lat, double Lon, double posz, double vel = -1, double TIMEOUT = -1 );
			
			//获取直线飞行已飞行距离
			bool Position_Control_get_LineFlightDistance( double* distance, double TIMEOUT = -1 );
			//获取直线飞行AB和已飞行距离（A-目标点 B-起始点）
			bool Position_Control_get_LineFlightABDistance( vector3<double>* AB, double* distance, double TIMEOUT = -1 );
		/*飞直线*/
		
		/*跟踪目标位置*/
			//跟踪目标水平位置（相对当前坐标）
			bool Position_Control_move_TargetPositionXYRelative( double posx, double posy, double TIMEOUT = -1 );
			//跟踪目标水平位置（相对当前坐标且偏移在Bodyheading系下）
			bool Position_Control_move_TargetPositionXYRelativeBodyheading( double posx, double posy, double TIMEOUT = -1 );
		/*跟踪目标位置*/
		
		//设定目标水平速度（ENU朝向）并限制最大角度（补偿风力后的角度）
		//maxAngle<0不限制角度
		bool Position_Control_set_TargetVelocityXY_AngleLimit( double velx, double vely, double maxAngle = -1, double TIMEOUT = -1 );
		//设定目标水平速度（Bodyheading朝向）并限制最大角度（补偿风力后的角度）
		//maxRoll>0且maxPitch<0时限制合角度
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( double velx, double vely, double maxRoll = -1, double maxPitch = -1, double TIMEOUT = -1 );
		//刹车后锁定水平位置
		bool Position_Control_set_XYLock( double maxAcc = -1, double TIMEOUT = -1 );
		
		//设定前方障碍物距离避障
		bool Position_Control_set_RouteLineAvoidanceRelative( double pos=-1, double vel=-1, double TIMEOUT=-1 );
	/*水平位置*/
	
	/*OffBoard模式*/
		/*XY*/
			//控制位置+速度+推力
			bool Position_Control_set_TargetPosVelAccXY_OffBoard( double posx, double posy, double velx, double vely, double accx, double accy, double TIMEOUT=-1 );
			//控制位置（经纬度）+速度+推力
			bool Position_Control_set_TargetGlobalPosVelAccXY_OffBoard( double Lat, double Lon, double velx, double vely, double accx, double accy, double TIMEOUT=-1 );
			//控制位置（相对当前）+速度+推力
			bool Position_Control_set_TargetPosRelativeVelAccXY_OffBoard( double posx, double posy, double velx, double vely, double accx, double accy, double TIMEOUT=-1 );
			
			bool Position_Control_set_TargetVelAccXY_OffBoard( double velx, double vely, double accx, double accy, double TIMEOUT=-1 );
		/*XY*/
			
		/*Z*/
			//控制位置+速度+推力
			bool Position_Control_set_TargetPosVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT=-1 );
			//控制位置（经纬度）+速度+推力
			bool Position_Control_set_TargetPosGlobalVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT=-1 );
			//控制位置（相对当前）+速度+推力
			bool Position_Control_set_TargetPosRelativeVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT=-1 );
			//控制位置（相对地面）+速度+推力
			bool Position_Control_set_TargetPosZRAVelAccZ_OffBoard( double posz, double velz, double accz, double TIMEOUT=-1 );
			
			bool Position_Control_set_TargetVelAccZ_OffBoard( double velz, double accz, double TIMEOUT=-1 );
		/*Z*/
	/*OffBoard模式*/
/*位置控制*/

/*安全接口*/
	//获取控制器上次控制时间
	//太久不进行控制将进入MSafe模式
	//XY为水平控制（包括姿态控制）
	//Z为高度控制（包括油门控制）
	bool get_lastXYCtrlTime( TIME* t, double TIMEOUT = -1 );
	bool get_lastZCtrlTime( TIME* t, double TIMEOUT = -1 );
	
	//将控制器上次控制时间设为不可用
	//强制进入MSafe模式
	//MSafe模式下无法关闭位置控制器
	//同时作出XYZ位置控制可退出MSafe
	//（水平控制不可用时控制角度）
	//forceRTL：是否在有遥控器器时也执行返航（遥控器需要回中）
	//TIMEOUT：超时时间
	bool enter_MSafe( bool forceRTL = false, double TIMEOUT = -1 );
	
	//获取是否进入了MSafe模式
	bool is_MSafeCtrl();
/*安全接口*/

/*震动数据*/
	bool get_Vibration( vector3<float>* vec, double TIMEOUT=-1 );
/*震动数据*/