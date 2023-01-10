#pragma once

#include "Basic.hpp"
#include "vector3.hpp"
#include "map_projection.hpp"
#include "quaternion.hpp"

/*IMU*/
	struct IMUConfig
	{
		double offset[3];
		double scale[3];
		double STTemperature;
		double TemperatureCoefficient[3];
	} __PACKED;
	#define IMUConfigLength 10

	/*IMU传感器定义*/
		#define IMU_Sensors_Count 3
		
		extern const uint8_t External_Magnetometer_Index;
		extern const uint8_t Internal_Magnetometer_Index;
		struct IMU_Sensor
		{
			SName name;	//传感器名称
			TIME last_update_time;	//上次更新时间
			double sample_time;	//采样时间
			bool calibrated;	//是否已校准
			bool have_temperature;	//是否有温度数据
			
			double temperature;	//温度
			double sensitivity;	//灵敏度（原始数据->实际单位 陀螺：rad/s 加速度：cm/s^2 磁场：gauss）
			
			bool data_error;	//数据是否错误（爆量程）
			vector3<int32_t> data_raw;	//原始数据
			vector3<double> data_rawTC;	//温度补偿后的原始数据
			vector3<double> data;	//实际单位数据
		};		
	/*IMU传感器定义*/
		
	/*IMU传感器读取函数*/
		bool GetAccelerometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT = -1 );
		bool GetGyroscope( uint8_t index, IMU_Sensor* sensor, double TIMEOUT = -1 );
		bool GetMagnetometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT = -1 );
	/*IMU传感器读取函数*/
/*IMU*/
		
/*双天线传感器*/
	#define DAO_Sensors_Count 1
		
	struct DAO_Sensor
	{
		SName name;	//传感器名称
		TIME last_update_time;	//上次更新时间
		double sample_time;	//采样时间
		
		bool available;	//数据是否可用
		double delay;	//延时时间
		vector3<double> st_relPos;	//基准机体侧向向量（前左上）
		vector3<double> relPos;	//侧向向量（东北天）
	};
	
/*双天线侧向传感器*/
	/*DAO传感器读取函数*/
		bool GetDAOSensor( uint8_t index, DAO_Sensor* sensor, double TIMEOUT = -1 );
	/*DAO传感器读取函数*/
/*双天线侧向传感器*/
	
/*位置传感器*/
	/*位置传感器定义*/
		#define Position_Sensors_Count 16
		
		extern const uint8_t default_ultrasonic_sensor_index;
		extern const uint8_t default_optical_flow_index;
		extern const uint8_t external_baro_sensor_index;
		extern const uint8_t internal_baro_sensor_index;
		extern const uint8_t default_gps_sensor_index;
		extern const uint8_t default_rtk_sensor_index;
		extern const uint8_t default_uwb_sensor_index;
		extern const uint8_t default_vision_sensor_index;
		
		enum Position_Sensor_Type
		{
			Position_Sensor_Type_GlobalPositioning = 0 ,	//全球定位（经纬度定位，如GPS）
			Position_Sensor_Type_RelativePositioning = 1 ,	//相对定位（如气压计，参照物不会改变）
			Position_Sensor_Type_RangePositioning = 2 ,	//距离定位（测距定位，如超声波，参照物可能会变化）
		};
		enum Position_Sensor_DataType
		{
			//s-位置 v-速度
			//如sv_xy表示该传感器具有：位置速度的xy数据
			Position_Sensor_DataType_s_xy = 0 ,
			Position_Sensor_DataType_s_z = 1 ,
			Position_Sensor_DataType_s_xyz = 2  ,
			
			Position_Sensor_DataType_v_xy = 8  ,
			Position_Sensor_DataType_v_z = 9  ,
			Position_Sensor_DataType_v_xyz = 10 ,
			
			Position_Sensor_DataType_sv_xy = 16  ,
			Position_Sensor_DataType_sv_z = 17  ,
			Position_Sensor_DataType_sv_xyz = 18  ,
		};
		enum Position_Sensor_frame
		{
			Position_Sensor_frame_ENU = 0 ,	//速度数据在ENU坐标系下
			Position_Sensor_frame_BodyHeading = 1 ,	//速度数据x为机头朝向（与地面平行），y为朝向机头左方（与地面平行），z为上方
		};
		struct Position_Sensor_Data
		{
			bool available;	//传感器是否可用
			TIME last_update_time;	//上次更新时间
			TIME available_status_update_time;	//传感器可用信号更新时间
			double delay;	//传感器延时
			double xy_trustD;	//信任度 0最高信任度
			double z_trustD;	//信任度 0最高信任度
			double sample_time;	//采样时间

			Position_Sensor_Type sensor_type;	//传感器类型（见枚举注释）
			Position_Sensor_DataType sensor_DataType;	//传感器数据类型（见枚举注释）
			Position_Sensor_frame velocity_data_frame;	//速度数据坐标系（见枚举注释）
			
			vector3<double> position_Global;	//经纬度
			vector3<double> position;	//位置(cm)
			vector3<double> velocity;	//速度(cm/s)
			
			Quaternion data_quat;	//数据更新时刻的姿态（已补偿延时）
			//经纬度转换信息
			Map_Projection mp;
		};
		struct Position_Sensor_Inf
		{
			SName name;
			/*附加信息
				Global传感器：
					0：sat_count卫星个数
					1：fix_type定位类型
					2：week
					3：TOW
					4：hAcc水平方向精度(cm)
					5：vAcc垂直方向精度(cm)
					6：sAcc速度精度(cm/s)
			*/
			double addition_inf[8];
		};
		struct Position_Sensor
		{
			Position_Sensor_Data data;
			Position_Sensor_Inf inf;
		};
	/*位置传感器定义*/
	
	/*位置传感器读取函数*/
		bool GetPositionSensor( uint8_t index, Position_Sensor* result_sensor, double TIMEOUT = -1 );
		bool GetPositionSensorData( uint8_t index, Position_Sensor_Data* result_sensor, double TIMEOUT = -1 );
	/*位置传感器读取函数*/
/*位置传感器*/
		
/*传感器位置偏移*/

	struct SensorPosOffset
	{
		//安装方向（相对机头）
		//机头方向角度(deg)
		//前-0 左90 后180 右270
		float Fc_dir[2];	//飞控
		float Mag0_dir[2];	//罗盘0
		float Mag1_dir[2];	//罗盘1
		
		//飞控位置偏移
		float Fc_x[2];
		float Fc_y[2];
		float Fc_z[2];
		
		//传感器0位置偏移
		float S0_x[2];
		float S0_y[2];
		float S0_z[2];
		
		//传感器1位置偏移
		float S1_x[2];
		float S1_y[2];
		float S1_z[2];
		
		//传感器2位置偏移
		float S2_x[2];
		float S2_y[2];
		float S2_z[2];
		
		//传感器3位置偏移
		float S3_x[2];
		float S3_y[2];
		float S3_z[2];
		
		//传感器4位置偏移
		float S4_x[2];
		float S4_y[2];
		float S4_z[2];
		
		//传感器5位置偏移
		float S5_x[2];
		float S5_y[2];
		float S5_z[2];
		
		//传感器6位置偏移
		float S6_x[2];
		float S6_y[2];
		float S6_z[2];
		
		//传感器7位置偏移
		float S7_x[2];
		float S7_y[2];
		float S7_z[2];
		
		//传感器8位置偏移
		float S8_x[2];
		float S8_y[2];
		float S8_z[2];
		
		//传感器9位置偏移
		float S9_x[2];
		float S9_y[2];
		float S9_z[2];
		
		//传感器10位置偏移
		float S10_x[2];
		float S10_y[2];
		float S10_z[2];
		
		//传感器11位置偏移
		float S11_x[2];
		float S11_y[2];
		float S11_z[2];
		
		//传感器12位置偏移
		float S12_x[2];
		float S12_y[2];
		float S12_z[2];
		
		//传感器13位置偏移
		float S13_x[2];
		float S13_y[2];
		float S13_z[2];
		
		//传感器14位置偏移
		float S14_x[2];
		float S14_y[2];
		float S14_z[2];
		
		//传感器15位置偏移
		float S15_x[2];
		float S15_y[2];
		float S15_z[2];
	};

/*传感器位置偏移*/