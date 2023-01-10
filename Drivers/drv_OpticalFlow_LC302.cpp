#include "drv_OpticalFlow_LC302.hpp"
#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

struct DriverInfo
{
	uint32_t param;
	Port port;
	uint32_t sensor_key;
};

typedef struct
{
	int16_t flow_x_integral;	// X 像素点累计时间内的累加位移(radians*10000)
														// [除以 10000 乘以高度(mm)后为实际位移(mm)]
	int16_t flow_y_integral;	// Y 像素点累计时间内的累加位移(radians*10000)
														// [除以 10000 乘以高度(mm)后为实际位移(mm)]
	uint16_t integration_timespan;	// 上一次发送光流数据到本次发送光流数据的累计时间（us）
	uint16_t ground_distance; // 预留。 默认为 999（0x03E7）
	uint8_t valid;	// 状态值:0(0x00)为光流数据不可用
									//245(0xF5)为光流数据可用
	uint8_t version; //版本号
}__PACKED _Flow;
static const unsigned char packet_ID[2] = { 0xfe , 0x0a };

static void OpticalFlow_Server(void* pvParameters)
{
	/*状态机*/
		_Flow  Flow;
		unsigned char rc_counter = 0;
		signed char sum = 0;
	/*状态机*/
	
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	while(1)
	{
		uint8_t rdata;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter < 2 )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					++rc_counter;
					sum = 0;
				}
			}
			else if( rc_counter < 12 )
			{	//接收数据
				( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
				sum ^= (signed char)rdata;
				++rc_counter;
			}
			else if( rc_counter == 12 )
			{	//校验
				if( sum != rdata )
					rc_counter = 0;
				else
					++rc_counter;
			}
			else
			{	//接收包尾
				if( rdata == 0x55 )
				{
					PosSensorHealthInf1 ZRange_inf;
					if( get_OptimalRange_Z( &ZRange_inf ) )
					{	//测距传感器可用
						if( ZRange_inf.last_healthy_TIME.is_valid() && ZRange_inf.last_healthy_TIME.get_pass_time() < 50 )
						{	//测距50秒内健康
							//获取高度
							double height = ZRange_inf.HOffset + ZRange_inf.PositionENU.z;
							//获取角速度
							vector3<double> AngularRate;
							get_AngularRate_Ctrl( &AngularRate );
							//补偿光流
							vector3<double> vel;
							double rotation_compensation_x = -constrain( AngularRate.y * 10000 , 4500000000.0 );
							double rotation_compensation_y = constrain(  AngularRate.x * 10000 , 4500000000.0 );
							double integral_time = (Flow.integration_timespan * 1e-6f);
							if( integral_time > 1e-3 )
							{
								double temp_flow_x, temp_flow_y;
								double flow_x, flow_y;
								temp_flow_x = Flow.flow_x_integral;
								temp_flow_y = -Flow.flow_y_integral;
								switch(driver_info.param)
								{
									case 0:
									default:
									{
										flow_x = temp_flow_x;
										flow_y = temp_flow_y;
										break;
									}
									case 1:
									{
										flow_x = temp_flow_y;
										flow_y = -temp_flow_x;
										break;
									}
									case 2:
									{
										flow_x = -temp_flow_x;
										flow_y = -temp_flow_y;
										break;
									}
									case 3:
									{
										flow_x = -temp_flow_y;
										flow_y = temp_flow_x;
										break;
									}
								}
								
								integral_time = 1.0 / integral_time;
								vel.x = ( flow_x*integral_time - rotation_compensation_x ) * 1e-4f * ( 1 + height );
								vel.y = ( flow_y*integral_time - rotation_compensation_y ) * 1e-4f * ( 1 + height ) ;
								PositionSensorUpdateVel( default_optical_flow_index,driver_info.sensor_key, vel , true );
							}
							else
								PositionSensorSetInavailable( default_optical_flow_index,driver_info.sensor_key );
						}
						else
							PositionSensorSetInavailable( default_optical_flow_index,driver_info.sensor_key );
					}
					else
						PositionSensorSetInavailable( default_optical_flow_index,driver_info.sensor_key );
				}
				rc_counter = 0;
			}
			
		}
	}
}

static bool OpticalFlow_LC302_DriverInit( Port port, uint32_t param )
{
	//波特率19200
	port.SetBaudRate( 19200, 2, 2 );
	//注册传感器
	uint32_t sensor_key = PositionSensorRegister( default_optical_flow_index , \
																								"LC302" ,\
																								Position_Sensor_Type_RelativePositioning , \
																								Position_Sensor_DataType_v_xy , \
																								Position_Sensor_frame_BodyHeading , \
																								0.1, 100 );
	if(sensor_key==0)
		return false;
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	driver_info->sensor_key = sensor_key;
	xTaskCreate( OpticalFlow_Server, "OptFlowLC302", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_OpticalFlow_LC302()
{
	PortFunc_Register( 32, OpticalFlow_LC302_DriverInit );
}