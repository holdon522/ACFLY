#pragma once

#include "stm32h743xx.h"
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
	extern "C" {
#endif

#ifdef SYSFREQ480
	#define SYSCLK 480000000
	#define HCLK 24000000
	#define APB1CLK 120000000
	#define APB1TIMERCLK 240000000
	#define APB2CLK 120000000
	#define APB2TIMERCLK 240000000
	#define APB3CLK 120000000
	#define APB4CLK 120000000
#else
	#define SYSCLK 400000000
	#define HCLK 20000000
	#define APB1CLK 100000000
	#define APB1TIMERCLK 200000000
	#define APB2CLK 100000000
	#define APB2TIMERCLK 200000000
	#define APB3CLK 100000000
	#define APB4CLK 100000000
#endif
#define USART234578CLK 100000000
#define USART16CLK 100000000


#define TIMEBASECLK 10000000
		
	/*
		鎿嶄綔绯荤粺寤舵椂鍑芥暟锛堥€氳繃鎸傝捣绾跨▼锛?
		鍙兘鍦ㄦ搷浣滅郴缁熶换鍔′腑浣跨敤
		t:绉掑崟浣嶅欢鏃舵椂闂?
	*/
	inline void os_delay(double t)
	{
		TickType_t ticks = t*configTICK_RATE_HZ;
		if( ticks < 1 )
			ticks = 1;
		vTaskDelay(ticks);
	}
	
	bool get_RTC_Updated();
	
	//閿佸畾RTC鎿嶄綔
	bool Lock_RTC();
	void UnLock_RTC();
	
	/*RTC鏃堕棿鑾峰彇*/
		typedef struct
		{
			uint16_t Year;
			uint8_t Month;
			uint8_t Date;
			uint8_t WeekDay;

			uint8_t Hours;
			uint8_t Minutes;
			uint8_t Seconds;
			uint32_t SubSeconds;
			uint8_t TimeFormat;
			
			double Seconds_f;
		}RTC_TimeStruct;
	
		//鑾峰彇RTC鏃堕棿
		RTC_TimeStruct Get_RTC_Time();

		//璁剧疆RTC鏃ユ湡鍜屾椂闂?
		void Set_RTC_Time(const RTC_TimeStruct* T);
		
		//鏍规嵁缁忕含搴﹁绠楁椂鍖?
		int GetTimeZone(double lat,double lon);
    
		//UTC杞湰鍦版椂闂?
		void UTC2LocalTime(RTC_TimeStruct* rtc_local, uint16_t utc_year,uint8_t utc_month,uint8_t utc_day,
											 uint8_t utc_hour,uint8_t utc_minute,uint8_t utc_second,int8_t TimeZone,int8_t leapS);
		
	/*RTC鏃堕棿鑾峰彇*/		
#ifdef __cplusplus
	}
#endif