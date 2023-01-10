#include "drv_Main.hpp"

#include "SensorsBackend.hpp"
#include "drv_CRC.hpp"
#include "drv_LED.hpp"
#include "drv_Oled.hpp"
#include "GUI.hpp"
#include "drv_USB.hpp"
#include "drv_Uart1.hpp"
#include "drv_Uart3.hpp"
#include "drv_Uart5.hpp"
#include "drv_Uart7.hpp"
#include "drv_Uart8.hpp"
#include "Commulink.hpp"
#include "drv_SDMMC.hpp"
#include "drv_Sensors.hpp"
#include "drv_Ultrasonic.hpp"
#include "drv_Flash.hpp"
#include "drv_RCSbus.hpp"
#include "drv_RCPPM.hpp"
#include "drv_PWMOut.hpp"
#include "drv_ADC.hpp"

#include "StorageSystem.hpp"
#include "Parameters.hpp"
#include "drv_BootLoader.hpp"
#include "drv_ExtIIC.hpp"
#include "drv_GPS.hpp"
#include "drv_RTK.hpp"
#include "drv_InternalMag.hpp"
#include "drv_ExtMag.hpp"
#include "drv_ExtBarometer.hpp"
#include "drv_ExtLed.hpp"

#include "drv_OpticalFlow_LC302.hpp"
#include "drv_OpticalFlow_LC306.hpp"
#include "drv_OpticalFlow_JL32xx.hpp"
#include "drv_OpticalFlow_GL9306.hpp"

#include "drv_UWB_LinkTrack.hpp"

//偶脡艃膭脥芒脡膷
#include "drv_TFMini.hpp"

#include "drv_SDI.hpp"

void init_drv_Main()
{	
	//LED鍒濆鍖?
	init_drv_LED();
	//ADC鍒濆鍖?
	init_drv_ADC();
	//绛夊緟鐢靛帇绋冲畾
	while( Get_VDDA_Voltage() < 3.0 )
	{	//绛夊緟鐢靛帇绋冲畾
		os_delay(0.1);
	}
	//鎵撳紑CRC
	//init_drv_CRC();
	//鍐呴儴瀛樺偍鍒濆鍖?
	init_drv_Flash();
	init_InternalStorage();	
	//鍙傛暟鍒濆鍖?
	init_Parameters();
	//瀛樺偍澶栬椹卞姩	
	init_drv_SDMMC();		
	//BL鏇存柊妫€娴?
	init_drv_BootLoader();
	
	//鍐呴儴浼犳劅鍣ㄩ┍鍔?
	init_Sensors();
	init_Commulink();
	init_drv_Oled();
	init_GUI();
	os_delay(0.1);
	
	//PWWM椹卞姩
	init_drv_PWMOut();	
	
	//SD鍗￠┍鍔?
	init_SDStorage();
	
	//绔彛椹卞姩
	init_drv_USB();	
	init_drv_Uart1();
	init_drv_Uart3();
	init_drv_Uart5();
	init_drv_Uart7();
	init_drv_Uart8();
	
	//鍐呴儴浼犳劅鍣ㄩ┍鍔?
	init_drv_Sensors();
	//鎺ユ敹鏈洪┍鍔?
	init_drv_RCSbus();
	init_drv_RCPPM();
	
	//澶栫疆IIC椹卞姩
	init_drv_ExtIIC();
	init_drv_ExtBarometer();
	init_drv_InternalMag();
	init_drv_ExtMag();
	
	//GPS椹卞姩
	init_drv_GPS();
	init_drv_RTK();
	
	//鍏夋祦椹卞姩
	init_drv_OpticalFlow_LC302();
	init_drv_OpticalFlow_LC306();
	init_drv_OpticalFlow_JL32xx();
	init_drv_OpticalFlow_GL9306();
	
	//UWB椹卞姩
	init_drv_UWB_LinkTrack();
	
	//娴嬭窛浼犳劅鍣ㄩ┍鍔?
	init_drv_ultrasonic();
	init_drv_TFMini();
	
	//澶栭儴LED椹卞姩
	init_drv_ExtLed();
	
	//浜屾寮€鍙戞帴鍙ｉ┍鍔?
	init_drv_SDI();
}