#pragma once

#include <stdint.h>

enum CRC_POLY_SIZE
{
	CRC_POLY_SIZE_7bit,
	CRC_POLY_SIZE_8bit,
	CRC_POLY_SIZE_16bit,
	CRC_POLY_SIZE_32bit,
};

typedef struct
{
	CRC_POLY_SIZE	Size;		//多项式宽度
	uint32_t			Poly;		//多项式
	uint32_t			Init;		//初始值
	uint32_t			XOROUT;		//结果异或值
	bool 					isREFIN;	//输入数据反转
	bool					isREFOUT;	//输出数据反转
}CRC_Cfg;

void init_drv_CRC();