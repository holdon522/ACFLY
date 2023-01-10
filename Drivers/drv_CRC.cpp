#include "drv_CRC.hpp"
#include "Basic.hpp"

#define CRC_REV_IN_None (0<<5)
#define CRC_REV_IN8 (1<<5)
#define CRC_REV_IN16 (2<<5)
#define CRC_REV_IN32 (3<<5)

#define CRC_REV_OUT_None (0<<7)
#define CRC_REV_OUT (1<<7)

#define CRC_POLYSIZE_32bit (0<<3)
#define CRC_POLYSIZE_16bit (1<<3)
#define CRC_POLYSIZE_8bit (2<<3)
#define CRC_POLYSIZE_7bit (3<<3)
#define CRC_RESET (1<<0)

#define CRC_DR8 (*(uint8_t*)&CRC->DR)
#define CRC_DR16 (*(uint16_t*)&CRC->DR)
#define CRC_DR32 (*(uint32_t*)&CRC->DR)
static volatile uint16_t ddcrc;

static inline bool calc_crc( CRC_Cfg cfg, const uint8_t* data, uint16_t length )
{
	//配置CRC
	uint32_t CRC_CR = 0;
	if( cfg.isREFOUT )
		CRC_CR |= CRC_REV_OUT;
	if( cfg.isREFIN )
		CRC_CR |= CRC_REV_IN8;
	switch( cfg.Size )
	{
		case CRC_POLY_SIZE_7bit:
			CRC_CR |= CRC_POLYSIZE_7bit;
			break;
		case CRC_POLY_SIZE_8bit:
			CRC_CR |= CRC_POLYSIZE_8bit;
			break;
		case CRC_POLY_SIZE_16bit:
			CRC_CR |= CRC_POLYSIZE_16bit;
			break;
		case CRC_POLY_SIZE_32bit:
			CRC_CR |= CRC_POLYSIZE_32bit;
			break;
	}
	CRC->CR = CRC_CR | CRC_RESET;
	CRC->POL = cfg.Poly;
	CRC->INIT = cfg.Init;
	
	//开始DMA传输CRC数据
	DMA2->HIFCR = (1<<11);
	DMA2_Stream5->PAR = (uint32_t)data;
	DMA2_Stream5->NDTR = length;
	DMA2_Stream5->CR |= 1;
}

bool CRC_Calc( CRC_Cfg cfg, const uint8_t* data, uint16_t length )
{
	uint16_t BLOCKSIZE = getFlashSectorSize();
	if( BLOCKSIZE==0 )
		return LFS_ERR_INVAL;
		
	uint16_t PAGESIZE = getFlashPageSize();
	if( off + size > BLOCKSIZE )
		return LFS_ERR_INVAL;
	uint16_t page_addr = off / PAGESIZE;
	uint16_t pages = size / PAGESIZE;
	
  int res = LFS_ERR_CORRUPT;
  uint32_t timer;

#ifdef DCACHE_SIZE
	if( isDMABuf( (uint32_t)data ) )
	{	//缓冲区为非cache区域
		//直接用dma发送
#endif
		if( FlashProgramPages( (uint8_t*)data, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	
#ifdef DCACHE_SIZE
	}
	else if( ((uint32_t)buff & 0x1f) == 0 )
	{	//缓冲区32字节对齐
		//直接用dma发送
		
		//把Cache内容写入内存
		SCB_CleanDCache_by_Addr((uint32_t*)buff, size);
		if( FlashProgramPages( (uint8_t*)buff, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	}
	else
	{	//缓冲区非32字节对齐
		//把数据先复制到内置缓冲区再发送	
		int i;
    uint8_t ret;
		
		memcpy((void *)scratch, (void *)buff, size);
		if( FlashProgramPages( (uint8_t*)scratch, block, page_addr, pages, Flash_TIMEOUT ) == true )
		{
			res = 0;
		}
	}
#endif
  return res;
}

void init_drv_CRC()
{
	//打开CRC时钟
	RCC->AHB4ENR |= (1<<19);
	//打开DMA2时钟
	RCC->AHB1ENR |= (1<<1);
	os_delay(0.01);
	
	CRC->CR = CRC_REV_OUT | CRC_REV_IN8 | CRC_POLYSIZE_16bit | CRC_RESET;
	CRC->POL = 0X1021;
	CRC->INIT = 0;
	
	//存储器-外设 MEM-MEM
	DMA2_Stream5->CR = (0<<13) | (0<<11) | (1<<9) | (2<<6);
	DMA2_Stream5->M0AR = (uint32_t)&CRC->DR;

	Static_AXIDMABuf uint8_t dd[] = { 0x31, 0x35, 0xf1, 0x15 };
	DMA2->HIFCR = (1<<11);
	DMA2_Stream5->PAR = (uint32_t)dd;
	DMA2_Stream5->NDTR = 4;
	DMA2_Stream5->CR |= 1;
	
	while( (DMA2->HISR & (1<<11)) == 0 );
//	CRC_DR8 = 0x31;
//	CRC_DR8 = 0x35;
//	CRC_DR8 = 0xf1;
//	CRC_DR8 = 0x15;
	
	ddcrc = CRC_DR16;
	ddcrc ^= 0x0;
}