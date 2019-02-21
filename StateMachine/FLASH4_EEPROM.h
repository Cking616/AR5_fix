#ifndef __FLASH_4_EEPROM_H
#define __FLASH_4_EEPROM_H


/* @file
---------------------------------------------------------------------------------
<PRE>
Module			: 参数保存模块
File			: FLASH4_EEPROM.h
Relevant File	: FLASH4_EEPROM.c
Description		: 将参数保存在内存中
Author			: zhenyonghit
Rev				: 2.0
project			: AR5-JC
Company			: Asage-Robots(http://www.asage-robots.com/)
---------------------------------------------------------------------------------
comment			: $Other comment
---------------------------------------------------------------------------------
Modify History	: 
Date				Rev			Mender				Content modification
2017/01/06			V1.0		zhenyonghit			Create file		
</PRE>
---------------------------------------------------------------------------------
*/

//------------------------------- Include files -------------------------------//
#include "stm32f4xx.h"



//----------------------------------- Define ----------------------------------//
//FLASH起始地址
#define STM32_FLASH_BASE		((u32)0x08000000U)		//STM32F407VGT6 FLASH的起始地址0x08000000
#define STM32_FLASH_END			((u32)0x080FFFFFU)		//STM32F407VGT6 FLASH的终止地址0x080FFFFF
 

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0		((u32)0x08000000U)		//扇区0起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_1		((u32)0x08004000U)		//扇区1起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_2		((u32)0x08008000U)		//扇区2起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_3		((u32)0x0800C000U)		//扇区3起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_4		((u32)0x08010000U)		//扇区4起始地址, 64 Kbytes
#define ADDR_FLASH_SECTOR_5		((u32)0x08020000U)		//扇区5起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_6		((u32)0x08040000U)		//扇区6起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_7		((u32)0x08060000U)		//扇区7起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_8		((u32)0x08080000U)		//扇区8起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_9		((u32)0x080A0000U)		//扇区9起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_10	((u32)0x080C0000U)		//扇区10起始地址,128 Kbytes
#define ADDR_FLASH_SECTOR_11	((u32)0x080E0000U)		//扇区11起始地址,128 Kbytes

#define ADDR_FLASH_SECTOR(x)	ADDR_FLASH_SECTOR_##x	//addr的快速访问方式

//FLASH 扇区的大小
#define SIZE_FLASH_SECTOR_0		((u32)0x00004000U)		//扇区0大小, 16Kbytes  = 16*1024bytes  = 16384bytes  = 0x00004000bytes
#define SIZE_FLASH_SECTOR_1		((u32)0x00004000U)		//扇区1大小, 16Kbytes  = 16*1024bytes  = 16384bytes  = 0x00004000bytes
#define SIZE_FLASH_SECTOR_2		((u32)0x00004000U)		//扇区2大小, 16Kbytes  = 16*1024bytes  = 16384bytes  = 0x00004000bytes
#define SIZE_FLASH_SECTOR_3		((u32)0x00004000U)		//扇区3大小, 16Kbytes  = 16*1024bytes  = 16384bytes  = 0x00004000bytes
#define SIZE_FLASH_SECTOR_4		((u32)0x00010000U)		//扇区4大小, 64Kbytes  = 64*1024bytes  = 65536bytes  = 0x00010000bytes
#define SIZE_FLASH_SECTOR_5		((u32)0x00020000U)		//扇区5大小, 128Kbytes = 128*1024bytes = 131072bytes = 0x00020000bytes
#define SIZE_FLASH_SECTOR_6		((u32)0x00020000U)		//扇区6大小, 128Kbytes = 128*1024bytes = 131072bytes = 0x00020000bytes
#define SIZE_FLASH_SECTOR_7		((u32)0x00020000U)		//扇区7大小, 128Kbytes = 128*1024bytes = 131072bytes = 0x00020000bytes
#define SIZE_FLASH_SECTOR_8		((u32)0x00020000U)		//扇区8大小, 128Kbytes = 128*1024bytes = 131072bytes = 0x00020000bytes
#define SIZE_FLASH_SECTOR_9		((u32)0x00020000U)		//扇区9大小, 128Kbytes = 128*1024bytes = 131072bytes = 0x00020000bytes
#define SIZE_FLASH_SECTOR_10	((u32)0x00020000U)		//扇区10大小,128Kbytes = 128*1024bytes = 131072bytes = 0x00020000bytes
#define SIZE_FLASH_SECTOR_11	((u32)0x00020000U)		//扇区11大小,128Kbytes = 128*1024bytes = 131072bytes = 0x00020000bytes

#define SIZE_FLASH_SECTOR(x)	SIZE_FLASH_SECTOR_##x	//size的快速访问方式


/*向EEPROM写数据，base：扇区其实地址，offset：偏移地址，data：要写入数据*/
//#define EEPROM_WRITE_CHAR(base, offset, data)			(*((volatile char *)(base + offset)) = ((char)data))
//#define EEPROM_WRITE_UCHAR(base, offset, data)			(*((volatile unsigned char *)(base + offset)) = ((unsigned char)data))
//#define EEPROM_WRITE_SHORT(base, offset, data)			(*((volatile short *)(base + offset)) = ((short)data))
//#define EEPROM_WRITE_USHORT(base, offset, data)			(*((volatile unsigned short *)(base + offset)) = ((unsigned short)data))
//#define EEPROM_WRITE_INT(base, offset, data)			(*((volatile int *)(base + offset)) = ((int)data))
//#define EEPROM_WRITE_UINT(base, offset, data)			(*((volatile unsigned int *)(base + offset)) = ((unsigned int)data))
//#define EEPROM_WRITE_FLOAT(base, offset, data)			(*((volatile float *)(base + offset)) = ((float)data))

/*从EEPROM读数据，base：扇区其实地址，offset：偏移地址，ptr：要写入数据*/
//#define EEPROM_READ_CHAR(base, offset, ptr)				((*((char *)ptr)) = (*((volatile char *)(base + offset))))
//#define EEPROM_READ_UCHAR(base, offset, ptr)			((*((unsigned char *)ptr)) = (*((volatile unsigned char *)(base + offset))))
//#define EEPROM_READ_SHORT(base, offset, ptr)			((*((short *)ptr)) = (*((volatile short *)(base + offset))))
//#define EEPROM_READ_USHORT(base, offset, ptr)			((*((unsigned short *)ptr)) = (*((volatile unsigned short *)(base + offset))))
//#define EEPROM_READ_INT(base, offset, ptr)				((*((int *)ptr)) = (*((volatile int *)(base + offset))))
//#define EEPROM_READ_UINT(base, offset, ptr)				((*((unsigned int *)ptr)) = (*((volatile unsigned int *)(base + offset))))
//#define EEPROM_READ_FLOAT(base, offset, ptr)			((*((float *)ptr)) = (*((volatile float *)(base + offset))))





//---------------------------------- Typedef ----------------------------------//
typedef struct
{
	/*写前函数*/
	u8 (*Write_Start)(u32);

	/*向Flash——EEPROM写数据函数*/
	FLASH_Status (*Write_Char)(uint32_t, uint32_t*, int8_t);
	FLASH_Status (*Write_UChar)(uint32_t, uint32_t*, uint8_t);
	FLASH_Status (*Write_Short)(uint32_t, uint32_t*, int16_t);
	FLASH_Status (*Write_UShort)(uint32_t, uint32_t*, uint16_t);
	FLASH_Status (*Write_Int)(uint32_t, uint32_t*, int32_t);
	FLASH_Status (*Write_UInt)(uint32_t, uint32_t*, uint32_t);
	FLASH_Status (*Write_Float)(uint32_t, uint32_t*, float);

	/*从Flash——EEPROM读数据函数*/
	void (*Read_Char)(uint32_t, uint32_t*, int8_t*);
	void (*Read_UChar)(uint32_t, uint32_t*, uint8_t*);
	void (*Read_Short)(uint32_t, uint32_t*, int16_t*);
	void (*Read_UShort)(uint32_t, uint32_t*, uint16_t*);
	void (*Read_Int)(uint32_t, uint32_t*, int32_t*);
	void (*Read_UInt)(uint32_t, uint32_t*, uint32_t*);
	void (*Read_Float)(uint32_t, uint32_t*, float*);

	/*写后函数*/
	void (*Write_End)(void);
}EEPROM_t;





//------------------------------ Extern Variable ------------------------------//
extern EEPROM_t g_EEPROM;



//------------------------------ Extern Function ------------------------------//



#endif


