
/* @file
---------------------------------------------------------------------------------
<PRE>
Module			: 参数保存模块
File			: FLASH4_EEPROM.c
Relevant File	: FLASH4_EEPROM.h
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
#include "FLASH4_EEPROM.h"
#include "GlobalVariable.h"
#include "stm32f4xx_flash.h"



//------------------------------ Static Variable ------------------------------//




//------------------------------ Static Function ------------------------------//
/*写前函数*/
static u8 Write_Start(u32 sector_No);

/*向Flash——EEPROM写数据函数*/
static FLASH_Status Write_Char(uint32_t sector_No, uint32_t* offset, int8_t data);
static FLASH_Status Write_UChar(uint32_t sector_No, uint32_t* offset, uint8_t data);
static FLASH_Status Write_Short(uint32_t sector_No, uint32_t* offset, int16_t data);
static FLASH_Status Write_UShort(uint32_t sector_No, uint32_t* offset, uint16_t data);
static FLASH_Status Write_Int(uint32_t sector_No, uint32_t* offset, int32_t data);
static FLASH_Status Write_UInt(uint32_t sector_No, uint32_t* offset, uint32_t data);
static FLASH_Status Write_Float(uint32_t sector_No, uint32_t* offset, float data);

/*从Flash——EEPROM读数据函数*/
static void Read_Char(uint32_t sector_No, uint32_t* offset, int8_t* ptr);
static void Read_UChar(uint32_t sector_No, uint32_t* offset, uint8_t* ptr);
static void Read_Short(uint32_t sector_No, uint32_t* offset, int16_t* ptr);
static void Read_UShort(uint32_t sector_No, uint32_t* offset, uint16_t* ptr);
static void Read_Int(uint32_t sector_No, uint32_t* offset, int32_t* ptr);
static void Read_UInt(uint32_t sector_No, uint32_t* offset, uint32_t* ptr);
static void Read_Float(uint32_t sector_No, uint32_t* offset, float* ptr);

/*写后函数*/
static void Write_End(void);

static u8 is_Addr_In_Sector(u32 addr, u32 sector_No);
static uint32_t get_Flash_Sector(u32 sector_No);
static uint32_t get_Sector_Addr(u32 sector_No);
//static uint32_t get_Sector_Size(u32 sector_No);

//------------------------------ Global Variable ------------------------------//
EEPROM_t g_EEPROM =
{
	.Write_Start = Write_Start,

	.Write_Char = Write_Char,
	.Write_UChar = Write_UChar,
	.Write_Short = Write_Short,
	.Write_UShort = Write_UShort,
	.Write_Int = Write_Int,
	.Write_UInt = Write_UInt,
	.Write_Float = Write_Float,

	.Read_Char = Read_Char,
	.Read_UChar = Read_UChar,
	.Read_Short = Read_Short,
	.Read_UShort = Read_UShort,
	.Read_Int = Read_Int,
	.Read_UInt = Read_UInt,
	.Read_Float = Read_Float,

	.Write_End = Write_End
};


//---------------------------------- Function ---------------------------------//

/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_Start
Description		: EEPROM写操作之前操作——解锁，禁止数据缓存，擦除
Parameter     	: Parameter List
					sector_No：扇区编号0-11   
Return			: 0：擦除完成
                  1：擦除失败
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static u8 Write_Start(u32 sector_No)
{
	FLASH_Status status = FLASH_COMPLETE;				//Flash擦除返回值
	
	FLASH_Unlock();										//解锁 
	FLASH_DataCacheCmd(DISABLE);						//FLASH擦除期间,必须禁止数据缓存
	FLASH_SetLatency(FLASH_Latency_5);					//FLASH Four Latency cycles
	
	status = FLASH_EraseSector(get_Flash_Sector(sector_No), VoltageRange_3);	//VCC=2.7~3.6V之间!!
	if (FLASH_COMPLETE != status)						//发生错误了
	{
		return(1);
	}
	else												//擦除完成
	{
		return(0);
	}
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_Char
Description		: Write_Char
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					int8_t Data ：写入的数据
Return			: FLASH_Status ：flash状态
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static FLASH_Status Write_Char(uint32_t sector_No, uint32_t* offset, int8_t Data)
{
	FLASH_Status status = FLASH_COMPLETE;
	
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+1;
	
	if (FALSE != is_Addr_In_Sector(addr, sector_No))
	{
		status = FLASH_ProgramByte(addr, *((uint8_t *)(&Data)));
		return(status);
	}
	else
	{
		return(FLASH_ERROR_OPERATION);
	}
	
}


/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_UChar
Description		: Write_UChar
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					uint8_t Data：写入的数据
Return			: FLASH_Status ：flash状态
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static FLASH_Status Write_UChar(uint32_t sector_No, uint32_t* offset, uint8_t Data)
{
	FLASH_Status status = FLASH_COMPLETE;
	
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+1;
	
	if (FALSE != is_Addr_In_Sector(addr, sector_No))
	{
		status = FLASH_ProgramByte(addr, *((uint8_t *)(&Data)));
		return(status);
	}
	else
	{
		return(FLASH_ERROR_OPERATION);
	}
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_Short
Description		: Write_Short
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					int16_t Data ：写入的数据
Return			: FLASH_Status ：flash状态
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static FLASH_Status Write_Short(uint32_t sector_No, uint32_t* offset, int16_t Data)
{
	FLASH_Status status = FLASH_COMPLETE;
	
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+2;
	
	if (FALSE != is_Addr_In_Sector(addr, sector_No))
	{
		status = FLASH_ProgramHalfWord(addr, *((uint16_t *)(&Data)));
		return(status);
	}
	else
	{
		return(FLASH_ERROR_OPERATION);
	}
}


/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_UShort
Description		: Write_UShort
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					uint16_t Data ：写入的数据
Return			: FLASH_Status ：flash状态
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static FLASH_Status Write_UShort(uint32_t sector_No, uint32_t* offset, uint16_t Data)
{
	FLASH_Status status = FLASH_COMPLETE;
	
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+2;
	
	if (FALSE != is_Addr_In_Sector(addr, sector_No))
	{
		status = FLASH_ProgramHalfWord(addr, *((uint16_t *)(&Data)));
		return(status);
	}
	else
	{
		return(FLASH_ERROR_OPERATION);
	}
}


/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_Int
Description		: Write_Int
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					int32_t Data ：写入的数据
Return			: FLASH_Status ：flash状态
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static FLASH_Status Write_Int(uint32_t sector_No, uint32_t* offset, int32_t Data)
{
	FLASH_Status status = FLASH_COMPLETE;
	
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+4;
	
	if (FALSE != is_Addr_In_Sector(addr, sector_No))
	{
		status = FLASH_ProgramWord(addr, *((uint32_t *)(&Data)));
		return(status);
	}
	else
	{
		return(FLASH_ERROR_OPERATION);
	}
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_UInt
Description		: Write_UInt
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					uint32_t Data ：写入的数据
Return			: FLASH_Status ：flash状态
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static FLASH_Status Write_UInt(uint32_t sector_No, uint32_t* offset, uint32_t Data)
{
	FLASH_Status status = FLASH_COMPLETE;
	
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+4;
	
	if (FALSE != is_Addr_In_Sector(addr, sector_No))
	{
		status = FLASH_ProgramWord(addr, *((uint32_t *)(&Data)));
		return(status);
	}
	else
	{
		return(FLASH_ERROR_OPERATION);
	}
}




/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_Float
Description		: Write_Float
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					float Data ：写入的数据
Return			: FLASH_Status ：flash状态
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static FLASH_Status Write_Float(uint32_t sector_No, uint32_t* offset, float Data)
{
	FLASH_Status status = FLASH_COMPLETE;
	
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+4;
	
	if (FALSE != is_Addr_In_Sector(addr, sector_No))
	{
		status = FLASH_ProgramWord(addr, *((uint32_t *)(&Data)));
		return(status);
	}
	else
	{
		return(FLASH_ERROR_OPERATION);
	}
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Read_Char
Description		: Read_Char
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					int8_t* ptr ：地址
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void Read_Char(uint32_t sector_No, uint32_t* offset, int8_t* ptr)
{
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+1;
	
	(*ptr) = (*((volatile int8_t*)addr));
	
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Read_UChar
Description		: Read_UChar
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					uint8_t* ptr ：地址
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void Read_UChar(uint32_t sector_No, uint32_t* offset, uint8_t* ptr)
{
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+1;
	
	(*ptr) = (*((volatile uint8_t*)addr));
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Read_Short
Description		: Read_Short
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					int16_t* ptr ：地址
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void Read_Short(uint32_t sector_No, uint32_t* offset, int16_t* ptr)
{
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+2;
	
	(*ptr) = (*((volatile int16_t*)addr));
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Read_UShort
Description		: Read_UShort
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					uint16_t* ptr ：地址
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void Read_UShort(uint32_t sector_No, uint32_t* offset, uint16_t* ptr)
{
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+2;
	
	(*ptr) = (*((volatile uint16_t*)addr));
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Read_Int
Description		: Read_Int
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					int32_t* ptr ：地址
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void Read_Int(uint32_t sector_No, uint32_t* offset, int32_t* ptr)
{
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+4;
	
	(*ptr) = (*((volatile int32_t*)addr));
}




/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Read_UInt
Description		: 读取32位无符号数据
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					uint32_t* ptr ：地址
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void Read_UInt(uint32_t sector_No, uint32_t* offset, uint32_t* ptr)
{
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+4;
	
	(*ptr) = (*((volatile uint32_t*)addr));
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Read_Float
Description		: 读取单精度数据
Parameter     	: Parameter List
					uint32_t sector_No ：扇区编号
					uint32_t* offset, ：偏移值
					float* ptr ：地址
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void Read_Float(uint32_t sector_No, uint32_t* offset, float* ptr)
{
//	uint32_t data= 0;
	
	uint32_t addr = get_Sector_Addr(sector_No) + (*offset);
	
	*offset = *offset+4;
	
//	data = (*((volatile uint32_t*)addr));
	
//	(*ptr) = (*((uint32_t *)(&data)));
	
	(*ptr) = (*((volatile float*)addr));
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: Write_End
Description		: EPROM写操作之后操作——开启数据缓存、上锁
Parameter     	: Parameter List
					void
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void Write_End(void)
{
	FLASH_DataCacheCmd(ENABLE);		//FLASH擦除结束,开启数据缓存
	FLASH_Lock();					//上锁
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: is_Addr_In_Sector
Description		: 判断地址Addr是否在Sector中
Parameter     	: Parameter List
					u32 addr : 地址
					u32 sector_No ；扇区编号0-11
Return			: u8
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static u8 is_Addr_In_Sector(u32 addr, u32 sector_No)
{
	switch (sector_No)
	{
		case 0:
			return(((addr>=ADDR_FLASH_SECTOR_0)&&(addr<ADDR_FLASH_SECTOR_1))?(TRUE):(FALSE));
		case 1:
			return(((addr>=ADDR_FLASH_SECTOR_1)&&(addr<ADDR_FLASH_SECTOR_2))?(TRUE):(FALSE));
		case 2:
			return(((addr>=ADDR_FLASH_SECTOR_2)&&(addr<ADDR_FLASH_SECTOR_3))?(TRUE):(FALSE));
		case 3:
			return(((addr>=ADDR_FLASH_SECTOR_3)&&(addr<ADDR_FLASH_SECTOR_4))?(TRUE):(FALSE));
		case 4:
			return(((addr>=ADDR_FLASH_SECTOR_4)&&(addr<ADDR_FLASH_SECTOR_5))?(TRUE):(FALSE));
		case 5:
			return(((addr>=ADDR_FLASH_SECTOR_5)&&(addr<ADDR_FLASH_SECTOR_6))?(TRUE):(FALSE));
		case 6:
			return(((addr>=ADDR_FLASH_SECTOR_6)&&(addr<ADDR_FLASH_SECTOR_7))?(TRUE):(FALSE));
		case 7:
			return(((addr>=ADDR_FLASH_SECTOR_7)&&(addr<ADDR_FLASH_SECTOR_8))?(TRUE):(FALSE));
		case 8:
			return(((addr>=ADDR_FLASH_SECTOR_8)&&(addr<ADDR_FLASH_SECTOR_9))?(TRUE):(FALSE));
		case 9:
			return(((addr>=ADDR_FLASH_SECTOR_9)&&(addr<ADDR_FLASH_SECTOR_10))?(TRUE):(FALSE));
		case 10:
			return(((addr>=ADDR_FLASH_SECTOR_10)&&(addr<ADDR_FLASH_SECTOR_11))?(TRUE):(FALSE));
		case 11:
			return(((addr>=ADDR_FLASH_SECTOR_11)&&(addr<=STM32_FLASH_END))?(TRUE):(FALSE));
		default:
			return(FALSE);
	}
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: get_Flash_Sector
Description		: get_Flash_Sector
Parameter     	: Parameter List
					void
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static uint32_t get_Flash_Sector(u32 sector_No)
{
	switch (sector_No)
	{
		case 0:
			return(FLASH_Sector_0);
		case 1:
			return(FLASH_Sector_1);
		case 2:
			return(FLASH_Sector_2);
		case 3:
			return(FLASH_Sector_3);
		case 4:
			return(FLASH_Sector_4);
		case 5:
			return(FLASH_Sector_5);
		case 6:
			return(FLASH_Sector_6);
		case 7:
			return(FLASH_Sector_7);
		case 8:
			return(FLASH_Sector_8);
		case 9:
			return(FLASH_Sector_9);
		case 10:
			return(FLASH_Sector_10);
		case 11:
			return(FLASH_Sector_11);
		default:
			return(0xFFFFFFFF);
	}
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: get_Sector_Addr
Description		: get_Sector_Addr
Parameter     	: Parameter List
					void
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static uint32_t get_Sector_Addr(u32 sector_No)
{
	switch (sector_No)
	{
		case 0:
			return(ADDR_FLASH_SECTOR_0);
		case 1:
			return(ADDR_FLASH_SECTOR_1);
		case 2:
			return(ADDR_FLASH_SECTOR_2);
		case 3:
			return(ADDR_FLASH_SECTOR_3);
		case 4:
			return(ADDR_FLASH_SECTOR_4);
		case 5:
			return(ADDR_FLASH_SECTOR_5);
		case 6:
			return(ADDR_FLASH_SECTOR_6);
		case 7:
			return(ADDR_FLASH_SECTOR_7);
		case 8:
			return(ADDR_FLASH_SECTOR_8);
		case 9:
			return(ADDR_FLASH_SECTOR_9);
		case 10:
			return(ADDR_FLASH_SECTOR_10);
		case 11:
			return(ADDR_FLASH_SECTOR_11);
		default:
			return(0xFFFFFFFF);
	}
}


/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: get_Sector_Size
Description		: get_Sector_Size
Parameter     	: Parameter List
					void
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
//static uint32_t get_Sector_Size(u32 sector_No)
//{
//	switch (sector_No)
//	{
//		case 0:
//			return(SIZE_FLASH_SECTOR_0);
//		case 1:
//			return(SIZE_FLASH_SECTOR_1);
//		case 2:
//			return(SIZE_FLASH_SECTOR_2);
//		case 3:
//			return(SIZE_FLASH_SECTOR_3);
//		case 4:
//			return(SIZE_FLASH_SECTOR_4);
//		case 5:
//			return(SIZE_FLASH_SECTOR_5);
//		case 6:
//			return(SIZE_FLASH_SECTOR_6);
//		case 7:
//			return(SIZE_FLASH_SECTOR_7);
//		case 8:
//			return(SIZE_FLASH_SECTOR_8);
//		case 9:
//			return(SIZE_FLASH_SECTOR_9);
//		case 10:
//			return(SIZE_FLASH_SECTOR_10);
//		case 11:
//			return(SIZE_FLASH_SECTOR_11);
//		default:
//			return(0xFFFFFFFF);
//	}
//}



