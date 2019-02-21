/* @file
---------------------------------------------------------------------------------
<PRE>
Module			: 参数保存模块
File			: EEPROM_RW.c
Relevant File	: EEPROM_RW.h
Description		: EEPROM数据的读取和写入
Author			: zhenyonghit
Rev				: 2.0
project			: AR5-JC
Company			: Asage-Robots(http://www.asage-robots.com/)
---------------------------------------------------------------------------------
comment			: $Other comment
---------------------------------------------------------------------------------
Modify History	: 
Date				Rev			Mender				Content modification
2017/01/18			V1.0		zhenyonghit			Create file		
</PRE>
---------------------------------------------------------------------------------
*/
//------------------------------- Include files -------------------------------//
#include "EEPROM_RW.h"

//#include "FLASH4_EEPROM.h"

#include "RS232_USART.h"
#include "GlobalVariable.h"
//#include "JC2HCCommunication.h"
#include "FLASH4_EEPROM.h"

//------------------------------ Static Variable ------------------------------//

const int i_IAP_AddrNo = 2;
const int i_LastAngleAddrNo = 3;
const int i_ParaAddrNo = 10;



//------------------------------ Static Function ------------------------------//
//static void SaveLastAngle(void);												//保存变量
//static void ReadLastAngle(void);												//读取变量
//static void SavePara(void);													//将参数保存到FLASGH中
//static void ReadPara(void);													//从FLASH中读取参数

static void Write_IAP_Flag(u32 *pbuf, u32 Num);

//static void SaveLastAngleToflash(float Data);									//角度保存
//static float GetLastAngleFromFlash(void);										//角度保存

//static void SaveParaToflash(float Data);										//参数保存
//static float GetParaFromFlash(void);											//参数读取
//------------------------------ Global Variable ------------------------------//
EEPROM_RW_t g_stEEPROM_RW =
{
//	.m_SaveLastAngle = SaveLastAngle,
//	.m_ReadLastAngle = ReadLastAngle,
//	.m_SavePara	= SavePara,														//将参数保存到FLASGH中
//	.m_ReadPara	= ReadPara,														//从FLASH中读取参数
	.m_Write_IAP_Flag = Write_IAP_Flag
};


IAP_TABLE_t g_stIAPFlag = 
{
	.iap_flag.IAP_Enable_Inst = NEED_IAP,
	.iap_flag.IAP_VREF_Inst = VERF_INST
};

//---------------------------------- Function ---------------------------------//

/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: saveVariable
Description		: saveVariable——将变量保存到EEPROM中
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
//static void saveVariable(void)
//{
//	int i=0;
//	
//	static Boolean_t OnceDo = FALSE;
//	if(FALSE == OnceDo)
//	{
//	/*写数据开始*/
//		g_EEPROM.Write_Start(11);				//准备向Flash_Sector_11读写数据
//		
//		/*写数据*/
//		for(i=0; i<JOINT_NUM; ++i)
//		{
//			//关机前关节最后位置
//			ParaSave(g_pEncoder[i]->m_fLastAngle);
//		}

//		/*写数据结束*/
//		g_EEPROM.Write_End();
//		OnceDo = TRUE;
//		
//		my_printf("SOK\r\n");
//	}	
//}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: readVariable
Description		: readVariable——从EEPROM中读取变量
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
//static void readVariable(void)
//{
//	int i=0;
//	
//	static Boolean_t OnceDo = FALSE;
//	if(FALSE == OnceDo)
//	{
//		/*读数据*/
//		for(i=0; i<JOINT_NUM; ++i)
//		{
//			//关机前关节最后位置
//			g_pEncoder[i]->m_fLastAngle = (float)ParaSavedRead();
//		}

//		OnceDo = TRUE;
//		
//		my_printf("Read Variable\r\n");
//	}
//}
/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: SavePara
Description		: SavePara——将参数保存到FLASGH中
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
//static void SavePara(void)														//将参数保存到FLASGH中
//{
//	int i=0;
//	static Boolean_t OnceDo = FALSE;
//	if(FALSE == OnceDo)
//	{
//	/*写数据开始*/
//		g_EEPROM.Write_Start(i_ParaAddrNo);										//准备向Flash_Sector_4读写数据
//		
//		/*写数据*/
//		for(i=0; i<JOINT_NUM; ++i)
//		{
//			 SaveParaToflash(g_gfAngleZero[i]);									//参数保存
//		}
//		
//		/*写数据结束*/
//		g_EEPROM.Write_End();
//		OnceDo = TRUE;
//		
//		my_printf("save para\r\n");
//	}
//}
/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: ReadPara
Description		: ReadPara——从FLASH中读取参数
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
//static void ReadPara(void)														//从FLASH中读取参数
//{
//	int i=0;
//	
//	static Boolean_t OnceDo = FALSE;
//	if(FALSE == OnceDo)
//	{
//		/*读数据*/
//		for(i=0; i<JOINT_NUM; ++i)
//		{
//																				//保存参数值
//			g_gfAngleZero[i] = (float)GetParaFromFlash();
//			g_pEncoder[i]->m_fZeroAngle = g_gfAngleZero[i];
//		}

//		OnceDo = TRUE;
//		
//		my_printf("Read para\r\n");
//	}
//}

/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: write_IAP_Flag
Description		: 向10扇区写入标志位
Parameter     	: Parameter List
					mode_Flag ：1——JC程序更新，2——JD程序更新，3——JC参数更新，JD——JD参数更新  
					equip_No  ：0——JC设备编号，1-6——JD设备编号
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

static void Write_IAP_Flag(u32 *pbuf, u32 Num)
{
	int i = 0;								//关节遍历参数，0——JOINT_NUM-1

	u32 offset = 0;							//用于做偏移

	FLASH_Status status = FLASH_COMPLETE;	//用于返回EEPROM操作后状态
//	my_printf("romstart");
	/*写数据开始*/
	g_EEPROM.Write_Start(i_IAP_AddrNo);		//准备向Flash_Sector_2读写数据

	for(i = 0;i<Num;i++)
	{
		if (FLASH_COMPLETE == status)
		{
			status = g_EEPROM.Write_UInt(i_IAP_AddrNo, &offset,*(pbuf+i));
		}
	}
	/*写数据结束*/
	g_EEPROM.Write_End();					//EEPROM写数据结束
//	my_printf("romok");
}




/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: main
Description		: 任务调用 
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
//static void ParaSave(float Data)
//{

//	static u32 offset = 0;
//	FLASH_Status status = FLASH_COMPLETE;																			//向Flash_Sector_11读写数据
//	if(status==FLASH_COMPLETE)
//	{
//		if(FLASH_COMPLETE != g_EEPROM.Write_Float(11, &offset,(float)Data))//写入数据
//		{
//			my_printf("Flash Write Error!\r\n");
//			return;												//写入异常
//		}
//	}
//}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: main
Description		: 任务调用 
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
//static float ParaSavedRead(void)
//{
//	static u32 offset = 0;
//	float s_fnum =0;
//	g_EEPROM.Read_Float(11, &offset, &s_fnum);
//	return s_fnum;
//}
/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: main
Description		: 任务调用 
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

//static void SaveParaToflash(float Data)										//参数保存
//{
//	static u32 offset = 0;
//	FLASH_Status status = FLASH_COMPLETE;																			//向Flash_Sector_11读写数据
//	if(status==FLASH_COMPLETE)
//	{
//		if(FLASH_COMPLETE != g_EEPROM.Write_Float(i_ParaAddrNo, &offset,(float)Data))//写入数据
//		{
//			my_printf("Flash Write Error!\r\n");
//			return;												//写入异常
//		}
//	}
//}
/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: main
Description		: 任务调用 
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
//static float GetParaFromFlash(void)											//参数读取
//{
//	static u32 offset = 0;
//	float s_fnum =0;
//	g_EEPROM.Read_Float(i_ParaAddrNo, &offset, &s_fnum);
//	return s_fnum;
//}
