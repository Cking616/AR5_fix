#ifndef __EEPROM_R_W_H
#define __EEPROM_R_W_H

/* @file
---------------------------------------------------------------------------------
<PRE>
Module			: 参数保存模块
File			: EEPROM_RW.h
Relevant File	: EEPROM_RW.c
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
2017/01/18  		V1.0  		zhenyonghit			Create 	file	
</PRE>
---------------------------------------------------------------------------------
*/

//------------------------------- Include files -------------------------------//

#include "GlobalVariable.h"


//----------------------------------- Define ----------------------------------//




#define NORMAL_FRAME_HEAD  (0xA2)                      //Õý³£Êý¾ÝÖ¡°üÍ·
#define LAST_FRAME_HEAD    (0xA3)                      //×îºóÒ»Ö¡Êý¾Ý°ü
 
#define CMD_RESP		    (0XA4)
#define IAP_DATA_LENGTH    (128)                       //IAPÊý¾ÝÖ¡³¤¶È
#define FIRST_DATA_FRAME   (1)

#define APP_START_ADDR     (0x08010000)                //APPÆðÊ¼µØÖ·
#define IAP_FLAG_ADDR      (0x08008000)                //IAP Flag´æ·ÅµØÖ·,ÉÈÇø10 



#define VERF_INST        (0xAAAAAAAA)

#define NEED_IAP         (0x55556666)
#define NOT_NEED_IAP     (0x77778888)



typedef struct 
{
	uint32_t IAP_Enable_Inst;
	uint32_t IAP_VREF_Inst;
}IAP_FLAG_t;

//---------------------------------- Typedef ----------------------------------//

typedef union
{
	uint32_t buf[2];
	IAP_FLAG_t iap_flag;
}IAP_TABLE_t;

typedef struct 
{
//	void (*saveVariable)(void);						//将变量保存到EEPROM中
//	void (*m_SaveLastAngle)(void);						//将变量保存到EEPROM中
//	void (*m_ReadLastAngle)(void);						//从EEPROM中读取变量
//	
//	void (*m_SavePara)(void);							//将参数保存到FLASGH中
//	void (*m_ReadPara)(void);							//从FLASH中读取参数
	
	void (*m_Write_IAP_Flag)(u32*, u32);				//向10扇区写入IAP标志位
}EEPROM_RW_t;


//------------------------------ Extern Variable ------------------------------//
extern EEPROM_RW_t g_stEEPROM_RW;
extern IAP_TABLE_t g_stIAPFlag;


//------------------------------ Extern Function ------------------------------//


//extern float f_RW_V_Ctrl_VL_KD[JOINT_NUM];						//速度控制模式的速度环PID控制比例增益系数Kd


//------------------------------ Extern Function ------------------------------//


#endif

