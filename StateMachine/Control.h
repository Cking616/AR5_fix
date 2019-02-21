#ifndef _CONTROL_H_
#define _CONTROL_H_

/******************************************************************************
* Includes
******************************************************************************/
/* Including needed modules to compile this module/procedure */
#include "Configuration.h"
#include "stm32f4xx.h"

/* application constans */

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define TORQUE_CONTROL (u32)0x0000
#define SPEED_CONTROL (u32)0x0001
#define POSITION_CONTROL (u32)0x0002
#define TRACK_CONTROL (u32)0x0003
#define OPENLOOP_CONTROL (u32)0x00A4  //(u32)0x0004
#define OPENLOOP_PWM (u32)0x00AB      //(u32)0x0010

#define NB_CONVERSIONS 16
#define ADC_RIGHT_ALIGNMENT 3

#define SQRT_3 1.732051
#define T (PWM_PERIOD * 4)
#define T_SQRT3 (u16)(T * SQRT_3)

#define SECTOR_1 (u32)1
#define SECTOR_2 (u32)2
#define SECTOR_3 (u32)3
#define SECTOR_4 (u32)4
#define SECTOR_5 (u32)5
#define SECTOR_6 (u32)6

#define divSQRT_3 (s16)0x49E6

#define BUFFER_SIZE (u8)128

#define STM32_FLASH_BASE 0x08000000

#define ADDR_FLASH_SECTOR_0 ((u32)0x08000000)   //16 Kbytes
#define ADDR_FLASH_SECTOR_1 ((u32)0x08004000)   //16 Kbytes
#define ADDR_FLASH_SECTOR_2 ((u32)0x08008000)   //16 Kbytes
#define ADDR_FLASH_SECTOR_3 ((u32)0x0800C000)   //16 Kbytes
#define ADDR_FLASH_SECTOR_4 ((u32)0x08010000)   //64 Kbytes
#define ADDR_FLASH_SECTOR_5 ((u32)0x08020000)   //128 Kbytes
#define ADDR_FLASH_SECTOR_6 ((u32)0x08040000)   //128 Kbytes
#define ADDR_FLASH_SECTOR_7 ((u32)0x08060000)   //128 Kbytes
#define ADDR_FLASH_SECTOR_8 ((u32)0x08080000)   //128 Kbytes
#define ADDR_FLASH_SECTOR_9 ((u32)0x080A0000)   //128 Kbytes
#define ADDR_FLASH_SECTOR_10 ((u32)0x080C0000)  //128 Kbytes
#define ADDR_FLASH_SECTOR_11 ((u32)0x080E0000)  //128 Kbytes

typedef struct
{
    s16 qI_Component1;
    s16 qI_Component2;
} Curr_Components;

typedef struct
{
    s16 qV_Component1;
    s16 qV_Component2;
} Volt_Components;

typedef struct
{
    s16 hCos;
    s16 hSin;
} Trig_Components;

extern volatile u8 Flag_2_ms;
extern volatile u8 Flag_10_ms;
extern volatile u8 Flag_50_ms;
extern volatile u8 Flag_100_ms;
extern volatile u8 Flag_500_ms;
extern volatile u8 Flag_1000_ms;
extern volatile int Tick_ms;

extern float Ext_Temperature;

/******************************************************************************
* Global functions
******************************************************************************/
void Led_Fault_Indication_Set(void);

void Led_Fault_Indication_Clr(void);

void gLEDState(void);

void One_ms_Tick(void);

void TB_Set_Delay(u16 hDelay);

u8 TB_delay_IsElapsed(void);

u8 crc8_4B(u32 bb);

void MagnetEncDataRead(void);

void Fault_Management(void);

void MCU_Temp_Cal(void);

void ADC_Value_Read(void);

void PWM_OUT_ENABLE(void);

void PWM_OUT_DISABLE(void);

void V_AlphaBeta_Reset(void);
void I_AlphaBeta_Reset(void);
void V_QD_Reset(void);
void I_QD_Reset(void);
void I_QD_Ref_Reset(void);

Volt_Components RevPark_Circle_Limitation(Volt_Components Volt_q_d);

void Ext_Temp_Sensor_Cal(void);

void Brake_On(void);

void Brake_Hold(void);

void Brake_Off(void);

u8 Brake_Off_Check(void);

u8 Brake_Start_Check(void);

u32 STMFLASH_ReadWord(u32 faddr);

void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u32 NumToWrite);

void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u32 NumToRead);

u8 Flash_Paramenter_Read(void);

void Flash_Paramenter_Write(void);

u8 Flash_Paramenter_Write_Verify(void);

void Flash_Parameter_Init(void);

#endif /* CONTROL */
