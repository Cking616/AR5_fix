#include "main.h"
#include "ARM_MATH.h" 
#include "mb.h"
#include <stm32f4xx.h>
#include "Usart.h"

#include "JC2JDCommunication.h"

#include "GlobalVariable.h"
#include "RS485_JC2JD_ModBus.h"

/***********************************************************************/
#include "Configuration.h"
#include "Control.h"
#include "M1_statemachine.h"
#include "Motor_Drive.h"
#include "include_c.h"

u32 tickcnt;
u16 delaycnt;

int main(void)
{ 
		SysTick->CTRL |= 0x0004; 
		SysTick->LOAD = 0x00FFFFFF;
		SysTick->CTRL |= 0x0001;
		
		while(delaycnt < 200)
		{
			tickcnt = SysTick->VAL;
			while(tickcnt - SysTick->VAL < 168000)
			{

			}
			
			delaycnt++;
		}
		
		delaycnt = 0;
	
		GPIO_Config();
		

		#ifdef  ETHERCAT_ENABLE
			HW_Init();
			MainInit();
			CiA402_Init();
			APPL_GenerateMapping(&nPdInputSize,&nPdOutputSize);
			bRunApplication = TRUE;
		#endif
		
		ENC_Config();	
		
		DMA_Config();
		
		ADC_Config();		
		#ifdef ETHERCAT_ENABLE
		SPI2_Config();
		#else
		SPI3_Config();	
		#endif
		
		PWM_Config();		
		
		TIM_Config();
				
		NVIC_Config();

		g_stJC2JD.init(BAUDRATE_RS485);

		
		while(delaycnt < 200)
		{
			tickcnt = SysTick->VAL;
			while(tickcnt - SysTick->VAL < 168000)
			{
			}
			
			delaycnt++;
		}
		
		delaycnt = 0;

				

#if MAGNET_ENCODER_ALERT_ON == 1		
		PositionCtrlInit();
#endif		
		
#if AUTORUN == 1

		u8 autoruncnt = 10;
		u8 autocalibcnt = 5;

#endif

		

		initGlobal();
		

    while(1)
    { 				
				
       
#if Modbus_RTU_ENABLE ==1

			eMBPoll();

#else 

		#if SIN_POSITION_TEST == 0
			
		if(guc_RS485_Flag == 1)
		{
			g_stJC2JD.Inst_Process();
		}
			
		#endif

#endif 
					
			if(Flag_10_ms == 1)
			{
				#ifdef  ETHERCAT_ENABLE
					if(bRunApplication == TRUE)
					{
						MainLoop();
					}
				#endif								
				Flag_10_ms = 0;
			}
			
			if(Flag_50_ms == 1)
			{
				Flag_50_ms = 0;
			}
			
			if(Flag_100_ms == 1)
			{			
				MCU_Temp_Cal();				
				Ext_Temp_Sensor_Cal();
				Flag_100_ms = 0;
			}
			
			if(Flag_500_ms == 1)
			{
				if(gsM1_Drive.sFaultId.B.OverCurrent == 0)
				{
					gLEDState();
				}
				Flag_500_ms = 0;
			}
			
			if(Flag_1000_ms == 1)
			{
				
				#if AUTORUN == 1	

					if(autocalibcnt)
					{
						autocalibcnt--;
						if(autocalibcnt == 0)
						{
							gsM1_Ctrl.uiCtrl = 8;
						}
					}	
					
					if((autoruncnt != 0)&&(autocalibcnt == 0)&&(gsM1_Drive.uw16CtrlMode == SPEED_CONTROL))
					{
						autoruncnt--;
						if(autoruncnt == 0)
						{
							gsM1_Ctrl.uiCtrl = 1;

							gsM1_Drive.sSpeed.f32SpeedCmd = AUTORUN_SPEED;
						}
					}
					
				#endif
					
				Flag_1000_ms = 0;
			}

		}
	
}

