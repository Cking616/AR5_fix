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

#ifdef SYSVIEW_DEBUG
#include "SEGGER_SYSVIEW.h"
#endif

uint32_t _reboot_cookie __attribute__ ((section (".noinit")));
extern char __initial_sp;
//extern char _estack; // provided by the linker script

int g_is_upgrade = 0;
// Gets called from the startup assembly code
void early_start_checks(void) {
  if(_reboot_cookie == 0xDEADFE75) {
    /* The STM DFU bootloader enables internal pull-up resistors on PB10 (AUX_H)
    * and PB11 (AUX_L), thereby causing shoot-through on the brake resistor
    * FETs and obliterating them unless external 3.3k pull-down resistors are
    * present. Pull-downs are only present on ODrive 3.5 or newer.
    * On older boards we disable DFU by default but if the user insists
    * there's only one thing left that might save it: time.
    * The brake resistor gate driver needs a certain 10V supply (GVDD) to
    * make it work. This voltage is supplied by the motor gate drivers which get
    * disabled at system reset. So over time GVDD voltage _should_ below
    * dangerous levels. This is completely handwavy and should not be relied on
    * so you are on your own on if you ignore this warning.
    *
    * This loop takes 5 cycles per iteration and at this point the system runs
    * on the internal 16MHz RC oscillator so the delay is about 2 seconds.
    */
    for (size_t i = 0; i < (16000000UL / 5UL * 2UL); ++i) {
      __NOP();
    }
    _reboot_cookie = 0xDEADBEEF;
  }

  /* We could jump to the bootloader directly on demand without rebooting
  but that requires us to reset several peripherals and interrupts for it
  to function correctly. Therefore it's easier to just reset the entire chip. */
  if(_reboot_cookie == 0xDEADBEEF) {
    _reboot_cookie = 0xCAFEFEED;  //Reset bootloader trigger	
		__set_MSP((uintptr_t)&__initial_sp);
    //__set_MSP((uintptr_t)&_estack);
    // http://www.st.com/content/ccc/resource/technical/document/application_note/6a/17/92/02/58/98/45/0c/CD00264379.pdf/files/CD00264379.pdf
    void (*builtin_bootloader)(void) = (void (*)(void))(*((uint32_t *)0x1FFF0004));
    builtin_bootloader();
  }

  /* The bootloader might fail to properly clean up after itself,
  so if we're not sure that the system is in a clean state we
  just reset it again */
  if(_reboot_cookie != 42) {
    _reboot_cookie = 42;
    NVIC_SystemReset();
  }
}

void __init_delay_ms(int ms)
{
	static u32 tickcnt;
	static u16 delaycnt;
	tickcnt =0;
	delaycnt = 0;
	while(delaycnt < ms)
	{
		tickcnt = SysTick->VAL;
		while(1)
		{
			if(tickcnt > SysTick->VAL)
			{
				if(tickcnt - SysTick->VAL >= 168000) break;
			}
			else
			{
				if(((1 << 24) - SysTick->VAL + tickcnt) >= 168000) break;
			}
		}
		delaycnt++;
	}
}

void enter_dfu_mode() {
    //__asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
		__disable_irq();
    _reboot_cookie = 0xDEADBEEF;
    NVIC_SystemReset();
}

int main(void)
{		
		SysTick->CTRL |= 0x0004; 
		SysTick->LOAD = 0x00FFFFFF;
		SysTick->CTRL |= 0x0001;
		

		__init_delay_ms(200);

		GPIO_Config();

#ifdef  ETHERCAT_ENABLE
	#ifdef ETHERCAT_RUN
		HW_Init();
		MainInit();
		CiA402_Init();
		APPL_GenerateMapping(&nPdInputSize,&nPdOutputSize);
		bRunApplication = TRUE;
	#endif
#endif	

		ENC_Config();	
		
		DMA_Config();
					
#ifdef ETHERCAT_ENABLE
		SPI2_Config();
#else
		SPI3_Config();	
#endif
		
		TIM_Config();
		
		PWM_Config();	
		
		ADC_Config();
				
#ifdef  USB_ENABLE
	    USBConfig();                        // USB  ÅäÖÃ
	    USBInitial();
#endif

#ifdef SYSVIEW_DEBUG
		SEGGER_SYSVIEW_Conf();
#endif

		NVIC_Config();

#ifndef ETHERCAT_ENABLE
		g_stJC2JD.init(BAUDRATE_RS485);	
#endif

		__init_delay_ms(50);

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
		
		#ifndef ETHERCAT_ENABLE
		if(guc_RS485_Flag == 1)
		{
			g_stJC2JD.Inst_Process();
		}
		#endif	

		#endif

#endif 

#if 1
		if ((gsM1_Ctrl.eState != RUN) && (gsM1_Ctrl.eState != ALIGN))
		{
			if(stParaList.iParaRecover == 1)
			{
				PararecoverProcess();
				stParaList.iParaRecover = 0;
			}
			if (stParaList.iParaSave == 1)
			{
				ParaSaveProcess();
				stParaList.iParaSave = 0;
			}

			if ((stParaList.iSysReset == 1) && (stParaList.iParaRecover == 0) && (stParaList.iParaSave == 0))
			{
				SysResetProcess();
				stParaList.iSysReset = 0;
			}
		}
#endif

#ifdef  USB_ENABLE
	    if(Flag_1000_ms == 1)
		{
			if(lResetCnt != 0)
			{
			    lResetCnt++;
			}
			if(lResetCnt >= 4) //3S
            {
                //ResertUsb();
                lResetCnt = 0;
            }
		}
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
				#ifdef SYSVIEW_DEBUG
				if(gsM1_Drive.sFaultId.R > 0)
				{
					if(gsM1_Drive.sFaultId.B.UnderDCBusVoltage == 1)
					{
						SEGGER_SYSVIEW_Error("E: Under DCBusVoltage");
					}

					if(gsM1_Drive.sFaultId.B.OverDCBusVoltage == 1)
					{
						SEGGER_SYSVIEW_Error("E: Over DCBusVoltage");
					}

					if(gsM1_Drive.sFaultId.B.OverCurrent == 1)
					{
						SEGGER_SYSVIEW_Error("E: Over Current");
					}

					if(gsM1_Drive.sFaultId.B.Overload == 1)
					{
						SEGGER_SYSVIEW_Error("E: Overload");
					}

					if(gsM1_Drive.sFaultId.B.OverHeat == 1)
					{
						SEGGER_SYSVIEW_Error("E: Over Heat");
					}

					if(gsM1_Drive.sFaultId.B.OverSpeed == 1)
					{
						SEGGER_SYSVIEW_Error("E: Over Speed");
					}

					if(gsM1_Drive.sFaultId.B.LockRotor == 1)
					{
						SEGGER_SYSVIEW_Error("E: LockRotor");
					}

					if(gsM1_Drive.sFaultId.B.LossPhase == 1)
					{
						SEGGER_SYSVIEW_Error("E: Loss Phase");
					}

					if(gsM1_Drive.sFaultId.B.OffCancError == 1)
					{
						SEGGER_SYSVIEW_Error("E: Off CancError");
					}

					if(gsM1_Drive.sFaultId.B.MagnetEncoderError == 1)
					{
						SEGGER_SYSVIEW_Error("E: Magnet Encoder Error");
					}

					if(gsM1_Drive.sFaultId.B.OpticalEncoderError == 1)
					{
						SEGGER_SYSVIEW_Error("E: Optical Encoder Error");
					}

					if(gsM1_Drive.sFaultId.B.BrakeError == 1)
					{
						SEGGER_SYSVIEW_Error("E: Brake Error");
					}

					if(gsM1_Drive.sFaultId.B.OppositeDirection == 1)
					{
						SEGGER_SYSVIEW_Error("E: Opposite Direction");
					}

					if(gsM1_Drive.sFaultId.B.StartUpFail == 1)
					{
						SEGGER_SYSVIEW_Error("E: StartUp Fail");
					}

					if(gsM1_Drive.sFaultId.B.MagnetEncoderCRCError == 1)
					{
						SEGGER_SYSVIEW_Error("E: Magnet EncoderCRC Error");
					}

					if(gsM1_Drive.sFaultId.B.FlashError == 1)
					{
						SEGGER_SYSVIEW_Error("E: Flash Error");
					}

					if(gsM1_Drive.sFaultId.B.PhaseWShortCirciut == 1)
					{
						SEGGER_SYSVIEW_Error("E: PhaseW ShortCirciut");
					}

					if(gsM1_Drive.sFaultId.B.PowerSupplyOff == 1)
					{
						SEGGER_SYSVIEW_Error("E: Power Supply Off");
					}

					if(gsM1_Drive.sFaultId.B.BrakeStuckError == 1)
					{
						SEGGER_SYSVIEW_Error("E: Brake Stuck Error");
					}
				}
				else{
					SEGGER_SYSVIEW_Print("Running!");
				}
				#endif

				#if AUTORUN == 1	

					if(autocalibcnt)
					{
						autocalibcnt--;
						if(autocalibcnt == 0)
						{
							gsM1_Ctrl.uiCtrl = 1;
							gsM1_Drive.uw16CtrlMode = SPEED_CONTROL;
						}
					}	
					
					if(autocalibcnt == 0)
					{
						autoruncnt--;
						if(autoruncnt == 0)
						{
							gsM1_Drive.uw16CtrlMode = SPEED_CONTROL;
							gsM1_Drive.sSpeed.f32SpeedCmd = AUTORUN_SPEED;
						}
					}
					
				#endif
				
				if(g_is_upgrade)
				{
					gsM1_Ctrl.uiCtrl = SM_CTRL_FAULT;
					__init_delay_ms(200);
					__init_delay_ms(200);
					__init_delay_ms(200);
					__init_delay_ms(200);
					__init_delay_ms(200);
					enter_dfu_mode();
				}
				Flag_1000_ms = 0;
			}
		}
	
}

