/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-September-2013
  * @brief   Main Interrupt Service Routines.
  *      This file provides template for all exceptions handler and 
  *      peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *    http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

#include "GlobalVariable.h"
#include "main.h"
#include "math.h"
#include "stm32f4xx_conf.h"

#include "Configuration.h"
#include "Control.h"
#include "M1_statemachine.h"
#include "Motor_Drive.h"
#include "StateMachine.h"
#include "include_c.h"

#ifdef SYSVIEW_DEBUG
#include "SEGGER_SYSVIEW.h"
#endif

extern s32 Turn_Number;
extern s16 Electrical_Angle;

float tim4_duration;
s32 tim4_duration_start;

s32 ADC_duration;
float ADC_duration_max, ADC_duration_ratio;
s32 ADC_duration_start;

extern USB_OTG_CORE_HANDLE USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler(USB_OTG_CORE_HANDLE *pdev);

void NMI_Handler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

void HardFault_Handler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) {
    }
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) {
    }
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) {
    }
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  
void SysTick_Handler(void)
{
}
*/

/******************************************************************************/
/*         STM32F4xx Peripherals Interrupt Handlers           */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                           */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

int g_motor_position_cmd = 0;
int g_motor_speed = 0;
int g_motor_torque = 0;

//extern float f32Idh_table[180];
int g_Idh_table = 0;
int table_index = 0;

void TIM4_IRQHandler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    //g_Idh_table = f32Idh_table[table_index] * 1000;
    table_index = (table_index + 1) % 180;
    g_motor_position_cmd = (int)(gsM1_Drive.sPositionControl.f32PositionCmd * 100) & 0xFFFFFFFF;
    g_motor_speed = (int)(gsM1_Drive.sSpeed.f32SpeedFilt * 100);
    g_motor_torque = (short)(gsM1_Drive.sFocPMSM.sIDQ.f32Q * Kt * 10000.0f);
    if (SET == TIM_GetITStatus(TIM4, TIM_IT_Update)) {
        if (gsM1_Drive.uw16CtrlMode != OPENLOOP_PWM) {
            Fault_Management();
        }

        One_ms_Tick();

        Enc_Speed_Cal();

        MagnetEncDataRead();
      
        geM1_StateRunLoop = SLOW;

        SM_StateMachine(&gsM1_Ctrl);

#ifdef USB_ENABLE
        USBD_OTG_ISR_Handler(&USB_OTG_dev);
        USB_Reply_Func();
#endif

        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

void DMA1_Stream0_IRQHandler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
#if RENISHAW == 1

    if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0)) {
        SPI_Cmd(SPI3, DISABLE);

        GPIO_SetBits(GPIOA, GPIO_Pin_15);

        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
    }

#else

    if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0)) {
        SPI_Cmd(SPI3, DISABLE);

        GPIO_ResetBits(GPIOA, GPIO_Pin_15);

        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
    }

#endif
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

void DMA1_Stream3_IRQHandler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3)) {
        SPI_Cmd(SPI2, DISABLE);

        GPIO_SetBits(GPIOB, GPIO_Pin_12);

        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);

        //table_index++;
    }
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}

#if 0
int g_last_out_time = 0;
int g_this_in_time = 0;
int g_max_adc_time = 0;
int g_max_adc_delta = 0;
int lDeltaTim1 = 0;
#endif

void ADC_IRQHandler(void) {
#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordEnterISR();
#endif

    if ((ADC1->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC) {
#if 0
    g_this_in_time = SysTick->VAL;
	int lStartTim  = SysTick->VAL;
    
    if(g_last_out_time >= g_this_in_time)
    {
    	lDeltaTim1 = g_last_out_time - g_this_in_time;
    }
    else
    {
        lDeltaTim1 = (1 << 24) - g_this_in_time + g_last_out_time;
    }

    if(lDeltaTim1 > g_max_adc_delta)
    {
        g_max_adc_delta = lDeltaTim1;
    }
#endif

        ADC_Value_Read();

        geM1_StateRunLoop = FAST;

        SM_StateMachine(&gsM1_Ctrl);

        geM1_StateRunLoop = SLOW;
#ifdef USB_ENABLE

        if ((stScopePara.iCommType == COMM_USB_MODE) &&
            (stScopePara.iTransEnable == 1)) {
            USB_ScopeDataBuffer();
        }
#endif

        ADC1->SR = ~(u32)ADC_FLAG_JEOC;

#if 0
	int lEndTim   = SysTick->VAL;
	int lDeltaTim = 0;
	if(lStartTim >= lEndTim)
    {
    	lDeltaTim = lStartTim - lEndTim;
    }
    else
    {
        lDeltaTim = (1 << 24) - lEndTim + lStartTim;
    }

	if(lDeltaTim > g_max_adc_time)
	{
		g_max_adc_time = lDeltaTim;
	}

    g_last_out_time =  SysTick->VAL;
#endif
    }

#ifdef SYSVIEW_DEBUG
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}
