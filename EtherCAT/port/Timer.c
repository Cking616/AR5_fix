/****************************************Copyright (c)****************************************************
**
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               .c
** Descriptions:          timer2  hardware driver
**
**--------------------------------------------------------------------------------------------------------
** Created by:               lyd
** Created date:            2015-05-01
** Version:                 	v1.0
** Descriptions:           The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Descriptions:
**
*********************************************************************************************************/

#include "stm32f4xx.h"
#include "ecat_def.h"

/*******************************************************************************
* Function Name  : TIM_Configuration
* Description    : TIM_Configuration program.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TIM_Configuration(uint8_t period)		//100us
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);

    TIM_DeInit(TIM5);
    TIM_TimeBaseStructure.TIM_Period = period * 200 ; //200;		 					/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
      																    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    TIM_TimeBaseStructure.TIM_Prescaler = (42 - 1);				        /* ʱ��Ԥ��Ƶ��   ���磺ʱ��Ƶ��=72MHZ/(ʱ��Ԥ��Ƶ+1) */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			/* ������Ƶ */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		/* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);							    /* �������жϱ�־ */
    #if ECAT_TIMER_INT
        TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
    #endif
    TIM_Cmd(TIM5, ENABLE);											/* ����ʱ�� */

	/*#if ECAT_TIMER_INT
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	#endif*/

}
