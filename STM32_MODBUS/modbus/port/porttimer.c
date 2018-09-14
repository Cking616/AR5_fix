/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"


/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );

/*
****************************************************************************
*��������:xMBPortTimersInit( USHORT usTim1Timerout50us )
*��������:��ʼ����ʱ��ʱ��
*��ڲ�����
*���ڲ�������
*���ڣ�2013 12 30
*�汾��V1.0
*���ߣ�yrj
*�û�Ӧ������ʹ�õ�Ӳ����ʼ����ʱ��ʱ����ʹ֮�ܲ����ж�ʱ��
ΪusTim1Timerout50us*50us���жϡ���������ֵ���ΪTRUE��
****************************************************************************
*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  //
  //ʹ�ܶ�ʱ��2ʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 3*usTim1Timerout50us;      //��һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ������ȡֵ������0x0000��0xFFFF֮�䡣
  TIM_TimeBaseStructure.TIM_Prescaler = 83;              //Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    //ʱ�ӷָ�
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  //Ԥװ��ʹ��
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  //
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //��ʱ��2�ж����ȼ�
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //�������жϱ�־λ
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
  //��ʱ��2����жϹر�
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  //��ʱ��2����
  TIM_Cmd(TIM3,  DISABLE);
  return TRUE;
}

/*
****************************************************************************
*��������:vMBPortTimersEnable(  )
*��������:ʹ�ܳ�ʱ��ʱ��
*��ڲ�����
*���ڲ�������
*���ڣ�2013 12 30
*�汾��V1.0
*���ߣ�yrj
*�˺����Ĺ���Ϊʹ�ܳ�ʱ��ʱ�����û����ڴ˺���������жϱ�־λ��
*���㶨ʱ������ֵ��������ʹ�ܶ�ʱ���жϡ�
****************************************************************************
*/

void
vMBPortTimersEnable(  ) //��������һ��ʱ��
{
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  //�趨��ʱ��4�ĳ�ʼֵ
  TIM_SetCounter(TIM3,0x0000); 
  //��ʱ��2����
  TIM_Cmd(TIM3, ENABLE);
}

/*
****************************************************************************
*��������:void vMBPortTimersDisable(  )
*��������:ʹ�رճ�ʱ��ʱ��
*��ڲ�����
*���ڲ�������
*���ڣ�2013 12 30
*�汾��V1.0
*���ߣ�yrj
*�˺����Ĺ���Ϊ�رճ�ʱ��ʱ�����û����ڴ˺��������㶨ʱ������ֵ��
���رն�ʱ���ж�
****************************************************************************
*/

void
vMBPortTimersDisable(  )  //��������ʱ��
{
   /* Disable any pending timers. */
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  TIM_SetCounter(TIM3,0x0000); 
  //�رն�ʱ��2
  TIM_Cmd(TIM3, DISABLE);
}

/*
****************************************************************************
*��������:static void prvvTIMERExpiredISR( void )
*��������:��ʱ���жϺ���
*��ڲ�����
*���ڲ�������
*���ڣ�2013 12 30
*�汾��V1.0
*���ߣ�yrj
*��ʱ���жϺ������˺��������޸ġ�ֻ�����û��Ķ�ʱ���ж��е��ô˺������ɣ�
ͬʱ���û�Ӧ�ڵ��ô˺���������жϱ�־λ��
****************************************************************************
*/

static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired();
}

///**��һ�������ͷ�ļ� ת�Ƶ�  it.c�ļ���
//  * @brief  ��ʱ��4�жϷ�����
//  * @param  None
//  * @retval None
//  */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    //�����ʱ��T2����жϱ�־λ
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    prvvTIMERExpiredISR( );
  }
}

