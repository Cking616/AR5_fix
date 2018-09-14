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
*函数名称:xMBPortTimersInit( USHORT usTim1Timerout50us )
*函数功能:初始化超时定时器
*入口参数：
*出口参数：无
*日期：2013 12 30
*版本：V1.0
*作者：yrj
*用户应根据所使用的硬件初始化超时定时器，使之能产生中断时间
为usTim1Timerout50us*50us的中断。函数返回值务必为TRUE。
****************************************************************************
*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  //
  //使能定时器2时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 3*usTim1Timerout50us;      //下一个更新事件装入活动的自动重装载寄存器周期的值。它的取值必须在0x0000和0xFFFF之间。
  TIM_TimeBaseStructure.TIM_Prescaler = 83;              //预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    //时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  //预装载使能
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  //
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  //定时器2中断优先级
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //清除溢出中断标志位
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
  //定时器2溢出中断关闭
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  //定时器2禁能
  TIM_Cmd(TIM3,  DISABLE);
  return TRUE;
}

/*
****************************************************************************
*函数名称:vMBPortTimersEnable(  )
*函数功能:使能超时定时器
*入口参数：
*出口参数：无
*日期：2013 12 30
*版本：V1.0
*作者：yrj
*此函数的功能为使能超时定时器。用户需在此函数中清除中断标志位、
*清零定时器计数值，并重新使能定时器中断。
****************************************************************************
*/

void
vMBPortTimersEnable(  ) //负责配置一个时基
{
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  //设定定时器4的初始值
  TIM_SetCounter(TIM3,0x0000); 
  //定时器2启动
  TIM_Cmd(TIM3, ENABLE);
}

/*
****************************************************************************
*函数名称:void vMBPortTimersDisable(  )
*函数功能:使关闭超时定时器
*入口参数：
*出口参数：无
*日期：2013 12 30
*版本：V1.0
*作者：yrj
*此函数的功能为关闭超时定时器。用户需在此函数中清零定时器计数值，
并关闭定时器中断
****************************************************************************
*/

void
vMBPortTimersDisable(  )  //启动上述时基
{
   /* Disable any pending timers. */
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  TIM_SetCounter(TIM3,0x0000); 
  //关闭定时器2
  TIM_Cmd(TIM3, DISABLE);
}

/*
****************************************************************************
*函数名称:static void prvvTIMERExpiredISR( void )
*函数功能:定时器中断函数
*入口参数：
*出口参数：无
*日期：2013 12 30
*版本：V1.0
*作者：yrj
*定时器中断函数。此函数无需修改。只需在用户的定时器中断中调用此函数即可，
同时，用户应在调用此函数后清除中断标志位。
****************************************************************************
*/

static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired();
}

///**下一步整理好头文件 转移到  it.c文件中
//  * @brief  定时器4中断服务函数
//  * @param  None
//  * @retval None
//  */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    //清除定时器T2溢出中断标志位
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    prvvTIMERExpiredISR( );
  }
}

