/*
 * FreeModbus Libary: MSP430 Port
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
 * File: $Id: portserial.c,v 1.3 2006/11/19 03:57:49 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Static variables ---------------------------------*/
UCHAR           ucGIEWasEnabled = FALSE;
UCHAR           ucCriticalNesting = 0x00;



/*
****************************************************************************
*函数名称:void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
*函数功能:设置串口状态
*入口参数：xRxEnable 接收使能, xTxEnable 发送使能
*出口参数：无
*日期：2013 12 30
*版本：V1.0
*作者：yrj
*在mbrtu.c串口函数中调用
* RS485的收发控制位为PA8/RS485_EN1(原来为PD8)
****************************************************************************
*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    ENTER_CRITICAL_SECTION(  );
    if( xRxEnable )
    {
      
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
        //MAX485操作 低电平为接收模式
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    }
    else
    {
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
        //MAX485操作 高电平为发送模式
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
    }
    
    if( xTxEnable )
    {
      //使能发送完成中断
       USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    }
    else
    {
        //禁止发送完成中断
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    }
    EXIT_CRITICAL_SECTION(  );
}


/*
****************************************************************************
*函数名称:xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
*函数功能:初始化串行通讯端口
*入口参数：此处仅修改波特率
*出口参数：无
*日期：2013 12 30
*版本：V1.0
*作者：yrj
*ucPORT 串口号 ulBaudRate  波特率 ucDataBits 数据位 eParity 校验位 
****************************************************************************
*/
BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  BOOL    bInitialized = TRUE;
  GPIO_InitTypeDef GPIO_InitStructure; 
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
  
   /*USART clock source enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
   /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    

  USART_InitStructure.USART_BaudRate = ulBaudRate;
  switch ( eParity )
  {
  case MB_PAR_NONE:
    USART_InitStructure.USART_Parity = USART_Parity_No;
    break;
  case MB_PAR_ODD:
    USART_InitStructure.USART_Parity = USART_Parity_Odd;
    break;
  case MB_PAR_EVEN:
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    break;
  }
  switch ( ucDataBits )
  {
  case 8:
    if(eParity==MB_PAR_NONE)
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    else
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    break;
  case 7:
    break;
  default:
    bInitialized = FALSE;
  }
  if( bInitialized )
  {
    ENTER_CRITICAL_SECTION(  );
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
    EXIT_CRITICAL_SECTION(  );
  }
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  //最后配置485发送和接收模式
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  return bInitialized;

}


/*
****************************************************************************
*函数名称:xMBPortSerialPutByte( CHAR ucByte )
*函数功能:串行通讯端口发送一个字节数据
*入口参数：要发送的字节
*出口参数：无
*日期：2013 12 30
*版本：V1.0
*作者：yrj
*由于使用的是中断发送，只需将数据放到发送寄存器即可，无等待查询发送完成标志
*函数返回值务必为TRUE
****************************************************************************
*/
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  USART_SendData(USART1, ucByte);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);  //等待发送完成
    return TRUE;
}



BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
  USART_ClearFlag(USART1, USART_IT_RXNE) ;         //中断接收
//  while(USART_GetFlagStatus(USART1, USART_IT_RXNE) == 0);  //等待接收标志  
    *pucByte = (u8)USART_ReceiveData(USART1);  //获取接收BUFFER的数据

    return TRUE;
}


void
EnterCriticalSection( void )
{
  __disable_irq();
}

void
ExitCriticalSection( void )
{
  __enable_irq();
}


/*
****************************************************************************
*函数名称:static void prvvUARTTxReadyISR( void )
*函数功能:接收中断函数。
*入口参数：
*出口参数：
*日期：2013 12 30
*版本：V1.0
*作者：yrj
*接收中断函数。此函数无需修改。只需在用户的接收中断函数中调用此函数即可，
*同时，用户应在调用此函数后，清除接收中断标志位。
****************************************************************************
*/
void USART1_IRQHandler(void)
{
   if(USART_GetITStatus(USART1,USART_IT_TC))
  {
      pxMBFrameCBTransmitterEmpty(  );

  }
  else if(USART_GetITStatus(USART1,USART_IT_RXNE))
  {
      pxMBFrameCBByteReceived(  );
  }
}
