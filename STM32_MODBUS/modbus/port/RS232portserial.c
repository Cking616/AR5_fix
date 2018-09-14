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
*��������:void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
*��������:���ô���״̬
*��ڲ�����xRxEnable ����ʹ��, xTxEnable ����ʹ��
*���ڲ�������
*���ڣ�2013 12 30
*�汾��V1.0
*���ߣ�yrj
*��mbrtu.c���ں����е���
* RS485���շ�����λΪPA8/RS485_EN1(ԭ��ΪPD8)
****************************************************************************
*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    ENTER_CRITICAL_SECTION(  );
    if( xRxEnable )
    {
      
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
        //MAX485���� �͵�ƽΪ����ģʽ
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    }
    else
    {
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
        //MAX485���� �ߵ�ƽΪ����ģʽ
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
    }
    
    if( xTxEnable )
    {
      //ʹ�ܷ�������ж�
       USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    }
    else
    {
        //��ֹ��������ж�
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    }
    EXIT_CRITICAL_SECTION(  );
}


/*
****************************************************************************
*��������:xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
*��������:��ʼ������ͨѶ�˿�
*��ڲ������˴����޸Ĳ�����
*���ڲ�������
*���ڣ�2013 12 30
*�汾��V1.0
*���ߣ�yrj
*ucPORT ���ں� ulBaudRate  ������ ucDataBits ����λ eParity У��λ 
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
  //�������485���ͺͽ���ģʽ
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
*��������:xMBPortSerialPutByte( CHAR ucByte )
*��������:����ͨѶ�˿ڷ���һ���ֽ�����
*��ڲ�����Ҫ���͵��ֽ�
*���ڲ�������
*���ڣ�2013 12 30
*�汾��V1.0
*���ߣ�yrj
*����ʹ�õ����жϷ��ͣ�ֻ�轫���ݷŵ����ͼĴ������ɣ��޵ȴ���ѯ������ɱ�־
*��������ֵ���ΪTRUE
****************************************************************************
*/
BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
  USART_SendData(USART1, ucByte);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == 0);  //�ȴ��������
    return TRUE;
}



BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
  USART_ClearFlag(USART1, USART_IT_RXNE) ;         //�жϽ���
//  while(USART_GetFlagStatus(USART1, USART_IT_RXNE) == 0);  //�ȴ����ձ�־  
    *pucByte = (u8)USART_ReceiveData(USART1);  //��ȡ����BUFFER������

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
*��������:static void prvvUARTTxReadyISR( void )
*��������:�����жϺ�����
*��ڲ�����
*���ڲ�����
*���ڣ�2013 12 30
*�汾��V1.0
*���ߣ�yrj
*�����жϺ������˺��������޸ġ�ֻ�����û��Ľ����жϺ����е��ô˺������ɣ�
*ͬʱ���û�Ӧ�ڵ��ô˺�������������жϱ�־λ��
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
