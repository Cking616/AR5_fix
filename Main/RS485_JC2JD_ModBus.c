/*
----------------------------------------------------------------------------
 * File					: RS485_JC2JD_ModBus.c
 * Description			: 用RS485实现周期性ModBus协议的发送和接受数据，串口使用USART1
 * Author				: zhenyonghit
 * Copyright			: Copyright (c) 2016
 * Company				: Asage-Robots(http://www.asage-robots.com/)
 * 
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.11.28 18:57:27
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */

//---------------------------- Include files -----------------------------//
#include "RS485_JC2JD_ModBus.h"
#include "RS485_DMA.h"
#include "RS485_USART.h"
#include "RS232_USART.h"
//#include "Timer4.h"

#include "CRC_16.h"

#include "stm32f4xx.h"

#include "HardwareConfig.h"
#include "GlobalVariable.h"

//---------------------- Static Function Prototype ------------------------//
static void init(unsigned long int);
static void MSG_Tx(u8 *, int);

//---------------------------- Static Variable ----------------------------//
/*用于调试*/
//static uint32_t time_Start_ModBus = 0;		//用于记录程序运行起始时间

//---------------------------- Global Variable ----------------------------//
RS485_JC2JD_t rs485_JC2JD = 
{
	.Status_Flag = TX_ABLE,
	.Error_Flag = NO_ERROR,
	
#if USART1_RS485_ENABLE == 1
	.len_Tx = USART1_TX_BUF_SIZE,
	.len_Rx = USART1_RX_BUF_SIZE,
	
	.rs485_dma_usart = &rs485_dma_usart1,				/*RS485串口USART1*/
	.rs485_usart = &rs485_usart1,						/*RS485串口USART1*/
#endif
	
#if USART2_RS485_ENABLE == 1
	.len_Tx = USART2_TX_BUF_SIZE,
	.len_Rx = USART2_RX_BUF_SIZE,
	
	.rs485_dma_usart = &rs485_dma_usart2,				/*RS485串口USART2*/
	.rs485_usart = &rs485_usart2,						/*RS485串口USART2*/
#endif
	
#if USART3_RS485_ENABLE == 1
	.len_Tx = USART3_TX_BUF_SIZE,
	.len_Rx = USART3_RX_BUF_SIZE,
	
	.rs485_dma_usart = &rs485_dma_usart3,				/*RS485串口USART3*/
	.rs485_usart = &rs485_usart3,						/*RS485串口USART3*/
#endif

#if USART4_RS485_ENABLE == 1
	.len_Tx = USART4_TX_BUF_SIZE,
	.len_Rx = USART4_RX_BUF_SIZE,
	
	.rs485_dma_usart = &rs485_dma_usart4,				/*RS485串口USART4*/
	.rs485_usart = &rs485_usart4,						/*RS485串口USART4*/
#endif

#if USART5_RS485_ENABLE == 1
	.len_Tx = USART5_TX_BUF_SIZE,
	.len_Rx = USART5_RX_BUF_SIZE,
	
	.rs485_dma_usart = &rs485_dma_usart5,				/*RS485串口USART5*/
	.rs485_usart = &rs485_usart5,						/*RS485串口USART5*/
#endif

#if USART6_RS485_ENABLE == 1
	.len_Tx = USART6_TX_BUF_SIZE,
	.len_Rx = USART6_RX_BUF_SIZE,
	
	.rs485_dma_usart = &rs485_dma_usart6,				/*RS485串口USART6*/
	.rs485_usart = &rs485_usart6,						/*RS485串口USART6*/
#endif

	.init = init,
	.MSG_Tx = MSG_Tx
};

//------------------------------- Function --------------------------------//

/*
----------------------------------------------------------------------------
 * Name					: init_RS485
 * Description			: RS485硬件初始化
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: $NOW
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
static void init(unsigned long int baudrate)
{
	rs485_JC2JD.rs485_usart->initTXE();
	rs485_JC2JD.rs485_usart->init(baudrate);
	
	rs485_JC2JD.rs485_dma_usart->init_DMA_Tx();
	rs485_JC2JD.rs485_dma_usart->init_DMA_Rx();
	
//	rs485_JC2JD.Status_Flag = RX_ABLE;
//	rs485_JC2JD.Error_Flag = NO_ERROR;
}


/*
----------------------------------------------------------------------------
 * Name					: MSG_Tx
 * Description			: 用于发送、接收、校验数据
 * Author				: zhenyonghit
 * return				: NO_ERROR 0：正常；NO_RX_ERROR 1：无返回值；CRC_CHECK_ERROR 2：校验错误；OTHER_ERROR 3：其他错误。
 * Para					: TxBuf			发送缓冲区
 * Para					: Tx_Length		发送缓冲区大小，包含两个CRC校验位
* Para					: nus			等待间隔us，u16,最大为65535，在改时间内没有完成回复则认为没有应答
 * Para					: RxBuf			接收缓冲区
 * Para					: Rx_Length		接收缓冲区大小
 *
 * History
 * ----------------------
 * Rev					: V1.00
* Create Date			: 2016.12.07 14:07:56
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
static void MSG_Tx(u8 *TxBuf, int Tx_Length)
{
	/*发送数据*/
	int i;
	rs485_JC2JD.len_Tx = Tx_Length;
	for(i=0; i<Tx_Length; ++i)
	{
		rs485_JC2JD.rs485_dma_usart->TxBuf[i] = (TxBuf)[i];						//将数据写入TxBuf
	}
	getCRC(rs485_JC2JD.rs485_dma_usart->TxBuf, Tx_Length);
	DMA_Cmd(USART_TX_DMA_STREAM(1), DISABLE);									//关闭DMA传输
	DMA_ClearFlag(USART_TX_DMA_STREAM(1), USART_TX_DMA_FLAG_TCIF(1));			//请中断标志
	while (DMA_GetCmdStatus(USART_TX_DMA_STREAM(1)) != DISABLE)					//等待DMA1_Stream4可配置
	{	
	}		
	while(RESET==USART_GetFlagStatus(USART1,USART_FLAG_TXE));					//解决循环发送指令的CRC校验码丢失	
	RS485_TX_EN(1);																/*使能RS485发送功能*/	
	DMA_SetCurrDataCounter(USART_TX_DMA_STREAM(1), rs485_JC2JD.len_Tx);			//数据传输量
	DMA_Cmd(USART_TX_DMA_STREAM(1), ENABLE);									//开启TX DMA传输

}

