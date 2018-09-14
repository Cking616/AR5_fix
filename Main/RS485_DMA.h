#ifndef __RS485_DMA_H
#define __RS485_DMA_H

/*
----------------------------------------------------------------------------
 * File					: RS485_DMA.h
 * Description			: TODO
 * Author				: zhenyonghit
 * Copyright			: Copyright (c) 2016
 * Company				: Asage-Robots(http://www.asage-robots.com/)
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

//---------------------------- Include files -----------------------------//
#include "stm32f4xx.h"
#include "HardwareConfig.h"

//-------------------------------- Define --------------------------------//
/*定义USART——DMA缓冲区大小，DMA流和通道*/
/*USART1_DMA信息*/
#if USART1_RS485_ENABLE == 1
	#define USART1_TX_BUF_SIZE			60						//发送缓冲区大小
	#define USART1_TX_DMA_STREAM		DMA2_Stream7			//发送端DMA流
	#define USART1_TX_DMA_CHANNEL		DMA_Channel_4			//发送端DMA流
	#define USART1_TX_DMA_CLK			RCC_AHB1Periph_DMA2		//时钟使能信息
	#define USART1_TX_DMA_FLAG_TCIF		DMA_FLAG_TCIF7			//发送中断标志位
	

	#define USART1_RX_BUF_SIZE			60						//接收缓冲区大小
	#define USART1_RX_DMA_STREAM		DMA2_Stream5			//接收端DMA流
	#define USART1_RX_DMA_CHANNEL		DMA_Channel_4			//接收端DMA流
	#define USART1_RX_DMA_CLK			RCC_AHB1Periph_DMA2		//时钟使能信息
	#define USART1_RX_DMA_FLAG_TCIF		DMA_FLAG_TCIF5			//接收中断标志位
#endif

/*USART2_DMA信息*/
#if USART2_RS485_ENABLE == 1
	#define USART2_TX_BUF_SIZE			60						//发送缓冲区大小
	#define USART2_TX_DMA_STREAM		DMA1_Stream4			//发送端DMA流
	#define USART2_TX_DMA_CHANNEL		DMA_Channel_4			//发送端DMA流
	#define USART2_TX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART2_TX_DMA_FLAG_TCIF		DMA_FLAG_TCIF7			//发送中断标志位

	#define USART2_RX_BUF_SIZE			60						//接收缓冲区大小
	#define USART2_RX_DMA_STREAM		DMA1_Stream2			//接收端DMA流
	#define USART2_RX_DMA_CHANNEL		DMA_Channel_4			//接收端DMA流
	#define USART2_RX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART2_RX_DMA_FLAG_TCIF		DMA_FLAG_TCIF5			//接收中断标志位
#endif

/*USART3_DMA信息*/
#if USART3_RS485_ENABLE == 1
	#define USART3_TX_BUF_SIZE			60						//发送缓冲区大小
	#define USART3_TX_DMA_STREAM		DMA1_Stream4			//发送端DMA流
	#define USART3_TX_DMA_CHANNEL		DMA_Channel_4			//发送端DMA流
	#define USART3_TX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART3_TX_DMA_FLAG_TCIF		DMA_FLAG_TCIF7			//发送中断标志位

	#define USART3_RX_BUF_SIZE			60						//接收缓冲区大小
	#define USART3_RX_DMA_STREAM		DMA1_Stream2			//接收端DMA流
	#define USART3_RX_DMA_CHANNEL		DMA_Channel_4			//接收端DMA流
	#define USART3_RX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART3_RX_DMA_FLAG_TCIF		DMA_FLAG_TCIF5			//接收中断标志位
#endif

/*USART4_DMA信息*/
#if USART4_RS485_ENABLE == 1
	#define USART4_TX_BUF_SIZE			60						//发送缓冲区大小
	#define USART4_TX_DMA_STREAM		DMA1_Stream4			//发送端DMA流
	#define USART4_TX_DMA_CHANNEL		DMA_Channel_4			//发送端DMA流
	#define USART4_TX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART4_TX_DMA_FLAG_TCIF		DMA_FLAG_TCIF4			//发送中断标志位

	#define USART4_RX_BUF_SIZE			60						//接收缓冲区大小
	#define USART4_RX_DMA_STREAM		DMA1_Stream2			//接收端DMA流
	#define USART4_RX_DMA_CHANNEL		DMA_Channel_4			//接收端DMA流
	#define USART4_RX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART4_RX_DMA_FLAG_TCIF		DMA_FLAG_TCIF2			//接收中断标志位
#endif

/*USART5_DMA信息*/
#if USART5_RS485_ENABLE == 1
	#define USART5_TX_BUF_SIZE			60						//发送缓冲区大小
	#define USART5_TX_DMA_STREAM		DMA1_Stream4			//发送端DMA流
	#define USART5_TX_DMA_CHANNEL		DMA_Channel_4			//发送端DMA流
	#define USART5_TX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART5_TX_DMA_FLAG_TCIF		DMA_FLAG_TCIF7			//发送中断标志位

	#define USART5_RX_BUF_SIZE			60						//接收缓冲区大小
	#define USART5_RX_DMA_STREAM		DMA1_Stream2			//接收端DMA流
	#define USART5_RX_DMA_CHANNEL		DMA_Channel_4			//接收端DMA流
	#define USART5_RX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART5_RX_DMA_FLAG_TCIF		DMA_FLAG_TCIF5			//接收中断标志位
#endif

/*USART6_DMA信息*/
#if USART6_RS485_ENABLE == 1
	#define USART6_TX_BUF_SIZE			60						//发送缓冲区大小
	#define USART6_TX_DMA_STREAM		DMA1_Stream4			//发送端DMA流
	#define USART6_TX_DMA_CHANNEL		DMA_Channel_4			//发送端DMA流
	#define USART6_TX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART6_TX_DMA_FLAG_TCIF		DMA_FLAG_TCIF7			//发送中断标志位

	#define USART6_RX_BUF_SIZE			60						//接收缓冲区大小
	#define USART6_RX_DMA_STREAM		DMA1_Stream2			//接收端DMA流
	#define USART6_RX_DMA_CHANNEL		DMA_Channel_4			//接收端DMA流
	#define USART6_RX_DMA_CLK			RCC_AHB1Periph_DMA1		//时钟使能信息
	#define USART6_RX_DMA_FLAG_TCIF		DMA_FLAG_TCIF5			//接收中断标志位
#endif

/*宏访问方式*/
#define USART_TX_BUF_SIZE(x)		USART##x##_TX_BUF_SIZE				//发送缓冲区大小
#define USART_TX_DMA_STREAM(x)		USART##x##_TX_DMA_STREAM			//发送端DMA流
#define USART_TX_DMA_CHANNEL(x)		USART##x##_TX_DMA_CHANNEL			//发送端DMA流
#define USART_TX_DMA_CLK(x)			USART##x##_TX_DMA_CLK				//时钟使能信息
#define USART_TX_DMA_FLAG_TCIF(x)	USART##x##_TX_DMA_FLAG_TCIF			//发送中断标志位

#define USART_RX_BUF_SIZE(x)		USART##x##_RX_BUF_SIZE				//接收缓冲区大小
#define USART_RX_DMA_STREAM(x)		USART##x##_RX_DMA_STREAM			//接收端DMA流
#define USART_RX_DMA_CHANNEL(x)		USART##x##_RX_DMA_CHANNEL			//接收端DMA流
#define USART_RX_DMA_CLK(x)			USART##x##_RX_DMA_CLK				//时钟使能信息
#define USART_RX_DMA_FLAG_TCIF(x)	USART##x##_RX_DMA_FLAG_TCIF			//接收中断标志位

//------------------------------- Typedef---------------------------------//
/*定义RS485_DMA_USART_t结构体*/
typedef struct
{
	u8 *TxBuf;								//发送数据缓冲区
	u8 *RxBuf;								//接收数据缓冲区
	void (*init_DMA_Tx)(void);				//初始化RS485的发送端DMA
	void (*init_DMA_Rx)(void);				//初始化RS485的接收端DMA
}RS485_DMA_USART_t;


//---------------------------- Global Variable ---------------------------//
/*定义RS485_DMA_USART全局变量*/
#if USART1_RS485_ENABLE == 1
	extern RS485_DMA_USART_t rs485_dma_usart1;
#endif

#if USART2_RS485_ENABLE == 1
	extern RS485_DMA_USART_t rs485_dma_usart2;
#endif

#if USART3_RS485_ENABLE == 1
	extern RS485_DMA_USART_t rs485_dma_usart3;
#endif

#if USART4_RS485_ENABLE == 1
	extern RS485_DMA_USART_t rs485_dma_usart4;
#endif

#if USART5_RS485_ENABLE == 1
	extern RS485_DMA_USART_t rs485_dma_usart5;
#endif

#if USART6_RS485_ENABLE == 1
	extern RS485_DMA_USART_t rs485_dma_usart6;
#endif


//---------------------------- Global Function ---------------------------//



//--------------------------------- Extern -------------------------------//



#endif

