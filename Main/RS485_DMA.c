/*
----------------------------------------------------------------------------
 * File					: RS485_DMA.c
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
#include "RS485_DMA.h"

#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "RS485_USART.h"
//---------------------- Static Function Prototype ------------------------//
/*定义初始化函数*/
#if USART1_RS485_ENABLE == 1
	static void init_DMA_Tx1(void);
	static void init_DMA_Rx1(void);
#endif

#if USART2_RS485_ENABLE == 1
	static void init_DMA_Tx2(void);
	static void init_DMA_Rx2(void);
#endif

#if USART3_RS485_ENABLE == 1
	static void init_DMA_Tx3(void);
	static void init_DMA_Rx3(void);
#endif

#if USART4_RS485_ENABLE == 1
	static void init_DMA_Tx4(void);
	static void init_DMA_Rx4(void);
#endif

#if USART5_RS485_ENABLE == 1
	static void init_DMA_Tx5(void);
	static void init_DMA_Rx5(void);
#endif

#if USART6_RS485_ENABLE == 1
	static void init_DMA_Tx6(void);
	static void init_DMA_Rx6(void);
#endif

//---------------------------- Static Variable ----------------------------//
/* 定义缓冲区 */
#if USART1_RS485_ENABLE == 1
	static u8 TxBuf1[USART1_TX_BUF_SIZE];			/*发送缓冲区*/
	static u8 RxBuf1[USART1_RX_BUF_SIZE];			/*接受缓冲区*/
#endif

#if USART2_RS485_ENABLE == 1
	static u8 TxBuf2[USART2_TX_BUF_SIZE];			/*发送缓冲区*/
	static u8 RxBuf2[USART2_RX_BUF_SIZE];			/*接受缓冲区*/
#endif

#if USART3_RS485_ENABLE == 1
	static u8 TxBuf3[USART3_TX_BUF_SIZE];			/*发送缓冲区*/
	static u8 RxBuf3[USART3_RX_BUF_SIZE];			/*接受缓冲区*/
#endif

#if USART4_RS485_ENABLE == 1
	static u8 TxBuf4[USART4_TX_BUF_SIZE];			/*发送缓冲区*/
	static u8 RxBuf4[USART4_RX_BUF_SIZE];			/*接受缓冲区*/
#endif

#if USART5_RS485_ENABLE == 1
	static u8 TxBuf5[USART5_TX_BUF_SIZE];			/*发送缓冲区*/
	static u8 RxBuf5[USART5_RX_BUF_SIZE];			/*接受缓冲区*/
#endif

#if USART6_RS485_ENABLE == 1
	static u8 TxBuf6[USART6_TX_BUF_SIZE];			/*发送缓冲区*/
	static u8 RxBuf6[USART6_RX_BUF_SIZE];			/*接受缓冲区*/
#endif

//---------------------------- Global Variable ----------------------------//
/*定义RS485_DMA_USART全局变量*/
#if USART1_RS485_ENABLE == 1
	RS485_DMA_USART_t rs485_dma_usart1 = {
		.TxBuf = TxBuf1,
		.RxBuf = RxBuf1,
		.init_DMA_Tx = init_DMA_Tx1,
		.init_DMA_Rx = init_DMA_Rx1
	};
#endif

#if USART2_RS485_ENABLE == 1
	RS485_DMA_USART_t rs485_dma_usart2 = {
		.TxBuf = TxBuf2,
		.RxBuf = RxBuf2,
		.init_DMA_Tx = init_DMA_Tx2,
		.init_DMA_Rx = init_DMA_Rx2
	};
#endif

#if USART3_RS485_ENABLE == 1
	RS485_DMA_USART_t rs485_dma_usart3 = {
		.TxBuf = TxBuf3,
		.RxBuf = RxBuf3,
		.init_DMA_Tx = init_DMA_Tx3,
		.init_DMA_Rx = init_DMA_Rx3
	};
#endif

#if USART4_RS485_ENABLE == 1
	RS485_DMA_USART_t rs485_dma_usart4 = {
		.TxBuf = TxBuf4,
		.RxBuf = RxBuf4,
		.init_DMA_Tx = init_DMA_Tx4,
		.init_DMA_Rx = init_DMA_Rx4
	};
#endif

#if USART5_RS485_ENABLE == 1
	RS485_DMA_USART_t rs485_dma_usart5 = {
		.TxBuf = TxBuf5,
		.RxBuf = RxBuf5,
		.init_DMA_Tx = init_DMA_Tx5,
		.init_DMA_Rx = init_DMA_Rx5
	};
#endif

#if USART6_RS485_ENABLE == 1
	RS485_DMA_USART_t rs485_dma_usart6 = {
		.TxBuf = TxBuf6,
		.RxBuf = RxBuf6,
		.init_DMA_Tx = init_DMA_Tx6,
		.init_DMA_Rx = init_DMA_Rx6
	};
#endif
	
	
//------------------------------- Function --------------------------------//
/*
----------------------------------------------------------------------------
 * Name					: init_DMA_Tx
 * Description			: 初始化发送端口DMA
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.12.07 11:15:35
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
/*初始化USART1发送端口DMA*/
#if USART1_RS485_ENABLE == 1
static void init_DMA_Tx1(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_TX_DMA_CLK(1), ENABLE);					//DMA时钟使能
	
	DMA_DeInit(USART_TX_DMA_STREAM(1));
	
	while (DMA_GetCmdStatus(USART_TX_DMA_STREAM(1)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_TX_DMA_CHANNEL(1);					//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART1->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)TxBuf1;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//存储器到外设模式
	DMA_InitStruct.DMA_BufferSize = USART_TX_BUF_SIZE(1);					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_TX_DMA_STREAM(1), &DMA_InitStruct);//初始化DMA Stream
	
//	DMA_ITConfig(USART1_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
	
//	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);							//使能串口的DMA发送
		
//	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif

/*初始化USART2发送端口DMA*/
#if USART2_RS485_ENABLE == 1
static void init_DMA_Tx2(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_TX_DMA_CLK(2), ENABLE);						//DMA时钟使能
	
	DMA_DeInit(USART_TX_DMA_STREAM(2));
	
	while (DMA_GetCmdStatus(USART_TX_DMA_STREAM(2)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_TX_DMA_CHANNEL(2);						//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART2->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)TxBuf2;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//存储器到外设模式
	DMA_InitStruct.DMA_BufferSize = USART_TX_BUF_SIZE(2);						//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_TX_DMA_STREAM(2), &DMA_InitStruct);//初始化DMA Stream
	
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);							//使能串口的DMA发送
	
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x10;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);	
}
#endif

/*初始化USART3发送端口DMA*/
#if USART3_RS485_ENABLE == 1
static void init_DMA_Tx3(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART3_TX_DMA_CLK(3), ENABLE);						//DMA时钟使能
	
	DMA_DeInit(USART_TX_DMA_STREAM(3));
	
	while (DMA_GetCmdStatus(USART_TX_DMA_STREAM(3)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_TX_DMA_CHANNEL(3);						//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART3->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)TxBuf3;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//存储器到外设模式
	DMA_InitStruct.DMA_BufferSize = USART_TX_BUF_SIZE(3);						//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_TX_DMA_STREAM(3), &DMA_InitStruct);//初始化DMA Stream
	
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);							//使能串口的DMA发送
	
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x10;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);	
}
#endif

/*初始化USART4发送端口DMA*/
#if USART4_RS485_ENABLE == 1
static void init_DMA_Tx4(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_TX_DMA_CLK(4), ENABLE);						//DMA时钟使能
	
	DMA_DeInit(USART_TX_DMA_STREAM(4));
	
	while (DMA_GetCmdStatus(USART_TX_DMA_STREAM(4)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_TX_DMA_CHANNEL(4);						//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&UART4->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)TxBuf4;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//存储器到外设模式
	DMA_InitStruct.DMA_BufferSize = USART_TX_BUF_SIZE(4);						//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_TX_DMA_STREAM(4), &DMA_InitStruct);//初始化DMA Stream
	
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);							//使能串口的DMA发送
	
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x10;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);	
}
#endif

/*初始化USART5发送端口DMA*/
#if USART5_RS485_ENABLE == 1
static void init_DMA_Tx5(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_TX_DMA_CLK(5), ENABLE);						//DMA时钟使能
	
	DMA_DeInit(USART_TX_DMA_STREAM(5));
	
	while (DMA_GetCmdStatus(USART_TX_DMA_STREAM(5)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_TX_DMA_CHANNEL(5);						//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&UART5->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)TxBuf5;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//存储器到外设模式
	DMA_InitStruct.DMA_BufferSize = USART_TX_BUF_SIZE(5);						//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_TX_DMA_STREAM(5), &DMA_InitStruct);//初始化DMA Stream
	
	USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);							//使能串口的DMA发送
	
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x10;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);	
}
#endif

/*初始化USART6发送端口DMA*/
#if USART6_RS485_ENABLE == 1
static void init_DMA_Tx6(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_TX_DMA_CLK(6), ENABLE);						//DMA时钟使能
	
	DMA_DeInit(USART_TX_DMA_STREAM(6));
	
	while (DMA_GetCmdStatus(USART_TX_DMA_STREAM(6)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_TX_DMA_CHANNEL(6);						//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART6->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)TxBuf6;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//存储器到外设模式
	DMA_InitStruct.DMA_BufferSize = USART_TX_BUF_SIZE(6);						//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_TX_DMA_STREAM(6), &DMA_InitStruct);//初始化DMA Stream
	
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);							//使能串口的DMA发送
	
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x10;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);	
}
#endif



/*
----------------------------------------------------------------------------
 * Name					: init_DMA_Rx
 * Description			: 初始化接收端口DMA
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
* Create Date			: 2016.12.07 11:28:26
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
/*初始化USART1接收端口DMA*/
#if USART1_RS485_ENABLE == 1
static void init_DMA_Rx1(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_RX_DMA_CLK(1), ENABLE);					//DMA时钟使能
	
	DMA_DeInit(USART_RX_DMA_STREAM(1));
	
	while (DMA_GetCmdStatus(USART_RX_DMA_STREAM(1)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_RX_DMA_CHANNEL(1);					//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART1->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)RxBuf1;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;					//外设到存储器模式
	DMA_InitStruct.DMA_BufferSize = USART_RX_BUF_SIZE(1);					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_RX_DMA_STREAM(1), &DMA_InitStruct);						//初始化DMA Stream
	
//	DMA_ITConfig(USART_RX_DMA_STREAM(1), DMA_IT_HT, ENABLE);				//打开半传输中标志
	
//	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);							//使能串口的DMA接收
	
	DMA_Cmd(USART_RX_DMA_STREAM(1), ENABLE);								//开启DMA传输
			
//	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream5_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif

/*初始化USART2接收端口DMA*/
#if USART2_RS485_ENABLE == 1
static void init_DMA_Rx2(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_RX_DMA_CLK(2), ENABLE);					//DMA时钟使能
	
	DMA_DeInit(USART_RX_DMA_STREAM(2));
	
	while (DMA_GetCmdStatus(USART_RX_DMA_STREAM(2)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_RX_DMA_CHANNEL(2);					//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART2->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)RxBuf2;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;					//外设到存储器模式
	DMA_InitStruct.DMA_BufferSize = USART_RX_BUF_SIZE(2);					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_RX_DMA_STREAM(2), &DMA_InitStruct);						//初始化DMA Stream
	
//	DMA_ITConfig(USART2_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
	
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);							//使能串口的DMA接收
	
//	DMA_Cmd(USART2_RX_DMA_STREAM, ENABLE);									//开启DMA传输
			
//	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif

/*初始化USART3接收端口DMA*/
#if USART3_RS485_ENABLE == 1
static void init_DMA_Rx3(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_RX_DMA_CLK(3), ENABLE);					//DMA时钟使能
	
	DMA_DeInit(USART_RX_DMA_STREAM(3));
	
	while (DMA_GetCmdStatus(USART_RX_DMA_STREAM(3)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_RX_DMA_CHANNEL(3);					//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART3->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)RxBuf3;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;					//外设到存储器模式
	DMA_InitStruct.DMA_BufferSize = USART_RX_BUF_SIZE(3);					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_RX_DMA_STREAM(3), &DMA_InitStruct);						//初始化DMA Stream
	
//	DMA_ITConfig(USART3_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
	
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);							//使能串口的DMA接收
	
//	DMA_Cmd(USART3_RX_DMA_STREAM, ENABLE);									//开启DMA传输
			
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif

/*初始化UART4接收端口DMA*/
#if USART4_RS485_ENABLE == 1
static void init_DMA_Rx4(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_RX_DMA_CLK(4), ENABLE);					//DMA时钟使能
	
	DMA_DeInit(USART_RX_DMA_STREAM(4));
	
	while (DMA_GetCmdStatus(USART_RX_DMA_STREAM(4)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_RX_DMA_CHANNEL(4);					//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&UART4->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)RxBuf4;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;					//外设到存储器模式
	DMA_InitStruct.DMA_BufferSize = USART_RX_BUF_SIZE(4);					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_RX_DMA_STREAM(4), &DMA_InitStruct);//初始化DMA Stream
	
//	DMA_ITConfig(USART4_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
	
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);							//使能串口的DMA接收
	
//	DMA_Cmd(USART4_RX_DMA_STREAM, ENABLE);									//开启DMA传输
			
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif

/*初始化UART5接收端口DMA*/
#if USART5_RS485_ENABLE == 1
static void init_DMA_Rx5(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_RX_DMA_CLK(5), ENABLE);					//DMA时钟使能
	
	DMA_DeInit(USART_RX_DMA_STREAM(5));
	
	while (DMA_GetCmdStatus(USART_RX_DMA_STREAM(5)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_RX_DMA_CHANNEL(5);					//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&UART5->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)RxBuf5;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;					//外设到存储器模式
	DMA_InitStruct.DMA_BufferSize = USART_RX_BUF_SIZE(5);					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_RX_DMA_STREAM(5), &DMA_InitStruct);//初始化DMA Stream
	
//	DMA_ITConfig(USART5_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
	
	USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);							//使能串口的DMA接收
	
//	DMA_Cmd(USART5_RX_DMA_STREAM, ENABLE);									//开启DMA传输
			
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif

/*初始化USART6接收端口DMA*/
#if USART6_RS485_ENABLE == 1
static void init_DMA_Rx6(void)
{
	DMA_InitTypeDef  DMA_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART_RX_DMA_CLK(6), ENABLE);					//DMA时钟使能
	
	DMA_DeInit(USART_RX_DMA_STREAM(6));
	
	while (DMA_GetCmdStatus(USART_RX_DMA_STREAM(6)) != DISABLE)
	{
	}																		//等待DMA_Stream可配置
	
	/* 配置 DMA Stream */
	DMA_InitStruct.DMA_Channel = USART_RX_DMA_CHANNEL(6);					//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART6->DR;				//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)RxBuf6;						//DMA存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;					//外设到存储器模式
	DMA_InitStruct.DMA_BufferSize = USART_RX_BUF_SIZE(6);					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;					//存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;								//使用普通模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;					//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(USART_RX_DMA_STREAM(6), &DMA_InitStruct);//初始化DMA Stream
	
//	DMA_ITConfig(USART6_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
	
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);							//使能串口的DMA接收
	
//	DMA_Cmd(USART6_RX_DMA_STREAM, ENABLE);									//开启DMA传输
			
//	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif



/*
----------------------------------------------------------------------------
 * Name					: init_DMA_Rx
 * Description			: 初始化接收端口DMA
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
* Create Date			: 2016.12.07 11:28:26
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
//void DMA2_Stream7_IRQHandler(void)
//{
//	DMA_ClearITPendingBit(USART_TX_DMA_STREAM(1), DMA_IT_TCIF7);                //清DMA传输完成中断
//	DMA_Cmd(USART_TX_DMA_STREAM(1), DISABLE);									//关闭DMA传输
//	DMA_ClearFlag(USART_TX_DMA_STREAM(1), USART_TX_DMA_FLAG_TCIF(1));			//请中断标志
//    while((USART_GetFlagStatus(USART1,USART_FLAG_TC) != SET));                  //检测指令发送完成
//    RS485_RX_EN(1);																//使能RS485接收功能
//	USART_Cmd(USART1, DISABLE);                                                 //关闭串口
//	USART_ReceiveData(USART1);                                                  //读取寄存器操作
//	DMA_Cmd(USART_RX_DMA_STREAM(1), DISABLE);									//关闭DMA传输
//	DMA_ClearFlag(USART_RX_DMA_STREAM(1), USART_RX_DMA_FLAG_TCIF(1));			//请中断标志
//	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);                              //开启串口空闲中断
//	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);                              //开启DMA接收
//	DMA_Cmd(USART_RX_DMA_STREAM(1), ENABLE);									//关闭DMA传输
//	USART_Cmd(USART1, ENABLE);                                                  //打开串口
//}
