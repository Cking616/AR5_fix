/*
----------------------------------------------------------------------------
 * File					: RS485_USART.c
 * Description			: 配置RS485串口USART
 * Author				: zhenyonghit
 * Copyright			: Copyright (c) 2016
 * Company				: Asage-Robots(http://www.asage-robots.com/)
 * 
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.12.06 16:51:21
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */

//---------------------------- Include files -----------------------------//
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "main.h"

#include "RS485_USART.h"
#include "RS232_USART.h"

#include "JC2JDCommunication.h"

#include "GlobalVariable.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "CRC_16.h"

#include "M1_statemachine.h"
//---------------------- Static Function Prototype ------------------------//
#if USART1_RS485_ENABLE == 1
static void init1(unsigned long int);
static void initTXE1(void);
#endif

#if USART2_RS485_ENABLE == 1
static void init2(unsigned long int);
static void initTXE2(void);
#endif

#if USART3_RS485_ENABLE == 1
static void init3(unsigned long int);
static void initTXE3(void);
#endif

#if USART4_RS485_ENABLE == 1
static void init4(unsigned long int);
static void initTXE4(void);
#endif

#if USART5_RS485_ENABLE == 1
static void init5(unsigned long int);
static void initTXE5(void);
#endif

#if USART6_RS485_ENABLE == 1
static void init6(unsigned long int);
static void initTXE6(void);
#endif

/*定义每个串口中断函数*/
#if USART1_RS485_ENABLE == 1
	void USART1_IRQHandler(void);
#endif

#if USART2_RS485_ENABLE == 1
	void USART2_IRQHandler(void);
#endif

#if USART3_RS485_ENABLE == 1
	void USART3_IRQHandler(void);
#endif

#if USART4_RS485_ENABLE == 1
	void UART4_IRQHandler(void);
#endif

#if USART5_RS485_ENABLE == 1
	void UART5_IRQHandler(void);
#endif

#if USART6_RS485_ENABLE == 1
	void USART6_IRQHandler(void);
#endif

//---------------------------- Static Variable ----------------------------//

//---------------------------- Global Function ----------------------------//


//---------------------------- Global Variable ----------------------------//
/*定义RS485_USART全局变量*/
#if USART1_RS485_ENABLE == 1
	RS485_USART_t rs485_usart1 = {
		.init = init1,
		.initTXE = initTXE1
	};
#endif

#if USART2_RS485_ENABLE == 1
	RS485_USART_t rs485_usart2 = {
		.init = init2,
		.initTXE = initTXE2
	};
#endif

#if USART3_RS485_ENABLE == 1
	RS485_USART_t rs485_usart3 = {
		.init = init3,
		.initTXE = initTXE3
	};
#endif

#if USART4_RS485_ENABLE == 1
	RS485_USART_t rs485_usart4 = {
		.init = init4,
		.initTXE = initTXE4
	};
#endif

#if USART5_RS485_ENABLE == 1
	RS485_USART_t rs485_usart5 = {
		.init = init5,
		.initTXE = initTXE5
	};
#endif

#if USART6_RS485_ENABLE == 1
	RS485_USART_t rs485_usart6 = {
		.init = init6,
		.initTXE = initTXE6
	};
#endif

//u8 rcv_Test[100];	

//------------------------------- Function --------------------------------//
/*
----------------------------------------------------------------------------
 * Name					: init
 * Description			: 用于初始化串口USART4
 * Author				: zhenyonghit
 * return				:
 * Para					:baudrate波特率，波特率太高会接受乱码，波特率高会产生第一个字符乱码。以后调试注意
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.11.29 11:46:25
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
#if USART1_RS485_ENABLE == 1
static void init1(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
#if EN_USART1_RXIT == 1
	NVIC_InitTypeDef NVIC_InitStruct;
#endif
	
	RCC_AHB1PeriphClockCmd(USART1_TX_CLK,ENABLE);						//GPIOA时钟使能
	RCC_AHB1PeriphClockCmd(USART1_RX_CLK,ENABLE);						//GPIOA时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART1_TX_PIN;							//PA9为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART1_RX_PIN;							//PA10为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);
		
	GPIO_PinAFConfig(USART1_TX_PORT, USART1_TX_SOURCE, GPIO_AF_USART1);	//PA9引脚复用为USART1
	GPIO_PinAFConfig(USART1_RX_PORT, USART1_RX_SOURCE, GPIO_AF_USART1);	//PA10引脚复用为USART1
	
	USART_DeInit(USART1);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);				//USART1时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;							//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;			//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;					//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;					//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	USART_Init(USART1, &USART_InitStruct);								//初始化USART1
	
#if EN_USART1_RXIT == 1
//	USART_ITConfig(USART1, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);							//打开USART1的中断
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);							//打开USART1的空闲总线中断
	USART_ITConfig(USART1, USART_IT_TC, ENABLE);							//打开USART1的空闲总线中断

#endif
	
	USART_Cmd(USART1, ENABLE);												//使能USART1                                             	 //读取寄存器操作
#if EN_USART1_RXIT == 1
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
#endif
}
#endif


#if USART2_RS485_ENABLE == 1
static void init2(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct2;
	
	RCC_AHB1PeriphClockCmd(USART2_TX_CLK,ENABLE);						// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART2_RX_CLK,ENABLE);						// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART2_TX_PIN;							//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART2_RX_PIN;							//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART2_TX_PORT, USART2_TX_SOURCE, GPIO_AF_USART2);	//PA0引脚复用为USART2
	GPIO_PinAFConfig(USART2_RX_PORT, USART2_RX_SOURCE, GPIO_AF_USART2);	//PA1引脚复用为USART2
	
	USART_DeInit(USART2);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);				//USART2时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;							//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;			//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;					//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;					//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	USART_Init(USART2, &USART_InitStruct);								//初始化USART2
	
//	USART_ITConfig(USART2, USART_IT_PE, ENABLE);						//使能奇偶检验错误中断
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);						//打开USART2的中断
	
	USART_Cmd(USART2, ENABLE);											//使能USART2

//	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif


#if USART3_RS485_ENABLE == 1
static void init3(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART3_TX_CLK,ENABLE);						// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART3_RX_CLK,ENABLE);						// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART3_TX_PIN;							//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART3_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART3_RX_PIN;							//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART3_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART3_TX_PORT, USART3_TX_SOURCE, GPIO_AF_USART3);	//PA0引脚复用为USART3
	GPIO_PinAFConfig(USART3_RX_PORT, USART3_RX_SOURCE, GPIO_AF_USART3);	//PA1引脚复用为USART3
	
	USART_DeInit(USART3);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);				//USART3时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;							//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;			//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;					//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;					//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	USART_Init(USART3, &USART_InitStruct);								//初始化USART3
	
//	USART_ITConfig(USART3, USART_IT_PE, ENABLE);						//使能奇偶检验错误中断
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);						//打开USART3的中断
	
	USART_Cmd(USART3, ENABLE);											//使能USART3

//	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif


#if USART4_RS485_ENABLE == 1
static void init4(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART4_TX_CLK,ENABLE);						// GPIOC时钟使能
	RCC_AHB1PeriphClockCmd(USART4_RX_CLK,ENABLE);						// GPIOC时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART4_TX_PIN;							//PC10为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART4_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART4_RX_PIN;							//PC11为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART4_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART4_TX_PORT, USART4_TX_SOURCE, GPIO_AF_UART4);	//PC10引脚复用为UART4
	GPIO_PinAFConfig(USART4_RX_PORT, USART4_RX_SOURCE, GPIO_AF_UART4);	//PC11引脚复用为UART4
	
	USART_DeInit(UART4);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);				//UART4时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;							//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;			//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;					//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;					//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	USART_Init(UART4, &USART_InitStruct);								//初始化UART4
	
//	USART_ITConfig(UART4, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);						//打开UART4的中断
	
	USART_Cmd(UART4, ENABLE);											//使能UART4

//	NVIC_InitStruct.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif


#if USART5_RS485_ENABLE == 1
static void init5(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART5_TX_CLK,ENABLE);						// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART5_RX_CLK,ENABLE);						// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART5_TX_PIN;							//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART5_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART5_RX_PIN;							//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART5_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART5_TX_PORT, USART5_TX_SOURCE, GPIO_AF_UART5);	//PA0引脚复用为UART5
	GPIO_PinAFConfig(USART5_RX_PORT, USART5_RX_SOURCE, GPIO_AF_UART5);	//PA1引脚复用为UART5
	
	USART_DeInit(UART5);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);				//UART5时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;							//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;			//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;					//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;					//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	USART_Init(UART5, &USART_InitStruct);								//初始化UART5
	
//	USART_ITConfig(UART5, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);						//打开UART5的中断
	
	USART_Cmd(UART5, ENABLE);											//使能UART5

//	NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif


#if USART6_RS485_ENABLE == 1
static void init6(unsigned long int baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(USART6_TX_CLK, ENABLE);						// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART6_RX_CLK, ENABLE);						// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART6_TX_PIN;							//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART6_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART6_RX_PIN;							//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART6_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART6_TX_PORT, USART6_TX_SOURCE, GPIO_AF_USART6);	//PA0引脚复用为USART6
	GPIO_PinAFConfig(USART6_RX_PORT, USART6_RX_SOURCE, GPIO_AF_USART6);	//PA1引脚复用为USART6
	
	USART_DeInit(USART6);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);				//USART6时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;							//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;			//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;					//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;					//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//收发模式
	USART_Init(USART6, &USART_InitStruct);								//初始化USART6
	
//	USART_ITConfig(USART6, USART_IT_PE, ENABLE);						//使能奇偶检验错误中断
//	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);						//打开USART6的中断
	
	USART_Cmd(USART6, ENABLE);											//使能USART6

//	NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
}
#endif


/*
----------------------------------------------------------------------------
 * Name					: RS485_InitTXE
 * Description			: 配置RS485发送使能口线TXE
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.12.06 19:59:23
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
#if USART1_RS485_ENABLE == 1
void initTXE1(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_USART1_TXEN, ENABLE);	// GPIOA时钟使能

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			/* 设为输出模式 */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			/* 设为推挽 */
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		/* 无上下拉电阻 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		/* IO口最大速度 */
	GPIO_InitStruct.GPIO_Pin = PIN_USART1_TXEN;
		
	GPIO_Init(PORT_USART1_TXEN, &GPIO_InitStruct);
	
	RS485_RX_EN(1);										/*使能RS485接收功能*/
}
#endif

#if USART2_RS485_ENABLE == 1
void initTXE2(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_USART2_TXEN, ENABLE);	// GPIOA时钟使能

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			/* 设为输出模式 */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			/* 设为推挽 */
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		/* 无上下拉电阻 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		/* IO口最大速度 */
	GPIO_InitStruct.GPIO_Pin = PIN_USART2_TXEN;
	
	GPIO_Init(PORT_USART2_TXEN, &GPIO_InitStruct);
	
	RS485_RX_EN(2);										/*使能RS485接收功能*/
}
#endif

#if USART3_RS485_ENABLE == 1
void initTXE3(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_USART3_TXEN, ENABLE);	// GPIOA时钟使能

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			/* 设为输出模式 */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			/* 设为推挽 */
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		/* 无上下拉电阻 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		/* IO口最大速度 */
	GPIO_InitStruct.GPIO_Pin = PIN_USART3_TXEN;
	
	GPIO_Init(PORT_USART3_TXEN, &GPIO_InitStruct);
	
	RS485_RX_EN(3);										/*使能RS485接收功能*/
}
#endif

#if USART4_RS485_ENABLE == 1
void initTXE4(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_USART4_TXEN, ENABLE);	// GPIOA时钟使能

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			/* 设为输出模式 */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			/* 设为推挽 */
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		/* 无上下拉电阻 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		/* IO口最大速度 */
	GPIO_InitStruct.GPIO_Pin = PIN_USART4_TXEN;
	
	GPIO_Init(PORT_USART4_TXEN, &GPIO_InitStruct);
	
	RS485_RX_EN(4);										/*使能RS485接收功能*/
}
#endif

#if USART5_RS485_ENABLE == 1
void initTXE5(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_USART5_TXEN, ENABLE);	// GPIOA时钟使能

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			/* 设为输出模式 */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			/* 设为推挽 */
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		/* 无上下拉电阻 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		/* IO口最大速度 */
	GPIO_InitStruct.GPIO_Pin = PIN_USART5_TXEN;
	
	GPIO_Init(PORT_USART5_TXEN, &GPIO_InitStruct);
	
	RS485_RX_EN(5);										/*使能RS485接收功能*/
}
#endif

#if USART6_RS485_ENABLE == 1
void initTXE6(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_USART6_TXEN, ENABLE);	// GPIOA时钟使能

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			/* 设为输出模式 */
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			/* 设为推挽 */
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		/* 无上下拉电阻 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		/* IO口最大速度 */
	GPIO_InitStruct.GPIO_Pin = PIN_USART6_TXEN;
	
	GPIO_Init(PORT_USART6_TXEN, &GPIO_InitStruct);
	
	RS485_RX_EN(6);										/*使能RS485接收功能*/
}
#endif


/*
----------------------------------------------------------------------------
 * Name					: USARTx_IRQHandler
 * Description			: USARTx_IRQHandler中断函数
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.12.06 16:59:23
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
#if EN_USART1_RXIT == 1
void USART1_IRQHandler(void)
{
	/**********************************************************************************/
	// After 2 Comm Try, Start Communication Judgement
	if(gsM1_Drive.sFaultThresholds.u8CommunicationStartFlag <= 2)
	{
		gsM1_Drive.sFaultThresholds.u8CommunicationStartFlag++;
	}
	else
	{
		gsM1_Drive.sFaultThresholds.u32CommunicationCnt++;
	}
	
	/**********************************************************************************/
	
	static int i = 0;
	
	if(SET == USART_GetITStatus(USART1, USART_IT_IDLE))							
	{
	/* 得到数据帧长度 */
		i = DMA_GetCurrDataCounter(USART_RX_DMA_STREAM(1));							//得到空闲数据量
		gui_RS485_RX_Num = RS485_RCV_SIZE-i;										//计算传输数据量		
		USART_ReceiveData(USART1);																		
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);							/*判断空闲中断*/
		
		if(J_ADDR[JOINT_No] == rs485_dma_usart1.RxBuf[0])									//判断地址位
		{
			if(gui_RS485_RX_Num<= JC2JD_CMDPNUM)								//复制至缓冲区
			{
				for(i=0; i<gui_RS485_RX_Num; ++i)
				{
					gt_RS485_RX_Buf.buf[i] = rs485_dma_usart1.RxBuf[i];
				}
				guc_RS485_Flag  = 1;											//建立通讯标志位
			}
			else
			{
				guc_RS485_Error_Flag = RX_NUM_ERROR;							//写入错误标志位
			}
		}
	
		/* USART1的接收DMA重新启动——准备接收数据 */
		DMA_Cmd(USART_RX_DMA_STREAM(1), DISABLE);									//关闭RX DMA传输
		DMA_ClearFlag(USART_RX_DMA_STREAM(1), USART_RX_DMA_FLAG_TCIF(1));			//请RX中断标志
		DMA_Cmd(USART_RX_DMA_STREAM(1), ENABLE);									//开启RX DMA传输
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);
	}
	
	if(SET == USART_GetITStatus(USART1, USART_IT_TC))		
	{
		DMA_Cmd(USART_TX_DMA_STREAM(1), DISABLE);								//关闭DMA传输
		USART_ClearITPendingBit(USART1, USART_IT_TC);							//
		USART_ClearFlag(USART1,USART_FLAG_TC);
		USART_ReceiveData(USART1);                                             	//读取寄存器操作
		RS485_RX_EN(1);															//使能RS485接收功能
	}
}
#endif







