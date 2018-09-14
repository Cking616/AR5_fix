/*
----------------------------------------------------------------------------
 * File					: RS232_USART.c
 * Description			: 调试串口USART的配置，用于打印调试信息
 * Author				: zhenyonghit
 * Copyright			: Copyright (c) 2016
 * Company				: Asage-Robots(http://www.asage-robots.com/)
 * 
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.12.06 17:52:23
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

#include "RS232_USART.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

//---------------------- Static Function Prototype ------------------------//
int my_printf(const char * fmt,...);

//---------------------------- Static Variable ----------------------------//
#if EN_USART4_RXIT == 1
u32 reg_Addr = 0x00000000;			/*用于调试，寄存器的地址*/
u32 reg_Data = 0x00000000;			/*用于调试，寄存器的值*/
int flag_RW	 = 0;					/*用于调试，0——读寄存器，1——写寄存器*/
#endif

//---------------------------- Global Variable ----------------------------//
//#ifdef DEBUG_PRINT
/*定义每个串口结构体变量*/
#if USART1_RS232_ENABLE == 1
	static void init1(unsigned long int);
	//static void IT_TC_ENABLE1(void);
	//static void IT_TC_DISABLE1(void);
	//static void IT_RXNE_ENABLE1(void);
	//static void IT_RXNE_DISABLE1(void);
	static int SendBuffer1(void *, int);
	static int SendByte1(char);
	static int SendString1(char const*);

	RS232_USART_t rs232_usart1 = {
		.init = init1,
//		.IT_TC_ENABLE = IT_TC_ENABLE1,
//		.IT_TC_DISABLE = IT_TC_DISABLE1,
//		.IT_RXNE_ENABLE = IT_RXNE_ENABLE1,
//		.IT_RXNE_DISABLE = IT_RXNE_DISABLE1,
		.SendBuffer = SendBuffer1,
		.SendByte = SendByte1,
		.SendString = SendString1
	};
#endif

#if USART2_RS232_ENABLE == 1
	static void init2(unsigned long int);
	//static void IT_TC_ENABLE2(void);
	//static void IT_TC_DISABLE2(void);
	//static void IT_RXNE_ENABLE2(void);
	//static void IT_RXNE_DISABLE2(void);
	static int SendBuffer2(void *, int);
	static int SendByte2(char);
	static int SendString2(char const*);
	
	RS232_USART_t rs232_usart2 = {
		.init = init2,
//		.IT_TC_ENABLE = IT_TC_ENABLE2,
//		.IT_TC_DISABLE = IT_TC_DISABLE2,
//		.IT_RXNE_ENABLE = IT_RXNE_ENABLE2,
//		.IT_RXNE_DISABLE = IT_RXNE_DISABLE2,
		.SendBuffer = SendBuffer2,
		.SendByte = SendByte2,
		.SendString = SendString2
	};
#endif

#if USART3_RS232_ENABLE == 1
	static void init3(unsigned long int);
	//static void IT_TC_ENABLE3(void);
	//static void IT_TC_DISABLE3(void);
	//static void IT_RXNE_ENABLE3(void);
	//static void IT_RXNE_DISABLE3(void);
	static int SendBuffer3(void *, int);
	static int SendByte3(char);
	static int SendString3(char const*);
	
	RS232_USART_t rs232_usart3 = {
		.init = init3,
//		.IT_TC_ENABLE = IT_TC_ENABLE3,
//		.IT_TC_DISABLE = IT_TC_DISABLE3,
//		.IT_RXNE_ENABLE = IT_RXNE_ENABLE3,
//		.IT_RXNE_DISABLE = IT_RXNE_DISABLE3,
		.SendBuffer = SendBuffer3,
		.SendByte = SendByte3,
		.SendString = SendString3
	};
#endif

#if USART4_RS232_ENABLE == 1
	static void init4(unsigned long int);
	//static void IT_TC_ENABLE4(void);
	//static void IT_TC_DISABLE4(void);
	//static void IT_RXNE_ENABLE4(void);
	//static void IT_RXNE_DISABLE4(void);
	static int SendBuffer4(void *, int);
	static int SendByte4(char);
	static int SendString4(char const*);
	
	RS232_USART_t rs232_usart4 = {
		.init = init4,
//		.IT_TC_ENABLE = IT_TC_ENABLE4,
//		.IT_TC_DISABLE = IT_TC_DISABLE4,
//		.IT_RXNE_ENABLE = IT_RXNE_ENABLE4,
//		.IT_RXNE_DISABLE = IT_RXNE_DISABLE4,
		.SendBuffer = SendBuffer4,
		.SendByte = SendByte4,
		.SendString = SendString4
	};
#endif

#if USART5_RS232_ENABLE == 1
	static void init5(unsigned long int);
	//static void IT_TC_ENABLE5(void);
	//static void IT_TC_DISABLE5(void);
	//static void IT_RXNE_ENABLE5(void);
	//static void IT_RXNE_DISABLE5(void);
	static int SendBuffer5(void *, int);
	static int SendByte5(char);
	static int SendString5(char const*);
	
	RS232_USART_t rs232_usart5 = {
		.init = init5,
//		.IT_TC_ENABLE = IT_TC_ENABLE5,
//		.IT_TC_DISABLE = IT_TC_DISABLE5,
//		.IT_RXNE_ENABLE = IT_RXNE_ENABLE5,
//		.IT_RXNE_DISABLE = IT_RXNE_DISABLE5,
		.SendBuffer = SendBuffer5,
		.SendByte = SendByte5,
		.SendString = SendString5
	};
#endif

#if USART6_RS232_ENABLE == 1
	static void init6(unsigned long int);
	//static void IT_TC_ENABLE6(void);
	//static void IT_TC_DISABLE6(void);
	//static void IT_RXNE_ENABLE6(void);
	//static void IT_RXNE_DISABLE6(void);
	static int SendBuffer6(void *, int);
	static int SendByte6(char);
	static int SendString6(char const*);
	
	RS232_USART_t rs232_usart6 = {
		.init = init6,
//		.IT_TC_ENABLE = IT_TC_ENABLE6,
//		.IT_TC_DISABLE = IT_TC_DISABLE6,
//		.IT_RXNE_ENABLE = IT_RXNE_ENABLE6,
//		.IT_RXNE_DISABLE = IT_RXNE_DISABLE6,
		.SendBuffer = SendBuffer6,
		.SendByte = SendByte6,
		.SendString = SendString6
	};
#endif
//#endif

		
#if EN_USART4_RXIT == 1
	//串口1中断服务程序
	//注意,读取USARTx->SR能避免莫名其妙的错误
	u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
	//接收状态
	//bit15，	接收完成标志
	//bit14，	接收到0x0D
	//bit13~0，	接收到的有效字节数目
	u16 USART_RX_STA=0;       //接收状态标记
#endif

//------------------------------- Function --------------------------------//
/*
----------------------------------------------------------------------------
 * Name					: init
 * Description			: 用于初始化串口USART
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
/*USART1初始化*/
#if USART1_RS232_ENABLE == 1
static void init1(unsigned long int baudrate)
{
#ifdef DEBUG_PRINT
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;

	#if EN_USART1_RXIT == 1
	NVIC_InitTypeDef NVIC_InitStruct;
	#endif
	
	RCC_AHB1PeriphClockCmd(USART1_TX_CLK, ENABLE);							// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART1_RX_CLK, ENABLE);							// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART1_TX_PIN;								//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART1_RX_PIN;								//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART1_TX_PORT, USART1_TX_SOURCE, GPIO_AF_USART1);		//PA0引脚复用为USART1
	GPIO_PinAFConfig(USART1_RX_PORT, USART1_RX_SOURCE, GPIO_AF_USART1);		//PA1引脚复用为USART1
	
	USART_DeInit(USART1);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);					//USART1时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;								//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;				//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;						//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;						//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//收发模式
	USART_Init(USART1, &USART_InitStruct);									//初始化USART1
	
//	#if EN_USART1_RXIT == 1
////	USART_ITConfig(USART1, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);							//打开USART1的中断
//	#endif
//	
//	USART_Cmd(USART1, ENABLE);												//使能USART1

//	#if EN_USART1_RXIT == 1
//	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	#endif
	
#endif
}
#endif


/*USART2初始化*/
#if USART2_RS232_ENABLE == 1
static void init2(unsigned long int baudrate)
{
#ifdef DEBUG_PRINT
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;

	#if EN_USART2_RXIT == 1
	NVIC_InitTypeDef NVIC_InitStruct;
	#endif
	
	RCC_AHB1PeriphClockCmd(USART2_TX_CLK,ENABLE);							// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART2_RX_CLK,ENABLE);							// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART2_TX_PIN;								//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART2_RX_PIN;								//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART2_TX_PORT, USART2_TX_SOURCE, GPIO_AF_USART2);		//PA0引脚复用为USART2
	GPIO_PinAFConfig(USART2_RX_PORT, USART2_RX_SOURCE, GPIO_AF_USART2);		//PA1引脚复用为USART2
	
	USART_DeInit(USART2);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);					//USART2时钟使能
	  
	USART_InitStruct2.USART_BaudRate = baudrate;							//波特率
	USART_InitStruct2.USART_WordLength = USART_WordLength_8b;				//8个数据位
	USART_InitStruct2.USART_StopBits = USART_StopBits_1;					//1个停止位
	USART_InitStruct2.USART_Parity = USART_Parity_No;						//无奇偶校验位
	USART_InitStruct2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct2.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//收发模式
	USART_Init(USART2, &USART_InitStruct2);									//初始化USART2
	
//	#if EN_USART2_RXIT == 1
////	USART_ITConfig(USART2, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);							//打开USART2的中断
//	#endif
//	
//	USART_Cmd(USART2, ENABLE);												//使能USART2

//	#if EN_USART2_RXIT == 1
//	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	#endif
	
#endif
}
#endif


/*USART3初始化*/
#if USART3_RS232_ENABLE == 1
static void init3(unsigned long int baudrate)
{
#ifdef DEBUG_PRINT
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;

	#if EN_USART3_RXIT == 1
	NVIC_InitTypeDef NVIC_InitStruct;
	#endif
	
	RCC_AHB1PeriphClockCmd(USART3_TX_CLK, ENABLE);							// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART3_RX_CLK, ENABLE);							// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART3_TX_PIN;								//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART3_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART3_RX_PIN;								//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART3_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART3_TX_PORT, USART3_TX_SOURCE, GPIO_AF_USART3);		//PA0引脚复用为USART3
	GPIO_PinAFConfig(USART3_RX_PORT, USART3_RX_SOURCE, GPIO_AF_USART3);		//PA1引脚复用为USART3
	
	USART_DeInit(USART3);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);					//USART3时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;								//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;				//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;						//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;						//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//收发模式
	USART_Init(USART3, &USART_InitStruct);									//初始化USART3
	
//	#if EN_USART3_RXIT == 1
////	USART_ITConfig(USART3, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);							//打开USART3的中断
//	#endif
//	
//	USART_Cmd(USART3, ENABLE);												//使能USART3

//	#if EN_USART3_RXIT == 1
//	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	#endif
	
#endif
}
#endif


/*USART4初始化*/
#if USART4_RS232_ENABLE == 1
static void init4(unsigned long int baudrate)
{
#ifdef DEBUG_PRINT
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;

	#if EN_USART4_RXIT == 1
	NVIC_InitTypeDef NVIC_InitStruct;
	#endif
	
	RCC_AHB1PeriphClockCmd(USART4_TX_CLK, ENABLE);							// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART4_RX_CLK, ENABLE);							// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART4_TX_PIN;								//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART4_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART4_RX_PIN;								//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART4_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART4_TX_PORT, USART4_TX_SOURCE, GPIO_AF_UART4);		//PA0引脚复用为USART4
	GPIO_PinAFConfig(USART4_RX_PORT, USART4_RX_SOURCE, GPIO_AF_UART4);		//PA1引脚复用为USART4
	
	USART_DeInit(UART4);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);					//USART4时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;								//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;				//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;						//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;						//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//收发模式
	USART_Init(UART4, &USART_InitStruct);									//初始化USART4
	
//	#if EN_USART4_RXIT == 1
////	USART_ITConfig(UART4, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);							//打开USART4的中断
//	#endif
//	
//	USART_Cmd(UART4, ENABLE);												//使能USART4

//	#if EN_USART4_RXIT == 1
//	NVIC_InitStruct.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	#endif
	
#endif
}
#endif


/*USART5初始化*/
#if USART5_RS232_ENABLE == 1
static void init5(unsigned long int baudrate)
{
#ifdef DEBUG_PRINT
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
	#if EN_USART5_RXIT == 1
	NVIC_InitTypeDef NVIC_InitStruct;
	#endif
	
	RCC_AHB1PeriphClockCmd(USART5_TX_CLK,ENABLE);							// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART5_RX_CLK,ENABLE);							// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART5_TX_PIN;								//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART5_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART5_RX_PIN;								//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART5_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART5_TX_PORT, USART5_TX_SOURCE, GPIO_AF_UART5);		//PA0引脚复用为UART5
	GPIO_PinAFConfig(USART5_RX_PORT, USART5_RX_SOURCE, GPIO_AF_UART5);		//PA1引脚复用为UART5
	
	USART_DeInit(UART5);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);					//UART5时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;								//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;				//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;						//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;						//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//收发模式
	USART_Init(UART5, &USART_InitStruct);									//初始化UART5
	
//	#if EN_USART5_RXIT == 1
////	USART_ITConfig(UART5, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);							//打开UART5的中断
//	#endif
//	
//	USART_Cmd(UART5, ENABLE);												//使能UART5

//	#if EN_USART5_RXIT == 1
//	NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	#endif
	
#endif
}
#endif


/*USART6初始化*/
#if USART6_RS232_ENABLE == 1
static void init6(unsigned long int baudrate)
{
#ifdef DEBUG_PRINT
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;

	#if EN_USART6_RXIT == 1
	NVIC_InitTypeDef NVIC_InitStruct;
	#endif
	
	RCC_AHB1PeriphClockCmd(USART6_TX_CLK, ENABLE);							// GPIO时钟使能
	RCC_AHB1PeriphClockCmd(USART6_RX_CLK, ENABLE);							// GPIO时钟使能
	
	GPIO_InitStruct.GPIO_Pin = USART6_TX_PIN;								//PA0为复用推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART6_TX_PORT, &GPIO_InitStruct);    

	GPIO_InitStruct.GPIO_Pin = USART6_RX_PIN;								//PA1为浮空输入
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(USART6_RX_PORT, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(USART6_TX_PORT, USART6_TX_SOURCE, GPIO_AF_USART6);		//PA0引脚复用为USART6
	GPIO_PinAFConfig(USART6_RX_PORT, USART6_RX_SOURCE, GPIO_AF_USART6);		//PA1引脚复用为USART6
	
	USART_DeInit(USART6);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);					//USART6时钟使能
	  
	USART_InitStruct.USART_BaudRate = baudrate;								//波特率
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;				//8个数据位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;						//1个停止位
	USART_InitStruct.USART_Parity = USART_Parity_No;						//无奇偶校验位
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			//收发模式
	USART_Init(USART6, &USART_InitStruct);									//初始化USART6
	
//	#if EN_USART6_RXIT == 1
////	USART_ITConfig(USART6, USART_IT_PE, ENABLE);							//使能奇偶检验错误中断
//	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);							//打开USART6的中断
//	#endif
//	
//	USART_Cmd(USART6, ENABLE);												//使能USART6

//	#if EN_USART6_RXIT == 1
//	NVIC_InitStruct.NVIC_IRQChannel = USART6_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01; 
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//	#endif
	
#endif
}
#endif



/*
----------------------------------------------------------------------------
 * Name					: USART_IRQHandler
 * Description			: 中断函数
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
#if EN_USART4_RXIT == 1
void UART4_IRQHandler(void)                								//串口5中断服务程序
{
#ifdef DEBUG_PRINT
	
	u8 Res;
	static u32 value = 0x00000000;										//用于得到串口输入值
	static int i = 0;													//用于去除0x
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)				//接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(UART4);									//(USART1->DR);	//读取接收到的数据
		
		USART_SendData(UART4, Res);
					
		if(' ' == Res)
		{
			flag_RW = 1;
			i = 0;
			reg_Addr = value;
			value = 0x00000000;		
		}
		
		
		if(0 == (USART_RX_STA&0x8000))
		{
			if('!' != Res)
			{
//				USART_SendData(UART5, Res);
				if(i>1)
				{
					if((Res>='0')&&(Res<='9'))
					{
						value = (value<<4)+(Res-'0');
					}
					else if((Res>='A')&&(Res<='F'))
					{
						value = (value<<4)+(Res-'A'+10);
					}
					else if((Res>='a')&&(Res<='f'))
					{
						value = (value<<4)+(Res-'a'+10);
					}
					else
					{
						USART_RX_STA=0;//接收数据错误,重新开始接收
					}
				}
				++i;
				USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
				++USART_RX_STA;
				if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收
			}
			else
			{
//				USART_SendData(UART5, Res);
				USART_RX_STA|=0x8000;					//接收完成了
			}
		}
//				
//		if((USART_RX_STA&0x8000)==0)						//接收未完成
//		{
//			if(USART_RX_STA&0x4000)							//接收到了0x0d
//			{
//				if(Res!=0x0a)
//				{
//					USART_RX_STA=0;							//接收错误,重新开始
//				}
//				else
//				{
//					USART_RX_STA|=0x8000;					//接收完成了					
//				}
//			}
//			else //还没收到0X0D
//			{	
//				if(Res==0x0d)
//				{
//					USART_RX_STA|=0x4000;
//				}
//				else
//				{
//					if(i>1)
//					{
//						if((Res>='0')&&(Res<='9'))
//						{
//							value = (value<<4)+(Res-'0');
//						}
//						if((Res>='A')&&(Res<='F'))
//						{
//							value = (value<<4)+(Res-'A'+10);
//						}
//						if((Res>='a')&&(Res<='f'))
//						{
//							value = (value<<4)+(Res-'a'+10);
//						}
//					}
//					++i;
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//					++USART_RX_STA;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
//				}		 
//			}
//		}
		
		if(0 == USART_RX_STA)					//重新开始，参数初始化
		{
			reg_Addr = 0x00000000;
			reg_Data = 0x00000000;
			value = 0x00000000;
			flag_RW = 0;
			i = 0;
		}
		
		if((USART_RX_STA&0x8000) != 0)
		{
			if(0 == flag_RW)
			{
				reg_Addr = value;
				my_printf("The reg_Addr 0X%.8x is 0X%.8x!\n\r", reg_Addr, (*((volatile unsigned int *)reg_Addr)));
			}
			else
			{
				reg_Data = value;
				(*((volatile unsigned short int *)reg_Addr)) = reg_Data;
				my_printf("The reg_Addr 0X%.8x is 0X%.8x!\n\r", reg_Addr, (*((volatile unsigned int *)reg_Addr)));
			}
			USART_RX_STA = 0;
			reg_Addr = 0x00000000;
			reg_Data = 0x00000000;
			value = 0x00000000;
			flag_RW = 0;
			i = 0;		
		}
		
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
	
#endif
} 
#endif


/*
----------------------------------------------------------------------------
 * Name					: SendBuffer
 * Description			: USART_SendBuffer
 * Author				: zhenyonghit
 * return				:
 * Para					:
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
/*USART1的SendBuffer1*/
#if USART1_RS232_ENABLE == 1
static int SendBuffer1(void *buf, int len)
{
#ifdef DEBUG_PRINT
	
	u8 *p = (u8 *)buf;
	
	if(len <= 0) return -1;
	
	while(len --){
		SendByte1(*p);
		++p;
	}
	
#endif		
	
	return 0;
}
#endif


/*USART2的SendBuffer2*/
#if USART2_RS232_ENABLE == 1
static int SendBuffer2(void *buf, int len)
{
#ifdef DEBUG_PRINT
	
	u8 *p = (u8 *)buf;
	
	if(len <= 0) return -1;
	
	while(len --){
		SendByte2(*p);
		++p;
	}
	
#endif		
	
	return 0;
}
#endif


/*USART3的SendBuffer3*/
#if USART3_RS232_ENABLE == 1
static int SendBuffer3(void *buf, int len)
{
#ifdef DEBUG_PRINT
	
	u8 *p = (u8 *)buf;
	
	if(len <= 0) return -1;
	
	while(len --){
		SendByte3(*p);
		++p;
	}
	
#endif		
	
	return 0;
}
#endif


/*USART4的SendBuffer4*/
#if USART4_RS232_ENABLE == 1
static int SendBuffer4(void *buf, int len)
{
#ifdef DEBUG_PRINT
	
	u8 *p = (u8 *)buf;
	
	if(len <= 0) return -1;
	
	while(len --){
		SendByte4(*p);
		++p;
	}
	
#endif		
	
	return 0;
}
#endif


/*USART5的SendBuffer5*/
#if USART5_RS232_ENABLE == 1
static int SendBuffer5(void *buf, int len)
{
#ifdef DEBUG_PRINT
	
	u8 *p = (u8 *)buf;
	
	if(len <= 0) return -1;
	
	while(len --){
		SendByte5(*p);
		++p;
	}
	
#endif		
	
	return 0;
}
#endif


/*USART6的SendBuffer6*/
#if USART6_RS232_ENABLE == 1
static int SendBuffer6(void *buf, int len)
{
#ifdef DEBUG_PRINT
	
	u8 *p = (u8 *)buf;
	
	if(len <= 0) return -1;
	
	while(len --){
		SendByte6(*p);
		++p;
	}
	
#endif		
	
	return 0;
}
#endif




/*
----------------------------------------------------------------------------
 * Name					: SendByte
 * Description			: SendByte
 * Author				: zhenyonghit
 * return				:
 * Para					:
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
/*USART1的SendByte1*/
#if USART1_RS232_ENABLE == 1
static int SendByte1(char data)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = USART1;
		
	while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));		//等待发送数据完成
	USART_SendData(USARTx, data);									//将数据写入数据寄存器中
	
#endif

	return 0;	
}
#endif


/*USART2的SendByte2*/
#if USART2_RS232_ENABLE == 1
static int SendByte2(char data)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = USART2;
		
	while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));		//等待发送数据完成
	USART_SendData(USARTx, data);									//将数据写入数据寄存器中
	
#endif

	return 0;	
}
#endif


/*USART3的SendByte3*/
#if USART3_RS232_ENABLE == 1
static int SendByte3(char data)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = USART3;
		
	while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));		//等待发送数据完成
	USART_SendData(USARTx, data);									//将数据写入数据寄存器中
	
#endif

	return 0;	
}
#endif


/*USART4的SendByte4*/
#if USART4_RS232_ENABLE == 1
static int SendByte4(char data)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = UART4;
		
	while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));		//等待发送数据完成
	USART_SendData(USARTx, data);									//将数据写入数据寄存器中
	
#endif

	return 0;	
}
#endif


/*USART5的SendByte5*/
#if USART5_RS232_ENABLE == 1
static int SendByte5(char data)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = UART5;
		
	while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));		//等待发送数据完成
	USART_SendData(USARTx, data);									//将数据写入数据寄存器中
	
#endif

	return 0;	
}
#endif


/*USART6的SendByte6*/
#if USART6_RS232_ENABLE == 1
static int SendByte6(char data)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = UART6;
		
	while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));		//等待发送数据完成
	USART_SendData(USARTx, data);									//将数据写入数据寄存器中
	
#endif

	return 0;	
}
#endif



/*
----------------------------------------------------------------------------
 * Name					: SendString
 * Description			: SendString
 * Author				: zhenyonghit
 * return				:
 * Para					:
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
/*USART1的SendString1*/
#if USART1_RS232_ENABLE == 1
static int SendString1(char const * str)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = USART1;
		
	while(*str!='\0'){
		while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));
		USART_SendData(USARTx, *str++);	
	}
#endif
	
	return 0;
}
#endif


/*USART2的SendString2*/
#if USART2_RS232_ENABLE == 1
static int SendString2(char const * str)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = USART2;
		
	while(*str!='\0'){
		while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));
		USART_SendData(USARTx, *str++);	
	}
#endif
	
	return 0;
}
#endif


/*USART3的SendString3*/
#if USART3_RS232_ENABLE == 1
static int SendString3(char const * str)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = USART3;
		
	while(*str!='\0'){
		while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));
		USART_SendData(USARTx, *str++);	
	}
#endif
	
	return 0;
}
#endif


/*USART4的SendString4*/
#if USART4_RS232_ENABLE == 1
static int SendString4(char const * str)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = UART4;
		
	while(*str!='\0'){
		while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));
		USART_SendData(USARTx, *str++);	
	}
#endif
	
	return 0;
}
#endif


/*USART5的SendString5*/
#if USART5_RS232_ENABLE == 1
static int SendString5(char const * str)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = UART5;
		
	while(*str!='\0'){
		while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));
		USART_SendData(USARTx, *str++);	
	}
#endif
	
	return 0;
}
#endif


/*USART6的SendString6*/
#if USART6_RS232_ENABLE == 1
static int SendString6(char const * str)
{
#ifdef DEBUG_PRINT
	
	USART_TypeDef* USARTx = USART6;
		
	while(*str!='\0'){
		while(!(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == 1));
		USART_SendData(USARTx, *str++);	
	}
#endif
	
	return 0;
}
#endif




/*
----------------------------------------------------------------------------
 * Name					: my_printf
 * Description			: 调试打印信息——USART5
 * Author				: zhenyonghit
 * return				:
 * Para					:
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
int my_printf(const char * fmt,...)             		                    //串口4输出
{
	#ifdef DEBUG_PRINT
	
	__va_list arg_ptr; 
	char buf[UART_BUFFER_SIZE];
  		
	memset(buf, '\0', sizeof(buf));

	va_start(arg_ptr, fmt);
	vsprintf(buf,fmt, arg_ptr);
	va_end(arg_ptr);

	SendString3(buf);

	#endif
	
	return 0;	
}
