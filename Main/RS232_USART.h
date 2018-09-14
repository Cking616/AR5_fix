#ifndef __RS232_USART_H
#define __RS232_USART_H

/*
----------------------------------------------------------------------------
 * File					: RS232_USART.h
 * Description			: 调试串口USART的配置
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
#include "stm32f4xx.h"
#include "HardwareConfig.h"

//-------------------------------- Define --------------------------------//
#ifdef DEBUG_PRINT
	#define UART_BUFFER_SIZE 100				/*最大打印信息长度*/	

	/* USART1的GPIO */
	#if USART1_RS232_ENABLE == 1
		#define USART1_TX_PORT		GPIOA
		#define USART1_TX_PIN		GPIO_Pin_9
		#define USART1_TX_CLK		RCC_AHB1Periph_GPIOA
		#define USART1_TX_SOURCE	GPIO_PinSource9

		#define USART1_RX_PORT		GPIOA
		#define USART1_RX_PIN		GPIO_Pin_10
		#define USART1_RX_CLK		RCC_AHB1Periph_GPIOA
		#define USART1_RX_SOURCE	GPIO_PinSource10
	#endif
		
	/* USART2的GPIO */
	#if USART2_RS232_ENABLE == 1
		#define USART2_TX_PORT      GPIOA
		#define USART2_TX_PIN       GPIO_Pin_2
		#define USART2_TX_CLK       RCC_AHB1Periph_GPIOA
		#define USART2_TX_SOURCE    GPIO_PinSource2

		#define USART2_RX_PORT      GPIOA
		#define USART2_RX_PIN       GPIO_Pin_3
		#define USART2_RX_CLK       RCC_AHB1Periph_GPIOA
		#define USART2_RX_SOURCE    GPIO_PinSource3
	#endif
	
	/* USART3的GPIO */
	#if USART3_RS232_ENABLE == 1
		#define USART3_TX_PORT      GPIOB
		#define USART3_TX_PIN       GPIO_Pin_10
		#define USART3_TX_CLK       RCC_AHB1Periph_GPIOB
		#define USART3_TX_SOURCE    GPIO_PinSource10

		#define USART3_RX_PORT      GPIOB
		#define USART3_RX_PIN       GPIO_Pin_11
		#define USART3_RX_CLK       RCC_AHB1Periph_GPIOB
		#define USART3_RX_SOURCE    GPIO_PinSource11
	#endif
	
	/* USART4的GPIO */
	#if USART4_RS232_ENABLE == 1
		#define USART4_TX_PORT      GPIOC
		#define USART4_TX_PIN       GPIO_Pin_10
		#define USART4_TX_CLK       RCC_AHB1Periph_GPIOC
		#define USART4_TX_SOURCE    GPIO_PinSource10

		#define USART4_RX_PORT      GPIOC
		#define USART4_RX_PIN       GPIO_Pin_11
		#define USART4_RX_CLK       RCC_AHB1Periph_GPIOC
		#define USART4_RX_SOURCE    GPIO_PinSource11
	#endif
	
	/* USART5的GPIO */
	#if USART5_RS232_ENABLE == 1
		#define USART5_TX_PORT      GPIOC
		#define USART5_TX_PIN       GPIO_Pin_12
		#define USART5_TX_CLK       RCC_AHB1Periph_GPIOC
		#define USART5_TX_SOURCE    GPIO_PinSource12

		#define USART5_RX_PORT      GPIOD
		#define USART5_RX_PIN       GPIO_Pin_2
		#define USART5_RX_CLK       RCC_AHB1Periph_GPIOD
		#define USART5_RX_SOURCE    GPIO_PinSource2
	#endif

	/* USART6的GPIO */
	#if USART6_RS232_ENABLE == 1
		#define USART6_TX_PORT      GPIOG
		#define USART6_TX_PIN       GPIO_Pin_14
		#define USART6_TX_CLK       RCC_AHB1Periph_GPIOG
		#define USART6_TX_SOURCE    GPIO_PinSource14

		#define USART6_RX_PORT      GPIOC
		#define USART6_RX_PIN       GPIO_Pin_7
		#define USART6_RX_CLK       RCC_AHB1Periph_GPIOC
		#define USART6_RX_SOURCE    GPIO_PinSource7
	#endif
#endif

#if EN_USART4_RXIT == 1
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#endif

//------------------------------- Typedef---------------------------------//

typedef struct
{
	void (*init)(unsigned long int);
//	void (*IT_TC_ENABLE)(void);
//	void (*IT_TC_DISABLE)(void);
//	void (*IT_RXNE_ENABLE)(void);
//	void (*IT_RXNE_DISABLE)(void);
	int (*SendBuffer)(void *, int);
	int (*SendByte)(char);
	int (*SendString)(char const*);
}RS232_USART_t;


//---------------------------- Global Variable ---------------------------//


//---------------------------- Global Function ---------------------------//
extern int my_printf(const char * ,...);


//--------------------------------- Extern -------------------------------//

/*定义每个RS232串口结构体变量*/
#if USART1_RS232_ENABLE == 1
	extern RS232_USART_t rs232_usart1;
#endif

#if USART2_RS232_ENABLE == 1
	extern RS232_USART_t rs232_usart2;
#endif

#if USART3_RS232_ENABLE == 1
	extern RS232_USART_t rs232_usart3;
#endif

#if USART4_RS232_ENABLE == 1
	extern RS232_USART_t rs232_usart4;
#endif

#if USART5_RS232_ENABLE == 1
	extern RS232_USART_t rs232_usart5;
#endif

#if USART6_RS232_ENABLE == 1
	extern RS232_USART_t rs232_usart6;
#endif


#if EN_USART4_RXIT == 1
extern u8  USART_RX_BUF[USART_REC_LEN];			//接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;						//接收状态标记
#endif

#endif

