#ifndef __RS485_USART_H
#define __RS485_USART_H

/*
----------------------------------------------------------------------------
 * File					: RS485_USART.h
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
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

#include "HardwareConfig.h"
#include "Configuration.h"
//-------------------------------- Define --------------------------------//
/*USART的引脚和TX使能引脚*/
/* USART1的GPIO */
#if USART1_RS485_ENABLE == 1
	#define USART1_TX_PORT		GPIOB
	#define USART1_TX_PIN		GPIO_Pin_6
	#define USART1_TX_CLK		RCC_AHB1Periph_GPIOB
	#define USART1_TX_SOURCE	GPIO_PinSource6

	#define USART1_RX_PORT		GPIOB
	#define USART1_RX_PIN		GPIO_Pin_7
	#define USART1_RX_CLK		RCC_AHB1Periph_GPIOB
	#define USART1_RX_SOURCE	GPIO_PinSource7


#if (HARDWARE_VERSION_2_0 == 1)||(HARDWARE_VERSION_2_1 == 1)
	/* RS485芯片发送使能GPIO，PA12 */
	#define RCC_USART1_TXEN		RCC_AHB1Periph_GPIOA
	#define PORT_USART1_TXEN	GPIOA
	#define PIN_USART1_TXEN		GPIO_Pin_12
#endif

#if HARDWARE_VERSION_2_2 == 1
	/* RS485芯片发送使能GPIO，PA6 */
	#define RCC_USART1_TXEN		RCC_AHB1Periph_GPIOA
	#define PORT_USART1_TXEN	GPIOA
	#define PIN_USART1_TXEN		GPIO_Pin_6
#endif
	
#endif

/* USART2的GPIO */
#if USART2_RS485_ENABLE == 1
	#define USART2_TX_PORT      GPIOA
	#define USART2_TX_PIN       GPIO_Pin_2
	#define USART2_TX_CLK       RCC_AHB1Periph_GPIOA
	#define USART2_TX_SOURCE    GPIO_PinSource2

	#define USART2_RX_PORT      GPIOA
	#define USART2_RX_PIN       GPIO_Pin_3
	#define USART2_RX_CLK       RCC_AHB1Periph_GPIOA
	#define USART2_RX_SOURCE    GPIO_PinSource3
	
	/* RS485芯片发送使能GPIO，PB2 */
	#define RCC_USART2_TXEN		RCC_AHB1Periph_GPIOA
	#define PORT_USART2_TXEN	GPIOA
	#define PIN_USART2_TXEN		GPIO_Pin_11
#endif

/* USART3的GPIO */
#if USART3_RS485_ENABLE == 1
	#define USART3_TX_PORT      GPIOB
	#define USART3_TX_PIN       GPIO_Pin_10
	#define USART3_TX_CLK       RCC_AHB1Periph_GPIOB
	#define USART3_TX_SOURCE    GPIO_PinSource10

	#define USART3_RX_PORT      GPIOB
	#define USART3_RX_PIN       GPIO_Pin_11
	#define USART3_RX_CLK       RCC_AHB1Periph_GPIOB
	#define USART3_RX_SOURCE    GPIO_PinSource11
	
	/* RS485芯片发送使能GPIO，PB2 */
	#define RCC_USART3_TXEN		RCC_AHB1Periph_GPIOA
	#define PORT_USART3_TXEN	GPIOA
	#define PIN_USART3_TXEN		GPIO_Pin_11
#endif

/* USART4的GPIO */
#if USART4_RS485_ENABLE == 1
	#define USART4_TX_PORT      GPIOC
	#define USART4_TX_PIN       GPIO_Pin_10
	#define USART4_TX_CLK       RCC_AHB1Periph_GPIOC
	#define USART4_TX_SOURCE    GPIO_PinSource10

	#define USART4_RX_PORT      GPIOC
	#define USART4_RX_PIN       GPIO_Pin_11
	#define USART4_RX_CLK       RCC_AHB1Periph_GPIOC
	#define USART4_RX_SOURCE    GPIO_PinSource11
	
	/* RS485芯片发送使能GPIO，PC8 */
	#define RCC_USART4_TXEN		RCC_AHB1Periph_GPIOC
	#define PORT_USART4_TXEN	GPIOC
	#define PIN_USART4_TXEN		GPIO_Pin_8
#endif

/* USART5的GPIO */
#if USART5_RS485_ENABLE == 1
	#define USART5_TX_PORT      GPIOC
	#define USART5_TX_PIN       GPIO_Pin_12
	#define USART5_TX_CLK       RCC_AHB1Periph_GPIOC
	#define USART5_TX_SOURCE    GPIO_PinSource12

	#define USART5_RX_PORT      GPIOD
	#define USART5_RX_PIN       GPIO_Pin_2
	#define USART5_RX_CLK       RCC_AHB1Periph_GPIOD
	#define USART5_RX_SOURCE    GPIO_PinSource2
	
	/* RS485芯片发送使能GPIO，PB2 */
	#define RCC_USART5_TXEN		RCC_AHB1Periph_GPIOA
	#define PORT_USART5_TXEN	GPIOA
	#define PIN_USART5_TXEN		GPIO_Pin_11
#endif

/* USART6的GPIO */
#if USART6_RS485_ENABLE == 1
	#define USART6_TX_PORT      GPIOG
	#define USART6_TX_PIN       GPIO_Pin_14
	#define USART6_TX_CLK       RCC_AHB1Periph_GPIOG
	#define USART6_TX_SOURCE    GPIO_PinSource14

	#define USART6_RX_PORT      GPIOC
	#define USART6_RX_PIN       GPIO_Pin_7
	#define USART6_RX_CLK       RCC_AHB1Periph_GPIOC
	#define USART6_RX_SOURCE    GPIO_PinSource7
	
	/* RS485芯片发送使能GPIO，PB2 */
	#define RCC_USART6_TXEN		RCC_AHB1Periph_GPIOA
	#define PORT_USART6_TXEN	GPIOA
	#define PIN_USART6_TXEN		GPIO_Pin_11
#endif

/*宏定义访问方式*/
#define USART_TX_PORT(x)			USART##x##_TX_PORT
#define USART_TX_PIN(x)				USART##x##_TX_PIN
#define USART_TX_CLK(x)				USART##x##_TX_CLK
#define USART_TX_SOURCE(x)			USART##x##_TX_SOURCE

#define USART_RX_PORT(x)			USART##x##_RX_PORT
#define USART_RX_PIN(x)				USART##x##_RX_PIN
#define USART_RX_CLK(x)				USART##x##_RX_CLK
#define USART_RX_SOURCE(x)			USART##x##_RX_SOURCE

#define RCC_USART_TXEN(x)			RCC_USART##x##_TXEN
#define PORT_USART_TXEN(x)			PORT_USART##x##_TXEN
#define PIN_USART_TXEN(x)			PIN_USART##x##_TXEN

/*宏定义，RS485发送使能*/
#define RS485_RX_EN(x)		(PORT_USART##x##_TXEN->BSRRH = PIN_USART##x##_TXEN)			//USARTx接收使能，ResetBits，设置为0
#define RS485_TX_EN(x)		(PORT_USART##x##_TXEN->BSRRL = PIN_USART##x##_TXEN)			//USARTx发送使能，SetBits，设置为1

//------------------------------- Typedef---------------------------------//
/*定义RS485_USART结构体*/
typedef struct
{
	void (*init)(unsigned long int);
	void (*initTXE)(void);
}RS485_USART_t;


//---------------------------- Global Variable ---------------------------//



//---------------------------- Global Function ---------------------------//
/*定义每个串口中断函数*/
#if USART1_RS485_ENABLE == 1
	extern void USART1_IRQHandler(void);
#endif

#if USART2_RS485_ENABLE == 1
	extern void USART2_IRQHandler(void);
#endif

#if USART3_RS485_ENABLE == 1
	extern void USART3_IRQHandler(void);
#endif

#if USART4_RS485_ENABLE == 1
	extern void UART4_IRQHandler(void);
#endif

#if USART5_RS485_ENABLE == 1
	extern void UART5_IRQHandler(void);
#endif

#if USART6_RS485_ENABLE == 1
	extern void USART6_IRQHandler(void);
#endif


//--------------------------------- Extern -------------------------------//
/*定义RS485_USART全局变量*/
#if USART1_RS485_ENABLE == 1
	extern RS485_USART_t rs485_usart1;
#endif

#if USART2_RS485_ENABLE == 1
	extern RS485_USART_t rs485_usart2;
#endif

#if USART3_RS485_ENABLE == 1
	extern RS485_USART_t rs485_usart3;
#endif

#if USART4_RS485_ENABLE == 1
	extern RS485_USART_t rs485_usart4;
#endif

#if USART5_RS485_ENABLE == 1
	extern RS485_USART_t rs485_usart5;
#endif

#if USART6_RS485_ENABLE == 1
	extern RS485_USART_t rs485_usart6;
#endif

//extern u8 rcv_Test[100];

#endif

