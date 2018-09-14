#ifndef __HARDWARE_CONFIG_H
#define __HARDWARE_CONFIG_H

/*
----------------------------------------------------------------------------
 * File					: 配置硬件信息
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



//-------------------------------- Define --------------------------------//
/*配置LED灯信息，LED连上定义为1，否则定义为0*/
#define	LED_1_ENABLE				1
#define	LED_2_ENABLE				1
#define	LED_3_ENABLE				0


/*配置RS485串口信息，串口用于RS485定义为1，否则定义为0*/
#define	USART1_RS485_ENABLE			1				/*用于JC和JD通信RS485*/
#define	USART2_RS485_ENABLE			0
#define	USART3_RS485_ENABLE			0
#define	USART4_RS485_ENABLE			0
#define	USART5_RS485_ENABLE			0
#define	USART6_RS485_ENABLE			0

/*配置RS232串口信息，串口用于RS232定义为1，否则定义为0*/
#define	USART1_RS232_ENABLE			0
#define	USART2_RS232_ENABLE			0
#define	USART3_RS232_ENABLE			0
#define	USART4_RS232_ENABLE			0				/*用于程序调试*/
#define	USART5_RS232_ENABLE			0
#define	USART6_RS232_ENABLE			0

/*配置串口中断信息：1——打开中断、0——关闭中断*/
#define EN_USART1_RXIT				1				/*开启串口1中断——JC和JD通信RS485*/
#define EN_USART2_RXIT				0				/*关闭串口2中断*/
#define EN_USART3_RXIT				0				/*关闭串口3中断*/
#define EN_USART4_RXIT				0				/*开启串口4中断——用于程序调试串口*/
#define EN_USART5_RXIT				0				/*关闭串口5中断*/
#define EN_USART6_RXIT				0				/*关闭串口6中断*/

//------------------------------- Typedef---------------------------------//



//---------------------------- Global Variable ---------------------------//



//---------------------------- Global Function ---------------------------//



//--------------------------------- Extern -------------------------------//



#endif

