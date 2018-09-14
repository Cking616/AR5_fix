#ifndef __RS485_JC2JD_ModBus_H
#define __RS485_JC2JD_ModBus_H

/*
----------------------------------------------------------------------------
 * File					: RS485_JC2JD_ModBus.h
 * Description			: 用RS485实现ModBus协议的发送和接受数据
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
#include "stm32f4xx.h"

#include "RS485_USART.h"
#include "RS485_DMA.h"

#include "GlobalVariable.h"
#include "HardwareConfig.h"

//-------------------------------- Define --------------------------------//
//#ifndef LITTLE_ENDIAN
//#define LITTLE_ENDIAN 1234
//#endif

//#ifndef BIG_ENDIAN
//#define BIG_ENDIAN 4321
//#endif

//#ifndef BYTE_ORDER
//#define BYTE_ORDER LITTLE_ENDIAN
//#endif

//#if BYTE_ORDER == BIG_ENDIAN
//	#define MB_HTONS(x) x
//	#define MB_NTOHS(x) x
//	#define MB_HTONL(x) x
//	#define MB_NTOHL(x) x
//#else /* BYTE_ORDER != BIG_ENDIAN */
//	#define MB_HTONS(x) ((((x) & 0xff) << 8) | (((x) & 0xff00) >> 8))
//	#define MB_NTOHS(x) MB_HTONS(x)
//	#define MB_HTONL(x) ((((x) & 0xff) << 24) | \
//                     (((x) & 0xff00) << 8) | \
//                     (((x) & 0xff0000UL) >> 8) | \
//                     (((x) & 0xff000000UL) >> 24))
//	#define MB_NTOHL(x) MB_HTONL(x)
//#endif

//#define MB_HTONS(x) ((((x) & 0xff) << 8) | (((x) & 0xff00) >> 8))
//#define MB_NTOHS(x) MB_HTONS(x)
//#define MB_HTONL(x) ((((x) & 0xff) << 8) | \
//					(((x) & 0xff00) >> 8) | \
//					(((x) & 0xff0000UL) << 8) | \
//					(((x) & 0xff000000UL) >> 8))
//#define MB_NTOHL(x) MB_HTONL(x)


//------------------------------- Typedef---------------------------------//
typedef enum
{
	TX_ABLE			= 0x00,					/*允许发送*/
	TX_ING			= 0x01,					/*正在发送*/
	TX_ED			= 0x03,					/*发送完成*/
	RX_ABLE			= 0x03,					/*允许接收*/
	RX_ING			= 0x04,					/*正在接收*/
	RX_ED			= 0x05					/*接收完成*/
}RS485_Status_t;							//RS485状态




typedef struct
{
	volatile unsigned int Status_Flag;					/*RS485状态标志位*/
	volatile unsigned int Error_Flag;					/*RS485错误标志位*/
	
//	int RS485_ReSend;									/*RS485错误重复发送次数，收到数据后清零*/
	
	volatile unsigned len_Tx;							/*Tx字符长度*/
	volatile unsigned len_Rx;							/*Rx字符长度*/
	
	
	RS485_DMA_USART_t *rs485_dma_usart;					/*DMA设备*/
	RS485_USART_t *rs485_usart;							/*USART设备*/
	
	void (*init)(unsigned long int);					/*用于初始化RS485*/
	
	void (*MSG_Tx)(u8 *, int);							/*用于发送数据*/
	
}RS485_JC2JD_t;

//---------------------------- Global Variable ---------------------------//
extern RS485_JC2JD_t rs485_JC2JD;


//---------------------------- Global Function ---------------------------//



//--------------------------------- Extern -------------------------------//




#endif

