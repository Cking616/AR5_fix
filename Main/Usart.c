/* ------------------------------------------------------------------------------
  File: chr6dm_usart.c
  Author: CH Robotics
  Version: 1.0
  
  Description: Functions and interrupt handlers for USART communication  
------------------------------------------------------------------------------ */ 
#include "stm32f4xx_conf.h"
#include "Usart.h"
#include "stdint.h"
//#include "Packet_handler.h"
//#include "FIFO.h"
//#include "bsp.h"
#include "HMIDataMaintain.h"
#include "MC_type.h"
#include "GlobalVariable.h"





DMA_InitTypeDef DMA_InitUSART1_TX;
//DMA_InitTypeDef DMA_InitUART4_TX;

DMA_InitTypeDef DMA_InitUSART1_RX;
//DMA_InitTypeDef DMA_InitUART4_RX;

u8 USART1_TXBuf[TX_BUF_SIZE]={0};
//u8 UART4_TXBuf[TX_BUF_SIZE];

u8 USART1_RXBuf[RX_BUF_SIZE]={0};
//u8 UART4_RXBuf[RX_BUF_SIZE];


u32  USART1_RXBufPtr=0;
//u32  UART4_RXBufPtr=0;

uint8_t USART1_State;
//uint8_t UART4_State;


/* Table of CRC values for high–order byte */
static unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;

/* Table of CRC values for low–order byte */
static char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
} ;

extern SystStatus_t StateRQ;
extern volatile SystStatus_t State;

extern vs32   PositionActual;  

extern vs32   PositionActual2;  

extern vs16 hTorque_Reference;

extern float h_BusV_Average;
extern s32 temperature;
extern Curr_Components Stat_Curr_q_d; 

extern vs16 hRot_Speed2;
extern vs16 hRot_Speed;


/*******************************************************************************
* Function Name  : USART_Init( int baud )
* Description    : Configures the USART at the selected baud rate
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
void USART1_Configuration(void)
{   
    USART_InitTypeDef USART_InitStructure;    
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 

    //USART clock source enable 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    //收发控制 PA11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    
    GPIO_ResetBits(GPIOA,GPIO_Pin_11); //使能485芯片接收
  

  
    //Configure USART1 Rx as alternate function 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
    //Connect PXx to USARTx_Tx 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    //Connect PXx to USARTx_Rx 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    
  
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
//        
//    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  
 

//    /* Enable the DMA Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;   // 发送DMA通道的中断配置
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 优先级设置
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);


    
    DMA_Cmd(DMA2_Stream7, DISABLE);                           // 关DMA通道
    DMA_DeInit(DMA2_Stream7);       

    //USART1 TX DMA2 channel config (triggered by USART1 Tx event)
    DMA_InitUSART1_TX.DMA_Channel = DMA_Channel_4;
    DMA_InitUSART1_TX.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    DMA_InitUSART1_TX.DMA_Memory0BaseAddr = (uint32_t)USART1_TXBuf;
    DMA_InitUSART1_TX.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitUSART1_TX.DMA_BufferSize = TX_BUF_SIZE;    // This will need to be set again
    DMA_InitUSART1_TX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitUSART1_TX.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitUSART1_TX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitUSART1_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitUSART1_TX.DMA_Mode = DMA_Mode_Normal;
    DMA_InitUSART1_TX.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitUSART1_TX.DMA_FIFOMode = DMA_FIFOMode_Disable;     
    DMA_InitUSART1_TX.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitUSART1_TX.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitUSART1_TX.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream7, &DMA_InitUSART1_TX);

  //  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);   
    //USART1接收 

    DMA_Cmd(DMA2_Stream2, DISABLE);                           // 关DMA通道
    DMA_DeInit(DMA2_Stream2);  
    DMA_InitUSART1_RX.DMA_Channel = DMA_Channel_4;  
    DMA_InitUSART1_RX.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    DMA_InitUSART1_RX.DMA_Memory0BaseAddr = (uint32_t)USART1_RXBuf;
    DMA_InitUSART1_RX.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitUSART1_RX.DMA_BufferSize = RX_BUF_SIZE;
    DMA_InitUSART1_RX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitUSART1_RX.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitUSART1_RX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitUSART1_RX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitUSART1_RX.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
    DMA_InitUSART1_RX.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitUSART1_RX.DMA_FIFOMode = DMA_FIFOMode_Disable;     
    DMA_InitUSART1_RX.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitUSART1_RX.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitUSART1_RX.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &DMA_InitUSART1_RX);
    //Enable DMA1 USART3 RX channel

//    DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);    

    
    DMA_Cmd(DMA2_Stream2, ENABLE);
    //Enable USART1 TX and RX DMA requests

    // Enable the TIM4 Interrupt 
//    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure); 
    
//    USART_InitStructure.USART_BaudRate = 1840000;

    USART_InitStructure.USART_BaudRate = 5120000;
//    USART_InitStructure.USART_BaudRate = 115200;

    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    //Configure USART
    USART_Init(USART1, &USART_InitStructure);
    // Enable USART1


    /* Enable USART3 Receive and Transmit interrupts */
   USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  // 开启 串口空闲IDEL 中断
//   USART_ITConfig(USART1,USART_IT_TC,ENABLE);
  
      USART_Cmd(USART1, ENABLE);

    
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
        
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); 
    
    USART1_State=USART_STATE_WAIT;
 



   // make_crc_table();
}

   

/*******************************************************************************
* Function Name  : 
* Description    :  
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void gSendUSART1CMD1(void)
{  

//     u32 ltime_cnt_now;
//     u32 ltime_cnt; 
//     
//     u32 lPrevious_time_cnt=TIM2->CNT; 
//     

//    DMA_Cmd(DMA2_Stream2, DISABLE);  

//    USART1_RXBufPtr=0;
    
    GPIO_SetBits(GPIOA,GPIO_Pin_11); //使能485芯片发送

    

    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7); 

//    USART_ClearITPendingBit(USART1, USART_IT_TC); 

 #if MODBUS_ID1_ENABLE ==1 
    
        USART1_TXBuf[0]=0X10;
        USART1_TXBuf[1]=0X10;
        USART1_TXBuf[2]=0x00;
        USART1_TXBuf[3]=0x88;
        USART1_TXBuf[4]=0X00;
        USART1_TXBuf[5]=0X02;
        USART1_TXBuf[6]=0xC2;
        USART1_TXBuf[7]=0xA3;

#endif

 #if MODBUS_ID2_ENABLE ==1 
    
        USART1_TXBuf[0]=0X20;
        USART1_TXBuf[1]=0X10;
        USART1_TXBuf[2]=0x00;
        USART1_TXBuf[3]=0x88;
        USART1_TXBuf[4]=0X00;
        USART1_TXBuf[5]=0X02;
        USART1_TXBuf[6]=0xC7;
        USART1_TXBuf[7]=0x53;

#endif

 #if MODBUS_ID3_ENABLE ==1 
    
        USART1_TXBuf[0]=0X30;
        USART1_TXBuf[1]=0X10;
        USART1_TXBuf[2]=0x00;
        USART1_TXBuf[3]=0x88;
        USART1_TXBuf[4]=0X00;
        USART1_TXBuf[5]=0X02;
        USART1_TXBuf[6]=0xC5;
        USART1_TXBuf[7]=0xC3;

#endif

 #if MODBUS_ID4_ENABLE ==1 
    
        USART1_TXBuf[0]=0X40;
        USART1_TXBuf[1]=0X10;
        USART1_TXBuf[2]=0x00;
        USART1_TXBuf[3]=0x88;
        USART1_TXBuf[4]=0X00;
        USART1_TXBuf[5]=0X02;
        USART1_TXBuf[6]=0xCE;
        USART1_TXBuf[7]=0xF3;

#endif

 #if MODBUS_ID5_ENABLE ==1 
    
        USART1_TXBuf[0]=0X50;
        USART1_TXBuf[1]=0X10;
        USART1_TXBuf[2]=0x00;
        USART1_TXBuf[3]=0x88;
        USART1_TXBuf[4]=0X00;
        USART1_TXBuf[5]=0X02;
        USART1_TXBuf[6]=0xCC;
        USART1_TXBuf[7]=0x63;

#endif

 #if MODBUS_ID6_ENABLE ==1 
    
        USART1_TXBuf[0]=0X60;
        USART1_TXBuf[1]=0X10;
        USART1_TXBuf[2]=0x00;
        USART1_TXBuf[3]=0x88;
        USART1_TXBuf[4]=0X00;
        USART1_TXBuf[5]=0X02;
        USART1_TXBuf[6]=0xC9;
        USART1_TXBuf[7]=0x93;

#endif

 #if MODBUS_ID7_ENABLE ==1 
    
        USART1_TXBuf[0]=0X70;
        USART1_TXBuf[1]=0X10;
        USART1_TXBuf[2]=0x00;
        USART1_TXBuf[3]=0x88;
        USART1_TXBuf[4]=0X00;
        USART1_TXBuf[5]=0X02;
        USART1_TXBuf[6]=0xCB;
        USART1_TXBuf[7]=0x03;

#endif


    DMA_Cmd(DMA2_Stream7, DISABLE);  

    DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);
    
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//等待DMA可配置 
    

    DMA_SetCurrDataCounter(DMA2_Stream7, 8);
		
  //  delay_us(100);

    DMA_Cmd(DMA2_Stream7, ENABLE);  

    //delay_us(100);

//    while((DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)&&(USART_GetITStatus(USART1,USART_IT_TC) != RESET))
//    {
////           ltime_cnt_now=TIM2->CNT;
//// 
////           ltime_cnt=ltime_cnt_now-lPrevious_time_cnt;


////           if ( ltime_cnt>8400 )
////           {
////                   DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7); 

////           }


//    }

    while(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
    {
//           ltime_cnt_now=TIM2->CNT;
// 
//           ltime_cnt=ltime_cnt_now-lPrevious_time_cnt;


//           if ( ltime_cnt>8400 )
//           {
//                   DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7); 

//           }


    }

////    //USART_ClearITPendingBit(USART1,USART_IT_TC); 
////    
    GPIO_ResetBits(GPIOA,GPIO_Pin_11); //使能485芯片接收

//    DMA_Cmd(DMA2_Stream2, ENABLE);  

    
    //getUsartdata2();

}


/*******************************************************************************
* Function Name  : 
* Description    :  
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void gSendUSART1CMD2(void)
{  
//     u32 ltime_cnt_now;
//     u32 ltime_cnt; 
//     
//     u32 lPrevious_time_cnt=TIM2->CNT;PositionActual/4000*360*1000/101; 
//PositionActual2/131072*360*1000
//     
    unsigned short iCRC;
//    s32 temp=PositionActual*90/101;

//        s32 temp=temperature;

//        s32 temp=PositionActual2/131072*360*1000;

//        s32 temp=PositionActual2*360*1000/131072;

        s32 temp=PositionActual2*2.74658;  //折算成输出端角度，单位0.001度

//          s16 temp2=hRot_Speed2*2746.58;

//         s16 temp2=hRot_Speed*1000/4000*360*1000/101;
//         s16 temp2=(float)hRot_Speed*891.09F;

#if BIG_ID_ENABLE==1

//         s16 temp2=hRot_Speed*1000/20000*360*1000/101;

 s16 temp2=hRot_Speed*17.82F;    //折算成输出端速度，单位0.01度

#endif

#if BIG_ID_ENABLE==0

 s16 temp2=hRot_Speed*89.1F;    //折算成输出端速度，单位0.01度

#endif

        
 //        s16 temp2=hRot_Speed;



//     DMA_Cmd(DMA2_Stream2, DISABLE);  

//     USART1_RXBufPtr=0;

    
    GPIO_SetBits(GPIOA,GPIO_Pin_11); //使能485芯片发送
   

    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7); 

//    USART_ClearITPendingBit(USART1, USART_IT_TC); 

 #if MODBUS_ID1_ENABLE ==1 
    
        USART1_TXBuf[0]=0X10;
       
#endif

 #if MODBUS_ID2_ENABLE ==1 
    
        USART1_TXBuf[0]=0X20;
       
#endif

 #if MODBUS_ID3_ENABLE ==1 
    
        USART1_TXBuf[0]=0X30;
       

#endif

 #if MODBUS_ID4_ENABLE ==1 
    
        USART1_TXBuf[0]=0X40;
        

#endif

 #if MODBUS_ID5_ENABLE ==1 
    
        USART1_TXBuf[0]=0X50;
       
#endif

 #if MODBUS_ID6_ENABLE ==1 
    
        USART1_TXBuf[0]=0X60;
        

#endif

 #if MODBUS_ID7_ENABLE ==1 
    
        USART1_TXBuf[0]=0X70;
        

#endif

//    USART1_TXBuf[0]=0X10;
    USART1_TXBuf[1]=0X04;
    USART1_TXBuf[2]=0X0C;
    USART1_TXBuf[3]=(u8)State;
    USART1_TXBuf[4]=0X00;
    USART1_TXBuf[8]=(u8)((temp&0xFF000000)>>24);
    USART1_TXBuf[7]=(u8)((temp&0x00FF0000)>>16);
    USART1_TXBuf[6]=(u8)((temp&0x0000FF00)>>8);
    USART1_TXBuf[5]=(u8)(temp&0x000000FF);
    
 //   USART1_TXBuf[9]=;
//    USART1_TXBuf[10]=;
    

//    USART1_TXBuf[11]=(u8)(Stat_Curr_q_d.qI_Component1&0x00FF);
//    USART1_TXBuf[12]=(u8)((Stat_Curr_q_d.qI_Component1&0xFF00)>>8);

    USART1_TXBuf[9]=(u8)(temp2&0x00FF);
    USART1_TXBuf[10]=(u8)((temp2&0xFF00)>>8);
    

//    USART1_TXBuf[11]=(s8)h_BusV_Average;
//    USART1_TXBuf[12]=(s8)temperature;

    USART1_TXBuf[11]=(u8)(Stat_Curr_q_d.qI_Component1&0x00FF);
    USART1_TXBuf[12]=(u8)((Stat_Curr_q_d.qI_Component1&0xFF00)>>8);

    USART1_TXBuf[13]=(s8)h_BusV_Average;
    USART1_TXBuf[14]=(s8)temperature;
    
    
    iCRC=CRC16(USART1_TXBuf,15);
    
    USART1_TXBuf[15]=(u8)((iCRC&0xFF00)>>8);
    USART1_TXBuf[16]=(u8)(iCRC&0x00FF);
    
    DMA_Cmd(DMA2_Stream7, DISABLE);  

    DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);
    
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//等待DMA可配置
    


    DMA_SetCurrDataCounter(DMA2_Stream7, 17);

   // delay_us(100);
    
    DMA_Cmd(DMA2_Stream7, ENABLE);  

//    while((DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)&&(USART_GetITStatus(USART1,USART_IT_TC) != RESET) )
//    {

////           ltime_cnt_now=TIM2->CNT;
//// 
////           ltime_cnt=ltime_cnt_now-lPrevious_time_cnt;


////           if ( ltime_cnt>8400 )
////           {

////                   DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7); 

////           }


//    }
    while(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
    {

//           ltime_cnt_now=TIM2->CNT;
// 
//           ltime_cnt=ltime_cnt_now-lPrevious_time_cnt;


//           if ( ltime_cnt>8400 )
//           {

//                   DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7); 

//           }


    }

////       // USART_ClearITPendingBit(USART1,USART_IT_TC); 
////    
        GPIO_ResetBits(GPIOA,GPIO_Pin_11); //使能485芯片接收

//        DMA_Cmd(DMA2_Stream2, ENABLE);  

 //   getUsartdata1();

}


#if Modbus_RTU_ENABLE ==0


/*******************************************************************************
* Function Name  : 
* Description    :  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void USART1_IRQHandler(void)
//{    

//    uint32_t temp = 0;  


// 
//    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
//    {  

//         //USART_ClearITPendingBit(USART1,USART_IT_IDLE); 

//         temp = USART1->SR;  
//         temp = USART1->DR; 

//         DMA_Cmd(DMA2_Stream2, DISABLE);

//		temp = DMA_GetCurrDataCounter(DMA2_Stream2);

//  
//    	  //检查对是否接收到数据
//        while( USART1_RXBufPtr != (RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream2)) )
//        {
//            USART1_ProcessNextCharacter(USART1_RXBuf[USART1_RXBufPtr++]); 
//          
//            if( USART1_RXBufPtr == RX_BUF_SIZE )
//            {
//                USART1_RXBufPtr = 0;
//            }
//        }


//        DMA_SetCurrDataCounter(DMA2_Stream2,RX_BUF_SIZE);

//        //打开DMA  
//        DMA_Cmd(DMA2_Stream2,ENABLE);  

//        
//    } 

//}    
#endif 




/*******************************************************************************
* Function Name  : 
* Input      : None
* Output     : None
* Return     : None
* Description    : 
*******************************************************************************/
//void HandleUSARTReception(void)
//{
//	  //检查对是否接收到数据
//    if( USART1_RXBufPtr != (RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream2)) )
//    {
//        USART1_ProcessNextCharacter(USART1_RXBuf[USART1_RXBufPtr++]); 
//      
//        if( USART1_RXBufPtr == RX_BUF_SIZE )
//        {
//            USART1_RXBufPtr = 0;
//        }
//    }
// 
//}


/*******************************************************************************
* Function Name  : 
* Input      : None
* Output     : None
* Return     : None
* Description    : 
*******************************************************************************/
void USART1_ProcessNextCharacter(u8 data)
{
     static uint8_t data_counter = 0;
     static uint8_t data_rev[RX_BUF_SIZE];

	
     u8 idex;

     unsigned short iCRC;
  
     switch( USART1_State )
     {
          case USART_STATE_WAIT:
             if( data_counter == 0 )    
             {

                 #if MODBUS_ID1_ENABLE ==1 
                    
                      if( data == 0X10 )
                     {
                         data_counter++;
                         data_rev[0]=data;
                     }
                     else
                     {
                         data_counter = 0;
                     }  
                     
                 #endif


                 #if MODBUS_ID2_ENABLE ==1 
                    
                      if( data == 0X20 )
                     {
                         data_counter++;
                         data_rev[0]=data;
                     }
                     else
                     {
                         data_counter = 0;
                     }  
                     
                 #endif

                 
                 #if MODBUS_ID3_ENABLE ==1 
                    
                      if( data == 0X30 )
                     {
                         data_counter++;
                         data_rev[0]=data;
                     }
                     else
                     {
                         data_counter = 0;
                     }  
                     
                 #endif



                 #if MODBUS_ID4_ENABLE ==1 
                    
                      if( data == 0X40 )
                     {
                         data_counter++;
                         data_rev[0]=data;
                     }
                     else
                     {
                         data_counter = 0;
                     }  
                     
                 #endif



                 #if MODBUS_ID5_ENABLE ==1 
                    
                      if( data == 0X50 )
                     {
                         data_counter++;
                         data_rev[0]=data;
                     }
                     else
                     {
                         data_counter = 0;
                     }  
                     
                 #endif


                 #if MODBUS_ID6_ENABLE ==1 
                    
                      if( data == 0X60 )
                     {
                         data_counter++;
                         data_rev[0]=data;
                     }
                     else
                     {
                         data_counter = 0;
                     }  
                     
                 #endif



                  #if MODBUS_ID7_ENABLE ==1 
                    
                      if( data == 0X70 )
                     {
                         data_counter++;
                         data_rev[0]=data;
                     }
                     else
                     {
                         data_counter = 0;
                     }  
                     
                 #endif                


                 
                 break;
                 
             }              

             if( data_counter == 1 )    
             { 
                 if( data == 0X10 )
                 {
                     
                     data_counter++;
                     data_rev[1]=data;

                     break;
                 }
                 

                  if( data == 0X04 )
                 {

                     data_counter++;
                     data_rev[1]=data;
                     USART1_State=USART_STATE_TYPE;
                     
                 }
                 else
                 {

                     data_counter = 0;

                 }

                 break;
                 
             }

             if ( data_counter == 2 )
             {

                 if( data == 0X00 )
                 {
                    
                     data_counter++;
                     data_rev[2]=data;
                 }
                 else
                 {
                     data_counter = 0;

                 }

                 break;
                 
             }

              if ( data_counter == 3 )
             {

                 if( data == 0X88 )
                 {
                     
                     data_counter++;
                     data_rev[3]=data;
                 }
                 else
                 {
                     data_counter = 0;
                 }

                 break;
                 
             }

              if ( data_counter == 4 )
             {

                 if( data == 0X00 )
                 {
                     
                     data_counter++;
                     data_rev[4]=data;
                 }
                 else
                 {
                     data_counter = 0;
                 }

                 break;
                 
             }

              if ( data_counter == 5 )
             {

                 if( data == 0X02 )
                 {
                    
                     data_counter++;
                     data_rev[5]=data;
                 }
                 else
                 {
                     data_counter = 0;
                 }

                 break;
                 
             }

              if ( data_counter == 6 )
             {

                 if( data == 0X04 )
                 {

                     data_counter++;                   
                     data_rev[6]=data;
                     USART1_State=USART_STATE_DATA;

                 }
                 else
                 {
                     data_counter = 0;
                 }
                 
             }
              else
              {
                  data_counter = 0;
                  USART1_State=USART_STATE_WAIT;          
              }
             
          break;

          case USART_STATE_TYPE:
             if ( data_counter == 2 )
             {

                 if( data == 0X00 )
                 {
                    
                     data_counter++;
        //             data_rev[2]=data;
                 }
                 else
                 {
                     data_counter = 0;
                     USART1_State=USART_STATE_WAIT;                     

                 }

                 break;
                 
             }

             if ( data_counter == 3 )
             {

                 if( data == 0X78 )
                 {
                     
                     data_counter++;
             //        data_rev[3]=data;
                 }
                 else
                 {
                     data_counter = 0;
                     USART1_State=USART_STATE_WAIT;
                 }

                 break;
                 
             }

              if ( data_counter == 4 )
             {

                 if( data == 0X00 )
                 {
                     
                     data_counter++;
         //            data_rev[4]=data;
                 }
                 else
                 {
                     data_counter = 0;
                     USART1_State=USART_STATE_WAIT;
                 }

                 break;
                 
             }

              if ( data_counter == 5 )
             {

                 if( data == 0X06 )
                 {
                    
                     data_counter++;
             //        data_rev[5]=data;
                 }
                 else
                 {
                     data_counter = 0;
                     USART1_State=USART_STATE_WAIT;
                 }

                 break;
                 
             }

#if MODBUS_ID1_ENABLE ==1 
                    
                      if ( data_counter == 6 )
                     {

                         if( data == 0XF3 )
                         {
                             data_counter ++;                  
                         //    data_rev[6]=data;
                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;

                         }

                         break;
                         
                     }	

                      if ( data_counter == 7 )
                     {

                         if( data == 0X50 )
                         {
                             
                             data_counter = 0;                   
                           //  data_rev[6]=data;
                             gSendUSART1CMD2();
                             USART1_State=USART_STATE_WAIT;


                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                         }

                         
                         
                     }	
                      else
                      {

                            data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                             
                      }
                  break;
                     
#endif



#if MODBUS_ID2_ENABLE ==1 
                    
                     if ( data_counter == 6 )
                     {

                         if( data == 0XF6 )
                         {
                             data_counter ++;                  
                         //    data_rev[6]=data;
                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;

                         }

                         break;
                         
                     }	

                      if ( data_counter == 7 )
                     {

                         if( data == 0XA0 )
                         {
                             
                             data_counter = 0;                   
                           //  data_rev[6]=data;
                             gSendUSART1CMD2();
                             USART1_State=USART_STATE_WAIT;


                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                         }

                         
                         
                     }	
                      else
                      {

                            data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                             
                      }
                  break;
                     

#endif



                 
#if MODBUS_ID3_ENABLE ==1 
                    
                     if ( data_counter == 6 )
                     {

                         if( data == 0XF4 )
                         {
                             data_counter ++;                  
                         //    data_rev[6]=data;
                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;

                         }

                         break;
                         
                     }	

                      if ( data_counter == 7 )
                     {

                         if( data == 0X30 )
                         {
                             
                             data_counter = 0;                   
                           //  data_rev[6]=data;
                             gSendUSART1CMD2();
                             USART1_State=USART_STATE_WAIT;


                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                         }

                         
                         
                     }	
                      else
                      {

                            data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                             
                      }
                  break;
                     
#endif




#if MODBUS_ID4_ENABLE ==1 
                    
                      if ( data_counter == 6 )
                     {

                         if( data == 0XFF )
                         {
                             data_counter ++;                  
                         //    data_rev[6]=data;
                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;

                         }

                         break;
                         
                     }	

                      if ( data_counter == 7 )
                     {

                         if( data == 0X00 )
                         {
                             
                             data_counter = 0;                   
                           //  data_rev[6]=data;
                             gSendUSART1CMD2();
                             USART1_State=USART_STATE_WAIT;


                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                         }

                         
                         
                     }	
                      else
                      {

                            data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                             
                      }
                  break;
                  

#endif




#if MODBUS_ID5_ENABLE ==1 
                    
                     if ( data_counter == 6 )
                     {

                         if( data == 0XFD )
                         {
                             data_counter ++;                  
                         //    data_rev[6]=data;
                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;

                         }

                         break;
                         
                     }	

                      if ( data_counter == 7 )
                     {

                         if( data == 0X90 )
                         {
                             
                             data_counter = 0;                   
                           //  data_rev[6]=data;
                             gSendUSART1CMD2();
                             USART1_State=USART_STATE_WAIT;


                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                         }

                         
                         
                     }	
                      else
                      {

                            data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                             
                      }
                  break;
                     

#endif




#if MODBUS_ID6_ENABLE ==1 

                    
                      if ( data_counter == 6 )
                     {

                         if( data == 0XF8 )
                         {
                             data_counter ++;                  
                         //    data_rev[6]=data;
                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;

                         }

                         break;
                         
                     }	

                      if ( data_counter == 7 )
                     {

                         if( data == 0X60 )
                         {
                             
                             data_counter = 0;                   
                           //  data_rev[6]=data;
                             gSendUSART1CMD2();
                             USART1_State=USART_STATE_WAIT;


                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                         }

                         
                         
                     }	
                      else
                      {

                            data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                             
                      }
                  break;
                     

#endif




#if MODBUS_ID7_ENABLE ==1 

                    
                     if ( data_counter == 6 )
                     {

                         if( data == 0XFA )
                         {
                             data_counter ++;                  
                         //    data_rev[6]=data;
                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;

                         }

                         break;
                         
                     }	

                      if ( data_counter == 7 )
                     {

                         if( data == 0XF0 )
                         {
                             
                             data_counter = 0;                   
                           //  data_rev[6]=data;
                             gSendUSART1CMD2();
                             USART1_State=USART_STATE_WAIT;


                         }
                         else
                         {
                             data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                         }

                         
                         
                     }	
                      else
                      {

                            data_counter = 0;
                             USART1_State=USART_STATE_WAIT;
                             
                      }
                  break;
                     

#endif                
                 

          

          case USART_STATE_DATA:
               data_rev[data_counter]=data;   
               data_counter++;

               if(data_counter>=13)
	        {

                     StateRQ=(SystStatus_t)data_rev[7];
                     hTorque_Reference=(s16)((data_rev[9])|data_rev[10]<<8);

                  
                     iCRC =(u16) ((data_rev[11]<< 8)|data_rev[12]);
			if( CRC16(data_rev,11)==iCRC)
			{   
									
				gSendUSART1CMD1();

			}
                  
		 
		       for(idex=0;idex<RX_BUF_SIZE;idex++)
		       {
				data_rev[idex]=0;
		       }

                     USART1_State=USART_STATE_WAIT;  
		       data_counter=0;
                                    
		 }								 
          break;

          
     }       
}


//void DMA2_Stream7_IRQHandler(void)
//{
//       
//        if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
//        {


//               GPIO_ResetBits(GPIOA,GPIO_Pin_11); //使能485芯片接收

//              
//          DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7); 
//          
//        }
//        
//}

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen)
{
    unsigned char uchCRCHi = 0xFF ; /* 初始化高字节*/
    unsigned char uchCRCLo = 0xFF ; /* 初始化低字节*/
    unsigned uIndex ; /*把CRC表*/
    
    while (usDataLen--) /*通过数据缓冲器*/
    {
        uIndex = uchCRCHi ^(*puchMsg++) ; /*计算CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }
    
    return (uchCRCHi << 8 | uchCRCLo) ;
}


//void DMA2_Stream2_IRQHandler(void)
//{
//       
//        if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2))
//        {

//               HandleUSARTReception();


//               DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2); 

//        }
//        
//}


//粗延时函数，微秒 
void delay_us(u16 time)
{         
      u16 i=0;      
      while(time--)   
     {       
         i=10;  //自己定义      
         while(i--)
            {}
      } 

}  

////毫秒级的延时  
//void delay_ms(u16 time) 
//{        

//        u16 i=0;      
//        while(time--)   
//        {       
//                  i=12000;  //自己定义       
//                   while(i--)
//                    {}
//        }
//        
//} 



/*******************************************************************************
* Function Name  : 
* Description    :  
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

//void getUsartdata1(void)
//{  

//     u32 ltime_cnt_now;
//     u32 ltime_cnt; 
//     
//     u32 lPrevious_time_cnt=TIM2->CNT; 

//     

//    GPIO_ResetBits(GPIOA,GPIO_Pin_11); //使能485芯片接收

//    DMA_Cmd(DMA2_Stream2, DISABLE);  

//    DMA_ClearFlag(DMA2_Stream2, DMA_IT_TCIF2);

//    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}//等待DMA可配置
//    
//    DMA_SetCurrDataCounter(DMA2_Stream2, 13);

//     DMA_Cmd(DMA2_Stream2, ENABLE);  

//    while(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) != RESET)
//    {

//           ltime_cnt_now=TIM2->CNT;
// 
//           ltime_cnt=ltime_cnt_now-lPrevious_time_cnt;


//           if ( ltime_cnt>8400 )
//           {
//                   DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); 

//           }

//               

//    }

//    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); 
//    
//    if ( ((USART1_RXBuf[11]<< 8)|USART1_RXBuf[12])== CRC16(USART1_RXBuf,11) )
//    {
//        if ( USART1_RXBuf[0]==0x10 )
//        {
//            if ( USART1_RXBuf[1] == 0x10 )
//            {
//                     StateRQ=(SystStatus_t)USART1_RXBuf[7];
//                     hTorque_Reference=(s16)((USART1_RXBuf[9])|USART1_RXBuf[10]<<8);
////                      for(u8 idex=0;idex<13;idex++)
////		       {
////				USART1_RXBuf[idex]=0;
////		       }
//                     gSendUSART1CMD1();

//            }
//        }
//    }

//    
// 

//}

//void getUsartdata2(void)
//{  

//     u32 ltime_cnt_now;
//     u32 ltime_cnt; 
//     
//     u32 lPrevious_time_cnt=TIM2->CNT; 
//    
//    GPIO_ResetBits(GPIOA,GPIO_Pin_11); //使能485芯片接收

//    DMA_Cmd(DMA2_Stream2, DISABLE);  

//    DMA_ClearFlag(DMA2_Stream2, DMA_IT_TCIF2);

//    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}//等待DMA可配置
//    
//    DMA_SetCurrDataCounter(DMA2_Stream2, 8);

//     DMA_Cmd(DMA2_Stream2, ENABLE);  

//    while(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) != RESET)
//    {

//           ltime_cnt_now=TIM2->CNT;
// 
//           ltime_cnt=ltime_cnt_now-lPrevious_time_cnt;


//           if ( ltime_cnt>8400 )
//           {
//                   DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); 

//           }



//    }

//    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); 
//    
//    if ( ((USART1_RXBuf[6]<< 8)|USART1_RXBuf[7])== CRC16(USART1_RXBuf,6) );
//    {
//        if ( USART1_RXBuf[0]==0x10 )
//        {
//            if ( USART1_RXBuf[1] == 0x04 )
//            {
//                     
//                     gSendUSART1CMD2();

//            }
//        }
//    }

//    

//}
