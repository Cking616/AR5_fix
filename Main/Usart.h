/* ------------------------------------------------------------------------------
  File: chr6d_usart.h
  Author: CH Robotics
  Version: 1.0
  
  Description: Function declarations for USART communication
------------------------------------------------------------------------------ */ 
#include "stm32f4xx.h"
#include "stdint.h"
#ifndef __USART_H
#define __USART_H


#define	MODBUS_ID1_ENABLE			1				
#define	MODBUS_ID2_ENABLE			0
#define	MODBUS_ID3_ENABLE			0
#define	MODBUS_ID4_ENABLE			0
#define	MODBUS_ID5_ENABLE			0
#define	MODBUS_ID6_ENABLE			0
#define	MODBUS_ID7_ENABLE			0


// Size of TX and RX buffers. These buffers hold raw transmitted and received data from the USART.
// These buffers should never have to be accessed directly by user code.
#define    RX_BUF_SIZE         20
#define    TX_BUF_SIZE         20

// Maximum number of data bytes that can be stored in a packet
#define    MAX_PACKET_DATA        70 

// Sizes of buffers for storing RX and TX packet data.  The TX Packet Buffer is used
// to store packets that are "waiting" to be transmitted.  This is necessary since 
// multiple resources may need access to the USART hardware simultaneously.  The TX buffer
// acts as a moderator, storing packets and transmitting them when the hardware becomes available.
// The RX Packet Buffer stores packets received over the USART receiver.  Since multiple packets
// may arrive before a packet is processed, the RX Packet Buffer is neccessary to ensure that no data
// is lost.  Both the TX and RX Packet Buffers are circular.
#define    TX_PACKET_BUFFER_SIZE    10
#define    RX_PACKET_BUFFER_SIZE    10

// Definitions of states for USART receiver state machine (for receiving packets)
#define    USART_STATE_WAIT        1
#define    USART_STATE_TYPE        2
#define    USART_STATE_ADDRESS     3
#define    USART_STATE_DATA        4
#define    USART_STATE_CHECKSUM    5

// Flags for interpreting the packet type byte in communication packets
#define    PACKET_HAS_DATA        (1 << 7)
#define    PACKET_IS_BATCH        (1 << 6)
#define    PACKET_BATCH_LENGTH_MASK    (0x0F)

#define    PACKET_BATCH_LENGTH_OFFSET    2

#define    BATCH_SIZE_2        2
#define    BATCH_SIZE_3        3
#define    BATCH_SIZE_4        4
#define    BATCH_SIZE_5        5
#define    BATCH_SIZE_6        6


#define    PACKET_NO_DATA        0
#define    PACKET_COMMAND_FAILED    (1 << 0)

// Define flags for identifying the type of packet address received
#define    ADDRESS_TYPE_CONFIG        0
#define    ADDRESS_TYPE_DATA        1
#define    ADDRESS_TYPE_COMMAND    2
#define    ADDRESS_TYPE_FLOAT_ANGLE    3
#define USART1_DR_Base         ((uint32_t)0x40013804)


     
// Structure for storing TX and RX packet data
typedef struct __USARTPacket
{
     uint8_t PT;        // Packet type
     uint8_t address;    // Packet address
     uint16_t checksum;    // Checksum
     
     // Data included for convenience, but that isn't stored in the packet itself
     uint8_t data_length;     // Number of bytes in data section 
     uint8_t address_type;    // Specified the address type (DATA, CONFIG, OR COMMAND)
     
     uint8_t packet_data[MAX_PACKET_DATA];
} USARTPacket;


typedef struct __USARTStructure
{
    // Buffer, buffer index, and TX status flag for USART transmit
    char USART_TXBuf[TX_BUF_SIZE];   
    char USART_TXBusy;   
    // USART RX buffer and associated index and flags
    char USART_RXBuf[RX_BUF_SIZE];
    char USART_RXPacketReceived;
    char USART_RXBufOverrun;
  
    uint8_t USART_TXPacketBufferStart;
    uint8_t USART_TXPacketBufferEnd;
  
    uint8_t USART_RXPacketBufferStart;
    uint8_t USART_RXPacketBufferEnd;  
    // Flag for storing the current USART state
    uint8_t USART_State;  
    uint8_t gSendStateData;
    // Flags used to ensure that packet data isn't corrupted if
    // multiple sections of code try to access the buffer "simultaneously"
    // (can happen when interrupt-driven code accesses the buffer)
    char USART_TXPacketBufferReady;
    char Copying_USART_TXPacketToBuffer;  
  
    int32_t USART_TXBufPtr;
    int32_t USART_RXBufPtr;    
    // Queue for packets to be transmitted over the USART.  This is a FIFO circular buffer.
    USARTPacket USART_TXPacketBuffer[TX_PACKET_BUFFER_SIZE];
    // Queue for packets received by the USART.  This is a FIFO circular buffer.
    USARTPacket USART_RXPacketBuffer[RX_PACKET_BUFFER_SIZE];
    
  
} USARTStructure;


// USART interface functions
int32_t USART_transmit(USARTStructure* USARTSt, uint8_t* txdata, int32_t length );
int32_t USART_TXBufPush(USARTStructure* USARTSt, char txdata );
char USART_TXBufPop( USARTStructure* USARTSt );
void USART_TX_start(USARTStructure* USARTSt);

void hexPrint16( uint16_t byte );
void hexPrint8( uint8_t byte );


// Function for copying next packet in the TXPacketBuffer into the TX buffer.
void USART_SendNextPacket(USARTStructure* USARTSt);

void USART_AddTXPacket(USARTStructure* USARTSt, USARTPacket* new_packet );
void USART_AddRXPacket( USARTStructure*,USARTPacket* new_packet );
void HandleUSARTReception(void);
void USART_ProcessNextCharacter( USARTStructure* USARTSt );

uint16_t ComputeChecksum( USARTPacket* new_packet );
void USART_SendTXPacket(USARTStructure* USARTSt, USARTPacket* new_packet );
void USART_SendTXPacketSafe(USARTStructure* USARTSt, USARTPacket* new_packet );

void USART_Configuration(void);
void USARTStructureInt( USARTStructure* USARTSt,USART_TypeDef* USARTmun,DMA_Stream_TypeDef* DMA_Stream_tx,
                        DMA_Stream_TypeDef* DMA_Stream_rx, uint32_t DMAy_Channeltx, uint32_t DMAy_Channelrx);
void HandleUSARTSendBuff(USARTStructure* USARTSt);
void delay_us(u16 time);

void delay_ms(u16 time) ;

void HandleUSART(void);

void getUsartdata1(void);
void getUsartdata2(void);



extern USARTStructure gUSART;
void USART1_Configuration(void);

void gSendUSART1CMD1(void);
void gSendUSART1CMD2(void);

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);

//void gSendUART4(u8 data);
//u8 g_DrvCrc8_CalcBuff(const u8 * const pData, const u8 length);
//void make_crc_table( void ); 
void USART1_ProcessNextCharacter(u8 data);
//void UART4_ProcessNextCharacter(u8 data);
#endif

