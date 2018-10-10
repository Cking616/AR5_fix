//########################################################################################
//
//                  Copyright (c) 2018  XXX INC.
//
// File Name:       usb.h
//
// Author:
//
// Version:         V1.0
//
// Date:            2018/04/17
//
// Description:
//
// History:
//
//########################################################################################
#ifndef _USB_H
#define _USB_H
#include "typedef.h"

#define SCOPE_MAXCNT   400
#define SCOPE_BUFFER   500
typedef	union
{
	int16	iWord[8];
	struct
	{
		Uint16	bHeadLow	: 8;
		Uint16	bHeadHigh	: 8;
		Uint16	bCmdIndex	: 8;
		Uint16	bParaIDLow	: 8;
		Uint16	bParaIDHigh	: 8;
		Uint16	bDataLength	: 8;

		Uint16	bByte0		: 8;
		Uint16	bByte1		: 8;
		Uint16	bByte2		: 8;
		Uint16	bByte3		: 8;
		Uint16	bByte4		: 8;
		Uint16	bByte5		: 8;
		Uint16	bByte6		: 8;
		Uint16	bByte7		: 8;
		Uint16	bResult		: 8;
		Uint16	bCRC		: 8;
	} Bits;
} UnionTypeUSBSingleData;

typedef union
{
	int32	lWord;
	struct
	{
		Uint16	bFrameHead	: 16;
		Uint16	bChIndex	: 4;
		Uint16	bDataCntr	: 4;
		Uint16	bFrameCntr	: 8;
	} Bits;
} UnionTypeScopeCtrlInfo;

typedef struct
{
	int32  lCh1DataBuffer[SCOPE_BUFFER];
	int32  lCh2DataBuffer[SCOPE_BUFFER];
	int32  lCh3DataBuffer[SCOPE_BUFFER];
	int32  lCh4DataBuffer[SCOPE_BUFFER];

} StructTypeUSBScopeData;

typedef union
{
	int8	iWord[SCOPE_MAXCNT * 16 + 8];
	struct
	{
        int32  head;
		int32  lCh1DataBuffer[SCOPE_MAXCNT];
		int32  lCh2DataBuffer[SCOPE_MAXCNT];
		int32  lCh3DataBuffer[SCOPE_MAXCNT];
		int32  lCh4DataBuffer[SCOPE_MAXCNT];
        int32  frameindex;
	} Data;
} UnionTypeUSBScopeDataTemp;

#ifdef _USB_C
	#define	PROTO
#else
	#define	PROTO	extern
#endif	// end of #ifdef _USB_C

PROTO  int32 iScopeWritePtr;
PROTO  int16 USB_Read_Cnt;
PROTO  int16 iUSB_Data_Arrived;
PROTO  int32 lResetCnt;

PROTO  UnionTypeUSBSingleData	    unUSBHeartBeatData;
PROTO  UnionTypeUSBSingleData	    unUSBSingleData;
PROTO  StructTypeUSBScopeData	    stUSBScopeData;
PROTO  UnionTypeUSBScopeDataTemp    unUSBScopeDataTemp;

PROTO  void USBInitial(void);
PROTO  void ResertUsb(void);
PROTO  void USB_DataRead(Uint8 *data,int16 length);
PROTO  int32 UsbFrameUnpack(UnionTypeUSBSingleData unRecivFrame, UnionTypeUSBSingleData* pSendFrame);
PROTO  void USB_DataUnpack(void);
PROTO  void USB_HeartBeat(void);
PROTO  void USB_ScopeDataBuffer(void);
PROTO  void USB_ScopeDataWrite(void);
PROTO  int16 USB_CRCCalculate(int16* iTxBufferPtr);
PROTO  void USB_DataWriteEndpoint2(int16 iLength, Uint8 *pBuff);

PROTO  void USB_Reply_Func(void);

#undef	PROTO

#endif

