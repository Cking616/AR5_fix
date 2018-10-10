//########################################################################################
//
//                  Copyright (c) 2018  XXX INC.
//
// File Name:       usb.c
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
#ifndef _USB_C
#define _USB_C

#include "Include_c.h"

uint8_t USB_Reply_Data[256];
int16 USB_Reply_Cnt;
int16 timer_counter;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

extern uint8_t USB_Rx_Buffer[256] ;
extern void  USB_OTG_CoreReset1(USB_OTG_CORE_HANDLE *pdev);
extern void  DCD_DevDisconnect  (USB_OTG_CORE_HANDLE *pdev);

//########################################################################################
// Function Name:   ResertUsb
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void ResertUsb(void)
{
    DCD_DevDisconnect(&USB_OTG_dev);
    USB_OTG_CoreReset1(&USB_OTG_dev);
    USBInitial();
}

//########################################################################################
// Function Name:   USBInitial
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void USBInitial(void)
{
    unUSBScopeDataTemp.Data.head = 0x55aaaa55;
    unUSBScopeDataTemp.Data.frameindex = 0;
    USBD_Init(&USB_OTG_dev,	USB_OTG_FS_CORE_ID,	&USR_desc, &USBD_CDC_cb, &USR_cb);
}

//########################################################################################
// Function Name:   AddToTransmit
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void AddToTransmit(int len,uint8_t* data)
{
    int i = 0;
    for( i = 0;i < len;i++)
    {
        USB_Reply_Data[USB_Reply_Cnt + i] = data[i];
    }
    USB_Reply_Cnt += len;
}

//########################################################################################
// Function Name:   USB_DataBuffer
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void USB_DataBuffer(Uint8 *data,int16 length)
{
      int16  i = 0 ,j = 0;
      int32  lResult = 0;
      UnionTypeUSBSingleData unUsbSendFrame;

      for (;i < length - 1;)
      {
          if((data[i] == 0xAA) && (data[i + 1] == 0x55))
          {
              for (j = 0; j < 8; j++)
              {
                  unUSBSingleData.iWord[j] = data[i] + (data[(i + 1) & 0xFF] * 256);
                  i = (i + 2) & 0xFF;
              }

              lResult = UsbFrameUnpack(unUSBSingleData, &unUsbSendFrame);
              if(!lResult)
              {
                  AddToTransmit(16,((Uint8 *)(&unUsbSendFrame)));
              }
          }
          else
          {
              i++;
          }
      }
}

//########################################################################################
// Function Name:   USB_Reply_Func
// Version:         V1.0
// Input:           none
// Output:  回复普通读取与设置参数，端口0x82
// Description:
//########################################################################################
void USB_Reply_Func()
{
    if(iUSB_Data_Arrived)
    {
        USB_Reply_Cnt = 0;
        USB_DataBuffer(USB_Rx_Buffer, USB_Read_Cnt);

        if(timer_counter >= 5)
        {
            timer_counter = 0;
            USB_HeartBeat();
        }
        timer_counter++;
        if(USB_Reply_Cnt > 0)
        {
		    DCD_EP_Tx (&USB_OTG_dev, 0x82, USB_Reply_Data, USB_Reply_Cnt );
        }
        iUSB_Data_Arrived = 0;
    }
}

//########################################################################################
// Function Name:   USB_CRCCalculate
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
int16 USB_CRCCalculate(int16* iTxBufferPtr)
{
    int16   i           = 0;
    int16   iCRCTemp    = 0;
    int16   iTemp       = 0;

    for (i = 0; i < 7; i++)
    {
        iTemp       = *(iTxBufferPtr + i);
        iCRCTemp    = iCRCTemp + ((iTemp & 0xFF00) >> 0x8) + (iTemp & 0x00FF);
    }

    iTemp       = *(iTxBufferPtr + 7);
    iCRCTemp    = iCRCTemp + (iTemp & 0x00FF);

    return (iCRCTemp & 0xFF);
}

//########################################################################################
// Function Name:   UsbFrameUnpack
// Input:           unRecivFrame
// Output:          iReturnValue 0:正常 1 CRC错误
// Description:     对USB协议的单个帧结构进行解析
//########################################################################################
int32 UsbFrameUnpack(UnionTypeUSBSingleData unRecivFrame, UnionTypeUSBSingleData* pSendFrame)
{
    int32 lReturnValue  = 0;
    int16 iDataLength   = 0;
    int32 lDataID       = 0;
    int32 lDataValue   = 0;

    //int32 lDataIDRaw;

    if ((USB_CRCCalculate(unRecivFrame.iWord) & 0x00FF) == unRecivFrame.Bits.bCRC)
    {
        lDataValue = (int32)((Uint32)unRecivFrame.iWord[4] * 65536) + (int32)((Uint32)unRecivFrame.iWord[3] & 0xFFFF);
        iDataLength = unRecivFrame.Bits.bDataLength;
        lDataID     = (int32)((Uint32)unRecivFrame.Bits.bParaIDHigh * 65536) + (int32)(((Uint32)unRecivFrame.Bits.bParaIDLow * 256) & 0xFFFF);

        if (lDataID >= 0x200000)
        {
            if(unRecivFrame.Bits.bCmdIndex == 0x01)//写操作
            {
				lReturnValue = DataInterface(COMM_PROTOCOL_DS402, COMM_DATA_INPUT, lDataID, (Uint16*)&iDataLength, &lDataValue);

			    pSendFrame->Bits.bHeadLow       = 0xAA;
                pSendFrame->Bits.bHeadHigh      = 0x55;
                pSendFrame->Bits.bCmdIndex      = 0x02;
                pSendFrame->Bits.bParaIDLow     = (lDataID & 0xFF00) >> 8;
                pSendFrame->Bits.bParaIDHigh    = (lDataID & 0xFF0000) >> 16;
                pSendFrame->Bits.bDataLength    = 0x03;
                pSendFrame->Bits.bByte0         = lDataValue & 0xFF;
                pSendFrame->Bits.bByte1         = (lDataValue & 0xFF00) >> 8;
                pSendFrame->Bits.bByte2         = (lDataValue & 0xFF0000) >> 16;
                pSendFrame->Bits.bByte3         = (lDataValue & 0xFF000000) >> 24;
                pSendFrame->Bits.bByte4         = 0;
                pSendFrame->Bits.bByte5         = 0;
                pSendFrame->Bits.bByte6         = 0;
                pSendFrame->Bits.bByte7         = 0;
                pSendFrame->Bits.bResult        = 0x01;
                pSendFrame->Bits.bCRC           = USB_CRCCalculate(pSendFrame->iWord);
            }
            else if (unRecivFrame.Bits.bCmdIndex == 0x03)// 读操作
            {
                lDataValue = 0;

			    // 读取内部数据
                lReturnValue    = DataInterface(COMM_PROTOCOL_DS402, COMM_DATA_OUTPUT, lDataID, (Uint16*)&iDataLength, &lDataValue);

				// 回复数据组帧
                pSendFrame->Bits.bHeadLow       = 0xAA;
                pSendFrame->Bits.bHeadHigh      = 0x55;
                pSendFrame->Bits.bCmdIndex      = 0x04;
                pSendFrame->Bits.bParaIDLow     = (lDataID & 0xFF00) >> 8;
                pSendFrame->Bits.bParaIDHigh    = (lDataID & 0xFF0000) >> 16;
                if (lReturnValue == 0)
                {
                    pSendFrame->Bits.bDataLength    = iDataLength;
                    pSendFrame->Bits.bByte0         = lDataValue & 0xFF;
                    pSendFrame->Bits.bByte1         = (lDataValue & 0xFF00) >> 8;
                    pSendFrame->Bits.bByte2         = (lDataValue & 0xFF0000) >> 16;
                    pSendFrame->Bits.bByte3         = (lDataValue & 0xFF000000) >> 24;
                    pSendFrame->Bits.bByte4         = 0;
                    pSendFrame->Bits.bByte5         = 0;
                    pSendFrame->Bits.bByte6         = 0;
                    pSendFrame->Bits.bByte7         = 0;
                    pSendFrame->Bits.bResult        = 0x0;
                }
                else
                {
                    pSendFrame->Bits.bDataLength    = 0x03;
                    pSendFrame->Bits.bByte0         = lDataValue & 0xFF;
                    pSendFrame->Bits.bByte1         = (lDataValue & 0xFF00) >> 8;
                    pSendFrame->Bits.bByte2         = (lDataValue & 0xFF0000) >> 16;
                    pSendFrame->Bits.bByte3         = (lDataValue & 0xFF000000) >> 24;
                    pSendFrame->Bits.bByte4         = 0;
                    pSendFrame->Bits.bByte5         = 0;
                    pSendFrame->Bits.bByte6         = 0;
                    pSendFrame->Bits.bByte7         = 0;
                    pSendFrame->Bits.bResult        = 0x01;
                }
                pSendFrame->Bits.bCRC           = USB_CRCCalculate(pSendFrame->iWord);
            }
        }
    }
    else
    {
	    lReturnValue = -1;
    }
    return lReturnValue;
}

//########################################################################################
// Function Name:   USB_HeartBeat
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void USB_HeartBeat(void)
{
    Uint8 hearbeat[16];

    hearbeat[0]    =  0xAA;
    hearbeat[1]    =  0x55;
    hearbeat[2]    =  0x04;
    hearbeat[3]    =  0xFF;
    hearbeat[4]    =  0xFF;
    hearbeat[5]    =  0x03;
    hearbeat[6]    =  0x33;
    hearbeat[7]    =  0xCC;
    hearbeat[8]    =  0xCC;
    hearbeat[9]    =  0x33;
    hearbeat[10]   =  0x0;
    hearbeat[11]   =  0x0;
    hearbeat[12]   =  0x0;
    hearbeat[13]   =  0x0;
    hearbeat[14]   =  0x0;
    hearbeat[15]   =  2;
    AddToTransmit(16, hearbeat);
}

//########################################################################################
// Function Name:   USB_ScopeDataBuffer
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void USB_ScopeDataBuffer(void)
{
	// 波形数据读取
    ScopeDataProcess(stScopePara.iChMode[0], stScopePara.iChParaLength[0], stScopePara.ulChIndex[0], &stScopePara.pChAddr[0], (Uint32*)&stUSBScopeData.lCh1DataBuffer[iScopeWritePtr]);
    ScopeDataProcess(stScopePara.iChMode[1], stScopePara.iChParaLength[1], stScopePara.ulChIndex[1], &stScopePara.pChAddr[1], (Uint32*)&stUSBScopeData.lCh2DataBuffer[iScopeWritePtr]);
    ScopeDataProcess(stScopePara.iChMode[2], stScopePara.iChParaLength[2], stScopePara.ulChIndex[2], &stScopePara.pChAddr[2], (Uint32*)&stUSBScopeData.lCh3DataBuffer[iScopeWritePtr]);
    ScopeDataProcess(stScopePara.iChMode[3], stScopePara.iChParaLength[3], stScopePara.ulChIndex[3], &stScopePara.pChAddr[3], (Uint32*)&stUSBScopeData.lCh4DataBuffer[iScopeWritePtr]);

    iScopeWritePtr ++;
    if (iScopeWritePtr >= SCOPE_BUFFER)
    {
        iScopeWritePtr = 0;
    }
}
#endif

