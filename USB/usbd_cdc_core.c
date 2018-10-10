/**
  ******************************************************************************
  * @file    usbd_cdc_core.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the high layer firmware functions to manage the
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *
  *  @verbatim
  *
  *          ===================================================================
  *                                CDC Class Driver Description
  *          ===================================================================
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class

  *           @note
  *             For the Abstract Control Model, this core allows only transmitting the requests to
  *             lower layer dispatcher (ie. usbd_cdc_vcp.c/.h) which should manage each request and
  *             perform relative actions.
  *
  *           These aspects may be enriched or modified for a specific user application.
  *
  *            This driver doesn't implement the following aspects of the specification
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"
#include "USB.h"
#include "Include_c.h"
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup usbd_cdc
  * @brief usbd core module
  * @{
  */

/** @defgroup usbd_cdc_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup usbd_cdc_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup usbd_cdc_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup usbd_cdc_Private_FunctionPrototypes
  * @{
  */

/*********************************************
   CDC Device library callbacks
 *********************************************/
static uint8_t  usbd_cdc_Init        (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cdc_DeInit      (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cdc_Setup       (void  *pdev, USB_SETUP_REQ *req);
//static uint8_t  usbd_cdc_EP0_RxReady  (void *pdev);
//static uint8_t  usbd_cdc_DataIn      (void *pdev, uint8_t epnum);
static uint8_t  usbd_cdc_DataOut     (void *pdev, uint8_t epnum);
static uint8_t  usbd_cdc_SOF         (void *pdev);

/*********************************************
   CDC specific management functions
 *********************************************/
//static void Handle_USBAsynchXfer  (void *pdev);
static uint8_t  *USBD_cdc_GetCfgDesc (uint8_t speed, uint16_t *length);
#ifdef USE_USB_OTG_HS
static uint8_t  *USBD_cdc_GetOtherCfgDesc (uint8_t speed, uint16_t *length);
#endif
extern uint32_t DCD_HandleUsbReset_ISR(USB_OTG_CORE_HANDLE *pdev);
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;


/**
  * @}
  */

/** @defgroup usbd_cdc_Private_Variables
  * @{
  */
//extern CDC_IF_Prop_TypeDef  APP_FOPS;
extern uint8_t USBD_DeviceDesc   [USB_SIZ_DEVICE_DESC];

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_cdc_CfgDesc  [46] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_cdc_OtherCfgDesc  [USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN static __IO uint32_t  usbd_cdc_AltSet  __ALIGN_END = 0;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USB_Rx_Buffer   [256] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t APP_Rx_Buffer   [APP_RX_DATA_SIZE] __ALIGN_END ;
__ALIGN_BEGIN uint8_t APP_Rx_Buffer1   [APP_RX_DATA_SIZE] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t CmdBuff[CDC_CMD_PACKET_SZE] __ALIGN_END ;

uint32_t APP_Rx_ptr_in  = 0;
uint32_t APP_Rx_ptr_out = 0;
uint32_t APP_Rx_length  = 0;


uint32_t APP_Rx_ptr_in1  = 0;
uint32_t APP_Rx_ptr_out1 = 0;
uint32_t APP_Rx_length1  = 0;

uint8_t  USB_Tx_State1 = 0;
uint8_t  USB_Tx_State = 0;

/* CDC interface class callbacks structure */
USBD_Class_cb_TypeDef  USBD_CDC_cb =
{
  usbd_cdc_Init,
  usbd_cdc_DeInit,
  usbd_cdc_Setup,
  NULL,                 /* EP0_TxSent, */
  NULL,//usbd_cdc_EP0_RxReady,
   NULL,//usbd_cdc_DataIn,
  usbd_cdc_DataOut,
  usbd_cdc_SOF,
  NULL,
  NULL,
  USBD_cdc_GetCfgDesc,
#ifdef USE_USB_OTG_HS
  USBD_cdc_GetOtherCfgDesc, /* use same cobfig as per FS */
#endif /* USE_USB_OTG_HS  */
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t usbd_cdc_CfgDesc[46]  __ALIGN_END =
{
	0x09, // bLength: Configuration Descriptor size
    USB_CONFIGURATION_DESCRIPTOR_TYPE, // bDescriptorType: Configuration
    46,//Size of cfgDesc - including the others descriptors
    0x00,
    0x01,         //bNumInterfaces: 1 interface
    0x01,         //bConfigurationValue: Configuration value
    0x00,         //iConfiguration: Index of string descriptor describing the configuration
    //0xC0,         //bmAttributes: bus powered
    0xE0,         //bmAttributes: bus powered and Support Remote Wake-up
    0x32,         //MaxPower 100 mA: this current is used for detecting Vbus

    //------Descriptor of Custom (vendor-defined) interface----------
    0x09,         // bLength: Interface Descriptor size
    USB_INTERFACE_DESCRIPTOR_TYPE, // bDescriptorType: Interface descriptor type
    0x00,         // bInterfaceNumber: Number of Interface
    0x00,         // bAlternateSetting: Alternate setting
    0x04,         // bNumEndpoints
    0xFF,         // bInterfaceClass: Vendor-specific
    0x00,         // bInterfaceSubClass
    0x00,         // nInterfaceProtocol
    0,            // iInterface: Index of string descriptor
    //Endpoint desciptors
    //Out endpoint
    7,				//bLenght
    USB_ENDPOINT_DESCRIPTOR_TYPE,	//bDescriptorType - endpoint
    1,		//bEndpointAdress, OUT
    0x02,			//bmAttributes - bulk, data, no synchronisation
    64,
    0,					//wMaxPacketSize
    0,				//bInterval - ignored for bulk
    //In endpoint


    7,				//bLenght
    USB_ENDPOINT_DESCRIPTOR_TYPE,	//bDescriptorType - endpoint
    0x82,		//bEndpointAdress, IN
    0x02,			//bmAttributes - bulk, data, no synchronisation
    64,
    0,				//wMaxPacketSize
    0	,			//bInterval - ignored for bulk

	 	 //In endpoint
    7,				//bLenght
    USB_ENDPOINT_DESCRIPTOR_TYPE,	//bDescriptorType - endpoint
    0x83,		//bEndpointAdress, IN
    0x02,			//bmAttributes - bulk, data, no synchronisation
    64,
    0,					//wMaxPacketSize
    0,				//bInterval - ignored for bulk

		 //In endpoint
    7,				//bLenght
    USB_ENDPOINT_DESCRIPTOR_TYPE,	//bDescriptorType - endpoint
    0x84,		//bEndpointAdress, IN
    0x02,			//bmAttributes - bulk, data, no synchronisation
    64,
    0,					//wMaxPacketSize
    0				//bInterval - ignored for bulk
};


/**
  * @}
  */

/** @defgroup usbd_cdc_Private_Functions
  * @{
  */

/**
  * @brief  usbd_cdc_Init
  *         Initilaize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_cdc_Init (void  *pdev,
                               uint8_t cfgidx)
{
    /* Open EP IN */
    DCD_EP_Open(pdev,
                1,
                64,
                USB_OTG_EP_BULK);

    /* Open EP OUT */
    DCD_EP_Open(pdev,
                0x82,
                64,
                USB_OTG_EP_BULK);


    DCD_EP_Open(pdev,
                0x83,
                64,
                USB_OTG_EP_BULK);

    /* Prepare Out endpoint to receive next packet */
    DCD_EP_PrepareRx(pdev,
                     1,
                     (uint8_t*)(USB_Rx_Buffer),
                     160);

    return USBD_OK;
}

/**
  * @brief  usbd_cdc_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_cdc_DeInit (void  *pdev,
                                 uint8_t cfgidx)
{
  /* Open EP IN */
  DCD_EP_Close(pdev,
              1);

  /* Open EP OUT */
  DCD_EP_Close(pdev,
              0x82);


  DCD_EP_Close(pdev,
              0x83);
// 关闭多余管道
//  DCD_EP_Close(pdev,
//              0x84);


  return USBD_OK;
}

/**
  * @brief  usbd_cdc_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  usbd_cdc_Setup (void  *pdev,
                                USB_SETUP_REQ *req)
{
    uint16_t len = USB_CDC_DESC_SIZ;
    uint8_t  *pbuf = usbd_cdc_CfgDesc + 9;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {

        /* Standard Requests -------------------------------*/
        case USB_REQ_TYPE_STANDARD:
            switch (req->bRequest)
                {
                    case USB_REQ_GET_DESCRIPTOR:
                        if( (req->wValue >> 8) == CDC_DESCRIPTOR_TYPE)
                        {
                            #ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
                                    pbuf = usbd_cdc_Desc;
                            #else
                                    pbuf = usbd_cdc_CfgDesc + 9 + (9 * USBD_ITF_MAX_NUM);
                            #endif
                            len = MIN(USB_CDC_DESC_SIZ , req->wLength);
                        }

                        USBD_CtlSendData (pdev,
                                          pbuf,
                                          len);
                        break;

                     case USB_REQ_GET_INTERFACE :
                         USBD_CtlSendData (pdev,
                                       (uint8_t *)&usbd_cdc_AltSet,
                                       1);
                     break;

                     case USB_REQ_SET_INTERFACE :
                        if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
                        {
                          usbd_cdc_AltSet = (uint8_t)(req->wValue);
                        }
                        else
                        {
                          /* Call the error management function (command will be nacked */
                          USBD_CtlError (pdev, req);
                        }
                        break;
              }
    }
    return USBD_OK;
}



/**
  * @brief  usbd_audio_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
/*static uint8_t  usbd_cdc_DataIn (void *pdev, uint8_t epnum)
{

  return USBD_OK;
}*/

/**
  * @brief  usbd_cdc_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cdc_DataOut (void *pdev, uint8_t epnum)
{
    uint16_t USB_Rx_Cnt;

    /* Get the received data buffer and update the counter */
    USB_Rx_Cnt = ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;


    iUSB_Data_Arrived = 1;
    USB_Read_Cnt = USB_Rx_Cnt;

    DCD_EP_PrepareRx(pdev, 1, (uint8_t*)(USB_Rx_Buffer), 176);
    return USBD_OK;
}

/**
  * @brief  usbd_audio_SOF
  *         Start Of Frame event management
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */

int32 iScopeWritePtrOld;
int32 iScopeCnt;
static uint8_t  usbd_cdc_SOF (void *pdev)
{
    uint16_t i = 0;
	uint16_t Index = 0;

    lResetCnt = 1;

	//20（ms） * 20(k) * 4(Byte) * 4(通道)
    iScopeCnt = iScopeWritePtr - iScopeWritePtrOld;
    if(iScopeCnt < 0)
    {
        iScopeCnt = iScopeCnt + SCOPE_BUFFER;
    }
    if(iScopeCnt >= SCOPE_MAXCNT)  //本应为SCOPE_MAXCNT-1，避免iScopeWritePtr已经加1，但是并未进缓存数组
    {
        Index = iScopeWritePtrOld;
        for (i = 0; i < SCOPE_MAXCNT; i++)
        {
            unUSBScopeDataTemp.Data.lCh1DataBuffer[i] = stUSBScopeData.lCh1DataBuffer[Index];
            unUSBScopeDataTemp.Data.lCh2DataBuffer[i] = stUSBScopeData.lCh2DataBuffer[Index];
            unUSBScopeDataTemp.Data.lCh3DataBuffer[i] = stUSBScopeData.lCh3DataBuffer[Index];
            unUSBScopeDataTemp.Data.lCh4DataBuffer[i] = stUSBScopeData.lCh4DataBuffer[Index];

            Index ++;
            if(Index >= SCOPE_BUFFER)
            {
                Index = 0;
            }
        }
        iScopeWritePtrOld = iScopeWritePtrOld + SCOPE_MAXCNT;
        if(iScopeWritePtrOld >= SCOPE_BUFFER)
        {
            iScopeWritePtrOld = iScopeWritePtrOld - SCOPE_BUFFER;
        }
        DCD_EP_Tx (pdev, 0x83, (uint8_t*)&unUSBScopeDataTemp.iWord[0], 6408);
    }
    return USBD_OK;
}


/**
  * @brief  USBD_cdc_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_cdc_GetCfgDesc (uint8_t speed, uint16_t *length)
{
    *length = sizeof (usbd_cdc_CfgDesc);
    return usbd_cdc_CfgDesc;
}

/**
  * @brief  USBD_cdc_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
#ifdef USE_USB_OTG_HS
static uint8_t  *USBD_cdc_GetOtherCfgDesc (uint8_t speed, uint16_t *length)
{
    *length = sizeof (usbd_cdc_OtherCfgDesc);
    return usbd_cdc_OtherCfgDesc;
}
#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
