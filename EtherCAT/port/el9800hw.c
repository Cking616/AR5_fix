
/*--------------------------------------------------------------------------------------
------
------    Includes
------
--------------------------------------------------------------------------------------*/
#include "ecat_def.h"
#if STM32F407_HW
#include "SPI1.h"
#include "ecatslv.h"

#define    _EL9800HW_ 1
#include "el9800hw.h"
#undef    _EL9800HW_
/* ECATCHANGE_START(V5.11) ECAT10*/
/*remove definition of _EL9800HW_ (#ifdef is used in el9800hw.h)*/
/* ECATCHANGE_END(V5.11) ECAT10*/

#include "ecatappl.h"
#include "system.h"


#include "stm32f4xx_it.h"
#include "includes.h"
void mem_test(void);
void IRQ_EXTI0_Configuration(void);
void SYNC0_EXTI1_Configuration(void);
void SYNC1_EXTI2_Configuration(void);
/*--------------------------------------------------------------------------------------
------
------    internal Types and Defines
------
--------------------------------------------------------------------------------------*/

typedef union
{
    unsigned short    Word;
    unsigned char    Byte[2];
} UBYTETOWORD;

typedef union
{
    UINT8           Byte[2];
    UINT16          Word;
}
UALEVENT;

/*-----------------------------------------------------------------------------------------
------
------    SPI defines/macros
------
-----------------------------------------------------------------------------------------*/
#define SPI_DEACTIVE                    1
#define SPI_ACTIVE                        0


#if INTERRUPTS_SUPPORTED
/*-----------------------------------------------------------------------------------------
------
------    Global Interrupt setting
------
-----------------------------------------------------------------------------------------*/

#define 	DISABLE_GLOBAL_INT           __disable_irq()
#define 	ENABLE_GLOBAL_INT            __enable_irq()
#define    DISABLE_AL_EVENT_INT         DISABLE_GLOBAL_INT//NVIC_DisableIRQ(EXTI15_10_IRQn);// DISABLE_GLOBAL_INT
#define    ENABLE_AL_EVENT_INT          ENABLE_GLOBAL_INT// NVIC_EnableIRQ(EXTI15_10_IRQn);// ENABLE_GLOBAL_INT



/*-----------------------------------------------------------------------------------------
------
------    ESC Interrupt
------
-----------------------------------------------------------------------------------------*/
#if AL_EVENT_ENABLED
#define    INIT_ESC_INT           IRQ_EXTI0_Configuration();
#define    EcatIsr                EXTI15_10_IRQHandler//EXTI12_IRQHandler
#define    ACK_ESC_INT         	  EXTI_ClearITPendingBit(EXTI_Line12);
#define IS_ESC_INT_ACTIVE
#endif //#if AL_EVENT_ENABLED


/*-----------------------------------------------------------------------------------------
------
------    SYNC0 Interrupt
------
-----------------------------------------------------------------------------------------*/
#if DC_SUPPORTED && _STM32_IO8
#define    INIT_SYNC0_INT                 SYNC0_EXTI1_Configuration();
#define    Sync0Isr                       EXTI2_IRQHandler
#define    DISABLE_SYNC0_INT              NVIC_DisableIRQ(EXTI2_IRQn);
#define    ENABLE_SYNC0_INT               NVIC_EnableIRQ(EXTI2_IRQn);
#define    ACK_SYNC0_INT                  EXTI_ClearITPendingBit(EXTI_Line2);
#define    IS_SYNC0_INT_ACTIVE


/*ECATCHANGE_START(V5.10) HW3*/

#define    INIT_SYNC1_INT                  SYNC1_EXTI2_Configuration();
#define    Sync1Isr                        EXTI1_IRQHandler
#define    DISABLE_SYNC1_INT               NVIC_DisableIRQ(EXTI1_IRQn);
#define    ENABLE_SYNC1_INT                NVIC_EnableIRQ(EXTI1_IRQn);
#define    ACK_SYNC1_INT                   EXTI_ClearITPendingBit(EXTI_Line1);
#define    IS_SYNC1_INT_ACTIVE

/*ECATCHANGE_END(V5.10) HW3*/

#endif //#if DC_SUPPORTED && _STM32_IO8

#endif	//#if INTERRUPTS_SUPPORTED
/*-----------------------------------------------------------------------------------------
------
------    Hardware timer
------
-----------------------------------------------------------------------------------------*/
#if _STM32_IO8
#if ECAT_TIMER_INT
#define ECAT_TIMER_INT_STATE
#define ECAT_TIMER_ACK_INT        		 	TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);
#define TimerIsr                            TIM5_IRQHandler
#define ENABLE_ECAT_TIMER_INT               NVIC_EnableIRQ(TIM5_IRQn) ;
#define DISABLE_ECAT_TIMER_INT              NVIC_DisableIRQ(TIM5_IRQn) ;

#define INIT_ECAT_TIMER           			TIM_Configuration(10) ;

#define STOP_ECAT_TIMER            			DISABLE_ECAT_TIMER_INT;/*disable timer interrupt*/ \

#define START_ECAT_TIMER          			ENABLE_ECAT_TIMER_INT


#else    //#if ECAT_TIMER_INT

#define INIT_ECAT_TIMER      		TIM_Configuration(10);

#define STOP_ECAT_TIMER              TIM_Cmd(TIM5, DISABLE);

#define START_ECAT_TIMER              TIM_Cmd(TIM5, ENABLE);

#endif //#else #if ECAT_TIMER_INT

#elif _STM32_IO4

#if !ECAT_TIMER_INT
#define    ENABLE_ECAT_TIMER_INT       NVIC_EnableIRQ(TIM5_IRQn) ;
#define    DISABLE_ECAT_TIMER_INT      NVIC_DisableIRQ(TIM5_IRQn) ;
#define INIT_ECAT_TIMER               TIM_Configuration(10) ;
#define STOP_ECAT_TIMER              	TIM_Cmd(TIM5, DISABLE);
#define START_ECAT_TIMER           		TIM_Cmd(TIM5, ENABLE);

#else    //#if !ECAT_TIMER_INT

#warning "define Timer Interrupt Macros"

#endif //#else #if !ECAT_TIMER_INT
#endif //#elif _STM32_IO4

/*-----------------------------------------------------------------------------------------
------
------    Configuration Bits
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    LED defines
------
-----------------------------------------------------------------------------------------*/
#if _STM32_IO8
// EtherCAT Status LEDs -> StateMachine
#define LED_ECATGREEN
#define LED_ECATRED
#endif //_STM32_IO8


/*--------------------------------------------------------------------------------------
------
------    internal Variables
------
--------------------------------------------------------------------------------------*/
UALEVENT         EscALEvent;            //contains the content of the ALEvent register (0x220), this variable is updated on each Access to the Esc

/*--------------------------------------------------------------------------------------
------
------    internal functions
------
--------------------------------------------------------------------------------------*/


/*******************************************************************************
  Function:
    void GetInterruptRegister(void)

  Summary:
    The function operates a SPI access without addressing.

  Description:
    The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
    It will be saved in the global "EscALEvent"
  *****************************************************************************/
static void GetInterruptRegister(void)   //main��  ��״̬
{

      DISABLE_AL_EVENT_INT;
      //stTaskStat[2].lStartTim  = SysTick->VAL;
      HW_EscReadIsr((MEM_ADDR *)&EscALEvent.Word, 0x220, 2);
      //stTaskStat[2].lEndTim   = SysTick->VAL;
      ENABLE_AL_EVENT_INT;
      /*if(stTaskStat[2].lStartTim >= stTaskStat[2].lEndTim)
      {
          stTaskStat[2].lDeltaTim = stTaskStat[2].lStartTim - stTaskStat[2].lEndTim;
      }
      else
      {
          stTaskStat[2].lDeltaTim = (1 << 24) - stTaskStat[2].lEndTim + stTaskStat[2].lStartTim;
      }
      stTaskStat[2].fRatio     = (float32)stTaskStat[2].lDeltaTim * 100.0f / 8400.0f;
      stTaskStat[2].fRatioMax  = (stTaskStat[2].fRatioMax > stTaskStat[2].fRatio) ? stTaskStat[2].fRatioMax : stTaskStat[2].fRatio;
      */
}


/*******************************************************************************
  Function:
    void ISR_GetInterruptRegister(void)

  Summary:
    The function operates a SPI access without addressing.
        Shall be implemented if interrupts are supported else this function is equal to "GetInterruptRegsiter()"

  Description:
    The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
        It will be saved in the global "EscALEvent"
  *****************************************************************************/

static void ISR_GetInterruptRegister(void)
{
    HW_EscReadIsr((MEM_ADDR *)&EscALEvent.Word, 0x220, 2);
}

 /*
 * ��    ����LAN9252_reset
 * ��    �ܣ���λLAN9252��оƬ
 * ��ڲ�������
 * ���ڲ�������
 */
void LAN9252_reset(void)// PF8Ϊ#RST
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);     //EtherCAT_EN  ʹ���ڲ�1.2V
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_SET);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOC,GPIO_Pin_10);
    Delayms_666M(2000);
    GPIO_SetBits(GPIOC,GPIO_Pin_10);
    Delayms_666M(2000);
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0 if initialization was successful

 \brief    This function intialize the Process Data Interface (PDI) and the host controller.
*////////////////////////////////////////////////////////////////////////////////////////
UINT8 TestIsr;
UINT8 HW_Init(void)
{
	UINT16 intMask;
	UINT32 data;
	/*initialize the led and switch port*/
	//GPIO_Config();
	LAN9252_reset();// PF8Ϊ#RST
	/* initialize the SSP registers for the ESC SPI */
	SPI1_GPIO_Init();

	mem_test();// ����PDI�ӿ�

	/*initialize ADC configration*/
    //	Adc_Init();
    //ADC_Configuration();
	do
	{
		intMask = 0x0093;
		HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
		intMask = 0;
		HW_EscReadDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
	} while (intMask!= 0x0093);

	//IRQ enable,IRQ polarity, IRQ buffer type in Interrupt Configuration register.
    //Wrte 0x54 - 0x00000101
    data = 0x00000101;

    SPIWriteDWord (0x54,data);

    //Write in Interrupt Enable register -->
    //Write 0x5c - 0x00000001
    data = 0x00000001;
    SPIWriteDWord (0x5C, data);

    SPIReadDWord(0x58);
	intMask = 0x00;
    HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);


#if AL_EVENT_ENABLED
    INIT_ESC_INT;
    ENABLE_ESC_INT();
#endif

#if DC_SUPPORTED && _STM32_IO8
    INIT_SYNC0_INT
    //INIT_SYNC1_INT
    ENABLE_SYNC0_INT;
    //ENABLE_SYNC1_INT;
#endif

    INIT_ECAT_TIMER;
    START_ECAT_TIMER;

#if INTERRUPTS_SUPPORTED
    /* enable all interrupts */
    ENABLE_GLOBAL_INT;
#endif

    return 0;
}



/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This function shall be implemented if hardware resources need to be release
        when the sample application stops
*////////////////////////////////////////////////////////////////////////////////////////
void HW_Release(void)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    first two Bytes of ALEvent register (0x220)

 \brief  This function gets the current content of ALEvent register
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 HW_GetALEventRegister(void)
{
    GetInterruptRegister();
    return EscALEvent.Word;
}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    first two Bytes of ALEvent register (0x220)

 \brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
        The behaviour is equal to "HW_GetALEventRegister()"
*////////////////////////////////////////////////////////////////////////////////////////
#if _STM32_IO4  && AL_EVENT_ENABLED
/* the pragma interrupt_level is used to tell the compiler that these functions will not
   be called at the same time from the main function and the interrupt routine */
//#pragma interrupt_level 1
#endif
UINT16 HW_GetALEventRegister_Isr(void)
{
     ISR_GetInterruptRegister();
    return EscALEvent.Word;
}
#endif


#if UC_SET_ECAT_LED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param RunLed            desired EtherCAT Run led state
 \param ErrLed            desired EtherCAT Error led state

  \brief    This function updates the EtherCAT run and error led
*////////////////////////////////////////////////////////////////////////////////////////
void HW_SetLed(UINT8 RunLed,UINT8 ErrLed)
{
#if _STM32_IO8
 //     LED_ECATGREEN = RunLed;
//      LED_ECATRED   = ErrLed;
#endif
}
#endif //#if UC_SET_ECAT_LED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves read data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  This function operates the SPI read access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscRead( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    /* HBu 24.01.06: if the SPI will be read by an interrupt routine too the
                     mailbox reading may be interrupted but an interrupted
                     reading will remain in a SPI transmission fault that will
                     reset the internal Sync Manager status. Therefore the reading
                     will be divided in 1-byte reads with disabled interrupt */
    UINT16 i;
    UINT8 *pTmpData = (UINT8 *)pData;

    /* loop for all bytes to be read */
    while ( Len > 0 )
    {
        if (Address >= 0x1000)
        {
            i = Len;
        }
        else
        {
            i= (Len > 4) ? 4 : Len;

            if(Address & 01)
            {
               i=1;
            }
            else if (Address & 02)
            {
               i= (i&1) ? 1:2;
            }
            else if (i == 03)
            {
                i=1;
            }
        }


       DISABLE_AL_EVENT_INT;
       //stTaskStat[3].lStartTim  = SysTick->VAL;
       SPIReadDRegister(pTmpData,Address,i);
       //stTaskStat[3].lEndTim   = SysTick->VAL;
       ENABLE_AL_EVENT_INT;
       /*if(stTaskStat[3].lStartTim >= stTaskStat[3].lEndTim)
       {
           stTaskStat[3].lDeltaTim = stTaskStat[3].lStartTim - stTaskStat[3].lEndTim;
       }
       else
       {
           stTaskStat[3].lDeltaTim = (1 << 24) - stTaskStat[3].lEndTim + stTaskStat[3].lStartTim;
       }
       stTaskStat[3].fRatio     = (float32)stTaskStat[3].lDeltaTim * 100.0f / 8400.0f;
       stTaskStat[3].fRatioMax  = (stTaskStat[3].fRatioMax > stTaskStat[3].fRatio) ? stTaskStat[3].fRatioMax : stTaskStat[3].fRatio;
       */

       Len -= i;
       pTmpData += i;
       Address += i;
    }


}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves read data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

\brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
        The behaviour is equal to "HW_EscRead()"
*////////////////////////////////////////////////////////////////////////////////////////
#if _STM32_IO4  && AL_EVENT_ENABLED
/* the pragma interrupt_level is used to tell the compiler that these functions will not
   be called at the same time from the main function and the interrupt routine */
//#pragma interrupt_level 1
#endif
 void HW_EscReadIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{

   UINT16 i;
   UINT8 *pTmpData = (UINT8 *)pData;

    /* send the address and command to the ESC */

    /* loop for all bytes to be read */
   while ( Len > 0 )
   {

        if (Address >= 0x1000)
        {
            i = Len;
        }
        else
        {
            i= (Len > 4) ? 4 : Len;

            if(Address & 01)
            {
               i=1;
            }
            else if (Address & 02)
            {
               i= (i&1) ? 1:2;
            }
            else if (i == 03)
            {
                i=1;
            }
        }

        SPIReadDRegister(pTmpData, Address,i);

        Len -= i;
        pTmpData += i;
        Address += i;
    }

}
#endif //#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves write data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

  \brief  This function operates the SPI write access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscWrite( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{

    UINT16 i;
    UINT8 *pTmpData = (UINT8 *)pData;

    /* loop for all bytes to be written */
    while ( Len )
    {

        if (Address >= 0x1000)
        {
            i = Len;
        }
        else
        {
            i= (Len > 4) ? 4 : Len;

            if(Address & 01)
            {
               i=1;
            }
            else if (Address & 02)
            {
               i= (i&1) ? 1:2;
            }
            else if (i == 03)
            {
                i=1;
            }
        }

        DISABLE_AL_EVENT_INT;

        /* start transmission */
        SPIWriteRegister(pTmpData, Address, i);


        ENABLE_AL_EVENT_INT;



        /* next address */
        Len -= i;
        pTmpData += i;
        Address += i;

    }

}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves write data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  The SPI PDI requires an extra ESC write access functions from interrupts service routines.
        The behaviour is equal to "HW_EscWrite()"
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscWriteIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{

    UINT16 i ;
    UINT8 *pTmpData = (UINT8 *)pData;


    /* loop for all bytes to be written */
    while ( Len )
    {

        if (Address >= 0x1000)
        {
            i = Len;
        }
        else
        {
            i= (Len > 4) ? 4 : Len;

            if(Address & 01)
            {
               i=1;
            }
            else if (Address & 02)
            {
               i= (i&1) ? 1:2;
            }
            else if (i == 03)
            {
                i=1;
            }
        }

       /* start transmission */


       SPIWriteRegister(pTmpData, Address, i);


       /* next address */
        Len -= i;
        pTmpData += i;
        Address += i;
    }

}

#endif

/*
 * ����:  EcatIsr
 * ����:  IRQ�жϷ�����
 * ���:  ��
 * ����:  ��
 */
void  EcatIsr(void)
{
   PDI_Isr();

   /* reset the interrupt flag */
   ACK_ESC_INT;


}
#endif     // AL_EVENT_ENABLED



#if DC_SUPPORTED&& _STM32_IO8
/*
 * ����:  Sync0Isr
 * ����:  SYNC0�жϷ�����
 * ���:  ��
 * ����:  ��
 */
void Sync0Isr(void)
{

   Sync0_Isr();

   ACK_SYNC0_INT;

}
/*ECATCHANGE_START(V5.10) HW3*/
/*
 * ����:  Sync1Isr
 * ����:  SYNC1�жϷ�����
 * ���:  ��
 * ����:  ��
 */
void Sync1Isr(void)
{
   DISABLE_ESC_INT();
   Sync1_Isr();
	 ACK_SYNC1_INT;
   ENABLE_ESC_INT();

}
/*ECATCHANGE_END(V5.10) HW3*/
#endif

#if _STM32_IO8 && ECAT_TIMER_INT

/*
 * ����:  TimerIsr
 * ����:  timer�жϷ�����---��ʱ1ms---2000��ticks
 * ���:  ��
 * ����:  ��
 */
void TimerIsr(void)
{
		ECAT_CheckTimer();
		ECAT_TIMER_ACK_INT;
}

#endif

//#endif //#if EL9800_HW
/** @} */




