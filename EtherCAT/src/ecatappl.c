/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecatslv.h"

#define    _ECATAPPL_ 1
#include "ecatappl.h"
#undef _ECATAPPL_
/* ECATCHANGE_START(V5.11) ECAT10*/
/*remove definition of _ECATAPPL_ (#ifdef is used in ecatappl.h)*/
/* ECATCHANGE_END(V5.11) ECAT10*/

#include "coeappl.h"


/* ECATCHANGE_START(V5.11) ECAT11*/
#define _APPL_INTERFACE_ 1
#include "applInterface.h"
#undef _APPL_INTERFACE_
/* ECATCHANGE_END(V5.11) ECAT11*/

#include "cia402appl.h"

#include "stm32f4xx_it.h"


/*--------------------------------------------------------------------------------------
------
------    local Types and Defines
------
--------------------------------------------------------------------------------------*/


#ifndef ECAT_TIMER_INC_P_MS
/**
 * \todo Define the timer ticks per ms
 */
#warning "Define the timer ticks per ms"
#endif /* #ifndef ECAT_TIMER_INC_P_MS */


/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/
/*variables only required to calculate values for SM Synchronisation objects (0x1C3x)*/
UINT16 u16BusCycleCntMs;        //used to calculate the bus cycle time in Ms
UINT32 StartTimerCnt;    //variable to store the timer register value when get cycle time was triggered
BOOL bCycleTimeMeasurementStarted; // indicates if the bus cycle measurement is started

UINT16             aPdOutputData[(MAX_PD_OUTPUT_SIZE>>1)];
UINT16           aPdInputData[(MAX_PD_INPUT_SIZE>>1)];

/*variables are declared in ecatslv.c*/
    extern VARVOLATILE UINT16    u16dummy;
BOOL bInitFinished = FALSE; /** < \brief indicates if the initialization is finished*/
/*-----------------------------------------------------------------------------------------
------
------    local functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    Functions
------
-----------------------------------------------------------------------------------------*/


/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void PDO_InputMapping(void)
{
    APPL_InputMapping((UINT16*)aPdInputData);
    HW_EscWriteIsr(((MEM_ADDR *) aPdInputData), nEscAddrInputData, nPdInputSize );
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will copies the outputs from the ESC memory to the local memory
          to the hardware. This function is only called in case of an SM2
          (output process data) event.
*////////////////////////////////////////////////////////////////////////////////////////
void PDO_OutputMapping(void)
{
    HW_EscReadIsr(((MEM_ADDR *)aPdOutputData), nEscAddrOutputData, nPdOutputSize );  //读SM2对应的物理地址0x1200中的值
    APPL_OutputMapping((UINT16*) aPdOutputData);
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This function shall be called every 1ms.
 \brief If the switch ECAT_TIMER_INT is 0, the watchdog control is implemented without using
 \brief interrupts. In this case a local timer register is checked every ECAT_Main cycle
 \brief and the function is triggered if 1 ms is elapsed
 *////////////////////////////////////////////////////////////////////////////////////////

void ECAT_CheckTimer(void)
{
    if(sSyncManOutPar.u32CycleTime == 0)
    {
        u16BusCycleCntMs++;
    }

    /*decrement the state transition timeout counter*/
    if(bEcatWaitForAlControlRes &&  (EsmTimeoutCounter > 0))
    {
        EsmTimeoutCounter--;
    }



     DC_CheckWatchdog();
}

/*ECATCHANGE_START(V5.11) ECAT6*/
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This function is called from the PDI_Isr and is used to calculate the bus cycle time
  *////////////////////////////////////////////////////////////////////////////////////////
void HandleBusCycleCalculation(void)
{
    /*calculate the cycle time if device is in SM Sync mode and Cycle time was not calculated yet*/
    if ( !bDcSyncActive && bEscIntEnabled)
    {
        BOOL bTiggerCalcCycleTime = FALSE;

        if(sSyncManOutPar.u16GetCycleTime == 1)
            bTiggerCalcCycleTime = TRUE;
        if(bTiggerCalcCycleTime)
        {
            /*get bus cycle time triggered */
            sSyncManOutPar.u32CycleTime = 0;
            sSyncManOutPar.u16GetCycleTime = 0;

            sSyncManInPar.u32CycleTime  = 0;
            sSyncManInPar.u16GetCycleTime = 0;

            u16BusCycleCntMs = 0;
            bCycleTimeMeasurementStarted = TRUE;
            StartTimerCnt = (UINT32) HW_GetTimer();
        }
        else
        {
            if(bCycleTimeMeasurementStarted == TRUE)
            {
                UINT32 CurTimerCnt = (UINT32)HW_GetTimer();
/*ECATCHANGE_START(V5.11) ECAT3*/
                UINT32 CalcCycleTime = 0;


#if ECAT_TIMER_INC_P_MS
                CalcCycleTime = (UINT32)u16BusCycleCntMs * 1000000 + (((INT32)(CurTimerCnt-StartTimerCnt))*1000000/ECAT_TIMER_INC_P_MS);    //get elapsed cycle time in ns
#endif

/*ECATCHANGE_START(V5.11) ECAT4*/
                sSyncManOutPar.u32CycleTime = CalcCycleTime;
/*ECATCHANGE_END(V5.11) ECAT4*/
                sSyncManInPar.u32CycleTime  = CalcCycleTime;
                u16BusCycleCntMs = 0;
                StartTimerCnt = 0;
                bCycleTimeMeasurementStarted = FALSE;

/*ECATCHANGE_END(V5.11) ECAT3*/
            /* CiA402 Motion controller cycle time is only set if DC Synchronisation is active*/
            }
        }
    }
}
/*ECATCHANGE_END(V5.11) ECAT6*/

void PDI_Isr(void)
{
    if(bEscIntEnabled)    //Synchron-Mode  DC-Mode:
    {
        /* get the AL event register */
        UINT16  ALEvent = HW_GetALEventRegister_Isr();   //AL Event Request 0x220
        ALEvent = SWAPWORD(ALEvent);

        if ( ALEvent & PROCESS_OUTPUT_EVENT )  //SM2中断
        {
            if(bDcRunning && bDcSyncActive)
            {
                /* Reset SM/Sync0 counter. Will be incremented on every Sync0 event*/
                u16SmSync0Counter = 0;
            }
            if(sSyncManOutPar.u16SmEventMissedCounter > 0)
                sSyncManOutPar.u16SmEventMissedCounter--;


/*ECATCHANGE_START(V5.11) ECAT6*/
            //calculate the bus cycle time if required
            HandleBusCycleCalculation();
/*ECATCHANGE_END(V5.11) ECAT6*/

            /* Outputs were updated, set flag for watchdog monitoring */
            bEcatFirstOutputsReceived = TRUE;
            /*
                handle output process data event
            */
            if ( bEcatOutputUpdateRunning )   //进入OP态
            {
                /* slave is in OP, update the outputs */
                PDO_OutputMapping();
            }
            else
            {
                /* Just acknowledge the process data event in the INIT,PreOP and SafeOP state */
                HW_EscReadWordIsr(u16dummy,nEscAddrOutputData);
                HW_EscReadWordIsr(u16dummy,(nEscAddrOutputData+nPdOutputSize-2));
            }
        }

/*ECATCHANGE_START(V5.11) ECAT4*/
        if (( ALEvent & PROCESS_INPUT_EVENT ) && (nPdOutputSize == 0))
        {
            //calculate the bus cycle time if required
            HandleBusCycleCalculation();
        }
/*ECATCHANGE_END(V5.11) ECAT4*/

        /*
            Call ECAT_Application() in SM Sync mode
        */
        if (sSyncManOutPar.u16SyncType == SYNCTYPE_SM_SYNCHRON)
        {
            /* The Application is synchronized to process data Sync Manager event*/
            ECAT_Application();
        }

    if ( bEcatInputUpdateRunning
/*ECATCHANGE_START(V5.11) ESM7*/
       && ((sSyncManInPar.u16SyncType == SYNCTYPE_SM_SYNCHRON) || (sSyncManInPar.u16SyncType == SYNCTYPE_SM2_SYNCHRON))
/*ECATCHANGE_END(V5.11) ESM7*/
        )
    {
        /* EtherCAT slave is at least in SAFE-OPERATIONAL, update inputs */
        PDO_InputMapping();
    }

    /*
      Check if cycle exceed
    */
    /*if next SM event was triggered during runtime increment cycle exceed counter*/
    ALEvent = HW_GetALEventRegister_Isr();
    ALEvent = SWAPWORD(ALEvent);

    if ( ALEvent & PROCESS_OUTPUT_EVENT )
    {
        sSyncManOutPar.u16CycleExceededCounter++;
        sSyncManInPar.u16CycleExceededCounter = sSyncManOutPar.u16CycleExceededCounter;

      /* Acknowledge the process data event*/
            HW_EscReadWordIsr(u16dummy,nEscAddrOutputData);
            HW_EscReadWordIsr(u16dummy,(nEscAddrOutputData+nPdOutputSize-2));
    }
    } //if(bEscIntEnabled)
}

void Sync0_Isr(void)
{
   // stTaskStat[4].lStartTim  = stTaskStat[4].lEndTim;
   // stTaskStat[4].lEndTim   = SysTick->VAL;
    // 测执行时间
	//if(stTaskStat[4].lStartTim >= stTaskStat[4].lEndTim)
	//{
	//	stTaskStat[4].lDeltaTim = stTaskStat[4].lStartTim - stTaskStat[4].lEndTim;
	//}
	//else
	//{
	//	stTaskStat[4].lDeltaTim = (1 << 24) - stTaskStat[4].lEndTim + stTaskStat[4].lStartTim;
	//}
  //  stTaskStat[4].fRatio     = (float32)stTaskStat[4].lDeltaTim * 100.0f / 8400.0f;
  //  stTaskStat[4].fRatioMax  = (stTaskStat[4].fRatioMax > stTaskStat[4].fRatio) ? stTaskStat[4].fRatioMax : stTaskStat[4].fRatio;

    Sync0WdCounter = 0;  //LED_9=0 ;
    if(bDcSyncActive)
    {
        if ( bEcatInputUpdateRunning )
        {
            LatchInputSync0Counter++;
        }

        /*ECATCHANGE_START(V5.11) ECAT4*/
        if(u16SmSync0Value > 0)
        {
           /* Check if Sm-Sync sequence is invalid */
           if (u16SmSync0Counter > u16SmSync0Value)
           {
              /*ECATCHANGE_START(V5.11) COE3*/
              if ((nPdOutputSize > 0) && (sSyncManOutPar.u16SmEventMissedCounter <= sErrorSettings.u16SyncErrorCounterLimit))
              {
                 /*ECATCHANGE_END(V5.11) COE3*/
                 sSyncManOutPar.u16SmEventMissedCounter = sSyncManOutPar.u16SmEventMissedCounter + 3;
              }

           /*ECATCHANGE_START(V5.11) COE3*/
           if ((nPdInputSize > 0) && (nPdOutputSize == 0) && (sSyncManInPar.u16SmEventMissedCounter <= sErrorSettings.u16SyncErrorCounterLimit))
           {
           /*ECATCHANGE_END(V5.11) COE3*/
               sSyncManInPar.u16SmEventMissedCounter = sSyncManInPar.u16SmEventMissedCounter + 3;
           }

           } // if (u16SmSync0Counter > u16SmSync0Value)

           if ((nPdOutputSize == 0) && (nPdInputSize > 0))
           {
              /* Input only with DC, check if the last input data was read*/
              UINT16  ALEvent = HW_GetALEventRegister_Isr();
              ALEvent = SWAPWORD(ALEvent);

              if ((ALEvent & PROCESS_INPUT_EVENT) == 0)
              {
                 /* no input data was read by the master, increment the sm missed counter*/
                 u16SmSync0Counter++;
              }
              else
              {
                 /* Reset SM/Sync0 counter*/
                 u16SmSync0Counter = 0;
                 sSyncManInPar.u16SmEventMissedCounter = 0;
              }
           }
           else
           {
              u16SmSync0Counter++;
           }
        }//SM -Sync monitoring enabled
/*ECATCHANGE_END(V5.11) ECAT4*/

        if(!bEscIntEnabled && bEcatOutputUpdateRunning)
        {
            /* Output mapping was not done by the PDI ISR */
            PDO_OutputMapping();
        }

        /* Application is synchronized to SYNC0 event*/
        ECAT_Application();

        if ( bEcatInputUpdateRunning
           && (LatchInputSync0Value > 0) && (LatchInputSync0Value == LatchInputSync0Counter) ) /* Inputs shall be latched on a specific Sync0 event */
        {
            /* EtherCAT slave is at least in SAFE-OPERATIONAL, update inputs */
            PDO_InputMapping();

            if(LatchInputSync0Value == 1)
            {
                /* if inputs are latched on every Sync0 event (otherwise the counter is reset on the next Sync1 event) */
                LatchInputSync0Counter = 0;
            }
        }

    }
}

void Sync1_Isr(void)
{
    Sync1WdCounter = 0;
    if ( bEcatInputUpdateRunning
        && (sSyncManInPar.u16SyncType == SYNCTYPE_DCSYNC1)
        && (LatchInputSync0Value == 0)) /* Inputs are latched on Sync1 (LatchInputSync0Value == 0), if LatchInputSync0Value > 0 inputs are latched with Sync0 */
    {
        /* EtherCAT slave is at least in SAFE-OPERATIONAL, update inputs */
        PDO_InputMapping();
    }

    /* Reset Sync0 latch counter (to start next Sync0 latch cycle) */
    LatchInputSync0Counter = 0;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This function shall called within a 1ms cycle.
        Set Run and Error Led depending on the Led state

*////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     pObjectDictionary   Pointer to application specific object dictionary.
                                NULL if no specific object are available.
\return     0 if initialization was successful

 \brief    This function initialize the EtherCAT Sample Code

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 MainInit(void)
{
    UINT16 Error = 0;
/*Hardware init function need to be called from the application layer*/

/*ECATCHANGE_START(V5.11) EEPROM1*/
#ifdef SET_EEPROM_PTR
    SET_EEPROM_PTR
#endif
/*ECATCHANGE_END(V5.11) EEPROM1*/

    /* initialize the EtherCAT Slave Interface */
    ECAT_Init();
    /* initialize the objects */
    COE_ObjInit();


    /*Timer initialization*/
    u16BusCycleCntMs = 0;
    StartTimerCnt = 0;
    bCycleTimeMeasurementStarted = FALSE;

    /*indicate that the slave stack initialization finished*/
    bInitFinished = TRUE;

/*Application Init need to be called from the application layer*/
     return Error;
}


/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This function shall be called cyclically from main

*////////////////////////////////////////////////////////////////////////////////////////
UINT16 TestIntEn;
void MainLoop(void)
{
    /*return if initialization not finished */
    if(bInitFinished == FALSE)
        return;

        /* FreeRun-Mode:  bEscIntEnabled = FALSE, bDcSyncActive = FALSE
           Synchron-Mode: bEscIntEnabled = TRUE, bDcSyncActive = FALSE
           DC-Mode:       bEscIntEnabled = TRUE, bDcSyncActive = TRUE */
        if (
            (!bEscIntEnabled || !bEcatFirstOutputsReceived)     /* SM-Synchronous, but not SM-event received */
          && !bDcSyncActive                                               /* DC-Synchronous */
            )
        {
            /* if the application is running in ECAT Synchron Mode the function ECAT_Application is called
               from the ESC interrupt routine (in mcihw.c or spihw.c),
               in ECAT Synchron Mode it should be additionally checked, if the SM-event is received
               at least once (bEcatFirstOutputsReceived = 1), otherwise no interrupt is generated
               and the function ECAT_Application has to be called here (with interrupts disabled,
               because the SM-event could be generated while executing ECAT_Application) */
            if ( !bEscIntEnabled )
            {
                /* application is running in ECAT FreeRun Mode,
                   first we have to check, if outputs were received */
                UINT16 ALEvent = HW_GetALEventRegister();
                ALEvent = SWAPWORD(ALEvent);

                if ( ALEvent & PROCESS_OUTPUT_EVENT )
                {
                    /* set the flag for the state machine behaviour */
                    bEcatFirstOutputsReceived = TRUE;
                    if ( bEcatOutputUpdateRunning )
                    {
                        /* update the outputs */
                        PDO_OutputMapping();
                    }
                }
                else if ( nPdOutputSize == 0 )
                {
                    /* if no outputs are transmitted, the watchdog must be reset, when the inputs were read */
                    if ( ALEvent & PROCESS_INPUT_EVENT )
                    {
                        /* Outputs were updated, set flag for watchdog monitoring */
                        bEcatFirstOutputsReceived = TRUE;
                    }
                }
            }

            DISABLE_ESC_INT();
            TestIntEn = 1;
            ECAT_Application();

            if ( bEcatInputUpdateRunning )
            {
                /* EtherCAT slave is at least in SAFE-OPERATIONAL, update inputs */
                PDO_InputMapping();
            }
            ENABLE_ESC_INT();
        }
        else
        {
            TestIntEn = 0;
        }
	#if !ECAT_TIMER_INT
        /* there is no interrupt routine for the hardware timer so check the timer register if the desired cycle elapsed*/
        {
            UINT32 CurTimer = (UINT32)HW_GetTimer();

            if(CurTimer >= ECAT_TIMER_INC_P_MS)
            {
                ECAT_CheckTimer();
                HW_ClearTimer();
            }
        }
#endif
     /* call EtherCAT functions */
    ECAT_Main();
    if(bEcatInputUpdateRunning)  //EtherCAT slave is at least in SAFE-OPERATIONAL,
        CiA402_StateMachine();

     /* call lower prior application part */
    COE_Main();
    CheckIfEcatError();
}

/*The main function was moved to the application files.*/
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    ECAT_Application (prev. SSC versions "COE_Application")
 this function calculates and the physical process signals and triggers the input mapping
*////////////////////////////////////////////////////////////////////////////////////////
void ECAT_Application(void)
{
    {
        APPL_Application();
    }
/* PDO Input mapping is called from the specific trigger ISR */
}




/** @} */

