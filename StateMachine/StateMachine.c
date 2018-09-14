/*******************************************************************************
*
* State machine.
*
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "StateMachine.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Local variables
******************************************************************************/

/******************************************************************************
* Local functions
******************************************************************************/


static void SM_StateFault(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateInit(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateStop(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateRun(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateAlign(SM_APP_CTRL_T *psAppCtrl);



const PFCN_VOID_PSM gSM_STATE_TABLE[5] = {SM_StateFault, SM_StateInit, SM_StateStop, SM_StateRun, SM_StateAlign};


void SM_StateMachine(SM_APP_CTRL_T *sAppCtrl)
{
	gSM_STATE_TABLE[sAppCtrl -> eState](sAppCtrl);
}


static void SM_StateFault(SM_APP_CTRL_T *psAppCtrl)
{

    psAppCtrl -> psState -> Fault();

    if ((psAppCtrl -> uiCtrl & SM_CTRL_FAULT_CLEAR) > 0 )
    {
				psAppCtrl -> uiCtrl &= ~(SM_CTRL_INIT_DONE | SM_CTRL_FAULT | SM_CTRL_FAULT_CLEAR);

        psAppCtrl -> psTrans -> FaultInit();

				psAppCtrl -> eState = INIT;
    }
}


static void SM_StateInit(SM_APP_CTRL_T *psAppCtrl)
{
    psAppCtrl -> psState -> Init();

		if ((psAppCtrl -> uiCtrl & SM_CTRL_FAULT) > 0)
		{
				psAppCtrl -> psTrans -> InitFault();

				psAppCtrl -> eState = FAULT;
		}
		else if ((psAppCtrl -> uiCtrl & SM_CTRL_INIT_DONE) > 0)
		{
				psAppCtrl -> uiCtrl &= ~(SM_CTRL_INIT_DONE | SM_CTRL_STOP | SM_CTRL_START | SM_CTRL_STOP_ACK | SM_CTRL_RUN_ACK);

				psAppCtrl -> psTrans -> InitStop();

				psAppCtrl -> eState = STOP;
		}
}


static void SM_StateStop(SM_APP_CTRL_T *psAppCtrl)
{
		psAppCtrl -> psState -> Stop();

		if ((psAppCtrl -> uiCtrl & SM_CTRL_FAULT) > 0)
		{
			psAppCtrl -> psTrans -> StopFault();

			psAppCtrl -> eState = FAULT;	
		}
		
		else if ((psAppCtrl -> uiCtrl & SM_CTRL_START) > 0)
		{
			psAppCtrl -> psTrans -> StopRun();

			if ((psAppCtrl -> uiCtrl & SM_CTRL_RUN_ACK) > 0)
			{
				psAppCtrl -> uiCtrl &= ~(SM_CTRL_RUN_ACK | SM_CTRL_START);

				psAppCtrl -> eState = RUN;	
			}
		}
		
		else if ((psAppCtrl -> uiCtrl & SM_CTRL_ALIGN) > 0)
		{
			psAppCtrl -> psTrans -> StopAlign();
			
			psAppCtrl -> uiCtrl &= ~(SM_CTRL_ALIGN);
				
			psAppCtrl -> eState = ALIGN;	
		}
}

static void SM_StateRun(SM_APP_CTRL_T *psAppCtrl)
{
    psAppCtrl -> psState -> Run();

		if ((psAppCtrl -> uiCtrl & SM_CTRL_FAULT) > 0)
		{
			psAppCtrl -> psTrans -> RunFault();

			psAppCtrl -> eState = FAULT;	
		}
		else if ((psAppCtrl -> uiCtrl & SM_CTRL_STOP) > 0)
		{
			psAppCtrl -> psTrans -> RunStop();

			if ((psAppCtrl -> uiCtrl & SM_CTRL_STOP_ACK) > 0)
			{
				psAppCtrl -> uiCtrl &= ~(SM_CTRL_STOP_ACK | SM_CTRL_STOP);

				psAppCtrl -> eState = STOP;	
			}
		}
}


static void SM_StateAlign(SM_APP_CTRL_T *psAppCtrl)
{
		psAppCtrl -> psState -> Align();

		if ((psAppCtrl -> uiCtrl & SM_CTRL_FAULT) > 0)
		{
			psAppCtrl -> psTrans -> AlignFault();

			psAppCtrl -> eState = FAULT;	
		}
		else if ((psAppCtrl -> uiCtrl & SM_CTRL_ALIGN_DONE) > 0)
		{
			psAppCtrl -> uiCtrl &= ~(SM_CTRL_ALIGN_DONE | SM_CTRL_STOP | SM_CTRL_START | SM_CTRL_STOP_ACK | SM_CTRL_RUN_ACK);

			psAppCtrl -> psTrans -> AlignStop();

			psAppCtrl -> eState = STOP;
		}
}

/******************************************************************************
* Inline functions
******************************************************************************/
