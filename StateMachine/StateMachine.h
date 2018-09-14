/*******************************************************************************
*
* State machine.
*
******************************************************************************/

#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

/******************************************************************************
* Includes
******************************************************************************/


/******************************************************************************
* Constants
******************************************************************************/

typedef enum {
    FAULT           = 0,
    INIT            = 1,
    STOP           	= 2,
		RUN							= 3,
		ALIGN						= 4,
} SM_APP_STATE_T;         



typedef enum {
    READY           = 0,
		ROTATION				= 1,
} SM_RUN_SUBSTATE_T;         

typedef unsigned short SM_APP_CTRL;
typedef unsigned long SM_APP_FAULT;

typedef void (*PFCN_VOID_VOID)(void); 


typedef struct
{
	PFCN_VOID_VOID	Fault;
	PFCN_VOID_VOID	Init;
	PFCN_VOID_VOID	Stop;
	PFCN_VOID_VOID	Run;
	PFCN_VOID_VOID	Align;
} SM_APP_STATE_FCN_T;


typedef struct
{
	PFCN_VOID_VOID	FaultInit;
	PFCN_VOID_VOID	InitFault;
	PFCN_VOID_VOID	InitStop;
	PFCN_VOID_VOID	StopFault;
	PFCN_VOID_VOID	StopInit;
	PFCN_VOID_VOID	StopRun;
	PFCN_VOID_VOID	RunFault;
	PFCN_VOID_VOID	RunStop;
	PFCN_VOID_VOID	AlignStop;
	PFCN_VOID_VOID	StopAlign;
	PFCN_VOID_VOID	AlignFault;
} SM_APP_TRANS_FCN_T;


typedef struct
{
	SM_APP_STATE_FCN_T const*		psState;			
	SM_APP_TRANS_FCN_T const* 	psTrans; 		
  
	SM_APP_CTRL						uiCtrl;				
	SM_APP_STATE_T				eState;				
} SM_APP_CTRL_T;


typedef void (*PFCN_VOID_PSM)(SM_APP_CTRL_T *sAppCtrl); 


#define SM_CTRL_NONE								0x0
#define SM_CTRL_START								0x1
#define SM_CTRL_STOP								0x2
#define SM_CTRL_FAULT_CLEAR 				0x4
#define SM_CTRL_ALIGN								0x8
#define SM_CTRL_FAULT								0x10
#define SM_CTRL_STOP_ACK						0x20
#define SM_CTRL_RUN_ACK							0x40
#define SM_CTRL_INIT_DONE						0x80
#define SM_CTRL_ALIGN_DONE					0x100

void SM_StateMachine(SM_APP_CTRL_T *sAppCtrl);

#endif //_STATE_MACHINE_H_
