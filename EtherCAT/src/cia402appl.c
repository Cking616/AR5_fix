/**
\addtogroup CiA402appl CiA402 Sample Application
@{
*/

/**
\file cia402appl.c
\author EthercatSSC@beckhoff.com
\brief Implementation
This file contains all ciA402 specific functions

\version 5.11

<br>Changes to version V5.10:<br>
V5.11 ECAT11: create application interface function pointer, add eeprom emulation interface functions<br>
<br>Changes to version V5.01:<br>
V5.10 CIA402 1: Update complete access handling for 0xF030<br>
V5.10 ECAT6: Add "USE_DEFAULT_MAIN" to enable or disable the main function<br>
<br>Changes to version V5.0:<br>
V5.01 ESC2: Add missed value swapping<br>
<br>Changes to version V4.40:<br>
V5.0 CIA402 1: Syntax bugfix in dummy motion controller<br>
V5.0 CIA402 2: Handle 0xF030/0xF050 in correlation do PDO assign/mapping objects<br>
V5.0 CIA402 3: Trigger dummy motion controller if valid mode of operation is set.<br>
V5.0 CIA402 4: Change Axes structure handling and resources allocation.<br>
V5.0 ECAT2: Create generic application interface functions. Documentation in Application Note ET9300.<br>
<br>Changes to version V4.30:<br>
V4.40 CoE 6: add AL Status code to Init functions<br>
V4.40 CIA402 2: set motion control trigger depending on "Synchronisation", "mode of operation" and "cycle time"<br>
V4.40 CIA402 1: change behaviour and name of bit12 of the status word (0x6041) (WG CIA402 24.02.2010)<br>
V4.30 : create file (state machine; handling state transition options; input feedback)
*/

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/


#include "ecat_def.h"

#include "include_c.h"


/* ECATCHANGE_START(V5.11) ECAT11*/
#include "applInterface.h"
/* ECATCHANGE_END(V5.11) ECAT11*/

#include "coeappl.h"

#define _CiA402_
#include "cia402appl.h"
#undef _CiA402_

#if STM32F407_HW
#include "el9800hw.h"
#endif

struct SERVO_CONTROL_WORDS_Bit{
	u16 Switch_On         :1;
	u16 Enable_Voltage    :1;
	u16 Quick_Stop        :1;
	u16 Operation_mode    :3;
	u16 Fault_Reset       :1;
	u16 Halt              :1;
	u16  Reserve           :1;
	u16  Manufacturer_Specific :5;
};
union SERVO_CONTROL_WORDS{
	u16 all;
	struct SERVO_CONTROL_WORDS_Bit bit;
}Servo_control;


struct  SERVO_STATE_WORDS_Bit{
	u16 Ready_Switch     :1;
	u16 Switch_On         :1;
	u16  Serve_En_ok      :1;
	u16 Fault              :1;
  u16  Voltage_Enabled   :1;
  u16  Quick_Stop        :1;
	u16  Switch_Disabled  :1;
	u16  Warning         :1;
  u16   Reserved8      :1;
  u16   Remote          :1;
  u16    Target_Limit  :1;
  u16    Internal_Limit  :1;
	u16      Target_Value    :1;
	u16       Reserved13       :1;
	u16     Reserved14_15     :1;
};
union SERVO_STATE_WORDS{
	u16 all;
	struct SERVO_STATE_WORDS_Bit bit;
}Servo_State;
/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/
TCiA402Axis       LocalAxes[MAX_AXES];
/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    0               Init CiA402 device successful
            ALSTATUSCODE_XX Init CiA402 device failed

 \brief    This function initializes the Axes structures
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 CiA402_Init(void)
{
    UINT16 result = 0;
    UINT16 AxisCnt = 0;
    UINT16 j = 0;
    UINT32 ObjectOffset = 0x800;
    UINT8 PDOOffset = 0x10;


    for(AxisCnt = 0; AxisCnt < MAX_AXES ; AxisCnt++)
    {
        /*Reset Axis buffer*/
        HMEMSET(&LocalAxes[AxisCnt],0,SIZEOF(TCiA402Axis));

        LocalAxes[AxisCnt].bAxisIsActive = FALSE;
        LocalAxes[AxisCnt].bBrakeApplied = TRUE;
        LocalAxes[AxisCnt].bLowLevelPowerApplied = TRUE;
        LocalAxes[AxisCnt].bHighLevelPowerApplied = FALSE;
        LocalAxes[AxisCnt].bAxisFunctionEnabled = FALSE;
        LocalAxes[AxisCnt].bConfigurationAllowed = TRUE;

        LocalAxes[AxisCnt].i16State = STATE_NOT_READY_TO_SWITCH_ON;
        LocalAxes[AxisCnt].u16PendingOptionCode = 0x00;

        LocalAxes[AxisCnt].fCurPosition = 0;
        LocalAxes[AxisCnt].u32CycleTime = 0;


        /***********************************
            init objects
        *************************************/

        /*set default values*/
        HMEMCPY(&LocalAxes[AxisCnt].Objects,&DefCiA402ObjectValues,CIA402_OBJECTS_SIZE);

        /***set Object offset to PDO entries***/

        /*csv/csp RxPDO*/
        for(j =0; j < LocalAxes[AxisCnt].Objects.sRxPDOMap0.u16SubIndex0;j++)
            LocalAxes[AxisCnt].Objects.sRxPDOMap0.aEntries[j] += AxisCnt* (ObjectOffset<<16);

        /*csp RxPDO*/
        for(j =0; j < LocalAxes[AxisCnt].Objects.sRxPDOMap1.u16SubIndex0;j++)
            LocalAxes[AxisCnt].Objects.sRxPDOMap1.aEntries[j] += AxisCnt* (ObjectOffset<<16);

        /*csv RxPDO*/
        for(j =0; j < LocalAxes[AxisCnt].Objects.sRxPDOMap2.u16SubIndex0;j++)
            LocalAxes[AxisCnt].Objects.sRxPDOMap2.aEntries[j] += AxisCnt* (ObjectOffset<<16);

			  for(j =0; j < LocalAxes[AxisCnt].Objects.sRxPDOMap3.u16SubIndex0;j++)
            LocalAxes[AxisCnt].Objects.sRxPDOMap0.aEntries[j] += AxisCnt* (ObjectOffset<<16);


        /*csv/csp TxPDO*/
        for(j =0; j < LocalAxes[AxisCnt].Objects.sTxPDOMap0.u16SubIndex0;j++)
            LocalAxes[AxisCnt].Objects.sTxPDOMap0.aEntries[j] += AxisCnt* (ObjectOffset<<16);

        /*csp TxPDO*/
        for(j =0; j < LocalAxes[AxisCnt].Objects.sTxPDOMap1.u16SubIndex0;j++)
            LocalAxes[AxisCnt].Objects.sTxPDOMap1.aEntries[j] += AxisCnt* (ObjectOffset<<16);

        /*csv TxPDO*/
        for(j =0; j < LocalAxes[AxisCnt].Objects.sTxPDOMap2.u16SubIndex0;j++)
            LocalAxes[AxisCnt].Objects.sTxPDOMap2.aEntries[j] += AxisCnt* (ObjectOffset<<16);
         /*csv/csp TxPDO*/
        for(j =0; j < LocalAxes[AxisCnt].Objects.sTxPDOMap3.u16SubIndex0;j++)
            LocalAxes[AxisCnt].Objects.sTxPDOMap0.aEntries[j] += AxisCnt* (ObjectOffset<<16);


        /***********************************
            init objects dictionary entries
        *************************************/
        LocalAxes[AxisCnt].ObjDic = (TOBJECT *) ALLOCMEM(SIZEOF(DefCiA402AxisObjDic));
        HMEMCPY(LocalAxes[AxisCnt].ObjDic,&DefCiA402AxisObjDic,SIZEOF(DefCiA402AxisObjDic));
        {
        TOBJECT OBJMEM *pDiCEntry = LocalAxes[AxisCnt].ObjDic;

        /*adapt Object index and assign Var pointer*/
        while(pDiCEntry->Index != 0xFFFF)
        {
            //BOOL bObjectFound = TRUE;

            switch(pDiCEntry->Index)
            {
            case 0x1600:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.sRxPDOMap0;
                break;
            case 0x1601:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.sRxPDOMap1;
                break;
            case 0x1602:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.sRxPDOMap2;
                break;
			case 0x1603:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.sRxPDOMap3;
                break;
            case 0x1A00:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.sTxPDOMap0;
                break;
            case 0x1A01:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.sTxPDOMap1;
                break;
            case 0x1A02:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.sTxPDOMap2;
                break;
			case 0x1A03:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.sTxPDOMap3;
                break;
            case 0x603F:
                //pDiCEntry->pVarPtr = &gsM1_Drive.sFaultId.unCtrlErr.ulWord;//&LocalAxes[AxisCnt].Objects.objErrorCode;        //ECAT_MOD
								pDiCEntry->pVarPtr = &gsM1_Drive.sFaultId.R;
                break;
            case 0x6040:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objControlWord;
                break;
            case 0x6041:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objStatusWord;
                break;
            case 0x605A:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objQuickStopOptionCode;
                break;
            case 0x605B:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objShutdownOptionCode;
                break;
            case 0x605C:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objDisableOperationOptionCode;
                break;
            case 0x605E:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objFaultReactionCode;
                break;
            case 0x6060:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objModesOfOperation;
                break;
            case 0x6061:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objModesOfOperationDisplay;
                break;
            case 0x6064:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objPositionActualValue;
                break;
            case 0x606C:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objVelocityActualValue;
                break;
            case 0x6077:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objTorqueActualValue;
                break;
            case 0x607A:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objTargetPosition;
                break;
            case 0x607D:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objSoftwarePositionLimit;
                break;
            case 0x6085:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objQuickStopDeclaration;
                break;
             case 0x60B2:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objTorqueOffset;
                break;
            case 0x60C2:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objInterpolationTimePeriod;
                break;
            case 0x60FF:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objTargetVelocity;
                break;
            case 0x6502:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objSupportedDriveModes;
                break;
            case 0x20B6:
                pDiCEntry->pVarPtr = &LocalAxes[AxisCnt].Objects.objExternalPositionValue;
                break;
            default :
                //bObjectFound = FALSE;
            break;
            }//switch(pDiCEntry->Index)

            /*increment object index*/
            if(pDiCEntry->Index >= 0x1400 && pDiCEntry->Index <= 0x1BFF)    //PDO region
                pDiCEntry->Index += AxisCnt* PDOOffset;
            else
                pDiCEntry->Index += AxisCnt* (UINT16)ObjectOffset;

            pDiCEntry++;
        }//while(pDiCEntry->Index != 0xFFFF)

        }
    }// for "all active axes"
    return result;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402_DeallocateAxis
 \brief    Remove all allocated axes resources
*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_DeallocateAxis(void)
{
    UINT8 cnt = 0;

    for(cnt = 0 ; cnt < MAX_AXES ; cnt++)
    {
    /*Remove object dictionary entries*/
    if(LocalAxes[cnt].ObjDic != NULL)
    {
        TOBJECT OBJMEM *pEntry = LocalAxes[cnt].ObjDic;

        while(pEntry->Index != 0xFFFF)
        {
            COE_RemoveDicEntry(pEntry->Index);

            pEntry++;
        }

        FREEMEM(LocalAxes[cnt].ObjDic);
    }

    nPdOutputSize = 0;
    nPdInputSize = 0;

    }

}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402-Statemachine
        This function handles the state machine for devices using the CiA402 profile.
        called cyclic from MainLoop()
        All described transition numbers are referring to the document
        "ETG Implementation Guideline for the CiA402 Axis Profile" located on the EtherCAT.org download section

*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_StateMachine(void)    //在main中运行
{
    TCiA402Axis *pCiA402Axis;
    UINT16 StatusWord = 0;
    UINT16 ControlWord6040 = 0;
    UINT16 counter = 0;

    for(counter = 0; counter < MAX_AXES;counter++)
    {
        if(!LocalAxes[counter].bAxisIsActive)
        {
            continue;
        }

        pCiA402Axis = &LocalAxes[counter];
        StatusWord = pCiA402Axis->Objects.objStatusWord;
        ControlWord6040 = pCiA402Axis->Objects.objControlWord;

        /*clear statusword state and controlword processed complete bits*/
        StatusWord &= ~(STATUSWORD_STATE_MASK | STATUSWORD_REMOTE);  //0x6F  0x200

        /*skip state state transition if the previous transition is pending*/
        if(pCiA402Axis->u16PendingOptionCode!= 0x0)
        {
            return;
        }
        /*skip transition 1 and 2*/
        if(pCiA402Axis->i16State < STATE_READY_TO_SWITCH_ON && nAlStatus == STATE_OP)
            pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON;

        switch(pCiA402Axis->i16State)
        {
        case STATE_NOT_READY_TO_SWITCH_ON:
            StatusWord |= (STATUSWORD_STATE_NOTREADYTOSWITCHON);
            if(nAlStatus == STATE_OP)
            {
                // Automatic transition -> Communication shall be activated
                pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 1
            }
            else
            {
                /*
                CiA402 statemachine shall stay in "STATE_NOT_READY_TO_SWITCH_ON" if EtherCAT state is not OP.
                */
                pCiA402Axis->i16State = STATE_NOT_READY_TO_SWITCH_ON; // stay in current state
            }

            break;
        case STATE_SWITCH_ON_DISABLED:
            StatusWord |= (STATUSWORD_STATE_SWITCHEDONDISABLED);
            if ((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 2
            }
            break;
        case STATE_READY_TO_SWITCH_ON:
            StatusWord |= (STATUSWORD_STATE_READYTOSWITCHON);
            if (((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)
                || ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
            {
                pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 7
            } else
                if (((ControlWord6040 & CONTROLWORD_COMMAND_SWITCHON_MASK) == CONTROLWORD_COMMAND_SWITCHON) ||
                    ((ControlWord6040 & CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION))
                {
                    pCiA402Axis->i16State = STATE_SWITCHED_ON;           // Transition 3
                }
                break;
        case STATE_SWITCHED_ON:
            StatusWord |= (STATUSWORD_STATE_SWITCHEDON);

            if ((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
            {
                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 6

            } else
                if (((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP
                    || (ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE))
                {
                    pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 10

                } else
                    if ((ControlWord6040 & CONTROLWORD_COMMAND_ENABLEOPERATION_MASK) == CONTROLWORD_COMMAND_ENABLEOPERATION)
                    {
                        pCiA402Axis->i16State = STATE_OPERATION_ENABLED;  // Transition 4
                        //The Axis function shall be enabled and all internal set-points cleared.
                    }
                    break;
        case STATE_OPERATION_ENABLED:
            StatusWord |= (STATUSWORD_STATE_OPERATIONENABLED);

            if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEOPERATION_MASK) == CONTROLWORD_COMMAND_DISABLEOPERATION)
            {
                if(pCiA402Axis->Objects.objDisableOperationOptionCode!= DISABLE_DRIVE)
                {
                    /*disable operation pending*/
                    pCiA402Axis->u16PendingOptionCode = 0x605C; //STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_SWITCHED_ON);
                    return;
                }
                pCiA402Axis->i16State = STATE_SWITCHED_ON;           // Transition 5
            } else
                if ((ControlWord6040 & CONTROLWORD_COMMAND_QUICKSTOP_MASK) == CONTROLWORD_COMMAND_QUICKSTOP)
                {
                    pCiA402Axis->i16State = STATE_QUICK_STOP_ACTIVE;  // Transition 11
                } else
                    if ((ControlWord6040 & CONTROLWORD_COMMAND_SHUTDOWN_MASK) == CONTROLWORD_COMMAND_SHUTDOWN)
                    {
                        if(pCiA402Axis->Objects.objShutdownOptionCode != DISABLE_DRIVE)
                        {
                            /*shutdown operation required*/
                            pCiA402Axis->u16PendingOptionCode = 0x605B; //STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_READY_TO_SWITCH_ON);
                            return;
                        }

                        pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON; // Transition 8

                    } else
                        if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE)
                        {
                            pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 9
                        }
                        break;
        case STATE_QUICK_STOP_ACTIVE:
            StatusWord |= STATUSWORD_STATE_QUICKSTOPACTIVE;

            if((pCiA402Axis->Objects.objQuickStopOptionCode != DISABLE_DRIVE) &&
                ((pCiA402Axis->Objects.objStatusWord & STATUSWORD_STATE_MASK)!= STATUSWORD_STATE_QUICKSTOPACTIVE))
            {
                /*Only execute quick stop action in state transition 11*/
                pCiA402Axis->u16PendingOptionCode = 0x605A;//STATE_TRANSITION (STATE_OPERATION_ENABLED,STATE_QUICK_STOP_ACTIVE);
                return;
            }

            if ((ControlWord6040 & CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK) == CONTROLWORD_COMMAND_DISABLEVOLTAGE)
            {
                pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED; // Transition 12
            }

            /*NOTE: it is not recommend to support transition 16 */
            break;
        case STATE_FAULT_REACTION_ACTIVE:
            StatusWord |= (STATUSWORD_STATE_FAULTREACTIONACTIVE);
            if(pCiA402Axis->Objects.objFaultReactionCode!= DISABLE_DRIVE)
            {
                /*fault reaction pending*/
                pCiA402Axis->u16PendingOptionCode = 0x605E;
                return;
            }

            // Automatic transition
            pCiA402Axis->i16State = STATE_FAULT;// Transition 14
            break;
        case STATE_FAULT:
            StatusWord |= (STATUSWORD_STATE_FAULT);

            if ((ControlWord6040 & CONTROLWORD_COMMAND_FAULTRESET_MASK) == CONTROLWORD_COMMAND_FAULTRESET)
            {
                pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED;// Transition 15
                gsM1_Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR;         //ECAT_MOD
            }
            break;

        default:    //the sate variable is set to in invalid value => rest Axis
            StatusWord = (STATUSWORD_STATE_NOTREADYTOSWITCHON);
            pCiA402Axis->i16State = STATE_NOT_READY_TO_SWITCH_ON;
            break;

        }// switch(current state)



        /*Update operational functions (the low level power supply is always TRUE*/
        switch(pCiA402Axis->i16State)
        {
        case STATE_NOT_READY_TO_SWITCH_ON:
        case STATE_SWITCH_ON_DISABLED:
        case STATE_READY_TO_SWITCH_ON:
            pCiA402Axis->bBrakeApplied = TRUE;
            pCiA402Axis->bHighLevelPowerApplied =  FALSE;
            pCiA402Axis->bAxisFunctionEnabled = FALSE;
            pCiA402Axis->bConfigurationAllowed = TRUE;
            break;
        case STATE_SWITCHED_ON:
            pCiA402Axis->bBrakeApplied = TRUE;
            pCiA402Axis->bHighLevelPowerApplied =  TRUE;
            pCiA402Axis->bAxisFunctionEnabled = FALSE;
            pCiA402Axis->bConfigurationAllowed = TRUE;
            break;
        case STATE_OPERATION_ENABLED:
        case STATE_QUICK_STOP_ACTIVE:
        case STATE_FAULT_REACTION_ACTIVE:
            pCiA402Axis->bBrakeApplied = FALSE;
            pCiA402Axis->bHighLevelPowerApplied =  TRUE;
            pCiA402Axis->bAxisFunctionEnabled = TRUE;
            pCiA402Axis->bConfigurationAllowed = FALSE;
            break;
        case STATE_FAULT:
            pCiA402Axis->bBrakeApplied = TRUE;
            pCiA402Axis->bHighLevelPowerApplied =  FALSE;
            pCiA402Axis->bAxisFunctionEnabled = FALSE;
            pCiA402Axis->bConfigurationAllowed = TRUE;
            break;
        default:
            pCiA402Axis->bBrakeApplied = TRUE;
            pCiA402Axis->bHighLevelPowerApplied =  FALSE;
            pCiA402Axis->bAxisFunctionEnabled = FALSE;
            pCiA402Axis->bConfigurationAllowed = TRUE;
            break;
        }

        if(    pCiA402Axis->bHighLevelPowerApplied == TRUE)
            StatusWord |= STATUSWORD_VOLTAGE_ENABLED;
        else
            StatusWord &= ~STATUSWORD_VOLTAGE_ENABLED;

        /*state transition finished set controlword complete bit and update status object 0x6041*/
        pCiA402Axis->Objects.objStatusWord = (StatusWord | STATUSWORD_REMOTE);

        if(pCiA402Axis->i16State == STATE_OPERATION_ENABLED)   //ECAT_MOD
        {
            gsM1_Ctrl.uiCtrl = SM_CTRL_START;
        }
        else
        {
            gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP;
         }

    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param ErrorCode

 \brief    CiA402_LocalError
 \brief this function is called if an error was detected
*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_LocalError(UINT16 ErrorCode)
{
    UINT16 counter = 0;
    for(counter = 0; counter < MAX_AXES; counter++)
    {
        if(LocalAxes[counter].bAxisIsActive)
        {
            LocalAxes[counter].i16State = STATE_FAULT_REACTION_ACTIVE;
            LocalAxes[counter].Objects.objErrorCode = ErrorCode;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402_DummyMotionControl
 \brief this functions provides an simple feedback functionality
*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_DummyMotionControl(TCiA402Axis *pCiA402Axis)
{

//	  LED_7=0;
    float IncFactor    = (float)0.0010922 * (float) pCiA402Axis->u32CycleTime;

    INT32 i32TargetVelocity = 0;

    /*Motion Controller shall only be triggered if application is trigger by DC Sync Signals,
    and a valid mode of operation is set*/

    /*calculate actual position */
    pCiA402Axis->fCurPosition += ((double)pCiA402Axis->Objects.objVelocityActualValue) * IncFactor;
    pCiA402Axis->Objects.objPositionActualValue = (INT32)(pCiA402Axis->fCurPosition);


    if(pCiA402Axis->bAxisFunctionEnabled &&
    pCiA402Axis->bLowLevelPowerApplied &&
    pCiA402Axis->bHighLevelPowerApplied &&
    !pCiA402Axis->bBrakeApplied)
    {
        if((pCiA402Axis->Objects.objSoftwarePositionLimit.i32MaxLimit> pCiA402Axis->Objects.objPositionActualValue
            || pCiA402Axis->Objects.objPositionActualValue > pCiA402Axis->Objects.objTargetPosition) &&
            (pCiA402Axis->Objects.objSoftwarePositionLimit.i32MinLimit < pCiA402Axis->Objects.objPositionActualValue
            || pCiA402Axis->Objects.objPositionActualValue < pCiA402Axis->Objects.objTargetPosition))
        {
            pCiA402Axis->Objects.objStatusWord &= ~STATUSWORD_INTERNAL_LIMIT;

            switch(pCiA402Axis->Objects.objModesOfOperationDisplay)
            {
            case CYCLIC_SYNC_POSITION_MODE:
                if(IncFactor != 0)
                    i32TargetVelocity = (pCiA402Axis->Objects.objTargetPosition - pCiA402Axis->Objects.objPositionActualValue) / ((long)IncFactor);
                else
                    i32TargetVelocity = 0;
                break;
            case CYCLIC_SYNC_VELOCITY_MODE:
                if(pCiA402Axis->i16State == STATE_OPERATION_ENABLED)
                    i32TargetVelocity = pCiA402Axis->Objects.objTargetVelocity;
                else
                        i32TargetVelocity = 0;
                break;
            default:
                break;
            }
        }
        else
        {
            pCiA402Axis->Objects.objStatusWord |= STATUSWORD_INTERNAL_LIMIT;
        }
    }
    pCiA402Axis->Objects.objVelocityActualValue= i32TargetVelocity;

    /*Accept new mode of operation*/
    pCiA402Axis->Objects.objModesOfOperationDisplay = pCiA402Axis->Objects.objModesOfOperation;
//     LED_7=1;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return TRUE if moving on predefined ramp is finished

 \brief    CiA402-TransitionAction
 \brief this function shall calculate the desired Axis input values to move on a predefined ramp
 \brief if the ramp is finished return TRUE
*////////////////////////////////////////////////////////////////////////////////////////
BOOL CiA402_TransitionAction(INT16 Characteristic,TCiA402Axis *pCiA402Axis)
{
   switch(Characteristic)
   {

   case SLOW_DOWN_RAMP:
#if _WIN32
      #pragma message ("Warning: Implement slowdown ramp")
#else
      //#warning "Implement slowdown ramp"
#endif
      return TRUE;
      //break;
   case QUICKSTOP_RAMP:
#if _WIN32
      #pragma message ("Warning: Implement quick stop ramp ramp")
#else
      //#warning "Implement quick stop ramp ramp"
#endif
      return TRUE;
      //break;
   case STOP_ON_CURRENT_LIMIT:
#if _WIN32
      #pragma message ("Warning: Implement slowdown on current limit ramp")
#else
      //#warning "Implement slowdown on current limit ramp"
#endif
      return TRUE;
      //break;
   case STOP_ON_VOLTAGE_LIMIT:
#if _WIN32
      #pragma message ("Warning: Implement slowdown on voltage limit ramp")
#else
      //#warning "Implement slowdown on voltage limit ramp"
#endif
      return TRUE;
      //break;
   default:
      break;
   }
    return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    CiA402-Application
 \brief check if a state transition is pending and pass desired ramp-code to CiA402TransitionAction()
 \brief if this functions returns true the state transition is finished.
*////////////////////////////////////////////////////////////////////////////////////////
void CiA402_Application(TCiA402Axis *pCiA402Axis)
{


    /*clear "Drive follows the command value" flag if the target values from the master overwritten by the local application*/
    if(pCiA402Axis->u16PendingOptionCode != 0 &&
        (pCiA402Axis->Objects.objModesOfOperationDisplay == CYCLIC_SYNC_POSITION_MODE ||
        pCiA402Axis->Objects.objModesOfOperationDisplay == CYCLIC_SYNC_VELOCITY_MODE))
    {
        pCiA402Axis->Objects.objStatusWord &= ~ STATUSWORD_DRIVE_FOLLOWS_COMMAND;
    }
    else
        pCiA402Axis->Objects.objStatusWord |= STATUSWORD_DRIVE_FOLLOWS_COMMAND;


    switch(pCiA402Axis->u16PendingOptionCode)
    {
    case 0x605A:
        /*state transition 11 is pending analyse shutdown option code (0x605A)*/
        {
            UINT16 ramp = pCiA402Axis->Objects.objQuickStopOptionCode;
            /*masked and execute specified quick stop ramp characteristic */
            if(pCiA402Axis->Objects.objQuickStopOptionCode > 4 && pCiA402Axis->Objects.objQuickStopOptionCode <9)
            {
                if(pCiA402Axis->Objects.objQuickStopOptionCode == 5)
                    ramp = 1;
                if(pCiA402Axis->Objects.objQuickStopOptionCode == 6)
                    ramp = 2;
                if(pCiA402Axis->Objects.objQuickStopOptionCode == 7)
                    ramp = 3;
                if(pCiA402Axis->Objects.objQuickStopOptionCode == 8)
                    ramp = 4;
            }

            if(CiA402_TransitionAction(ramp,pCiA402Axis))
            {
                /*quick stop ramp is finished complete state transition*/
                pCiA402Axis->u16PendingOptionCode = 0x0;
                if(pCiA402Axis->Objects.objQuickStopOptionCode > 0 && pCiA402Axis->Objects.objQuickStopOptionCode < 5)
                {
                    pCiA402Axis->i16State = STATE_SWITCH_ON_DISABLED;    //continue state transition 12
                }
                else if(pCiA402Axis->Objects.objQuickStopOptionCode > 4 && pCiA402Axis->Objects.objQuickStopOptionCode < 9)
                    pCiA402Axis->Objects.objStatusWord |= STATUSWORD_TARGET_REACHED;
            }
        }
        break;
    case 0x605B:
        /*state transition 8 is pending analyse shutdown option code (0x605B)*/
        {
            if(CiA402_TransitionAction(pCiA402Axis->Objects.objShutdownOptionCode,pCiA402Axis))
            {
                /*shutdown ramp is finished complete state transition*/
                pCiA402Axis->u16PendingOptionCode = 0x0;
                pCiA402Axis->i16State = STATE_READY_TO_SWITCH_ON;    //continue state transition 8
            }
        }
        break;
    case 0x605C:
        /*state transition 5 is pending analyse Disable operation option code (0x605C)*/
        {
            if(CiA402_TransitionAction(pCiA402Axis->Objects.objDisableOperationOptionCode,pCiA402Axis))
            {
                /*disable operation ramp is finished complete state transition*/
                pCiA402Axis->u16PendingOptionCode = 0x0;
                pCiA402Axis->i16State = STATE_SWITCHED_ON;    //continue state transition 5
            }
        }
        break;

    case 0x605E:
        /*state transition 14 is pending analyse Fault reaction option code (0x605E)*/
        {
            if(CiA402_TransitionAction(pCiA402Axis->Objects.objFaultReactionCode,pCiA402Axis))
            {
                /*fault reaction ramp is finished complete state transition*/
                pCiA402Axis->u16PendingOptionCode = 0x0;
                pCiA402Axis->i16State = STATE_FAULT;    //continue state transition 14
            }
        }
        break;
    default:
        //pending transition code is invalid => values from the master are used
        pCiA402Axis->Objects.objStatusWord |= STATUSWORD_DRIVE_FOLLOWS_COMMAND;
        break;
    }


		if(bDcSyncActive && (pCiA402Axis->u32CycleTime != 0)
      && ((pCiA402Axis->Objects.objSupportedDriveModes>>(pCiA402Axis->Objects.objModesOfOperation-1))&0x01)) //Mode of Operation (0x6060) - 1 specifies the Bit within Supported Drive Modes (0x6502)
    {
		//CiA402_DummyMotionControl(pCiA402Axis);  //ECAT_MOD
    }


}


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     index                 index of the requested object.
 \param     subindex                subindex of the requested object.
 \param     dataSize                received data size of the SDO Download
 \param     pObjEntry            handle to the dictionary object returned by
                                     OBJ_GetObjectHandle which was called before
 \param    pData                    Pointer to the buffer where the written data can be copied from
 \param    bCompleteAccess    Indicates if a complete write of all subindices of the
                                     object shall be done or not

 \return    result of the write operation (0 (success) or an abort code (ABORTIDX_.... defined in
            sdosrv.h))

 \brief    This function writes "Configured Modules" Object 0xF030
*////////////////////////////////////////////////////////////////////////////////////////

UINT8 Write0xF030( UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess )
{

    UINT16 i = subindex;
    UINT16 maxSubindex = sConfiguredModuleIdentList.u16SubIndex0;
    OBJCONST TSDOINFOENTRYDESC OBJMEM *pEntry;
    /* lastSubindex is used for complete access to make loop over the requested entries
       to be read, we initialize this variable with the requested subindex that only
       one loop will be done for a single access */
    UINT8 lastSubindex = subindex;

    if ( bCompleteAccess )
    {
        if ( subindex == 0 )
        {
            /* we change the subindex 0 */
            maxSubindex = (UINT8) pData[0];
        }

        /* we write until the maximum subindex */
        lastSubindex = (UINT8)maxSubindex;
    }
    else
    if ( subindex > maxSubindex )
        /* the maximum subindex is reached */
        return ABORTIDX_SUBINDEX_NOT_EXISTING;
    else
    {
        /* we check the write access for single accesses here, a complete write access
           is allowed if at least one entry is writable (in this case the values for the
            read only entries shall be ignored) */
        /* we get the corresponding entry description */
        pEntry = &asEntryDesc0xF030[subindex];

        /* check if we have write access (bits 3-5 (PREOP, SAFEOP, OP) of ObjAccess)
           by comparing with the actual state (bits 1-3 (PREOP, SAFEOP, OP) of AL Status) */
        if ( ((UINT8) ((pEntry->ObjAccess & ACCESS_WRITE) >> 2)) < (nAlStatus & STATE_MASK) )
        {
            /* we don't have write access */
            if ( (pEntry->ObjAccess & ACCESS_WRITE) == 0 )
            {
                /* it is a read only entry */
                return ABORTIDX_READ_ONLY_ENTRY;
            }
            else
            {
                /* we don't have write access in this state */
                return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
            }
        }
    }

        /* we use the standard write function */
        for (i = subindex; i <= lastSubindex; i++)
        {
            /* we have to copy the entry */
            if (i == 0)
            {
                /*check if the value for subindex0 is valid */
                if(MAX_AXES < (UINT8) pData[0])
                {
                    return ABORTIDX_VALUE_TOO_GREAT;
                }

                sConfiguredModuleIdentList.u16SubIndex0 =  pData[0];

                /* we increment the destination pointer by 2 because the subindex 0 will be
                    transmitted as UINT16 for a complete access */
                pData++;
            }
            else
            {
                UINT32 CurValue = sConfiguredModuleIdentList.aEntries[(i-1)];
                UINT16 MBXMEM *pVarPtr = (UINT16 MBXMEM *) &sConfiguredModuleIdentList.aEntries[(i-1)];

                pVarPtr[0] = pData[0];
                pVarPtr[1] = pData[1];
                pData += 2;

                /*Check if valid value was written*/
                if((sConfiguredModuleIdentList.aEntries[(i-1)] != (UINT32)CSV_CSP_MODULE_ID)
                && (sConfiguredModuleIdentList.aEntries[(i-1)] != (UINT32)CSP_MODULE_ID)
                && (sConfiguredModuleIdentList.aEntries[(i-1)] != (UINT32)CSV_MODULE_ID)
                && (sConfiguredModuleIdentList.aEntries[(i-1)] != 0))
                {
                    /*write previous value*/
                    sConfiguredModuleIdentList.aEntries[(i-1)] = CurValue;

                    /*reset subindex 0 (if required)*/
                    if(sConfiguredModuleIdentList.aEntries[(i-1)] != 0)
                    {
                        sConfiguredModuleIdentList.u16SubIndex0 = i;
                    }
                    else
                    {
                        /*current entry is 0 => set subindex0 value i-1*/
                        sConfiguredModuleIdentList.u16SubIndex0 = (i-1);
                    }


                    return ABORTIDX_VALUE_EXCEEDED;
                }
           }
        }

        /*Update PDO assign objects and 0xF010 (Module Profile List)*/
        {
        UINT8 cnt = 0;

        /*Update 0xF010.0 */
        sModuleProfileInfo.u16SubIndex0 = sConfiguredModuleIdentList.u16SubIndex0;

        /*Update PDO assign SI0*/
        sRxPDOassign.u16SubIndex0 = sConfiguredModuleIdentList.u16SubIndex0;
        sTxPDOassign.u16SubIndex0 = sConfiguredModuleIdentList.u16SubIndex0;

        for (cnt = 0 ; cnt < sConfiguredModuleIdentList.u16SubIndex0; cnt++)
        {
            /*all Modules have the same profile number*/
            sModuleProfileInfo.aEntries[cnt] = DEVICE_PROFILE_TYPE;

            switch(sConfiguredModuleIdentList.aEntries[cnt])
            {
                case CSV_CSP_MODULE_ID:
                    sRxPDOassign.aEntries[cnt] = (0x1600 +(0x10*cnt));
                    sTxPDOassign.aEntries[cnt] = (0x1A00 +(0x10*cnt));
                break;
                case CSP_MODULE_ID:
                    sRxPDOassign.aEntries[cnt] = (0x1601 +(0x10*cnt));
                    sTxPDOassign.aEntries[cnt] = (0x1A01 +(0x10*cnt));
                break;
                case CSV_MODULE_ID:
                    sRxPDOassign.aEntries[cnt] = (0x1602 +(0x10*cnt));
                    sTxPDOassign.aEntries[cnt] = (0x1A02 +(0x10*cnt));
                break;
				case CSVP_MODULE_ID:
                    sRxPDOassign.aEntries[cnt] = (0x1603 +(0x10*cnt));
                    sTxPDOassign.aEntries[cnt] = (0x1A03 +(0x10*cnt));
                break;
                default:
                    sRxPDOassign.aEntries[cnt] = 0;
                    sTxPDOassign.aEntries[cnt] = 0;

                    sModuleProfileInfo.aEntries[cnt] = 0;
                break;
            }
        }
    }
    return 0;
}

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*////////////////////////////////////////////////////////////////////////////////////////

void    APPL_AckErrorInd(UINT16 stateTrans)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
           all general settings were checked to start the mailbox handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete
           the transition by calling ECAT_StateChange.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
           to stop the mailbox handler. This functions informs the application
           about the state transition, the application cannot refuse
           the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
             all general settings were checked to start the input handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case the application need to be complete
            the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    UINT32 Sync0CycleTime = 0;

    if(sConfiguredModuleIdentList.u16SubIndex0 == 0)
    {
        /* Object 0xF030 was not written before
        * => update object 0xF010 (Module profile list) and 0xF050 (Detected Module List)*/

      UINT8 cnt = 0;

        /*Update 0xF010.0 */
        sModuleProfileInfo.u16SubIndex0 = sRxPDOassign.u16SubIndex0;

        /*Update 0xF050.0*/
        sDetectedModuleIdentList.u16SubIndex0 = sRxPDOassign.u16SubIndex0;



        for (cnt = 0 ; cnt < sRxPDOassign.u16SubIndex0; cnt++)
        {
            /*all Modules have the same profile number*/
            sModuleProfileInfo.aEntries[cnt] = DEVICE_PROFILE_TYPE;

            switch((sRxPDOassign.aEntries[cnt] & 0x000F)) //get only identification of the PDO mapping object
            {
                case 0x0:   //csv/csp PDO selected
                    sDetectedModuleIdentList.aEntries[cnt] = CSV_CSP_MODULE_ID;
                break;
                case 0x1:   //csp PDO selected
                    sDetectedModuleIdentList.aEntries[cnt] = CSP_MODULE_ID;
                break;
                case 0x2:   //csv PDO selected
                    sDetectedModuleIdentList.aEntries[cnt] = CSV_MODULE_ID;
                break;

            }

        }
			}

    HW_EscReadDWord(Sync0CycleTime, ESC_DC_SYNC0_CYCLETIME_OFFSET);
    Sync0CycleTime = SWAPDWORD(Sync0CycleTime);

    /*Init CiA402 structure if the device is in SM Sync mode
    the CiA402 structure will be Initialized after calculation of the Cycle time*/
    if(bDcSyncActive == TRUE)
    {
        UINT16 i;
        Sync0CycleTime = Sync0CycleTime / 1000; //get cycle time in us
        for(i = 0; i< MAX_AXES;i++)
        {
            if(LocalAxes[i].bAxisIsActive)
                LocalAxes[i].u32CycleTime = Sync0CycleTime;
        }
    }

		 return ALSTATUSCODE_NOERROR;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete
           the transition by calling ECAT_StateChange.
*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16* pInputSize,UINT16* pOutputSize)
{
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 PDOAssignEntryCnt = 0;
    UINT8 AxisIndex = 0;
    UINT16 u16cnt = 0;
    UINT16 InputSize = 0;
    UINT16 OutputSize = 0;
    TOBJECT OBJMEM *pDiCEntry = NULL;

    if(sRxPDOassign.u16SubIndex0 > MAX_AXES)
        return ALSTATUSCODE_NOVALIDOUTPUTS;

    /*Update object dictionary according to activated axis
    which axes are activated is calculated by object 0x1C12*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        /*The PDO mapping objects are specified with an 0x10 offset => get the axis index*/
        AxisIndex = (sRxPDOassign.aEntries[PDOAssignEntryCnt] & 0xF0) >> 4;

        if(AxisIndex == PDOAssignEntryCnt)
        {
            /*Axis is mapped to process data check if axis objects need to be added to the object dictionary*/

            if(!LocalAxes[PDOAssignEntryCnt].bAxisIsActive)
            {
                /*add objects to dictionary*/
                pDiCEntry = LocalAxes[PDOAssignEntryCnt].ObjDic;

                while(pDiCEntry->Index != 0xFFFF)
                {
                    result = COE_AddObjectToDic(pDiCEntry);

                    if(result != 0)
                    {
                        return result;
                    }

                    pDiCEntry++;    //get next entry
                }


                LocalAxes[PDOAssignEntryCnt].bAxisIsActive = TRUE;
            }

        }
        else
        {
            /*Axis is not mapped to process data check if axis objects need to be removed from object dictionary*/
            if(LocalAxes[PDOAssignEntryCnt].bAxisIsActive)
            {
                /*add objects to dictionary*/
                pDiCEntry = LocalAxes[PDOAssignEntryCnt].ObjDic;

                while(pDiCEntry->Index != 0xFFFF)
                {
                    COE_RemoveDicEntry(pDiCEntry->Index);

                    pDiCEntry++;    //get next entry
                }


                LocalAxes[PDOAssignEntryCnt].bAxisIsActive = FALSE;
            }
        }

    }

    /*Scan object 0x1C12 RXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        switch ((sRxPDOassign.aEntries[PDOAssignEntryCnt] & 0x000F))    //mask Axis type (supported modes)
        {
            case 0:
                /*drive mode supported    csv(cyclic sync velocity) : bit 8
                                        csp (cyclic sync position) : bit 7*/
                LocalAxes[PDOAssignEntryCnt].Objects.objSupportedDriveModes = 0x180;
                for(u16cnt =0 ; u16cnt < LocalAxes[PDOAssignEntryCnt].Objects.sRxPDOMap0.u16SubIndex0;u16cnt++)
                {
                    OutputSize +=(UINT16)(LocalAxes[PDOAssignEntryCnt].Objects.sRxPDOMap0.aEntries[u16cnt] & 0xFF);
                }
                break;
            case 1:
                /*drive mode supported    csp (cyclic sync position) : bit 7*/
                LocalAxes[PDOAssignEntryCnt].Objects.objSupportedDriveModes = 0x80;
                for(u16cnt =0 ; u16cnt < LocalAxes[PDOAssignEntryCnt].Objects.sRxPDOMap1.u16SubIndex0;u16cnt++)
                {
                    OutputSize +=(UINT16)(LocalAxes[PDOAssignEntryCnt].Objects.sRxPDOMap1.aEntries[u16cnt] & 0xFF);
                }
                break;
            case 2:
                    /*drive mode supported    csv(cyclic sync velocity) : bit 8*/
                LocalAxes[PDOAssignEntryCnt].Objects.objSupportedDriveModes= 0x100;
                for(u16cnt =0 ; u16cnt < LocalAxes[PDOAssignEntryCnt].Objects.sRxPDOMap2.u16SubIndex0;u16cnt++)
                {
                    OutputSize += (UINT16)(LocalAxes[PDOAssignEntryCnt].Objects.sRxPDOMap2.aEntries[u16cnt] & 0xFF);;
                }
                break;
							case 3:
                /*drive mode supported    csv(cyclic sync velocity) : bit 8
                                        csp (cyclic sync position) : bit 7*/
                LocalAxes[PDOAssignEntryCnt].Objects.objSupportedDriveModes = 0x180;
                for(u16cnt =0 ; u16cnt < LocalAxes[PDOAssignEntryCnt].Objects.sRxPDOMap3.u16SubIndex0;u16cnt++)
                {
                    OutputSize +=(UINT16)(LocalAxes[PDOAssignEntryCnt].Objects.sRxPDOMap3.aEntries[u16cnt] & 0xFF);
                }
                break;
        }
    }

    OutputSize = OutputSize >> 3;

    if(result == 0)
    {
        /*Scan Object 0x1C13 TXPDO assign*/
        for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
        {
            switch ((sTxPDOassign.aEntries[PDOAssignEntryCnt] & 0x000F))    //mask Axis type (supported modes)
            {
            case 0: /*csp/csv*/
                for(u16cnt =0 ; u16cnt < LocalAxes[PDOAssignEntryCnt].Objects.sTxPDOMap0.u16SubIndex0;u16cnt++)
                {
                    InputSize +=(UINT16)(LocalAxes[PDOAssignEntryCnt].Objects.sTxPDOMap0.aEntries[u16cnt] & 0xFF);
                }
                break;
            case 1: /*csp*/
                for(u16cnt =0 ; u16cnt < LocalAxes[PDOAssignEntryCnt].Objects.sTxPDOMap1.u16SubIndex0;u16cnt++)
                {
                    InputSize +=(UINT16)(LocalAxes[PDOAssignEntryCnt].Objects.sTxPDOMap1.aEntries[u16cnt] & 0xFF);
                }
                break;
            case 2: /*csv*/
                for(u16cnt =0 ; u16cnt < LocalAxes[PDOAssignEntryCnt].Objects.sTxPDOMap2.u16SubIndex0;u16cnt++)
                {
                    InputSize +=(UINT16)(LocalAxes[PDOAssignEntryCnt].Objects.sTxPDOMap2.aEntries[u16cnt] & 0xFF);
                }
                break;
						case 3: /*csp/csv*/
                for(u16cnt =0 ; u16cnt < LocalAxes[PDOAssignEntryCnt].Objects.sTxPDOMap3.u16SubIndex0;u16cnt++)
                {
                    InputSize +=(UINT16)(LocalAxes[PDOAssignEntryCnt].Objects.sTxPDOMap3.aEntries[u16cnt] & 0xFF);
                }
                break;

            }
        }

        InputSize = InputSize >> 3;
    }

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data
\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_InputMapping(UINT16* pData)
{
    UINT16 j = 0;
    UINT16 *pTmpData = (UINT16 *)pData;
    UINT8 AxisIndex;

    for (j = 0; j < sTxPDOassign.u16SubIndex0; j++)
    {
        /*The Axis index is based on the PDO mapping offset (0x10)*/


        AxisIndex = ((sTxPDOassign.aEntries[j] & 0xF0) >> 4);

//        LocalAxes[AxisIndex].Objects.objPositionActualValue = ((int32)stPosLoop.fFbkAcc) & 0xFFFFFFFF; //ECAT_MOD
				LocalAxes[AxisIndex].Objects.objPositionActualValue = ((int32)gsM1_Drive.sPositionControl.f32PositionComp) & 0xFFFFFFFF;
        LocalAxes[AxisIndex].Objects.objVelocityActualValue = (int32)gsM1_Drive.sSpeed.f32SpeedFilt;
        LocalAxes[AxisIndex].Objects.objExternalPositionValue = LocalAxes[AxisIndex].Objects.objPositionActualValue + 10;//TEST
//        LocalAxes[AxisIndex].Objects.objTorqueActualValue = (int16)(gsM1_Drive.sFocPMSM.sIDQ.f32Q * Kt * 1000.0f / TE_RATED);
				LocalAxes[AxisIndex].Objects.objTorqueActualValue = (int16)(gsM1_Drive.sFocPMSM.sIDQ.f32Q * Kt * 1000.0f);


        switch ((sTxPDOassign.aEntries[j]& 0x000F))
        {
        case 0:    //copy csp/csv TxPDO entries
				*pTmpData++ = LocalAxes[AxisIndex].Objects.objStatusWord;    //0x60410010
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objPositionActualValue & 0x0000FFFF;  //0x60640020
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objPositionActualValue >> 16;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objVelocityActualValue & 0x0000FFFF;  //0x606C0020
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objVelocityActualValue >> 16;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objExternalPositionValue & 0x0000FFFF;//0x20B60020
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objExternalPositionValue >> 16;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objTorqueActualValue;                 //0x60770010
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objModesOfOperationDisplay & 0x00FF;  //0x60610008
            break;
        case 1://copy csp TxPDO entries
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objStatusWord;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objPositionActualValue & 0x0000FFFF;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objPositionActualValue>>16;
            break;
        case 2://copy csv TxPDO entries
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objStatusWord;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objPositionActualValue & 0x0000FFFF;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objPositionActualValue>>16;
            break;
		   case 3:
				*pTmpData++ = LocalAxes[AxisIndex].Objects.objStatusWord;    //0x60410010
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objPositionActualValue & 0x0000FFFF;  //0x60640020
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objPositionActualValue >> 16;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objVelocityActualValue & 0x0000FFFF;  //0x606C0020
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objVelocityActualValue >> 16;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objExternalPositionValue & 0x0000FFFF;//0x20B60020
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objExternalPositionValue >> 16;
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objTorqueActualValue;                 //0x60770010
                *pTmpData++ = LocalAxes[AxisIndex].Objects.objModesOfOperationDisplay & 0x00FF;  //0x60610008
            break;
        }    //switch TXPDO entry
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_OutputMapping(UINT16* pData)
{
    UINT16 j = 0;
	 // UINT16  Servo_controlall=0;
	    UINT16   Data =0;
    UINT16 *pTmpData = (UINT16 *)pData;
    UINT8 AxisIndex;


    for (j = 0; j < sRxPDOassign.u16SubIndex0; j++)
    {
        /*The Axis index is based on the PDO mapping offset (0x10)*/
        AxisIndex = ((sRxPDOassign.aEntries[j] & 0xF0) >> 4);
        /*1600 1610  1620*/
        switch ((sRxPDOassign.aEntries[j] & 0x000F))
        {          /*1600 1601 1602*/
        case 0:    //csp/csv RxPDO    entries
			LocalAxes[AxisIndex].Objects.objControlWord = SWAPWORD(*pTmpData++);                   //0x60400010
            Data = (SWAPWORD(*pTmpData++));
            LocalAxes[AxisIndex].Objects.objTargetPosition = Data+ ((SWAPWORD(*pTmpData++)) << 16);  //0x607A0020
            Data = (SWAPWORD(*pTmpData++));
            LocalAxes[AxisIndex].Objects.objTargetVelocity = Data+ ((SWAPWORD(*pTmpData++)) << 16);  //0x60FF0020
            LocalAxes[AxisIndex].Objects.objTorqueOffset = SWAPWORD(*pTmpData++);                  //0x60B20010
            LocalAxes[AxisIndex].Objects.objModesOfOperation = (SWAPWORD(*pTmpData++))& 0x00FF;    //0x60600008
            break;
          case 1:    //csp RxPDO    entries
		    LocalAxes[AxisIndex].Objects.objControlWord = SWAPWORD(*pTmpData++);
            Data  =(SWAPWORD(*pTmpData++));
            LocalAxes[AxisIndex].Objects.objTargetPosition  = Data+((SWAPWORD(*pTmpData++))<<16);

            break;
        case 2:    //csv RxPDO    entries
            LocalAxes[AxisIndex].Objects.objControlWord = SWAPWORD(*pTmpData++);
            Data  =(SWAPWORD(*pTmpData++));
            LocalAxes[AxisIndex].Objects.objTargetVelocity  = Data+((SWAPWORD(*pTmpData++))<<16);
            break;

		case 3:
			LocalAxes[AxisIndex].Objects.objControlWord = SWAPWORD(*pTmpData++);                   //0x60400010
            Data  =(SWAPWORD(*pTmpData++));
            LocalAxes[AxisIndex].Objects.objTargetPosition  = Data+((SWAPWORD(*pTmpData++))<<16);  //0x607A0020
            Data  =(SWAPWORD(*pTmpData++));
            LocalAxes[AxisIndex].Objects.objTargetVelocity  = Data+((SWAPWORD(*pTmpData++))<<16);  //0x60FF0020
            LocalAxes[AxisIndex].Objects.objTorqueOffset = SWAPWORD(*pTmpData++);                  //0x60B20010
            LocalAxes[AxisIndex].Objects.objModesOfOperation = (SWAPWORD(*pTmpData++))& 0x00FF;    //0x60600008;
            break;
        }

//        stPosLoop.lCmd = LocalAxes[AxisIndex].Objects.objTargetPosition;               //ECAT_MOD
				gsM1_Drive.sPositionControl.f32PositionCmd = LocalAxes[AxisIndex].Objects.objTargetPosition;
        gsM1_Drive.sSpeed.f32SpeedCmd = LocalAxes[AxisIndex].Objects.objTargetVelocity;
        switch (LocalAxes[AxisIndex].Objects.objModesOfOperation)
		{
		case 2:  //Velocity Mode
        case 3:  //Profile Velocity Mode
        case 9:  //Profile Velocity Mode
			gsM1_Drive.uw16CtrlMode = SPEED_CONTROL;
			break;
        case 4:  //Torque Profile Mode
        case 10:  //Torque Profile Mode
			gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;
			break;
		case 1:
        case 7:  //Interpolated Position Mod
        case 8:  //Interpolated Position Mod
			gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;
			break;
        default:
            break;
		}
        LocalAxes[AxisIndex].Objects.objModesOfOperationDisplay = LocalAxes[AxisIndex].Objects.objModesOfOperation;

    }
}
/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will called from the synchronisation ISR
            or from the mainloop if no synchronisation is supported
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_Application(void)
{
    UINT16 i;

    for(i = 0; i < MAX_AXES;i++)
    {
        if(LocalAxes[i].bAxisIsActive)
			 CiA402_Application(&LocalAxes[i]);
    }

}



//void GPIOPWM_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
//
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
//	//GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStructure);
//
//}
//void TIM5_Config(void)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	//u16 CCR1=36000;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
//
//	TIM_TimeBaseStructure.TIM_Period =19999;
//	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
//
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	//TIM_OCInitStructure.TIM_Pulse = CCR1;
//	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;
//	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
//	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
//
//	    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    //TIM_OCInitStructure.TIM_Pulse = CCR2;                                       //????2??????,??????????PWM
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;                    //?????????CCR2?????
//    TIM_OC2Init(TIM5, &TIM_OCInitStructure);                                    //????2
//    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
//
//	TIM_ARRPreloadConfig(TIM5, ENABLE);
//	TIM_Cmd(TIM5, ENABLE);
//}
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This is the main function

*////////////////////////////////////////////////////////////////////////////////////////
/*
int main(void)
{
//  GPIOPWM_Config();
//  TIM5_Config();
//	 	TIM_SetCompare1(TIM5,10000);
//	    TIM_SetCompare2(TIM5,10000);
    //LED_Init();


    HW_Init();
    MainInit();
    CiA402_Init();
    APPL_GenerateMapping(&nPdInputSize,&nPdOutputSize);
    bRunApplication = TRUE;
    do
    {

			MainLoop();

    } while (bRunApplication == TRUE);

    CiA402_DeallocateAxis();

    HW_Release();


    return 0;
}
*/

/** @} */

