/******************************************************************************
* Includes
******************************************************************************/
#include "M1_statemachine.h"
#include "ARM_MATH.h"
#include "Configuration.h"
#include "Control.h"
#include "Motor_Drive.h"
#include "include_c.h"
#include "motor_structure.h"
#include "EEPROM_RW.h"

u8 braketest;
u8 TBtest;
float SpeedRefMax;

extern vs32 PositionTarget;
s16 Electrical_Angle;

extern unsigned char AlignmentDone;
extern Volt_Components Stat_Volt_alpha_beta;
extern vs16 hTorque_Reference;
extern s32 Turn_Number;
unsigned char FlashWriteReady;

SM_RUN_SUBSTATE_T meM1_StateRun;

MCSTRUC_CONTROL_LOOP_T geM1_StateRunLoop;
MCSTRUC_FOC_PMSM_ENCODER_T gsM1_Drive;

static void M1_StateFault(void);
static void M1_StateInit(void);
static void M1_StateStop(void);
static void M1_StateRun(void);
static void M1_StateAlign(void);
static void M1_StateUpdate(void);

static void M1_TransFaultInit(void);
static void M1_TransInitFault(void);
static void M1_TransInitStop(void);
static void M1_TransStopFault(void);
static void M1_TransStopInit(void);
static void M1_TransStopRun(void);
static void M1_TransRunFault(void);
static void M1_TransRunStop(void);
static void M1_TransStopAlign(void);
static void M1_TransAlignStop(void);
static void M1_TransAlignFault(void);
static void M1_TransAlignFault(void);
static void M1_TransStopUpdate(void);

static const SM_APP_STATE_FCN_T msSTATE = {M1_StateFault, M1_StateInit, M1_StateStop, M1_StateRun, M1_StateAlign, M1_StateUpdate};

static const SM_APP_TRANS_FCN_T msTRANS = {M1_TransFaultInit, M1_TransInitFault, M1_TransInitStop, M1_TransStopFault, M1_TransStopInit, M1_TransStopRun, M1_TransRunFault, M1_TransRunStop, M1_TransAlignStop, M1_TransStopAlign, M1_TransAlignFault, M1_TransStopUpdate};

SM_APP_CTRL_T gsM1_Ctrl =
    {
        &msSTATE,

        &msTRANS,

        SM_CTRL_NONE,

        INIT};

static void M1_StateRunReady(void);
static void M1_StateRunRotation(void);
static void M1_StateRunReadySlow(void);
static void M1_StateRunRotationSlow(void);

static void M1_TransRunReadyRotation(void);
//static void M1_TransRunRotationReady(void);

static const PFCN_VOID_VOID mM1_STATE_RUN_TABLE[2][2] =
    {
        {M1_StateRunReady, M1_StateRunReadySlow},
        {M1_StateRunRotation, M1_StateRunRotationSlow},
};

static void M1_StateUpdate(void)
{
    PWM_OUT_DISABLE();
    Brake_On();

    static int i = 0;
    static bool oncedo = FALSE;
    if(geM1_StateRunLoop == SLOW)
    {
        if(FALSE == oncedo)
        {
            i++;
            if(i>20)
            {
                i = 0;
                g_stEEPROM_RW.m_Write_IAP_Flag(g_stIAPFlag.buf,(u32)2);//
                //Fanpengcan TODO need to confirm the waite date
                oncedo = TRUE;
            }
        }
    }
}

u8 FaultStep;
u16 Fault_Hold_Delay;
u32 lCtrlErrOld;
static void M1_StateFault(void) {
    PWM_OUT_DISABLE();

    switch (FaultStep) {
        case 0:
            if ((Brake_Off_Check() == 1) && (gsM1_Drive.sFaultId.B.OverCurrent != 1) && (gsM1_Drive.sFaultId.B.PhaseWShortCirciut != 1) && (gsM1_Drive.sFaultId.B.PowerSupplyOff != 1) &&
                (gsM1_Drive.sFaultId.B.IdDeltaError != 1) && (gsM1_Drive.sFaultId.B.IqDeltaError != 1)) {
                FaultStep = 1;
                PWM_OUT_ENABLE();
            } else {
                FaultStep = 2;
            }

            if ((gsM1_Drive.sSpeed.f32Speed == 0) && (Brake_Off_Check() == 0)) {
                FaultStep = 3;
            }

            break;

        case 1:
            if ((Fault_Hold_Delay >= 20000) || (fabsf(gsM1_Drive.sSpeed.f32Speed) < 60.0f)) {
                FaultStep = 2;
                Fault_Hold_Delay = 0;
            } else {
                if (gsM1_Drive.uw16CtrlMode != TORQUE_CONTROL)
                    PositionHold();
                else
                    PWM_OUT_DISABLE();

                Fault_Hold_Delay++;
            }

            break;

        case 2:
            Brake_On();

            Fault_Hold_Delay++;

            if (Fault_Hold_Delay >= 20)
                FaultStep = 3;

            break;

        case 3:
            PWM_OUT_DISABLE();

            Fault_Hold_Delay = 0;

            V_AlphaBeta_Reset();
            I_AlphaBeta_Reset();
            V_QD_Reset();
            I_QD_Reset();

            PositionLocatingReset();

            Led_Fault_Indication_Set();

            FaultStep = 4;

            break;

        case 4:
            break;

        default:
            break;
    }

    if (gsM1_Drive.sFaultId.R > 0) {
        gsM1_Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }
		
#ifdef ETHERCAT_RUN
    if ((lCtrlErrOld == 0) && (gsM1_Drive.sFaultId.R > 0)) {
        CiA402_LocalError((Uint16)(gsM1_Drive.sFaultId.R & 0xFFFF));
    }
#endif
		
    lCtrlErrOld = gsM1_Drive.sFaultId.R;
}

static void M1_StateInit(void) {
    PWM_OUT_DISABLE();

    MotorDrive_Init();

    V_AlphaBeta_Reset();
    I_AlphaBeta_Reset();
    V_QD_Reset();
    I_QD_Reset();

    Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

    gsM1_Drive.sFaultId.R = 0;

    meM1_StateRun = READY;

    gsM1_Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;

#if HARDWARE_VERSION_2_0 == 1

    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 1)
        gsM1_Drive.sFaultId.B.PhaseWShortCirciut = 1;

#endif

#if HARDWARE_VERSION_2_2 == 1
    //if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14) == 1)
    if ((GPIOB->IDR & GPIO_Pin_14) == 1)
        gsM1_Drive.sFaultId.B.PhaseWShortCirciut = 1;

#endif

    if (gsM1_Drive.sFaultId.R > 0) {
        gsM1_Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }
}

u8 StopStep;
u16 Hold_Delay;
static void M1_StateStop(void) {
#if (INIT_LOCATING == 1) || (ANGLE_SEARCHING == 1)
    switch (StopStep) {
        case 0:
            if ((Brake_Off_Check() == 1)) {
                StopStep = 2;
                PWM_OUT_DISABLE();
            } else {
                PWM_OUT_DISABLE();
                StopStep = 4;
            }

            break;

        case 1:
            if ((Hold_Delay >= 20000) || (fabsf(gsM1_Drive.sSpeed.f32Speed) < 60.0f)) {
                StopStep = 2;
                Hold_Delay = 0;
            } else {
                if (gsM1_Drive.uw16CtrlMode != TORQUE_CONTROL)
                    PositionHold();
                else
                    PWM_OUT_DISABLE();

                Hold_Delay++;
            }

            break;

        case 2:
            Brake_On();

            Hold_Delay++;

            if (Hold_Delay >= 1000)
                StopStep = 3;

            break;

        case 3:
            PWM_OUT_DISABLE();

            V_AlphaBeta_Reset();
            I_AlphaBeta_Reset();
            V_QD_Reset();
            I_QD_Reset();
            I_QD_Ref_Reset();
            Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);
            Hold_Delay = 0;

            MotorDrive_Init();

            StopStep = 4;

            break;

        case 4:
            TIM1->CCER &= (~0x111);
            PWM_OUT_ENABLE();

            Current_Offset_Adjust();
            break;

        default:
            break;
    }

#else

#ifndef HFI
    static u8 Delay_Set;

    PWM_OUT_DISABLE();

    Current_Offset_Adjust();

    if ((gsM1_Drive.sSpeed.f32Speed == 0) && (Brake_Off_Check() == 1) && (Delay_Set == 0)) {
        TB_Set_Delay(BRAKE_ON_DELAY);
        Delay_Set = 1;
    }

    if ((TB_delay_IsElapsed()) && (Delay_Set == 1)) {
        Brake_On();
        Delay_Set = 0;
    }

#else

    switch (StopStep) {
        case 0:
            if ((Brake_Off_Check() == 1)) {
                if (meM1_StateRun == READY) {
                    StopStep = 2;
                } else {
                    StopStep = 1;
                    Motor_Drive_Get_Electrical_Angle();
                    PWM_OUT_ENABLE();
                }
            } else {
                PWM_OUT_DISABLE();
                StopStep = 3;
            }

            break;

        case 1:
            if ((Hold_Delay >= 20000) || (fabsf(gsM1_Drive.sSpeed.f32Speed) < 60.0f)) {
                StopStep = 2;
                Hold_Delay = 0;
            } else {
                if (gsM1_Drive.uw16CtrlMode != TORQUE_CONTROL)
                    PositionHold();
                else
                    PWM_OUT_DISABLE();

                Hold_Delay++;
            }

            break;

        case 2:
            Brake_On();

            Hold_Delay++;

            if (Hold_Delay >= 1000)
                StopStep = 3;

            break;

        case 3:
            PWM_OUT_DISABLE();

            V_AlphaBeta_Reset();
            I_AlphaBeta_Reset();
            V_QD_Reset();
            I_QD_Reset();
            I_QD_Ref_Reset();
            Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);
            Hold_Delay = 0;

            MotorDrive_Init();

            StopStep = 4;

            break;

        case 4:
            TIM1->CCER &= (~0x111);
            PWM_OUT_ENABLE();

            Current_Offset_Adjust();
            break;

        default:
            break;
    }

#endif

#endif

    if (gsM1_Drive.sFaultId.R > 0) {
        gsM1_Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }
}

static void M1_StateRun(void) {
#if (INIT_LOCATING == 0) && (ANGLE_SEARCHING == 0)

#ifndef HFI
    if (TB_delay_IsElapsed()) {
        Brake_Hold();

        mM1_STATE_RUN_TABLE[meM1_StateRun][geM1_StateRunLoop]();
    }
#else
    mM1_STATE_RUN_TABLE[meM1_StateRun][geM1_StateRunLoop]();
#endif

#else

    mM1_STATE_RUN_TABLE[meM1_StateRun][geM1_StateRunLoop]();

#endif

    if (gsM1_Drive.sFaultId.R > 0) {
        gsM1_Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }
}

static void M1_StateAlign(void) {
    if ((TB_delay_IsElapsed()) && (AlignmentDone == 0)) {
        Brake_Hold();

        if (geM1_StateRunLoop == FAST) {
            Rotor_Alignment();
        }
    }

    if (gsM1_Drive.sFaultId.R > 0) {
        gsM1_Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }

    if (AlignmentDone == 1) {
        if (FlashWriteReady == 0) {
            TB_Set_Delay(FLASH_STORAGE_DELAY);
            FlashWriteReady = 1;
        }

        if (TB_delay_IsElapsed()) {
            gsM1_Drive.sPositionEnc.f32MagnetOffsetRadEl = 0;

            gsM1_Drive.sPositionEnc.s32MagnetOffset = 0;

            gsM1_Ctrl.uiCtrl |= SM_CTRL_ALIGN_DONE;

            AlignmentDone = 0;
            FlashWriteReady = 0;
        }
    }
}

static void M1_TransFaultInit(void) {
    FaultStep = 0;

    Led_Fault_Indication_Clr();
}

static void M1_TransInitFault(void) {
    V_AlphaBeta_Reset();
    I_AlphaBeta_Reset();
    V_QD_Reset();
    I_QD_Reset();

    Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

    Led_Fault_Indication_Set();
}

static void M1_TransInitStop(void) {
    V_AlphaBeta_Reset();
    I_AlphaBeta_Reset();
    V_QD_Reset();
    I_QD_Reset();
    I_QD_Ref_Reset();

    Motor_Drive_Get_Electrical_Angle();
}

static void M1_TransStopFault(void) {
    PWM_OUT_DISABLE();

    V_AlphaBeta_Reset();
    I_AlphaBeta_Reset();
    V_QD_Reset();
    I_QD_Reset();

    Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

    Led_Fault_Indication_Set();

    StopStep = 0;
}

static void M1_TransStopUpdate(void) {
    PWM_OUT_DISABLE();

    V_AlphaBeta_Reset();
    I_AlphaBeta_Reset();
    V_QD_Reset();
    I_QD_Reset();

    Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

    StopStep = 0;
}

static void M1_TransStopInit(void) {
}

static void M1_TransStopRun(void) {
    StopStep = 0;

#if (INIT_LOCATING == 0) && (ANGLE_SEARCHING == 0)

#ifndef HFI
    Brake_Off();
    TB_Set_Delay(BRAKE_OFF_DELAY);
#endif

#endif

    meM1_StateRun = READY;

    gsM1_Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}

static void M1_TransRunFault(void) {
}

static void M1_TransRunStop(void) {
    if (meM1_StateRun == READY) {
#if INIT_LOCATING == 1

        if (gsM1_Drive.sInitLocating.u8LocatingStep <= 13) {
            Brake_Hold();

            gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
        } else if (gsM1_Drive.sInitLocating.u8LocatingStep == 14) {
            return;
        } else {
            Motor_Drive_Get_Electrical_Angle();

            gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
        }
#else

        gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;

#endif

    } else {
        gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
    }
}

static void M1_TransStopAlign(void) {
    PWM_OUT_ENABLE();

    Brake_Off();
    TB_Set_Delay(BRAKE_OFF_DELAY);
}

static void M1_TransAlignStop(void) {
    Motor_Drive_Get_Electrical_Angle();

    V_AlphaBeta_Reset();
    I_AlphaBeta_Reset();
    V_QD_Reset();
    I_QD_Reset();
    I_QD_Ref_Reset();

    gsM1_Drive.sFocPMSM.sIABC_Peak.f32A = 0;
    gsM1_Drive.sFocPMSM.sIABC_Peak.f32B = 0;
    gsM1_Drive.sFocPMSM.sIABC_Peak.f32C = 0;

    gsM1_Drive.sFocPMSM.sUAlBeReq_Peak.f32Alpha = 0;
    gsM1_Drive.sFocPMSM.sUAlBeReq_Peak.f32Beta = 0;

    gsM1_Drive.sFocPMSM.sUDQReq_Peak.f32D = 0;
    gsM1_Drive.sFocPMSM.sUDQReq_Peak.f32Q = 0;

    gsM1_Drive.sFocPMSM.sIdPiParams.f32ErrPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32IntegPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32IntegPartK_1 = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32OutputPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32PropPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.i16LimitFlag = 0;

    gsM1_Drive.sFocPMSM.sIqPiParams.f32ErrPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32IntegPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32IntegPartK_1 = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32OutputPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32PropPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.i16LimitFlag = 0;

    gsM1_Drive.sSpeed.sSpeedPiParams.f32ErrPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32IntegPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32IntegPartK_1 = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32OutputPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32PropPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.i16LimitFlag = 0;

    Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

    PWM_OUT_DISABLE();
}

static void M1_TransAlignFault(void) {
    PWM_OUT_DISABLE();

    V_AlphaBeta_Reset();
    I_AlphaBeta_Reset();
    V_QD_Reset();
    I_QD_Reset();

    Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

    Led_Fault_Indication_Set();
}

static void M1_StateRunReady(void) {
#if INIT_LOCATING == 1

    InitPositionLocating();

    if ((gsM1_Drive.sInitLocating.u8LocatingStep >= 15) && (gsM1_Drive.uw16CtrlMode == TORQUE_CONTROL) && (gsM1_Drive.sFocPMSM.sIDQReq.f32Q != 0)) {
        M1_TransRunReadyRotation();
    }

#else

#ifdef HFI

    HFI_Search();

    if (gsM1_Drive.sHFISearch.u8Step >= 12) {
        M1_TransRunReadyRotation();
    }

#else

    if ((gsM1_Drive.uw16CtrlMode == TORQUE_CONTROL) && (gsM1_Drive.sFocPMSM.sIDQReq.f32Q != 0)) {
        M1_TransRunReadyRotation();
    }

#endif

#endif
}

static void M1_StateRunReadySlow(void) {
#if (INIT_LOCATING == 1) || (ANGLE_SEARCHING == 1)

#if INIT_LOCATING == 1
    if (gsM1_Drive.sInitLocating.u8LocatingStep >= 15)
#endif
    {

        if ((gsM1_Drive.uw16CtrlMode == SPEED_CONTROL) || (gsM1_Drive.uw16CtrlMode == OPENLOOP_CONTROL)) {
            if (gsM1_Drive.sSpeed.f32SpeedCmd != 0) {
                M1_TransRunReadyRotation();
            }
        }

        if ((gsM1_Drive.uw16CtrlMode == POSITION_CONTROL) && (fabsf(gsM1_Drive.sPositionControl.f32PositionCmd - gsM1_Drive.sPositionEnc.f32PositionMech) > gsM1_Drive.sPositionControl.f32StartInterval)) {
            M1_TransRunReadyRotation();
        }

        if ((gsM1_Drive.uw16CtrlMode == TRACK_CONTROL) && (fabsf(gsM1_Drive.sPositionControl.f32PositionCmd - gsM1_Drive.sPositionEnc.f32PositionMech) > gsM1_Drive.sPositionControl.f32StartInterval)) {
            M1_TransRunReadyRotation();
        }

        if (gsM1_Drive.uw16CtrlMode == OPENLOOP_PWM) {
            if (gsM1_Drive.sFocPMSM.sUDQReq.f32Q != 0) {
                M1_TransRunReadyRotation();
            }
        }
    }
#else

#ifndef HFI
    if (gsM1_Drive.uw16CtrlMode == TORQUE_CONTROL) {
        if (gsM1_Drive.sFocPMSM.sIDQReq.f32Q != 0) {
            M1_TransRunReadyRotation();
        }
    }

    if ((gsM1_Drive.uw16CtrlMode == SPEED_CONTROL) || (gsM1_Drive.uw16CtrlMode == OPENLOOP_CONTROL)) {
        if (gsM1_Drive.sSpeed.f32SpeedCmd != 0) {
            M1_TransRunReadyRotation();
        }
    }

    if ((gsM1_Drive.uw16CtrlMode == POSITION_CONTROL) && (fabsf(gsM1_Drive.sPositionControl.f32PositionCmd - gsM1_Drive.sPositionEnc.f32PositionMech) > gsM1_Drive.sPositionControl.f32StartInterval)) {
        M1_TransRunReadyRotation();
    }

    if ((gsM1_Drive.uw16CtrlMode == TRACK_CONTROL) && (fabsf(gsM1_Drive.sPositionControl.f32PositionCmd - gsM1_Drive.sPositionEnc.f32PositionMech) > gsM1_Drive.sPositionControl.f32StartInterval)) {
        M1_TransRunReadyRotation();
    }

    if (gsM1_Drive.uw16CtrlMode == OPENLOOP_PWM) {
        if (gsM1_Drive.sFocPMSM.sUDQReq.f32Q != 0) {
            M1_TransRunReadyRotation();
        }
    }
#endif

#endif
}

static void M1_TransRunReadyRotation(void) {
    PWM_OUT_ENABLE();

    meM1_StateRun = ROTATION;
}

static void M1_StateRunRotation(void) {
    Motor_Drive_Get_Electrical_Angle();

    Motor_Drive_FOC();
}

static void M1_StateRunRotationSlow(void) {
    gsM1_Drive.sSpeed.sSpeedPiParams.f32PropGain = gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32PropGain;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32IntegGain = gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32IntegGain;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32AntiSatGain = gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32AntiSatGain;

    if (gsM1_Drive.uw16CtrlMode == SPEED_CONTROL) {
        RampControl(gsM1_Drive.sSpeed.f32SpeedCmd, gsM1_Drive.sSpeed.f32SpeedRampStep, SPEEDLOOP_PERIOD, &gsM1_Drive.sSpeed.f32SpeedReq);

        gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);

        if ((gsM1_Drive.sSpeed.f32SpeedCmd == 0) && (gsM1_Drive.sPositionEnc.f32Speed == 0)) {
            gsM1_Drive.sSpeed.f32SpeedReq = 0;

            V_AlphaBeta_Reset();
            I_AlphaBeta_Reset();

            Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

            V_QD_Reset();
            I_QD_Reset();
            I_QD_Ref_Reset();
        }
    }

    if (gsM1_Drive.uw16CtrlMode == POSITION_CONTROL) {
#if MAGNET_ENCODER_FBK == 0
        gsM1_Drive.sPositionControl.f32PositionComp = gsM1_Drive.sPositionEnc.f32PositionMech;
#else

#endif

        if (gsM1_Drive.sPositionControl.f32PositionCmd != gsM1_Drive.sPositionControl.f32PositionCmdPrevious) {
            gsM1_Drive.sPositionControl.bPositionTargetChange = 1;
            gsM1_Drive.sPositionControl.f32PositionStart = gsM1_Drive.sPositionControl.f32PositionComp;
            gsM1_Drive.sPositionControl.f32PositionRampOut = gsM1_Drive.sPositionControl.f32PositionStart;
            gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionControl.f32PositionCmd;
        } else {
            gsM1_Drive.sPositionControl.bPositionTargetChange = 0;
        }
				
				float __f_a = 1.0f;
				/*        
        float __error = fabs(gsM1_Drive.sPositionControl.sPositionPiParams.f32ErrPartK);

        if(__error * gsM1_Drive.sSpeed.f32SpeedFF > 0.0f)
        {
            if(__error >= 3.0f)
            {
                __f_a = 2.0f;
            }
            else if(__error <= 0.4f)
            {
                __f_a = 0.5f;
            }
            else
            {
             __f_a = 0.5f + 1.5f * (__error - 0.4f) / 2.60f;
            }
        }
        else
        {
            __f_a = 0.5f;
        }
        */

        gsM1_Drive.sPositionControl.sPositionPiParams.f32FFPartK = gsM1_Drive.sSpeed.f32SpeedFF * __f_a;
        gsM1_Drive.sSpeed.f32SpeedCmd = Motor_Drive_Pos_PID_Regulator(gsM1_Drive.sPositionControl.f32PositionCmd, gsM1_Drive.sPositionControl.f32PositionComp, &gsM1_Drive.sPositionControl.sPositionPiParams);
        RampControl(gsM1_Drive.sSpeed.f32SpeedCmd, gsM1_Drive.sSpeed.f32SpeedRampStep, SPEEDLOOP_PERIOD, &gsM1_Drive.sSpeed.f32SpeedReq);

#if TORQUE_FEEDFORWARD == 1
        gsM1_Drive.sSpeed.sSpeedPiParams.f32FFPartK = gsM1_Drive.sPositionControl.f32FeedforwardTorque;
        gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);
#else
        gsM1_Drive.sSpeed.sSpeedPiParams.f32FFPartK = 0;
        gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);
#endif
    }
}

/******************************************************************************
* Global functions
******************************************************************************/
