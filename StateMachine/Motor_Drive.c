#include "Motor_Drive.h"
#include "ARM_MATH.h"
#include "Configuration.h"
#include "Control.h"
#include "M1_statemachine.h"

extern uint8_t SPI3_Rx_Buff[7];
extern uint32_t SACE_SPI3_Rx_Buff;
extern unsigned char AlignmentDone;

u32 magnet_encoder_delta_max;
u32 magnet_encoder_delta_min;

void MotorDrive_Init(void) {
    gsM1_Drive.sADCOffset.f32PhA = DEFAULT_I_A_OFFSET;
    gsM1_Drive.sADCOffset.f32PhB = DEFAULT_I_B_OFFSET;
    gsM1_Drive.sADCOffset.f32Vdc = DEFAULT_VDC_OFFSET;

    gsM1_Drive.sFaultId.R = 0;

    gsM1_Drive.sFaultThresholds.f32CurrentOver = OVER_CURRENT_THRESHOLD;
    gsM1_Drive.sFaultThresholds.f32loadOver = OVER_LOAD_THRESHOLD;
    gsM1_Drive.sFaultThresholds.f32LockRotorOver = LOCK_ROTOR_THRESHOLD;
    gsM1_Drive.sFaultThresholds.f32LossPhaseOver = LOSS_PHASE_THRESHOLD;
    gsM1_Drive.sFaultThresholds.f32OffsetOver = ADC_OFFSET_THRESHOLD;
    gsM1_Drive.sFaultThresholds.f32SpeedOver = OVER_SPEED_THRESHOLD;
    gsM1_Drive.sFaultThresholds.f32UDcBusOver = OVER_VOLTAGE_THRESHOLD;
    gsM1_Drive.sFaultThresholds.f32UDcBusUnder = UNDER_VOLTAGE_THRESHOLD;
    gsM1_Drive.sFaultThresholds.u8OpticalEncoderOver = OPTICAL_ENCODER_THRESHOLD;
    gsM1_Drive.sFaultThresholds.f32LostRotorOver = LOST_ROTOR_THRESHOLD;

    gsM1_Drive.sFaultThresholds.u32UDcBusOverCnt = 0;
    gsM1_Drive.sFaultThresholds.u32UDcBusUnderCnt = 0;
    gsM1_Drive.sFaultThresholds.u32SpeedOverCnt = 0;
    gsM1_Drive.sFaultThresholds.u32OffCancErrorCnt = 0;
    gsM1_Drive.sFaultThresholds.u32OppositeDirectionCnt = 0;
    gsM1_Drive.sFaultThresholds.u32OpticalEncoderErrorCnt = 0;
    gsM1_Drive.sFaultThresholds.u32LostRotorErrorCnt = 0;
    gsM1_Drive.sFaultThresholds.u32LockRotorErrorCnt = 0;
    gsM1_Drive.sFaultThresholds.u32CurrentOverErrorCnt = 0;
    gsM1_Drive.sFaultThresholds.u32CommunicationErrorCnt = 0;
    gsM1_Drive.sFaultThresholds.u32BrakeErrorCnt = 0;
    gsM1_Drive.sFaultThresholds.u32CommunicationLostCnt = 0;
    gsM1_Drive.sFaultThresholds.u32OverHeatCnt = 0;
    gsM1_Drive.sFaultThresholds.u32LossPhaseCnt = 0;
    gsM1_Drive.sFaultThresholds.u32AngleCompareErrorCnt = 0;

    gsM1_Drive.sFocPMSM.bOpenLoop = 0;
    gsM1_Drive.sFocPMSM.bUseMaxBus = 0;
    gsM1_Drive.sFocPMSM.bUseZc = 0;
    gsM1_Drive.sFocPMSM.f32DutyCycleLimit = DEFAULT_DUTY_CYCLE_LIMIT;
    gsM1_Drive.sFocPMSM.f32UDcBus = DEFAULT_VDC;
    gsM1_Drive.sFocPMSM.f32UDcBusFilt = DEFAULT_VDC_FILT;
    gsM1_Drive.sFocPMSM.i16IdPiSatFlag = 0;
    gsM1_Drive.sFocPMSM.i16IqPiSatFlag = 0;

    gsM1_Drive.sFocPMSM.sAlignment.f32IMax = DEFAULT_ALIGNMENT_CURRENT;
    gsM1_Drive.sFocPMSM.sAlignment.f32Position = ALIGNMENT_ANGLE_RAD;
    gsM1_Drive.sFocPMSM.sAlignment.f32Speed = 0;
    gsM1_Drive.sFocPMSM.sAlignment.f32U = 0;
    gsM1_Drive.sFocPMSM.sAlignment.f32UMax = 0;
    gsM1_Drive.sFocPMSM.sAlignment.f32UStep = 0;
    gsM1_Drive.sFocPMSM.sAlignment.uw16TimeAlignment = 0;

    gsM1_Drive.sFocPMSM.sAnglePosEl.f32Cos = 0;
    gsM1_Drive.sFocPMSM.sAnglePosEl.f32Sin = 0;

    gsM1_Drive.sFocPMSM.sAnglePosElUpdate.f32Cos = 0;
    gsM1_Drive.sFocPMSM.sAnglePosElUpdate.f32Sin = 0;

    gsM1_Drive.sFocPMSM.sDutyABC.f32A = 0;
    gsM1_Drive.sFocPMSM.sDutyABC.f32B = 0;
    gsM1_Drive.sFocPMSM.sDutyABC.f32C = 0;

    gsM1_Drive.sFocPMSM.sIABC_Peak.f32A = 0;
    gsM1_Drive.sFocPMSM.sIABC_Peak.f32B = 0;
    gsM1_Drive.sFocPMSM.sIABC_Peak.f32C = 0;

    gsM1_Drive.sFocPMSM.sIABC.f32A = 0;
    gsM1_Drive.sFocPMSM.sIABC.f32B = 0;
    gsM1_Drive.sFocPMSM.sIABC.f32C = 0;

    gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha = 0;
    gsM1_Drive.sFocPMSM.sIAlBe.f32Beta = 0;

    gsM1_Drive.sFocPMSM.sIdPiParams.f32IntegGain = CURRENT_D_KI;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32IntegPartK_1 = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32IntegPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32LowerLimit = CURRENT_D_LOWER_LIMIT;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32PropGain = CURRENT_D_KP;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32UpperLimit = CURRENT_D_UPPER_LIMIT;
    gsM1_Drive.sFocPMSM.sIdPiParams.i16LimitFlag = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32ErrPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32OutputPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32SampleTime = SAMPLE_PERIOD;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32IntegScale = CURRENT_D_INTEG_SCALE;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32DiffGain = CURRENT_D_KD;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32DiffPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32ErrPartK_1 = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32AntiSatGain = 2.0f / gsM1_Drive.sFocPMSM.sIdPiParams.f32PropGain;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32AntiSatPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32FFPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32SampleFreq = 20000.0f;

    gsM1_Drive.sFocPMSM.sIqPiParams.f32IntegGain = CURRENT_Q_KI;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32IntegPartK_1 = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32IntegPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32LowerLimit = CURRENT_Q_LOWER_LIMIT;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32PropGain = CURRENT_Q_KP;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32UpperLimit = CURRENT_Q_UPPER_LIMIT;
    gsM1_Drive.sFocPMSM.sIqPiParams.i16LimitFlag = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32ErrPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32OutputPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32SampleTime = SAMPLE_PERIOD;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32IntegScale = CURRENT_Q_INTEG_SCALE;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32DiffGain = CURRENT_Q_KD;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32DiffPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32ErrPartK_1 = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32AntiSatGain = 2.0f / gsM1_Drive.sFocPMSM.sIqPiParams.f32PropGain;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32AntiSatPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32FFPartK = 0;
    gsM1_Drive.sFocPMSM.sIqPiParams.f32SampleFreq = 20000.0f;

    gsM1_Drive.sFocPMSM.sIDQ.f32D = 0;
    gsM1_Drive.sFocPMSM.sIDQ.f32Q = 0;

    gsM1_Drive.sFocPMSM.sIDQError.f32D = 0;
    gsM1_Drive.sFocPMSM.sIDQError.f32Q = 0;

    gsM1_Drive.sFocPMSM.sIDQReq.f32D = 0;
    gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

    gsM1_Drive.sFocPMSM.sIDQReqZc.f32D = 0;
    gsM1_Drive.sFocPMSM.sIDQReqZc.f32Q = 0;

    gsM1_Drive.sFocPMSM.sIdZcFilter.f32DummyVar = 0;
    gsM1_Drive.sFocPMSM.sIdZcFilter.f32FiltBufferX[0] = 0;
    gsM1_Drive.sFocPMSM.sIdZcFilter.f32FiltBufferX[1] = 0;
    gsM1_Drive.sFocPMSM.sIdZcFilter.f32FiltBufferY[0] = 0;
    gsM1_Drive.sFocPMSM.sIdZcFilter.f32FiltBufferY[1] = 0;
    gsM1_Drive.sFocPMSM.sIdZcFilter.udtFiltCoeff.f32A2 = 0;
    gsM1_Drive.sFocPMSM.sIdZcFilter.udtFiltCoeff.f32B1 = 0;
    gsM1_Drive.sFocPMSM.sIdZcFilter.udtFiltCoeff.f32B2 = 0;

    gsM1_Drive.sFocPMSM.sIqZcFilter.f32DummyVar = 0;
    gsM1_Drive.sFocPMSM.sIqZcFilter.f32FiltBufferX[0] = 0;
    gsM1_Drive.sFocPMSM.sIqZcFilter.f32FiltBufferX[1] = 0;
    gsM1_Drive.sFocPMSM.sIqZcFilter.f32FiltBufferY[0] = 0;
    gsM1_Drive.sFocPMSM.sIqZcFilter.f32FiltBufferY[1] = 0;
    gsM1_Drive.sFocPMSM.sIqZcFilter.udtFiltCoeff.f32A2 = 0;
    gsM1_Drive.sFocPMSM.sIqZcFilter.udtFiltCoeff.f32B1 = 0;
    gsM1_Drive.sFocPMSM.sIqZcFilter.udtFiltCoeff.f32B2 = 0;

    gsM1_Drive.sFocPMSM.sUAlBeComp.f32Alpha = 0;
    gsM1_Drive.sFocPMSM.sUAlBeComp.f32Beta = 0;

    gsM1_Drive.sFocPMSM.sUAlBeReq.f32Alpha = 0;
    gsM1_Drive.sFocPMSM.sUAlBeReq.f32Beta = 0;

    gsM1_Drive.sFocPMSM.sUDcBusFilter.f32DummyVar = 0;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.f32FiltBufferX[0] = 0;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.f32FiltBufferX[1] = 0;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.f32FiltBufferY[0] = 0;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.f32FiltBufferY[1] = 0;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.udtFiltCoeff.f32A2 = 0;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.udtFiltCoeff.f32B1 = 0;
    gsM1_Drive.sFocPMSM.sUDcBusFilter.udtFiltCoeff.f32B2 = 0;

    gsM1_Drive.sFocPMSM.sUDQController.f32D = 0;
    gsM1_Drive.sFocPMSM.sUDQController.f32Q = 0;

    gsM1_Drive.sFocPMSM.sUDQReq.f32D = 0;
    gsM1_Drive.sFocPMSM.sUDQReq.f32Q = 0;

    gsM1_Drive.sFocPMSM.uw16SectorSVM = 0;

    gsM1_Drive.sFocPMSM.bOverModulation = 0;

    gsM1_Drive.sPositionEnc.f16PositionMechScale = 0;
    gsM1_Drive.sPositionEnc.f32Omega = 0;
    gsM1_Drive.sPositionEnc.f32PositionEl = 0;
    gsM1_Drive.sPositionEnc.f32PositionElUpdate = 0;
    gsM1_Drive.sPositionEnc.f32PositionMech = 0;
    gsM1_Drive.sPositionEnc.f32Speed = 0;
    gsM1_Drive.sPositionEnc.f32SpeedCoef = 0;
    gsM1_Drive.sPositionEnc.f32SpeedFilt = 0;
    gsM1_Drive.sPositionEnc.sAnglePosEl.f32Cos = 0;
    gsM1_Drive.sPositionEnc.sAnglePosEl.f32Sin = 0;
    gsM1_Drive.sPositionEnc.sAnglePosElUpdate.f32Cos = 0;
    gsM1_Drive.sPositionEnc.sAnglePosElUpdate.f32Sin = 0;
    gsM1_Drive.sPositionEnc.sSpeedFilter.f32DummyVar = 0;
    gsM1_Drive.sPositionEnc.sSpeedFilter.f32FiltBufferX[0] = 0;
    gsM1_Drive.sPositionEnc.sSpeedFilter.f32FiltBufferX[1] = 0;
    gsM1_Drive.sPositionEnc.sSpeedFilter.f32FiltBufferY[0] = 0;
    gsM1_Drive.sPositionEnc.sSpeedFilter.f32FiltBufferY[1] = 0;
    gsM1_Drive.sPositionEnc.sSpeedFilter.udtFiltCoeff.f32A2 = 0;
    gsM1_Drive.sPositionEnc.sSpeedFilter.udtFiltCoeff.f32B1 = 0;
    gsM1_Drive.sPositionEnc.sSpeedFilter.udtFiltCoeff.f32B2 = 0;
    gsM1_Drive.sPositionEnc.uw16NewPulse = 0;
    gsM1_Drive.sPositionEnc.uw16PositionMechRaw = 0;
    gsM1_Drive.sPositionEnc.uw16PositionMechRawMask = 0;
    gsM1_Drive.sPositionEnc.w16EncDifferenceMax = 0;
    gsM1_Drive.sPositionEnc.w16EncPulsesRevMech = ENCODER_PPR;
    gsM1_Drive.sPositionEnc.w16PolePairs = POLE_PAIR_NUM;
    gsM1_Drive.sPositionEnc.w16PositionMechScaleShift = 0;
    gsM1_Drive.sPositionEnc.s32TimerCaptured = 0;
    gsM1_Drive.sPositionEnc.s32TimerCapturedOld = 0;
    gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition = 0;
    gsM1_Drive.sPositionEnc.u32MagnetEncoderPositionPrev = 0;
    gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeed = 0;
    gsM1_Drive.sPositionEnc.u8MagnetEncoderErrCode = 0;
    gsM1_Drive.sPositionEnc.u32IndexPulseCaptured = 0;
    gsM1_Drive.sPositionEnc.u32IndexPulseCapturedOld = 0;
    gsM1_Drive.sPositionEnc.s32DeltaIndexPulseCaptured = 0;
    gsM1_Drive.sPositionEnc.s32PulseCaptured = TIM8->CNT;
    gsM1_Drive.sPositionEnc.s32PulseCapturedPre = gsM1_Drive.sPositionEnc.s32PulseCaptured;

    gsM1_Drive.sPositionEnc.s32EncCapturedOld = 0;
    gsM1_Drive.sPositionEnc.s32EncCaptured = 0;

    gsM1_Drive.sSpeed.bOpenLoop = 0;
    gsM1_Drive.sSpeed.f32Speed = 0;
    gsM1_Drive.sSpeed.f32SpeedCmd = 0;
    gsM1_Drive.sSpeed.f32SpeedCmdPre = 0;
    gsM1_Drive.sSpeed.f32SpeedStart = 0;
    gsM1_Drive.sSpeed.f32SpeedError = 0;
    gsM1_Drive.sSpeed.f32SpeedFilt = 0;
    gsM1_Drive.sSpeed.f32SpeedRamp = 0;
    gsM1_Drive.sSpeed.f32SpeedReq = 0;
    gsM1_Drive.sSpeed.i16SpeedPiSatFlag = 0;
    gsM1_Drive.sSpeed.f32SpeedRampStep = SPEED_RAMP_STEP_DEFAULT;

    gsM1_Drive.sSpeed.sSpeedFilter.f32DummyVar = 0;
    gsM1_Drive.sSpeed.sSpeedFilter.f32FiltBufferX[0] = 0;
    gsM1_Drive.sSpeed.sSpeedFilter.f32FiltBufferX[1] = 0;
    gsM1_Drive.sSpeed.sSpeedFilter.f32FiltBufferY[0] = 0;
    gsM1_Drive.sSpeed.sSpeedFilter.f32FiltBufferY[1] = 0;
    gsM1_Drive.sSpeed.sSpeedFilter.udtFiltCoeff.f32A2 = 0;
    gsM1_Drive.sSpeed.sSpeedFilter.udtFiltCoeff.f32B1 = 0;
    gsM1_Drive.sSpeed.sSpeedFilter.udtFiltCoeff.f32B2 = 0;

    gsM1_Drive.sSpeed.sSpeedPiParams.f32IntegGain = SPEED_KI;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32IntegPartK_1 = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32IntegPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32PropGain = SPEED_KP;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32UpperLimit = SPEEDLOOP_UPPER_LIMIT;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32LowerLimit = SPEEDLOOP_LOWER_LIMIT;
    gsM1_Drive.sSpeed.sSpeedPiParams.i16LimitFlag = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32ErrPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32OutputPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32SampleTime = SPEEDLOOP_PERIOD;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32DiffGain = SPEED_KD;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32DiffPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32ErrPartK_1 = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32AntiSatGain = 1.0f / gsM1_Drive.sSpeed.sSpeedPiParams.f32PropGain;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32AntiSatPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32FFPartK = 0;

    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32IntegGain = SPEED_KI;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32IntegPartK_1 = 0;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32IntegPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32PropGain = SPEED_KP;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32UpperLimit = SPEEDLOOP_UPPER_LIMIT;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32LowerLimit = SPEEDLOOP_LOWER_LIMIT;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.i16LimitFlag = 0;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32ErrPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32OutputPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32SampleTime = SPEEDLOOP_PERIOD;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32DiffGain = SPEED_KD;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32DiffPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32ErrPartK_1 = 0;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32AntiSatGain = 1.0f / gsM1_Drive.sSpeed.sSpeedPiParams.f32PropGain;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32AntiSatPartK = 0;
    gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32FFPartK = 0;

    gsM1_Drive.sSpeed.sSpeedRampParams.f32RampDown = SPEED_RAMP_DOWN;
    gsM1_Drive.sSpeed.sSpeedRampParams.f32RampUp = SPEED_RAMP_UP;
    gsM1_Drive.sSpeed.w16SpeedScale = SPEEDLOOP_SCALE;
    gsM1_Drive.sSpeed.w16SpeedScaleCnt = 0;
    gsM1_Drive.sSpeed.sSpeedPiParams.f32IntegScale = SPEEDLOOP_SCALE;

    gsM1_Drive.sSpeed.f32SpeedAcceleration = SPEED_MAX_ACCELERATION;
    gsM1_Drive.sSpeed.f32SpeedAccelerationSet = SPEED_MAX_ACCELERATION;
    gsM1_Drive.sSpeed.f32SpeedJerk = SPEED_JERK;
    gsM1_Drive.sSpeed.f32SpeedJerkSet = SPEED_JERK;

    gsM1_Drive.sSpeed.f32SpeedFF = 0;
    gsM1_Drive.sSpeed.f32SpeedFF_Pre = 0;

    gsM1_Drive.sPositionControl.bOpenLoop = 0;
    gsM1_Drive.sPositionControl.f32Position = 0;
    gsM1_Drive.sPositionControl.f32PositionComp = 0;
    gsM1_Drive.sPositionControl.f32PositionCompPre = gsM1_Drive.sPositionControl.f32PositionCompPre;
    gsM1_Drive.sPositionControl.f32PositionCmd = 0;
    gsM1_Drive.sPositionControl.f32PositionError = 0;
    gsM1_Drive.sPositionControl.f32PositionRampOut = 0;
    gsM1_Drive.sPositionControl.f32PositionRampOutFilt = 0;
    gsM1_Drive.sPositionControl.f32PositionRampStep = POSITION_RAMP_STEP;
    gsM1_Drive.sPositionControl.bPositionTargetChange = 0;
    gsM1_Drive.sPositionControl.f32PositionCmd = 0;
    gsM1_Drive.sPositionControl.f32PositionCmdPrevious = 0;
    gsM1_Drive.sPositionControl.f32StartInterval = START_INTERVAL;
    gsM1_Drive.sPositionControl.f32StopInterval = STOP_INTERVAL;
    gsM1_Drive.sPositionControl.f32AccelerationFF = 0;

    gsM1_Drive.sPositionControl.sPositionPiParams.f32ErrPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32IntegGain = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32IntegPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32IntegPartK_1 = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32LowerLimit = POSITIONLOOP_LOWER_LIMIT;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32OutputPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32PropGain = POSITIONLOOP_KP;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32UpperLimit = POSITIONLOOP_UPPER_LIMIT;
    gsM1_Drive.sPositionControl.sPositionPiParams.i16LimitFlag = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32DiffGain = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32DiffPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32ErrPartK_1 = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32SampleTime = SPEEDLOOP_PERIOD;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32AntiSatGain = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32AntiSatPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParams.f32FFPartK = 0;

    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32ErrPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32IntegGain = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32IntegPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32IntegPartK_1 = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32LowerLimit = POSITIONLOOP_LOWER_LIMIT;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32OutputPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32PropGain = POSITIONLOOP_KP;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32UpperLimit = POSITIONLOOP_UPPER_LIMIT;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.i16LimitFlag = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32DiffGain = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32DiffPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32ErrPartK_1 = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32SampleTime = SPEEDLOOP_PERIOD;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32AntiSatGain = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32AntiSatPartK = 0;
    gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32FFPartK = 0;

    gsM1_Drive.sPositionControl.sPositionFilter.f32FiltBufferM = 0;
    gsM1_Drive.sPositionControl.sPositionFilter.f32FiltBufferM_K_1 = 0;
    gsM1_Drive.sPositionControl.sPositionFilter.f32FiltBufferY = 0;
    gsM1_Drive.sPositionControl.sPositionFilter.f32FiltBufferY_K_1 = 0;
    gsM1_Drive.sPositionControl.sPositionFilter.f32Omega = POSITION_FILTER_OMEGA;

    gsM1_Drive.sInitLocating.f32Theta_H = PI * 2.0f;
    gsM1_Drive.sInitLocating.f32Theta_L = 0.0f;
    gsM1_Drive.sInitLocating.f32Theta_M = PI;
    gsM1_Drive.sInitLocating.f32AccuracyLimit = LOCATING_ACCURACY;
    gsM1_Drive.sInitLocating.f32LocatingCurrent = LOCATING_CURRENT;
    gsM1_Drive.sInitLocating.f32PositionMech = 0;
    gsM1_Drive.sInitLocating.f32PositionMechPre = 0;
    gsM1_Drive.sInitLocating.f32Theta_M_Pre = 0;
    gsM1_Drive.sInitLocating.f32Theta_Offset = 0;
    gsM1_Drive.sInitLocating.s32TurnsMech = 0;
    gsM1_Drive.sInitLocating.s32TurnsMechPre = 0;
    gsM1_Drive.sInitLocating.u16LocatingPrescaler = 0;
    gsM1_Drive.sInitLocating.u8LocatingCnt = 0;
    gsM1_Drive.sInitLocating.u8LocatingStep = 0;
    gsM1_Drive.sInitLocating.u32StartCnt = 0;
    gsM1_Drive.sInitLocating.u32LockCnt = 0;
    gsM1_Drive.sInitLocating.u32Delay = 0;
    gsM1_Drive.sInitLocating.u8BrakeCnt = 0;

#ifdef HFI
    gsM1_Drive.sHFISearch.sHFIPiParams.f32AntiSatGain = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32AntiSatPartK = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32DiffGain = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32DiffPartK = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32ErrPartK = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32ErrPartK_1 = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32FFPartK = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32IntegGain = 50.0f;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32IntegPartK = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32IntegPartK_1 = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32IntegScale = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32LowerLimit = -2.0f * PI;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32OutputPartK = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32PropGain = 0.0f;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32PropPartK = 0;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32SampleTime = SAMPLE_PERIOD;
    gsM1_Drive.sHFISearch.sHFIPiParams.f32UpperLimit = 2.0f * PI;
    gsM1_Drive.sHFISearch.sHFIPiParams.i16LimitFlag = 0;
    gsM1_Drive.sHFISearch.sSinFilter.f32Omega = IIR2_FREQ;
    gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferM = 0;
    gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferM_K_1 = 0;
    gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferY = 0;
    gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferY_K_1 = 0;
    gsM1_Drive.sHFISearch.sCosFilter.f32Omega = IIR2_FREQ;
    gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferM = 0;
    gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferM_K_1 = 0;
    gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferY = 0;
    gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferY_K_1 = 0;
    gsM1_Drive.sHFISearch.u32CtrlDelay = 0;
    gsM1_Drive.sHFISearch.u32Scale = 0;
    gsM1_Drive.sHFISearch.u8Step = 0;
    gsM1_Drive.sHFISearch.f32EstAngle = 0;
    gsM1_Drive.sHFISearch.f32IdcosFlt = 0;
    gsM1_Drive.sHFISearch.f32Idh = 0;
    gsM1_Drive.sHFISearch.f32IdhPre = 0;
    gsM1_Drive.sHFISearch.f32IdsinFlt = 0;
    gsM1_Drive.sHFISearch.f32IqAvg = 0;
    gsM1_Drive.sHFISearch.f32IqcosFlt = 0;
    gsM1_Drive.sHFISearch.f32Iqh = 0;
    gsM1_Drive.sHFISearch.f32IqsinFlt = 0;
    gsM1_Drive.sHFISearch.f32Offset = 0;
    gsM1_Drive.sHFISearch.f32OffsetFinal = 0;
    gsM1_Drive.sHFISearch.f32RadCnt = 0;
    gsM1_Drive.sHFISearch.f32ResponseCurrent[0] = 0;
    gsM1_Drive.sHFISearch.f32ResponseCurrent[1] = 0;
    gsM1_Drive.sHFISearch.f32ResponseCurrent[2] = 0;
    gsM1_Drive.sHFISearch.f32ResponseCurrent[3] = 0;
    gsM1_Drive.sHFISearch.f32ResponseCurrent[4] = 0;
    gsM1_Drive.sHFISearch.f32SearchingAngle[0] = 0;
    gsM1_Drive.sHFISearch.f32SearchingAngle[1] = 0;
    gsM1_Drive.sHFISearch.f32SearchingAngle[2] = 0;
    gsM1_Drive.sHFISearch.f32SearchingAngle[3] = 0;
    gsM1_Drive.sHFISearch.f32SearchingAngle[4] = 0;
    gsM1_Drive.sHFISearch.u16ResponseCurrentCnt = 0;
    //gsM1_Drive.sHFISearch.u32EstAngleCnt                 = 0;
    gsM1_Drive.sHFISearch.u32HFI_Duration = 0;
    gsM1_Drive.sHFISearch.u32IqAvgOverDuration = 0;
    gsM1_Drive.sHFISearch.u32IqAvgUnderDuration = 0;
    gsM1_Drive.sHFISearch.u8SearchingAngleCnt = 0;
#endif

#if AUTORUN == 1
    gsM1_Drive.uw16CtrlMode = SPEED_CONTROL;
#else
    gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;
#endif
}

u8 PositionCompDeltaCheck;

void Enc_Speed_Cal(void) {
    float MagPos;

    Enc_Speed_Cal_Pulse();

    Enc_Speed_Cal_Magnet();

#if MAGNET_ENCODER_FBK == 0
    gsM1_Drive.sPositionControl.f32Position = (float)gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition / MAGNET_ENCODER_BITS * 360.0f;
#else

    MagPos = (float)gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition / MAGNET_ENCODER_BITS * 360.0f + gsM1_Drive.sPositionControl.f32PositionLastCheckDelta;
    if (MagPos >= 360.0f)
        gsM1_Drive.sPositionControl.f32Position = MagPos - 360.0f;
    else if (MagPos < 0)
        gsM1_Drive.sPositionControl.f32Position = MagPos + 360.0f;
    else
        gsM1_Drive.sPositionControl.f32Position = MagPos;

#endif

#if MAGNET_ENCODER_FBK == 0
    gsM1_Drive.sPositionControl.f32PositionComp = gsM1_Drive.sPositionEnc.f32PositionMech;
#else
    if (gsM1_Drive.sPositionEnc.u8MagnetEncInitPositionCheck == 2) {
        if (gsM1_Drive.sPositionControl.f32Position - gsM1_Drive.sPositionControl.f32PositionPre < -180.0f)
            gsM1_Drive.sPositionControl.f32PositionTurns += 1.0f;
        else if (gsM1_Drive.sPositionControl.f32Position - gsM1_Drive.sPositionControl.f32PositionPre > 180.0f)
            gsM1_Drive.sPositionControl.f32PositionTurns -= 1.0f;

        gsM1_Drive.sPositionControl.f32PositionComp = gsM1_Drive.sPositionControl.f32Position + 360.0f * gsM1_Drive.sPositionControl.f32PositionTurns;

        gsM1_Drive.sPositionControl.f32PositionPre = gsM1_Drive.sPositionControl.f32Position;

        if (PositionCompDeltaCheck < 1) {
            PositionCompDeltaCheck++;
            gsM1_Drive.sPositionControl.f32PositionCompPre = gsM1_Drive.sPositionControl.f32PositionComp;
        } else {
            gsM1_Drive.sPositionControl.f32PositionCompPre = gsM1_Drive.sPositionControl.f32PositionComp;
        }
    }
#endif

    gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeed = gsM1_Drive.sPositionEnc.f32Speed / HARMONIC_AMPLIFY * 6.0f;

    gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeedFilt = gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeed;

    gsM1_Drive.sPositionEnc.f32MagnetEncoderAcceleration = (gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeedFilt - gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeedFiltPrev) / (SPEEDLOOP_PERIOD);

    gsM1_Drive.sPositionEnc.f32MagnetEncoderAccelerationFlt = gsM1_Drive.sPositionEnc.f32MagnetEncoderAcceleration;

    gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeedFiltPrev = gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeedFilt;
}

float speedmax, omegamax, wDelta_enc_max, speedmin;
u8 speedoverrun, speedoverruncnt;
s32 g_delta_enc = 0;
static int  g_last_in_speed_count = 0;

void Enc_Speed_Cal_Pulse(void) {
    //static s32 wDelta_enc;

    gsM1_Drive.sPositionEnc.s32PulseCaptured = TIM8->CNT;

    if (gsM1_Drive.sPositionEnc.s32PulseCaptured - gsM1_Drive.sPositionEnc.s32PulseCapturedPre > (2 * ENCODER_PPR)) {
        gsM1_Drive.sPositionEnc.s32EncoderTurns--;
    } else if (gsM1_Drive.sPositionEnc.s32PulseCaptured - gsM1_Drive.sPositionEnc.s32PulseCapturedPre < -(2 * ENCODER_PPR)) {
        gsM1_Drive.sPositionEnc.s32EncoderTurns++;
    }

    gsM1_Drive.sPositionEnc.s32PulseCapturedPre = gsM1_Drive.sPositionEnc.s32PulseCaptured;

    gsM1_Drive.sPositionEnc.s32EncCaptured = gsM1_Drive.sPositionEnc.s32EncoderTurns * ENCODER_PPR * 4 + gsM1_Drive.sPositionEnc.s32PulseCaptured;

    g_delta_enc = gsM1_Drive.sPositionEnc.s32EncCaptured - gsM1_Drive.sPositionEnc.s32EncCapturedOld;


    int _this_in_adc_time = SysTick->VAL;
    float _delta = 0.0f;
    if(g_last_in_speed_count > _this_in_adc_time)
    {
        _delta =  g_last_in_speed_count - _this_in_adc_time;
    }
    else
    {
        _delta = (1 << 24) - _this_in_adc_time + g_last_in_speed_count;
    }
    g_last_in_speed_count = _this_in_adc_time;
    float _delta_time = (float)_delta / 168000.0f;

    gsM1_Drive.sPositionEnc.f32Speed = ((float)(g_delta_enc) / ((float)ENCODER_PPR * 4.0f)) / SPEEDLOOP_PERIOD * 60.0f / _delta_time;

    gsM1_Drive.sPositionEnc.s32EncCapturedOld = gsM1_Drive.sPositionEnc.s32EncCaptured;

    gsM1_Drive.sPositionEnc.f32SpeedFilt = 0.05f * gsM1_Drive.sPositionEnc.f32SpeedFilt + 0.95f * gsM1_Drive.sPositionEnc.f32Speed;

    gsM1_Drive.sPositionEnc.f32Omega = gsM1_Drive.sPositionEnc.f32Speed * (float)gsM1_Drive.sPositionEnc.w16PolePairs * 2.0f * PI_CONST / 60.0f;

    gsM1_Drive.sSpeed.f32SpeedFilt = gsM1_Drive.sPositionEnc.f32SpeedFilt;
    gsM1_Drive.sSpeed.f32Speed = gsM1_Drive.sPositionEnc.f32Speed;

    gsM1_Drive.sPositionEnc.f32PositionMech = gsM1_Drive.sPositionControl.f32PositionMechnicalStart + (float)gsM1_Drive.sPositionEnc.s32EncoderTurns / HARMONIC_AMPLIFY * 360.0f + (float)TIM8->CNT / HARMONIC_AMPLIFY / (float)ENCODER_PPR * 90.0f;
}

unsigned char odd16_check(int p) {
    union {
        int BYTE;
        struct
        {
            unsigned char bit0 : 1;
            unsigned char bit1 : 1;
            unsigned char bit2 : 1;
            unsigned char bit3 : 1;
            unsigned char bit4 : 1;
            unsigned char bit5 : 1;
            unsigned char bit6 : 1;
            unsigned char bit7 : 1;
            unsigned char bit8 : 1;
            unsigned char bit9 : 1;
            unsigned char bit10 : 1;
            unsigned char bit11 : 1;
            unsigned char bit12 : 1;
            unsigned char bit13 : 1;
            unsigned char bit14 : 1;
            unsigned char bit15 : 1;
        } BIT;
    } odd16;

    odd16.BYTE = p;

    if (odd16.BIT.bit0 == (0x01 & (1 + odd16.BIT.bit1 + odd16.BIT.bit2 + odd16.BIT.bit3 + odd16.BIT.bit4 + odd16.BIT.bit5 + odd16.BIT.bit6 + odd16.BIT.bit7 +
                                   odd16.BIT.bit8 + odd16.BIT.bit9 + odd16.BIT.bit10 + odd16.BIT.bit11 + odd16.BIT.bit12 + odd16.BIT.bit13 + odd16.BIT.bit14 + odd16.BIT.bit15))) {
        return 0;
    } else {
        return 1;
    }
}

u8 CRC_CAL;
s32 Enc_Speed_Cal_Magnet(void) {
#if RENISHAW == 1

    static u32 Encoder_Position;
    static u32 General_Status;
    static u32 Detailed_Status;
    static u32 CRC_Value;
    static s32 wDelta_enc;

    CRC_Value = SPI3_Rx_Buff[4];

    Detailed_Status = ((SPI3_Rx_Buff[3] & 0xFC) >> 2) | ((SPI3_Rx_Buff[2] & 0x03) << 6);

    General_Status = (SPI3_Rx_Buff[2] & 0x0C) >> 2;

#if IS_17BITS_MAGNET_ENCODER == 1

    Encoder_Position = ((u32)(SPI3_Rx_Buff[2] & 0x80) >> 7) | (((u32)(SPI3_Rx_Buff[1] & 0xFF)) << 1) | (((u32)(SPI3_Rx_Buff[0] & 0xFF)) << 9);

#else

    Encoder_Position = ((u32)(SPI3_Rx_Buff[2] & 0xE0) >> 5) | (((u32)(SPI3_Rx_Buff[1] & 0xFF)) << 3) | (((u32)(SPI3_Rx_Buff[0] & 0xFF)) << 11);

#endif

    gsM1_Drive.sPositionEnc.u8MagnetEncoderErrCode |= Detailed_Status;

#if MAGNET_ENCODER_ALERT_ON == 1

    if (gsM1_Drive.sPositionEnc.u8MagnetEncInitPositionCheck == 1) {
        gsM1_Drive.sFaultId.B.MagnetEncoderError |= ((General_Status == 0) ? 0 : 1);
    } else {
        gsM1_Drive.sPositionEnc.u8MagnetEncoderGeneralError |= ((General_Status == 0) ? 0 : 1);

        if (gsM1_Drive.sPositionEnc.u8MagnetEncoderGeneralError == 1)
            gsM1_Drive.sFaultThresholds.u32MagnetEncoderErrorCnt++;
        else
            gsM1_Drive.sFaultThresholds.u32MagnetEncoderErrorCnt = 0;

        if (gsM1_Drive.sFaultThresholds.u32MagnetEncoderErrorCnt >= 4) {
            gsM1_Drive.sFaultThresholds.u32MagnetEncoderErrorCnt = 0;
            //gsM1_Drive.sFaultId.B.MagnetEncoderError = 1;
        }
    }

#endif

    CRC_CAL = crc8_4B(SPI3_Rx_Buff[3] | (SPI3_Rx_Buff[2] << 8) | (SPI3_Rx_Buff[1] << 16) | (SPI3_Rx_Buff[0] << 24));

    if ((crc8_4B(SPI3_Rx_Buff[3] | (SPI3_Rx_Buff[2] << 8) | (SPI3_Rx_Buff[1] << 16) | (SPI3_Rx_Buff[0] << 24)) == CRC_Value) && (General_Status == 0)) {
#if NEW_MAG_FIXTURE == 0
        gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition = (u32)MAGNET_ENCODER_BITS - Encoder_Position;
#else
        gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition = Encoder_Position;
#endif

        wDelta_enc = gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition - gsM1_Drive.sPositionEnc.u32MagnetEncoderPositionPrev;

        gsM1_Drive.sPositionEnc.u32MagnetEncoderPositionPrev = gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition;

        gsM1_Drive.sPositionEnc.u8MagnetEncoderUpdateFlag = 1;

        gsM1_Drive.sPositionEnc.u8MagnetEncoderCRCError = 0;

#if MAGNET_ENCODER_FBK == 1
        gsM1_Drive.sPositionControl.f32PositionLastCheck = gsM1_Drive.sPositionEnc.f32PositionMech;
        gsM1_Drive.sPositionControl.f32PositionLastCheckDelta = 0;

#endif

    } else {
#if MAGNET_ENCODER_FBK == 1
        gsM1_Drive.sPositionControl.f32PositionLastCheckDelta = gsM1_Drive.sPositionEnc.f32PositionMech - gsM1_Drive.sPositionControl.f32PositionLastCheck;
#endif

#if MAGNET_ENCODER_ALERT_ON == 1
        if (gsM1_Drive.sPositionEnc.u8MagnetEncInitPositionCheck == 1) {
            gsM1_Drive.sFaultId.B.MagnetEncoderCRCError |= 1;
        } else {
            if (crc8_4B(SPI3_Rx_Buff[3] | (SPI3_Rx_Buff[2] << 8) | (SPI3_Rx_Buff[1] << 16) | (SPI3_Rx_Buff[0] << 24)) != CRC_Value)
                gsM1_Drive.sPositionEnc.u8MagnetEncoderCRCError = 1;
            else
                gsM1_Drive.sPositionEnc.u8MagnetEncoderCRCError = 0;

            if (gsM1_Drive.sPositionEnc.u8MagnetEncoderCRCError == 1)
                gsM1_Drive.sFaultThresholds.u32MagnetEncoderCRCErrorCnt++;
            else
                gsM1_Drive.sFaultThresholds.u32MagnetEncoderCRCErrorCnt = 0;

            if (gsM1_Drive.sFaultThresholds.u32MagnetEncoderCRCErrorCnt >= 4) {
                gsM1_Drive.sFaultThresholds.u32MagnetEncoderCRCErrorCnt = 0;
                //gsM1_Drive.sFaultId.B.MagnetEncoderCRCError = 1;
            }
        }

#endif
    }

    return wDelta_enc;

#else

    static s16 wDelta_enc = 0;

    gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition = SACE_SPI3_Rx_Buff;

    if (gsM1_Drive.sSpeed.f32Speed == 0) {
        if (magnet_encoder_delta_max < gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition)
            magnet_encoder_delta_max = gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition;

        if (magnet_encoder_delta_min > gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition)
            magnet_encoder_delta_min = gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition;
    } else {
        magnet_encoder_delta_min = gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition;
        magnet_encoder_delta_max = gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition;
    }

    /****************************************************/

#if MAGNET_ENCODER_ALERT_ON == 1

    gsM1_Drive.sFaultId.B.MagnetEncoderError |= odd16_check((s16)gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition);

#endif

    wDelta_enc = gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition - gsM1_Drive.sPositionEnc.u32MagnetEncoderPositionPrev;

    gsM1_Drive.sPositionEnc.u32MagnetEncoderPositionPrev = gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition;

    return wDelta_enc;

#endif
}

void Rotor_Alignment(void) {
    static float wTimebase = 0;

    gsM1_Drive.sFocPMSM.sAnglePosEl.f32Cos = arm_cos_f32(gsM1_Drive.sFocPMSM.sAlignment.f32Position);
    gsM1_Drive.sFocPMSM.sAnglePosEl.f32Sin = arm_sin_f32(gsM1_Drive.sFocPMSM.sAlignment.f32Position);

    wTimebase++;

    if (wTimebase < T_ALIGNMENT_PWM_STEPS) {
        gsM1_Drive.sFocPMSM.sIDQReq.f32D = DEFAULT_ALIGNMENT_CURRENT * wTimebase / (float)T_ALIGNMENT_PWM_STEPS;
        gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

        gsM1_Drive.sFocPMSM.sIAlBe = Motor_Drive_Clarke(gsM1_Drive.sFocPMSM.sIABC);
        gsM1_Drive.sFocPMSM.sIDQ = Motor_Drive_Park(gsM1_Drive.sFocPMSM.sIAlBe, gsM1_Drive.sFocPMSM.sAnglePosEl);

        gsM1_Drive.sFocPMSM.sUDQReq.f32Q = Motor_Drive_Cur_PID_Regulator(gsM1_Drive.sFocPMSM.sIDQReq.f32Q, gsM1_Drive.sFocPMSM.sIDQ.f32Q, &gsM1_Drive.sFocPMSM.sIqPiParams);

        gsM1_Drive.sFocPMSM.sUDQReq.f32D = Motor_Drive_Cur_PID_Regulator(gsM1_Drive.sFocPMSM.sIDQReq.f32D, gsM1_Drive.sFocPMSM.sIDQ.f32D, &gsM1_Drive.sFocPMSM.sIdPiParams);

        gsM1_Drive.sFocPMSM.sUAlBeReq = Motor_Drive_Rev_Park(gsM1_Drive.sFocPMSM.sUDQReq, gsM1_Drive.sFocPMSM.sAnglePosEl);

        Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);
    } else {
        wTimebase = 0;
        Encoder_Reset();

        gsM1_Drive.sFocPMSM.sUAlBeReq.f32Alpha = 0;
        gsM1_Drive.sFocPMSM.sUAlBeReq.f32Beta = 0;

        Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

        gsM1_Drive.sFocPMSM.sIDQReq.f32D = 0;
        gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

        AlignmentDone = 1;
    }
}

void Encoder_Reset(void) {
    gsM1_Drive.sPositionEnc.s32EncCapturedOld = 0;
    gsM1_Drive.sPositionEnc.s32EncCaptured = 0;

    gsM1_Drive.sPositionEnc.s32EncoderTurns = 0;

    gsM1_Drive.sPositionEnc.f32PositionEl = 0;
    gsM1_Drive.sPositionEnc.f32PositionElUpdate = 0;

    TIM8->CNT = 0;
    gsM1_Drive.sPositionEnc.s32PulseCaptured = 0;
    gsM1_Drive.sPositionEnc.s32PulseCapturedPre = 0;
}

MCLIB_2_COOR_SYST_ALPHA_BETA_T Motor_Drive_Clarke(MCLIB_3_COOR_SYST_T I_abc) {
    MCLIB_2_COOR_SYST_ALPHA_BETA_T I_alpha_beta;

    I_alpha_beta.f32Alpha = I_abc.f32A;

    I_alpha_beta.f32Beta = -(2.0f * I_abc.f32B + I_abc.f32A) / SQRT3;

    return (I_alpha_beta);
}

MCLIB_2_COOR_SYST_D_Q_T Motor_Drive_Park(MCLIB_2_COOR_SYST_ALPHA_BETA_T I_alpha_beta, MCLIB_ANGLE_T Vector) {
    MCLIB_2_COOR_SYST_D_Q_T Curr_Output;

    Curr_Output.f32D = I_alpha_beta.f32Alpha * Vector.f32Sin + I_alpha_beta.f32Beta * Vector.f32Cos;

    Curr_Output.f32Q = I_alpha_beta.f32Alpha * Vector.f32Cos - I_alpha_beta.f32Beta * Vector.f32Sin;

    return (Curr_Output);
}

MCLIB_2_COOR_SYST_ALPHA_BETA_T Motor_Drive_Rev_Park(MCLIB_2_COOR_SYST_D_Q_T V_DQ, MCLIB_ANGLE_T Vector) {
    MCLIB_2_COOR_SYST_ALPHA_BETA_T Volt_Output;

    Volt_Output.f32Alpha = V_DQ.f32Q * Vector.f32Cos + V_DQ.f32D * Vector.f32Sin;

    Volt_Output.f32Beta = -V_DQ.f32Q * Vector.f32Sin + V_DQ.f32D * Vector.f32Cos;

    return (Volt_Output);
}

float Motor_Drive_Cur_PID_Regulator(float hReference, float hPresentFeedback, GFLIB_CONTROLLER_PI_P_PARAMS_T *PID_Struct) {
    static float Integral_Term;

    PID_Struct->f32ErrPartK = hReference - hPresentFeedback;

    PID_Struct->f32PropPartK = PID_Struct->f32PropGain * PID_Struct->f32ErrPartK;

    PID_Struct->f32DiffPartK = PID_Struct->f32DiffGain * (PID_Struct->f32ErrPartK - PID_Struct->f32ErrPartK_1) * PID_Struct->f32SampleFreq;

    PID_Struct->f32ErrPartK_1 = PID_Struct->f32ErrPartK;

    if ((PID_Struct->f32IntegGain == 0) || (PID_Struct->i16LimitFlag == 1)) {
        PID_Struct->f32IntegPartK = 0;
        PID_Struct->f32IntegPartK_1 = PID_Struct->f32IntegPartK;
    } else {
        Integral_Term = PID_Struct->f32ErrPartK * PID_Struct->f32IntegGain * PID_Struct->f32SampleTime;

        PID_Struct->f32IntegPartK = PID_Struct->f32IntegPartK_1 + Integral_Term;

        PID_Struct->f32IntegPartK_1 = PID_Struct->f32IntegPartK;
    }

    PID_Struct->f32OutputPartK = PID_Struct->f32PropPartK + PID_Struct->f32IntegPartK + PID_Struct->f32DiffPartK + PID_Struct->f32FFPartK;

    if (PID_Struct->f32OutputPartK > PID_Struct->f32UpperLimit) {
        PID_Struct->i16LimitFlag = 1;
        return (PID_Struct->f32UpperLimit);
    } else if (PID_Struct->f32OutputPartK < PID_Struct->f32LowerLimit) {
        PID_Struct->i16LimitFlag = 1;
        return (PID_Struct->f32LowerLimit);
    } else {
        PID_Struct->i16LimitFlag = 0;
        return (PID_Struct->f32OutputPartK);
    }
}

float Motor_Drive_Spd_PID_Regulator(float hReference, float hPresentFeedback, GFLIB_CONTROLLER_PI_P_PARAMS_T *PID_Struct) {
    static float Integral_Term;

    PID_Struct->f32ErrPartK = hReference - hPresentFeedback;

    PID_Struct->f32PropPartK = PID_Struct->f32PropGain * PID_Struct->f32ErrPartK;

    PID_Struct->f32DiffPartK = PID_Struct->f32DiffGain * (PID_Struct->f32ErrPartK - PID_Struct->f32ErrPartK_1) / PID_Struct->f32SampleTime;

    PID_Struct->f32ErrPartK_1 = PID_Struct->f32ErrPartK;

    if ((PID_Struct->f32IntegGain == 0) || (PID_Struct->i16LimitFlag == 1)) {
        PID_Struct->f32IntegPartK = 0;
        PID_Struct->f32IntegPartK_1 = PID_Struct->f32IntegPartK;
    } else {
        Integral_Term = PID_Struct->f32ErrPartK * PID_Struct->f32IntegGain * PID_Struct->f32SampleTime;

        PID_Struct->f32IntegPartK = PID_Struct->f32IntegPartK_1 + Integral_Term;

        PID_Struct->f32IntegPartK_1 = PID_Struct->f32IntegPartK;
    }

    PID_Struct->f32OutputPartK = PID_Struct->f32PropPartK + PID_Struct->f32IntegPartK + PID_Struct->f32DiffPartK + PID_Struct->f32FFPartK;

    if (PID_Struct->f32OutputPartK > PID_Struct->f32UpperLimit) {
        PID_Struct->i16LimitFlag = 1;
        return (PID_Struct->f32UpperLimit);
    } else if (PID_Struct->f32OutputPartK < PID_Struct->f32LowerLimit) {
        PID_Struct->i16LimitFlag = 1;
        return (PID_Struct->f32LowerLimit);
    } else {
        PID_Struct->i16LimitFlag = 0;
        return (PID_Struct->f32OutputPartK);
    }
}

float Motor_Drive_Pos_PID_Regulator(float hReference, float hPresentFeedback, GFLIB_CONTROLLER_PI_P_PARAMS_T *PID_Struct) {
    static float Integral_Term;

    PID_Struct->f32ErrPartK = hReference - hPresentFeedback;

    PID_Struct->f32PropPartK = PID_Struct->f32PropGain * PID_Struct->f32ErrPartK;

    PID_Struct->f32DiffPartK = PID_Struct->f32DiffGain * (PID_Struct->f32ErrPartK - PID_Struct->f32ErrPartK_1) / PID_Struct->f32SampleTime;

    PID_Struct->f32ErrPartK_1 = PID_Struct->f32ErrPartK;

    if ((PID_Struct->f32IntegGain == 0) || (PID_Struct->i16LimitFlag == 1)) {
        PID_Struct->f32IntegPartK = 0;
        PID_Struct->f32IntegPartK_1 = PID_Struct->f32IntegPartK;
    } else {
        Integral_Term = PID_Struct->f32ErrPartK * PID_Struct->f32IntegGain * PID_Struct->f32SampleTime;

        PID_Struct->f32IntegPartK = PID_Struct->f32IntegPartK_1 + Integral_Term;

        PID_Struct->f32IntegPartK_1 = PID_Struct->f32IntegPartK;
    }

    PID_Struct->f32OutputPartK = PID_Struct->f32PropPartK + PID_Struct->f32IntegPartK + PID_Struct->f32DiffPartK + PID_Struct->f32FFPartK;

    if (PID_Struct->f32OutputPartK > PID_Struct->f32UpperLimit) {
        PID_Struct->i16LimitFlag = 1;
        return (PID_Struct->f32UpperLimit);
    } else if (PID_Struct->f32OutputPartK < PID_Struct->f32LowerLimit) {
        PID_Struct->i16LimitFlag = 1;
        return (PID_Struct->f32LowerLimit);
    } else {
        PID_Struct->i16LimitFlag = 0;
        return (PID_Struct->f32OutputPartK);
    }
}

void Motor_Drive_DutyCycleCal(MCLIB_2_COOR_SYST_ALPHA_BETA_T Stat_Volt_Input, float Udc) {
    static float x, y, z, max, min, dutyA, dutyB, dutyC, temp, Ta, Tb, Tc;

    x = Stat_Volt_Input.f32Alpha / Udc;
    y = (-0.5f * Stat_Volt_Input.f32Alpha - 0.5f * SQRT3 * Stat_Volt_Input.f32Beta) / Udc;
    z = (-0.5f * Stat_Volt_Input.f32Alpha + 0.5f * SQRT3 * Stat_Volt_Input.f32Beta) / Udc;

    max = fmaxf(x, fmaxf(y, z));
    min = fminf(x, fminf(y, z));
    temp = 0.5f * (max + min);

    if (max - min > 2.0f * DEFAULT_DUTY_CYCLE_LIMIT - 1.0f) {
        dutyA = (x - temp) * (2.0f * DEFAULT_DUTY_CYCLE_LIMIT - 1.0f) / (max - min) + 0.5f;
        dutyB = (y - temp) * (2.0f * DEFAULT_DUTY_CYCLE_LIMIT - 1.0f) / (max - min) + 0.5f;
        dutyC = (z - temp) * (2.0f * DEFAULT_DUTY_CYCLE_LIMIT - 1.0f) / (max - min) + 0.5f;
    } else {
        dutyA = (x - temp) + 0.5f;
        dutyB = (y - temp) + 0.5f;
        dutyC = (z - temp) + 0.5f;
    }

    Ta = (float)PWM_PERIOD * dutyA;
    Tb = (float)PWM_PERIOD * dutyB;
    Tc = (float)PWM_PERIOD * dutyC;

    TIM1->CCR1 = (u16)Ta;
    TIM1->CCR2 = (u16)Tb;
    TIM1->CCR3 = (u16)Tc;

    gsM1_Drive.sFocPMSM.sDutyABC.f32A = dutyA;
    gsM1_Drive.sFocPMSM.sDutyABC.f32B = dutyB;
    gsM1_Drive.sFocPMSM.sDutyABC.f32C = dutyC;
}

void Motor_Drive_FOC(void) {
    gsM1_Drive.sFocPMSM.sIAlBe = Motor_Drive_Clarke(gsM1_Drive.sFocPMSM.sIABC);
    gsM1_Drive.sFocPMSM.sIDQ = Motor_Drive_Park(gsM1_Drive.sFocPMSM.sIAlBe, gsM1_Drive.sFocPMSM.sAnglePosEl);

    if (gsM1_Drive.uw16CtrlMode != OPENLOOP_PWM) {
        gsM1_Drive.sFocPMSM.sUDQReq.f32D = Motor_Drive_Cur_PID_Regulator(gsM1_Drive.sFocPMSM.sIDQReq.f32D, gsM1_Drive.sFocPMSM.sIDQ.f32D, &gsM1_Drive.sFocPMSM.sIdPiParams);

        gsM1_Drive.sFocPMSM.sUDQController.f32D = gsM1_Drive.sFocPMSM.sUDQReq.f32D;
    }

    if (gsM1_Drive.uw16CtrlMode != OPENLOOP_PWM) {
        gsM1_Drive.sFocPMSM.sUDQReq.f32Q = Motor_Drive_Cur_PID_Regulator(gsM1_Drive.sFocPMSM.sIDQReq.f32Q, gsM1_Drive.sFocPMSM.sIDQ.f32Q, &gsM1_Drive.sFocPMSM.sIqPiParams);

        gsM1_Drive.sFocPMSM.sUDQController.f32Q = gsM1_Drive.sFocPMSM.sUDQReq.f32Q;
    }

    gsM1_Drive.sFocPMSM.sUAlBeReq = Motor_Drive_Rev_Park(gsM1_Drive.sFocPMSM.sUDQController, gsM1_Drive.sFocPMSM.sAnglePosEl);

    if (gsM1_Drive.uw16CtrlMode != OPENLOOP_PWM) {
        Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);
    } else {
        Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, 48.0f);
    }
}

void Motor_Drive_Get_Electrical_Angle(void) {
    gsM1_Drive.sPositionEnc.f32PositionEl = fmodf((float)(TIM8->CNT) / 2.0f / (float)ENCODER_PPR * (float)POLE_PAIR_NUM * PI + gsM1_Drive.sPositionEnc.f32MagnetOffsetRadEl, PI * 2.0f);
    gsM1_Drive.sFocPMSM.sAnglePosEl.f32Cos = arm_cos_f32(gsM1_Drive.sPositionEnc.f32PositionEl);
    gsM1_Drive.sFocPMSM.sAnglePosEl.f32Sin = arm_sin_f32(gsM1_Drive.sPositionEnc.f32PositionEl);
}

void Current_Offset_Adjust(void) {
    gsM1_Drive.sADCOffset.f32PhA = 0.95f * gsM1_Drive.sADCOffset.f32PhA + 0.05f * (float)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    gsM1_Drive.sADCOffset.f32PhB = 0.95f * gsM1_Drive.sADCOffset.f32PhB + 0.05f * (float)ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
}

void RampControl(float cmd, float step, float period, float *out) {
    if (cmd > 0) {
        if (*out < cmd - step * period) {
            *out += step * period;
        } else if (*out > cmd + step * period) {
            *out -= step * period;
        } else {
            *out = cmd;
        }
    } else {
        if (*out > cmd + step * period) {
            *out -= step * period;
        } else if (*out < cmd - step * period) {
            *out += step * period;
        } else {
            *out = cmd;
        }
    }
}

void PositionCtrlInit(void) {
    static u16 inc;
    static u32 u32MagnetEncoderPosition_AVG;
    float f32ME_Position = 0;

    gsM1_Drive.sPositionEnc.u8MagnetEncInitPositionCheck = 1;

    while (inc < 512) {
        if (gsM1_Drive.sPositionEnc.u8MagnetEncoderUpdateFlag == 1) {
            u32MagnetEncoderPosition_AVG += gsM1_Drive.sPositionEnc.u32MagnetEncoderPosition;
            gsM1_Drive.sPositionEnc.u8MagnetEncoderUpdateFlag = 0;
            inc++;
        }

        if ((gsM1_Drive.sFaultId.B.MagnetEncoderError != 0) || (gsM1_Drive.sFaultId.B.MagnetEncoderCRCError != 0))
            break;
    }

    if ((gsM1_Drive.sFaultId.B.MagnetEncoderError == 0) && (gsM1_Drive.sFaultId.B.MagnetEncoderCRCError == 0)) {
        u32MagnetEncoderPosition_AVG = u32MagnetEncoderPosition_AVG >> 9;
    } else {
        u32MagnetEncoderPosition_AVG = 0;
    }

    f32ME_Position = (float)u32MagnetEncoderPosition_AVG / MAGNET_ENCODER_BITS * 360.0f;

    gsM1_Drive.sPositionControl.f32PositionCmd = f32ME_Position;
    gsM1_Drive.sPositionControl.f32PositionCmdPrevious = f32ME_Position;
    gsM1_Drive.sPositionControl.f32PositionComp = f32ME_Position;
    gsM1_Drive.sPositionControl.f32PositionMechnicalStart = f32ME_Position;
    gsM1_Drive.sPositionControl.f32Position = f32ME_Position;
    gsM1_Drive.sPositionControl.f32PositionPre = f32ME_Position;

    gsM1_Drive.sPositionEnc.u8MagnetEncInitPositionCheck = 2;
}

#if INIT_LOCATING == 1
float Position1;
float Position2;
float Position3;

void InitPositionLocating(void) {
#if STUCK_CHECK == 1
    if (gsM1_Drive.sInitLocating.u8StartDone == 0)
        gsM1_Drive.sInitLocating.u32StartCnt++;
    else
        gsM1_Drive.sInitLocating.u32StartCnt = 0;

    if (gsM1_Drive.sInitLocating.u32StartCnt >= 300000) {
        gsM1_Drive.sFaultId.B.StartUpFail = 1;
    }

    if ((fabsf(gsM1_Drive.sPositionEnc.f32SpeedFilt) <= 0.1f) && (fabsf(gsM1_Drive.sFocPMSM.sIDQ.f32Q) >= 0.3f * OVER_CURRENT_THRESHOLD) && (gsM1_Drive.sInitLocating.u8BrakeStuck == 0)) {
        gsM1_Drive.sInitLocating.u32LockCnt++;
    } else {
        gsM1_Drive.sInitLocating.u32LockCnt = 0;
    }
    if (gsM1_Drive.sInitLocating.u32LockCnt >= 10) {
        gsM1_Drive.sInitLocating.u8BrakeStuck = 1;
        gsM1_Drive.sInitLocating.f32StuckPosition = gsM1_Drive.sPositionEnc.f32PositionMech;
    }

#endif

    switch (gsM1_Drive.sInitLocating.u8LocatingStep) {
        case 0:
            PWM_OUT_DISABLE();
            TIM1->CCER |= 0x111;
            gsM1_Drive.sInitLocating.u8LocatingStep = 1;
            break;

        case 1:
            gsM1_Drive.sInitLocating.u8LocatingStep = 2;
            break;

        case 2:
            gsM1_Drive.sInitLocating.u8LocatingStep = 3;
            break;

        case 3:
            gsM1_Drive.sInitLocating.u8LocatingStep = 4;
            break;

        case 4:
            gsM1_Drive.sInitLocating.u8LocatingStep = 5;

            break;

        case 5:
            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;
            if ((gsM1_Ctrl.eState == RUN) && (meM1_StateRun == READY) && (gsM1_Drive.sInitLocating.u8BrakeFlag == 0)) {
                PWM_OUT_ENABLE();

                Brake_Off();
                TB_Set_Delay(100);
                gsM1_Drive.sInitLocating.u8BrakeFlag = 1;
            }

            if (TB_delay_IsElapsed() && (gsM1_Drive.sInitLocating.u8BrakeFlag == 1)) {
                Brake_Hold();
                gsM1_Drive.sInitLocating.u8LocatingStep = 6;
                gsM1_Drive.sInitLocating.u8BrakeFlag = 0;
            }

            break;

        case 6:
            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;
            gsM1_Drive.sInitLocating.f32PositionMech = TIM8->CNT;
            gsM1_Drive.sInitLocating.s32TurnsMech = gsM1_Drive.sPositionEnc.s32EncoderTurns;

            gsM1_Drive.sInitLocating.u8LocatingCnt++;

            if (gsM1_Drive.sInitLocating.u8LocatingCnt > 15)
                gsM1_Drive.sFaultId.B.StartUpFail = 1;

            gsM1_Drive.sInitLocating.u8LocatingStep = 7;
            break;

        case 7:
            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;
            if (gsM1_Drive.sInitLocating.u16LocatingPrescaler < (LOCATING_DURATION + HOLDING_DURATION)) {
                gsM1_Drive.sInitLocating.u16LocatingPrescaler++;

                if (gsM1_Drive.sInitLocating.u8LocatingCnt == 1) {
                    gsM1_Drive.sFocPMSM.sIDQReq.f32D = gsM1_Drive.sInitLocating.f32LocatingCurrent * (float)gsM1_Drive.sInitLocating.u16LocatingPrescaler / (float)(LOCATING_DURATION + HOLDING_DURATION);
                } else {
                    gsM1_Drive.sFocPMSM.sIDQReq.f32D = gsM1_Drive.sInitLocating.f32LocatingCurrent;
                }

                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

                if (gsM1_Drive.sInitLocating.u8LocatingCnt == 1) {
                    gsM1_Drive.sPositionEnc.f32PositionEl = gsM1_Drive.sInitLocating.f32Theta_M;
                } else {
                    if (gsM1_Drive.sInitLocating.u16LocatingPrescaler <= LOCATING_DURATION)
                        gsM1_Drive.sPositionEnc.f32PositionEl = gsM1_Drive.sInitLocating.f32Theta_M_Pre + gsM1_Drive.sInitLocating.f32Theta_M_Delta * gsM1_Drive.sInitLocating.u16LocatingPrescaler / LOCATING_DURATION;
                    else
                        gsM1_Drive.sPositionEnc.f32PositionEl = gsM1_Drive.sInitLocating.f32Theta_M;
                }

                FOC_Cal();

            } else {
                gsM1_Drive.sInitLocating.u16LocatingPrescaler = 0;
                gsM1_Drive.sInitLocating.u8LocatingStep = 8;
            }

            break;

        case 8:
            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;

            gsM1_Drive.sInitLocating.f32PositionMechPre = gsM1_Drive.sInitLocating.f32PositionMech;
            gsM1_Drive.sInitLocating.s32TurnsMechPre = gsM1_Drive.sInitLocating.s32TurnsMech;

            gsM1_Drive.sInitLocating.f32PositionMech = TIM8->CNT;
            gsM1_Drive.sInitLocating.s32TurnsMech = gsM1_Drive.sPositionEnc.s32EncoderTurns;

            gsM1_Drive.sInitLocating.f32Theta_Offset = (gsM1_Drive.sInitLocating.s32TurnsMech * ENCODER_PPR * 4.0f + gsM1_Drive.sInitLocating.f32PositionMech -
                                                        gsM1_Drive.sInitLocating.s32TurnsMechPre * ENCODER_PPR * 4.0f - gsM1_Drive.sInitLocating.f32PositionMechPre) /
                                                       ENCODER_PPR / 4.0f * 2.0f * PI * POLE_PAIR_NUM;

            gsM1_Drive.sInitLocating.f32Theta_M_Pre = gsM1_Drive.sInitLocating.f32Theta_M;

            gsM1_Drive.sInitLocating.f32Theta_H += gsM1_Drive.sInitLocating.f32Theta_Offset;
            gsM1_Drive.sInitLocating.f32Theta_L += gsM1_Drive.sInitLocating.f32Theta_Offset;
            gsM1_Drive.sInitLocating.f32Theta_M += gsM1_Drive.sInitLocating.f32Theta_Offset;

            if (gsM1_Drive.sInitLocating.f32Theta_Offset >= 0) {
                gsM1_Drive.sInitLocating.f32Theta_H = gsM1_Drive.sInitLocating.f32Theta_M;
            } else {
                gsM1_Drive.sInitLocating.f32Theta_L = gsM1_Drive.sInitLocating.f32Theta_M;
            }

            gsM1_Drive.sInitLocating.f32Theta_M = 0.5f * (gsM1_Drive.sInitLocating.f32Theta_H + gsM1_Drive.sInitLocating.f32Theta_L);

            gsM1_Drive.sInitLocating.f32Theta_M_Delta = gsM1_Drive.sInitLocating.f32Theta_M - gsM1_Drive.sInitLocating.f32Theta_M_Pre;

            if ((fabsf((gsM1_Drive.sInitLocating.f32Theta_M_Pre - gsM1_Drive.sInitLocating.f32Theta_M) / POLE_PAIR_NUM) > gsM1_Drive.sInitLocating.f32AccuracyLimit) || (gsM1_Drive.sInitLocating.u8LocatingCnt <= ITERATION_TIMES))
                gsM1_Drive.sInitLocating.u8LocatingStep = 6;
            else {
                gsM1_Drive.sInitLocating.u8LocatingStep = 9;
                gsM1_Drive.sInitLocating.f32PositionMech = TIM8->CNT;
                gsM1_Drive.sInitLocating.s32TurnsMech = gsM1_Drive.sPositionEnc.s32EncoderTurns;
                gsM1_Drive.sInitLocating.f32PositionMechPre = gsM1_Drive.sInitLocating.f32PositionMech;
                gsM1_Drive.sInitLocating.s32TurnsMechPre = gsM1_Drive.sInitLocating.s32TurnsMech;

#if STUCK_CHECK == 1

                gsM1_Drive.sPositionControl.f32PositionMechnicalStart = gsM1_Drive.sPositionEnc.f32PositionMech;
                gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionEnc.f32PositionMech;
                gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionEnc.f32PositionMech;
                gsM1_Drive.sPositionControl.f32PositionComp = gsM1_Drive.sPositionEnc.f32PositionMech;

                gsM1_Drive.sPositionEnc.f32MagnetOffsetRadEl = fmodf(gsM1_Drive.sPositionEnc.f32PositionEl, 2.0f * PI);

                Encoder_Reset();
                gsM1_Drive.sFocPMSM.sIDQReq.f32D = 0;
                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

#endif
            }

            Position1 = gsM1_Drive.sInitLocating.f32Theta_M;

            break;

        case 9:

#if STUCK_CHECK == 1

            if (gsM1_Drive.sInitLocating.u8BrakeStuck == 0) {
                if ((gsM1_Drive.sPositionEnc.f32PositionMech < gsM1_Drive.sPositionControl.f32PositionMechnicalStart + 120.0f / HARMONIC_AMPLIFY) && (gsM1_Drive.sInitLocating.u8OffsetPosition1Get == 0)) {
                    gsM1_Drive.sPositionControl.f32PositionCmd += 120.0f / HARMONIC_AMPLIFY / 2000.0f;
                    gsM1_Drive.sInitLocating.u8LocatingStep = 10;
                } else {
                    gsM1_Drive.sInitLocating.u8OffsetPosition1Get = 1;
                    if (gsM1_Drive.sPositionEnc.f32PositionMech > gsM1_Drive.sPositionControl.f32PositionMechnicalStart) {
                        gsM1_Drive.sPositionControl.f32PositionCmd -= 120.0f / HARMONIC_AMPLIFY / 2000.0f;
                        gsM1_Drive.sInitLocating.u8LocatingStep = 10;
                    } else {
                        gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionControl.f32PositionMechnicalStart;
                        gsM1_Drive.sInitLocating.u8OffsetPosition1Get = 0;
                        gsM1_Drive.sInitLocating.u8LocatingStep = 13;
                    }
                }

            } else {
                if (gsM1_Drive.sPositionEnc.f32PositionMech > gsM1_Drive.sInitLocating.f32StuckPosition - 5.0f / HARMONIC_AMPLIFY) {
                    gsM1_Drive.sPositionControl.f32PositionCmd -= 120.0f / HARMONIC_AMPLIFY / 2000.0f;
                    gsM1_Drive.sInitLocating.u8LocatingStep = 10;
                } else {
                    gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sInitLocating.f32StuckPosition - 5.0f / HARMONIC_AMPLIFY;
                    gsM1_Drive.sInitLocating.u8LocatingStep = 11;  //brake off
                    gsM1_Drive.sInitLocating.u8BrakeStuck = 0;
                }
            }

            break;

#else

            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;

            if (gsM1_Drive.sInitLocating.u16LocatingPrescaler < (BRAKING_DURATION + HOLDING_DURATION)) {
                gsM1_Drive.sInitLocating.u16LocatingPrescaler++;

                gsM1_Drive.sFocPMSM.sIDQReq.f32D = gsM1_Drive.sInitLocating.f32LocatingCurrent;
                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

                if (gsM1_Drive.sInitLocating.u16LocatingPrescaler <= BRAKING_DURATION)
                    gsM1_Drive.sPositionEnc.f32PositionEl = gsM1_Drive.sInitLocating.f32Theta_M + (float)gsM1_Drive.sInitLocating.u16LocatingPrescaler / (float)BRAKING_DURATION * LOCATING_OFFSET * (float)POLE_PAIR_NUM;
                else
                    gsM1_Drive.sPositionEnc.f32PositionEl = gsM1_Drive.sInitLocating.f32Theta_M + LOCATING_OFFSET * (float)POLE_PAIR_NUM;

                Position2 = gsM1_Drive.sPositionEnc.f32PositionEl;

                FOC_Cal();

            } else {
                gsM1_Drive.sInitLocating.u16LocatingPrescaler = 0;
                gsM1_Drive.sInitLocating.u8LocatingStep = 10;

                Brake_Off();
                TB_Set_Delay(100);
            }
            break;

#endif

        case 10:

#if STUCK_CHECK == 1
            if (gsM1_Drive.sInitLocating.u32Delay < 19) {
                gsM1_Drive.sInitLocating.u32Delay++;

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            } else {
                gsM1_Drive.sInitLocating.u32Delay = 0;

                gsM1_Drive.sPositionControl.f32PositionComp = gsM1_Drive.sPositionEnc.f32PositionMech;

                if (gsM1_Drive.sPositionControl.f32PositionCmd != gsM1_Drive.sPositionControl.f32PositionCmdPrevious) {
                    gsM1_Drive.sPositionControl.bPositionTargetChange = 1;
                    gsM1_Drive.sPositionControl.f32PositionStart = gsM1_Drive.sPositionControl.f32PositionComp;
                    gsM1_Drive.sPositionControl.f32PositionRampOut = gsM1_Drive.sPositionControl.f32PositionStart;
                    gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionControl.f32PositionCmd;
                } else {
                    gsM1_Drive.sPositionControl.bPositionTargetChange = 0;
                }

                gsM1_Drive.sSpeed.f32SpeedCmd = Motor_Drive_Pos_PID_Regulator(gsM1_Drive.sPositionControl.f32PositionCmd, gsM1_Drive.sPositionControl.f32PositionComp, &gsM1_Drive.sPositionControl.sPositionPiParams);

                RampControl(gsM1_Drive.sSpeed.f32SpeedCmd, gsM1_Drive.sSpeed.f32SpeedRampStep, SPEEDLOOP_PERIOD, &gsM1_Drive.sSpeed.f32SpeedReq);

                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);

                gsM1_Drive.sInitLocating.u8LocatingStep = 9;
            }

            break;

#else

            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;

            if (TB_delay_IsElapsed()) {
                Brake_Hold();

                gsM1_Drive.sInitLocating.f32PositionMech = TIM8->CNT;
                gsM1_Drive.sInitLocating.s32TurnsMech = gsM1_Drive.sPositionEnc.s32EncoderTurns;

                gsM1_Drive.sInitLocating.f32Theta_Offset = (gsM1_Drive.sInitLocating.s32TurnsMech * ENCODER_PPR * 4.0f + gsM1_Drive.sInitLocating.f32PositionMech -
                                                            gsM1_Drive.sInitLocating.s32TurnsMechPre * ENCODER_PPR * 4.0f - gsM1_Drive.sInitLocating.f32PositionMechPre) /
                                                           ENCODER_PPR / 4.0f * 2.0f * PI;

                gsM1_Drive.sInitLocating.f32PositionMechPre = gsM1_Drive.sInitLocating.f32PositionMech;
                gsM1_Drive.sInitLocating.s32TurnsMechPre = gsM1_Drive.sInitLocating.s32TurnsMech;

                if (fabsf(gsM1_Drive.sInitLocating.f32Theta_Offset) < 0.6f * LOCATING_OFFSET)
                    gsM1_Drive.sInitLocating.u8LocatingStep = 11;
                else
                    gsM1_Drive.sInitLocating.u8LocatingStep = 12;
            }

            break;

#endif

        case 11:

#if STUCK_CHECK == 1
            Brake_Off();
            gsM1_Drive.sInitLocating.u8BrakeCnt++;

            if (gsM1_Drive.sInitLocating.u8BrakeCnt >= 3)
                gsM1_Drive.sFaultId.B.StartUpFail = 1;

            gsM1_Drive.sInitLocating.u32Delay = 0;
            gsM1_Drive.sInitLocating.u8LocatingStep = 12;

            break;

#else

            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;

            if (gsM1_Drive.sInitLocating.u16LocatingPrescaler < (BRAKING_DURATION + HOLDING_DURATION)) {
                gsM1_Drive.sInitLocating.u16LocatingPrescaler++;

                gsM1_Drive.sFocPMSM.sIDQReq.f32D = gsM1_Drive.sInitLocating.f32LocatingCurrent;

                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

                if (gsM1_Drive.sInitLocating.u16LocatingPrescaler < BRAKING_DURATION)
                    gsM1_Drive.sPositionEnc.f32PositionEl -= 1.5f * LOCATING_OFFSET / (float)BRAKING_DURATION * POLE_PAIR_NUM;

                Position3 = gsM1_Drive.sPositionEnc.f32PositionEl;
                FOC_Cal();

            } else {
                gsM1_Drive.sInitLocating.u16LocatingPrescaler = 0;
                gsM1_Drive.sInitLocating.u8LocatingStep = 13;

                gsM1_Drive.sInitLocating.f32PositionMech = TIM8->CNT;
                gsM1_Drive.sInitLocating.s32TurnsMech = gsM1_Drive.sPositionEnc.s32EncoderTurns;

                gsM1_Drive.sInitLocating.f32Theta_Offset = (gsM1_Drive.sInitLocating.s32TurnsMech * ENCODER_PPR * 4.0f + gsM1_Drive.sInitLocating.f32PositionMech -
                                                            gsM1_Drive.sInitLocating.s32TurnsMechPre * ENCODER_PPR * 4.0f - gsM1_Drive.sInitLocating.f32PositionMechPre) /
                                                           ENCODER_PPR / 4.0f * 2.0f * PI;

                gsM1_Drive.sInitLocating.f32PositionMechPre = gsM1_Drive.sInitLocating.f32PositionMech;
                gsM1_Drive.sInitLocating.s32TurnsMechPre = gsM1_Drive.sInitLocating.s32TurnsMech;

                gsM1_Drive.sInitLocating.f32LastMoveCheck = fabsf(gsM1_Drive.sInitLocating.f32Theta_Offset);

                if (fabsf(gsM1_Drive.sInitLocating.f32LastMoveCheck) < 0.1f * LOCATING_OFFSET) {
                    gsM1_Drive.sFaultId.B.StartUpFail = 1;
                    gsM1_Drive.sFaultId.B.BrakeStuckError = 1;
                }
            }
            break;

#endif

        case 12:

#if STUCK_CHECK == 1

            if (gsM1_Drive.sInitLocating.u32Delay < 2000) {
                gsM1_Drive.sInitLocating.u32Delay++;
            } else {
                gsM1_Drive.sInitLocating.u32Delay = 0;
                Brake_Hold();
                gsM1_Drive.sInitLocating.u8LocatingStep = 9;
            }
            break;

#else

            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;

            if (gsM1_Drive.sInitLocating.u16LocatingPrescaler < (BRAKING_DURATION + HOLDING_DURATION)) {
                gsM1_Drive.sInitLocating.u16LocatingPrescaler++;

                gsM1_Drive.sFocPMSM.sIDQReq.f32D = gsM1_Drive.sInitLocating.f32LocatingCurrent;

                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

                if (gsM1_Drive.sInitLocating.u16LocatingPrescaler < BRAKING_DURATION)
                    gsM1_Drive.sPositionEnc.f32PositionEl -= 0.8f * LOCATING_OFFSET / (float)BRAKING_DURATION * POLE_PAIR_NUM;

                Position3 = gsM1_Drive.sPositionEnc.f32PositionEl;
                FOC_Cal();

            } else {
                gsM1_Drive.sInitLocating.u16LocatingPrescaler = 0;
                gsM1_Drive.sInitLocating.u8LocatingStep = 13;

                gsM1_Drive.sInitLocating.f32PositionMech = TIM8->CNT;
                gsM1_Drive.sInitLocating.s32TurnsMech = gsM1_Drive.sPositionEnc.s32EncoderTurns;

                gsM1_Drive.sInitLocating.f32Theta_Offset = (gsM1_Drive.sInitLocating.s32TurnsMech * ENCODER_PPR * 4.0f + gsM1_Drive.sInitLocating.f32PositionMech -
                                                            gsM1_Drive.sInitLocating.s32TurnsMechPre * ENCODER_PPR * 4.0f - gsM1_Drive.sInitLocating.f32PositionMechPre) /
                                                           ENCODER_PPR / 4.0f * 2.0f * PI;

                gsM1_Drive.sInitLocating.f32PositionMechPre = gsM1_Drive.sInitLocating.f32PositionMech;
                gsM1_Drive.sInitLocating.s32TurnsMechPre = gsM1_Drive.sInitLocating.s32TurnsMech;

                gsM1_Drive.sInitLocating.f32LastMoveCheck = fabsf(gsM1_Drive.sInitLocating.f32Theta_Offset);

                if (fabsf(gsM1_Drive.sInitLocating.f32LastMoveCheck) < 0.1f * LOCATING_OFFSET) {
                    gsM1_Drive.sFaultId.B.StartUpFail = 1;
                    gsM1_Drive.sFaultId.B.BrakeStuckError = 1;
                }
            }
            break;

#endif

        case 13:

#if STUCK_CHECK == 1

            gsM1_Drive.sInitLocating.u8StartDone = 1;
            gsM1_Drive.sInitLocating.u8BrakeCnt = 0;
            gsM1_Drive.sInitLocating.u8LocatingStep = 14;

            break;

#else

            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;

            Brake_Off();
            TB_Set_Delay(100);
            gsM1_Drive.sInitLocating.u8LocatingStep = 14;
            break;

#endif

        case 14:

#if STUCK_CHECK == 1

            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;

            Brake_Hold();

            gsM1_Drive.sFocPMSM.sIDQReq.f32D = 0;
            gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;

            PWM_OUT_DISABLE();

            gsM1_Drive.sPositionEnc.f32MagnetOffsetRadEl = gsM1_Drive.sPositionEnc.f32PositionEl;

            Encoder_Reset();

            gsM1_Drive.sPositionControl.f32PositionMechnicalStart = gsM1_Drive.sPositionEnc.f32PositionMech;
            gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionEnc.f32PositionMech;
            gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionEnc.f32PositionMech;
            gsM1_Drive.sPositionControl.f32PositionComp = gsM1_Drive.sPositionEnc.f32PositionMech;

            gsM1_Drive.sInitLocating.u8LocatingStep = 15;

#if SIN_POSITION_TEST == 1
            gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;
#endif

            V_AlphaBeta_Reset();
            I_AlphaBeta_Reset();
            V_QD_Reset();
            I_QD_Reset();
            I_QD_Ref_Reset();

            gsM1_Drive.sSpeed.f32SpeedCmd = 0;
            gsM1_Drive.sSpeed.f32SpeedReq = 0;

            //			}
            break;

#else

            gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;

            if (TB_delay_IsElapsed()) {
                Brake_Hold();

                gsM1_Drive.sFocPMSM.sIDQReq.f32D = 0;

                PWM_OUT_DISABLE();

                gsM1_Drive.sPositionEnc.f32MagnetOffsetRadEl = gsM1_Drive.sPositionEnc.f32PositionEl;

                Encoder_Reset();

#if MAGNET_ENCODER_FBK == 0

                gsM1_Drive.sPositionControl.f32PositionMechnicalStart = gsM1_Drive.sPositionEnc.f32PositionMech;
                gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionEnc.f32PositionMech;
                gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionEnc.f32PositionMech;
                gsM1_Drive.sPositionControl.f32PositionComp = gsM1_Drive.sPositionEnc.f32PositionMech;

#else

                gsM1_Drive.sPositionControl.f32PositionMechnicalStart = gsM1_Drive.sPositionControl.f32PositionComp;
                gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionControl.f32PositionComp;
                gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionControl.f32PositionComp;
                gsM1_Drive.sPositionControl.f32PositionStart = gsM1_Drive.sPositionControl.f32PositionComp;
                gsM1_Drive.sPositionControl.f32PositionPre = gsM1_Drive.sPositionControl.f32Position;

#endif

                gsM1_Drive.sInitLocating.u8LocatingStep = 15;

#if SIN_POSITION_TEST == 1
                gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;
#endif

                V_AlphaBeta_Reset();
                I_AlphaBeta_Reset();
                V_QD_Reset();
                I_QD_Reset();
                I_QD_Ref_Reset();

                gsM1_Drive.sSpeed.f32SpeedCmd = 0;
                gsM1_Drive.sSpeed.f32SpeedReq = 0;
            }
            break;

#endif

        default:
            break;
    }
}

#endif

void PositionLocatingReset(void) {
#if INIT_LOCATING == 1

    gsM1_Drive.sInitLocating.f32Theta_H = PI * 2.0f;
    gsM1_Drive.sInitLocating.f32Theta_L = 0.0f;
    gsM1_Drive.sInitLocating.f32Theta_M = PI;
    gsM1_Drive.sInitLocating.f32AccuracyLimit = LOCATING_ACCURACY;
    gsM1_Drive.sInitLocating.f32LocatingCurrent = LOCATING_CURRENT;
    gsM1_Drive.sInitLocating.f32PositionMech = 0;
    gsM1_Drive.sInitLocating.f32PositionMechPre = 0;
    gsM1_Drive.sInitLocating.f32Theta_M_Pre = 0;
    gsM1_Drive.sInitLocating.f32Theta_Offset = 0;
    gsM1_Drive.sInitLocating.s32TurnsMech = 0;
    gsM1_Drive.sInitLocating.s32TurnsMechPre = 0;
    gsM1_Drive.sInitLocating.u16LocatingPrescaler = 0;
    gsM1_Drive.sInitLocating.u8LocatingCnt = 0;
    gsM1_Drive.sInitLocating.u8LocatingStep = 0;
    gsM1_Drive.sInitLocating.f32Theta_M_Delta = 0;

    gsM1_Drive.sInitLocating.u32StartCnt = 0;
    gsM1_Drive.sInitLocating.u32LockCnt = 0;
    gsM1_Drive.sInitLocating.u32Delay = 0;

#endif
}

void FOC_Cal(void) {
    gsM1_Drive.sFocPMSM.sAnglePosEl.f32Cos = arm_cos_f32(gsM1_Drive.sPositionEnc.f32PositionEl);
    gsM1_Drive.sFocPMSM.sAnglePosEl.f32Sin = arm_sin_f32(gsM1_Drive.sPositionEnc.f32PositionEl);

    gsM1_Drive.sFocPMSM.sIAlBe = Motor_Drive_Clarke(gsM1_Drive.sFocPMSM.sIABC);
    gsM1_Drive.sFocPMSM.sIDQ = Motor_Drive_Park(gsM1_Drive.sFocPMSM.sIAlBe, gsM1_Drive.sFocPMSM.sAnglePosEl);

    gsM1_Drive.sFocPMSM.sIqPiParams.f32FFPartK = 0;
    gsM1_Drive.sFocPMSM.sIdPiParams.f32FFPartK = 0;

    gsM1_Drive.sFocPMSM.sUDQReq.f32Q = Motor_Drive_Cur_PID_Regulator(gsM1_Drive.sFocPMSM.sIDQReq.f32Q, gsM1_Drive.sFocPMSM.sIDQ.f32Q, &gsM1_Drive.sFocPMSM.sIqPiParams);

    gsM1_Drive.sFocPMSM.sUDQReq.f32D = Motor_Drive_Cur_PID_Regulator(gsM1_Drive.sFocPMSM.sIDQReq.f32D, gsM1_Drive.sFocPMSM.sIDQ.f32D, &gsM1_Drive.sFocPMSM.sIdPiParams);

    gsM1_Drive.sFocPMSM.sUAlBeReq = Motor_Drive_Rev_Park(gsM1_Drive.sFocPMSM.sUDQReq, gsM1_Drive.sFocPMSM.sAnglePosEl);

    Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);
}

void PositionHold(void) {
    static u32 speed_scale;

    gsM1_Drive.sSpeed.f32SpeedRampStep = SPEED_RAMP_STEP_DEFAULT;

    if (speed_scale < SPEEDLOOP_PERIOD / SAMPLE_PERIOD - 1) {
        speed_scale++;
    } else {
        gsM1_Drive.sSpeed.f32SpeedCmd = 0;
        RampControl(gsM1_Drive.sSpeed.f32SpeedCmd, gsM1_Drive.sSpeed.f32SpeedRampStep, SPEEDLOOP_PERIOD, &gsM1_Drive.sSpeed.f32SpeedReq);
        speed_scale = 0;

        gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);
        gsM1_Drive.sFocPMSM.sIDQReq.f32D = 0;
    }

    Motor_Drive_Get_Electrical_Angle();

    Motor_Drive_FOC();
}

#ifdef HFI
u32 rst_delay = 0;

//float f32Idh_table[180] = {0};

void HFI_Search(void) {
    switch (gsM1_Drive.sHFISearch.u8Step) {
        case 0:
            PWM_OUT_DISABLE();
            TIM1->CCER |= 0x111;

            if ((gsM1_Ctrl.eState == RUN) && (meM1_StateRun == READY)) {
                gsM1_Drive.sHFISearch.f32EstAngle = 0;
                gsM1_Drive.sHFISearch.f32RadCnt = 0;

#if BIG_ID_ENABLE == 1
                gsM1_Drive.sHFISearch.f32IdhPre = 0.0f;
                gsM1_Drive.sHFISearch.f32Idh = 0.0f;
#else
                gsM1_Drive.sHFISearch.f32IdhPre = 10000.0f;
                gsM1_Drive.sHFISearch.f32Idh = 10000.0f;
#endif

                gsM1_Drive.sFocPMSM.sUDQReq.f32D = HFI_AMPLITUDE * arm_cos_f32(2.0f * PI * gsM1_Drive.sHFISearch.f32RadCnt / HFI_PERIOD);
                gsM1_Drive.sFocPMSM.sUDQReq.f32Q = 0;

                gsM1_Drive.sHFISearch.u8Step = 1;

                PWM_OUT_ENABLE();
            }
            break;

        case 1:
            gsM1_Drive.sPositionEnc.f32PositionEl = gsM1_Drive.sHFISearch.f32EstAngle * PI / 180.0f;

            gsM1_Drive.sFocPMSM.sAnglePosEl.f32Cos = arm_cos_f32(gsM1_Drive.sPositionEnc.f32PositionEl);
            gsM1_Drive.sFocPMSM.sAnglePosEl.f32Sin = arm_sin_f32(gsM1_Drive.sPositionEnc.f32PositionEl);

            gsM1_Drive.sFocPMSM.sUAlBeReq = Motor_Drive_Rev_Park(gsM1_Drive.sFocPMSM.sUDQReq, gsM1_Drive.sFocPMSM.sAnglePosEl);

            Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

            gsM1_Drive.sFocPMSM.sIAlBe = Motor_Drive_Clarke(gsM1_Drive.sFocPMSM.sIABC);
            gsM1_Drive.sFocPMSM.sIDQ = Motor_Drive_Park(gsM1_Drive.sFocPMSM.sIAlBe, gsM1_Drive.sFocPMSM.sAnglePosEl);

            gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferM = gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferM_K_1 + gsM1_Drive.sHFISearch.sSinFilter.f32Omega * SAMPLE_PERIOD *
                                                                                                                        (gsM1_Drive.sFocPMSM.sIDQ.f32D * arm_sin_f32(2.0f * PI * (gsM1_Drive.sHFISearch.f32RadCnt - 1) / HFI_PERIOD) - 2.0f * gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferM_K_1 - gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferY_K_1);

            gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferY = gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferY_K_1 + gsM1_Drive.sHFISearch.sSinFilter.f32Omega * SAMPLE_PERIOD *
                                                                                                                        gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferM;

            gsM1_Drive.sHFISearch.f32IdsinFlt = gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferY;

            gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferY_K_1 = gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferY;

            gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferM_K_1 = gsM1_Drive.sHFISearch.sSinFilter.f32FiltBufferM;

            gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferM = gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferM_K_1 + gsM1_Drive.sHFISearch.sCosFilter.f32Omega * SAMPLE_PERIOD *
                                                                                                                        (gsM1_Drive.sFocPMSM.sIDQ.f32D * arm_cos_f32(2.0f * PI * (gsM1_Drive.sHFISearch.f32RadCnt - 1) / HFI_PERIOD) - 2.0f * gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferM_K_1 - gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferY_K_1);

            gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferY = gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferY_K_1 + gsM1_Drive.sHFISearch.sCosFilter.f32Omega * SAMPLE_PERIOD *
                                                                                                                        gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferM;

            gsM1_Drive.sHFISearch.f32IdcosFlt = gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferY;

            gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferY_K_1 = gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferY;

            gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferM_K_1 = gsM1_Drive.sHFISearch.sCosFilter.f32FiltBufferM;

            gsM1_Drive.sHFISearch.f32Idh = gsM1_Drive.sHFISearch.f32IdsinFlt * gsM1_Drive.sHFISearch.f32IdsinFlt + gsM1_Drive.sHFISearch.f32IdcosFlt * gsM1_Drive.sHFISearch.f32IdcosFlt;

            if (gsM1_Drive.sHFISearch.f32EstAngle >= 10.0f) {
#if BIG_ID_ENABLE == 1
                if (gsM1_Drive.sHFISearch.f32Idh > gsM1_Drive.sHFISearch.f32IdhPre)
#else
                if (gsM1_Drive.sHFISearch.f32Idh < gsM1_Drive.sHFISearch.f32IdhPre)
#endif
                {
                    gsM1_Drive.sHFISearch.f32Offset = gsM1_Drive.sPositionEnc.f32PositionEl;
                    gsM1_Drive.sHFISearch.f32IdhPre = gsM1_Drive.sHFISearch.f32Idh;
                }
            }

            if (gsM1_Drive.sHFISearch.u32HFI_Duration < HFI_DURATION) {
                gsM1_Drive.sHFISearch.u32HFI_Duration++;
                gsM1_Drive.sHFISearch.f32RadCnt += 1.0f;
                gsM1_Drive.sFocPMSM.sUDQReq.f32D = HFI_AMPLITUDE * arm_cos_f32(2.0f * PI * gsM1_Drive.sHFISearch.f32RadCnt / HFI_PERIOD);
                gsM1_Drive.sFocPMSM.sUDQReq.f32Q = 0;

                gsM1_Drive.sHFISearch.u8Step = 1;
            } else {
                gsM1_Drive.sHFISearch.u32HFI_Duration = 0;

                gsM1_Drive.sHFISearch.f32RadCnt += 1.0f;
                gsM1_Drive.sFocPMSM.sUDQReq.f32D = HFI_AMPLITUDE * arm_cos_f32(2.0f * PI * gsM1_Drive.sHFISearch.f32RadCnt / HFI_PERIOD);
                gsM1_Drive.sFocPMSM.sUDQReq.f32Q = 0;

                if (gsM1_Drive.sHFISearch.f32EstAngle >= 190.0f) {
                    gsM1_Drive.sHFISearch.u8Step = 3;
                } else {
                    gsM1_Drive.sHFISearch.f32EstAngle += 1.0f;
                    //if(gsM1_Drive.sHFISearch.f32EstAngle >= 10.0f)
                    //{
                    //	int index = gsM1_Drive.sHFISearch.f32EstAngle - 10;
                    //f32Idh_table[index] = gsM1_Drive.sHFISearch.f32Idh;
                    //}
                }
            }

            break;

        case 3:
            PWM_OUT_DISABLE();

            if (rst_delay < 1000)
                rst_delay++;
            else {
                rst_delay = 0;
                gsM1_Drive.sHFISearch.u8Step = 4;
            }
            break;

        case 4:
            gsM1_Drive.sHFISearch.f32SearchingAngle[0] = gsM1_Drive.sHFISearch.f32Offset;
            gsM1_Drive.sHFISearch.f32SearchingAngle[1] = gsM1_Drive.sHFISearch.f32Offset + PI;
            gsM1_Drive.sHFISearch.f32SearchingAngle[2] = gsM1_Drive.sHFISearch.f32Offset;

            gsM1_Drive.sHFISearch.f32ResponseCurrent[0] = 0;
            gsM1_Drive.sHFISearch.f32ResponseCurrent[1] = 0;
            gsM1_Drive.sHFISearch.f32ResponseCurrent[2] = 0;

            gsM1_Drive.sFocPMSM.sUDQReq.f32D = PULSE_VOLTAGE;
            gsM1_Drive.sFocPMSM.sUDQReq.f32Q = 0;

            gsM1_Drive.sHFISearch.u8Step = 5;

            break;

        case 5:
            gsM1_Drive.sPositionEnc.f32PositionEl = gsM1_Drive.sHFISearch.f32SearchingAngle[gsM1_Drive.sHFISearch.u8SearchingAngleCnt];

            gsM1_Drive.sFocPMSM.sAnglePosEl.f32Cos = arm_cos_f32(gsM1_Drive.sPositionEnc.f32PositionEl);
            gsM1_Drive.sFocPMSM.sAnglePosEl.f32Sin = arm_sin_f32(gsM1_Drive.sPositionEnc.f32PositionEl);

            gsM1_Drive.sFocPMSM.sIAlBe = Motor_Drive_Clarke(gsM1_Drive.sFocPMSM.sIABC);

            gsM1_Drive.sFocPMSM.sUAlBeReq = Motor_Drive_Rev_Park(gsM1_Drive.sFocPMSM.sUDQReq, gsM1_Drive.sFocPMSM.sAnglePosEl);

            Motor_Drive_DutyCycleCal(gsM1_Drive.sFocPMSM.sUAlBeReq, gsM1_Drive.sFocPMSM.f32UDcBus);

            gsM1_Drive.sHFISearch.u8Step = 6;

            PWM_OUT_ENABLE();

            break;

        case 6:
            gsM1_Drive.sFocPMSM.sIAlBe = Motor_Drive_Clarke(gsM1_Drive.sFocPMSM.sIABC);

            if (gsM1_Drive.sHFISearch.u16ResponseCurrentCnt <= PULSE_WIDTH - 1) {
                if (gsM1_Drive.sHFISearch.f32ResponseCurrent[gsM1_Drive.sHFISearch.u8SearchingAngleCnt] < gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha * gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha + gsM1_Drive.sFocPMSM.sIAlBe.f32Beta * gsM1_Drive.sFocPMSM.sIAlBe.f32Beta)
                    gsM1_Drive.sHFISearch.f32ResponseCurrent[gsM1_Drive.sHFISearch.u8SearchingAngleCnt] = gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha * gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha + gsM1_Drive.sFocPMSM.sIAlBe.f32Beta * gsM1_Drive.sFocPMSM.sIAlBe.f32Beta;

                gsM1_Drive.sHFISearch.u16ResponseCurrentCnt++;
            } else if (gsM1_Drive.sHFISearch.u16ResponseCurrentCnt <= MAX_I_RESPONSE_CNT - 1) {
                PWM_OUT_DISABLE();

                if (gsM1_Drive.sHFISearch.f32ResponseCurrent[gsM1_Drive.sHFISearch.u8SearchingAngleCnt] < gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha * gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha + gsM1_Drive.sFocPMSM.sIAlBe.f32Beta * gsM1_Drive.sFocPMSM.sIAlBe.f32Beta)
                    gsM1_Drive.sHFISearch.f32ResponseCurrent[gsM1_Drive.sHFISearch.u8SearchingAngleCnt] = gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha * gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha + gsM1_Drive.sFocPMSM.sIAlBe.f32Beta * gsM1_Drive.sFocPMSM.sIAlBe.f32Beta;

                gsM1_Drive.sHFISearch.u16ResponseCurrentCnt++;
            } else {
                gsM1_Drive.sHFISearch.u16ResponseCurrentCnt = 0;

                if (gsM1_Drive.sHFISearch.u8SearchingAngleCnt < 2) {
                    gsM1_Drive.sHFISearch.u8SearchingAngleCnt++;
                    gsM1_Drive.sHFISearch.u8Step = 5;
                } else {
                    gsM1_Drive.sHFISearch.u8SearchingAngleCnt = 0;
                    gsM1_Drive.sHFISearch.u8Step = 7;
                }
            }
            break;

        case 7:

            if ((gsM1_Drive.sHFISearch.f32ResponseCurrent[1] > gsM1_Drive.sHFISearch.f32ResponseCurrent[2])) {
                gsM1_Drive.sHFISearch.f32OffsetFinal = gsM1_Drive.sHFISearch.f32SearchingAngle[1];
            } else {
                gsM1_Drive.sHFISearch.f32OffsetFinal = gsM1_Drive.sHFISearch.f32SearchingAngle[2];
            }

            gsM1_Drive.sHFISearch.u8Step = 8;

            gsM1_Drive.sPositionEnc.f32MagnetOffsetRadEl = fmodf(gsM1_Drive.sHFISearch.f32OffsetFinal, 2.0f * PI);

            Encoder_Reset();

#if MAGNET_ENCODER_FBK == 0

            gsM1_Drive.sPositionControl.f32PositionMechnicalStart = gsM1_Drive.sPositionEnc.f32PositionMech;
            gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionEnc.f32PositionMech;
            gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionEnc.f32PositionMech;
            gsM1_Drive.sPositionControl.f32PositionComp = gsM1_Drive.sPositionEnc.f32PositionMech;

#else

            gsM1_Drive.sPositionControl.f32PositionMechnicalStart = gsM1_Drive.sPositionControl.f32PositionComp;
            gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionControl.f32PositionComp;
            gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionControl.f32PositionComp;
            gsM1_Drive.sPositionControl.f32PositionStart = gsM1_Drive.sPositionControl.f32PositionComp;
            gsM1_Drive.sPositionControl.f32PositionPre = gsM1_Drive.sPositionControl.f32Position;

#endif

            V_AlphaBeta_Reset();
            I_AlphaBeta_Reset();
            V_QD_Reset();
            I_QD_Reset();
            I_QD_Ref_Reset();

            gsM1_Drive.sSpeed.f32SpeedCmd = 0;
            gsM1_Drive.sSpeed.f32SpeedReq = 0;

            PWM_OUT_ENABLE();

            Brake_Off();

#if BIG_ID_ENABLE == 1
            gsM1_Drive.sPositionControl.sPositionPiParams.f32UpperLimit = 100.0f;
            gsM1_Drive.sPositionControl.sPositionPiParams.f32LowerLimit = -100.0f;
#endif

            break;

        case 8:
            gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;

            if (gsM1_Drive.sHFISearch.u32CtrlDelay < JITTER_COUNT) {
                gsM1_Drive.sHFISearch.u32CtrlDelay++;
            } else {
                gsM1_Drive.sHFISearch.u32CtrlDelay = 0;
                gsM1_Drive.sHFISearch.u8Step = 9;
            }

            gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionControl.f32PositionMechnicalStart + 1.0f;

            if (gsM1_Drive.sHFISearch.u32Scale < 19) {
                gsM1_Drive.sHFISearch.u32Scale++;

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            } else {
                gsM1_Drive.sHFISearch.u32Scale = 0;

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

                gsM1_Drive.sSpeed.f32SpeedCmd = Motor_Drive_Pos_PID_Regulator(gsM1_Drive.sPositionControl.f32PositionCmd, gsM1_Drive.sPositionControl.f32PositionComp, &gsM1_Drive.sPositionControl.sPositionPiParams);

                RampControl(gsM1_Drive.sSpeed.f32SpeedCmd, gsM1_Drive.sSpeed.f32SpeedRampStep, SPEEDLOOP_PERIOD, &gsM1_Drive.sSpeed.f32SpeedReq);

                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            }

            break;

        case 9:
            gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;

            if (gsM1_Drive.sHFISearch.u32CtrlDelay < JITTER_COUNT) {
                gsM1_Drive.sHFISearch.u32CtrlDelay++;
            } else {
                gsM1_Drive.sHFISearch.u32CtrlDelay = 0;
                gsM1_Drive.sHFISearch.u8Step = 10;

                Brake_Hold();
            }

            gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionControl.f32PositionMechnicalStart - 1.0f;

            if (gsM1_Drive.sHFISearch.u32Scale < 19) {
                gsM1_Drive.sHFISearch.u32Scale++;

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            } else {
                gsM1_Drive.sHFISearch.u32Scale = 0;

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

                gsM1_Drive.sSpeed.f32SpeedCmd = Motor_Drive_Pos_PID_Regulator(gsM1_Drive.sPositionControl.f32PositionCmd, gsM1_Drive.sPositionControl.f32PositionComp, &gsM1_Drive.sPositionControl.sPositionPiParams);

                RampControl(gsM1_Drive.sSpeed.f32SpeedCmd, gsM1_Drive.sSpeed.f32SpeedRampStep, SPEEDLOOP_PERIOD, &gsM1_Drive.sSpeed.f32SpeedReq);

                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            }
            break;

        case 10:
            gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;

            gsM1_Drive.sSpeed.sSpeedPiParams.f32UpperLimit = STUCK_CHECK_CURRENT;
            gsM1_Drive.sSpeed.sSpeedPiParams.f32LowerLimit = -gsM1_Drive.sSpeed.sSpeedPiParams.f32UpperLimit;

#if BIG_ID_ENABLE == 1
            gsM1_Drive.sPositionControl.sPositionPiParams.f32UpperLimit = 50.0f;
            gsM1_Drive.sPositionControl.sPositionPiParams.f32LowerLimit = -50.0f;
#endif

            if (gsM1_Drive.sHFISearch.u32CtrlDelay < 10000) {
                gsM1_Drive.sHFISearch.u32CtrlDelay++;
            } else {
                gsM1_Drive.sHFISearch.u32CtrlDelay = 0;
                gsM1_Drive.sHFISearch.u8Step = 11;

                if (gsM1_Drive.sPositionEnc.f32PositionMech - gsM1_Drive.sPositionControl.f32PositionMechnicalStart < 0.5f)
                    gsM1_Drive.sFaultId.B.StartUpFail = 1;
            }

            gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionControl.f32PositionMechnicalStart + 2.0f;

            if (gsM1_Drive.sHFISearch.u32Scale < 19) {
                gsM1_Drive.sHFISearch.u32Scale++;

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            } else {
                gsM1_Drive.sHFISearch.u32Scale = 0;

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

                gsM1_Drive.sSpeed.f32SpeedCmd = Motor_Drive_Pos_PID_Regulator(gsM1_Drive.sPositionControl.f32PositionCmd, gsM1_Drive.sPositionControl.f32PositionComp, &gsM1_Drive.sPositionControl.sPositionPiParams);

                RampControl(gsM1_Drive.sSpeed.f32SpeedCmd, gsM1_Drive.sSpeed.f32SpeedRampStep, SPEEDLOOP_PERIOD, &gsM1_Drive.sSpeed.f32SpeedReq);

                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            }
            break;

        case 11:
            gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;

            if (gsM1_Drive.sHFISearch.u32CtrlDelay < 10000) {
                gsM1_Drive.sHFISearch.u32CtrlDelay++;
            } else {
                gsM1_Drive.sHFISearch.u32CtrlDelay = 0;
                gsM1_Drive.sHFISearch.u8Step = 12;

                if (fabsf(gsM1_Drive.sPositionEnc.f32PositionMech - gsM1_Drive.sPositionControl.f32PositionMechnicalStart) > 0.5f)
                    gsM1_Drive.sFaultId.B.StartUpFail = 1;

                gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;
                PWM_OUT_DISABLE();

                V_AlphaBeta_Reset();
                I_AlphaBeta_Reset();
                V_QD_Reset();
                I_QD_Reset();
                I_QD_Ref_Reset();

                gsM1_Drive.sSpeed.f32SpeedCmd = 0;
                gsM1_Drive.sSpeed.f32SpeedReq = 0;

                gsM1_Drive.sSpeed.sSpeedPiParams.f32UpperLimit = SPEEDLOOP_UPPER_LIMIT;
                gsM1_Drive.sSpeed.sSpeedPiParams.f32LowerLimit = SPEEDLOOP_LOWER_LIMIT;

#if BIG_ID_ENABLE == 1
                gsM1_Drive.sPositionControl.sPositionPiParams.f32UpperLimit = POSITIONLOOP_UPPER_LIMIT;
                gsM1_Drive.sPositionControl.sPositionPiParams.f32LowerLimit = POSITIONLOOP_LOWER_LIMIT;
#endif
            }

            gsM1_Drive.sPositionControl.f32PositionCmd = gsM1_Drive.sPositionControl.f32PositionMechnicalStart;

            if (gsM1_Drive.sHFISearch.u32Scale < 19) {
                gsM1_Drive.sHFISearch.u32Scale++;

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            } else {
                gsM1_Drive.sHFISearch.u32Scale = 0;

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

                gsM1_Drive.sSpeed.f32SpeedCmd = Motor_Drive_Pos_PID_Regulator(gsM1_Drive.sPositionControl.f32PositionCmd, gsM1_Drive.sPositionControl.f32PositionComp, &gsM1_Drive.sPositionControl.sPositionPiParams);

                RampControl(gsM1_Drive.sSpeed.f32SpeedCmd, gsM1_Drive.sSpeed.f32SpeedRampStep, SPEEDLOOP_PERIOD, &gsM1_Drive.sSpeed.f32SpeedReq);

                gsM1_Drive.sFocPMSM.sIDQReq.f32Q = Motor_Drive_Spd_PID_Regulator(gsM1_Drive.sSpeed.f32SpeedReq, gsM1_Drive.sSpeed.f32SpeedFilt, &gsM1_Drive.sSpeed.sSpeedPiParams);

                Motor_Drive_Get_Electrical_Angle();

                Motor_Drive_FOC();
            }
            break;

        default:
            PWM_OUT_DISABLE();
            break;
    }
}

#endif
