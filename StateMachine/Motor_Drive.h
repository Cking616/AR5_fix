#ifndef _MOTOR_DRIVE_H_
#define _MOTOR_DRIVE_H_

#include "Configuration.h"
#include "motor_structure.h"
#include "stm32f4xx.h"

#define PI_CONST 3.1415926535F
#define SQRT3 1.732050807568877f

#if BIG_ID_ENABLE == 1

#if KOLLMORGEN == 1

#define POLE_PAIR_NUM (u8)6 /* 极对数Number of motor pole pairs */
#define RS 0.388F           /* 电阻Stator resistance , ohm*/
#define LS 0.00055F         /*电感 Stator inductance , H */
#define Lamda 0.0167F

#endif

#if ZZ_MOTOR == 1

#define POLE_PAIR_NUM (u8)4 /* 极对数Number of motor pole pairs */
#define RS 0.09F            /* 电阻Stator resistance , ohm*/
#define LS 0.000335F        /* 电感 Stator inductance , H */
#define Lamda 0.019F
#define Kt 0.114F   /* Torque Constant Nm/A  */
#define Kv 8.1F     /* BEMF Constant V/KRPM */
#define J 0.000061F /* Inertia kgm^2 */

#else

#define POLE_PAIR_NUM (u8)8 /* 极对数Number of motor pole pairs */
#define RS 0.08669F         /* 电阻Stator resistance , ohm*/
#define LS 0.0002238F       /* 电感 Stator inductance , H */
//		#define Lamda											0.00918F
#define Lamda 0.00992F
#define Kt 0.119F   /* Torque Constant Nm/A  */
#define Kv 14.52F   /* BEMF Constant V/KRPM */
#define J 0.000111F /* Inertia kgm^2 */

#endif

#else

#if ZZ_MOTOR == 1

#define POLE_PAIR_NUM (u8)4 /* 极对数Number of motor pole pairs */
#define RS 0.9F             /* 电阻Stator resistance , ohm*/
#define LS 0.00206F         /* 电感 Stator inductance , H */
#define Lamda 0.0175F
#define Kt 0.105F   /* Torque Constant Nm/A  */
#define Kv 7.43F    /* BEMF Constant V/KRPM */
#define J 0.000014F /* Inertia kgm^2 */

#else

#define POLE_PAIR_NUM (u8)8 /* 极对数Number of motor pole pairs */
#define RS 0.3537F          /* 电阻Stator resistance , ohm*/
#define LS 0.0002988F       /* 电感 Stator inductance , H */
#define Lamda 0.00705F
#define Kt 0.0846F     /* Torque Constant Nm/A  */
#define Kv 10.32F      /* BEMF Constant V/KRPM */
#define J 0.000018281F /* Inertia kgm^2 */

#endif

#endif

#define HARMONIC_AMPLIFY 101.0F

#if IS_17BITS_MAGNET_ENCODER == 1
#define MAGNET_ENCODER_BITS 131072.0F
#else
#define MAGNET_ENCODER_BITS 524288.0F
#endif

#if BIG_ID_ENABLE == 1
#define ENCODER_PPR (u16)(5000)
#else
#if SMALL_PPR_982 == 1
#define ENCODER_PPR (u16)(982)
#else
#define ENCODER_PPR (u16)(1000)
#endif
#endif

#define PULSES_PER_MAG_BIT 4.0F * (float)ENCODER_PPR *HARMONIC_AMPLIFY / MAGNET_ENCODER_BITS

#define T_ALIGNMENT (u16)1000

#define ALIGNMENT_ANGLE (u16)0
#define ALIGNMENT_ANGLE_RAD (float)ALIGNMENT_ANGLE * 2.0f * PI_CONST / 360.0f

#define I_ALIGNMENT (u16)10000

#define T_ALIGNMENT_PWM_STEPS (u32)((T_ALIGNMENT * SAMPLING_FREQ) / 1000)
#define ALIGNMENT_ANGLE_S16 (s16)((s32)(ALIGNMENT_ANGLE)*65536 / 360)

#define COUNTER_RESET (u16)((((s32)(ALIGNMENT_ANGLE)*4 * ENCODER_PPR / 360) - 1) / POLE_PAIR_NUM)

#define BRAKE_ON_DELAY 500
#define BRAKE_OFF_DELAY 100
#define FLASH_STORAGE_DELAY 800

#define DEFAULT_VDC_OFFSET 0.0f

#if BIG_ID_ENABLE == 1
#if HARDWARE_VERSION_2_2
#define DEFAULT_I_A_OFFSET 2048.0f
#define DEFAULT_I_B_OFFSET 2048.0f
#else
#define DEFAULT_I_A_OFFSET 2110.9f
#define DEFAULT_I_B_OFFSET 2110.9f
#endif
#else
#define DEFAULT_I_A_OFFSET 2048.0f
#define DEFAULT_I_B_OFFSET 2048.0f
#endif

#if BIG_ID_ENABLE == 1
#define PEAK_CURRENT_THRESHOLD 50.0f
#define OVER_CURRENT_THRESHOLD 45.0f
#else
#if ZZ_MOTOR == 1
#define OVER_CURRENT_THRESHOLD 12.0f
#define PEAK_CURRENT_THRESHOLD 12.0f
#else
#define OVER_CURRENT_THRESHOLD 12.0f
#define PEAK_CURRENT_THRESHOLD 12.0f
#endif
#endif

#define OVER_HEAT_UPPER_THRESHOLD 70.0f
#define OVER_HEAT_LOWER_THRESHOLD 0.0f

#if BIG_ID_ENABLE == 1
#define OVER_LOAD_THRESHOLD 0.9f * OVER_CURRENT_THRESHOLD
#else
#define OVER_LOAD_THRESHOLD 0.8f * OVER_CURRENT_THRESHOLD
#endif

#define LOCK_ROTOR_THRESHOLD 0.8f * OVER_CURRENT_THRESHOLD
#define LOSS_PHASE_THRESHOLD 0.0f
#define ADC_OFFSET_THRESHOLD 60.0f
#define OVER_SPEED_THRESHOLD 3200.0f
#define OVER_VOLTAGE_THRESHOLD 50.0f
#define UNDER_VOLTAGE_THRESHOLD 46.0f
#define OPTICAL_ENCODER_THRESHOLD 1
#define LOST_ROTOR_THRESHOLD 5.0f
#define POWER_SUPPLY_OFF_THRESHOLD 40.0f

#if BIG_ID_ENABLE == 1
#define SPEED_COMPARE_THRESHOLD 20.0f
#define SPEED_RESOLUTION 3.0f
#else
#define SPEED_COMPARE_THRESHOLD 100.0f
#define SPEED_RESOLUTION 15.0f
#endif

#if BIG_ID_ENABLE == 1
#define IN_OUT_COMP_ERROR 5.0f
#else
#define IN_OUT_COMP_ERROR 5.0f
#endif

#define DEFAULT_DUTY_CYCLE_LIMIT 0.96f
#define DEFAULT_VDC 48.0f
#define DEFAULT_VDC_FILT 48.0f

#define DEFAULT_ALIGNMENT_CURRENT 0.8f * OVER_CURRENT_THRESHOLD

#if BIG_ID_ENABLE == 1

#if ZZ_MOTOR == 1

//#define CURRENT_D_KP 1.675f
//#define CURRENT_D_KI 450.0f

#define CURRENT_D_KP 2.68f
#define CURRENT_D_KI 720.0f

#define CURRENT_D_KD 0.0f
#define CURRENT_D_UPPER_LIMIT 12.0f
#define CURRENT_D_LOWER_LIMIT -CURRENT_D_UPPER_LIMIT
#define CURRENT_D_INTEG_SCALE 1.0f

//#define CURRENT_Q_KP 1.675f
//#define CURRENT_Q_KI 450.0f

#define CURRENT_Q_KP 2.68f
#define CURRENT_Q_KI 720.0f

#define CURRENT_Q_KD 0.00001f

#define CURRENT_Q_UPPER_LIMIT 26.5f
#define CURRENT_Q_LOWER_LIMIT -CURRENT_Q_UPPER_LIMIT
#define CURRENT_Q_INTEG_SCALE 1.0f

#else

#define CURRENT_D_KP 1.119f
#define CURRENT_D_KI 433.445f
#define CURRENT_D_KD 0.00005f
#define CURRENT_D_UPPER_LIMIT 12.0f
#define CURRENT_D_LOWER_LIMIT -CURRENT_D_UPPER_LIMIT
#define CURRENT_D_INTEG_SCALE 1.0f

#define CURRENT_Q_KP 1.119
#define CURRENT_Q_KI 433.445f
#define CURRENT_Q_KD 0.00005f

#define CURRENT_Q_UPPER_LIMIT 26.5f
#define CURRENT_Q_LOWER_LIMIT -CURRENT_Q_UPPER_LIMIT
#define CURRENT_Q_INTEG_SCALE 1.0f
#endif

#else

#if ZZ_MOTOR == 1

#define CURRENT_D_KP 10.3f
#define CURRENT_D_KI 4500.0f

#define CURRENT_D_KD 0.0f
#define CURRENT_D_UPPER_LIMIT 12.0f
#define CURRENT_D_LOWER_LIMIT -CURRENT_D_UPPER_LIMIT
#define CURRENT_D_INTEG_SCALE 1.0f

#define CURRENT_Q_KP 10.3f
#define CURRENT_Q_KI 4500.0f

#define CURRENT_Q_KD 0.0f

#define CURRENT_Q_UPPER_LIMIT 26.5f
#define CURRENT_Q_LOWER_LIMIT -CURRENT_Q_UPPER_LIMIT
#define CURRENT_Q_INTEG_SCALE 1.0f

#else
#define CURRENT_D_KP 1.494f
#define CURRENT_D_KI 1768.5f
#define CURRENT_D_KD 0.0f
#define CURRENT_D_UPPER_LIMIT 12.0f
#define CURRENT_D_LOWER_LIMIT -CURRENT_D_UPPER_LIMIT
#define CURRENT_D_INTEG_SCALE 1.0f

#define CURRENT_Q_KP 1.494f
#define CURRENT_Q_KI 1768.5f
#define CURRENT_Q_KD 0.0f

#define CURRENT_Q_UPPER_LIMIT 26.5f
#define CURRENT_Q_LOWER_LIMIT -CURRENT_Q_UPPER_LIMIT
#define CURRENT_Q_INTEG_SCALE 1.0f

#endif

#endif

#define SPEED_RAMP_STEP_DEFAULT 10000.0f

#define SPEED_MAX_ACCELERATION 6000.0f
#define SPEED_JERK 6000.0f

#if BIG_ID_ENABLE == 1

#if ZZ_MOTOR == 1

#define SPEED_KP 0.05f
#define SPEED_KI 2.5f
#define SPEED_KD 0.0f

#else

#define SPEED_KP 0.101f
#define SPEED_KI 1.01f
#define SPEED_KD 0.0f

#endif

//#define SPEEDLOOP_UPPER_LIMIT 0.9f * OVER_CURRENT_THRESHOLD
//#define SPEEDLOOP_LOWER_LIMIT -SPEEDLOOP_UPPER_LIMIT
#define SPEEDLOOP_UPPER_LIMIT 5.2f
#define SPEEDLOOP_LOWER_LIMIT -SPEEDLOOP_UPPER_LIMIT

#define SPEED_RAMP_UP 0.15f
#define SPEED_RAMP_DOWN -SPEED_RAMP_UP
#define SPEEDLOOP_SCALE 1

#else

#if ZZ_MOTOR == 1

#define SPEED_KP 0.012f  //0.003f
#define SPEED_KI 0.6f    //0.15f
#define SPEED_KD 0.0f

#else

#define SPEED_KP 0.019f
#define SPEED_KI 0.19f
#define SPEED_KD 0.0f

#endif

#define SPEEDLOOP_UPPER_LIMIT 0.9f * OVER_CURRENT_THRESHOLD
#define SPEEDLOOP_LOWER_LIMIT -SPEEDLOOP_UPPER_LIMIT
#define SPEED_RAMP_UP 0.15f
#define SPEED_RAMP_DOWN -SPEED_RAMP_UP
#define SPEEDLOOP_SCALE 1

#endif

#define POSITION_RAMP_STEP 60.0f

#define START_INTERVAL 360.0f / (float)ENCODER_PPR / 4.0f / HARMONIC_AMPLIFY * 1.0f
#define STOP_INTERVAL 360.0f / (float)ENCODER_PPR / 4.0f / HARMONIC_AMPLIFY * 0.5f

//#define POSITIONLOOP_KP 200.0f
#define POSITIONLOOP_KP 250.0f
#define POSITIONLOOP_UPPER_LIMIT 3030.0f
#define POSITIONLOOP_LOWER_LIMIT -POSITIONLOOP_UPPER_LIMIT
#define POSITION_FILTER_OMEGA 20.0f

#define LOCATING_ACCURACY 360.0f / 4.0f / ENCODER_PPR * 1.0f
#define LOCATING_OFFSET 10.0f / 360.0f * 2.0f * PI

#if BIG_ID_ENABLE == 1
#define LOCATING_CURRENT 0.6f * OVER_CURRENT_THRESHOLD
#else
#define LOCATING_CURRENT 0.6f * OVER_CURRENT_THRESHOLD
#endif

#define LOCATING_DURATION 16000
#define HOLDING_DURATION 4000
#define BRAKING_DURATION 20000
#define ITERATION_TIMES 6

/******************************************************************************
* Functions
******************************************************************************/
void MotorDrive_Init(void);

void Enc_Speed_Cal(void);

void Enc_Speed_Cal_Pulse(void);

s32 Enc_Speed_Cal_Magnet(void);

void Rotor_Alignment(void);

void Encoder_Reset(void);

MCLIB_2_COOR_SYST_ALPHA_BETA_T Motor_Drive_Clarke(MCLIB_3_COOR_SYST_T I_abc);

MCLIB_2_COOR_SYST_D_Q_T Motor_Drive_Park(MCLIB_2_COOR_SYST_ALPHA_BETA_T I_alpha_beta, MCLIB_ANGLE_T Vector);

MCLIB_2_COOR_SYST_ALPHA_BETA_T Motor_Drive_Rev_Park(MCLIB_2_COOR_SYST_D_Q_T V_DQ, MCLIB_ANGLE_T Vector);

float Motor_Drive_Cur_PID_Regulator(float hReference, float hPresentFeedback, GFLIB_CONTROLLER_PI_P_PARAMS_T *PID_Struct);
float Motor_Drive_Spd_PID_Regulator(float hReference, float hPresentFeedback, GFLIB_CONTROLLER_PI_P_PARAMS_T *PID_Struct);
float Motor_Drive_Pos_PID_Regulator(float hReference, float hPresentFeedback, GFLIB_CONTROLLER_PI_P_PARAMS_T *PID_Struct);

void Motor_Drive_DutyCycleCal(MCLIB_2_COOR_SYST_ALPHA_BETA_T Stat_Volt_Input, float Udc);

void Motor_Drive_FOC(void);

void Motor_Drive_Get_Electrical_Angle(void);

void Current_Offset_Adjust(void);

void RampControl(float cmd, float step, float period, float *out);

void PositionCtrlInit(void);

void BrakeControl(void);

void MagnetEncoderOffsetCal(void);

void InitPositionLocating(void);

void PositionLocatingReset(void);

void FOC_Cal(void);

void PositionHold(void);

#ifdef HFI

void HFI_Search(void);

#endif

#endif  //_MOTORDRIVE_H_
