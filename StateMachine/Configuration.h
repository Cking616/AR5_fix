#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H


#include "stm32f4xx.h"



#define   HARDWARE_VERSION_2_0									0
#define   HARDWARE_VERSION_2_1									0
#define   HARDWARE_VERSION_2_2									1
#define   ETHERCAT_ENABLE
//#define   ETHERCAT_RUN
//#define   SYSVIEW_DEBUG
//#define   USB_ENABLE

#define		ZZ_MOTOR															1
#define		SMALL_PPR_982													0
#define		MAGNET_ENCODER_FBK										1
#define		NEW_MAG_FIXTURE												1

#define   Modbus_RTU_ENABLE                     0
#define   BIG_ID_ENABLE                         1

#define		HARDWARE_TEST													0			
#define   RENISHAW															1
#define   KOLLMORGEN														0


#define   AUTORUN																1
#define   AUTORUN_SPEED													500   


#if BIG_ID_ENABLE == 1
	#define   HALL_SENSOR                         1
#else
	#define   HALL_SENSOR                         0
#endif

#if BIG_ID_ENABLE == 1
#define		R_SAMPLE															0.0005f 
#else
#define		R_SAMPLE															0.002f 
#endif

#define		MAGNET_ENCODER_ALERT_ON							  1

#define   IS_17BITS_MAGNET_ENCODER							1					
																															
#define   INIT_LOCATING													0

#define   COMM_ERROR_ALERT											1		

#define   TORQUE_FEEDFORWARD										0

#define   ANGLE_SEARCHING    										0

#define   STUCK_CHECK                           0

#define		HFI																		


/* ---------------ADC--------------*/

#define PHASE_A_ADC_CHANNEL     ADC_Channel_10
#define PHASE_B_ADC_CHANNEL     ADC_Channel_11


#define SAMPLING_TIME_NS   700 


#if (SAMPLING_TIME_NS == 200)
#define SAMPLING_TIME_CK  ADC_SampleTime_3Cycles
#elif (SAMPLING_TIME_NS == 700)
#define SAMPLING_TIME_CK  ADC_SampleTime_15Cycles
#elif (SAMPLING_TIME_NS == 1200)
#define SAMPLING_TIME_CK  ADC_SampleTime_28Cycles
#elif (SAMPLING_TIME_NS == 2450)
#define SAMPLING_TIME_CK  ADC_SampleTime_56Cycles
#else
#warning "Sampling time is not a possible value"
#endif


#define CKTIM    ((u32)168000000uL)     


#define PWM_PRSC ((u8)0)



#define PWM_FREQ 								((u16) 20000) 		
#define SAMPLE_PERIOD						0.00005f
#define SPEEDLOOP_PERIOD 				0.001f

              
#define PWM_PERIOD ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) 


#define DEADTIME_NS    ((u16) 1000) 


#define MDEADTIME  (u16)((unsigned long long)CKTIM/2 *(unsigned long long)DEADTIME_NS/1000000000uL) 
	

#define REP_RATE (1)  

#define SAMPLING_FREQ   ((u16)PWM_FREQ/((REP_RATE+1)/2))  



void GPIO_Config(void);

void TIM_Config(void);

void ADC_Config(void);

void ENC_Config(void);

void DMA_Config(void);

void SPI3_Config(void);

void PWM_Config(void);

void NVIC_Config(void);

#ifdef ETHERCAT_ENABLE
void SPI2_Config(void);
#endif

#ifdef USB_ENABLE
void USBConfig(void);
#endif 

#endif

