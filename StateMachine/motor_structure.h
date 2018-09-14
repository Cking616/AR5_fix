/******************************************************************************
*
* Motor control structure.
*
******************************************************************************/
#ifndef _MCSTRUC_H_
#define _MCSTRUC_H_

#include "stm32f4xx.h"
#include "Configuration.h"

typedef struct
{
    float f32A;
    float f32B;
    float f32C;
} MCLIB_3_COOR_SYST_T;

typedef struct
{
    float f32A;
    float f32B;
} MCLIB_2_COOR_SYST_T;

typedef struct
{
    float f32Alpha;
    float f32Beta;
} MCLIB_2_COOR_SYST_ALPHA_BETA_T;

typedef struct
{
    float f32D;
    float f32Q;
} MCLIB_2_COOR_SYST_D_Q_T;

typedef struct
{
    float f32Sin;
    float f32Cos;
} MCLIB_ANGLE_T;

typedef union
{
    unsigned long R;
    struct
    {
			  unsigned long UnderDCBusVoltage      : 1;   
        unsigned long OverDCBusVoltage       : 1;   
				unsigned long OverCurrent            : 1;   
				unsigned long Overload				       : 1;   
				unsigned long OverHeat               : 1;  
				unsigned long OverSpeed              : 1;   
				unsigned long LockRotor				       : 1;      
        unsigned long LossPhase				       : 1;   
				unsigned long MagnetEncoderError		 : 1;   
				unsigned long OffCancError           : 1;   
				unsigned long OpticalEncoderError	   : 1;   
				unsigned long BrakeError						 : 1;   
        unsigned long OppositeDirection    	 : 1;   
        unsigned long StartUpFail						 : 1;   	
        unsigned long FlashError						 : 1;   
				unsigned long MagnetEncoderCRCError  : 1;   
			
			
        unsigned long LostRotor							 : 1;   
        unsigned long OverCurrentRef				 : 1;   
        unsigned long CommunicationError		 : 1;   
        unsigned long TrackError					   : 1;   
				unsigned long CommunicationLost			 : 1;   
				unsigned long PowerSupplyOff				 : 1;   
        unsigned long PhaseWShortCirciut		 : 1;   
				unsigned long AngleCompareError			 : 1;   
        unsigned long BrakeStuckError				 : 1;   
				unsigned long SpeedDeltaError				 : 1;   
				unsigned long IdDeltaError				   : 1;   
				unsigned long IqDeltaError				   : 1;   
				unsigned long NegPhaseSeq 				   : 1;   
				unsigned long PosCmdDiffError 			 : 1;   
				unsigned long CPUOverUse             : 1;
				unsigned long               				 : 1;   

    } B;
} MCSTRUC_FAULT_STATUS_T;    


typedef struct
{
	float					f32UDcBusOver;							
	float					f32UDcBusUnder;							
	float					f32loadOver;		  					
	float					f32HeatOver;								
	float					f32CurrentOver;							
	float					f32OffsetOver;							
	float					f32SpeedOver;								
	float					f32LockRotorOver;						
	float					f32LossPhaseOver;						
	float					f32LostRotorOver;						
	u32						u32CommunicationCnt;				
	u32						u32CommunicationCntPre;
	u8						u8OpticalEncoderOver;
	u8            u8CommunicationStartFlag;   
	
	u32						u32UDcBusOverCnt;
	u32						u32UDcBusUnderCnt;
	u32						u32SpeedOverCnt;
	u32						u32OffCancErrorCnt;
	u32						u32OppositeDirectionCnt;
	u32						u32OpticalEncoderErrorCnt;
	u32						u32LostRotorErrorCnt;
	u32						u32LockRotorErrorCnt;
	u32						u32CurrentOverErrorCnt;
	u32						u32CommunicationErrorCnt;
	u32						u32BrakeErrorCnt;
	u32						u32CommunicationLostCnt;
	u32						u32OverHeatCnt;
	u32						u32OverloadCnt;
	u32						u32PhaseWShortCircuitCnt;
	u32						u32PhaseWShortCircuitCntPre;
	u32						u32PhaseWShortCircuitErrorCnt;
	u32           u32AngleCompareErrorCnt;
	u32						u32LossPhaseCnt;
	u32           u32SpeedDeltaErrorCnt;
	u32						u32IdDeltaErrorCnt;
	u32           u32IqDeltaErrorCnt;
	u32						u32MagnetEncoderErrorCnt;
	u32						u32MagnetEncoderCRCErrorCnt;
} MCSTRUC_FAULT_THRESHOLDS_T;


typedef struct
{
  float					f32PhA;					
  float					f32PhB;						
  float					f32Vdc; 	
} MCSTRUC_ADC_OFFSET_CH_T;


typedef struct
{
  float											f32Position;					
  float											f32U;					  		
  float											f32Speed;							
  float											f32UStep;							 
  float											f32IMax;							
  float											f32UMax;							
  unsigned short						uw16TimeAlignment;		
} MCSTRUC_ALIGNMENT_T;

typedef struct
{
   float f32PropGain;
   float f32IntegGain;
	 float f32DiffGain;
	 float f32AntiSatGain;
   float f32IntegPartK_1;
	 float f32IntegPartK;
	 float f32PropPartK;
	 float f32DiffPartK;
	 float f32AntiSatPartK;
   float f32UpperLimit;  
   float f32LowerLimit;    
   int   i16LimitFlag;
	 float f32ErrPartK;
	 float f32ErrPartK_1;
	 float f32OutputPartK;
	 float f32SampleTime;
	 float f32SampleFreq;
	 float f32IntegScale;
	 float f32FFPartK;
}GFLIB_CONTROLLER_PI_P_PARAMS_T;

typedef struct
{
	float			f32B1;
	float			f32B2;
	float			f32A2;
} GDFLIB_FILTER_IIR_COEFF1_T;

typedef struct 
{
	GDFLIB_FILTER_IIR_COEFF1_T	udtFiltCoeff;
	float 						f32FiltBufferX[2];
	float 						f32DummyVar;
	float 						f32FiltBufferY[2];    
} GDFLIB_FILTER_IIR1_T;


typedef struct
{
	float  						f32Vrms;   										
	float  						f32Vavg;   										
	float  						f32Vrect_avg;   									
}MCSTRUC_SIN_ANALYZER_T;



typedef struct
{
  GFLIB_CONTROLLER_PI_P_PARAMS_T  sIdPiParams;					
  GFLIB_CONTROLLER_PI_P_PARAMS_T  sIqPiParams;					
  GDFLIB_FILTER_IIR1_T						sIdZcFilter;		      
  GDFLIB_FILTER_IIR1_T						sIqZcFilter;		      
  GDFLIB_FILTER_IIR1_T						sUDcBusFilter;	     
  MCLIB_3_COOR_SYST_T    					sIABC;				     	 	
	MCLIB_3_COOR_SYST_T    					sIABC_Peak;						
	MCLIB_3_COOR_SYST_T    					sIABC_RMS;						
  MCLIB_2_COOR_SYST_ALPHA_BETA_T	sIAlBe;   						
  MCLIB_2_COOR_SYST_D_Q_T					sIDQ;     			    	
	MCLIB_2_COOR_SYST_D_Q_T					sIDQ_Peak;     			  
  MCLIB_2_COOR_SYST_D_Q_T					sIDQReq;  			   	 	
  MCLIB_2_COOR_SYST_D_Q_T					sIDQReqZc;  		   		
  MCLIB_2_COOR_SYST_D_Q_T					sIDQError;  		    	
  MCLIB_3_COOR_SYST_T    					sDutyABC;			      	
  MCLIB_2_COOR_SYST_ALPHA_BETA_T	sUAlBeReq;   					
	MCLIB_2_COOR_SYST_ALPHA_BETA_T	sUAlBeReq_Peak;   		
  MCLIB_2_COOR_SYST_ALPHA_BETA_T	sUAlBeComp;	 					
  MCLIB_2_COOR_SYST_D_Q_T					sUDQReq;     		    	
  MCLIB_2_COOR_SYST_D_Q_T					sUDQController; 	  	
	MCLIB_2_COOR_SYST_D_Q_T					sUDQReq_Peak;     		
	MCLIB_ANGLE_T										sAnglePosEl; 		      
	MCLIB_ANGLE_T										sAnglePosElUpdate;	  
  MCSTRUC_ALIGNMENT_T							sAlignment;		        
	MCSTRUC_SIN_ANALYZER_T					sIA_Analyzer;
	MCSTRUC_SIN_ANALYZER_T					sIB_Analyzer;
	MCSTRUC_SIN_ANALYZER_T					sIC_Analyzer;
  unsigned short 									uw16SectorSVM;		    
	float														f32DutyCycleLimit;	  
	float														f32UDcBus;			      
	float														f32UDcBusFilt;		   
	int 					 								 	i16IdPiSatFlag;		    
	int 					  								i16IqPiSatFlag;		    
	unsigned char										bOpenLoop;			      
	unsigned char										bUseMaxBus;			      
	unsigned char										bUseZc;				        
	unsigned char 									bOverModulation;		  
} MCSTRUC_FOC_PMSM_T;


typedef struct
{
  GDFLIB_FILTER_IIR1_T						sFwErrorFilter;	    							
  GFLIB_CONTROLLER_PI_P_PARAMS_T  sFwPiParams;							
	float														f32UFwError;		          
	float														f32IFwError;		          			
	float														f32FwError;			          		
	float														f32FwErrorFilt;		        				
	int 					        					i16FwPiSatFlag;		    		
	float														f32SpeedFwOn;		         
	float														f32ILimit;	              
} MCSTRUC_FOC_FW_T;


typedef struct
{
    float f32RampUp;
    float f32RampDown;
} GFLIB_RAMP16_T;


typedef struct
{
	GDFLIB_FILTER_IIR1_T						sSpeedFilter;		  							
	GFLIB_CONTROLLER_PI_P_PARAMS_T  sSpeedPiParams;						
	GFLIB_CONTROLLER_PI_P_PARAMS_T  sSpeedPiParamsSet;
	GFLIB_RAMP16_T									sSpeedRampParams; 				
	float														f32Speed;			            
	float														f32SpeedStart;			      
	float														f32SpeedFilt;		          
	float														f32SpeedError;		        
	float														f32SpeedRamp;		          
	float														f32SpeedRampStep;		      
	float														f32SpeedAccelerationSet;	
	float														f32SpeedAcceleration;			
	float														f32SpeedJerkSet;					
	float														f32SpeedJerk;							
	float														f32SpeedReq;		          
	float														f32SpeedCmd;		          
	float														f32SpeedCmdPre;		        
	float														f32SpeedFF;			          
	float														f32SpeedFF_Pre;			      
	int 					    							i16SpeedPiSatFlag;	  		
  short														w16SpeedScale;		      	
	short														w16SpeedScaleCnt;		      
	unsigned char										bOpenLoop;			         
} MCSTRUC_SPEED_T;

typedef struct
{
	MCLIB_ANGLE_T					sAnglePosEl; 				    				
	MCLIB_ANGLE_T					sAnglePosElUpdate;							
	GDFLIB_FILTER_IIR1_T	sSpeedFilter;				    							
	float									f32SpeedCoef;				    				
	short									w16EncPulsesRevMech;						
	unsigned short				uw16PositionMechRawMask;				
	short									f16PositionMechScale;						
	short									w16PositionMechScaleShift;			
	unsigned short				uw16PositionMechRaw;						
	short									w16PolePairs;				    				
	float									f32PositionMech;			  				
	float									f32PositionEl;				  				
	float									f32PositionElUpdate;						
	s32   								s32EncCaptured;				    		
	s32 									s32TimerCaptured;			    			
	s32   								s32EncCapturedOld;			  			
	s32 									s32TimerCapturedOld;		  			
	unsigned short 				uw16NewPulse;				      			
	short 								w16EncDifferenceMax;		  			
	float									f32Speed;					        			
	float									f32SpeedFilt;				      		
	float									f32Omega;					        			
	s32										s32MagnetOffset;								
	float									f32MagnetOffsetRadEl;						
	s32										s32EncoderTurns;								
	s32										s32PulseCaptured;
	s32										s32PulseCapturedPre;
	float									f32MagnetEncoderSpeed;					
	float									f32MagnetEncoderSpeedFilt;			
	float									f32MagnetEncoderSpeedFiltPrev;	
	float									f32MagnetEncoderAcceleration;		
	float									f32MagnetEncoderAccelerationFlt;
	u32										u32MagnetEncoderPosition;				
	u32										u32MagnetEncoderPositionPrev;		
	float									f32PulsesPerMagnetBit;					
  u8										u8MagnetEncoderErrCode; 				
	u32   								u32IndexPulseCaptured;				  
	u32   								u32IndexPulseCapturedOld;				
	s32   								s32DeltaIndexPulseCaptured;			
	u8    								u8IndexPulseCaptureEnable;			
	u8    								u8MagnetEncoderUpdateFlag;		
	u8    								u8MagnetEncInitPositionCheck;		
	u8    								u8MagnetEncoderCalib;					
	u8										u8MagnetEncoderGeneralError;
	u8										u8MagnetEncoderCRCError;
} MCSTRUC_POS_SPEED_ENCODER_T;


typedef struct 
{
	float 						f32FiltBufferM;
	float 						f32FiltBufferM_K_1;
	float 						f32Omega;
	float 						f32FiltBufferY;
	float 						f32FiltBufferY_K_1;    
} GDFLIB_FILTER_IIR2_T;



typedef struct
{
	GDFLIB_FILTER_IIR2_T						sPositionFilter;		  			
	GFLIB_CONTROLLER_PI_P_PARAMS_T  sPositionPiParams;					
	GFLIB_CONTROLLER_PI_P_PARAMS_T  sPositionPiParamsSet;										
	float														f32Position;			            
	float														f32PositionMechnicalStart;		
	float														f32PositionComp;		          
	float														f32PositionCompPre;		        
	float														f32PositionError;		         
	float														f32PositionRampOut;		        
	float														f32PositionRampStep;		      
	float														f32PositionRampOutFilt;		   
	float														f32PositionCmd;		            
	float														f32PositionCmdPrevious;		    
	float														f32PositionStart;		    			
	float														f32StartInterval;		    			
	float														f32StopInterval;		    		
	unsigned char										bPositionTargetChange;			  
	unsigned char										bOpenLoop;			              
	float 													f32FeedforwardTorque;					
	u8															u8PositionSettleFlag;
	u16															u16PositionSettleCnt;
	float														f32AccelerationFF;
	float														f32PositionPre;			          
	float														f32PositionTurns;			        
	float														f32PositionLastCheck;			    
	float														f32PositionLastCheckDelta;		
	
} MCSTRUC_POSITION_T;



typedef struct
{
	float 													f32LocatingCurrent;						
	float 													f32Theta_H;										
	float 													f32Theta_L;										
	float 													f32Theta_M;										
	float 													f32Theta_M_Pre;								
	float 													f32Theta_Offset;							
	float 													f32Theta_M_Delta;							
	float 													f32PositionMech;							
	float 													f32PositionMechPre;						
	s32 														s32TurnsMech;									
	s32 														s32TurnsMechPre;							
	float 													f32AccuracyLimit;							
	u8															u8LocatingStep;								
	u8															u8LocatingCnt;								
	u16															u16LocatingPrescaler;				
	u8															u8BrakeFlag;									
	u8                              u8StartDone;
	u8                              u8BrakeStuck;
	u8															u8OffsetPosition1Get;
	u32															u32StartCnt;
	u32															u32LockCnt;
	u32															u32Delay;
	float														f32StuckPosition;
	u8															u8BrakeCnt;
	float														f32LastMoveCheck;
} MCSTRUC_INIT_LOCATING_T;


typedef struct
{
#if   BIG_ID_ENABLE == 1         
	#define HFI_FREQ										1000.0f												
	#define HFI_PERIOD									20.0f				
	#define HFI_AMPLITUDE								15.0f												
	#define IQ_AVG_THRESHOLD            0.05f
	#define IQ_AVG_DURATION							20000
	#define EST_ANGLE_THRESHOLD					90.0f
	#define MAX_I_RESPONSE_CNT          5000
	#define HFI_DURATION								100
	#define PULSE_VOLTAGE								6.0f
	#define IIR2_FREQ										300.0f
	#define STUCK_CHECK_CURRENT					12.0f
	#define PULSE_WIDTH									10
	#define	JITTER_COUNT								4000
	
#else
	#define HFI_FREQ										625.0f									
	#define HFI_PERIOD									32.0f		
	#define HFI_AMPLITUDE								25.0f													
	#define MAX_I_RESPONSE_CNT          5000
	#define HFI_DURATION								100		
	#define PULSE_VOLTAGE								20.0f
	#define IIR2_FREQ										300.0f
	#define STUCK_CHECK_CURRENT					8.0f
	#define PULSE_WIDTH									12
	#define	JITTER_COUNT								8000
#endif

	
	u8																u8Step;
	float															f32EstAngle;
	u32																u32EstAngleCnt;
	float															f32RadCnt;
	float															f32IqAvg;
	u32																u32IqAvgOverDuration;
	u32																u32IqAvgUnderDuration;
	float															f32Offset;
	GFLIB_CONTROLLER_PI_P_PARAMS_T    sHFIPiParams;
	GDFLIB_FILTER_IIR2_T						  sFilter;
	float															f32SearchingAngle[5];
	float															f32ResponseCurrent[5];
	u8																u8SearchingAngleCnt;
	u16																u16ResponseCurrentCnt;
	float															f32OffsetFinal;
	float 														f32IdsinFlt;
	float 														f32IdcosFlt;
	float															f32Idh;
	float															f32IdhPre;
	u32 															u32HFI_Duration;
	float 														f32IqsinFlt;
	float 														f32IqcosFlt;
	float															f32Iqh;
	GDFLIB_FILTER_IIR2_T						  sSinFilter;
	GDFLIB_FILTER_IIR2_T						  sCosFilter;
	u32																u32CtrlDelay;
	u32																u32Scale;
} MCSTRUC_HFI_SEARCH_T;


typedef struct
{
	MCSTRUC_FAULT_STATUS_T					sFaultId;
	MCSTRUC_FAULT_THRESHOLDS_T			sFaultThresholds;
	MCSTRUC_ADC_OFFSET_CH_T					sADCOffset;
	MCSTRUC_FOC_PMSM_T							sFocPMSM;
	MCSTRUC_POS_SPEED_ENCODER_T			sPositionEnc;
	MCSTRUC_SPEED_T									sSpeed;
	MCSTRUC_POSITION_T							sPositionControl;
	MCSTRUC_INIT_LOCATING_T 				sInitLocating;
	
#ifdef HFI
	MCSTRUC_HFI_SEARCH_T            sHFISearch;
#endif

	unsigned short               		uw16CtrlMode;
	float														f32MotorTemp;
} MCSTRUC_FOC_PMSM_ENCODER_T;



#endif /* _MCSTRUC_H_ */
