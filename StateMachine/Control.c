
#include "Control.h"
#include "ARM_MATH.h"
#include "Configuration.h"
#include "M1_statemachine.h"
#include "Motor_Drive.h"

static u8 tableCRC[256] = {
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
    0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
    0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
    0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
    0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
    0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
    0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
    0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
    0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
    0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
    0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
    0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
    0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
    0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
    0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C};

#define CNT_10_MS 10
#define CNT_50_MS 5
#define CNT_100_MS 2
#define CNT_500_MS 5
#define CNT_1000_MS 2

#define T_AV_ARRAY_SIZE 32.0F

Curr_Components Stat_Curr_a_b;
Curr_Components Stat_Curr_alpha_beta;
Curr_Components Stat_Curr_q_d;
Volt_Components Stat_Volt_a_b;
Volt_Components Stat_Volt_q_d;
Volt_Components Stat_Volt_alpha_beta;
volatile Curr_Components Stat_Curr_q_d_ref;

volatile u8 counter_1_ms = 0;
volatile u8 counter_10_ms = 0;
volatile u8 counter_50_ms = 0;
volatile u8 counter_100_ms = 0;
volatile u8 counter_500_ms = 0;

volatile u8 Flag_2_ms = 0;
volatile u8 Flag_10_ms = 0;
volatile u8 Flag_50_ms = 0;
volatile u8 Flag_100_ms = 0;
volatile u8 Flag_500_ms = 0;
volatile u8 Flag_1000_ms = 0;
volatile int Tick_ms = 0;

extern uint8_t SPI3_Rx_Buff[];
extern __IO u8 guc_RS485_Error_Flag;

float Ext_Temperature;
float h_BusV_Average;
static volatile u32 hTimebase = 0;
s32 Turn_Number = 0;
float w_Temp_Average = 700.0f;
u16 h_ADCBusvolt = 0;
u16 h_ADCTemp = 0;
u16 ADC_MCU_Temp = 0;
u16 ADC_Vrefint = 0;
u16 ADC_Motor_Temp;
float fMUC_TEMP = 25.0f;
unsigned char AlignmentDone;

u32 Flash_Read_Buffer[BUFFER_SIZE];
u32 Flash_Write_Buffer[BUFFER_SIZE];

void Led_Fault_Indication_Set(void) {
#if HARDWARE_VERSION_2_0 == 1
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
#endif

#if (HARDWARE_VERSION_2_1 == 1) || (HARDWARE_VERSION_2_2 == 1)
#if BIG_ID_ENABLE == 1
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
#else
#if HARDWARE_VERSION_2_2
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
#else
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
#endif

#endif
#endif
}

void Led_Fault_Indication_Clr(void) {
#if HARDWARE_VERSION_2_0 == 1
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
#endif

#if (HARDWARE_VERSION_2_1 == 1) || (HARDWARE_VERSION_2_2 == 1)
#if BIG_ID_ENABLE == 1
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
#else
#if HARDWARE_VERSION_2_2
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
#else
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
#endif
#endif
#endif
}

void gLEDState(void) {
#if HARDWARE_VERSION_2_0 == 1
    GPIO_ToggleBits(GPIOB, GPIO_Pin_13);
#endif

#if (HARDWARE_VERSION_2_1 == 1) || (HARDWARE_VERSION_2_2 == 1)
#if BIG_ID_ENABLE == 1
    GPIO_ToggleBits(GPIOC, GPIO_Pin_8);
#else
#if HARDWARE_VERSION_2_2
    GPIO_ToggleBits(GPIOC, GPIO_Pin_8);
#else
    GPIO_ToggleBits(GPIOB, GPIO_Pin_13);
#endif
#endif
#endif
}

void One_ms_Tick(void) {
    counter_1_ms++;
	Tick_ms++;
	
    if (counter_1_ms % 2 == 0) {
        Flag_2_ms = 1;
    }

    if (counter_1_ms >= CNT_10_MS) {
        counter_1_ms = 0;
        counter_10_ms++;
        Flag_10_ms = 1;
    }

    if (counter_10_ms >= CNT_50_MS) {
        counter_10_ms = 0;
        counter_50_ms++;
        Flag_50_ms = 1;
    }

    if (counter_50_ms >= CNT_100_MS) {
        counter_50_ms = 0;
        counter_100_ms++;
        Flag_100_ms = 1;
    }

    if (counter_100_ms >= CNT_500_MS) {
        counter_100_ms = 0;
        counter_500_ms++;
        Flag_500_ms = 1;
    }

    if (counter_500_ms >= CNT_1000_MS) {
        counter_500_ms = 0;
        Flag_1000_ms = 1;
    }

    if (hTimebase != 0) {
        hTimebase--;
    }
}

void TB_Set_Delay(u16 hDelay) {
    hTimebase = hDelay;
}

u8 TB_delay_IsElapsed(void) {
    if (hTimebase == 0) {
        return (1);
    } else {
        return (0);
    }
}

u8 crc8_4B(u32 bb) {
    u8 crc;
    u32 t;
    t = (bb >> 24) & 0x000000FF;
    crc = ((bb >> 16) & 0x000000FF);
    t = crc ^ tableCRC[t];
    crc = ((bb >> 8) & 0x000000FF);
    t = crc ^ tableCRC[t];
    crc = (bb & 0x000000FF);
    t = crc ^ tableCRC[t];
    crc = tableCRC[t];
    return crc;
}

void MagnetEncDataRead(void) {
#if RENISHAW == 1

#if HARDWARE_VERSION_2_2

    SPI_I2S_ReceiveData(SPI2);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);

    SPI_Cmd(SPI2, ENABLE);
    SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);

    DMA_Cmd(DMA1_Stream3, DISABLE);
    DMA1_Stream0->NDTR = 5;
    DMA_Cmd(DMA1_Stream3, ENABLE);

#else

    SPI_I2S_ReceiveData(SPI3);
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);

    SPI_Cmd(SPI3, ENABLE);
    SPI_I2S_ClearITPendingBit(SPI3, SPI_I2S_IT_RXNE);

    DMA_Cmd(DMA1_Stream0, DISABLE);
    DMA1_Stream0->NDTR = 5;
    DMA_Cmd(DMA1_Stream0, ENABLE);

#endif

#else

    GPIO_SetBits(GPIOA, GPIO_Pin_15);

    SPI_Cmd(SPI3, ENABLE);
    SPI_I2S_ClearITPendingBit(SPI3, SPI_I2S_IT_RXNE);

    DMA_Cmd(DMA1_Stream0, ENABLE);

#endif
}

void Fault_Management(void) {
    if ((gsM1_Ctrl.eState == RUN) && (meM1_StateRun == READY)) {
    } else {
        if (gsM1_Drive.sFocPMSM.f32UDcBusFilt < gsM1_Drive.sFaultThresholds.f32UDcBusUnder) {
            gsM1_Drive.sFaultThresholds.u32UDcBusUnderCnt++;
        } else {
            gsM1_Drive.sFaultThresholds.u32UDcBusUnderCnt = 0;
        }

        if (gsM1_Drive.sFaultThresholds.u32UDcBusUnderCnt >= 2000) {
            gsM1_Drive.sFaultId.B.UnderDCBusVoltage = 1;
            gsM1_Drive.sFaultThresholds.u32UDcBusUnderCnt = 0;
        }
    }

    if (gsM1_Drive.sFocPMSM.f32UDcBusFilt > gsM1_Drive.sFaultThresholds.f32UDcBusOver) {
        gsM1_Drive.sFaultThresholds.u32UDcBusOverCnt++;
    } else {
        gsM1_Drive.sFaultThresholds.u32UDcBusOverCnt = 0;
    }

    if (gsM1_Drive.sFaultThresholds.u32UDcBusOverCnt >= 2000) {
        gsM1_Drive.sFaultId.B.OverDCBusVoltage = 1;
        gsM1_Drive.sFaultThresholds.u32UDcBusOverCnt = 0;
    }

    if ((Ext_Temperature > OVER_HEAT_UPPER_THRESHOLD) || (Ext_Temperature < OVER_HEAT_LOWER_THRESHOLD)) {
        gsM1_Drive.sFaultThresholds.u32OverHeatCnt++;
    } else {
        gsM1_Drive.sFaultThresholds.u32OverHeatCnt = 0;
    }
    if (gsM1_Drive.sFaultThresholds.u32OverHeatCnt >= 5000) {
        gsM1_Drive.sFaultId.B.OverHeat = 1;
        gsM1_Drive.sFaultThresholds.u32OverHeatCnt = 0;
    }

    if (fabsf(gsM1_Drive.sPositionEnc.f32SpeedFilt) > gsM1_Drive.sFaultThresholds.f32SpeedOver) {
        gsM1_Drive.sFaultThresholds.u32SpeedOverCnt++;
    } else {
        gsM1_Drive.sFaultThresholds.u32SpeedOverCnt = 0;
    }

    if (gsM1_Drive.sFaultThresholds.u32SpeedOverCnt >= 1000) {
        gsM1_Drive.sFaultId.B.OverSpeed = 1;
        gsM1_Drive.sFaultThresholds.u32SpeedOverCnt = 0;
    }

    if ((gsM1_Drive.sHFISearch.u8Step >= 12) && (gsM1_Drive.uw16CtrlMode != TORQUE_CONTROL) && (fabsf(gsM1_Drive.sPositionEnc.f32SpeedFilt) <= 0.2f) && (fabsf(gsM1_Drive.sFocPMSM.sIDQ.f32Q) >= gsM1_Drive.sFaultThresholds.f32LockRotorOver)) {
        gsM1_Drive.sFaultThresholds.u32LockRotorErrorCnt++;
    } else {
        gsM1_Drive.sFaultThresholds.u32LockRotorErrorCnt = 0;
    }

    if (gsM1_Drive.sFaultThresholds.u32LockRotorErrorCnt >= 2) {
        gsM1_Drive.sFaultId.B.LockRotor = 1;
        gsM1_Drive.sFaultThresholds.u32LockRotorErrorCnt = 0;
    }

    if ((fabsf(gsM1_Drive.sADCOffset.f32PhA - DEFAULT_I_A_OFFSET) > gsM1_Drive.sFaultThresholds.f32OffsetOver) ||
        (fabsf(gsM1_Drive.sADCOffset.f32PhB - DEFAULT_I_B_OFFSET) > gsM1_Drive.sFaultThresholds.f32OffsetOver)) {
        gsM1_Drive.sFaultThresholds.u32OffCancErrorCnt++;
    } else {
        gsM1_Drive.sFaultThresholds.u32OffCancErrorCnt = 0;
    }

    if (gsM1_Drive.sFaultThresholds.u32OffCancErrorCnt >= 100) {
        gsM1_Drive.sFaultId.B.OffCancError = 1;
        gsM1_Drive.sFaultThresholds.u32OffCancErrorCnt = 0;
    }

    if ((gsM1_Drive.sPositionEnc.f32SpeedFilt * gsM1_Drive.sFocPMSM.sIDQReq.f32Q < 0) && (gsM1_Drive.uw16CtrlMode == SPEED_CONTROL)) {
        gsM1_Drive.sFaultThresholds.u32OppositeDirectionCnt++;
    } else {
        gsM1_Drive.sFaultThresholds.u32OppositeDirectionCnt = 0;
    }

    if (gsM1_Drive.sFaultThresholds.u32OppositeDirectionCnt >= 500) {
        gsM1_Drive.sFaultId.B.OppositeDirection = 1;
        gsM1_Drive.sFaultThresholds.u32OppositeDirectionCnt = 0;
    }

#if COMM_ERROR_ALERT == 1
    if (guc_RS485_Error_Flag == 0) {
        gsM1_Drive.sFaultThresholds.u32CommunicationErrorCnt = 0;

    } else {
        gsM1_Drive.sFaultThresholds.u32CommunicationErrorCnt++;
    }

    if (gsM1_Drive.sFaultThresholds.u32CommunicationErrorCnt >= 10) {
        gsM1_Drive.sFaultId.B.CommunicationError = 1;
        gsM1_Drive.sFaultThresholds.u32CommunicationErrorCnt = 0;
    }

    if (gsM1_Drive.sFaultThresholds.u8CommunicationStartFlag >= 3) {
        if (gsM1_Drive.sFaultThresholds.u32CommunicationCnt != gsM1_Drive.sFaultThresholds.u32CommunicationCntPre) {
            gsM1_Drive.sFaultThresholds.u32CommunicationCntPre = gsM1_Drive.sFaultThresholds.u32CommunicationCnt;
            gsM1_Drive.sFaultThresholds.u32CommunicationLostCnt = 0;
        } else {
            gsM1_Drive.sFaultThresholds.u32CommunicationLostCnt++;
        }
    } else {
        gsM1_Drive.sFaultThresholds.u32CommunicationLostCnt = 0;
    }
    if (gsM1_Drive.sFaultThresholds.u32CommunicationLostCnt >= 20) {
        gsM1_Drive.sFaultId.B.CommunicationLost = 1;
        gsM1_Drive.sFaultThresholds.u32CommunicationLostCnt = 0;
    }

#endif

    if ((gsM1_Drive.sFocPMSM.f32UDcBusFilt < POWER_SUPPLY_OFF_THRESHOLD) && (gsM1_Ctrl.eState == RUN)) {
        gsM1_Drive.sFaultId.B.PowerSupplyOff = 1;
    }
}

void Ext_Temp_Sensor_Cal(void) {
    float R;
    float Temp;

    w_Temp_Average = ((T_AV_ARRAY_SIZE - 1.0f) * w_Temp_Average + (float)h_ADCTemp) / T_AV_ARRAY_SIZE;

    R = w_Temp_Average / 4096.0F * 3.3F * 1000.0F;

    Temp = R - 424.0F;

    Ext_Temperature = Temp / 6.25F;

    static float Motor_TempSensor_Res;
    Motor_TempSensor_Res = 549.0f * (float)ADC_Motor_Temp / (4096.0f - (float)ADC_Motor_Temp);
    gsM1_Drive.f32MotorTemp = 0.9f * gsM1_Drive.f32MotorTemp + 0.1f * (-3.9215f + sqrtf(3.9215f * 3.9215f - 0.044f * (497.55f - Motor_TempSensor_Res))) / 0.022f;
}

void MCU_Temp_Cal(void) {
    fMUC_TEMP = (25.0f + ((float)(ADC_MCU_Temp)*3.3f / 4096.0f - 0.76f) / 0.0025f) * 0.1f + fMUC_TEMP * 0.9f;
}

float   g_fq_on_over_current = 0.0f;
int     g_pha_value_on_over_current = 0;
int     g_phb_value_on_over_current = 0;
float   g_offseta_value_on_over_current = 0.0f;
float   g_offsetb_value_on_over_current = 0.0f;
void ADC_Value_Read(void) {
	  int pha = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    int phb = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
#if HALL_SENSOR == 1


#if HARDWARE_VERSION_2_2
  	gsM1_Drive.sFocPMSM.sIABC.f32A = ((float)pha - gsM1_Drive.sADCOffset.f32PhA) * 1.65f / 9.1f * 13.8f / 0.02f / 2048;
		gsM1_Drive.sFocPMSM.sIABC.f32B = ((float)phb - gsM1_Drive.sADCOffset.f32PhB) * 1.65f / 9.1f * 13.8f / 0.02f / 2048;
#else
		gsM1_Drive.sFocPMSM.sIABC.f32A = ((float)pha - gsM1_Drive.sADCOffset.f32PhA) * 1.65f / 10.0f * 14.7f / 0.066f / 2048;
		gsM1_Drive.sFocPMSM.sIABC.f32B = ((float)phb - gsM1_Drive.sADCOffset.f32PhB) * 1.65f / 10.0f * 14.7f / 0.066f / 2048;
#endif

#else
	#if HARDWARE_VERSION_2_2
  	gsM1_Drive.sFocPMSM.sIABC.f32A = ((float)pha - gsM1_Drive.sADCOffset.f32PhA) * 1.65f / 9.1f * 13.8f / 0.066f / 2048;
	gsM1_Drive.sFocPMSM.sIABC.f32B = ((float)phb - gsM1_Drive.sADCOffset.f32PhB) * 1.65f / 9.1f * 13.8f / 0.066f / 2048;
#else
    /* Res AD8417 */
    gsM1_Drive.sFocPMSM.sIABC.f32A = (((float)pha - gsM1_Drive.sADCOffset.f32PhA) / 2048.0f) * 1.65f / (60.0f * R_SAMPLE);
    gsM1_Drive.sFocPMSM.sIABC.f32B = (((float)phb - gsM1_Drive.sADCOffset.f32PhB) / 2048.0f) * 1.65f / (60.0f * R_SAMPLE);
#endif

#endif

    gsM1_Drive.sFocPMSM.sIABC.f32C = -gsM1_Drive.sFocPMSM.sIABC.f32A - gsM1_Drive.sFocPMSM.sIABC.f32B;

    if (fabsf(gsM1_Drive.sFocPMSM.sIABC.f32A) > gsM1_Drive.sFocPMSM.sIABC_Peak.f32A) {
        g_pha_value_on_over_current = pha;
        gsM1_Drive.sFocPMSM.sIABC_Peak.f32A = fabsf(gsM1_Drive.sFocPMSM.sIABC.f32A);
    }
    if (fabsf(gsM1_Drive.sFocPMSM.sIABC.f32B) > gsM1_Drive.sFocPMSM.sIABC_Peak.f32B) {
        g_phb_value_on_over_current = phb;
        gsM1_Drive.sFocPMSM.sIABC_Peak.f32B = fabsf(gsM1_Drive.sFocPMSM.sIABC.f32B);
    }
    if (fabsf(gsM1_Drive.sFocPMSM.sIABC.f32C) > gsM1_Drive.sFocPMSM.sIABC_Peak.f32C) {
        gsM1_Drive.sFocPMSM.sIABC_Peak.f32C = fabsf(gsM1_Drive.sFocPMSM.sIABC.f32C);
    }

    if (gsM1_Drive.uw16CtrlMode != OPENLOOP_PWM) {
        if ((fabsf(gsM1_Drive.sFocPMSM.sIABC.f32A) > PEAK_CURRENT_THRESHOLD) ||
            (fabsf(gsM1_Drive.sFocPMSM.sIABC.f32B) > PEAK_CURRENT_THRESHOLD) ||
            (fabsf(gsM1_Drive.sFocPMSM.sIABC.f32C) > PEAK_CURRENT_THRESHOLD) ||
            (fabsf(gsM1_Drive.sFocPMSM.sIDQ.f32D) > PEAK_CURRENT_THRESHOLD) ||
            (fabsf(gsM1_Drive.sFocPMSM.sIDQ.f32Q) > PEAK_CURRENT_THRESHOLD)) {
            gsM1_Drive.sFaultThresholds.u32CurrentOverErrorCnt++;
        } else {
            gsM1_Drive.sFaultThresholds.u32CurrentOverErrorCnt = 0;
        }

        if (gsM1_Drive.sFaultThresholds.u32CurrentOverErrorCnt >= 3) {
            g_fq_on_over_current = gsM1_Drive.sFocPMSM.sIDQReq.f32Q;
            g_pha_value_on_over_current = pha;
            g_phb_value_on_over_current = phb;
            g_offseta_value_on_over_current = gsM1_Drive.sADCOffset.f32PhA;
            g_offsetb_value_on_over_current = gsM1_Drive.sADCOffset.f32PhB;
            gsM1_Drive.sFaultId.B.OverCurrent |= 1;
            PWM_OUT_DISABLE();
            gsM1_Drive.sFaultThresholds.u32CurrentOverErrorCnt = 0;
        }
    }

    ADC_Motor_Temp = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
    ADC_MCU_Temp = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);

    h_ADCBusvolt = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);
    h_ADCTemp = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_3);

    gsM1_Drive.sFocPMSM.f32UDcBus = (float)(h_ADCBusvolt) / 4096.0F * 3.3F / 5.6F * 105.6F;
    gsM1_Drive.sFocPMSM.f32UDcBusFilt = 0.98f * gsM1_Drive.sFocPMSM.f32UDcBusFilt + 0.02f * gsM1_Drive.sFocPMSM.f32UDcBus;
}

void PWM_OUT_ENABLE(void) {
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM_OUT_DISABLE(void) {
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

void V_AlphaBeta_Reset(void) {
    Stat_Volt_alpha_beta.qV_Component1 = 0;
    Stat_Volt_alpha_beta.qV_Component2 = 0;

    gsM1_Drive.sFocPMSM.sUAlBeReq.f32Alpha = 0;
    gsM1_Drive.sFocPMSM.sUAlBeReq.f32Beta = 0;
}

void I_AlphaBeta_Reset(void) {
    Stat_Curr_alpha_beta.qI_Component1 = 0;
    Stat_Curr_alpha_beta.qI_Component2 = 0;

    gsM1_Drive.sFocPMSM.sIAlBe.f32Alpha = 0;
    gsM1_Drive.sFocPMSM.sIAlBe.f32Beta = 0;
}

void V_QD_Reset(void) {
    Stat_Volt_q_d.qV_Component1 = 0;
    Stat_Volt_q_d.qV_Component2 = 0;

    gsM1_Drive.sFocPMSM.sUDQReq.f32D = 0;
    gsM1_Drive.sFocPMSM.sUDQReq.f32Q = 0;

    gsM1_Drive.sFocPMSM.sUDQController.f32D = 0;
    gsM1_Drive.sFocPMSM.sUDQController.f32Q = 0;
}

void I_QD_Reset(void) {
    Stat_Curr_q_d.qI_Component1 = 0;
    Stat_Curr_q_d.qI_Component2 = 0;

    gsM1_Drive.sFocPMSM.sIDQ.f32D = 0;
    gsM1_Drive.sFocPMSM.sIDQ.f32Q = 0;
}

void I_QD_Ref_Reset(void) {
    Stat_Curr_q_d_ref.qI_Component1 = 0;
    Stat_Curr_q_d_ref.qI_Component2 = 0;

    gsM1_Drive.sFocPMSM.sIDQReq.f32D = 0;
    gsM1_Drive.sFocPMSM.sIDQReq.f32Q = 0;
}

void Brake_On(void) {
#if HARDWARE_VERSION_2_2
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);
#else
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
#endif
}

void Brake_Hold(void) {
#if HARDWARE_VERSION_2_2
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
#else
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
#endif
}

void Brake_Off(void) {
#if HARDWARE_VERSION_2_2
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_SetBits(GPIOA, GPIO_Pin_2);
#else
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
#endif
}

u8 Brake_Off_Check(void) {
#if HARDWARE_VERSION_2_2
    return GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3) || GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2);
#else
    return GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9) || GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_8);
#endif
}

u8 Brake_Start_Check(void) {
#if HARDWARE_VERSION_2_2
    return GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3);
#else
    return GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9);
#endif
}

/*******************************************************************************
* Function Name  :
* Description    : This function
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
u32 STMFLASH_ReadWord(u32 faddr) {
    return *(vu32 *)faddr;
}

/*******************************************************************************
* Function Name  :
* Description    : This function
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u32 NumToRead) {
    u32 i;
    for (i = 0; i < NumToRead; i++) {
        pBuffer[i] = STMFLASH_ReadWord(ReadAddr);
        ReadAddr += 4;
    }
}

/*******************************************************************************
* Function Name  :
* Description    : This function
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
uint16_t STMFLASH_GetFlashSector(u32 addr) {
    if (addr < ADDR_FLASH_SECTOR_1)
        return FLASH_Sector_0;
    else if (addr < ADDR_FLASH_SECTOR_2)
        return FLASH_Sector_1;
    else if (addr < ADDR_FLASH_SECTOR_3)
        return FLASH_Sector_2;
    else if (addr < ADDR_FLASH_SECTOR_4)
        return FLASH_Sector_3;
    else if (addr < ADDR_FLASH_SECTOR_5)
        return FLASH_Sector_4;
    else if (addr < ADDR_FLASH_SECTOR_6)
        return FLASH_Sector_5;
    else if (addr < ADDR_FLASH_SECTOR_7)
        return FLASH_Sector_6;
    else if (addr < ADDR_FLASH_SECTOR_8)
        return FLASH_Sector_7;
    else if (addr < ADDR_FLASH_SECTOR_9)
        return FLASH_Sector_8;
    else if (addr < ADDR_FLASH_SECTOR_10)
        return FLASH_Sector_9;
    else if (addr < ADDR_FLASH_SECTOR_11)
        return FLASH_Sector_10;
    return FLASH_Sector_11;
}

/*******************************************************************************
* Function Name  :
* Description    : This function
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u32 NumToWrite) {
    FLASH_Status status = FLASH_COMPLETE;
    u32 addrx;
    u32 endaddr;

    if ((WriteAddr < STM32_FLASH_BASE) || (WriteAddr % 4)) {
        return;
    }

    FLASH_Unlock();
    FLASH_DataCacheCmd(DISABLE);

    addrx = WriteAddr;
    endaddr = WriteAddr + NumToWrite * 4;
    if (addrx < 0x1FFF0000) {
        while (addrx < endaddr) {
            if (STMFLASH_ReadWord(addrx) != 0xFFFFFFFF) {
                status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx), VoltageRange_3);
                if (status != FLASH_COMPLETE)
                    break;
            } else {
                addrx += 4;
            }
        }
    }

    while (WriteAddr < endaddr) {
        if (FLASH_ProgramWord(WriteAddr, *pBuffer) != FLASH_COMPLETE) {
            //gsM1_Drive.sFaultId.unCtrlErr.bit.ParaFlashError = 1;
            break;
        }
        WriteAddr += 4;
        pBuffer++;
    }

    FLASH_DataCacheCmd(ENABLE);
    FLASH_Lock();
}

/*******************************************************************************
* Function Name  :
* Description    : This function
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
u32 crc_val;
//u32 crc_val2;
//u32 crc_val3;
u8 Flash_Paramenter_Read(void) {
    //u32 crc_val;

    STMFLASH_Read(ADDR_FLASH_SECTOR_3, (u32 *)Flash_Read_Buffer, BUFFER_SIZE);

    CRC->CR = 0x00000001;
    crc_val = CRC_CalcBlockCRC((u32 *)Flash_Read_Buffer, BUFFER_SIZE - 1);
    //CRC->CR = 0x00000001;
    //crc_val2 = CRC_CalcBlockCRC((u32*)Flash_Read_Buffer,BUFFER_SIZE-1);
    //CRC->CR = 0x00000001;
    //crc_val3 = CRC_CalcBlockCRC((u32*)Flash_Read_Buffer,BUFFER_SIZE-1);

    if ((crc_val != Flash_Read_Buffer[BUFFER_SIZE - 1]) && (Flash_Read_Buffer[BUFFER_SIZE - 1] != 0xFFFFFFFF))
        return 1;
    else
        return 0;
}

/*******************************************************************************
* Function Name  :
* Description    : This function
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
void Flash_Paramenter_Write(void) {
    CRC->CR = 0x00000001;
    /* Compute the CRC, store in the end of array */
    Flash_Write_Buffer[BUFFER_SIZE - 1] = CRC_CalcBlockCRC((uint32_t *)Flash_Write_Buffer, BUFFER_SIZE - 1);

    STMFLASH_Write(ADDR_FLASH_SECTOR_3, (u32 *)Flash_Write_Buffer, BUFFER_SIZE);
}

/*******************************************************************************
* Function Name  :
* Description    : This function
* Input      : None
* Output     : None
* Return     : None
*******************************************************************************/
u8 Flash_Paramenter_Write_Verify(void) {
    for (u8 i = 0; i < BUFFER_SIZE; i++) {
        if (Flash_Read_Buffer[i] != Flash_Write_Buffer[i]) {
            return 1;
        }
    }
    return 0;
}
