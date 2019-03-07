/*
----------------------------------------------------------------------------
 * File					: GlobalVariable.c
 * Description			: 全局变量/类型
 * Author				: zhenyonghit
 * Copyright			: Copyright (c) 2016
 * Company				: Asage-Robots(http://www.asage-robots.com/)
 * 
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.12.05 12:26:23
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */

//---------------------------- Include files -----------------------------//
#include "GlobalVariable.h"
//#include "EEPROM_RW.h"
#include "M1_statemachine.h"


//---------------------- Static Function Prototype ------------------------//
static void init_Para_Cod(void);


//---------------------------- Static Variable ----------------------------//


//---------------------------- Global Variable ----------------------------//
__IO uint32_t gui_EnvirStatus = 0x00000000;		//控制系统的环境状态，主要是软件状态和硬件状态

__IO uint32_t gui_LocalTime = 0x00000000;		//关节控制软件系统时钟，每us加1


u16 gus_JD_Status = 0;							//关节驱动器状态码
u16 gus_JD_Status_Set = 0;						//设置关节驱动器状态码，JC通过RS485下发JD，从JD_Status_t取值
u16 gus_JD_Error = 0;							//关节驱动器故障码

//__IO u8 guc_RS485_Error_Flag = OTHER_ERROR;	//标记RS485通信错误信息，0x0：发送无错误；0x1：无接受错误；0x2：CRC校验错误；0x3：其他错误
__IO u8 guc_RS485_Error_Flag = NO_ERROR;		//标记RS485通信错误信息，0x0：发送无错误；0x1：无接受错误；0x2：CRC校验错误；0x3：其他错误

__IO u8 guc_RS485_Flag = 0;						//标志位，标志RS485收到一帧数据：0——没有数据帧，1——有数据帧
__IO u32 gui_RS485_RX_Num = 0;					//接收到的数据帧的长度


Para_t g_gstParaCodeDef[256];					//生成参数编码
/*参数列表值*/
int JOINT_No = Joint2;							//关节编号
int g_iJDVersion = 2;							//JD主版本编号
int g_iJDSubVersion = 13;						//JD副版本编号




/**/
float gf_Angle_In = 0.0f;						//关节谐波减速器输入端实际角度
float gf_Velocity_In = 0.0f;					//关节谐波减速器输入端实际角速度
float gf_Accelerated_In = 0.0f;					//关节谐波减速器输入端实际角加速度

float gf_Angle_Out = 0.0f;						//关节谐波减速器输出端实际角度
float gf_Velocity_Out = 0.0f;					//关节谐波减速器输出端实际角速度
float gf_Accelerated_Out = 0.0f;				//关节谐波减速器输出端实际角加速度

float gf_Desire_Angle = 0.0f;					//关节期望角度
float gf_Desire_Velocity = 0.0f;				//关节期望角速度
float gf_Desire_Accelerated = 0.0f;				//关节期望角加速度

float gf_Torque = 0.0f;							//关节实际力矩
float gf_Desire_Torque = 0.0f;					//关节期望力矩

int gi_PWM = 0;									//关节实际力矩

float gf_Current_Q = 0.0f;						//电机A相电流
float gf_Current_D = 0.0f;						//电机B相电流

float gf_Temperature = 0.0f;					//关节温度
float gf_Voltage = 0.0f;						//关节电压
int g_iQueryValue = 0;							//用于存放Int32的参数查询值
//u16 gs_ADC_CheckBack[2] = {0, 0};				//MB和IO电压ADC


//------------------------------- Function --------------------------------//
void initGlobal(void)
{
	init_Para_Cod();							//建立参数编号表

}


/*
----------------------------------------------------------------------------
 * Name					: init_Para_Cod
 * Description			: 建立参数编号表
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: $NOW
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
 /*
电流环D轴	
	PID调节器Kp	float	5000	0	1000
	PID调节器Ki	float	5000	0	1000
	PID调节器Kd	float	5000	0	1000
电流环Q轴	
	PID调节器Kp	float	5000	0	1000
	PID调节器Ki	float	5000	0	1000
	PID调节器Kd	float	5000	0	1000
速度环	
	PID调节器Kp	float	5000	0	1000
	PID调节器Ki	float	5000	0	1000
	PID调节器Kd	float	5000	0	1000
	速度曲线斜率u32	    60000	1	1
位置环	
	PID调节器Kp	float	5000	0	1000
	PID调节器Ki	float	5000	0	1000
	PID调节器Kd	float	5000	0	1000
	输出上限	float	3000	0	1
	输出下限	float	-3000	0	1
*/
static void init_Para_Cod(void)
{
	int i = 0;
	int tempCode = 0;
	for(i=0; i<0xFF; ++i)
	{
		g_gstParaCodeDef[i].m_Ptr = NULL;						//给空地址
		g_gstParaCodeDef[i].m_cType = 0x00;
		g_gstParaCodeDef[i].m_fUnit = 1.0f;						//变量单位: 1
	}
	/*0x91	关节编号	Int32	1*/
	tempCode = 0x91;
	g_gstParaCodeDef[tempCode].m_Ptr = &JOINT_No;				//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x01;					//int
	g_gstParaCodeDef[tempCode].m_fUnit = 1.0f;					//变量单位: 1
	/*0x92	主版本号	Int32	1*/
	tempCode = 0x92;
	g_gstParaCodeDef[tempCode].m_Ptr = &g_iJDVersion;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x01;					//int
	g_gstParaCodeDef[tempCode].m_fUnit = 1.0f;					//变量单位: 1
	
	/*0x93	副版本号	Int32	1*/
	tempCode = 0x93;
	g_gstParaCodeDef[tempCode].m_Ptr = &g_iJDSubVersion;		//变量: g_iJCSubVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x01;					//int
	g_gstParaCodeDef[tempCode].m_fUnit = 1.0f;					//变量单位: 1
	
	/*0xB0	电流环——D轴-PID_KP参数	Int32	1*/
	tempCode = 0xB0;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sFocPMSM.sIdPiParams.f32PropGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000
	/*0xB1	电流环——D轴-PID_KI参数	Int32	1*/
	tempCode = 0xB1;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sFocPMSM.sIdPiParams.f32IntegGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000
	/*0xB2	电流环——D轴-PID_KD参数	Int32	1*/
	tempCode = 0xB2;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sFocPMSM.sIdPiParams.f32DiffGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000
	
	/*0xB3	电流环——Q轴-PID_KP参数	Int32	1*/
	tempCode = 0xB3;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sFocPMSM.sIqPiParams.f32PropGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000
	/*0xB4	电流环——Q轴-PID_KI参数	Int32	1*/
	tempCode = 0xB4;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sFocPMSM.sIqPiParams.f32IntegGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000
	/*0xB5	电流环——Q轴-PID_KD参数	Int32	1*/
	tempCode = 0xB5;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sFocPMSM.sIqPiParams.f32DiffGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000
	
	/*0xB6	速度环-PID_KP参数	Int32	1*/
	tempCode = 0xB6;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32PropGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000
	/*0xB7	速度环-PID_KI参数	Int32	1*/
	tempCode = 0xB7;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32IntegGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000	
	/*0xB8	速度环-PID_KD参数	Int32	1*/
	tempCode = 0xB8;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sSpeed.sSpeedPiParamsSet.f32DiffGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000	
	
	/*0xB9	位置环-PID_KP参数	Int32	1*/
	tempCode = 0xB9;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32PropGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000
	/*0xBA	位置环-PID_KI参数	Int32	1*/
	tempCode = 0xBA;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32IntegGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000	
	/*0xBB	位置环-PID_KD参数	Int32	1*/
	tempCode = 0xBB;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sPositionControl.sPositionPiParamsSet.f32DiffGain;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1000.0f;				//变量单位: 1000	
	
	/*0xBC	速度曲线斜率	Unt32	1*/
	tempCode = 0xBC;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sSpeed.f32SpeedRampStep;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1.0f;						//变量单位: 1	
	
	/*0xBD	输出上限	Unt32	1*/
	tempCode = 0xBD;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sPositionControl.sPositionPiParams.f32UpperLimit;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1.0f;				//变量单位: 1	
	
	/*0xBE	输出下限	Unt32	1*/
	tempCode = 0xBE;
	g_gstParaCodeDef[tempCode].m_Ptr = &gsM1_Drive.sPositionControl.sPositionPiParams.f32LowerLimit;			//变量: g_iJCVersion
	g_gstParaCodeDef[tempCode].m_cType = 0x00;					//float
	g_gstParaCodeDef[tempCode].m_fUnit = 1.0f;				//变量单位: 1	
	

}




