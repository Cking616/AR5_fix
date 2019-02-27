/*
----------------------------------------------------------------------------
 * File					: JC2JDCommunication.c
 * Description			: JC to JD 通信模块
 * Author				: zhenyonghit
 * Copyright			: Copyright (c) 2016
 * Company				: Asage-Robots(http://www.asage-robots.com/)
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.11.28 16:44:55
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
--------------------------------------------------------------------------*/

//---------------------------- Include files -----------------------------//
#include "JC2JDCommunication.h"
#include "GlobalVariable.h"

#include "RS485_JC2JD_ModBus.h"
#include "RS232_USART.h"

#include "CRC_16.h"

#include "arm_math.h"

#include "StateMachine.h"
#include "Control.h"
#include "Configuration.h"
#include "Motor_Structure.h"
#include "M1_statemachine.h"
#include "Motor_Drive.h"

//---------------------- Static Function Prototype ------------------------//
static void init(unsigned long int);

static void Inst_Process(void);
static void CheckData(void);

static void Cycle_Rsp(u16, s16);
static void ParaSetCallback(void);
static void ParaQueryCallback(void);

//---------------------------- Static Variable ----------------------------//
JC2JDParaSet_t ParaSet_JC2JD;						//参数设置指令变量
JD2JCParaSetRsp_t ParaSetRsp_JC2JD;					//参数设置指令应答变量

JC2JDParaQuery_t ParaQuery_JC2JD;					//参数查询指令变量
JD2JCParaQueryRsp_t ParaQueryRsp_JC2JD;				//参数查询指令应答变量

JC2JDCycle_t Cycle_JC2JD; 							//周期性指令变量
JD2JCCycleRsp_t CycleRsp_JC2JD;						//周期性指令应答变量


/*用于调试*/
//static uint32_t time_Start_JC2JD = 0;		//用于记录程序运行起始时间
//一阶低通滤波器


//---------------------------- Global Variable ----------------------------//
JC2JDCycle_Buf_t gt_RS485_RX_Buf;			//RS485接受缓冲区，用于处理，不用于接受

JC2JD_t g_stJC2JD =
{
	.init = init,
	.Inst_Process = Inst_Process,
	.ParaSetCallback = ParaSetCallback,
	.ParaQueryCallback = ParaQueryCallback
};
int J_ADDR[8] = {NULL_ADDR,J1_ADDR,J2_ADDR,J3_ADDR,J4_ADDR,J5_ADDR,J6_ADDR,J7_ADDR};

//------------------------------- Function --------------------------------//
/*
----------------------------------------------------------------------------
 * Name					: init
 * Description			: 初始化JC to JD变量	需要时间16.91us
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2016.11.28 16:48:55
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
static void init(unsigned long int baudrate)
{
	int i = 0;

	//2.3.1 JC→JD参数设置指令
	ParaSet_JC2JD.Para_Set.m_ucAddr = 0x00;				//设备地址
	ParaSet_JC2JD.Para_Set.m_ucFunCode = 0x00;			//指令码0x51
	ParaSet_JC2JD.Para_Set.m_ucParamNo = 0xA0U;				//参数编码
	ParaSet_JC2JD.Para_Set.m_ucParamValue = 0;				//参数值
	ParaSet_JC2JD.Para_Set.m_ucCRCHigh = 0x00U;				//CRC校验高8位
	ParaSet_JC2JD.Para_Set.m_ucCRCLow = 0x00U;				//CRC校验低8位


	//2.3.2 JD→JC参数设置指令应答
	ParaSetRsp_JC2JD.Para_Set_Rsp.m_ucAddr = J_ADDR[JOINT_No];			//设备地址
	ParaSetRsp_JC2JD.Para_Set_Rsp.m_ucFunCode = 0x55U;	//指令码0x55
	ParaSetRsp_JC2JD.Para_Set_Rsp.m_ucErrorCode = 0x00U;	//错误码
	ParaSetRsp_JC2JD.Para_Set_Rsp.m_ucCRCHigh	= 0x00U;		//CRC校验高8位
	ParaSetRsp_JC2JD.Para_Set_Rsp.m_ucCRCLow = 0x00U;		//CRC校验低8位


	//2.3.3 JC→JD参数查询指令
	ParaQuery_JC2JD.Para_Query.m_ucAddr = 0x00;			//设备地址
	ParaQuery_JC2JD.Para_Query.m_ucFunCode = 0x00;		//指令码0x71
	ParaQuery_JC2JD.Para_Query.m_ucParamNo = 0x00U;			//参数编码
	ParaQuery_JC2JD.Para_Query.m_ucCRCHigh = 0x00U;			//CRC校验高8位
	ParaQuery_JC2JD.Para_Query.m_ucCRCLow = 0x00U;			//CRC校验低8位


	//2.3.4 JD→JC参数查询指令应答
	ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucAddr = J_ADDR[JOINT_No];		//设备地址
	ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucFunCode = 0x75U;//指令码0x75
	ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucParamValue = 0;	//参数值
	ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucCRCHigh = 0x00U;	//CRC校验高8位
	ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucCRCLow = 0x00U;	//CRC校验低8位


	//2.3.5 周期性指令
	Cycle_JC2JD.Cycle.m_ucAddr = 0x00;						//设备地址
	Cycle_JC2JD.Cycle.m_ucFunCode = 0x00U;				//指令码0x91
	Cycle_JC2JD.Cycle.m_usStatusKey = 0x0000U;					//状态码/故障码
	Cycle_JC2JD.Cycle.m_sTorque = 0;						//关节期望输出力矩,0.01N	100
	Cycle_JC2JD.Cycle.m_ucCRCHigh = 0x00U;					//CRC校验高8位
	Cycle_JC2JD.Cycle.m_ucCRCLow = 0x00U;					//CRC校验低8位


	//2.3.6 周期性指令应答
	CycleRsp_JC2JD.Cyc_Rsp.m_ucAddr	= J_ADDR[JOINT_No];				//设备地址
	CycleRsp_JC2JD.Cyc_Rsp.m_ucFunCode = 0x95U;			//指令码0x95
	CycleRsp_JC2JD.Cyc_Rsp.m_ucJDStatus = 0x00U;			//状态码/故障码
	CycleRsp_JC2JD.Cyc_Rsp.m_iAngleOut = 0;					//关节期望角度，单位0.001°
	CycleRsp_JC2JD.Cyc_Rsp.m_iAngleIn = 0;					//关节期望角度，单位0.001°
	CycleRsp_JC2JD.Cyc_Rsp.m_sVelocityOut = 0;					//关节期望角速度，单位0.001°/s
//	CycleRsp_JC2JD.Cyc_Rsp.v_in = 0;					//关节期望角速度，单位0.001°/s
	CycleRsp_JC2JD.Cyc_Rsp.m_sAccOut = 0;					//关节期望角加速度，单位0.001°/s2
//	CycleRsp_JC2JD.Cyc_Rsp.a_in = 0;					//关节期望角加速度，单位0.001°/s2
	CycleRsp_JC2JD.Cyc_Rsp.m_sCurrentEquality = 0;		//电机电流Q，单位1
//	CycleRsp_JC2JD.Cyc_Rsp.i_D = 0;						//电机电流D，单位0.001A
	CycleRsp_JC2JD.Cyc_Rsp.m_sVoltage = 0;					//关节电压，单位0.1V
	CycleRsp_JC2JD.Cyc_Rsp.m_cTemperature = 0;				//关节温度，单位1度
	CycleRsp_JC2JD.Cyc_Rsp.m_ucCRCHigh = 0x00U;				//CRC校验高8位
	CycleRsp_JC2JD.Cyc_Rsp.m_ucCRCLow = 0x00U;				//CRC校验低8位

 	for(i=0; i<RS485_RCV_SIZE; ++i)							//需要判断接收JC指令中数组数据最大的指令
	{
		gt_RS485_RX_Buf.buf[i] = 0x00U;
	}

	rs485_JC2JD.init(baudrate);
}




/*
----------------------------------------------------------------------------
 * Name					: Inst_Process
 * Description			: 处理JC发送的指令
 * Author				: zhenyonghit
 * return				:
 * Para					:
 *
 * History
 * ----------------------
 * Rev					: V1.00
 * Create Date			: 2017.03.01 13:48:55
 * ----------------------
 * Modify Author		: zhenyonghit
 * Modify Description	: TODO
 * Modify Date			: $NOW
 * ----------------------
----------------------------------------------------------------------------
 */
static void Inst_Process(void)
{
	/* CRC检测*/
	CheckData();
	/*指令响应*/
	if(NO_ERROR == guc_RS485_Error_Flag)
	{
		/*参数设置指令*/
		if((0x51U==gt_RS485_RX_Buf.Para_Set.m_ucFunCode)&&(gui_RS485_RX_Num == JC2JD_PARA_SETNUM))
		{
			g_stJC2JD.ParaSetCallback();
		}
		//参数查询指令
		if((0x71U==gt_RS485_RX_Buf.Para_Query.m_ucFunCode)&&(gui_RS485_RX_Num == JC2JD_PARA_QUERYNUM))
		{
			g_stJC2JD.ParaQueryCallback();
		}
		//周期性指令
		if((0x91U==gt_RS485_RX_Buf.Cycle.m_ucFunCode)&&(gui_RS485_RX_Num == JC2JD_CMDPNUM))
		{
			Cycle_Rsp(gt_RS485_RX_Buf.Cycle.m_usStatusKey, gt_RS485_RX_Buf.Cycle.m_sTorque);
		}
//		guc_RS485_Error_Flag = OTHER_ERROR;
	}
	/*清通讯标志位*/
	guc_RS485_Flag = 0;
}

/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: ParaSetCallback
Description		: 参数设置指令处理函数，只有在g_ucJC_Ctrl_Status=STANDBY、g_ucCtrl_Mode=CTRL_AWAIT下能够进行参数设置
Parameter     	: Parameter List
					void
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void ParaSetCallback(void)
{
	Para_t temp = {NULL, 1};		//用于放置参数列表变量

	temp.m_Ptr = g_gstParaCodeDef[gt_RS485_RX_Buf.Para_Set.m_ucParamNo].m_Ptr;
	temp.m_cType = g_gstParaCodeDef[gt_RS485_RX_Buf.Para_Set.m_ucParamNo].m_cType;
	temp.m_fUnit = g_gstParaCodeDef[gt_RS485_RX_Buf.Para_Set.m_ucParamNo].m_fUnit;
	if (NULL != temp.m_Ptr)		//确保参数已经在参数列表中定义，以免产生访问异常
	{
		if (gt_RS485_RX_Buf.Para_Set.m_ucParamNo < PARA_CODE_SEGM2)	//编号大于于0x91属于JD的参数，参数赋值
		{

			if(temp.m_cType)
			{
				(*((int*)temp.m_Ptr)) = gt_RS485_RX_Buf.Para_Set.m_ucParamValue/temp.m_fUnit;
				guc_RS485_Error_Flag = NO_ERROR;
			}
			else
			{
				(*((float*)temp.m_Ptr)) = gt_RS485_RX_Buf.Para_Set.m_ucParamValue/temp.m_fUnit;
				guc_RS485_Error_Flag = NO_ERROR;
			}
		}
	}
	else								//参数没有在参数列表中定义
	{
		guc_RS485_Error_Flag = OTHER_ERROR;
	}
	ParaSetRsp_JC2JD.Para_Set_Rsp.m_ucErrorCode  = guc_RS485_Error_Flag;
	ParaSetRsp_JC2JD.Para_Set_Rsp.m_ucAddr = J_ADDR[JOINT_No];
	rs485_JC2JD.MSG_Tx(ParaSetRsp_JC2JD.buf, JD2JC_PARA_SETRSPNUM);
}



/* @function
---------------------------------------------------------------------------------
<PRE>
Function		: ParaQueryCallback
Description		: 参数查询指令处理函数
Parameter     	: Parameter List
					void
Return			: void
Throw Exception	: Exception List
					Exception1 : Description
					Exception2 : Description
				...
---------------------------------------------------------------------------------
comment			: $Other comment
Typical usage	: $Usage
---------------------------------------------------------------------------------
Author			: zhenyonghit
</PRE>
---------------------------------------------------------------------------------
*/
static void ParaQueryCallback(void)
{
	Para_t temp = {NULL, 1};		//用于放置参数列表变量

	temp.m_Ptr = g_gstParaCodeDef[gt_RS485_RX_Buf.Para_Query.m_ucParamNo].m_Ptr;
	temp.m_cType = g_gstParaCodeDef[gt_RS485_RX_Buf.Para_Query.m_ucParamNo].m_cType;
	temp.m_fUnit = g_gstParaCodeDef[gt_RS485_RX_Buf.Para_Query.m_ucParamNo].m_fUnit;

	if (NULL != g_gstParaCodeDef[gt_RS485_RX_Buf.Para_Query.m_ucParamNo].m_Ptr)		//确保参数已经在参数列表中定义，以免产生访问异常
	{
		if (gt_RS485_RX_Buf.Para_Query.m_ucParamNo < PARA_CODE_SEGM2)				//编号小于0x90属于JC的参数，参数赋值
		{
			if(temp.m_cType)
			{
				ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucParamValue = (int)((*((int*)temp.m_Ptr))*temp.m_fUnit);
			}
			else
			{
				ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucParamValue = (int)((*((float*)temp.m_Ptr))*temp.m_fUnit);
			}
		}
	}
	else									//参数没有在参数列表中定义
	{
		guc_RS485_Error_Flag = OTHER_ERROR;
	}
	ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucAddr = J_ADDR[JOINT_No];
	ParaQueryRsp_JC2JD.Para_Query_Rsp.m_ucFunCode = 0x75;
	rs485_JC2JD.MSG_Tx(ParaQueryRsp_JC2JD.buf, JD2JC_PARA_QUERYRSPNUM);
}





/*
----------------------------------------------------------------------------
 * Name					: CycleCtl_Rsp
 * Description			: 周期性伺服控制指令：发送——接收——校验，需要轨迹插补模块，控制模块，有待完成
 * Author				: zhenyonghit
 * return				: NO_ERROR 0：正常； NO_RX_ERROR 1：无返回值； CRC_CHECK_ERROR 2：校验错误； OTHER_ERROR 3：其他错误。
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
static void Cycle_Rsp(u16 status, s16 torque)
{
	if(guc_RS485_Error_Flag == NO_ERROR)
	{
		gsM1_Ctrl.uiCtrl = gt_RS485_RX_Buf.Cycle.m_usStatusKey;
		
		switch(gt_RS485_RX_Buf.Cycle.m_ucModeCtrl)
		{
			case(0x10):  
				gsM1_Drive.uw16CtrlMode = TORQUE_CONTROL;
			
				#if (INIT_LOCATING == 1)||(ANGLE_SEARCHING == 1)
						#if INIT_LOCATING == 1
						if((gsM1_Drive.uw16CtrlMode == TORQUE_CONTROL)&&(gsM1_Drive.sInitLocating.u8LocatingStep >= 14))
						#endif
						{
							gsM1_Drive.sFocPMSM.sIDQReq.f32Q = (float)gt_RS485_RX_Buf.Cycle.m_sTorque / HARMONIC_AMPLIFY;
						}
				#else
						if(gsM1_Drive.uw16CtrlMode == TORQUE_CONTROL)
						{
							gsM1_Drive.sFocPMSM.sIDQReq.f32Q = (float)gt_RS485_RX_Buf.Cycle.m_sTorque / HARMONIC_AMPLIFY;
						}
				#endif	
						
				if(gsM1_Drive.sFocPMSM.sIDQReq.f32Q > gsM1_Drive.sFaultThresholds.f32CurrentOver)
					gsM1_Drive.sFocPMSM.sIDQReq.f32Q = gsM1_Drive.sFaultThresholds.f32CurrentOver;
				
				if(gsM1_Drive.sFocPMSM.sIDQReq.f32Q < -gsM1_Drive.sFaultThresholds.f32CurrentOver)
					gsM1_Drive.sFocPMSM.sIDQReq.f32Q = -gsM1_Drive.sFaultThresholds.f32CurrentOver;
				
				
				gsM1_Drive.sPositionControl.f32PositionCmd = (float)gt_RS485_RX_Buf.Cycle.m_iDesireAngle/10000.0f;
				gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionControl.f32PositionCmd;
				
				break;
			
			case(0x20):	
				gsM1_Drive.uw16CtrlMode = SPEED_CONTROL;

			
				gsM1_Drive.sSpeed.f32SpeedCmd = (float)gt_RS485_RX_Buf.Cycle.m_sDesireVelocity/100.0f*HARMONIC_AMPLIFY/360.0f*60.0f;		
			
				if(gsM1_Drive.sSpeed.f32SpeedCmd > gsM1_Drive.sFaultThresholds.f32SpeedOver)
					gsM1_Drive.sSpeed.f32SpeedCmd = gsM1_Drive.sFaultThresholds.f32SpeedOver;
				
				if(gsM1_Drive.sSpeed.f32SpeedCmd < -gsM1_Drive.sFaultThresholds.f32SpeedOver)
					gsM1_Drive.sSpeed.f32SpeedCmd = -gsM1_Drive.sFaultThresholds.f32SpeedOver;
			
				break;
			
			case(0x30):	
				#ifdef HFI
				if(gsM1_Drive.sHFISearch.u8Step >= 12)
				{
					gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;
			
					gsM1_Drive.sPositionControl.f32FeedforwardTorque = (float)gt_RS485_RX_Buf.Cycle.m_sTorque / HARMONIC_AMPLIFY / Kt / 100.0f;			
				
					gsM1_Drive.sPositionControl.f32PositionCmd = (float)gt_RS485_RX_Buf.Cycle.m_iDesireAngle/10000.0f;

					gsM1_Drive.sSpeed.f32SpeedFF = (float)gt_RS485_RX_Buf.Cycle.m_sDesireVelocity/100.0f*HARMONIC_AMPLIFY/6.0f;
					
					if(gsM1_Drive.sSpeed.f32SpeedFF > POSITIONLOOP_UPPER_LIMIT)
						gsM1_Drive.sSpeed.f32SpeedFF = POSITIONLOOP_UPPER_LIMIT;
					
					if(gsM1_Drive.sSpeed.f32SpeedFF < POSITIONLOOP_LOWER_LIMIT)
						gsM1_Drive.sSpeed.f32SpeedFF = POSITIONLOOP_LOWER_LIMIT;
				
					gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionControl.f32PositionCmd;
				}
				
				#else
				
				gsM1_Drive.uw16CtrlMode = POSITION_CONTROL;
		
				gsM1_Drive.sPositionControl.f32FeedforwardTorque = (float)gt_RS485_RX_Buf.Cycle.m_sTorque / HARMONIC_AMPLIFY / Kt / 100.0f;			
			
				gsM1_Drive.sPositionControl.f32PositionCmd = (float)gt_RS485_RX_Buf.Cycle.m_iDesireAngle/10000.0f;

				gsM1_Drive.sSpeed.f32SpeedFF = (float)gt_RS485_RX_Buf.Cycle.m_sDesireVelocity/100.0f*HARMONIC_AMPLIFY/6.0f;
				
				if(gsM1_Drive.sSpeed.f32SpeedFF > POSITIONLOOP_UPPER_LIMIT)
					gsM1_Drive.sSpeed.f32SpeedFF = POSITIONLOOP_UPPER_LIMIT;
				
				if(gsM1_Drive.sSpeed.f32SpeedFF < POSITIONLOOP_LOWER_LIMIT)
					gsM1_Drive.sSpeed.f32SpeedFF = POSITIONLOOP_LOWER_LIMIT; 
			
				gsM1_Drive.sPositionControl.f32PositionCmdPrevious = gsM1_Drive.sPositionControl.f32PositionCmd;
				
				#endif
			
				break;
				
			case(0x40):		
				gsM1_Drive.uw16CtrlMode = TRACK_CONTROL;
			
				gsM1_Drive.sPositionControl.f32FeedforwardTorque = (float)gt_RS485_RX_Buf.Cycle.m_sTorque / HARMONIC_AMPLIFY / Kt / 100.0f;			
			
				gsM1_Drive.sPositionControl.f32PositionCmd = (float)gt_RS485_RX_Buf.Cycle.m_iDesireAngle/10000.0f;
			
				break;
			
			default:
				break;
		
		
		}

	}


	
	
	
	/*周期性伺服控制指令应答变量组帧*/
	if(gsM1_Ctrl.eState == RUN)
	{
		#if (INIT_LOCATING == 1)||(ANGLE_SEARCHING == 1) 
		#if INIT_LOCATING == 1
		if(gsM1_Drive.sInitLocating.u8LocatingStep < 15)
		#endif
			CycleRsp_JC2JD.Cyc_Rsp.m_ucJDStatus = STOP;																	
		else
			CycleRsp_JC2JD.Cyc_Rsp.m_ucJDStatus = gsM1_Ctrl.eState;																	
		
		#else
			#ifdef HFI
			if(gsM1_Drive.sHFISearch.u8Step < 12)
				CycleRsp_JC2JD.Cyc_Rsp.m_ucJDStatus = STOP;	
			else
				CycleRsp_JC2JD.Cyc_Rsp.m_ucJDStatus = gsM1_Ctrl.eState;
			#else		
			CycleRsp_JC2JD.Cyc_Rsp.m_ucJDStatus = gsM1_Ctrl.eState;	
			#endif
		
		#endif
	}
	else
	{
		CycleRsp_JC2JD.Cyc_Rsp.m_ucJDStatus = gsM1_Ctrl.eState;																
	}
	
	CycleRsp_JC2JD.Cyc_Rsp.m_uiJDFaultCode = gsM1_Drive.sFaultId.R;													
	
	#if	MAGNET_ENCODER_FBK == 0
	CycleRsp_JC2JD.Cyc_Rsp.m_iAngleOut = (int)(gsM1_Drive.sPositionControl.f32Position*10000.0f);												
	#else
	CycleRsp_JC2JD.Cyc_Rsp.m_iAngleOut = (int)(gsM1_Drive.sPositionControl.f32PositionComp*10000.0f);									
	#endif

	
	CycleRsp_JC2JD.Cyc_Rsp.m_iAngleIn = (int)(gsM1_Drive.sPositionEnc.f32PositionMech*10000.0f);												
	

	
	CycleRsp_JC2JD.Cyc_Rsp.m_sVelocityOut = (short)(gsM1_Drive.sPositionEnc.f32MagnetEncoderSpeedFilt*100.0f);				
	
	CycleRsp_JC2JD.Cyc_Rsp.m_sAccOut = (short)(gsM1_Drive.sPositionEnc.f32MagnetEncoderAccelerationFlt*10.0f);				


	CycleRsp_JC2JD.Cyc_Rsp.m_sCurrentEquality = (short)(gf_Current_Q*100.0f);					

	CycleRsp_JC2JD.Cyc_Rsp.m_sVoltage	= (short)(gsM1_Drive.sFocPMSM.f32UDcBusFilt * 10.0f);									 
	
	CycleRsp_JC2JD.Cyc_Rsp.m_cTemperature = (char)Ext_Temperature;																					

	rs485_JC2JD.MSG_Tx(CycleRsp_JC2JD.buf, JD2JC_RESPNUM);

}




static void CheckData(void)
{
	static	int i = 0;
	/* CRC校验 */
	if(chechCRC(gt_RS485_RX_Buf.buf, gui_RS485_RX_Num) != 0)			//CRC校验
	{
		guc_RS485_Error_Flag = CRC_CHECK_ERROR;
		for(i=0; i<gui_RS485_RX_Num; ++i)
		{
		gt_RS485_RX_Buf.buf[i] = 0;
		}
	}
	else
	{
		guc_RS485_Error_Flag = NO_ERROR;
	}

}
