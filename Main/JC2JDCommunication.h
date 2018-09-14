#ifndef	__JC2JD_Communication_H
#define __JC2JD_Communication_H

/* @file
---------------------------------------------------------------------------------
<PRE>
Module			: JC to JD 通信模块数据和结构
File			: JC2JDCommunication.h
Relevant File	: JC2JDCommunication.c
Description		: 定义数据和结构
Author			: zhenyonghit
Rev				: 2.0
project			: AR5-JC
Company			: Asage-Robots(http://www.asage-robots.com/)
---------------------------------------------------------------------------------
comment			: $Other comment
---------------------------------------------------------------------------------
Modify History	: 
Date				Rev			Mender				Content modification
2016/11/28  		V1.0  		zhenyonghit			greate 	file
2017/12/07			V2.0		zhenyonghit			Reconfiguration
</PRE>
---------------------------------------------------------------------------------
*/
//------------------------------- Include files -------------------------------//
#include "Globalvariable.h"
#include "RS485_JC2JD_ModBus.h"
#include "JC2JDCommunication.h"
//-------------------------------- Define --------------------------------//
#define NULL_ADDR	0x00U					//空地址

#define J1_ADDR		0x10U					//Joint1地址
#define J2_ADDR		0x20U					//Joint2地址
#define J3_ADDR		0x30U					//Joint3地址
#define J4_ADDR		0x40U					//Joint4地址
#define J5_ADDR		0x50U					//Joint5地址
#define J6_ADDR		0x60U					//Joint6地址
#define J7_ADDR		0x70U					//Joint6地址
//#ifdef J_ADDR
//	#undefine J_ADDR
//#endif

//#if JOINT_No == 0x1U
//	#define J_ADDR		0x10U				//Joint1地址
//#elseif JOINT_No == 0x2U
//	#define J_ADDR		0x20U				//Joint2地址
//#elseif JOINT_No == 0x3U
//	#define J_ADDR		0x30U				//Joint3地址
//#elseif JOINT_No == 0x4U
//	#define J_ADDR		0x40U				//Joint4地址
//#elseif JOINT_No == 0x5U
//	#define J_ADDR		0x50U				//Joint5地址
//#elseif JOINT_No == 0x6U
//	#define J_ADDR		0x60U				//Joint6地址
//#else
//	#define J_ADDR		0x10U				//(默认)Joint1地址
//#endif


///*Joint1地址*/
//#if JOINT_No == 0x1U
//	#define J_ADDR		0x10U				//Joint1地址
//#endif

///*Joint2地址*/
//#if JOINT_No == 0x2U
//	#define J_ADDR		0x20U				//Joint2地址
//#endif

///*Joint3地址*/
//#if JOINT_No == 0x3U
//	#define J_ADDR		0x30U				//Joint3地址
//#endif

///*Joint4地址*/
//#if JOINT_No == 0x4U
//	#define J_ADDR		0x40U				//Joint4地址
//#endif

///*Joint5地址*/
//#if JOINT_No == 0x5U
//	#define J_ADDR		0x50U				//Joint5地址
//#endif

///*Joint6地址*/
//#if JOINT_No == 0x6U
//	#define J_ADDR		0x60U				//Joint6地址
//#endif

//#if JOINT_No == 0x7U
//	#define J_ADDR		0x70U				//Joint6地址
//#endif
//#define TCP_ADDR		0x70				//工具TCP地址


//---------------------------------- Typedef ----------------------------------//
//2.3.1 JC→JD参数设置指令
#pragma pack (1) /*指定按1字节对齐*/
struct JC2JDParaSet
{
	u8	m_ucAddr;			//设备地址
	u8	m_ucFunCode;		//指令码0x51
	u8	m_ucParamNo;		//参数编码
	s32	m_ucParamValue;		//参数值	
	u8	m_ucCRCHigh;		//CRC校验高8位
	u8	m_ucCRCLow;			//CRC校验低8位
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
#define JC2JD_PARA_SETNUM sizeof(struct JC2JDParaSet)
typedef union
{
	struct JC2JDParaSet Para_Set;
	u8 buf[sizeof(struct JC2JDParaSet)];
}JC2JDParaSet_t;



//2.3.2 JD→JC参数设置指令应答
#pragma pack (1) /*指定按1字节对齐*/
struct JD2JCParaSetRsp
{
	u8	m_ucAddr;			//设备地址
	u8	m_ucFunCode;		//指令码0x55
	u8	m_ucErrorCode;		//错误码
	u8	m_ucCRCHigh;		//CRC校验高8位
	u8	m_ucCRCLow;			//CRC校验低8位
};

#pragma pack () /*取消指定对齐，恢复缺省对齐*/
#define JD2JC_PARA_SETRSPNUM sizeof(struct JD2JCParaSetRsp)
typedef union
{
	struct JD2JCParaSetRsp Para_Set_Rsp;
	u8 buf[sizeof(struct JD2JCParaSetRsp)];
}JD2JCParaSetRsp_t;


//2.3.3 JC→JD参数查询指令
#pragma pack (1) /*指定按1字节对齐*/
struct JC2JDParaQuery
{
	u8	m_ucAddr;			//设备地址
	u8	m_ucFunCode;		//指令码0x71
	u8	m_ucParamNo;		//参数编码
	u8	m_ucCRCHigh;		//CRC校验高8位
	u8	m_ucCRCLow;			//CRC校验低8位
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
#define JC2JD_PARA_QUERYNUM sizeof(struct JC2JDParaQuery)
typedef union
{
	struct JC2JDParaQuery Para_Query;
	u8 buf[sizeof(struct JC2JDParaQuery)];
}JC2JDParaQuery_t;



//2.3.4 JD→JC参数查询指令应答
#pragma pack (1) /*指定按1字节对齐*/
struct JD2JCParaQueryRsp
{
	u8	m_ucAddr;			//设备地址
	u8	m_ucFunCode;		//指令码0x75
	s32	m_ucParamValue;		//参数值
	u8	m_ucCRCHigh;		//CRC校验高8位
	u8	m_ucCRCLow;			//CRC校验低8位
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
#define JD2JC_PARA_QUERYRSPNUM sizeof(struct JD2JCParaQueryRsp)
typedef union
{
	struct JD2JCParaQueryRsp Para_Query_Rsp;
	u8 buf[sizeof(struct JD2JCParaQueryRsp)];
}JD2JCParaQueryRsp_t;

//2.3.5 周期性指令
#pragma pack (1) /*指定按1字节对齐*/
struct JC2JDCycle
{
	u8	m_ucAddr;			//设备地址
	u8	m_ucFunCode;		//指令码0x91
	u16	m_usStatusKey;		//状态码控制字
	u8  m_ucModeCtrl;		//0x10:力矩模式；0x20:速度模式；0x30位置模式
	s32 m_iDesireAngle;		//位置模式下的期望角度
	s16 m_sDesireVelocity;	//速度模式下的期望速度
	s16	m_sTorque;			//关节期望输出力矩,0.01N。力矩模式下的期望力矩位置模式和速度模式下的前馈力矩（重力力矩和摩擦力矩的和
	u8	m_ucCRCHigh;		//CRC校验高8位
	u8	m_ucCRCLow;			//CRC校验低8位
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
#define JC2JD_CMDPNUM sizeof(struct JC2JDCycle) 
typedef union
{
	struct JC2JDCycle Cycle;
	u8 buf[sizeof(struct JC2JDCycle)];
}JC2JDCycle_t;



//2.3.6 周期性指令应答
#pragma pack (1) /*指定按1字节对齐*/
struct JD2JCCycleRsp
{
	u8	m_ucAddr;				//设备地址
	u8	m_ucFunCode;			//指令码0x95
	u8	m_ucJDStatus;			//状态码
	u32 m_uiJDInformationCode;	//提醒码
	u32 m_uiJDWarnCode;			//警告码
	u32 m_uiJDFaultCode;		//故障码
	s32	m_iAngleOut;			//关节输出端角度，单位0.0001°
	s32	m_iAngleIn;				//关节输入端角度，单位0.0001°
	s16	m_sVelocityOut;			//关节输出端角速度，单位0.01°/s
//	s16	m_sVelocityIn;			//关节期角速度，单位0.01°/s
	s16	m_sAccOut;				//关节输出端角加速度，单位0.01°/s2
//	s16	m_sAccIn;				//关节期望角加速度，单位0.01°/s2
	s16	m_sCurrentEquality;		//电机电流Q，单位0.01
	s16 m_sVoltage;				//关节电压，单位0.1V
	s8	m_cTemperature;			//关节温度，单位1度
	u8	m_ucCRCHigh;			//CRC校验高8位
	u8	m_ucCRCLow;				//CRC校验低8位
};
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
#define JD2JC_RESPNUM sizeof(struct JD2JCCycleRsp)

#pragma pack (1) /*指定按1字节对齐*/
typedef union
{
	struct JD2JCCycleRsp Cyc_Rsp;
	u8 buf[sizeof(struct JD2JCCycleRsp)];
}JD2JCCycleRsp_t;
#pragma pack () /*取消指定对齐，恢复缺省对齐*/

#pragma pack (1) /*指定按1字节对齐*/
typedef union
{
	struct JC2JDParaSet Para_Set;
	struct JC2JDParaQuery Para_Query;
	struct JC2JDCycle Cycle;
	u8 buf[RS485_RCV_SIZE];
}JC2JDCycle_Buf_t;
#pragma pack () /*取消指定对齐，恢复缺省对齐*/

/*构造结构体管理通信相关的变量和函数*/
typedef struct
{
	void (*init)(unsigned long int);
	void (*Inst_Process)(void);
	void (*ParaSetCallback)(void);
	void (*ParaQueryCallback)(void);
}JC2JD_t;



//------------------------------ Extern Variable ------------------------------//

extern JC2JD_t g_stJC2JD;									//关节函数结构体变量  

extern JC2JDParaSet_t ParaSet_JC2JD;						//参数设置指令变量
extern JD2JCParaSetRsp_t ParaSetRsp_JC2JD;					//参数设置指令应答变量

extern JC2JDParaQuery_t ParaQuery_JC2JD;					//参数查询指令变量
extern JD2JCParaQueryRsp_t ParaQueryRsp_JC2JD;				//参数查询指令应答变量

extern JC2JDCycle_Buf_t gt_RS485_RX_Buf;							//周期性指令变量
extern int J_ADDR[8];

//------------------------------ Extern Function ------------------------------//



#endif
