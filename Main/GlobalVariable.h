#ifndef __GLOBAL_VARIABLE_H
#define __GLOBAL_VARIABLE_H

/*
----------------------------------------------------------------------------
 * File					: GlobalVariable.h
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
#include "stm32f4xx.h"

#include "RS485_JC2JD_ModBus.h"



//-------------------------------- Define --------------------------------//
//#define JOINT_No				(0x1U)			/*关节编号，关节1*/

#define SYSTICK_PERIOD			(10U)			/*SysTick定时器的周期，单位us*/

#define MAIN_CTRL_PERIOD		(50U)			/*主控制周期50，单位us*/
#define MAIN_CTRL_PERIOD_S		(0.00005F)		/*主控制周期50/1000/1000，单位s*/
#define MAIN_CTRL_FREQUENCY		(20000U)		/*主控制频率20kHz*/

#define PERIOD_TIMES			(50U)			/*HC周期10ms/JC周期2ms=5,周期倍数*/

#define ENVIR_CHECK_PERIOD		(2000U)			/*监测周期2*1000，单位us*/

#ifndef NULL
	#define NULL 0								/* see <stddef.h> */
#endif

//#define JOINT_NUM				(6U)				/*关节数量*/

#define GRAVITY					(9.8F)				/*重力加速度*/
#define EPS						(0.00001F)			/*浮点数精度，两个浮点数差小于EPS则认为两者相等*/

#define BAUDRATE_RS485			(5120000UL)			/*RS485波特率*/
//#define BAUDRATE_RS485			(921600UL)		/*RS485波特率*/
//#define BAUDRATE_RS485			(115200UL)		/*RS485波特率*/
#define BAUDRATE_RS232			(38400UL)			/*RS232波特率*/
//#define BAUDRATE_RS232			(76800UL)		/*RS232波特率*/

#define	PARA_CODE_SEGM1			(0x91U)				//参数编号段：0x70之前的参数为JC参数，0x70之后的参数为JC和JD共有参数
#define	PARA_CODE_SEGM2			(0xFFU)				//参数编号段：0x90之后的参数为JD参数，0x90之前的参数为JC和JD共有参数

#define RS485_RCV_SIZE			(60U)				//接受RS485数据的缓冲区，该缓冲区用于指令处理，不用于接受

//------------------------------- Typedef---------------------------------//
typedef enum
{
	Joint1 = 1U,
	Joint2 = 2U,
	Joint3 = 3U,
	Joint4 = 4U,
	Joint5 = 5U,
	Joint6 = 6U,
}Joint_No_t;									/*关节编号类型*/
	

 
typedef enum {OFF = 0, ON = !OFF} Power_Status;
typedef enum {UNUSED = 0, USED = !UNUSED} Used_Status;
//typedef enum {FALSE = 0, TRUE = !FALSE} Boolean_t;


typedef enum
{
	JD_IDLE			= 0x00U,					/*关节驱动器状态机——IDLE	——空闲*/
	JD_INIT			= 0x01U,					/*关节驱动器状态机——INIT	——初始化*/
	JD_READY		= 0x04U,					/*关节驱动器状态机——READY	——待机，准备运动*/
	JD_RUN			= 0x06U,					/*关节驱动器状态机——RUN		——运行*/
	JD_STOP			= 0x08U						/*关节驱动器状态机——STOP	——停止*/
}JD_Status_t;			/*关节驱动器状态机*/


/*构造参数查询和设置对应的参数编号表，该方法会耗损3*4*256的变量空间*/
typedef struct
{
	void 	*m_Ptr;									//变量（设备号为0）地址，数组（设备号为1-6）首地址
	char	m_cType;								//变量类型：0x00——float，0x01——int	
	float	m_fUnit;								//单位（当量）
	 
}Para_t;


typedef enum
{
	NO_ERROR		= 0x0,			/*无错误*/
	NO_RX_ERROR		= 0x1,			/*无接收 错误*/
	CRC_CHECK_ERROR	= 0x2,			/*CRC校验错误*/
	RX_NUM_ERROR    = 0x3,			//接收数量错误
	OTHER_ERROR		= 0x4			/*其他错误，ETH协议数据不规范*/
}Communication_ERROR_t;				//通信错误位


//---------------------------- Global Variable ---------------------------//




//---------------------------- Global Function ---------------------------//



//--------------------------------- Extern -------------------------------//
extern __IO uint32_t gui_EnvirStatus;		//控制系统的环境状态，主要是软件状态和硬件状态

extern __IO uint32_t gui_LocalTime;			//关节控制软件系统时钟，每us加1

extern u16 gus_JD_Status;					//关节驱动器状态码，JD通过RS485上传JC，从JD_Status_t取值
extern u16 gus_JD_Status_Set;				//设置关节驱动器状态码，JC通过RS485下发JD，从JD_Status_t取值
extern u16 gus_JD_Error;					//关节驱动器故障码，JD通过RS485上传JC

extern __IO u8 guc_RS485_Error_Flag;		//标记RS485通信错误信息，0x0：发送无错误；0x1：无接受错误；0x2：CRC校验错误；0x3：其他错误

extern __IO u8 guc_RS485_Flag;				//标志位，标志RS485收到一帧数据
extern __IO u32 gui_RS485_RX_Num;			//接收到的数据帧的长度

extern Para_t g_gstParaCodeDef[256];			//生成参数编码表
/*参数列表值*/
extern int JOINT_No;
extern int g_iJDVersion;					//JD主版本编号
extern int g_iJDSubVersion;					//JD副版本编号

extern float gf_Angle_In;					//关节谐波减速器输入端实际角度
extern float gf_Velocity_In;				//关节谐波减速器输入端实际角速度
extern float gf_Accelerated_In;				//关节谐波减速器输入端实际角加速度

extern float gf_Angle_Out;					//关节谐波减速器输出端实际角度
extern float gf_Velocity_Out;				//关节谐波减速器输出端实际角速度
extern float gf_Accelerated_Out;			//关节谐波减速器输出端实际角加速度

extern float gf_Desire_Angle;				//关节期望角度
extern float gf_Desire_Velocity;			//关节期望角速度
extern float gf_Desire_Accelerated;			//关节期望角加速度

extern float gf_Torque;						//关节实际力矩
extern float gf_Desire_Torque;				//关节期望力矩

extern int gi_PWM;							//关节期望PWM

extern float gf_Current_Q;					//电机A相电流
extern float gf_Current_D;					//电机B相电流

extern float gf_Temperature;				//关节温度

extern float gf_Voltage;					//关节电压

//extern u16 gs_ADC_CheckBack[2];			//MB和IO电压ADC




extern void initGlobal(void);

#endif

