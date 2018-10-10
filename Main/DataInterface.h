//########################################################################################
//
//                  Copyright (c) 2018  XXX INC.
//
// File Name:       DataInterface.h
//
// Author:
//
// Version:         V1.0
//
// Date:            2018/03/30
//
// Description:
//
// History:
//
//########################################################################################
#ifndef _DATAINTERFACE_H
#define _DATAINTERFACE_H

#include <stdint.h>
#include "typedef.h"
//========================================================================================
// 常量定义
//========================================================================================

#define SETPARA_NUM         100
#define READPARA_NUM        100
#define ALLPARA_NUM         SETPARA_NUM + READPARA_NUM

// 数据访问故障码定义
#define OD_SUCCESSFUL                    0x00000000
#define OD_READ_NOT_ALLOWED              0x06010001
#define OD_WRITE_NOT_ALLOWED             0x06010002
#define OD_NO_SUCH_OBJECT                0x06020000
#define OD_NOT_MAPPABLE                  0x06040041
#define OD_LENGTH_DATA_INVALID           0x06070010
#define OD_NO_SUCH_SUBINDEX              0x06090011
#define OD_VALUE_RANGE_EXCEEDED          0x06090030
#define OD_VALUE_TOO_HIGH                0x06090031
#define OD_VALUE_TOO_LOW                 0x06090032
#define OD_DEVICE_STATE_ERROR            0x08000022

//数据方向
#define COMM_DATA_INPUT                  0            //数据输入
#define COMM_DATA_OUTPUT                 1            //数据输出

//外部通讯端口模式
#define COMM_PROTOCOL_DS402              0
#define COMM_PROTOCOL_MODBUS             1
#define COMM_PROTOCOL_ETERCAT_SPECIAL    3            //长度都按32位数据读取

//示波器数据读取方式
#define SCOPE_MODE_INDEX            0
#define SCOPE_MODE_ADDRESS          1

//示波器通道个数
#define SCOPE_CHANNEL_CNT           4

//CAN和USB通讯模式区分
#define COMM_USB_MODE               1   //PC机采用USB通讯
#define COMM_CAN_MODE               0   //PC机采用CAN通讯

#define _UNSAVE     				0	// 不保存
#define _SAVE     					1	// 保存

#define _B16      					1	// 16bit整数
#define _B32      					3	// 32bit整数
#define _F32      					2	// 32bit浮点

#define _INST                       0   // 立即生效
#define _RESET                      1   // 重启生效


//示波器数据结构体
typedef struct
{
    void*   pChAddr[SCOPE_CHANNEL_CNT];         // 示波器通道参数地址
    Uint32  ulChIndex[SCOPE_CHANNEL_CNT];       // 示波器通道参数序号
    int16   iChMode[SCOPE_CHANNEL_CNT];         // 0:参数序号模式;  1:参数地址模式
    int16   iChParaLength[SCOPE_CHANNEL_CNT];   // 1:16位;2:24位;3:32位;4:40位;5:48位;6:56位;7:64位
    int16   iProtocolType;                      // 0:DS402; 1:Profibus-DP;  2:Modbus
    int16   iCommType;                          // 0:OFF; 1:USB; 2:CAN; 3:Ethernet
    int16   iTransEnable;                       // [0]禁止/[1]使能
    int16   iTransSpd;                          // [0]低速/[1]高速
    int16   iChannelCnt;                        // 高速示波器通道数
} StructTypeScopeTag;


//参数接口
typedef struct
{
    void *pCacheData;       //内部缓存数据
    void *pControlData;     //内部控制数据
}StructTypeParaInterfaceTag;

//参数范围
typedef struct
{
    int32 lParaMin;     //参数最小值
    int32 lParaMax;     //参数最大值
    int32 lParaDefault; //参数默认值
	int32 lPrecision;   //参数精度
}StructTypeDataRangeTag;

typedef struct
{
    Uint32 bParaStorage     :2;     //参数存储属性:     00:不保存,01：保存，10：保留
    Uint32 iParaLengthOut 	:2;     //接口参数长度：    01:16bit,10:32bit,11:64bit
    Uint32 iParaLengthIn  	:2;     //内部参数长度:     01:16bit,10:32bit,11:64bit
    Uint32 bParaEffect    	:2;     //参数生效方式:     0直接生效，1开关生效，2重启生效
    Uint32 bReserve         :24;    //参数长度
}StructTypeParaAtribtTag;

//数据交换处理总表结构
typedef struct
{
    StructTypeParaInterfaceTag stParaInterface;
    StructTypeDataRangeTag stDataRange;
    void (*pSpecialProcess)(Uint16 uiProtocolType, Uint16 uiDataDirection, int16 iDataListIndex, int32 *pDataBuf);//特殊处理函数
    const StructTypeParaAtribtTag stParaAtribt;     //参数属性
}StructTypetDataInterfaceTag;


typedef struct
{
	int32 lSetIndex[SETPARA_NUM];
	int32 lReadIndex[READPARA_NUM];
	int32 lBuffer[SETPARA_NUM + 1];
	int16 iParaSave;
	int16 iParaRecover;
	int16 iRunEnable;
	int16 iAlignEnable;
	int16 iSysReset;
	int16 iRigidSet;
} StructTypeParaListTag;

#ifdef _DATAINTERFACE_C
    #define PROTO
#else
    #define PROTO   extern
#endif

//========================================================================================
// 变量定义
//========================================================================================
PROTO   int16 iParaRefFlag;//停机生效参数刷新
PROTO   StructTypeScopeTag  stScopePara;
PROTO   const StructTypetDataInterfaceTag stDataInterface[ALLPARA_NUM];
PROTO   StructTypeParaListTag  stParaList;

//========================================================================================
// 函数声明
//========================================================================================
PROTO   int32 DataInterface(Uint16 uiProtocolType, Uint16 uiDataDirection, Uint32 ulParaIndex, Uint16* uiParaLength, int32 *pDataBuf);
PROTO   int16 ScopeDataProcess(int16 iDataMode, int16 iDataLength, Uint32 ulParaIndex, void** pDataAddr, Uint32* pData);
PROTO   void  DataInterfaceInit(void);
PROTO   void  PararecoverProcess(void);
PROTO   void  ParaSaveProcess(void);
PROTO   void  SysResetProcess(void);

PROTO   void  RunEnableCtrl(Uint16 uiProtocolType,Uint16 uiDataDirection, int16 iDataListIndex, int32 *pDataBuf);
PROTO   void  AlignCtrl(Uint16 uiProtocolType,Uint16 uiDataDirection, int16 iDataListIndex, int32 *pDataBuf);
PROTO   void  CanTransmitFastFrame(void);

//========================================================================================
// Êý¾Ý½á¹¹¶¨Òå
//========================================================================================
typedef struct
{
    int16   iTstSglEnable;      //²âÊÔÐÅºÅÊ¹ÄÜ
    int16   iTstSglType;        //²âÊÔÐÅºÅÀàÐÍ,[0]ÕýÏÒ²¨|Sine wave/[1]·½²¨|Square wave/[2]Âö³å|Pulse wave/[3]ÐéÄâÂö³å·¢ÉúÆ÷|Virtual pulse generator
    float   fTstSglFreq;        //²âÊÔÐÅºÅ·¢ÉúÆµÂÊ,µ¥Î»0.01Hz£¬×î´ó4K
    float   fTstSglAmp;         //²âÊÔÐÅºÅ·ùÖµ,×îÐ¡µçÁ÷µ¥Î»ÊÇ0.01A,×îÐ¡×ª¾Øµ¥Î»ÊÇ0.01%,×îÐ¡ËÙ¶Èµ¥Î»ÊÇ0.01RPM,×îÐ¡Î»ÖÃµ¥Î»ÊÇ1PPU
    float   fTstSglOffset;      //²âÊÔÐÅºÅÆ«ÖÃ
    float   fTstSglPhase;       //²âÊÔÐÅºÅÆô¶¯ÏàÎ»
    float   fTstSglRatio;       //²âÊÔÂö³åÕ¼¿Õ±È,Á½¸öÐ¡Êýµã
    int32   lTstSglCycles;      //²âÊÔÐÅºÅ·¢ÉúÖÜÆÚÊý

    int16   iDestOutType;       //²âÊÔÐÅºÅÄ¿±êÊä³öÀàÐÍ
    float   fCrtCmd;            //²âÊÔÐÅºÅµçÁ÷Êä³ö
    float   fTrqCmd;            //²âÊÔÐÅºÅ×ª¾ØÊä³ö
    float   fSpdCmd;            //²âÊÔÐÅºÅËÙ¶ÈÊä³ö
    int32   lPosCmd;            //²âÊÔÐÅºÅÎ»ÖÃÊä³ö
	int16   iCallingFreq;       //²âÊÔÐÅºÅµ÷ÓÃÆµÂÊ,µ¥Î»KHz

	int16   iThetaMode;         //½Ç¶ÈÄ£Ê½  0£º·´À¡½Ç¶È  1£ºÈË¹¤½Ç¶È  2£ºÈË¹¤ÆµÂÊ
	float   fThetaMan;          //ÈË¹¤½Ç¶ÈÉè¶¨Öµ
	float	fOmegaEleMan;       //ÈË¹¤ÆµÂÊÉè¶¨Öµ
	float   fVdCmd;             //DÖáµçÑ¹Éè¶¨
	float   fVqCmd;             //QÖáµçÑ¹Éè¶¨
	float   fIdCmd;             //DÖáµçÁ÷Éè¶¨
	float   fIqCmd;             //QÖáµçÁ÷Éè¶¨
} StructTypeTestCtrlTag;

#define POSBUFNUM  100

typedef struct
{
    /****µçÁ÷»·²ÎÊýÉèÖÃ******/
	float fCurGain;              //µçÁ÷»·ÔöÒæ
	float fCurTi;                //µçÁ÷»·»ý·ÖÊ±¼ä³£Êý
    /****µçÁ÷»·PI******/
	float fCuKp;                 //µçÁ÷»·ÔöÒæ
	float fCurKpKi;              //µçÁ÷»·»ý·Ö

	float fCurFfwGain;           //µçÁ÷»·ÔöÒæ
	float fVdFfw;
	float fVqFfw;
	float fCurFfwCoef;
	float fCurFfwCoef1;
} StructTypeCurLoopTag;

typedef struct
{
	/****Î»ÖÃ»·²ÎÊýÉèÖÃ******/
	int16 iPosCmdMode;           //Î»ÖÃÖ¸ÁîÄ£Ê½
	float fPosGain;              //Î»ÖÃ»·ÔöÒæ
	float fSpdFfwGain;           //ËÙ¶ÈÇ°À¡ÔöÒæ
	float fSpdFfwFilterTime;     //ËÙ¶ÈÇ°À¡ÂË²¨Ê±¼ä
	    /****Î»ÖÃ»·¿ØÖÆ±äÁ¿*****/
	int32 lCmd;                  //Î»ÖÃÖ¸ÁîÔöÁ¿
	int32 lErrAcc;               //Ö±Á÷Âö³å
	int32 lCmdOld;               //Î»ÖÃ·´À¡ÔöÁ¿
	int32 lCmdInc;               //Î»ÖÃÖ¸ÁîÔöÁ¿
	int32 lFbkInc;               //Î»ÖÃ·´À¡ÔöÁ¿
	int32 lCmdAcc;              //Î»ÖÃÖ¸ÁîÔöÁ¿
	int32 lFbkAcc;              //Î»ÖÃ·´À¡ÔöÁ¿
	int32 lFbkPulse;             //Î»ÖÃ·´À¡ÔöÁ¿
	float fPosCoef;              //Î»ÖÃ»·ÏµÊý
	float fSpdFfwCoef;           //ËÙ¶ÈÇ°À¡ÏµÊý
	float fSpdFfw;               //ËÙ¶ÈÇ°À¡
	float fSpdFfwFilterCoef;     //ËÙ¶ÈÇ°À¡ÂË²¨ÏµÊý
	float fSpdFfwOut;            //ËÙ¶ÈÇ°À¡
	float fSpdMax;              //Î»ÖÃ»·ÏµÊý
	float fPosPOut;              //Î»ÖÃ»·PÊä³ö
	float fPosOut;               //Î»ÖÃ»·PÊä³ö

    int16 iFltTime;
	int16 iPosCmdCycle;
    int32 lPosCmdFltOut;
    int32 lPosCmdACCIn;
    int32 lPosCmdDivResidue;
	int32 lPosCmdInSigma;
    int32 lPosCmdFltCntr;
    int32 lPosCmdFltBuffer[POSBUFNUM];
    int32 iPosCmdFltBufferPtr;
    int32 iPosCmdFltCycleStar;
} StructTypePosLoopTag;

typedef struct
{
	/****ËÙ¶È»·²ÎÊýÉèÖÃ******/
	float fSpdFbkFilter;         //ËÙ¶È·´À¡ÂË²¨
	float fJratio;               //¸ºÔØ¹ßÁ¿±È
    float fSpdGain;              //ËÙ¶È»·ÔöÒæ
	float fSpdTi;                //ËÙ¶È»·»ý·ÖÊ±¼ä³£Êý
	float fTrqFfwGain;           //×ª¾ØÇ°À¡ÔöÒæ
	float fTrqFfwFilterTime;     //×ª¾ØÇ°À¡ÂË²¨Ê±¼ä
	float fTrqFilterTime;        //×ª¾ØÂË²¨Ê±¼ä
	/****PI¿ØÖÆ******/
	float fSpdErr;               //ËÙ¶È»·Îó²îÖµ
	float fSpeedReq;             //ËÙ¶È»·¸ø¶¨
	float fSpeedCmd;             //ËÙ¶È»·¸ø¶¨
	float fSpdIntegration;       //ËÙ¶È»·»ý·Ö
	float fSpdProportion;        //ËÙ¶È»·±ÈÀý
	float fSpdKpKi;              //ËÙ¶È»·PIÊä³ö
	float fSpdKpKiOut;           //ËÙ¶È»·PIÂË²¨Êä³ö
	float fSpdOut;               //ËÙ¶È»·Êä³ö
	float fSpdKiCoef;            //ËÙ¶È»·KP¡ÁKi
	float fSpdKP;                //ËÙ¶È»·KP
	float fTrqUpperLimit;        //ËÙ¶È»·×ª¾ØÊä³ö×î´óÖµ
	float fTrqLowerLimit;        //ËÙ¶È»·×ª¾ØÊä³ö×îÐ¡Öµ
	int16 iTrqLimitState;        //ËÙ¶È»·±¥ºÍ×´Ì¬
	float fTrqFilterCoef;        //ËÙ¶ÈÂË²¨ÏµÊý
	/****ËÙ¶È·´À¡******/
	float fSpdFbkDelt;           //ËÙ¶È»··´À¡²îÖµ
	float fSpdFbk;               //ËÙ¶È·´À¡
	float fSpdFbkCoef;           //ËÙ¶È·´À¡ÏµÊý
	float fSpdFbkFilterCoef;     //ËÙ¶È·´À¡ÂË²¨ÏµÊý
	/****×ª¾ØÇ°À¡******/
	float fSpeedReqDelt;         //ËÙ¶È»·¸ø¶¨²îÖµ
	float fSpeedReqOld;          //ËÙ¶È»·¸ø¶¨Old
	float fTrqFfw;               //×ª¾ØÇ°À¡
	float fTrqFfwCoef;           //×ª¾ØÇ°À¡ÏµÊý
	float fTrqFfwFilterCoef;     //×ª¾ØÇ°À¡ÂË²¨ÏµÊý
	float fTrqFfwOut;            //×ª¾ØÇ°À¡

	int16 iAccTime;              //¼ÓËÙÊ±¼ä
	int16 iDecTime;              //¼õËÙÊ±¼ä
	int16 iAccSCurveTime;        //¼ÓËÙSÇúÏßÊ±¼ä
	int16 iDecSCurveTime;        //¼ÓËÙSÇúÏßÊ±¼ä
} StructTypeSpdLoopTag;



PROTO StructTypeTestCtrlTag stTestCtrl;
PROTO StructTypeCurLoopTag  stCurLoop;
PROTO StructTypeSpdLoopTag  stSpdLoop;
PROTO StructTypePosLoopTag  stPosLoop;

//========================================================================================
// Êý¾Ý½á¹¹¶¨Òå
//========================================================================================
typedef struct
{
    int16 iEzJogMode;           // ³ÌÐòµã¶¯EasyJog ³ÌÐòJOGÄ£Ê½Ñ¡Ôñ
    int32 lEzJogPosSet;         // ³ÌÐòµã¶¯EasyJog ÒÆ¶¯Á¿Éè¶¨
    int16 iEzJogSpdSet;         // ³ÌÐòµã¶¯EasyJog ËÙ¶ÈÉè¶¨
    int16 iEzJogAccTime;        // ³ÌÐòµã¶¯EasyJog ¼Ó¼õËÙÊ±¼äÉè¶¨,ms
    int16 iEzJogDlyTime;        // ³ÌÐòµã¶¯EasyJog Í£Ö¹Ê±µÈ´ýÊ±¼äÉè¶¨,ms
    int16 iEzJogCycleSet;       // ³ÌÐòµã¶¯EasyJog ³ÌÐòÔËÐÐ´ÎÊýÉè¶¨
    int16 iEzJogDir;            // ³ÌÐòµã¶¯EasyJog·½Ïò

    int16 iINPos;               // O Ö¸ÁîÎ»ÖÃµ½´ï
    int16 iINPosFbk;            // O ·´À¡Î»ÖÃµ½´ï
    int16 iEzJogStep;
    int16 iEzJogDlyCnt;
    int16 iEzJogCycleCnt;
    int16 iEzJogStep2;

	int16 iEzJOGDirCmd;
	int32 lPosCmd;

} StructTypePosJogVarsTag;

//========================================================================================
// ±äÁ¿¶¨Òå
//========================================================================================
// ËÙ¶È»·½Ó¿Ú²ÎÊý¶¨Òå
PROTO   StructTypePosJogVarsTag  stPosJogPara;

// ÈÎÎñ¸ºÔØÍ³¼Æ
typedef struct
{
 // Para
    Uint32  ulPeriod;           // ÈÎÎñÖÜÆÚ
   	float32 fCoeff;             // ÈÎÎñ¸ºÔØÂÊÏµÊý
    float32 fThreshold;         // ÈÎÎñ¸ºÔØÂÊ·§Öµ
    // Var
    int32  lStartTim;         // Æô¶¯Ê±¿Ì
    int32  lEndTim;           // ½áÊøÊ±¿Ì
    int32  lDeltaTim;         // Ö´ÐÐÊ±¼ä
    // Output
    float32 fRatio;            // ÈÎÎñ¸ºÔØÂÊ
    float32 fRatioMax;         // ÈÎÎñ¸ºÔØÂÊ×î´óÖµ
	  int32   lRatio;            // µ¥Î»0.01%
	  int32   lRatioMax;         // µ¥Î»0.01%
    int16   iErrFlg;           // ÈÎÎñ¹ýÔØ±êÖ¾
} StructTypeTaskStatTag;

PROTO   StructTypeTaskStatTag stTaskStat[5];

#define   CTRL_FREQ_K                          20

typedef struct
{
	int32    lPosIncCnt;
    int32    lPosIncSum1ms;
    int32    lFbkIncBuf[CTRL_FREQ_K];
    int32    lPosIncSum1msOld;
    float32    fLpf20HzCoef1;
	float32    fLpf20HzCoef;
	float32    fLpf50HzCoef;
	int32    lDistCntThreshold;
	int32    lAccCntThreshold;
	float32    fJerkMin;

	int16    iCmdDynamicFlag;
	int16    iFbkDynamicFlag;
	int16    lFlagAcc;

	int16    iDistFlag;
	int32    lDistCnt;

	int16    lFlagAccOld;
	int32    lFlagAccCnt;

	int32    lJRawBufCnt;
	int32    lJRawFltCnt;
	float32  fAccCmd;
	float32  fAccFbk;
	float32  fAccCmdFlt1;
	float32  fAccFbkFlt1;

	float32  fJerkCmd;
	float32  fJerkFbk;
	float32  fJerkCmdFlt1;
	float32  fJerkFbkFlt1;
	float32  fJerkCmdFlt2;
	float32  fJerkFbkFlt2;
	float32  jerk_cmd;
	float32  jerk_acc;

	float32  fAccCmdRaw;
	float32  fAccCmdOld;
	float32  fAccFbkRaw;
	float32  fAccFbkOld;


	float32  fJerkCmdAcc; // fJerkCmdAcc: Integrated jerk_cmd
	float32  fJerkFbkAcc; // fJerkFbkAcc: Integrated jerk_acc

	float32  JK_RATIO;      // JK_RATIO: fJerkFbkAcc/fJerkCmdAcc, is inverse proportionalto load to motor inertia ratio
	float32  J_HAT;         // Total inertia estemated
	float32  J_HATMax;

	float32  fJRaw;

	float32  fJRawBuf[256];

	float32  fAccCoef;
	float32  fCmdCoef;

	float32  fJTune;
	float32  fJRawAcc;
} StructTypeInertiaCalTypeDef;


PROTO StructTypeInertiaCalTypeDef stInertiaCal;

#undef  PROTO

#endif  // end of #ifndef _DATAEXCHANGE_H


//===========================================================================
// No more.
//===========================================================================








