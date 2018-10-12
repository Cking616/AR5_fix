//########################################################################################
//
//                  Copyright (c) 2018  XXX INC.
//
// File Name:       DataInterface.c
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

#ifndef _DATAINTERFACE_C
#define _DATAINTERFACE_C

#include "include_c.h"
#include "stm32f4xx_it.h"

const float fParaPrecision[6] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f, 100000.0f};
//########################################################################################
// Function Name:   ParaReadDefault
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void  ParaReadDefault(void)
{
	int32 i;
	float fDefaultTemp;

	//设置参数 用默认值初始化缓存变量值
    for(i = 0; i < SETPARA_NUM; i++)
    {
		switch (stDataInterface[i].stParaAtribt.iParaLengthOut)
        {
        case _B16:
            *(int16*)(stDataInterface[i].stParaInterface.pCacheData) = (int16)stDataInterface[i].stDataRange.lParaDefault ;
	    	*(int16*)(stDataInterface[i].stParaInterface.pControlData) = (int16)stDataInterface[i].stDataRange.lParaDefault ;
            break;
        case _B32:
            *(int32*)(stDataInterface[i].stParaInterface.pCacheData) = stDataInterface[i].stDataRange.lParaDefault ;
	    	*(int32*)(stDataInterface[i].stParaInterface.pControlData) = stDataInterface[i].stDataRange.lParaDefault ;
            break;
		case _F32:
			fDefaultTemp = (float)stDataInterface[i].stDataRange.lParaDefault / fParaPrecision[stDataInterface[i].stDataRange.lPrecision];
            *(float*)(stDataInterface[i].stParaInterface.pCacheData) = fDefaultTemp;
	    	*(float*)(stDataInterface[i].stParaInterface.pControlData) = fDefaultTemp;
            break;
        default: break;
        }
    }
}

//########################################################################################
// Function Name:   ParaReadBuffer
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void  ParaReadBuffer(void)
{
	int32 i;
	//设置参数 用默认值初始化缓存变量值
    for(i = 0; i < SETPARA_NUM; i++)
    {
		switch (stDataInterface[i].stParaAtribt.iParaLengthOut)
        {
        case _B16:
            *(int16*)(stDataInterface[i].stParaInterface.pCacheData) = (int16)stParaList.lBuffer[i];
	    	*(int16*)(stDataInterface[i].stParaInterface.pControlData) = (int16)stParaList.lBuffer[i];
            break;
		case _B32:
		case _F32:
			*(int32*)(stDataInterface[i].stParaInterface.pCacheData) = stParaList.lBuffer[i];
	    	*(int32*)(stDataInterface[i].stParaInterface.pControlData) = stParaList.lBuffer[i];
            break;
        default: break;
        }
    }
}


//########################################################################################
// Function Name:   DataInterfaceInit
// Version:         V1.0
// Input:           none
// Output:
// Description:     参数初始化
//########################################################################################
void  DataInterfaceInit(void)
{
	int32 lTemp;

	STMFLASH_Read(ADDR_FLASH_SECTOR_6,(Uint32*)&stParaList.lBuffer[0],(SETPARA_NUM + 1));
	CRC->CR |= 0x1;
	lTemp = CRC_CalcBlockCRC((Uint32*)&stParaList.lBuffer[0], SETPARA_NUM);
	if(lTemp != stParaList.lBuffer[SETPARA_NUM])
	{
		if (stParaList.lBuffer[SETPARA_NUM] != 0xFFFFFFFF)    //没有FLASH数据，不报故障。
		{
			//gsM1_Drive.sFaultId.unCtrlErr.bit.ParaFlashError = 1;
		}
		ParaReadDefault();
	}
	else
	{
		ParaReadBuffer();
	}
}


//########################################################################################
// Function Name:   PararecoverProcess
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void  PararecoverProcess(void)
{
    //设置参数 用默认值初始化缓存变量值
    ParaReadDefault();
	ParaSaveProcess();
}


//########################################################################################
// Function Name:   ParaSaveBuffer
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void  ParaSaveBuffer(void)
{
	int32 i;
	float fDefaultTemp;
	//设置参数 用默认值初始化缓存变量值
    for(i = 0; i < SETPARA_NUM; i++)
    {
		if(stDataInterface[i].stParaAtribt.bParaStorage == _SAVE)
		{
			switch (stDataInterface[i].stParaAtribt.iParaLengthOut)
            {
            case _B16:
                 stParaList.lBuffer[i]= (int32)(*(int16*)(stDataInterface[i].stParaInterface.pCacheData));
                break;
			case _B32:
			case _F32:
                 stParaList.lBuffer[i]= *(int32*)(stDataInterface[i].stParaInterface.pCacheData);
                break;
            default: break;
            }
		}
		else
		{
			switch (stDataInterface[i].stParaAtribt.iParaLengthOut)
            {
            case _B16:
                 stParaList.lBuffer[i] = (int16)stDataInterface[i].stDataRange.lParaDefault;
                break;
            case _B32:
                 stParaList.lBuffer[i] = stDataInterface[i].stDataRange.lParaDefault;
                break;
			case _F32:
				fDefaultTemp = (float)stDataInterface[i].stDataRange.lParaDefault / fParaPrecision[stDataInterface[i].stDataRange.lPrecision];
                stParaList.lBuffer[i] = fDefaultTemp;
			default: break;
            }
		}
    }
}


//########################################################################################
// Function Name:   ParaSaveProcess
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void  ParaSaveProcess(void)
{
    ParaSaveBuffer();
	__disable_irq();
    CRC->CR |= 0x1;
	stParaList.lBuffer[SETPARA_NUM] = CRC_CalcBlockCRC((Uint32 *)&stParaList.lBuffer[0], SETPARA_NUM);
	STMFLASH_Write(ADDR_FLASH_SECTOR_6,(Uint32*)&stParaList.lBuffer[0],(SETPARA_NUM + 1));
	__enable_irq();
}

//########################################################################################
// Function Name:   ParaSaveProcess
// Version:         V1.0
// Input:           none
// Output:
// Description:
//########################################################################################
void  SysResetProcess(void)
{
	NVIC_SystemReset();
}



//设置参数数据交换处理常量表:保证(参数个数/EEPROM页存储参数个数)为整数，如果空余则用保留参数填充
const StructTypetDataInterfaceTag stDataInterface[] =
{
/******************内部缓存数据***************************************内部控制数据************************************最小值*********最大值*********默认值********精度************特殊处理函数接口********存储******字长*****字长****生效方式****/
/*P0.00*/{{&stParaList.iRunEnable,                             &stParaList.iRunEnable                          }, {0,           1,              0,             0     },       &RunEnableCtrl,    { _UNSAVE, _B16,   _B16,    _INST }},/*P0.00*///
/*P0.01*/{{&stParaList.iAlignEnable,                           &stParaList.iAlignEnable                        }, {0,           1,              0,             0     },       &AlignCtrl,        { _UNSAVE, _B16,   _B16,    _INST }},/*P0.01*///
/*P0.02*/{{&gsM1_Drive.uw16CtrlMode,                           &gsM1_Drive.uw16CtrlMode                        }, {0,           6,              2,             0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.02*///
/*P0.03*/{{&stParaList.iRigidSet,                              &stParaList.iRigidSet                           }, {0,           31,             0,             0     },       NULL,         { _SAVE,   _B16,   _B16,    _INST }},/*P0.03*///
/*P0.04*/{{&stParaList.lSetIndex[4],                           &stParaList.lSetIndex[4]                        }, {-32768,      32767,          0,             0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.04*///
/*P0.05*/{{&stParaList.lSetIndex[5],                           &stParaList.lSetIndex[5]                        }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.05*///
/*P0.06*/{{&stParaList.lSetIndex[6],                           &stParaList.lSetIndex[6]                        }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.06*///
/*P0.07*/{{&stParaList.lSetIndex[7],                           &stParaList.lSetIndex[7]                        }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.07*///
/*P0.08*/{{&stParaList.lSetIndex[8],                           &stParaList.lSetIndex[8]                        }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.08*///
/*P0.09*/{{&stParaList.lSetIndex[9],                           &stParaList.lSetIndex[9]                        }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.09*///
/*P0.10*/{{&gsM1_Drive.sFocPMSM.sIDQReq.f32Q,                  &gsM1_Drive.sFocPMSM.sIDQReq.f32Q               }, {-10000,      10000,          0,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.10*///
/*P0.11*/{{&stParaList.lSetIndex[11],                          &stParaList.lSetIndex[11]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.11*///
/*P0.12*/{{&stCurLoop.fCurGain,                                &stCurLoop.fCurGain                             }, {1,           100000,         5000,          1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.12*///
/*P0.13*/{{&stCurLoop.fCurTi,                                  &stCurLoop.fCurTi                               }, {1,           5000,           60,            2     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.13*///
/*P0.14*/{{&stSpdLoop.fTrqFilterTime,                          &stSpdLoop.fTrqFilterTime                       }, {0,           2500,           84,            2     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.14*///
/*P0.15*/{{&stCurLoop.fCurFfwGain,                             &stCurLoop.fCurFfwGain                          }, {0,           1000,           700,           1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.15*///
/*P0.16*/{{&stSpdLoop.iAccTime,                                &stSpdLoop.iAccTime                             }, {0,           30000,          0,             0     },		  NULL,				 { _SAVE,   _B16,   _B16,    _INST }},/*P0.16*///
/*P0.17*/{{&stSpdLoop.iDecTime,                                &stSpdLoop.iDecTime                             }, {0,           30000,          0,             0     },		  NULL,				 { _SAVE,   _B16,   _B16,    _INST }},/*P0.17*///
/*P0.18*/{{&stSpdLoop.iAccSCurveTime,                          &stSpdLoop.iAccSCurveTime                       }, {0,           1000,           0,             0     },		  NULL,				 { _SAVE,   _B16,   _B16,    _INST }},/*P0.18*///
/*P0.19*/{{&stSpdLoop.iDecSCurveTime,                          &stSpdLoop.iDecSCurveTime                       }, {0,           1000,           0,             0     },		  NULL,				 { _SAVE,   _B16,   _B16,    _INST }},/*P0.19*///
/*P0.20*/{{&gsM1_Drive.sSpeed.f32SpeedCmd,                     &gsM1_Drive.sSpeed.f32SpeedCmd                  }, {-30000,      30000,          0,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.20*///
/*P0.21*/{{&stSpdLoop.fSpdGain,                                &stSpdLoop.fSpdGain                             }, {0,           30000,          270,           1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.21*///
/*P0.22*/{{&stSpdLoop.fSpdTi,                                  &stSpdLoop.fSpdTi                               }, {1,           10000,          210,           1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.22*///
/*P0.23*/{{&stSpdLoop.fSpdFbkFilter,                           &stSpdLoop.fSpdFbkFilter                        }, {1,           5000,           500,           0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.23*///
/*P0.24*/{{&stSpdLoop.fTrqFfwGain,                             &stSpdLoop.fTrqFfwGain                          }, {0,           1000,           0,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.24*///
/*P0.25*/{{&stSpdLoop.fTrqFfwFilterTime,                       &stSpdLoop.fTrqFfwFilterTime                    }, {0,           6400,           0,             2     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.25*///
/*P0.26*/{{&stSpdLoop.fJratio,                                 &stSpdLoop.fJratio                              }, {0,           10000,          300,           0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.26*///
/*P0.27*/{{&stParaList.lSetIndex[27],                          &stParaList.lSetIndex[27]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.27*///
/*P0.28*/{{&stParaList.lSetIndex[28],                          &stParaList.lSetIndex[28]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.28*///
/*P0.29*/{{&stPosLoop.iFltTime,                                &stPosLoop.iFltTime                             }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.29*///
/*P0.30*/{{&stPosLoop.lCmd,                                    &stPosLoop.lCmd                                 }, {-2147483648, 2147483647,     0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.30*///
/*P0.31*/{{&stPosLoop.iPosCmdMode,                             &stPosLoop.iPosCmdMode                          }, {0,           1,              1,             0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.31*///
/*P0.32*/{{&stPosLoop.fPosGain,                                &stPosLoop.fPosGain                             }, {0,           30000,          480,           1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.32*///
/*P0.33*/{{&stPosLoop.fSpdFfwGain,                             &stPosLoop.fSpdFfwGain                          }, {0,           1000,           0,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.33*///
/*P0.34*/{{&stPosLoop.fSpdFfwFilterTime,                       &stPosLoop.fSpdFfwFilterTime                    }, {0,           6400,           5,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.34*///
/*P0.35*/{{&stPosJogPara.iEzJogDir,                            &stPosJogPara.iEzJogDir,                        }, {0,           3,              0,             0     }, 	  NULL, 		     { _UNSAVE, _B16,   _B16,    _INST }},/*P0.35*///
/*P0.36*/{{&stPosJogPara.iEzJogMode,                           &stPosJogPara.iEzJogMode,                       }, {0,           6,              0,             0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.36*///
/*P0.37*/{{&stPosJogPara.lEzJogPosSet,                         &stPosJogPara.lEzJogPosSet,                     }, {1,           1073741824,     50000,         0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.37*///
/*P0.38*/{{&stPosJogPara.iEzJogSpdSet,                         &stPosJogPara.iEzJogSpdSet,                     }, {1,           5000,           500,           0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.38*///
/*P0.39*/{{&stPosJogPara.iEzJogAccTime,                        &stPosJogPara.iEzJogAccTime,                    }, {2,           10000,          100,           0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.39*///
/*P0.40*/{{&stPosJogPara.iEzJogDlyTime,                        &stPosJogPara.iEzJogDlyTime,                    }, {0,           10000,          100,           0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.40*///
/*P0.41*/{{&stPosJogPara.iEzJogCycleSet,                       &stPosJogPara.iEzJogCycleSet,                   }, {0,           10000,          1,             0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.41*///
/*P0.42*/{{&stParaList.lSetIndex[42],                          &stParaList.lSetIndex[42]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.42*///
/*P0.43*/{{&stParaList.lSetIndex[43],                          &stParaList.lSetIndex[43]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.43*///
/*P0.44*/{{&stParaList.lSetIndex[44],                          &stParaList.lSetIndex[44]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.44*///
/*P0.45*/{{&stParaList.lSetIndex[45],                          &stParaList.lSetIndex[45]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.45*///
/*P0.46*/{{&stParaList.lSetIndex[46],                          &stParaList.lSetIndex[46]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.46*///
/*P0.47*/{{&stParaList.lSetIndex[47],                          &stParaList.lSetIndex[47]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.47*///
/*P0.48*/{{&stParaList.lSetIndex[48],                          &stParaList.lSetIndex[48]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.48*///
/*P0.49*/{{&stParaList.lSetIndex[49],                          &stParaList.lSetIndex[49]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.49*///
/*P0.50*/{{&stTestCtrl.iTstSglEnable,                          &stTestCtrl.iTstSglEnable                       }, {0,           1,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.50*///
/*P0.51*/{{&stTestCtrl.iTstSglType,                            &stTestCtrl.iTstSglType                         }, {0,           3,              0,             0     },   	  NULL,    			 { _SAVE,   _B16,   _B16,    _INST }},/*P0.51*///
/*P0.52*/{{&stTestCtrl.fTstSglFreq,                            &stTestCtrl.fTstSglFreq                         }, {0,           100000000,      0,             2     },       NULL,  			 { _SAVE,   _F32,   _F32,    _INST }},/*P0.52*///
/*P0.53*/{{&stTestCtrl.fTstSglAmp,                             &stTestCtrl.fTstSglAmp                          }, {0,           100000000,      0,             2     },   	  NULL,  			 { _SAVE,   _F32,   _F32,    _INST }},/*P0.53*///
/*P0.54*/{{&stTestCtrl.fTstSglOffset,                          &stTestCtrl.fTstSglOffset                       }, {-100000000,  100000000,      0,             2     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.54*///
/*P0.55*/{{&stTestCtrl.fTstSglPhase,                           &stTestCtrl.fTstSglPhase                        }, {0,           3599,           0,             2     },  	  NULL,				 { _SAVE,   _F32,   _F32,    _INST }},/*P0.55*///
/*P0.56*/{{&stTestCtrl.fTstSglRatio,                           &stTestCtrl.fTstSglRatio                        }, {0,           1000,           200,           1     },   	  NULL,				 { _SAVE,   _F32,   _F32,    _INST }},/*P0.56*///
/*P0.57*/{{&stTestCtrl.lTstSglCycles,                          &stTestCtrl.lTstSglCycles                       }, {0,           2147483647,     0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.57*///
/*P0.58*/{{&stTestCtrl.iDestOutType,                           &stTestCtrl.iDestOutType                        }, {0,           3,              0,             0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.58*///
/*P0.59*/{{&stTestCtrl.iThetaMode,                             &stTestCtrl.iThetaMode                          }, {0,           2,              0,             0     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.59*///
/*P0.60*/{{&stTestCtrl.fThetaMan,                              &stTestCtrl.fThetaMan                           }, {0,           3599,           0,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.60*///
/*P0.61*/{{&stTestCtrl.fOmegaEleMan,                           &stTestCtrl.fOmegaEleMan                        }, {0,           10000,          0,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.61*///
/*P0.62*/{{&stTestCtrl.fVdCmd,                                 &stTestCtrl.fVdCmd                              }, {0,           10000,          0,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.62*///
/*P0.63*/{{&stTestCtrl.fVqCmd,                                 &stTestCtrl.fVqCmd                              }, {0,           10000,          0,             1     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.63*///
/*P0.64*/{{&stTestCtrl.fIdCmd,                                 &stTestCtrl.fIdCmd                              }, {0,           100000,         0,             2     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.64*///
/*P0.65*/{{&stTestCtrl.fIqCmd,                                 &stTestCtrl.fIqCmd                              }, {0,           100000,         0,             2     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*P0.65*///
/*P0.66*/{{&stParaList.lSetIndex[66],                          &stParaList.lSetIndex[66]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.66*///
/*P0.67*/{{&stParaList.lSetIndex[67],                          &stParaList.lSetIndex[67]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.67*///
/*P0.68*/{{&stParaList.lSetIndex[68],                          &stParaList.lSetIndex[68]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.68*///
/*P0.69*/{{&stParaList.lSetIndex[69],                          &stParaList.lSetIndex[69]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.69*///
/*P0.70*/{{&stParaList.iParaSave,                              &stParaList.iParaSave                           }, {0,           1,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.70*///
/*P0.71*/{{&stParaList.iParaRecover,                           &stParaList.iParaRecover                        }, {0,           1,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.71*///
/*P0.72*/{{&stParaList.iSysReset,                              &stParaList.iSysReset                           }, {0,           1,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.72*///
/*P0.73*/{{&stParaList.lSetIndex[73],                          &stParaList.lSetIndex[73]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.73*///
/*P0.74*/{{&stParaList.lSetIndex[74],                          &stParaList.lSetIndex[74]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.74*///
/*P0.75*/{{&stParaList.lSetIndex[75],                          &stParaList.lSetIndex[75]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.75*///
/*P0.76*/{{&stParaList.lSetIndex[76],                          &stParaList.lSetIndex[76]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.76*///
/*P0.77*/{{&stParaList.lSetIndex[77],                          &stParaList.lSetIndex[77]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.77*///
/*P0.78*/{{&stParaList.lSetIndex[78],                          &stParaList.lSetIndex[78]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.78*///
/*P0.79*/{{&stParaList.lSetIndex[79],                          &stParaList.lSetIndex[79]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.79*///
/*P0.80*/{{&stScopePara.iTransEnable,                          &stScopePara.iTransEnable,                      }, {0,           2,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.80*///
/*P0.81*/{{&stScopePara.ulChIndex[0],                          &stScopePara.ulChIndex[0],                      }, {0x0,         0xFFFFFF,       0x300000,      0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.81*///
/*P0.82*/{{&stScopePara.ulChIndex[1],                          &stScopePara.ulChIndex[1],                      }, {0x0,         0xFFFFFF,       0x300100,      0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.82*///
/*P0.83*/{{&stScopePara.ulChIndex[2],                          &stScopePara.ulChIndex[2],                      }, {0x0,         0xFFFFFF,       0x300A00,      0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.83*///
/*P0.84*/{{&stScopePara.ulChIndex[3],                          &stScopePara.ulChIndex[3],                      }, {0x0,         0xFFFFFF,       0x301400,      0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.84*///
/*P0.85*/{{&stScopePara.pChAddr[0],                            &stScopePara.pChAddr[0],                        }, {0x0,         0x7FFFFFFF,     0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.85*///
/*P0.86*/{{&stScopePara.pChAddr[1],                            &stScopePara.pChAddr[1],                        }, {0x0,         0x7FFFFFFF,     0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.86*///
/*P0.87*/{{&stScopePara.pChAddr[2],                            &stScopePara.pChAddr[2],                        }, {0x0,         0x7FFFFFFF,     0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.87*///
/*P0.88*/{{&stScopePara.pChAddr[3],                            &stScopePara.pChAddr[3],                        }, {0x0,         0x7FFFFFFF,     0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.88*///
/*P0.89*/{{&stScopePara.iChMode[0],                            &stScopePara.iChMode[0],                        }, {0,           1,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.89*///
/*P0.90*/{{&stScopePara.iChMode[1],                            &stScopePara.iChMode[1],                        }, {0,           1,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.90*///
/*P0.91*/{{&stScopePara.iChMode[2],                            &stScopePara.iChMode[2],                        }, {0,           1,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.91*///
/*P0.92*/{{&stScopePara.iChMode[3],                            &stScopePara.iChMode[3],                        }, {0,           1,              0,             0     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.92*///
/*P0.93*/{{&stScopePara.iChParaLength[0],                      &stScopePara.iChParaLength[0],                  }, {0,           3,              3,             3     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.93*///
/*P0.94*/{{&stScopePara.iChParaLength[1],                      &stScopePara.iChParaLength[1],                  }, {0,           3,              3,             3     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.94*///
/*P0.95*/{{&stScopePara.iChParaLength[2],                      &stScopePara.iChParaLength[2],                  }, {0,           3,              3,             3     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.95*///
/*P0.96*/{{&stScopePara.iChParaLength[3],                      &stScopePara.iChParaLength[3],                  }, {0,           3,              3,             3     },       NULL,              { _UNSAVE, _B16,   _B16,    _INST }},/*P0.96*///
/*P0.97*/{{&stScopePara.iCommType,                             &stScopePara.iCommType                          }, {0,           3,              1,             1     },       NULL,              { _SAVE,   _B16,   _B16,    _INST }},/*P0.97*///
/*P0.98*/{{&stParaList.lSetIndex[98],                          &stParaList.lSetIndex[98]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.98*///
/*P0.99*/{{&stParaList.lSetIndex[99],                          &stParaList.lSetIndex[99]                       }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*P0.99*///


/*R0.00*/{{&gsM1_Drive.sFocPMSM.sIDQ.f32Q,                     &gsM1_Drive.sFocPMSM.sIDQ.f32Q                  }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.00*///
/*R0.01*/{{&gsM1_Drive.sFocPMSM.sIDQ.f32Q,                     &gsM1_Drive.sFocPMSM.sIDQ.f32Q                  }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.01*///
/*R0.02*/{{&gsM1_Drive.sFocPMSM.sIDQReq.f32D,                  &gsM1_Drive.sFocPMSM.sIDQReq.f32D,              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.02*///
/*R0.03*/{{&gsM1_Drive.sFocPMSM.sIDQ.f32D,                     &gsM1_Drive.sFocPMSM.sIDQ.f32D,                 }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.03*///
/*R0.04*/{{&gsM1_Drive.sFocPMSM.sIDQReq.f32Q,                  &gsM1_Drive.sFocPMSM.sIDQReq.f32Q,              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.04*///
/*R0.05*/{{&gsM1_Drive.sFocPMSM.sIDQ.f32Q,                     &gsM1_Drive.sFocPMSM.sIDQ.f32Q,                 }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.05*///
/*R0.06*/{{&gsM1_Drive.sFocPMSM.sUDQReq.f32D,                  &gsM1_Drive.sFocPMSM.sUDQReq.f32D,              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.06*///
/*R0.07*/{{&gsM1_Drive.sFocPMSM.sUDQReq.f32Q,                  &gsM1_Drive.sFocPMSM.sUDQReq.f32Q,              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.07*///
/*R0.08*/{{&stCurLoop.fVdFfw,                                  &stCurLoop.fVdFfw,                              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.08*///
/*R0.09*/{{&stCurLoop.fVqFfw,                                  &stCurLoop.fVqFfw,                              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.09*///
/*R0.10*/{{&gsM1_Drive.sSpeed.f32Speed,                        &gsM1_Drive.sSpeed.f32Speed,                    }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.10*///
/*R0.11*/{{&gsM1_Drive.sSpeed.f32SpeedReq,                     &gsM1_Drive.sSpeed.f32SpeedReq,                 }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.11*///
/*R0.12*/{{&stSpdLoop.fTrqFfwOut,                              &stSpdLoop.fTrqFfwOut,                          }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.12*///
/*R0.13*/{{&stInertiaCal.fJTune,                               &stInertiaCal.fJTune                            }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.13*///
/*R0.14*/{{&stParaList.lReadIndex[14],                         &stParaList.lReadIndex[14]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.14*///
/*R0.15*/{{&stParaList.lReadIndex[15],                         &stParaList.lReadIndex[15]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.15*///
/*R0.16*/{{&stParaList.lReadIndex[16],                         &stParaList.lReadIndex[16]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.16*///
/*R0.17*/{{&stParaList.lReadIndex[17],                         &stParaList.lReadIndex[17]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.17*///
/*R0.18*/{{&stParaList.lReadIndex[18],                         &stParaList.lReadIndex[18]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.18*///
/*R0.19*/{{&stParaList.lReadIndex[19],                         &stParaList.lReadIndex[19]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.19*///
/*R0.20*/{{&stPosLoop.lFbkAcc,                                 &stPosLoop.lFbkAcc                              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.20*///
/*R0.21*/{{&stPosLoop.lCmdAcc,                                 &stPosLoop.lCmdAcc                              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.21*///
/*R0.22*/{{&stPosLoop.lFbkInc,                                 &stPosLoop.lFbkInc                              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.22*///
/*R0.23*/{{&stPosLoop.lCmdInc,                                 &stPosLoop.lCmdInc                              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.23*///
/*R0.24*/{{&stPosLoop.lErrAcc,                                 &stPosLoop.lErrAcc                              }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.24*///
/*R0.25*/{{&stPosLoop.fSpdFfwOut,                              &stPosLoop.fSpdFfwOut                           }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.25*///
/*R0.26*/{{&stPosLoop.lPosCmdFltOut,                           &stPosLoop.lPosCmdFltOut                        }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.26*///
/*R0.27*/{{&stParaList.lReadIndex[27],                         &stParaList.lReadIndex[27]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.27*///
/*R0.28*/{{&stParaList.lReadIndex[28],                         &stParaList.lReadIndex[28]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.28*///
/*R0.29*/{{&stParaList.lReadIndex[29],                         &stParaList.lReadIndex[29]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.29*///
/*R0.30*/{{&lResetCnt,                                         &lResetCnt                                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.30*///
/*R0.31*/{{&stTaskStat[0].fRatio,                              &stTaskStat[0].fRatio                           }, {0,           10000,          0,             2     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.31*///
/*R0.32*/{{&stTaskStat[0].fRatioMax,                           &stTaskStat[0].fRatioMax                        }, {0,           10000,          0,             2     },       NULL,              { _SAVE,   _F32,   _F32,    _INST }},/*R0.32*///
/*R0.33*/{{&stParaList.lReadIndex[33],                         &stParaList.lReadIndex[33]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.33*///
/*R0.34*/{{&stParaList.lReadIndex[34],                         &stParaList.lReadIndex[34]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.34*///
/*R0.35*/{{&stParaList.lReadIndex[35],                         &stParaList.lReadIndex[35]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.35*///
/*R0.36*/{{&stParaList.lReadIndex[36],                         &stParaList.lReadIndex[36]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.36*///
/*R0.37*/{{&stParaList.lReadIndex[37],                         &stParaList.lReadIndex[37]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.37*///
/*R0.38*/{{&stParaList.lReadIndex[38],                         &stParaList.lReadIndex[38]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.38*///
/*R0.39*/{{&stParaList.lReadIndex[39],                         &stParaList.lReadIndex[39]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.39*///
/*R0.40*/{{&stParaList.lReadIndex[40],                         &stParaList.lReadIndex[40]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.40*///
/*R0.41*/{{&stParaList.lReadIndex[41],                         &stParaList.lReadIndex[41]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.41*///
/*R0.42*/{{&stParaList.lReadIndex[42],                         &stParaList.lReadIndex[42]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.42*///
/*R0.43*/{{&stParaList.lReadIndex[43],                         &stParaList.lReadIndex[43]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.43*///
/*R0.44*/{{&stParaList.lReadIndex[44],                         &stParaList.lReadIndex[44]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.44*///
/*R0.45*/{{&stParaList.lReadIndex[45],                         &stParaList.lReadIndex[45]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.45*///
/*R0.46*/{{&stParaList.lReadIndex[46],                         &stParaList.lReadIndex[46]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.46*///
/*R0.47*/{{&stParaList.lReadIndex[47],                         &stParaList.lReadIndex[47]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.47*///
/*R0.48*/{{&stParaList.lReadIndex[48],                         &stParaList.lReadIndex[48]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.48*///
/*R0.49*/{{&stParaList.lReadIndex[49],                         &stParaList.lReadIndex[49]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.49*///
/*R0.50*/{{&stParaList.lReadIndex[50],                         &stParaList.lReadIndex[50]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.50*///
/*R0.51*/{{&stParaList.lReadIndex[51],                         &stParaList.lReadIndex[51]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.51*///
/*R0.52*/{{&stParaList.lReadIndex[52],                         &stParaList.lReadIndex[52]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.52*///
/*R0.53*/{{&stParaList.lReadIndex[53],                         &stParaList.lReadIndex[53]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.53*///
/*R0.54*/{{&stParaList.lReadIndex[54],                         &stParaList.lReadIndex[54]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.54*///
/*R0.55*/{{&stParaList.lReadIndex[55],                         &stParaList.lReadIndex[55]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.55*///
/*R0.56*/{{&stParaList.lReadIndex[56],                         &stParaList.lReadIndex[56]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.56*///
/*R0.57*/{{&stParaList.lReadIndex[57],                         &stParaList.lReadIndex[57]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.57*///
/*R0.58*/{{&stParaList.lReadIndex[58],                         &stParaList.lReadIndex[58]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.58*///
/*R0.59*/{{&stParaList.lReadIndex[59],                         &stParaList.lReadIndex[59]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.59*///
/*R0.60*/{{&stParaList.lReadIndex[60],                         &stParaList.lReadIndex[60]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.60*///
/*R0.61*/{{&stParaList.lReadIndex[61],                         &stParaList.lReadIndex[61]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.61*///
/*R0.62*/{{&stParaList.lReadIndex[62],                         &stParaList.lReadIndex[62]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.62*///
/*R0.63*/{{&stParaList.lReadIndex[63],                         &stParaList.lReadIndex[63]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.63*///
/*R0.64*/{{&stParaList.lReadIndex[64],                         &stParaList.lReadIndex[64]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.64*///
/*R0.65*/{{&stParaList.lReadIndex[65],                         &stParaList.lReadIndex[65]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.65*///
/*R0.66*/{{&stParaList.lReadIndex[66],                         &stParaList.lReadIndex[66]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.66*///
/*R0.67*/{{&stParaList.lReadIndex[67],                         &stParaList.lReadIndex[67]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.67*///
/*R0.68*/{{&stParaList.lReadIndex[68],                         &stParaList.lReadIndex[68]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.68*///
/*R0.69*/{{&stParaList.lReadIndex[69],                         &stParaList.lReadIndex[69]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.69*///
/*R0.70*/{{&stParaList.lReadIndex[70],                         &stParaList.lReadIndex[70]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.70*///
/*R0.71*/{{&stParaList.lReadIndex[71],                         &stParaList.lReadIndex[71]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.71*///
/*R0.72*/{{&stParaList.lReadIndex[72],                         &stParaList.lReadIndex[72]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.72*///
/*R0.73*/{{&stParaList.lReadIndex[73],                         &stParaList.lReadIndex[73]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.73*///
/*R0.74*/{{&stParaList.lReadIndex[74],                         &stParaList.lReadIndex[74]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.74*///
/*R0.75*/{{&stParaList.lReadIndex[75],                         &stParaList.lReadIndex[75]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.75*///
/*R0.76*/{{&stParaList.lReadIndex[76],                         &stParaList.lReadIndex[76]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.76*///
/*R0.77*/{{&stParaList.lReadIndex[77],                         &stParaList.lReadIndex[77]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.77*///
/*R0.78*/{{&stParaList.lReadIndex[78],                         &stParaList.lReadIndex[78]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.78*///
/*R0.79*/{{&stParaList.lReadIndex[79],                         &stParaList.lReadIndex[79]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.79*///
/*R0.80*/{{&stParaList.lReadIndex[80],                         &stParaList.lReadIndex[80]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.80*///
/*R0.81*/{{&stParaList.lReadIndex[81],                         &stParaList.lReadIndex[81]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.81*///
/*R0.82*/{{&stParaList.lReadIndex[82],                         &stParaList.lReadIndex[82]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.82*///
/*R0.83*/{{&stParaList.lReadIndex[83],                         &stParaList.lReadIndex[83]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.83*///
/*R0.84*/{{&stParaList.lReadIndex[84],                         &stParaList.lReadIndex[84]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.84*///
/*R0.85*/{{&stParaList.lReadIndex[85],                         &stParaList.lReadIndex[85]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.85*///
/*R0.86*/{{&stParaList.lReadIndex[86],                         &stParaList.lReadIndex[86]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.86*///
/*R0.87*/{{&stParaList.lReadIndex[87],                         &stParaList.lReadIndex[87]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.87*///
/*R0.88*/{{&stParaList.lReadIndex[88],                         &stParaList.lReadIndex[88]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.88*///
/*R0.89*/{{&stParaList.lReadIndex[89],                         &stParaList.lReadIndex[89]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.89*///
/*R0.90*/{{&stParaList.lReadIndex[90],                         &stParaList.lReadIndex[90]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.90*///
/*R0.91*/{{&stParaList.lReadIndex[91],                         &stParaList.lReadIndex[91]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.91*///
/*R0.92*/{{&stParaList.lReadIndex[92],                         &stParaList.lReadIndex[92]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.92*///
/*R0.93*/{{&stParaList.lReadIndex[93],                         &stParaList.lReadIndex[93]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.93*///
/*R0.94*/{{&stParaList.lReadIndex[94],                         &stParaList.lReadIndex[94]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.94*///
/*R0.95*/{{&stParaList.lReadIndex[95],                         &stParaList.lReadIndex[95]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.95*///
/*R0.96*/{{&stParaList.lReadIndex[96],                         &stParaList.lReadIndex[96]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.96*///
/*R0.97*/{{&stParaList.lReadIndex[97],                         &stParaList.lReadIndex[97]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.97*///
/*R0.98*/{{&stParaList.lReadIndex[98],                         &stParaList.lReadIndex[98]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }},/*R0.98*///
/*R0.99*/{{&stParaList.lReadIndex[99],                         &stParaList.lReadIndex[99]                      }, {0,           10000,          0,             0     },       NULL,              { _SAVE,   _B32,   _B32,    _INST }}/*R0.99*///
};


//#####################################################################################
// Function Name:   DataIndexSearch
// Description:
// Input:
// Output:
// Execute time:
//#####################################################################################
int16 DataIndexSearch(Uint32 ulParaIndex)
{
	int32 iDataIndex;
	ulParaIndex = ulParaIndex >> 8;
	if ((ulParaIndex >= 0x2000) && (ulParaIndex < 0x2064))
	{
		iDataIndex = (ulParaIndex - 0x2000);
	}
	else if((ulParaIndex >= 0x3000) && (ulParaIndex < 0x3064))
	{
		iDataIndex = (ulParaIndex - 0x3000) + 100;
	}
	else
	{
		return -1;
	}
	return iDataIndex;
}

//#####################################################################################
// Function Name:   DataInterface
// Description:     1接收外部数据，存入参数列表、RAM变量、EEPROM；2 输出RAM变量
// Input:           iDataDirection, uiCommIndex, taskID
// Output:
// Execute time:
//#####################################################################################
int32 DataInterface(Uint16 uiProtocolType, Uint16 uiDataDirection, Uint32 ulParaIndex, Uint16* pParaLength, int32 *pDataBuf)
{
	int16 iParaInRefresh   = 0;    //内部参数更新标志0:不更新 1:更新
	int16 iDataIndex       = 0;    //数据Index
	int32 lParaIn          = 0;    //输入参数值
	float fParaIn          = 0;    //输入参数值

	iDataIndex = DataIndexSearch(ulParaIndex);
	if(iDataIndex == -1)
	{
		return OD_NO_SUCH_OBJECT;
	}

	if (uiDataDirection ==  COMM_DATA_INPUT)
	{
		switch (stDataInterface[iDataIndex].stParaAtribt.iParaLengthOut)
	    {
		case _B16:
			lParaIn    = *(int16*)(pDataBuf);
			if (lParaIn > stDataInterface[iDataIndex].stDataRange.lParaMax)
		    {
		    	return OD_VALUE_TOO_HIGH;
		    }
			else if (lParaIn < stDataInterface[iDataIndex].stDataRange.lParaMin)
		    {
		    	return OD_VALUE_TOO_LOW;
		    }
			*(int16*)(stDataInterface[iDataIndex].stParaInterface.pCacheData) = (int16)lParaIn;
			break;
		case _B32:
			lParaIn    = *(int32*)(pDataBuf);
			if(lParaIn > stDataInterface[iDataIndex].stDataRange.lParaMax)
		    {
		    	return OD_VALUE_TOO_HIGH;
		    }
		    else if(lParaIn < stDataInterface[iDataIndex].stDataRange.lParaMin )
		    {
		    	return OD_VALUE_TOO_LOW;
		    }
			*(int32*)(stDataInterface[iDataIndex].stParaInterface.pCacheData) = (int32)lParaIn;
			break;
		case _F32:
			fParaIn    = *(float*)(pDataBuf);
			if(fParaIn > ((float)stDataInterface[iDataIndex].stDataRange.lParaMax / fParaPrecision[stDataInterface[iDataIndex].stDataRange.lPrecision]))
		    {
		    	return OD_VALUE_TOO_HIGH;
		    }
		    else if(fParaIn < ((float)stDataInterface[iDataIndex].stDataRange.lParaMin / fParaPrecision[stDataInterface[iDataIndex].stDataRange.lPrecision]))
		    {
		    	return OD_VALUE_TOO_LOW;
		    }
			*(float*)(stDataInterface[iDataIndex].stParaInterface.pCacheData) = fParaIn;
			break;
		default:
			break;
	    }

		/***生效方式***/
		switch(stDataInterface[iDataIndex].stParaAtribt.bParaEffect)
		{
		case _INST:
			iParaInRefresh = 1;
			break;
		case _RESET:
			iParaInRefresh = 0;
			break;
		default:
			break;
		}

		if(iParaInRefresh == 1)
		{
			// 更新内部控制变量
			switch (stDataInterface[iDataIndex].stParaAtribt.iParaLengthIn)
			{
			case _B16:
				*(int16*)(stDataInterface[iDataIndex].stParaInterface.pControlData) = (int16)lParaIn;
				break;
			case _B32:
				*(int32*)(stDataInterface[iDataIndex].stParaInterface.pControlData) = (int32)lParaIn;
				break;
			case _F32:
				*(float*)(stDataInterface[iDataIndex].stParaInterface.pControlData) = fParaIn;
				break;
			default:
				break;
			}
		}

		/***特殊处理***/
		if(stDataInterface[iDataIndex].pSpecialProcess == NULL)
		{
			return OD_SUCCESSFUL;
		}
		else
		{
			stDataInterface[iDataIndex].pSpecialProcess(uiProtocolType, COMM_DATA_INPUT, iDataIndex, &lParaIn);
		}
	}
	else if(uiDataDirection == COMM_DATA_OUTPUT)
	{
		switch(stDataInterface[iDataIndex].stParaAtribt.iParaLengthOut)
		{
		case _B16:
			*pDataBuf   = *(int16*)stDataInterface[iDataIndex].stParaInterface.pCacheData;
			*pParaLength = _B16;
			break;
		case _B32:
		case _F32:
			*pDataBuf   = *(Uint32*)stDataInterface[iDataIndex].stParaInterface.pCacheData;
			*pParaLength = _B32;
			break;
		default:
			break;
		}
	}
	return OD_SUCCESSFUL;
}

//########################################################################################
// Function Name:   ScopeDataProcess
// Version:         V1.0
// Input:           none
// Output:
// Description:     发送快帧
//########################################################################################
int16 ScopeDataProcess(int16 iChMode, int16 iDataLength, Uint32 ulParaIndex, void** pDataAddr, Uint32* pData)
{
	int16   iDataIndex;

	iDataIndex = DataIndexSearch(ulParaIndex);
	if(iDataIndex == -1)
	{
		return FALSE;
	}

	if (iChMode == SCOPE_MODE_INDEX)
	{
	   switch(stDataInterface[iDataIndex].stParaAtribt.iParaLengthIn)
	   {
	   case _B16:
	       *pData = (Uint32)(*(int16*)(stDataInterface[iDataIndex].stParaInterface.pControlData));
		   break;
	   case _B32:
	   case _F32:
	       *(Uint32*)pData = *(Uint32*)(stDataInterface[iDataIndex].stParaInterface.pControlData);
	       break;

	   default:
	       break;
		}
	}
	else
	{
		switch (iDataLength)
		{
		case _B16:
			*pData = (Uint32)(*(Uint16*)(*pDataAddr));
			break;
		case _B32:
			*(Uint32*)pData = *(Uint32*)(*pDataAddr);
			break;
		 default:
	       break;
		}
	}
	return TRUE;
}


//########################################################################################
// Function Name:   EnableCtrl
// Version:         V1.0
// Input:           none
// Output:
// Description:     发送快帧
//########################################################################################
void RunEnableCtrl(Uint16 uiProtocolType,Uint16 uiDataDirection, int16 iDataListIndex, int32 *pDataBuf)
{
	if(uiDataDirection == COMM_DATA_INPUT)
    {
		if (stParaList.iRunEnable == 1)
		{
			gsM1_Ctrl.uiCtrl = SM_CTRL_START;
		}
		else
		{
			gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP;
		}
    }
}

//########################################################################################
// Function Name:   AlignCtrl
// Version:         V1.0
// Input:           none
// Output:
// Description:     发送快帧
//########################################################################################
void AlignCtrl(Uint16 uiProtocolType,Uint16 uiDataDirection, int16 iDataListIndex, int32 *pDataBuf)
{
	if(uiDataDirection == COMM_DATA_INPUT)
    {
		if (stParaList.iAlignEnable == 1)
		{
			gsM1_Ctrl.uiCtrl = SM_CTRL_ALIGN;
			stParaList.iAlignEnable = 0;
		}
		else
		{
			gsM1_Ctrl.uiCtrl |= SM_CTRL_STOP;
		}
    }
}

#endif  // end of #ifndef _DATAEXCHANGE_C

//===========================================================================
// No more.
//===========================================================================



