#ifndef _M1_STATEMACHINE_H_
#define _M1_STATEMACHINE_H_

#include "StateMachine.h"
#include "motor_structure.h"

typedef enum {
    FAST = 0,
    SLOW = 1,
} MCSTRUC_CONTROL_LOOP_T;

extern SM_APP_CTRL_T gsM1_Ctrl;
extern MCSTRUC_CONTROL_LOOP_T geM1_StateRunLoop;
extern MCSTRUC_FOC_PMSM_ENCODER_T gsM1_Drive;
extern SM_RUN_SUBSTATE_T meM1_StateRun;

#endif /* STATEMACHINE */
