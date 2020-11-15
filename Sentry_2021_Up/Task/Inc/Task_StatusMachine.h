#ifndef __TASKS_STATUSMACHINE_H__
#define __TASKS_STATUSMACHINE_H__

#ifdef __TASK_STATUSMACHINE_GLOBALS
#define TASK_STATUSMACHINE_EXT
#else
#define TASK_STATUSMACHINE_EXT extern
#endif

#include "Task_init.h"
typedef enum
{
    chassis_stop = 0,
    chassis_run,
}chassis_mode_Enum;


//¿ØÖÆÄ£Ê½ 
typedef enum
{
    ControlMode_Telecontrol_UP,            
    ControlMode_Aimbot,               
    ControlMode_Telecontrol_DOWN,            
}ControlMode_Enum;


//Ä¦²ÁÂÖ×´Ì¬
typedef enum
{
    FricStatus_Stop,
    FricStatus_Working_Low,
    FricStatus_Working_High,
}FricStatus_Enum;

//²¦ÅÌ×´Ì¬
typedef enum
{
    StirStatus_Stop,
    StirStatus_SpeedControl,
    StirStatus_AngleControl,
}StirStatus_Enum;


//·¢Éä×´Ì¬
typedef enum
{
	on,
	off,
	
}ShootStatus_Enum;




//×´Ì¬Á¿
extern ControlMode_Enum         ControlMode;            //¿ØÖÆÄ£Ê½
extern FricStatus_Enum          FricStatus;             //Ä¦²ÁÂÖ×´Ì¬
extern StirStatus_Enum          StirMotorStatus;				//²¦ÅÌµç»ú×´Ì¬
extern ShootStatus_Enum         ShootStatus;						//Éä»÷×´Ì¬

void StatusMachine_Init(void);
void StatusMachine_Update(void);
void Monitor(void);

#endif
