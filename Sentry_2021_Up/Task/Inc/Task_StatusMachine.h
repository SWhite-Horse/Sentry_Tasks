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


//����ģʽ 
typedef enum
{
    ControlMode_Telecontrol_UP,            
    ControlMode_Aimbot,               
    ControlMode_Telecontrol_DOWN,            
}ControlMode_Enum;


//Ħ����״̬
typedef enum
{
    FricStatus_Stop,
    FricStatus_Working_Low,
    FricStatus_Working_High,
}FricStatus_Enum;

//����״̬
typedef enum
{
    StirStatus_Stop,
    StirStatus_SpeedControl,
    StirStatus_AngleControl,
}StirStatus_Enum;


//����״̬
typedef enum
{
	on,
	off,
	
}ShootStatus_Enum;




//״̬��
extern ControlMode_Enum         ControlMode;            //����ģʽ
extern FricStatus_Enum          FricStatus;             //Ħ����״̬
extern StirStatus_Enum          StirMotorStatus;				//���̵��״̬
extern ShootStatus_Enum         ShootStatus;						//���״̬

void StatusMachine_Init(void);
void StatusMachine_Update(void);
void Monitor(void);

#endif
