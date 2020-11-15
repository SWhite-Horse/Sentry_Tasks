#ifndef __TASKS_FRICMOTOR_H
#define __TASKS_FRICMOTOR_H

#ifdef __TASK_FRICMOTOR_GLOBALS
#define TASK_FRICMOTOR_EXT
#else 
#define TASK_FRICMOTOR_EXT extern
#endif

#include "Task_Init.h"
#include "Task_Chassis.h"

//C620电调电流最大值
#define C620CURRENTMAX 											16000
#define ONECIRCLECOUNTVALUEAFTERDECELERATE  786432       //96*8192
#define ONECIRCLECOUNTVALUEORIGINAL         (uint16_t)8192 

#define CROSSZEROUPTHRESHOLD                6000
#define CROSSZERODOWNTHRESHOLD              2000

#define C610CURRENTMAX                      10000

typedef struct{
    uint16_t FrameCounter;
    int16_t  RealSpeed;
		int16_t  RealCurrent;
    int16_t  Mechanical_Angle;
    int16_t  TargetSpeed;
    int8_t  BlockedWarningTimes;
		PID_type PID;
	  int32_t Output;
} RM2006_Type;

extern Motor3508_type Fric_Motor_3508[2];

void Motor_3508_Send(int16_t Output);
void Motor_3508_PID_Init(void);
void Motor_3508_PID_Calculate(Motor3508_type *motor);

void StirMotor_Blocked_Detect(RM2006_Type* motor);
void Stir_Motor_Speed_Control(RM2006_Type* motor);
void StirMotor_Init(void);
void StirMotor_Control(void);
void Stir_CAN_Send(int16_t Output);


TASK_FRICMOTOR_EXT RM2006_Type  StirMotor;


#endif
