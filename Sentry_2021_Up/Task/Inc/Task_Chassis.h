#ifndef __TASKS_CHASSIS_H
#define __TASKS_CHASSIS_H

#include "Task_Init.h"

//C620电调电流最大值
#define C620CURRENTMAX 16000
   
extern Motor3508_type Chassis_Motor[2];

void Chassis_Ctrl_Init(void);
void Chassis_Speed_Set(void);
void Chassis_PID_Ctrl(Motor3508_type *chassis);
void Chassis_CAN_Send(int16_t Output1,int16_t Output2);
void ChassisPowerControl(void);
#endif
