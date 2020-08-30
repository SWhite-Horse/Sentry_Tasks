#ifndef __TASKS_FRICMOTOR_H
#define __TASKS_FRICMOTOR_H

#ifdef __TASK_FRICMOTOR_GLOBALS
#define TASK_FRICMOTOR_EXT
#else 
#define TASK_FRICMOTOR_EXT extern
#endif

#include "Task_init.h"

//C620电调电流最大值
#define C620CURRENTMAX 											16000
#define ONECIRCLECOUNTVALUEAFTERDECELERATE  786432       //96*8192
#define ONECIRCLECOUNTVALUEORIGINAL         (uint16_t)8192 

#define CROSSZEROUPTHRESHOLD                6000
#define CROSSZERODOWNTHRESHOLD              2000

typedef struct
{
  uint16_t Mechanical_Angle;       //机械角
  int16_t TargetSpeed;
  int16_t RealSpeed;
  int16_t RealCurrent;
  uint16_t FrameCounter; 	//帧率计数器
	int32_t Output;
}Motor3508_type;

extern Motor3508_type Motor_3508[2];
void Fric_3508_CAN_Send(int16_t Output0,int16_t Output1);
void Task_Shoot(void *parameters);
void Motor_3508_Send(int16_t Output);


#endif
