#ifndef __TASKS_FRICMOTOR_H
#define __TASKS_FRICMOTOR_H

#ifdef __TASK_FRICMOTOR_GLOBALS
#define TASK_FRICMOTOR_EXT
#else 
#define TASK_FRICMOTOR_EXT extern
#endif

#include "Task_init.h"

//C620����������ֵ
#define C620CURRENTMAX 											16000
#define ONECIRCLECOUNTVALUEAFTERDECELERATE  786432       //96*8192
#define ONECIRCLECOUNTVALUEORIGINAL         (uint16_t)8192 

#define CROSSZEROUPTHRESHOLD                6000
#define CROSSZERODOWNTHRESHOLD              2000


typedef struct                                   //PID�ṹ��
{
	float Kp, Ki, Kd;
  float Cur_Error, Last_Error, Sum_Error;
}PID_type;

typedef struct
{
  uint16_t Mechanical_Angle;       //��е��
  int16_t TargetSpeed;
  int16_t RealSpeed;
  int16_t RealCurrent;
  uint16_t FrameCounter; 	//֡�ʼ�����
	PID_type PID;
	int32_t Output;
}Motor3508_type;

extern Motor3508_type Motor_3508[2];

void Task_Shoot(void *parameters);
void Motor_3508_Send(int16_t Output);
void Motor_3508_PID_Init(void);
void Motor_3508_PID_Calculate(Motor3508_type *motor);


#endif
