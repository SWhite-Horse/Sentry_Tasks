#ifndef __TASKS_CHASSIS_H
#define __TASKS_CHASSIS_H

#include "Task_Init.h"

//C620电调电流最大值
#define C620CURRENTMAX 16000
   
#define CHASSIS_HIGH_SPEED 7500 //待定
#define CHASSIS_NORMAL_SPEED 5800
#define CHASSIS_SHOOT_SPEED 0

#define FLAG_LEFT 1
#define FLAG_RIGHT 0
#define MEASURE_CIRCLE 45

typedef struct{
	uint16_t Chassis_speed;
	uint8_t Fucking_pattrn;
	uint16_t Chassis_Power;
	
	
}Chassis_ZouWei_struct;

typedef struct{
	uint8_t Flag;
	uint8_t Rand_Numb;
	uint16_t Measure_tick;
	uint16_t Speedup_tick;
}Rand_Walk_struct;


typedef struct{
	uint8_t Is_Finished;
	uint8_t Shoot_Count;
	uint8_t Location;
	uint8_t Location_Flag;
	uint8_t Gimbal_Gryo;
	uint8_t Shoot_Flag;
	uint16_t Start_Time;
	uint16_t End_Time;
	uint16_t Bullet_RM;
}SHEN_WEI_struct;


typedef struct{
	uint8_t Status; 
	uint8_t Gimbal_Gryo;
}YUE_LU_struct;

extern Motor3508_type Chassis_Motor[2];

void Chassis_Ctrl_Init(void);
void Chassis_Speed_Set(void);
void Chassis_PID_Ctrl(Motor3508_type *chassis);
void Chassis_CAN_Send(int16_t Output1,int16_t Output2);
void ChassisPowerControl(void);
#endif
