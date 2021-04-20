#ifndef __TASKS_FRICMOTOR_H
#define __TASKS_FRICMOTOR_H

#ifdef __TASK_FRICMOTOR_GLOBALS
#define TASK_FRICMOTOR_EXT
#else 
#define TASK_FRICMOTOR_EXT extern
#endif

#include "Task_Init.h"
#include "Task_CAN.h"


#define SPEEDMAX 7700
#define SPEEDMID 5500
#define SPEEDMIN 4000

//C620电调电流最大值
#define C620CURRENTMAX 											16000
#define ONECIRCLECOUNTVALUEAFTERDECELERATE  786432       //96*8192
#define ONECIRCLECOUNTVALUEORIGINAL         (uint16_t)8192 

#define CROSSZEROUPTHRESHOLD                6000
#define CROSSZERODOWNTHRESHOLD              2000

#define C610CURRENTMAX                      10000

#define Shoot_Angle_Short 12
#define Shoot_Angle_Long 18

//激光开关宏
#define LASER_ON() HAL_GPIO_WritePin(LASER_GPIO_Port,LASER_Pin,GPIO_PIN_SET)
#define LASER_OFF() HAL_GPIO_WritePin(LASER_GPIO_Port,LASER_Pin,GPIO_PIN_RESET)

typedef struct{
	uint16_t Heat_Limitation_UP;
	uint16_t Heat_Limitation_DOWN;
	
}Heat_Limitation_t;

typedef enum{
	Three_Mode,
	Short_Mode,
	Long_Mode,
}Shoot_Method_enum;

extern RM2006_Type StirMotor;
extern Motor3508_type Fric_3508_Motor[2];

void Motor_3508_Send(int16_t Output0,int16_t Output1);
void Motor_3508_PID_Init(void);
void Motor_3508_PID_Calculate(Motor3508_type *motor);
void Fric_3508_Motor_Speed_Set(void);

void Shoot_Status_with_Heat(uint16_t angle);

void StirMotor_Blocked_Detect(RM2006_Type* motor);
void Stir_Motor_Speed_Control(RM2006_Type* motor);
void StirMotor_Init(void);
void StirMotor_Control(void);
void Stir_CAN_Send(int16_t Output);

#endif
