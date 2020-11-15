#ifndef __TASKS_INIT_H__
#define __TASKS_INIT_H__

#include "sysconfig.h"

#ifdef 	__TASK_INIT_GLOBALS
#define TASK_INIT_EXT
#else
#define TASK_INIT_EXT extern
#endif


//** 任务句柄定义

TASK_INIT_EXT TaskHandle_t TaskHandle_Shoot;
TASK_INIT_EXT TaskHandle_t TaskHandle_CAN;
TASK_INIT_EXT TaskHandle_t TaskHandle_LEDBlink;
TASK_INIT_EXT TaskHandle_t TaskHandle_RC;
TASK_INIT_EXT TaskHandle_t TaskHandle_Measure;
TASK_INIT_EXT TaskHandle_t TaskHandle_IMU;
TASK_INIT_EXT TaskHandle_t TaskHandle_StatusMachine;
TASK_INIT_EXT TaskHandle_t TaskHandle_JetsonComm;
TASK_INIT_EXT TaskHandle_t TaskHandle_Gimbal;
TASK_INIT_EXT TaskHandle_t TaskHandle_Communication;


//** CAN发送队列

TASK_INIT_EXT QueueHandle_t Queue_CANSend; 
void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Recieve(CAN_HandleTypeDef *hcan);


//** Task函数定义
void Task_CAN(void *parameters);
void Task_LEDBlink(void *parameters);
void Task_Gimbal(void *parameters);
void Task_Communication(void *parameters);
void Task_RC(void *parameters);
void Task_Shoot(void *parameters);
void Task_IMU(void *parameters);
void Task_JetsonComm(void *parameters);
void Task_Measure(void *parameters);
void Task_StatusMachine(void *parameters);
void Task_JetsonCome(void *parameters);
#endif
