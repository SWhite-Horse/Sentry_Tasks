#ifndef __TASKS_INIT_H__
#define __TASKS_INIT_H__

#include "stm32f4xx_hal.h"
#include "can.h"
#include "cmsis_os.h"

#ifdef 	__TASK_INIT_GLOBALS
#define TASK_INIT_EXT
#else
#define TASK_INIT_EXT extern
#endif

TASK_INIT_EXT TaskHandle_t TaskHandle_Shoot;
TASK_INIT_EXT TaskHandle_t TaskHandle_CAN;
TASK_INIT_EXT TaskHandle_t TaskHandle_LEDBlink;


//CAN∑¢ÀÕ∂”¡–
TASK_INIT_EXT QueueHandle_t Queue_CANSend; 

void Task_CAN(void *parameters);
void Task_LEDBlink(void *parameters);

#endif
