#include "Task_init.h"

void Task_LEDBlink(void *parameters)
{
  TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
		vTaskDelayUntil(&xLastWakeUpTime, 100);
	}
}
