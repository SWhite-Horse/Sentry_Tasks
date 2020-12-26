#include "Task_Init.h"

/**************************
  * @brief  LED闪烁任务
  * @param  unused
  * @retval void
  * @note   最低优先级任务，闪烁led判断是否阻死
  */
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
