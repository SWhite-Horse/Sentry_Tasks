#include "Task_Init.h"
#include "Task_Gimbal.h"

uint16_t clk=0;
uint16_t detla = 0;
extern MotorType_6020 Yaw;
void Task_LEDBlink(void *parameters)
{
  TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
	while(1)
	{
		detla = Yaw.FrameCounter - clk;
		clk = Yaw.FrameCounter;
		HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
		vTaskDelayUntil(&xLastWakeUpTime, 100);
	}
}
