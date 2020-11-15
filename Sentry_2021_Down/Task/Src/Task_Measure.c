#include "Task_Measure.h"
#include <stdlib.h>

float Distance=0.01;
L1_MEASURE_STRUCT L1_Measure_Struct;
L1_MEASURE_STRUCT L1_Measure_Struct_Temp;//** DMA读入缓存区


/*********************************
  * @brief  激光测距
  * @param  unused
  * @retval void
  * @note   L1激光测距返回距离至结构体
  */

void Task_Measure(void *parameters)
{
	while(1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);		
    L1_Measure_Update();		
	}
}

/*********************************
  * @brief  L1初始化，打开？？？？
  * @param  
  * @retval UART_HandleTypeDef
  * @note   
  */

void L1_Measure_Init(UART_HandleTypeDef *huart)
{
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  HAL_DMA_Start_IT(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)&L1_Measure_Struct, sizeof(L1_Measure_Struct));
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}


/*********************************
  * @brief  L1 数据更新
  * @param  void
  * @retval 
  * @note   
  */

void L1_Measure_Update()
{
	 memcpy(&L1_Measure_Struct_Temp, &L1_Measure_Struct, sizeof(L1_Measure_Struct));
	 Distance=(L1_Measure_Struct_Temp.n1-48)+0.1*(L1_Measure_Struct_Temp.n2-48)+0.01*(L1_Measure_Struct_Temp.n3-48)+0.001*(L1_Measure_Struct_Temp.n4-48);
}


/*********************************
  * @brief L1回调，DMA处理
  * @param  
  * @retval UART_HandleTypeDef
  * @note   Usart2
  */
int L1_Measure_counter;

void L1_Measure_Callback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskToWaken = pdFALSE;
	uint8_t counter;
	if(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		(void)huart->Instance->SR;
    (void)huart->Instance->DR;
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		
		__HAL_DMA_DISABLE(huart->hdmarx);
		counter = sizeof(L1_Measure_Struct) - __HAL_DMA_GET_COUNTER(huart->hdmarx);

		if(counter >= 0) //*** 有问题
		{
			if(L1_Measure_Struct.SOF == DISTANCE){
			if(huart == &huart6)
			{
				L1_Measure_counter = counter;
        vTaskNotifyGiveFromISR(TaskHandle_Measure,&xHigherPriorityTaskToWaken);
        portYIELD_FROM_ISR(xHigherPriorityTaskToWaken);
			}
		}
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, sizeof(L1_Measure_Struct));
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}
