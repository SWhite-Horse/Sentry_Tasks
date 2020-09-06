#define __TASK_INIT_GLOBALS

#include "Task_Init.h"
#include "Task_CAN.h"
#include "Task_Shoot.h"


void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Recieve(CAN_HandleTypeDef *hcan);


void Task_Init(void *parameters)
{
			CAN_Init(&hcan1);
			CAN_Recieve(&hcan1);
	
	    taskENTER_CRITICAL();          //进入临界区
			Queue_CANSend = xQueueCreate(30,sizeof(CanSend_Type));   //创建发送队列
	
		  xTaskCreate(Task_Shoot, "Task_Shoot", 300, NULL, 5, &TaskHandle_Shoot);
	    xTaskCreate(Task_CAN, "Task_CAN", 128, NULL, 6, &TaskHandle_CAN);
	    xTaskCreate(Task_LEDBlink, "Task_LEDBlink", 64, NULL, 3, &TaskHandle_LEDBlink);

	
		HAL_Delay(1000);             //延时五秒让任务完成
    vTaskDelete(NULL);            //删除初始化的任务
    taskEXIT_CRITICAL();          //退出临界区

}


void CAN_Init(CAN_HandleTypeDef *hcan){
		CAN_FilterTypeDef sFilterConfig;
	
		sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    HAL_CAN_Start(hcan);
};


void CAN_Recieve(CAN_HandleTypeDef *hcan){
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
};

