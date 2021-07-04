#define __TASK_INIT_GLOBALS

#include "Task_Init.h"
#include "Task_CAN.h"
#include "Task_StatusMachine.h"
#include "Task_Shoot.h"
#include "Task_RC.h"
#include "Task_IMU.h"
#include "Task_Measure.h"
#include "Task_JetsonComm.h"

void Task_Init(void *parameters)
{
		  taskENTER_CRITICAL();          //进入临界区
		//** Init **//
		 // HeartbeatCycleALL = xTaskGetTickCount();
	
			CAN_Init(&hcan1);
			CAN_Init(&hcan2);
			CAN_Recieve(&hcan1);
			CAN_Recieve(&hcan2);
	

			Queue_CANSend = xQueueCreate(30,sizeof(CanSend_Type));   //创建发送队列
	
			LASER_ON();
			//L1_Measure_Init(&huart6);
			RC_Receive_Enable(&huart1); //遥控初始化
			JetsonCommUart_Config(&huart8); //与jeston通讯串口初始化

	    //** 板载陀螺仪初始化
	    mpu_device_init();
		  init_quaternion();	
		  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); //陀螺仪加热电阻
		  __HAL_TIM_SET_COMPARE(&htim3,IMU_HEATING_Pin,HEAT_MID);
	
		//** 创建任务 **//
			xTaskCreate(Task_LEDBlink, "Task_LEDBlink", 100, NULL, 3, &TaskHandle_LEDBlink);
			xTaskCreate(Task_CAN, "Task_CAN", 128, NULL, 6, &TaskHandle_CAN);
		  xTaskCreate(Task_RC, "Task_RC", 256, NULL, 4, &TaskHandle_RC);
		  xTaskCreate(Task_Shoot, "Task_Shoot", 256, NULL, 4, &TaskHandle_Shoot);

//	    xTaskCreate(Task_Measure, "Task_Measure", 200, NULL, 4, &TaskHandle_Measure);
		  xTaskCreate(Task_IMU,"Task_IMU",256,NULL,5,&TaskHandle_IMU);
			xTaskCreate(Task_Gimbal,"Task_Gimbal",512,NULL,4,&TaskHandle_Gimbal);
			xTaskCreate(Task_StatusMachine, "Task_StatusMachine", 128, NULL, 5, &TaskHandle_StatusMachine);

		  xTaskCreate(Task_Communication,"Task_Communication",200,NULL,4,&TaskHandle_Communication);//
			xTaskCreate(Task_JetsonComm,"Task_JetsonComm",512,NULL,4,&TaskHandle_JetsonComm);
	
		vTaskDelay(1000);             //延时五秒让任务完成
    vTaskDelete(NULL);            //删除初始化的任务
    taskEXIT_CRITICAL();          //退出临界区
}


/**
  * @brief  CAN初始化
  * @param  hcan1 hcan2的地址
  * @retval void
  * @note   滤波
  */

void CAN_Init(CAN_HandleTypeDef *hcan)
{
    uint32_t FilterBank, FilterFIFO;
    CAN_FilterTypeDef sFilterConfig;
    if(hcan == &hcan1)
    {
        FilterBank = 0;
        FilterFIFO = CAN_RX_FIFO0;
    }
    else if(hcan == &hcan2)
    {
        FilterBank = 14;
        FilterFIFO = CAN_RX_FIFO1;
    }
    else
    {
        return;
    }
		//配置滤波器
    sFilterConfig.FilterBank = FilterBank;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = FilterFIFO;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    HAL_CAN_Start(hcan);
}
;


/**
  * @brief  使能CAN中断
  * @param  hcan1 hcan2的地址
  * @retval void
  * @note   can的中断使能
  */
void CAN_Recieve(CAN_HandleTypeDef *hcan)
{
	uint32_t ActiveITs;
	
  if(hcan == &hcan1)
  {
     ActiveITs = CAN_IT_RX_FIFO0_MSG_PENDING;
  }
  else if(hcan == &hcan2)
  {
     ActiveITs = CAN_IT_RX_FIFO1_MSG_PENDING;
  }
  else
  {
     return;
  }
  HAL_CAN_ActivateNotification(hcan, ActiveITs);
}




////计时器
//TickType_t HeartbeatCycleALL=0;//总心跳周期计数器
//	
//TickType_t HeartbeatCycleDifference(TickType_t * HeartbeatCycleAbsolute)//求此次任务和上次任务的心跳周期差,参数为上次心跳周期的绝对量
//{
//  TickType_t HeartbeatCycleReturn;//返回值
//	HeartbeatCycleALL = xTaskGetTickCount();
//	if(HeartbeatCycleALL>*HeartbeatCycleAbsolute)//正常工作无溢出
//			HeartbeatCycleReturn= HeartbeatCycleALL-*HeartbeatCycleAbsolute;
//	else 
//			HeartbeatCycleReturn= 0xFFFFFFFF+HeartbeatCycleALL-*HeartbeatCycleAbsolute;//溢出 0xFFFFFFFF为TickType_t的最大值;
//	* HeartbeatCycleAbsolute=HeartbeatCycleALL;//更新绝对量
//	return HeartbeatCycleReturn;
//}
//	
//void HeartbeatCycleAdd(TickType_t * HeartbeatCycleCount,TickType_t * HeartbeatCycleAbsolute)//心跳周期计数器(心跳周期计数可清零，上次运行的绝对量)
//{
//		*(HeartbeatCycleCount)+=HeartbeatCycleDifference(HeartbeatCycleAbsolute);
//}
//	
//uint16_t HeartbeatCycleToTime(TickType_t * HeartbeatCycle)//心跳周期转换为毫秒
//{
//		return(uint16_t)(*HeartbeatCycle)*portTICK_RATE_MS;
//}

