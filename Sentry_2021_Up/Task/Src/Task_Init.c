#define __TASK_INIT_GLOBALS

#include "Task_Init.h"
#include "Task_IMU.h"
#include "Task_CAN.h"
#include "Task_StatusMachine.h"
#include "Task_JetsonComm.h"
#include "Task_Shoot.h"
#include "Task_RC.h"
#include "Task_Chassis.h"
#include "Task_Measure.h"
#include "Task_JudgeReceive.h"

void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Recieve(CAN_HandleTypeDef *hcan);


void Task_Init(void *parameters)
{
		   taskENTER_CRITICAL();          //Ω¯»Î¡ŸΩÁ«¯
		//** Init **//
			CAN_Init(&hcan1);
			CAN_Init(&hcan2);
			CAN_Recieve(&hcan1);
			CAN_Recieve(&hcan2);
	
			Queue_CANSend = xQueueCreate(30,sizeof(CanSend_Type));   //¥x¥Ω®∑¢ÀÕ∂”¡–
				
			LASER_ON();
			JudgeConnection_Init(&huart7);//≤√≈–œµÕ≥¡¨Ω”≥ı ºªØ
//			L1_Measure_Init(&huart8);
			//RC_Receive_Enable(&huart1); //“£øÿ≥ı ºªØ
			JetsonCommUart_Config(&huart8); //”ÎjestonÕ®—∂¥Æø⁄≥ı ºªØ
	
		   //** ∞Â‘ÿÕ”¬›“«≥ı ºªØ
	    mpu_device_init();
		  init_quaternion();	
		  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); //Õ”¬›“«º”»»µÁ◊Ë
		  __HAL_TIM_SET_COMPARE(&htim3,IMU_HEATING_Pin,HEAT_MID);
	
		//** ¥¥Ω®»ŒŒÒ **//
			xTaskCreate(Task_Chassis, "Task_Chassis", 256, NULL, 4, &TaskHandle_Chassis);
		  xTaskCreate(Task_Shoot, "Task_Shoot", 300, NULL, 5, &TaskHandle_Shoot);
	    //xTaskCreate(Task_RC, "Task_RC", 20, NULL, 5, &TaskHandle_RC);
	    xTaskCreate(Task_CAN, "Task_CAN", 200, NULL, 6, &TaskHandle_CAN);
	    xTaskCreate(Task_LEDBlink, "Task_LEDBlink", 64, NULL, 3, &TaskHandle_LEDBlink);
	    //xTaskCreate(Task_Measure, "Task_Measure", 200, NULL, 4, &TaskHandle_Measure);
			xTaskCreate(Task_Detect, "Task_Detect", 200, NULL, 4, &TaskHandle_Detect);
			xTaskCreate(Task_StatusMachine, "Task_StatusMachine", 128, NULL, 4, &TaskHandle_StatusMachine);
			xTaskCreate(Task_JetsonComm,"Task_JetsonComm",512,NULL,5,&TaskHandle_JetsonComm);
		  xTaskCreate(Task_IMU,"Task_IMU",256,NULL,5,&TaskHandle_IMU);
			xTaskCreate(Task_Gimbal,"Task_Gimbal",512,NULL,5,&TaskHandle_Gimbal);
			xTaskCreate(Task_JudgeReceive, "Task_JudgeReceive", 128, NULL, 5, &TaskHandle_JudgeReceive);
      xTaskCreate(Task_Communication, "Task_Communication", 200, NULL, 4, &TaskHandle_Communication);

	
		vTaskDelay(1000);             //—” ±ŒÂ√Î»√»ŒŒÒÕÍ≥…
    vTaskDelete(NULL);            //…æ≥˝≥ı ºªØµƒ»ŒŒÒ
    taskEXIT_CRITICAL();          //ÕÀ≥ˆ¡ŸΩÁ«¯

}


/**
  * @brief  CAN≥ı ºªØ
  * @param  hcan1 hcan2µƒµÿ÷∑
  * @retval void
  * @note   ¬À≤®
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
		//≈‰÷√¬À≤®∆˜
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
  * @brief   πƒ‹CAN÷–∂œ
  * @param  hcan1 hcan2µƒµÿ÷∑
  * @retval void
  * @note   canµƒ÷–∂œ πƒ‹
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

