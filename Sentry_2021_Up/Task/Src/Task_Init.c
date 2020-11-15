#define __TASK_INIT_GLOBALS

#include "Task_Init.h"
#include "Task_CAN.h"
#include "Task_StatusMachine.h"

#include "Task_Shoot.h"
#include "Task_RC.h"
#include "Task_Chassis.h"
#include "Task_Measure.h"


void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Recieve(CAN_HandleTypeDef *hcan);


void Task_Init(void *parameters)
{
		    taskENTER_CRITICAL();          //�����ٽ���

		//** Init **//
			CAN_Init(&hcan1);
			CAN_Init(&hcan2);
			CAN_Recieve(&hcan1);
			CAN_Recieve(&hcan2);
	
			Queue_CANSend = xQueueCreate(30,sizeof(CanSend_Type));   //�������Ͷ���
	
			L1_Measure_Init(&huart6);
			RC_Receive_Enable(&huart1); //ң�س�ʼ��
	
		//** �������� **//
			xTaskCreate(Task_Chassis, "Task_Chassis", 256, NULL, 5, &TaskHandle_Chassis);
		  xTaskCreate(Task_Shoot, "Task_Shoot", 300, NULL, 5, &TaskHandle_Shoot);
	    xTaskCreate(Task_RC, "Task_RC", 256, NULL, 5, &TaskHandle_RC);
	    xTaskCreate(Task_CAN, "Task_CAN", 128, NULL, 6, &TaskHandle_CAN);
	    xTaskCreate(Task_LEDBlink, "Task_LEDBlink", 64, NULL, 3, &TaskHandle_LEDBlink);
	    xTaskCreate(Task_Measure, "Task_Measure", 200, NULL, 4, &TaskHandle_Measure);
			xTaskCreate(Task_StatusMachine, "Task_StatusMachine", 128, NULL, 5, &TaskHandle_StatusMachine);
			xTaskCreate(Task_JetsonCome,"Task_JetsonComm",300,NULL,5,&TaskHandle_JetsonComm);
		  xTaskCreate(Task_IMU,"Task_IMU",256,NULL,5,&TaskHandle_IMU);
			xTaskCreate(Task_Gimbal,"Task_Gimbal",512,NULL,4,&TaskHandle_Gimbal);
			xTaskCreate(Task_JudgeReceive, "Task_JudgeReceive", 128, NULL, 5, &TaskHandle_JudgeReceive);

	
		HAL_Delay(1000);             //��ʱ�������������
    vTaskDelete(NULL);            //ɾ����ʼ��������
    taskEXIT_CRITICAL();          //�˳��ٽ���

}


/**
  * @brief  CAN��ʼ��
  * @param  hcan1 hcan2�ĵ�ַ
  * @retval void
  * @note   �˲�
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
		//�����˲���
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
  * @brief  ʹ��CAN�ж�
  * @param  hcan1 hcan2�ĵ�ַ
  * @retval void
  * @note   can���ж�ʹ��
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
