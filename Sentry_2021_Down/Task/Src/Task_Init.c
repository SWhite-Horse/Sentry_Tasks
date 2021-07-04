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
		  taskENTER_CRITICAL();          //�����ٽ���
		//** Init **//
		 // HeartbeatCycleALL = xTaskGetTickCount();
	
			CAN_Init(&hcan1);
			CAN_Init(&hcan2);
			CAN_Recieve(&hcan1);
			CAN_Recieve(&hcan2);
	

			Queue_CANSend = xQueueCreate(30,sizeof(CanSend_Type));   //�������Ͷ���
	
			LASER_ON();
			//L1_Measure_Init(&huart6);
			RC_Receive_Enable(&huart1); //ң�س�ʼ��
			JetsonCommUart_Config(&huart8); //��jestonͨѶ���ڳ�ʼ��

	    //** ���������ǳ�ʼ��
	    mpu_device_init();
		  init_quaternion();	
		  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); //�����Ǽ��ȵ���
		  __HAL_TIM_SET_COMPARE(&htim3,IMU_HEATING_Pin,HEAT_MID);
	
		//** �������� **//
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
	
		vTaskDelay(1000);             //��ʱ�������������
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




////��ʱ��
//TickType_t HeartbeatCycleALL=0;//���������ڼ�����
//	
//TickType_t HeartbeatCycleDifference(TickType_t * HeartbeatCycleAbsolute)//��˴�������ϴ�������������ڲ�,����Ϊ�ϴ��������ڵľ�����
//{
//  TickType_t HeartbeatCycleReturn;//����ֵ
//	HeartbeatCycleALL = xTaskGetTickCount();
//	if(HeartbeatCycleALL>*HeartbeatCycleAbsolute)//�������������
//			HeartbeatCycleReturn= HeartbeatCycleALL-*HeartbeatCycleAbsolute;
//	else 
//			HeartbeatCycleReturn= 0xFFFFFFFF+HeartbeatCycleALL-*HeartbeatCycleAbsolute;//��� 0xFFFFFFFFΪTickType_t�����ֵ;
//	* HeartbeatCycleAbsolute=HeartbeatCycleALL;//���¾�����
//	return HeartbeatCycleReturn;
//}
//	
//void HeartbeatCycleAdd(TickType_t * HeartbeatCycleCount,TickType_t * HeartbeatCycleAbsolute)//�������ڼ�����(�������ڼ��������㣬�ϴ����еľ�����)
//{
//		*(HeartbeatCycleCount)+=HeartbeatCycleDifference(HeartbeatCycleAbsolute);
//}
//	
//uint16_t HeartbeatCycleToTime(TickType_t * HeartbeatCycle)//��������ת��Ϊ����
//{
//		return(uint16_t)(*HeartbeatCycle)*portTICK_RATE_MS;
//}

