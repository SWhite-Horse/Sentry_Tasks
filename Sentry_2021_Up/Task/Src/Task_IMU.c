#include "Task_IMU.h"

/**
 * @description: ���������ݸ�������
 * @param {unused} 
 * @return:void
 * @note: �������������ݸ���
 */
void Task_IMU(void *parameters)
{
	TickType_t xPreviousWakeTime;
	xPreviousWakeTime = xTaskGetTickCount();
	while(1)
	{
		#ifdef ONBOARDIMU
		mpu_get_data();						/*��ȡ����*/
    imu_ahrs_update();				/*��������*/
   	imu_attitude_update();		/*��������*/
    imu_temp_ctrl();					/*�¶ȱջ�*/
		vTaskDelayUntil(&xPreviousWakeTime, 2);
		#endif
		
	}
}



#ifdef BlackBlock
/**
  * @brief  �ٷ������ǳ�ʼ�� --CAN2
  * @param  void
  * @retval void
  * @note   ��λʱ����ȷ�������Ǿ�ֹ����
  * @note   δʹ�÷��Ͷ�������Ϊ�˺��������ڴ�������֮ǰ
  */
void Official_GYRO_Init(void)
{
	CanTxMsgTypeDef TX_Message;

	TX_Message.StdId = OFFICIALZGYRORESETCANID;
	TX_Message.IDE = CAN_ID_STD;
	TX_Message.RTR = CAN_RTR_DATA;
	TX_Message.DLC = 0x08;

	TX_Message.Data[0] = 0x00;
	TX_Message.Data[1] = 0x01;
	TX_Message.Data[2] = 0x02;
	TX_Message.Data[3] = 0x03;
	TX_Message.Data[4] = 0x04;
	TX_Message.Data[5] = 0x05;
	TX_Message.Data[6] = 0x06;
	TX_Message.Data[7] = 0x07;

	hcan2.pTxMsg = &TX_Message;
	HAL_CAN_Transmit_IT(&hcan2);
}



#endif

