#include "Task_IMU.h"

/**
 * @description: ���������ݸ�������
 * @param {unused} 
 * @return:void
 * @note: �������������ݸ���
 */
TickType_t CU, LAS;

void Task_IMU(void *parameters)
{
	TickType_t xPreviousWakeTime;
	xPreviousWakeTime = xTaskGetTickCount();
	while(1)
	{
		mpu_get_data();						/*��ȡ����*/
    imu_ahrs_update();				/*��������*/
   	imu_attitude_update();		/*��������*/
    imu_temp_ctrl();					/*�¶ȱջ�*/
		vTaskDelayUntil(&xPreviousWakeTime, 2);
		
	}
}

