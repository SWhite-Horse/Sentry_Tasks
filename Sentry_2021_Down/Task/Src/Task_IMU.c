#include "Task_IMU.h"

/**
 * @description: 陀螺仪数据更新任务
 * @param {unused} 
 * @return:void
 * @note: 进行陀螺仪数据更新
 */
TickType_t CU, LAS;

void Task_IMU(void *parameters)
{
	TickType_t xPreviousWakeTime;
	xPreviousWakeTime = xTaskGetTickCount();
	while(1)
	{
		mpu_get_data();						/*获取数据*/
    imu_ahrs_update();				/*更新数据*/
   	imu_attitude_update();		/*更新数据*/
    imu_temp_ctrl();					/*温度闭环*/
		vTaskDelayUntil(&xPreviousWakeTime, 2);
		
	}
}

