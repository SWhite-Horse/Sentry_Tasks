#include "Task_IMU.h"

/**
 * @description: 陀螺仪数据更新任务
 * @param {unused} 
 * @return:void
 * @note: 进行陀螺仪数据更新
 */
void Task_IMU(void *parameters)
{
	TickType_t xPreviousWakeTime;
	xPreviousWakeTime = xTaskGetTickCount();
	while(1)
	{
		#ifdef ONBOARDIMU
		mpu_get_data();						/*获取数据*/
    imu_ahrs_update();				/*更新数据*/
   	imu_attitude_update();		/*更新数据*/
    imu_temp_ctrl();					/*温度闭环*/
		vTaskDelayUntil(&xPreviousWakeTime, 2);
		#endif
		
	}
}



#ifdef BlackBlock
/**
  * @brief  官方陀螺仪初始化 --CAN2
  * @param  void
  * @retval void
  * @note   复位时必须确保陀螺仪静止！！
  * @note   未使用发送队列是因为此函数调用于创建任务之前
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

