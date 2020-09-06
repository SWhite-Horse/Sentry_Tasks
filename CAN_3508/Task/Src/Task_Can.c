#define __TASK_CAN_GLOBALS
#include "Task_CAN.h"
#include "Task_Shoot.h"

void Task_CAN(void *parameters){
	while(1)
    {
			CanSend_Type CAN_Tx_Msg;
			xQueueReceive(Queue_CANSend, &CAN_Tx_Msg, portMAX_DELAY);
			CANTransmit(&hcan1,CAN_Tx_Msg.stdid,CAN_Tx_Msg.Data);  //局部发送函数
    }
}

void CANTransmit(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[])
{
    CAN_TxHeaderTypeDef TxHeader; /*CAN发送结构体*/
    TxHeader.StdId = std_id;			/*设置ID*/
    TxHeader.IDE = CAN_ID_STD;		/*设置标准格式*/
    TxHeader.RTR = CAN_RTR_DATA;	/*选择数据帧*/
    TxHeader.DLC = 8;							/*数据长度8位*/
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, aData, (uint32_t*)CAN_TX_MAILBOX0); /*CAN发送函数*/
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t aData[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, aData);
		 switch(RxHeader.StdId)
    {
				case 0x201:	
				Motor_3508[0].FrameCounter++;
			  Motor_3508[0].Mechanical_Angle = aData[0] << 8 | aData[1];
		    Motor_3508[0].RealSpeed = aData[2] << 8 | aData[3];
				Motor_3508[0].RealCurrent = aData[4] << 8 | aData[5];
				break;
		};
	
}
