#define __TASK_CAN_GLOBALS
#include "Task_CAN.h"
#include "Task_Gimbal.h"
#include "Task_Shoot.h"
#include "Task_Communication.h"


void Task_CAN(void *parameters){
	while(1)
    {
			xQueueReceive(Queue_CANSend, &CAN_Tx_Msg, portMAX_DELAY);         
			switch (CAN_Tx_Msg.CANx)
			{
			case CANSEND_1:
				CANTransmit(&hcan1,CAN_Tx_Msg.stdid,CAN_Tx_Msg.Data);
				break;
			case CANSEND_2:
				CANTransmit(&hcan2,CAN_Tx_Msg.stdid,CAN_Tx_Msg.Data);
				break;
			default:
				break;
			}
    }
}

/**
 * @description: CAN发送的封装函数
 * @param {hcan1  stdid  aData} 
 * @return: void
 * @note: CAN发送函数
 */ 

void CANTransmit(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[])
{
    CAN_TxHeaderTypeDef TxHeader; /*CAN发送结构体*/
    TxHeader.StdId = std_id;			/*设置ID*/
    TxHeader.IDE = CAN_ID_STD;		/*设置标准格式*/
    TxHeader.RTR = CAN_RTR_DATA;	/*选择数据帧*/
    TxHeader.DLC = 8;							/*数据长度8位*/
		if(hcan == &hcan1)
			HAL_CAN_AddTxMessage(hcan, &TxHeader, aData, (uint32_t*)CAN_TX_MAILBOX0);
		else if(hcan == &hcan2)
			HAL_CAN_AddTxMessage(hcan, &TxHeader, aData, (uint32_t*)CAN_TX_MAILBOX1);
		else return;
}




/**
 * @description: CAN1 的回调函数
 * @param {hcan1  stdid  aData} 
 * @return: void
 * @note: 接受数据
 */ 

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CAN_RxHeaderTypeDef RxHeader;
   uint8_t aData[8];
   HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, aData);
	 switch(RxHeader.StdId){
			case 0x201:					
				break;
			case 0x202:	
				break;
			case 0x203:	
				break;	
			case 0x204:				
				break;				
		  //** 0x1ff
			case 0x205:	//Yaw轴电机返回数据（6020）
				Yaw.FrameCounter++;
				Yaw.Mechanical_Angle = aData[0] << 8 | aData[1];
				Yaw.Real_Speed = aData[2] << 8 | aData[3];
				Yaw.MotorTemp = aData[6];		
				break;				
			case 0x207:	//拨盘点击返回数据
				StirMotor.FrameCounter++;
				StirMotor.Mechanical_Angle = aData[0] << 8 | aData[1];
				StirMotor.RealSpeed = aData[2] << 8 | aData[3];		
				break;
			case 0x208:	
				break;
			case 0x69:
				RxMessage.Is_gaming=aData[0];
				RxMessage.Armour=aData[2];
				RxMessage.Heat=aData[3] << 8 | aData[4];
				RxMessage.Shoot_Speed_limit=aData[5];
				RxMessage.mains_power_shooter=aData[6];
				RxMessage.get_hurt=aData[7];
				break;	
	};
}

/**
 * @description: CAN2 的回调函数
 * @param {hcan2  stdid  aData} 
 * @return: void
 * @note: 接受数据
 */ 

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
  uint8_t aData[8];	/*接收数据缓存数组*/
  HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, aData); /*从邮箱中取回数据*/
  switch(RxHeader.StdId)	/*根据邮箱地址选择数据*/
  {
		case 0x201:
			Fric_3508_Motor[0].FrameCounter++;
			Fric_3508_Motor[0].Mechanical_Angle = aData[0] << 8 | aData[1];
			Fric_3508_Motor[0].RealSpeed = aData[2] << 8 | aData[3];
			Fric_3508_Motor[0].RealCurrent = aData[4] << 8 | aData[5];
			break;
		case 0x202:
			Fric_3508_Motor[1].FrameCounter++;
			Fric_3508_Motor[1].Mechanical_Angle = aData[0] << 8 | aData[1];
			Fric_3508_Motor[1].RealSpeed = aData[2] << 8 | aData[3];
			Fric_3508_Motor[1].RealCurrent = aData[4] << 8 | aData[5];
			break;
	  case 0x203:
			break;
		case 0x204:
			break;
    //上云台摩擦轮PID
		case 0x205:
			break;
		case 0x206: //Pitch轴电机返回数据（6020）
			Pitch.FrameCounter++;
			Pitch.Mechanical_Angle = aData[0] << 8 | aData[1];
			Pitch.Real_Speed = aData[2] << 8 | aData[3];
			Pitch.MotorTemp = aData[6];					
			break;
		case 0x207:	
			break;
		case 0x208:
			break;	
  }
}

