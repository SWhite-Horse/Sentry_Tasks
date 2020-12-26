#define __TASK_CAN_GLOBALS
#include "Task_CAN.h"
#include "Task_Shoot.h"	
#include "Task_Chassis.h"
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
		 switch(RxHeader.StdId)
    {
				case 0x201:	
					Chassis_Motor[0].FrameCounter++;
					Chassis_Motor[0].Mechanical_Angle = aData[0] << 8 | aData[1];
					Chassis_Motor[0].RealSpeed = aData[2] << 8 | aData[3];
					Chassis_Motor[0].RealCurrent = aData[4] << 8 | aData[5];
					break;
				case 0x202:	
					Chassis_Motor[1].FrameCounter++;
					Chassis_Motor[1].Mechanical_Angle = aData[0] << 8 | aData[1];
					Chassis_Motor[1].RealSpeed = aData[2] << 8 | aData[3];
					Chassis_Motor[1].RealCurrent = aData[4] << 8 | aData[5];
					break;
				case 0x203://空闲，因为和Chassis冲突，但可以并入Chassis的发送
					break;
				case 0x204://空闲，因为和Chassis冲突
					break;
				case 0x205://下云台Yaw
					break;
				case 0x206://下云台Pitch
					break;
				case 0x207://下云台拨盘
					break;
				case 0x208://空闲，因为下云台Gimbal冲突，但可以并入Gimbal发送
					break;
				case 0x209:	
					//Yaw轴电机返回数据（6020）
					Yaw.FrameCounter++;
					Yaw.Mechanical_Angle = aData[0] << 8 | aData[1];
					Yaw.Torque_Current_Real = aData[2] << 8 | aData[3];
					Yaw.MotorTemp = aData[6];		
					break;				
				case 0x66://接收下云台发送数据
					RxMessage.speed=aData[0] << 8 | aData[1];       //速度
					RxMessage.controlmode=aData[2];                 //控制模式
					ControlMode = RxMessage.controlmode;            //将接收到的控制模式赋给上云台控制模式结构体
					RxMessage.fricstatus = aData[3]>>4;                //摩擦轮状态					
					RxMessage.stirstatus = aData[3]&0x0F;                //拨盘状态
					RxMessage.yaw_speed = aData[4] << 8 | aData[5]; //yaw轴速度
					RxMessage.pitch_speed = aData[6]<< 8 | aData[7]; //pitch轴速度
					break;
				default:
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
    uint8_t aData[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, aData);
		 switch(RxHeader.StdId)
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
			//拨盘点击返回数据
					StirMotor.FrameCounter++;
					StirMotor.Mechanical_Angle = aData[0] << 8 | aData[1];
					StirMotor.RealSpeed = aData[2] << 8 | aData[3];
					break;
				case 0x20A: 
					//Pitch轴电机返回数据（6020）
					Pitch.FrameCounter++;
					Pitch.Mechanical_Angle = aData[0] << 8 | aData[1];
					Pitch.Torque_Current_Real = aData[2] << 8 | aData[3];
					Pitch.MotorTemp = aData[6];					
					break;
				default:
					break;				
		};
	
}
