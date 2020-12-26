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
 * @description: CAN���͵ķ�װ����
 * @param {hcan1  stdid  aData} 
 * @return: void
 * @note: CAN���ͺ���
 */ 
void CANTransmit(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[])
{
    CAN_TxHeaderTypeDef TxHeader; /*CAN���ͽṹ��*/
    TxHeader.StdId = std_id;			/*����ID*/
    TxHeader.IDE = CAN_ID_STD;		/*���ñ�׼��ʽ*/
    TxHeader.RTR = CAN_RTR_DATA;	/*ѡ������֡*/
    TxHeader.DLC = 8;							/*���ݳ���8λ*/
		if(hcan == &hcan1)
			HAL_CAN_AddTxMessage(hcan, &TxHeader, aData, (uint32_t*)CAN_TX_MAILBOX0);
		else if(hcan == &hcan2)
			HAL_CAN_AddTxMessage(hcan, &TxHeader, aData, (uint32_t*)CAN_TX_MAILBOX1);
		else return;
}

/**
 * @description: CAN1 �Ļص�����
 * @param {hcan1  stdid  aData} 
 * @return: void
 * @note: ��������
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
				case 0x203://���У���Ϊ��Chassis��ͻ�������Բ���Chassis�ķ���
					break;
				case 0x204://���У���Ϊ��Chassis��ͻ
					break;
				case 0x205://����̨Yaw
					break;
				case 0x206://����̨Pitch
					break;
				case 0x207://����̨����
					break;
				case 0x208://���У���Ϊ����̨Gimbal��ͻ�������Բ���Gimbal����
					break;
				case 0x209:	
					//Yaw�����������ݣ�6020��
					Yaw.FrameCounter++;
					Yaw.Mechanical_Angle = aData[0] << 8 | aData[1];
					Yaw.Torque_Current_Real = aData[2] << 8 | aData[3];
					Yaw.MotorTemp = aData[6];		
					break;				
				case 0x66://��������̨��������
					RxMessage.speed=aData[0] << 8 | aData[1];       //�ٶ�
					RxMessage.controlmode=aData[2];                 //����ģʽ
					ControlMode = RxMessage.controlmode;            //�����յ��Ŀ���ģʽ��������̨����ģʽ�ṹ��
					RxMessage.fricstatus = aData[3]>>4;                //Ħ����״̬					
					RxMessage.stirstatus = aData[3]&0x0F;                //����״̬
					RxMessage.yaw_speed = aData[4] << 8 | aData[5]; //yaw���ٶ�
					RxMessage.pitch_speed = aData[6]<< 8 | aData[7]; //pitch���ٶ�
					break;
				default:
					break;
		};
	
}

/**
 * @description: CAN2 �Ļص�����
 * @param {hcan2  stdid  aData} 
 * @return: void
 * @note: ��������
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
			//���̵����������
					StirMotor.FrameCounter++;
					StirMotor.Mechanical_Angle = aData[0] << 8 | aData[1];
					StirMotor.RealSpeed = aData[2] << 8 | aData[3];
					break;
				case 0x20A: 
					//Pitch�����������ݣ�6020��
					Pitch.FrameCounter++;
					Pitch.Mechanical_Angle = aData[0] << 8 | aData[1];
					Pitch.Torque_Current_Real = aData[2] << 8 | aData[3];
					Pitch.MotorTemp = aData[6];					
					break;
				default:
					break;				
		};
	
}
