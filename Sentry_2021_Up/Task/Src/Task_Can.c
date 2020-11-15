#define __TASK_CAN_GLOBALS
#include "Task_CAN.h"
#include "Task_Shoot.h"	
#include "Task_Chassis.h"
#include "sysconfig.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Shoot.h"
#include "Task_Communication.h"

void Task_CAN(void *parameters){
	while(1)
    {
			CanSend_Type CAN_Tx_Msg;
			xQueueReceive(Queue_CANSend, &CAN_Tx_Msg, portMAX_DELAY);
			CANTransmit(&hcan1,CAN_Tx_Msg.stdid,CAN_Tx_Msg.Data);  //�ֲ����ͺ���
    }
}

void CANTransmit(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[])
{
    CAN_TxHeaderTypeDef TxHeader; /*CAN���ͽṹ��*/
    TxHeader.StdId = std_id;			/*����ID*/
    TxHeader.IDE = CAN_ID_STD;		/*���ñ�׼��ʽ*/
    TxHeader.RTR = CAN_RTR_DATA;	/*ѡ������֡*/
    TxHeader.DLC = 8;							/*���ݳ���8λ*/
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, aData, (uint32_t*)CAN_TX_MAILBOX0); /*CAN���ͺ���*/
}
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
				case 0x203:
			//���̵����������
					StirMotor.FrameCounter++;
					StirMotor.Mechanical_Angle = aData[0] << 8 | aData[1];
					StirMotor.RealSpeed = aData[2] << 8 | aData[3];
					break;
				case 0x204:
					break;
				case 0x205:
					break;
				case 0x206:
					break;
				case 0x207:	
					//Yaw�����������ݣ�6020��
					Yaw.FrameCounter++;
					Yaw.Mechanical_Angle = aData[0] << 8 | aData[1];
					Yaw.Torque_Current_Real = aData[2] << 8 | aData[3];
					Yaw.MotorTemp = aData[6];		
					break;				
				case 0x208: 
					//Pitch�����������ݣ�6020��
					Pitch.FrameCounter++;
					Pitch.Mechanical_Angle = aData[0] << 8 | aData[1];
					Pitch.Torque_Current_Real = aData[2] << 8 | aData[3];
					Pitch.MotorTemp = aData[6];					
					break;
//				case 0x66://��������̨��������
//					Rxmessage.speed=aData[0] << 8 | aData[1];       //�ٶ�
//					Rxmessage.controlmode=aData[2];                 //����ģʽ
//					ControlMode = Rxmessage.controlmode;            //�����յ��Ŀ���ģʽ��������̨����ģʽ�ṹ��
//					Rxmessage.fricstatus = aData[3];                //Ħ����״̬					
//					Rxmessage.stirstatus = aData[4];                //����״̬
//					Rxmessage.yaw_speed = aData[5] << 8 | aData[6]; //yaw���ٶ�
//					break;
//				case 0x68:
//					Rxmessage.pitch_speed = aData[0]<< 8 | aData[1]; //pitch���ٶ�
//					break;
		};
	
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t aData[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, aData);
		 switch(RxHeader.StdId)
    {
				case 0x201:	
					Fric_Motor_3508[0].FrameCounter++;
					Fric_Motor_3508[0].Mechanical_Angle = aData[0] << 8 | aData[1];
					Fric_Motor_3508[0].RealSpeed = aData[2] << 8 | aData[3];
					Fric_Motor_3508[0].RealCurrent = aData[4] << 8 | aData[5];
					break;
				case 0x202:	
					Fric_Motor_3508[1].FrameCounter++;
					Fric_Motor_3508[1].Mechanical_Angle = aData[0] << 8 | aData[1];
					Fric_Motor_3508[1].RealSpeed = aData[2] << 8 | aData[3];
					Fric_Motor_3508[1].RealCurrent = aData[4] << 8 | aData[5];
					break;
				case 0x207:
					break;
		};
	
}
