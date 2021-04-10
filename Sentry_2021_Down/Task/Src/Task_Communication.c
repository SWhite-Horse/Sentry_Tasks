#include "Task_Init.h"
#include "Task_Communication.h"
#include "Task_StatusMachine.h"
#include "Task_RC.h"
#include "Task_CAN.h"
#include "Task_JetsonComm.h"
#include "stdlib.h"
#include "time.h"


Down_to_Up_Message TxMessage;
Up_to_Down_Message RxMessage;
uint16_t time_cnt=0;


	/**
  * @brief 底盘与云台通讯任务
  * @param  None
  * @retval void
	*	@note  	
  */
void Task_Communication(void *parameters)
{
	TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
	while(1)
	{ 
		if(ControlMode==ControlMode_Aimbot)
		{		
			if(DataRecFromJetson.SentryGimbalMode==ServoMode){
				if(RxMessage.get_hurt==3){
					TxMessage.Chassis_speed = 2400;	
				}
				else 
					TxMessage.Chassis_speed=1500;	
			}
			else
				TxMessage.Chassis_speed = 3000;		
						
		}	
		else if(ControlMode==ControlMode_Telecontrol_UP)
		{
				TxMessage.Chassis_speed = 8000 * Get_Channel_Val(&RC_ReceiveData,RC_CH0) / RC_CH_MAX_RELATIVE;
				TxMessage.Pitch_speed = Get_Channel_Val(&RC_ReceiveData, RC_CH3);
				TxMessage.Yaw_speed = Get_Channel_Val(&RC_ReceiveData, RC_CH2);
		}
		else {
				TxMessage.Chassis_speed = 8000 * Get_Channel_Val(&RC_ReceiveData,RC_CH0) / RC_CH_MAX_RELATIVE;
				TxMessage.Pitch_speed = 0;
				TxMessage.Yaw_speed = 0;
		}
		CAN_Com_Send();
    vTaskDelayUntil(&xLastWakeUpTime, 5);
	}
}
	/**
  * @brief  CAN发送函数
  * @param  void
  * @retval void
  */
void CAN_Com_Send(void)
{
   static CanSend_Type CANSend;

  CANSend.CANx = CANSEND_1;

  CANSend.stdid = 0x66;

  CANSend.Data[0] = (uint8_t)(TxMessage.Chassis_speed >> 8);
  CANSend.Data[1] = (uint8_t)TxMessage.Chassis_speed;
  CANSend.Data[2] = (uint8_t)ControlMode;
  CANSend.Data[3] = (uint8_t)(TxMessage.fricstatus<<4|TxMessage.stirstatus);
  CANSend.Data[4] = (uint8_t)(TxMessage.Yaw_speed >> 8);
  CANSend.Data[5] = (uint8_t)TxMessage.Yaw_speed;
  CANSend.Data[6] = (uint8_t)(TxMessage.Pitch_speed >> 8);
  CANSend.Data[7] = (uint8_t)TxMessage.Pitch_speed;
	
  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
	
//  CANSend.stdid = 0x68;

//  CANSend.Data[0] = (uint8_t)(TxMessage.Pitch_speed >> 8);
//  CANSend.Data[1] = (uint8_t)TxMessage.Pitch_speed;
//  CANSend.Data[2] = 0;
//  CANSend.Data[3] = 0;
//  CANSend.Data[4] = 0;
//  CANSend.Data[5] = 0;
//  CANSend.Data[6] = 0;
//  CANSend.Data[7] = 0;

//  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}
