//与云台通讯任务
#include "Task_Communication.h"
#include "Task_JudgeReceive.h"
#include "Task_Can.h"



Up_to_Down_Message TxMessage;
Down_to_Up_Message Rxmessage ={
	.speed = 0,
	.controlmode = ControlMode_Telecontrol_DOWN,
	.fricstatus = FricStatus_Stop,
	.stirstatus = StirStatus_Stop,
	.pitch_speed = 0,
	.yaw_speed = 0
};

extern int LeftSwitch;
extern int RightSwitch;
extern int LengthSwitchLeft;
extern int LengthSwitchRight;
extern uint8_t get_hurted;

void send(void);
void Task_Communication(void *parameters)
{
	
	TickType_t xLastWakeUpTime;
  	xLastWakeUpTime = xTaskGetTickCount();
	while(1)
	{
		 LeftSwitch=HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_11);
 	 	 RightSwitch=HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12); //光电开关引脚接在GH11和GH12
		
		//将裁判系统的值赋给发送结构体
		TxMessage.Blood = ext_game_robot_state.remain_HP;
		TxMessage.get_hurt = get_hurted;
    TxMessage.Armour = ext_robot_hurt.armor_id;
		TxMessage.Heat = ext_power_heat_data.shooter_heat0;
		TxMessage.Shoot_Speed = ext_shoot_data.bullet_speed;
		TxMessage.Bullet_remaining = ext_bullet_remaining.bullet_remaining_num;
		TxMessage.mains_power_shooter = ext_game_robot_state.mains_power_shooter_output;
		
		//判断左右光电开关检测到达轨道末端
		if(LeftSwitch==0||RightSwitch==0)
			TxMessage.Toppoint_Judge = 1;
		else
			TxMessage.Toppoint_Judge = 0;

		send();
		vTaskDelayUntil(&xLastWakeUpTime, 4);
	}
}

/**
  * @brief  发送裁判系统数据给云台（少写了云台控制数据）
  * @param  None
  * @retval None
  */
void send(void)
{
		static CanSend_Type CANSend;

  	CANSend.CANx = CANSEND_1;

		CANSend.stdid = 0x69;

  	CANSend.Data[0] = (uint8_t)(TxMessage.Blood>>8);
		CANSend.Data[1] = (uint8_t)(TxMessage.Blood);
  	CANSend.Data[2] = (uint8_t)(TxMessage.Armour);
  	CANSend.Data[3] = (uint8_t)(TxMessage.Heat>>8);
		CANSend.Data[4] = (uint8_t)(TxMessage.Heat);
  	CANSend.Data[5] = (uint8_t)(TxMessage.Shoot_Speed );
  	CANSend.Data[6] = (uint8_t)(TxMessage.Toppoint_Judge);
  	CANSend.Data[7] = (uint8_t)(TxMessage.get_hurt);

		xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);

		CANSend.stdid = 0x67;
		CANSend.Data[0] = (uint8_t)(TxMessage.Bullet_remaining>>8);
		CANSend.Data[1] = (uint8_t)(TxMessage.Bullet_remaining);
  	CANSend.Data[2] = (uint8_t)(TxMessage.mains_power_shooter);
		CANSend.Data[3] = 0;
		CANSend.Data[4] = 0;
  	CANSend.Data[5] = 0;
  	CANSend.Data[6] = 0;
  	CANSend.Data[7] = 0;
	
  	xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}






