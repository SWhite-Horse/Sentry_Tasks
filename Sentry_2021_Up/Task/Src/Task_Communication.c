//与云台通讯任务
#include "Task_Communication.h"
#include "Task_JudgeReceive.h"
#include "Task_Can.h"



Up_to_Down_Message TxMessage;
Down_to_Up_Message RxMessage ={
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


void Task_Communication(void *parameters)
{
	
	TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
	while(1)
	{	
		
		if(ext_game_state.game_type != 0){
			if(ext_game_state.game_progress == 4 || ext_game_state.game_progress == 0) TxMessage.Is_gaming = Gaming; 
			else TxMessage.Is_gaming = Game_prepare;
		}
		else TxMessage.Is_gaming = Debug_status;
		//将裁判系统的值赋给发送结构体
		TxMessage.get_hurt = get_hurted;
    TxMessage.Armour = ext_game_robot_state.robot_id;
		TxMessage.Heat = ext_power_heat_data.shooter_id2_17mm_cooling_heat;
		TxMessage.Shoot_Speed_Limit = ext_game_robot_state.shooter_id2_17mm_speed_limit;
		TxMessage.mains_power_shooter = ext_game_robot_state.mains_power_shooter_output;
		if(ext_shoot_data.shooter_id == 2 && ext_shoot_data.bullet_speed > 28.9f)
			TxMessage.Shoot_Speed = 1;
		if(ext_shoot_data.shooter_id == 2 && ext_shoot_data.bullet_speed < 26.5f)
			TxMessage.Shoot_Speed = 0;
		
		//判断左右光电开关检测到达轨道末端
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

  	CANSend.Data[0] = (uint8_t)(TxMessage.Is_gaming);
		CANSend.Data[1] = (uint8_t)(TxMessage.Shoot_Speed);
  	CANSend.Data[2] = (uint8_t)(TxMessage.Armour);
  	CANSend.Data[3] = (uint8_t)(TxMessage.Heat>>8);
		CANSend.Data[4] = (uint8_t)(TxMessage.Heat);
  	CANSend.Data[5] = (uint8_t)(TxMessage.Shoot_Speed_Limit);
  	CANSend.Data[6] = (uint8_t)(TxMessage.mains_power_shooter);
  	CANSend.Data[7] = (uint8_t)(TxMessage.get_hurt);

		xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);

}


