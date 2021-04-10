#ifndef __TASKS_Communication_H
#define __TASKS_Ccommunication_H
#include "Task_StatusMachine.h"
#include "sysconfig.h"
//云台数据发送CANID
#define MASTERSENDID 0x66
#define MASTERSENDID2 0x68

//底盘数据发送CANID
#define SLAVESENDID 0x69
#define SLAVESENDID2 0x67

extern ControlMode_Enum ControlMode;

typedef enum
{
    Debug_status,            
    Game_prepare,               
    Gaming,            
}Game_status_enum;

//云台给底盘发送结构体
typedef struct
{
	int16_t Chassis_speed;
	ControlMode_Enum controlmode;
	FricStatus_Enum fricstatus;
	StirStatus_Enum stirstatus;
	int16_t Pitch_speed;
	int16_t Yaw_speed;
}Down_to_Up_Message;

//底盘给云台发送结构体
typedef struct
{
	uint16_t Blood; //血量
	uint8_t Armour; //装甲板
	Game_status_enum Is_gaming;
	uint16_t Heat; //热量
	uint8_t Shoot_Speed_limit; //射速
	uint8_t Toppoint_Judge; //判断是否到达轨道末端
	uint8_t get_hurt; //是否受到伤害
	uint16_t Bullet_remaining; //子弹剩余
	uint8_t mains_power_shooter; //电源管理模块射击引脚是否有电
}Up_to_Down_Message;

extern Up_to_Down_Message RxMessage;
extern Down_to_Up_Message TxMessage;
extern int LeftSwitch;
extern int RightSwitch;

void CAN_Com_Send(void);
#endif
