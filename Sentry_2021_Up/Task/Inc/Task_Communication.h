#ifndef __TASKS_Communication_H
#define __TASKS_Ccommunication_H
#include "sysconfig.h"
#include "Task_StatusMachine.h"
//��̨���ݷ���CANID
#define MASTERSENDID 0x66
#define MASTERSENDID2 0x68

//�������ݷ���CANID
#define SLAVESENDID 0x69
#define SLAVESENDID2 0x67

extern ControlMode_Enum ControlMode;


typedef enum
{
    Debug_status,            
    Game_prepare,               
    Gaming,            
}Game_status_enum;

//��̨�����̷��ͽṹ��
typedef struct
{
	int16_t speed;
	ControlMode_Enum controlmode;
	FricStatus_Enum fricstatus;
	StirStatus_Enum stirstatus;
	int16_t pitch_speed;
	int16_t yaw_speed;
}Down_to_Up_Message;

//���̸���̨���ͽṹ��
typedef struct
{
	//uint16_t Blood; //Ѫ��
	uint8_t Armour; //װ�װ�
	uint16_t Heat; //����
	uint8_t Shoot_Speed_Limit; //����
	uint8_t Shoot_Speed; //�ж��Ƿ񵽴���ĩ��
	uint8_t get_hurt; //�Ƿ��ܵ��˺�
	Game_status_enum Is_gaming;
	//uint16_t Bullet_remaining; //�ӵ�ʣ��
	uint8_t mains_power_shooter; //��Դ����ģ����������Ƿ��е�
}Up_to_Down_Message;

extern Up_to_Down_Message TxMessage;
extern Down_to_Up_Message RxMessage;
extern int LeftSwitch;
extern int RightSwitch;

void send(void);
#endif
