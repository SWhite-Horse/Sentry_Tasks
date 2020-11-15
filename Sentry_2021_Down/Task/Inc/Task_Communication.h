#ifndef __TASKS_Communication_H
#define __TASKS_Ccommunication_H
#include "Task_StatusMachine.h"
#include "sysconfig.h"
//��̨���ݷ���CANID
#define MASTERSENDID 0x66
#define MASTERSENDID2 0x68

//�������ݷ���CANID
#define SLAVESENDID 0x69
#define SLAVESENDID2 0x67

extern ControlMode_Enum ControlMode;

//��̨�����̷��ͽṹ��
typedef struct
{
	int16_t Chassis_speed;
	ControlMode_Enum controlmode;
	FricStatus_Enum fricstatus;
	StirStatus_Enum stirstatus;
	int16_t Pitch_speed;
	int16_t Yaw_speed;
}Down_to_Up_Message;

//���̸���̨���ͽṹ��
typedef struct
{
	uint16_t Blood; //Ѫ��
	uint8_t Armour; //װ�װ�
	uint16_t Heat; //����
	uint8_t Shoot_Speed; //����
	uint8_t Toppoint_Judge; //�ж��Ƿ񵽴���ĩ��
	uint8_t get_hurt; //�Ƿ��ܵ��˺�
	uint16_t Bullet_remaining; //�ӵ�ʣ��
	uint8_t mains_power_shooter; //��Դ����ģ����������Ƿ��е�
}Up_to_Down_Message;

extern Up_to_Down_Message RxMessage;
extern Down_to_Up_Message TxMessage;
extern int LeftSwitch;
extern int RightSwitch;

void CAN_Com_Send(void);
#endif
