#ifndef __TASKS_CAN_H
#define __TASKS_CAN_H

#ifdef __TASK_CAN_GLOBALS
#define TASK_CAN_EXT
#else
#define TASK_CAN_EXT extern
#endif

#include "Task_Init.h"


#define CANSEND_1 1               /*CANSEND发送 1 0x200 2 0x1ff*/
#define CANSEND_2 2 
//C620电调电流最大值
#define C620CURRENTMAX 16000
   

typedef struct
{
  uint16_t Mechanical_Angle;       //机械角
  int16_t TargetSpeed;
  int16_t RealSpeed;
  int16_t RealCurrent;
  uint16_t FrameCounter; 	//帧率计数器
	PID_type PID;
	int32_t Output;
}Motor3508_type;

typedef struct
{
    uint8_t            CANx;               //CAN1 or CAN2 发送
    uint32_t           stdid;              // ID: CAN1 0X200  CAN2 0X1FF
		uint8_t            Data[8];
}CanSend_Type;

TASK_CAN_EXT CanSend_Type CAN_Tx_Msg;

void CANTransmit(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[]);


#endif
