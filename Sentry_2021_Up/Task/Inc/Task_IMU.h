#ifndef __TASKIMU_H
#define __TASKIMU_H

#include "Task_Init.h"
#include "Func_Imu_OB.h"


#define OFFICIALZGYRORESETCANID 0x404   //�ٷ������Ǹ�λCANID
#define OFFICIALZGYRORECEIVECANID 0x401 //�ٷ������ǽ���CANID

typedef struct
{
	float pitch;
	float yaw;
	float rol;
}myimu_t;


//extern myimu_t Integral_IMU;

float CharsToFloat(uint8_t *s);

#endif
