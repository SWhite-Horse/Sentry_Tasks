#ifndef __TASKS_MEASURE_H
#define __TASKS_MEASURE_H

#include "Task_Init.h"

#define DISTANCE 0x44
#define ERROR 0x45

typedef struct
{
	uint8_t SOF;  //D
	uint8_t Equal;//=
	uint8_t n1;  //ÕûÊýÎ»
	uint8_t D;  // .
	uint8_t n2; 
	uint8_t n3;
	uint8_t n4;
	uint8_t DW;
	uint8_t K;
	uint8_t H;
}L1_MEASURE_STRUCT;

extern float Distance;
extern L1_MEASURE_STRUCT L1_Measure_Struct;

void L1_Measure_Init(UART_HandleTypeDef *huart);
void L1_Measure_Callback(UART_HandleTypeDef *huart);
void L1_Measure_Update(void);

#endif
