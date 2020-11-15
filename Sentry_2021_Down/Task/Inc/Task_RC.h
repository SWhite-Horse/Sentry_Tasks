#ifndef  __TASKS_RC_H__
#define  __TASKS_RC_H__

#include "Task_Init.h"

#ifdef  __TASK_RC_GLOBALS
#define TASK_RC_EXT
#else
#define TASK_RC_EXT extern
#endif

//遥控器通道相对最大值
#define RC_CH_MAX_RELATIVE 660

#define RC_FRAME_LEN        18U         //遥控器数据帧长
#define RC_FRAME_LEN_BACK   7U          //增加两个字节保持稳定

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_CH0    ((uint8_t)0)
#define RC_CH1    ((uint8_t)1)
#define RC_CH2    ((uint8_t)2)
#define RC_CH3    ((uint8_t)3)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP    ((uint16_t)1)
#define RC_SW_MID   ((uint16_t)3)
#define RC_SW_DOWN  ((uint16_t)2)
#define RC_SW_Right ((uint8_t)0)
#define RC_SW_Left  ((uint8_t)1)



typedef struct
{ 
  uint16_t RCFrameCounter;       //遥控器帧率计数器

  uint16_t ch0;
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint8_t  Switch_Left;
  uint8_t  Switch_Right;
}RCDecoding_Type;


void RC_Receive_Enable(UART_HandleTypeDef *huart);
void RC_InitConfig(void);
void RC_Data_Update(void);
void RC_UART_IRQHandler(UART_HandleTypeDef *huart);
int16_t Get_Channel_Val(RCDecoding_Type *RC_ReceiveData,uint8_t channel_num);
uint8_t Get_Switch_Val(RCDecoding_Type *RC_ReceiveData,uint8_t switch_num);

TASK_RC_EXT RCDecoding_Type RC_ReceiveData,LastRC_ReceiveData;
TASK_RC_EXT uint8_t RCBuffer[2][RC_FRAME_LEN+RC_FRAME_LEN_BACK];
TASK_RC_EXT uint8_t RC_Rx_Mem;            //指示当前遥控器接收使用的缓冲区编号 

#endif
