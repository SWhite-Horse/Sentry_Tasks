#ifndef __SYSCONFIG_H__
#define __SYSCONFIG_H__

#include <stdlib.h>
#include <math.h>
#include <string.h> 

/**********   BSP   *********/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
/**********  OS   *********/
#include "cmsis_os.h"

/********** define *********/
/*led闪烁宏*/
#define Green_LED_Blink() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin) //绿灯闪烁
#define RED_LED_Blink() HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin) //绿灯闪烁

#define RM6623 0
#define RM6020 1
//阵营选择
#define WeAreRedTeam     HAL_GPIO_ReadPin(CampInput_GPIO_Port,CampInput_Pin)      //红方阵营选择
#define WeAreBlueTeam    !HAL_GPIO_ReadPin(CampInput_GPIO_Port,CampInput_Pin)     //蓝方阵营选择
/**********  uart *********/

//串口缓存区
#define MEMORY0 0
#define MEMORY1 1
#define MEMORYRESET 2

//********** PID结构体 *********//
typedef struct                               
{
	float Kp, Ki, Kd;
  float Cur_Error, Last_Error, Sum_Error;
}PID_type;

//********** 电机结构体 *********//
typedef struct
{
    //    uint16_t ID;
    uint16_t FrameCounter;
    float    TargetAngle;
    float    TargetSpeed;
		float    Real_Angle;
		int16_t Torque_Current_Real;
    int16_t Mechanical_Speed;
    int16_t NeedCurrent;
    int16_t Mechanical_Angle;
    uint8_t MotorTemp;
    struct
    {
        float Kp, Ki, Kd;
        float Cur_Error, Last_Error, Sum_Error;
        float Output;
    } SpeedPID;
    struct
    {
        float Kp, Ki, Kd;
        float Cur_Error, Last_Error, Sum_Error;
        float Output;
    } PositionPID;
}MotorType_6020;

typedef struct{
    uint16_t FrameCounter;
    int16_t  RealSpeed;
		int16_t  RealCurrent;
    int16_t  Mechanical_Angle;
    int16_t  TargetSpeed;
    int8_t  BlockedWarningTimes;
		PID_type PID;
	  int32_t Output;
} RM2006_Type;

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

#define xabs(x)  x>0 ? (x) : (-x)  //绝对值定义

#define FRICMOTOR_PWM_CTRL //FRICMOTOR_PID_CTRL FRICMOTOR_PWM_CTRL

#define ONBOARDIMU

#endif
