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
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"


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

typedef struct                                   //PID结构体
{
	float Kp, Ki, Kd;
  float Cur_Error, Last_Error, Sum_Error;
}PID_type;

#define xabs(x)  x>0 ? (x) : (-x)  //绝对值定义



#define FRICMOTOR_PWM_CTRL //FRICMOTOR_PID_CTRL FRICMOTOR_PWM_CTRL
//陀螺仪选择 PERSONALGYRO_HI219 PERSONALGYRO_JY901 ONBOARDIMU
//#define PERSONALGYRO_HI219
//#define PERSONALGYRO_ENABLE 
#define ONBOARDIMU



#endif
