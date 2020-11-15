#define __TASK_FRICMOTOR_GLOBALS
#include "Task_Shoot.h"
#include "Task_CAN.h"
#include "Task_Communication.h"


Motor3508_type Fric_Motor_3508[2];
RM2006_Type  StirMotor;

void Task_Shoot(void *parameters){
		StirMotor_Init();
		TickType_t xLastWakeUpTime;
		xLastWakeUpTime = xTaskGetTickCount();
//		Motor_3508_PID_Init();
//		Motor_3508[0].TargetSpeed=500;
		while(1){
//				Motor_3508_PID_Calculate(&Motor_3508[0]);
//				Motor_3508_Send(Motor_3508[0].Output);
			
		StirMotor_Control();
		Stir_CAN_Send(StirMotor.Output);
		vTaskDelayUntil(&xLastWakeUpTime, 5);
		}
			
};


void Motor_3508_PID_Init(void){
	
		Fric_Motor_3508[0].PID.Kp=0.5;
		Fric_Motor_3508[0].PID.Ki=0;
		Fric_Motor_3508[0].PID.Kd=0;

};

void Motor_3508_PID_Calculate(Motor3508_type *motor){
	
		motor->PID.Last_Error = motor->PID.Cur_Error;
		motor->PID.Cur_Error = motor->TargetSpeed - motor->RealSpeed;
		motor->PID.Sum_Error += motor->PID.Cur_Error;
	
		motor->PID.Sum_Error = motor->PID.Sum_Error > 15000 ? 15000 : motor->PID.Sum_Error;
		motor->PID.Sum_Error = motor->PID.Sum_Error < -15000 ? -15000 : motor->PID.Sum_Error;
	
		motor->Output = (motor->PID.Kp * motor->PID.Cur_Error + motor->PID.Ki * motor->PID.Sum_Error + motor->PID.Kd * (motor->PID.Cur_Error - motor->PID.Last_Error));

		 //限制输出电流
		motor->Output = (motor->Output >= C620CURRENTMAX) ? C620CURRENTMAX : motor->Output;
		motor->Output = (motor->Output <= -C620CURRENTMAX) ? -C620CURRENTMAX : motor->Output;
};

/**
 * @description: 拨盘电机初始化
 * @param {void} 
 * @return: void
 * @note: 
 */ 
void StirMotor_Init(void)
{
  StirMotor.PID.Kp = 8;
  StirMotor.PID.Ki = 0.022;
  StirMotor.PID.Kd = 0;
  StirMotor.TargetSpeed = 0;
}

/**
 * @description: 拨盘电机控制
 * @param {void} 
 * @return: void
 * @note: 
 */
uint8_t HeatFlag = 0; //是否超热量
int16_t targetspeed = -4200; //拨盘转速
uint8_t ShootCounter=0; 
void StirMotor_Control(void)
{
//	if(TxMessage.Heat>=200)
//	{
//		HeatFlag=0;
//	}
//	else if(TxMessage.Heat<=120)
//	{
//		HeatFlag=1;
//	}
	//遥控模式
	if(ControlMode != ControlMode_Telecontrol_UP)
	{
		if(StirMotorStatus == StirStatus_SpeedControl)// && HeatFlag==1 )
			StirMotor.TargetSpeed = targetspeed;
		else
			StirMotor.TargetSpeed = 0;
	}
	//堵转检测
	StirMotor_Blocked_Detect(&StirMotor);
	
	if(StirMotor.BlockedWarningTimes>0)
	{
		--StirMotor.BlockedWarningTimes;
		StirMotor.TargetSpeed = -targetspeed;	
	}
//	if(HeatFlag ==0)
//		StirMotor.TargetSpeed=0;

	Stir_Motor_Speed_Control(&StirMotor);
	
}

/**
  * @brief  拨盘电机速度闭环
  * @param  RM：拨盘电机结构体
  * @retval void
  */
void Stir_Motor_Speed_Control(RM2006_Type* motor)
{
  motor->PID.Last_Error = motor->PID.Cur_Error;
  motor->PID.Cur_Error = motor->TargetSpeed - motor->RealSpeed;
  motor->PID.Sum_Error += motor->PID.Cur_Error;
  
  motor->PID.Sum_Error = motor->PID.Sum_Error > 15000 ? 15000 : motor->PID.Sum_Error;
  motor->PID.Sum_Error = motor->PID.Sum_Error < -15000 ? -15000 : motor->PID.Sum_Error;

  motor->Output = (motor->PID.Kp * motor->PID.Cur_Error\
                                    + motor->PID.Ki * motor->PID.Sum_Error\
                                    + motor->PID.Kd * (motor->PID.Cur_Error - motor->PID.Last_Error));
  motor->Output = motor->Output > C610CURRENTMAX ? C610CURRENTMAX : motor->Output;
  motor->Output = motor->Output < -C610CURRENTMAX ? -C610CURRENTMAX : motor->Output;
}

/**
  * @brief  拨盘电机赌转检测
  * @param  RM：拨盘电机结构体
  * @retval void
  * @note   100ms检测 200ms反转
  */
void StirMotor_Blocked_Detect(RM2006_Type* motor)
{
  static uint8_t BlockedTimes = 0;
	//反转控制完成后再次检测
  if(motor->BlockedWarningTimes <= 0)
  {
    if(abs(motor->RealSpeed) < 100 && abs(motor->Output) == 10000) 
      BlockedTimes++;
    else
      BlockedTimes = 0;
    //连续100ms检测到堵转
    if(BlockedTimes >= 20)
    {
      motor->BlockedWarningTimes = 40;
      BlockedTimes = 0;
    }
  }
}


void Stir_CAN_Send(int16_t Output)
{
  static CanSend_Type CANSend;

  CANSend.CANx = CANSEND_1;

  CANSend.stdid = 0x1ff;

  CANSend.Data[0] = (uint8_t)0;
  CANSend.Data[1] = (uint8_t)0;
  CANSend.Data[2] = (uint8_t)0;
  CANSend.Data[3] = (uint8_t)0;
  CANSend.Data[4] = (uint8_t)(Output >> 8);
  CANSend.Data[5] = (uint8_t)Output;
  CANSend.Data[6] = (uint8_t)0;
  CANSend.Data[7] = (uint8_t)0;

  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}


//void Motor_3508_Send(int16_t Output)
//{
//  static CanSend_Type CANSend;

//  CANSend.CANx = CANSEND_1;

//  CANSend.stdid = 0x200;

//			CANSend.Data[0] = (uint8_t)0;
//			CANSend.Data[1] = (uint8_t)0;
//			CANSend.Data[2] = (uint8_t)0;
//			CANSend.Data[3] = (uint8_t)0;
//			CANSend.Data[4] = (uint8_t)0;
//			CANSend.Data[5] = (uint8_t)0;
//			CANSend.Data[6] = (uint8_t)0;
//			CANSend.Data[7] = (uint8_t)0;

//  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
//}
