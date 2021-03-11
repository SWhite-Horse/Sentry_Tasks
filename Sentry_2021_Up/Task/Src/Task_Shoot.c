#define __TASK_FRICMOTOR_GLOBALS
#include "Task_Shoot.h"
#include "Task_Communication.h"
#include "Task_JetsonComm.h"
#include "Task_StatusMachine.h"
#include "Task_CAN.h"
#include "Task_JudgeReceive.h"


Motor3508_type Fric_3508_Motor[2];
RM2006_Type  StirMotor;
int16_t fric_motor_debug = 0;

void Task_Shoot(void *parameters){
	
	StirMotor_Init();
	Motor_3508_PID_Init();

	TickType_t xLastWakeUpTime;
	xLastWakeUpTime = xTaskGetTickCount();
	while(1){
	
		Fric_3508_Motor_Speed_Set();
		Motor_3508_PID_Calculate(&Fric_3508_Motor[0]);
		Motor_3508_PID_Calculate(&Fric_3508_Motor[1]);
		
		StirMotor_Control();
		
		
		Shoot_CAN_Send(Fric_3508_Motor[0].Output,Fric_3508_Motor[1].Output,StirMotor.Output);
		vTaskDelayUntil(&xLastWakeUpTime, 5);
	}
			
};

/**
  * @brief  摩擦轮3508电机初始化
  * @param  void
  * @retval void
  * @note   PID的初始化
  */

void Motor_3508_PID_Init(void){
	
	Fric_3508_Motor[0].PID.Kp = 9.1;//9.1;  //8; //0.5,
	Fric_3508_Motor[0].PID.Ki = 0.05;//0.05; //0.05,
	Fric_3508_Motor[0].PID.Kd = 8;//8;	//1,

	Fric_3508_Motor[1].PID.Kp = 9.1;//9.1;  //8; //0.5,
	Fric_3508_Motor[1].PID.Ki = 0.05;//0.05; //0.05,
	Fric_3508_Motor[1].PID.Kd = 8;//8;	//1,

};


/**
  * @brief  摩擦轮3508 速度初始化
  * @param  void
  * @retval void
  * @note   
  */

void Fric_3508_Motor_Speed_Set(void){
	
	int speed=0;
	if(FricStatus == FricStatus_Stop){
		speed = 0;
	}
	else{	
		if (FricStatus == FricStatus_Working_Low)
			speed = SPEEDMID;
		else if (FricStatus == FricStatus_Working_High)
			speed = SPEEDMAX;
		}
	Fric_3508_Motor[0].TargetSpeed = -speed;
	Fric_3508_Motor[1].TargetSpeed = speed;
}

/**
  * @brief  摩擦轮3508 PID
  * @param  Motor3508_type
  * @retval void
  * @note   PID的计算
  */

void Motor_3508_PID_Calculate(Motor3508_type *motor){
	
	motor->PID.Last_Error = motor->PID.Cur_Error;
	motor->PID.Cur_Error = motor->TargetSpeed - motor->RealSpeed;
	motor->PID.Sum_Error += motor->PID.Cur_Error;
	
	motor->PID.Sum_Error = motor->PID.Sum_Error > 15000 ? 15000 : motor->PID.Sum_Error;
	motor->PID.Sum_Error = motor->PID.Sum_Error < -15000 ? -15000 : motor->PID.Sum_Error;
	// **考虑添加清零
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
  StirMotor.PID.Kp = 6;//8;
  StirMotor.PID.Ki = 0.022;//0.022;
  StirMotor.PID.Kd = 0;
  StirMotor.TargetSpeed = 0;
}


/**
 * @description: 拨盘电机控制
 * @param {void} 
 * @return: void
 * @note: 
 */

int HeatControl=0;

int HeatStatus=1;

uint8_t HeatFlag = 0; //是否超热量
int16_t targetspeed = 4200; //拨盘转速
uint8_t ShootCounter=0; 
void StirMotor_Control(void)
{	
	if(ext_power_heat_data.shooter_id1_17mm_cooling_heat>=200)
	{
		HeatFlag=0;
	}
	else if(ext_power_heat_data.shooter_id1_17mm_cooling_heat<120)
	{
		HeatFlag=1;
	}
	//遥控模式
	if(ControlMode == ControlMode_Telecontrol_UP)
	{
		if(StirMotorStatus == StirStatus_SpeedControl && HeatFlag==1 )
			StirMotor.TargetSpeed = targetspeed;
		else
			StirMotor.TargetSpeed = 0;
	}
	//自瞄并且已经瞄到
	else if(ControlMode == ControlMode_Aimbot && DataRecFromJetson.SentryGimbalMode == ServoMode&& HeatFlag==1)// && RxMessage.mains_power_shooter==1 && HeatFlag==1)
	{
		if((DataRecFromJetson.ShootMode >> 8) == (RunningFire >> 8) )
		{
			StirMotor.TargetSpeed = targetspeed;
			ShootCounter=0;
		}
		else if((DataRecFromJetson.ShootMode >> 8) != (RunningFire >> 8) && ShootCounter <= 10 )
		{
			StirMotor.TargetSpeed = targetspeed;
			ShootCounter++;
		} 
		else 
		  StirMotor.TargetSpeed = 0;
	}
	else
		  StirMotor.TargetSpeed = 0;
	//堵转检测
	StirMotor_Blocked_Detect(&StirMotor);
	
	if(StirMotor.BlockedWarningTimes>0)
	{
		--StirMotor.BlockedWarningTimes;
		StirMotor.TargetSpeed = -targetspeed;	
	}
	if(HeatFlag ==0)
		StirMotor.TargetSpeed=0;

	
	if(StirMotor.TargetSpeed!=0){
		if(HeatStatus)
		{
			HeatControl+=3;	
		}
		else HeatControl-=2;
	}
	//else if(HeatControl>=0)
		//HeatControl-=2;
	
	if(HeatControl>600)
	{	
		HeatStatus=0;
	}
	
	if(HeatControl<10)
	{
		HeatStatus=1;
	}
	if(HeatStatus == 0) StirMotor.TargetSpeed=0;
	
  //if(TxMessage.get_hurt) StirMotor.TargetSpeed=0;

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

/**
  * @brief  发送 发射 三个电机 数据
  * @param  Fric_1 (ID 1), Fric_2 (ID 2), Stir (ID 3),
  * @retval void
  * @note   CAN2
  */
void Shoot_CAN_Send(int16_t fric_1,int16_t fric_2,int16_t stir)
{
  static CanSend_Type CANSend;

  CANSend.CANx = CANSEND_2;

  CANSend.stdid = 0x200;

  CANSend.Data[0] = (uint8_t)(fric_1 >> 8);
  CANSend.Data[1] = (uint8_t)fric_1;
  CANSend.Data[2] = (uint8_t)(fric_2 >> 8);
  CANSend.Data[3] = (uint8_t)fric_2;
	CANSend.Data[4] = (uint8_t)(stir >> 8);;
  CANSend.Data[5] = (uint8_t)stir;
  CANSend.Data[6] = (uint8_t)0;
  CANSend.Data[7] = (uint8_t)0;

  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}
