 #define __TASK_FRICMOTOR_GLOBALS
#include "Task_Shoot.h"
#include "Task_Communication.h"
#include "Task_JetsonComm.h"
#include "Task_StatusMachine.h"
#include "Task_CAN.h"

Motor3508_type Fric_3508_Motor[2];
RM2006_Type  StirMotor;
int16_t fric_motor_debug = 0;

void Task_Shoot(void *parameters){
	
	StirMotor_Init();
	Motor_3508_PID_Init();

	TickType_t xLastWakeUpTime;
	xLastWakeUpTime = xTaskGetTickCount();
	while(1){
		
		if(ControlMode==ControlMode_Aimbot && CommStatus.CommSuccess == 1 ){
			//****** 发射口电源不断电
			if( RxMessage.mains_power_shooter==1 && RxMessage.Is_gaming != Game_prepare){
				FricStatus = FricStatus_Working_High;
			}
			else{
				FricStatus = FricStatus_Stop;
				StirMotorStatus = StirStatus_Stop;
			}
		}
		
		Fric_3508_Motor_Speed_Set();
		Motor_3508_PID_Calculate(&Fric_3508_Motor[0]);
		Motor_3508_PID_Calculate(&Fric_3508_Motor[1]);
		
		Motor_3508_Send(Fric_3508_Motor[0].Output,Fric_3508_Motor[1].Output);
		
		StirMotor_Control();
		//Stir_CAN_Send(StirMotor.Output);///发送在Gimbal里面
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
	
	Fric_3508_Motor[0].PID.Kp = 9.1;  //8; //0.5,
	Fric_3508_Motor[0].PID.Ki = 0.05; //0.05,
	Fric_3508_Motor[0].PID.Kd = 8;	//1,

	Fric_3508_Motor[1].PID.Kp = 9.1;  //8; //0.5,
	Fric_3508_Motor[1].PID.Ki = 0.05; //0.05,
	Fric_3508_Motor[1].PID.Kd = 8;	//1,

};


/**
  * @brief  摩擦轮3508 速度初始化
  * @param  void
  * @retval void
  * @note   
  */

void Fric_3508_Motor_Speed_Set(void)
{
	int speed=0;
		if (FricStatus == FricStatus_Stop){
			speed = 0;
		}
		else
		{	
			if (FricStatus == FricStatus_Working_Low)
				speed = SPEEDMID;
			else if (FricStatus == FricStatus_Working_High)
				speed = SPEEDMAX;
		}
	Fric_3508_Motor[0].TargetSpeed = speed;
	Fric_3508_Motor[1].TargetSpeed = -speed;
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
  StirMotor.PID.Kp = 6;
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
int16_t targetspeed = -5500; //拨盘转速
uint8_t ShootCounter=0; 
uint8_t Heat_limit_method=0;
int HeatControl=0;

void StirMotor_Control(void)
{ 
	// 根据上与台读到的裁判系统数据中是否含有下云台枪口射速上限来判断比赛是否开始，从而选择不同的热量限制策略
	Heat_limit_method = RxMessage.Shoot_Speed_limit==30 ? 1 : 0; 
	if(Heat_limit_method){
		if(RxMessage.Heat>=280)
		{
			HeatFlag=0;
		}
		else if(RxMessage.Heat<=120)
		{
			HeatFlag=1;
		}
	}
	else{
		if(HeatControl>800)
		{	
			HeatFlag=0;
		}
	
		if(HeatControl<10)
		{
			HeatFlag=1;
		}
	}
	//遥控模式
	if(ControlMode == ControlMode_Telecontrol_DOWN)
	{
		if(StirMotorStatus == StirStatus_SpeedControl && HeatFlag==1 )
			StirMotor.TargetSpeed = targetspeed;
		else
			StirMotor.TargetSpeed = 0;
	}
	//自瞄并且已经瞄到
	else if(ControlMode == ControlMode_Aimbot && DataRecFromJetson.SentryGimbalMode == ServoMode && RxMessage.mains_power_shooter==1 && HeatFlag==1)
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
		if(HeatFlag)
		{
			HeatControl+=2;	
		}
		else HeatControl-=1;
	}
	else if(StirMotor.TargetSpeed==0&&HeatControl>=10)
		HeatControl-=2;
	
//	
//	if(HeatControl>600)
//	{	
//		HeatStatus=0;
//	}
//	
//	if(HeatControl<10)
//	{
//		HeatStatus=1;
//	}
	//if(RxMessage.get_hurt==3) StirMotor.TargetSpeed=0;
	if (FricStatus == FricStatus_Stop)
				StirMotor.TargetSpeed = 0;

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
      motor->BlockedWarningTimes = 60;
      BlockedTimes = 0;
    }
  }
}



/**
  * @brief  发送摩擦轮电机数据
  * @param  output
  * @retval void
  * @note   CAN2
  */

void Motor_3508_Send(int16_t Output0,int16_t Output1)
{
  static CanSend_Type CANSend;

  CANSend.CANx = CANSEND_2;

  CANSend.stdid = 0x200;

  CANSend.Data[0] = (uint8_t)(Output0 >> 8);
  CANSend.Data[1] = (uint8_t)Output0;
  CANSend.Data[2] = (uint8_t)(Output1 >> 8);
  CANSend.Data[3] = (uint8_t)Output1;
	CANSend.Data[4] = (uint8_t)0;
  CANSend.Data[5] = (uint8_t)0;
  CANSend.Data[6] = (uint8_t)0;
  CANSend.Data[7] = (uint8_t)0;

  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}
