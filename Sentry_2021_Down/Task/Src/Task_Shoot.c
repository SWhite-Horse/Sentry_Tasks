 #define __TASK_FRICMOTOR_GLOBALS
#include "Task_Shoot.h"
#include "Task_Communication.h"
#include "Task_JetsonComm.h"
#include "Task_StatusMachine.h"
#include "Task_CAN.h"
#include "Task_Gimbal.h"
Shoot_Method_enum Shoot_Method = Long_Mode;
Heat_Limitation_t Heat_Limitation ={
	.Heat_Limitation_UP = 280,
	.Heat_Limitation_DOWN = 80,
};
Motor3508_type Fric_3508_Motor[2];
RM2006_Type  StirMotor;
int16_t fric_motor_debug = 0;

void Task_Shoot(void *parameters){
	
	StirMotor_Init();
	Motor_3508_PID_Init();

	TickType_t xLastWakeUpTime;
	xLastWakeUpTime = xTaskGetTickCount();
	while(1){
			//****** 发射口电源不断电
		if(RxMessage.mains_power_shooter==1 && RxMessage.Is_gaming != Game_prepare){
			if(ControlMode==ControlMode_Aimbot)
				FricStatus = FricStatus_Working_High;
		}
		else{
			FricStatus = FricStatus_Stop;
			StirMotorStatus = StirStatus_Stop;
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
	
	Fric_3508_Motor[0].PID.Kp = 16.5; //9.1;
	Fric_3508_Motor[0].PID.Ki = 0.1; //0.05;
	Fric_3508_Motor[0].PID.Kd = 0.2; //;//8;

	Fric_3508_Motor[1].PID.Kp = 16.5;
	Fric_3508_Motor[1].PID.Ki = 0.1; 
	Fric_3508_Motor[1].PID.Kd = 0.2; 

};


/**
  * @brief  摩擦轮3508 速度初始化
  * @param  void
  * @retval void
  * @note   
  */

int SPEEDMAX = 7300;

void Fric_3508_Motor_Speed_Set(void)
{
	if(RxMessage.Shoot_Speed)
	{
		SPEEDMAX=7000;
	}
	
	else SPEEDMAX=7300;

	
	
	
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
uint8_t HeatFlag = 0; //是否超热量  ????????????????  考虑整合 》？？？？？？？？？
int16_t targetspeed = -7000; //拨盘转速
uint8_t ShootCounter=0; 
uint8_t Heat_limit_method=0;
int HeatControl=0;

void StirMotor_Control(void)
{ 
	// 根据上与台读到的裁判系统数据中是否含有下云台枪口射速上限来判断比赛是否开始，从而选择不同的热量限制策略
	Heat_limit_method = RxMessage.Shoot_Speed_limit==30 ? 1 : 0;
	Shoot_Status_with_Heat(PITCH_ANGLE);
	
//	if(Heat_limit_method){
//		if(RxMessage.Heat>=280)
//		{
//			HeatFlag=0;
//		}
//		else if(RxMessage.Heat<=120)
//		{
//			HeatFlag=1;
//		}
//	}
//	else{
//		if(HeatControl>800)
//		{	
//			HeatFlag=0;
//		}
//	
//		if(HeatControl<10)
//		{
//			HeatFlag=1;
//		}
//	}
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
		if((DataRecFromJetson.ShootMode & (uint16_t)(0x2000)))//(DataRecFromJetson.ShootMode >> 8) == (RunningFire >> 8) )
		{
			StirMotor.TargetSpeed = targetspeed;
			ShootCounter=0;
		}
		else if((DataRecFromJetson.ShootMode & (uint16_t)(0x2000)) && ShootCounter <= 10 )
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
  * @brief  发射模式及热量控制(注意上下云台的Pitch坐标系相反）
  * @param  Pitch_Angle(以及头文件有模式角度宏定义）
  * @retval void 
  */

void Shoot_Status_with_Heat(uint16_t angle){
	if(angle <= Shoot_Angle_Short)
		Shoot_Method = Three_Mode;
	else if (angle <= Shoot_Angle_Long && angle > Shoot_Angle_Short)
		Shoot_Method  = Short_Mode;
	else if (angle > Shoot_Angle_Long && angle <= 44)
		Shoot_Method = Long_Mode;
	
	switch (Shoot_Method){
		case Three_Mode:
			Heat_Limitation.Heat_Limitation_UP = 50;
			Heat_Limitation.Heat_Limitation_DOWN = 10;
			break;
		case Short_Mode:
			Heat_Limitation.Heat_Limitation_UP = 200;
			Heat_Limitation.Heat_Limitation_DOWN = 80;
			break;
		case Long_Mode:
			Heat_Limitation.Heat_Limitation_UP = 290;
			Heat_Limitation.Heat_Limitation_DOWN = 160;
			break;
		default:
			break;
	}		
		if(Heat_limit_method){
			if(RxMessage.Heat >= Heat_Limitation.Heat_Limitation_UP){
				HeatFlag=0;
			}
			else if(RxMessage.Heat<=Heat_Limitation.Heat_Limitation_DOWN){
				HeatFlag=1;
			}
		}
		else{
			if(HeatControl>1200){	
				HeatFlag=0;
			}
			if(HeatControl<10){
				HeatFlag=1;
			}
		}
	
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
