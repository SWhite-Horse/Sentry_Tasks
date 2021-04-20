#define __TASK_FRICMOTOR_GLOBALS
#include "Task_Shoot.h"
#include "Task_Communication.h"
#include "Task_JetsonComm.h"
#include "Task_StatusMachine.h"
#include "Task_CAN.h"
#include "Task_JudgeReceive.h"
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
	
		//****** ����ڵ�Դ���ϵ��ұ�����ʽ��ʼ��4�����ʽģʽ��0��Ħ���ָ���
		if( TxMessage.mains_power_shooter==1 && TxMessage.Is_gaming != Game_prepare){
			if(ControlMode==ControlMode_Aimbot && CommStatus.CommSuccess == 1 )
				FricStatus = FricStatus_Working_High;
		}
		else{
			FricStatus = FricStatus_Stop;
			StirMotorStatus = StirStatus_Stop;
		}
		
		Fric_3508_Motor_Speed_Set();
		Motor_3508_PID_Calculate(&Fric_3508_Motor[0]);
		Motor_3508_PID_Calculate(&Fric_3508_Motor[1]);
		
		StirMotor_Control();
		Shoot_CAN_Send(Fric_3508_Motor[0].Output,Fric_3508_Motor[1].Output,StirMotor.Output);
		vTaskDelayUntil(&xLastWakeUpTime, 5);
	}
			
};

/**
  * @brief  Ħ����3508�����ʼ��
  * @param  void
  * @retval void
  * @note   PID�ĳ�ʼ��
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
  * @brief  Ħ����3508 �ٶȳ�ʼ��
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
  * @brief  Ħ����3508 PID
  * @param  Motor3508_type
  * @retval void
  * @note   PID�ļ���
  */

void Motor_3508_PID_Calculate(Motor3508_type *motor){
	
	motor->PID.Last_Error = motor->PID.Cur_Error;
	motor->PID.Cur_Error = motor->TargetSpeed - motor->RealSpeed;
	motor->PID.Sum_Error += motor->PID.Cur_Error;
	
	motor->PID.Sum_Error = motor->PID.Sum_Error > 15000 ? 15000 : motor->PID.Sum_Error;
	motor->PID.Sum_Error = motor->PID.Sum_Error < -15000 ? -15000 : motor->PID.Sum_Error;
	// **�����������
	motor->Output = (motor->PID.Kp * motor->PID.Cur_Error + motor->PID.Ki * motor->PID.Sum_Error + motor->PID.Kd * (motor->PID.Cur_Error - motor->PID.Last_Error));

	//�����������
	motor->Output = (motor->Output >= C620CURRENTMAX) ? C620CURRENTMAX : motor->Output;
	motor->Output = (motor->Output <= -C620CURRENTMAX) ? -C620CURRENTMAX : motor->Output;
};

/**
 * @description: ���̵����ʼ��
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
 * @description: ���̵������
 * @param {void} 
 * @return: void
 * @note: 
 */

int HeatControl=0;
uint8_t Heat_limit_method=0;
uint8_t HeatFlag = 0; //�Ƿ�����
int16_t targetspeed = 4200; //����ת��
uint8_t ShootCounter=0; 

void StirMotor_Control(void)
{	
	Heat_limit_method = ext_game_robot_state.shooter_id1_17mm_speed_limit==30 ? 1 : 0; 
	Shoot_Status_with_Heat(PITCH_ANGLE);
	
//	if(Heat_limit_method){
//		if(ext_power_heat_data.shooter_id1_17mm_cooling_heat>=240){
//			HeatFlag=0;
//		}	
//		else if(ext_power_heat_data.shooter_id1_17mm_cooling_heat<120){
//			HeatFlag=1;
//		}
//	}
//	else{
//		if(HeatControl>600){	
//			HeatFlag=0;
//		}
//		if(HeatControl<10){
//			HeatFlag=1;
//		}
//	}
	//ң��ģʽ
	if(ControlMode == ControlMode_Telecontrol_UP)
	{
		if(StirMotorStatus == StirStatus_SpeedControl && HeatFlag==1 )
			StirMotor.TargetSpeed = targetspeed;
		else
			StirMotor.TargetSpeed = 0;
	}
	//���鲢���Ѿ��鵽
	else if(ControlMode == ControlMode_Aimbot && DataRecFromJetson.SentryGimbalMode == ServoMode&& HeatFlag==1 && TxMessage.mains_power_shooter==1 )
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
	//��ת���
	StirMotor_Blocked_Detect(&StirMotor);
	
	if(StirMotor.BlockedWarningTimes>0)
	{
		--StirMotor.BlockedWarningTimes;
		StirMotor.TargetSpeed = -targetspeed;	
	}
	if(HeatFlag ==0)
		StirMotor.TargetSpeed=0;

	if(StirMotor.TargetSpeed!=0){
		if(HeatFlag){
			HeatControl+=2;	
		}
		else HeatControl-=1;
	}
	else if(StirMotor.TargetSpeed==0&&HeatControl>=10)
		HeatControl-=2;
	
	if (FricStatus == FricStatus_Stop)
		StirMotor.TargetSpeed=0;
	Stir_Motor_Speed_Control(&StirMotor);
	
}


/**
  * @brief  ����ģʽ����������(ע��������̨��Pitch����ϵ�෴��
  * @param  Pitch_Angle(�Լ�ͷ�ļ���ģʽ�ǶȺ궨�壩
  * @retval void 
  */

void Shoot_Status_with_Heat(uint16_t angle){
	if(angle <= Shoot_Angle_Short)
		Shoot_Method = Three_Mode;
	else if (angle <= Shoot_Angle_Long && angle > Shoot_Angle_Short)
		Shoot_Method  = Short_Mode;
	else if (angle > Shoot_Angle_Long && angle <= 27)
		Shoot_Method = Long_Mode;
	
	switch (Shoot_Method){
		case Three_Mode:
			Heat_Limitation.Heat_Limitation_UP = 50;
			Heat_Limitation.Heat_Limitation_DOWN = 20;
			break;
		case Short_Mode:
			Heat_Limitation.Heat_Limitation_UP = 200;
			Heat_Limitation.Heat_Limitation_DOWN = 20;
			break;
		case Long_Mode:
			Heat_Limitation.Heat_Limitation_UP = 280;
			Heat_Limitation.Heat_Limitation_DOWN = 20;
			break;
		default:
			break;
	}
	if(Heat_limit_method){
		if(ext_power_heat_data.shooter_id1_17mm_cooling_heat >= Heat_Limitation.Heat_Limitation_UP){
			HeatFlag=0;
		}	
		else if(ext_power_heat_data.shooter_id1_17mm_cooling_heat < Heat_Limitation.Heat_Limitation_DOWN){
			HeatFlag=1;
		}
	}
	else{
		if(HeatControl>600){	
			HeatFlag=0;
		}
		if(HeatControl<10){
			HeatFlag=1;
		}
	}
}


/**
  * @brief  ���̵���ٶȱջ�
  * @param  RM�����̵���ṹ��
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
  * @brief  ���̵����ת���
  * @param  RM�����̵���ṹ��
  * @retval void
  * @note   100ms��� 200ms��ת
  */
void StirMotor_Blocked_Detect(RM2006_Type* motor)
{
  static uint8_t BlockedTimes = 0;
	//��ת������ɺ��ٴμ��
  if(motor->BlockedWarningTimes <= 0)
  {
    if(abs(motor->RealSpeed) < 100 && abs(motor->Output) == 10000) 
      BlockedTimes++;
    else
      BlockedTimes = 0;
    //����100ms��⵽��ת
    if(BlockedTimes >= 20)
    {
      motor->BlockedWarningTimes = 40;
      BlockedTimes = 0;
    }
  }
}

/**
  * @brief  ���� ���� ������� ����
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
