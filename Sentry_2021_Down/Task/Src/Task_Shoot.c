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
		
			Fric_3508_Motor_Speed_Set();
			Motor_3508_PID_Calculate(&Fric_3508_Motor[0]);
			Motor_3508_PID_Calculate(&Fric_3508_Motor[1]);

			Motor_3508_Send(Fric_3508_Motor[0].Output,Fric_3508_Motor[1].Output);
			
			if(DataRecFromJetson.SentryGimbalMode == ServoMode && ControlMode==ControlMode_Aimbot && CommStatus.CommSuccess == 1 ){
			if(RxMessage.mains_power_shooter==0){
					FricStatus = FricStatus_Stop;
					StirMotorStatus = StirStatus_Stop;
				}
				else{
					FricStatus = FricStatus_Working_High;
				}
			}
			StirMotor_Control();
			Stir_CAN_Send(StirMotor.Output);
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
	
	Fric_3508_Motor[0].PID.Kp = 9.1;  //8; //0.5,
	Fric_3508_Motor[0].PID.Ki = 0.05; //0.05,
	Fric_3508_Motor[0].PID.Kd = 8;	//1,

	Fric_3508_Motor[1].PID.Kp = 9.1;  //8; //0.5,
	Fric_3508_Motor[1].PID.Ki = 0.05; //0.05,
	Fric_3508_Motor[1].PID.Kd = 8;	//1,

};


/**
  * @brief  Ħ����3508 �ٶȳ�ʼ��
  * @param  void
  * @retval void
  * @note   
  */

void Fric_3508_Motor_Speed_Set(void)
{
	int speed=0;
		if (FricStatus == FricStatus_Stop)
		{
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
  StirMotor.PID.Kp = 8;
  StirMotor.PID.Ki = 0.022;
  StirMotor.PID.Kd = 0;
  StirMotor.TargetSpeed = 0;
}

/**
 * @description: ���̵������
 * @param {void} 
 * @return: void
 * @note: 
 */
uint8_t HeatFlag = 0; //�Ƿ�����
int16_t targetspeed = -4200; //����ת��
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
	//ң��ģʽ
	if(ControlMode != ControlMode_Telecontrol_UP)
	{
		if(StirMotorStatus == StirStatus_SpeedControl)// && HeatFlag==1 )
			StirMotor.TargetSpeed = -4200;
		else
			StirMotor.TargetSpeed = 0;
	}
	//��ת���
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


void Stir_CAN_Send(int16_t Output)
{
  static CanSend_Type CANSend;

  CANSend.CANx = CANSEND_1;

  CANSend.stdid = 0x200;
	
  CANSend.Data[0] = (uint8_t)0;
  CANSend.Data[1] = (uint8_t)0;
	CANSend.Data[2] = (uint8_t)0;
  CANSend.Data[3] = (uint8_t)0;
  CANSend.Data[4] = (uint8_t)0;
  CANSend.Data[5] = (uint8_t)0;
  CANSend.Data[6] = (uint8_t)(Output >> 8);;
  CANSend.Data[7] = (uint8_t)Output;

  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}


void Motor_3508_Send(int16_t Output0,int16_t Output1)
{
  static CanSend_Type CANSend;

  CANSend.CANx = CANSEND_2;

  CANSend.stdid = 0x1ff;

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