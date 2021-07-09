#define __TASK_GIMBAL_GLOBALS

#include "Task_Gimbal.h"
#include "Task_StatusMachine.h"
#include "Task_RC.h"
#include "Task_CAN.h"
#include "Task_IMU.h"
#include "Func_Imu_OB.h"
#include "Task_Shoot.h"
#include "Task_JetsonComm.h"
#include "Task_Communication.h"
#include "Math.h"

extern uint16_t time1, time2;
//****** 变量定义区 *******//

MotorType_6020 Pitch,Yaw;
Aimbot_RotatinPatrol_PitchMode  Aimbot_RotatinPatrol_pitchmode ;
Aimbot_RotatinPatrol_YawMode Aimbot_RotatinPatrol_yawmode;

int KF_Versioninit = 0;

//卡尔曼DEBUG输出参数
int16_t P_Angle;
int16_t Y_Angle;
int16_t P_DeltaAngle;
int16_t Y_DeltaAngle;
int16_t P_KFoutput;
int16_t Y_KFoutput;

uint16_t target=0;


/*********************************
  * @brief  初始化任务
  * @param  unused
  * @retval void
  * @note   建立任务
  */

void Task_Gimbal(void *parameters)
{
	TickType_t xPreviousWakeTime;
	//vTaskDelay(2000);
	Gimbal_Init(); 
//	Gimal_Calibrate();
	//云台巡逻模式
	Aimbot_RotatinPatrol_pitchmode=downward;
	Aimbot_RotatinPatrol_yawmode=rightward;
	
	xPreviousWakeTime = xTaskGetTickCount();
	while(1)
	{
		GimbalMotor_AngleSet(&Yaw,&Pitch);
		GimbalMotor_PID(&Yaw,&Pitch);
		Gimbal_CAN_Send(Yaw.NeedCurrent,StirMotor.Output);
		Gimbal_CAN_Pitch_Send(Pitch.NeedCurrent);
    vTaskDelayUntil(&xPreviousWakeTime, 2);		
	}
}


/***************************
  * @brief  云台上电初始化
  * @param  void
  * @retval None
  * @note   此函数调用在控制任务while（1）进入之前  
  */

void Gimbal_Init(void)
{
	
	Yaw.SpeedPID.Kp =85;//60;//
	Yaw.SpeedPID.Ki = 0;//0.3;//
	Yaw.SpeedPID.Kd = 8;//12;//
	
	Yaw.PositionPID.Kp =12;//20;//
	Yaw.PositionPID.Ki =0;//0;//
	Yaw.PositionPID.Kd =3;//4;//
	
	Pitch.SpeedPID.Kp =100;//170;//
	Pitch.SpeedPID.Ki =0.1;//0.5;//
	Pitch.SpeedPID.Kd =0;//5;//

	Pitch.PositionPID.Kp =12;//18;//
	Pitch.PositionPID.Ki =0.1;//0.1;//
	Pitch.PositionPID.Kd =0;//3.5;//
	
	//抬头初始化	
	Pitch.TargetAngle =31;///20;
		
}
		
/**
  * @brief  云台目标角度设定
  * @param  两电机的结构体
  * @retval None
  * @note   取云台向左下为正
  */

float temp_p[2] = {0, 0}, temp_y[2] = {0, 0};
//float Aimbot_temp_p[2] = {0, 0}, Aimbot_temp_y[2] = {0, 0};

//uint16_t RotatinPatrol_Counter=0;
void GimbalMotor_AngleSet(MotorType_6020 *yaw, MotorType_6020 *pitch)
{
  static uint8_t i = 1;
	static uint8_t j = 0;
    
	//记录初始化完毕后Yaw角度
  if (i){
		yaw->TargetAngle = YAW_ANGLE;
    --i;
  }

	//遥控模式
	if(ControlMode == ControlMode_Telecontrol_DOWN){
		/************** YAW **************/
		temp_y[0] = temp_y[1];
		if(abs(Get_Channel_Val(&RC_ReceiveData, RC_CH2))>50)	
			temp_y[1] = 0.5 * Get_Channel_Val(&RC_ReceiveData, RC_CH2) / RC_CH_MAX_RELATIVE;
		else 
			temp_y[1] = 0;
		temp_y[1] = SmoothFilter(temp_y[0], temp_y[1]);
		
    yaw->TargetAngle += temp_y[1];	
    while (yaw->TargetAngle >= 360)
        yaw->TargetAngle -= 360;
    while (yaw->TargetAngle < 0)
        yaw->TargetAngle += 360;
    
	/************** PITCH **************/
    temp_p[0] = temp_p[1];
		if(abs(Get_Channel_Val(&RC_ReceiveData, RC_CH3))>50)
			temp_p[1] =  PhysicalAngleAddStep * Get_Channel_Val(&RC_ReceiveData, RC_CH3) / RC_CH_MAX_RELATIVE;
    else
			temp_p[1] = 0;
		temp_p[1] = SmoothFilter(temp_p[0], temp_p[1]);
				
		pitch->TargetAngle -= temp_p[1];
		pitch->TargetAngle = pitch->TargetAngle > Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN) ? Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN) : pitch->TargetAngle;
		pitch->TargetAngle = pitch->TargetAngle < 2.0f ? 2.0f : pitch->TargetAngle; //Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP) ? Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP) : pitch->TargetAngle;	
	}
			//自瞄模式
	else if(ControlMode == ControlMode_Aimbot){
		switch(DataRecFromJetson.SentryGimbalMode){ 
			case RotatinPatrol:
			{				
				StirMotorStatus = StirStatus_Stop;
				//pitch轴巡逻范围
				if(PITCH_ANGLE>=33)   Aimbot_RotatinPatrol_pitchmode = upward;
				if(PITCH_ANGLE<=5)	 Aimbot_RotatinPatrol_pitchmode = downward;
				
				//巡逻步进设置，注意下云台没有Yaw的左右之分，上云台有	 
				if(Aimbot_RotatinPatrol_pitchmode==upward)
				{	
					yaw->TargetAngle+=0.36f;
					pitch->TargetAngle-=0.38f;					
					++j;
				}
			
				if(Aimbot_RotatinPatrol_pitchmode==downward)
				{		
					yaw->TargetAngle+=0.36f;
					pitch->TargetAngle+=0.38f;	
					++j;
				}
				
				while(yaw->TargetAngle > 360)
					yaw->TargetAngle -= 360;
				while(yaw->TargetAngle < 0)
					yaw->TargetAngle += 360;			
				break;
			}
			
			//伺服模式
			case ServoMode:
			{
				if(KF_Versioninit == 0){
					KF_Versioninit = 1;
					Version_Init();
				}

//				   if(DataRecFromJetson.TargetYawAngle != 255 && DataRecFromJetson.TargetYawAngle != -255)
//				   {
//				   	KF_Cal_Desire();
//				   }		
//				
					
				 //瞄准之后角度确立
				 yaw->TargetAngle = -Yaw_Desire; //要特别注意这里的正负和电机正方向以及算法回调数据（Jeston里面Yaw_Desire的赋值语句）的关系，切记
				 while(yaw->TargetAngle > 360)
					yaw->TargetAngle -= 360;
				 while(yaw->TargetAngle < 0)
					yaw->TargetAngle += 360;
				 
				 pitch->TargetAngle = Pitch_Desire;//这个也要注意
				 
				 pitch->TargetAngle = pitch->TargetAngle > Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN) ? Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN) : pitch->TargetAngle;
				 pitch->TargetAngle = pitch->TargetAngle < Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP) ? Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP) : pitch->TargetAngle;	
						
				//卡尔曼DEBUG输出
				 P_KFoutput = (int16_t)(Pitch_Desire * 10);
				 Y_KFoutput = (int16_t)((Yaw_Desire) * 10);
				 
				 if (PITCH_ANGLE > 180)
					P_Angle = (int16_t)(PITCH_ANGLE - 360) * 10;
				 else
					P_Angle = (int16_t)((PITCH_ANGLE)*10);
				 
				 if (YAW_ANGLE > 180)
					Y_Angle = (int16_t)(YAW_ANGLE - 360) * 10;
				 else
					Y_Angle = (int16_t)((YAW_ANGLE)*10);
				 
				 P_DeltaAngle = (int16_t)(Jetson_AnglePitch * 10);
				 Y_DeltaAngle = (int16_t)(Jetson_AngleYaw * 10);
					/*
					
					if((DataRecFromJetson.ShootMode >> 8) == (RunningFire >> 8))
					{
						StirMotorStatus = StirStatus_SpeedControl;
						ShootStatus = on;
					}
					else
					{
						StirMotorStatus = StirStatus_Stop;
						ShootStatus = off;
					}
					*/
				break;
			}
		}
	}
}

/**
  * @brief  pitch轴重力补偿
  * @param  重力补偿角
  * @retval 
  */
float Gravity = -8000;
int16_t PitchGravityCompensation(float Angle)
{
    return (int16_t)(Gravity * sin((Angle-0.0f) / 180 * Pi));
}

/**
  * @brief  平滑遥控读数
  * @param  data_last：前次遥控数据 data：当前遥控数据
  * @retval 平滑后数据
  */
float SmoothFilter(float data_last, float data)
{
    static float alpha = 0.3;
    float temp;

    temp = alpha * data_last + (1 - alpha) * data;
    return temp;
}

/**************************** PID双环****************************/

/**
  * @brief  PID双环控制
  * @param  pitch轴结构体，yaw轴结构体
  * @retval 
  */
void GimbalMotor_PID(MotorType_6020 *yaw, MotorType_6020 *pitch)
{
    /************** PITCH **************/
	if (pitch != NULL){
		//pitch轴位置环

		pitch->PositionPID.Last_Error = pitch->PositionPID.Cur_Error;
		pitch->Real_Angle=PITCH_ANGLE;
	  
    pitch->PositionPID.Cur_Error = pitch->TargetAngle - pitch->Real_Angle;
		
    pitch->PositionPID.Cur_Error = pitch->PositionPID.Cur_Error > 180 ? pitch->PositionPID.Cur_Error - 360 : pitch->PositionPID.Cur_Error;
    pitch->PositionPID.Cur_Error = pitch->PositionPID.Cur_Error < -180 ? pitch->PositionPID.Cur_Error + 360 : pitch->PositionPID.Cur_Error;

    if ((pitch->PositionPID.Cur_Error) > -6 && (pitch->PositionPID.Cur_Error) < 6)
        pitch->PositionPID.Sum_Error += pitch->PositionPID.Cur_Error;
    else
        pitch->PositionPID.Sum_Error = 0;

    pitch->PositionPID.Sum_Error = pitch->PositionPID.Sum_Error > 25000 ? 25000 : pitch->PositionPID.Sum_Error;
    pitch->PositionPID.Sum_Error = pitch->PositionPID.Sum_Error < -25000 ? -5000 : pitch->PositionPID.Sum_Error;

    pitch->PositionPID.Output = pitch->PositionPID.Kp * pitch->PositionPID.Cur_Error + pitch->PositionPID.Ki * pitch->PositionPID.Sum_Error + pitch->PositionPID.Kd * (pitch->PositionPID.Cur_Error - pitch->PositionPID.Last_Error);
		
//    pitch->PositionPID.Output = pitch->PositionPID.Output > 150 ? 150 : pitch->PositionPID.Output;
//		pitch->PositionPID.Output = pitch->PositionPID.Output < -150 ? -150 : pitch->PositionPID.Output;
    pitch->TargetSpeed =pitch->PositionPID.Output;
		
		
	
		//pitch轴速度环
    pitch->SpeedPID.Last_Error = pitch->SpeedPID.Cur_Error;
    pitch->SpeedPID.Cur_Error = pitch->TargetSpeed - APITCH;//pitch->Real_Speed;//+ pitch->Real_Speed;//pitch->Torque_Current_Real;//
		

		pitch->SpeedPID.Sum_Error += pitch->SpeedPID.Cur_Error;

    pitch->SpeedPID.Sum_Error = pitch->SpeedPID.Sum_Error > 25000 ? 25000 : pitch->SpeedPID.Sum_Error;
    pitch->SpeedPID.Sum_Error = pitch->SpeedPID.Sum_Error < -25000 ? -25000 : pitch->SpeedPID.Sum_Error;

    pitch->SpeedPID.Output = pitch->SpeedPID.Kp * pitch->SpeedPID.Cur_Error + pitch->SpeedPID.Ki * pitch->SpeedPID.Sum_Error + pitch->SpeedPID.Kd * (pitch->SpeedPID.Cur_Error - pitch->SpeedPID.Last_Error) + PitchGravityCompensation(PITCH_ANGLE);
		
	
		//pitch轴输出限制幅度
    pitch->NeedCurrent = (pitch->SpeedPID.Output > 28000 ? 28000 : pitch->SpeedPID.Output);
    pitch->NeedCurrent = (pitch->SpeedPID.Output < -28000 ? -28000 : pitch->NeedCurrent);
		
		
		/*		//pitch轴输出限制幅度
    pitch->NeedCurrent = (pitch->PositionPID.Output > 25000 ? 25000 : pitch->PositionPID.Output);
    pitch->NeedCurrent = (pitch->PositionPID.Output < -25000 ? -25000 : pitch->NeedCurrent);
		*/
	}
    /************** YAW **************/
	if (yaw != NULL){
		//yaw轴位置环
    yaw->PositionPID.Last_Error = yaw->PositionPID.Cur_Error;
		yaw->Real_Angle=YAW_ANGLE;
////修正realangle
//		if(YAW_ANGLE>0)
//		Correct_Y_Angle=360-YAW_ANGLE;
//		else Correct_Y_Angle=-YAW_ANGLE;
//
		
    yaw->PositionPID.Cur_Error = yaw->TargetAngle + YAW_ANGLE;

    yaw->PositionPID.Cur_Error = yaw->PositionPID.Cur_Error > 180 ? yaw->PositionPID.Cur_Error - 360 : yaw->PositionPID.Cur_Error;
    yaw->PositionPID.Cur_Error = yaw->PositionPID.Cur_Error < -180 ? yaw->PositionPID.Cur_Error + 360 : yaw->PositionPID.Cur_Error;

		if(abs(yaw->PositionPID.Cur_Error) < 15)
			yaw->PositionPID.Sum_Error += yaw->PositionPID.Cur_Error;
		else
			yaw->PositionPID.Sum_Error = 0;

    yaw->PositionPID.Sum_Error = yaw->PositionPID.Sum_Error > 5000 ? 5000 : yaw->PositionPID.Sum_Error;
    yaw->PositionPID.Sum_Error = yaw->PositionPID.Sum_Error < -5000 ? -5000 : yaw->PositionPID.Sum_Error;

    yaw->PositionPID.Output = yaw->PositionPID.Kp * yaw->PositionPID.Cur_Error + yaw->PositionPID.Ki * yaw->PositionPID.Sum_Error + yaw->PositionPID.Kd * (yaw->PositionPID.Cur_Error - yaw->PositionPID.Last_Error);

//		yaw->PositionPID.Output = yaw->PositionPID.Output > 300 ? 300 : yaw->PositionPID.Output;
//		yaw->PositionPID.Output = yaw->PositionPID.Output < -300 ? -300 : yaw->PositionPID.Output;
    yaw->TargetSpeed = yaw->PositionPID.Output;
        
		//yaw轴速度环
    yaw->SpeedPID.Last_Error = yaw->SpeedPID.Cur_Error;
    yaw->SpeedPID.Cur_Error = yaw->TargetSpeed - AYAW;
    yaw->SpeedPID.Sum_Error += yaw->SpeedPID.Cur_Error;

    yaw->SpeedPID.Sum_Error = yaw->SpeedPID.Sum_Error > 2000 ? 2000 : yaw->SpeedPID.Sum_Error;
    yaw->SpeedPID.Sum_Error = yaw->SpeedPID.Sum_Error < -2000 ? -2000 : yaw->SpeedPID.Sum_Error;

    yaw->SpeedPID.Output = yaw->SpeedPID.Kp * yaw->SpeedPID.Cur_Error + yaw->SpeedPID.Ki * yaw->SpeedPID.Sum_Error + yaw->SpeedPID.Kd * (yaw->SpeedPID.Cur_Error - yaw->SpeedPID.Last_Error);
    //输出赋值
    yaw->NeedCurrent = (yaw->SpeedPID.Output > YAWCURRENTMAX ? YAWCURRENTMAX : yaw->SpeedPID.Output);
    yaw->NeedCurrent = (yaw->SpeedPID.Output < -YAWCURRENTMAX ? -YAWCURRENTMAX : yaw->NeedCurrent);
	}
	if(ControlMode == ControlMode_Telecontrol_UP){
		yaw->NeedCurrent=0;
	  pitch->NeedCurrent=0;
	}
}


/**
  * @brief  云台CAN发送
  * @param  pitch,yaw,拨盘电机输出
  * @retval
	* @note 下云台拨盘电机也何云台电机一起发送，原因是CAN ID的上限，以及如果将 此下两个电机分开发送，则会导致互相赋值为0的情况出现
  */
void Gimbal_CAN_Send(int16_t Yaw_Output,int16_t Stir_Output)
{
	CanSend_Type Gimbalsend;
	Gimbalsend.CANx = CANSEND_1;
	Gimbalsend.stdid = 0x1ff;
	Gimbalsend.Data[0] = (uint8_t)(Yaw_Output >> 8);
	Gimbalsend.Data[1] = (uint8_t)Yaw_Output;
	Gimbalsend.Data[2] = (uint8_t)0;
	Gimbalsend.Data[3] = (uint8_t)0;
	Gimbalsend.Data[4] = (uint8_t)(Stir_Output >> 8);
	Gimbalsend.Data[5] = (uint8_t)Stir_Output;
	Gimbalsend.Data[6] = (uint8_t)0;
	Gimbalsend.Data[7] = (uint8_t)0;

	xQueueSend(Queue_CANSend, &Gimbalsend, 3 / portTICK_RATE_MS);
}

void Gimbal_CAN_Pitch_Send(int16_t Pitch_Output)
{
	CanSend_Type Gimbalsend;
	Gimbalsend.CANx = CANSEND_2;
	Gimbalsend.stdid = 0x1ff;
	Gimbalsend.Data[0] = (uint8_t)0;
	Gimbalsend.Data[1] = (uint8_t)0;
	Gimbalsend.Data[2] = (uint8_t)(Pitch_Output >> 8);
	Gimbalsend.Data[3] = (uint8_t)Pitch_Output;
	Gimbalsend.Data[4] = (uint8_t)0;
	Gimbalsend.Data[5] = (uint8_t)0;
	Gimbalsend.Data[6] = (uint8_t)0;
	Gimbalsend.Data[7] = (uint8_t)0;

	xQueueSend(Queue_CANSend, &Gimbalsend, 3 / portTICK_RATE_MS);
}
