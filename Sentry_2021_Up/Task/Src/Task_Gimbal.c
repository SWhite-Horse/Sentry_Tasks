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
#include "arm_math.h"


//****** 变量定义区 *******//
MotorType_6020 Pitch,Yaw;
Aimbot_RotatinPatrol_PitchMode  Aimbot_RotatinPatrol_pitchmode ;
Aimbot_RotatinPatrol_YawMode Aimbot_RotatinPatrol_yawmode;
extern SHEN_WEI_struct SHEN_WEI;
int KF_Versioninit = 0;

//卡尔曼DEBUG输出参数
int16_t P_Angle;
int16_t Y_Angle;
int16_t P_DeltaAngle;
int16_t Y_DeltaAngle;
int16_t P_KFoutput;
int16_t Y_KFoutput;


/*********************************
  * @brief  初始化任务
  * @param  unused
  * @retval void
  * @note   建立任务
  */

void Task_Gimbal(void *parameters)
{
	TickType_t xPreviousWakeTime;
	Gimbal_Init(); 
//	Gimal_Calibrate();

	xPreviousWakeTime = xTaskGetTickCount();
	while(1)
	{
		GimbalMotor_AngleSet(&Yaw,&Pitch);
		
		GimbalMotor_PID(&Yaw,&Pitch);
		
		Gimbal_CAN_Yaw_Send(Yaw.NeedCurrent);
		Gimbal_CAN_Pitch_Send(Pitch.NeedCurrent);
		
    vTaskDelayUntil(&xPreviousWakeTime, 3);		
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
	//云台巡逻模式	
	Aimbot_RotatinPatrol_pitchmode=downward;
	Aimbot_RotatinPatrol_yawmode=rightward;
	
	Yaw.SpeedPID.Kp =560;//140;//
	Yaw.SpeedPID.Ki =3.5;//0.4;//
	Yaw.SpeedPID.Kd =5;//7;//
	
	Yaw.PositionPID.Kp = 12;//14;//
	Yaw.PositionPID.Ki =0;//0;// 
	Yaw.PositionPID.Kd = 1;//3;//
	
	Pitch.SpeedPID.Kp =300;//125;//
	Pitch.SpeedPID.Ki = 0;//0.4;//
	Pitch.SpeedPID.Kd = 4;//1;//

	Pitch.PositionPID.Kp = 18;//20;//
	Pitch.PositionPID.Ki = 0.2;//0.15;//
	Pitch.PositionPID.Kd = 1;//1;//
	
	//抬头初始化	
	Pitch.TargetAngle = 20;
		
}
		
/**
  * @brief  云台目标角度设定
  * @param  两电机的结构体
  * @retval None
  * @note   取云台向左下为正
  */

float temp_p[2] = {0, 0}, temp_y[2] = {0, 0};
float lastaimbotyaw=0;
uint16_t RotatinPatrol_Counter=0;
TickType_t Yaw_last_time;

void GimbalMotor_AngleSet(MotorType_6020 *yaw, MotorType_6020 *pitch)
{
	static uint8_t i = 1;
	static uint8_t j = 1;
  
	//记录初始化完毕后Yaw角度
  if (i){
		yaw->TargetAngle = YAW_ANGLE;
    --i;
  }

	//遥控模式
	if (ControlMode == ControlMode_Telecontrol_UP)
		{
		/************** YAW **************/
			// 遥控数据平滑处理
			temp_y[0] = temp_y[1];
			
			if(abs( RxMessage.yaw_speed)>50)  // 50 之下视为微小扰动
				temp_y[1] = 0.5 * RxMessage.yaw_speed / (float)RC_CH_MAX_RELATIVE;  // 0.5 为步长
			else 
				temp_y[1] = 0;
			
			temp_y[1] = SmoothFilter(temp_y[0], temp_y[1]);
			
			// 赋值限制
			yaw->TargetAngle += temp_y[1];	
			while (yaw->TargetAngle >= 180)
					yaw->TargetAngle -= 360;
			while (yaw->TargetAngle < -180)
					yaw->TargetAngle += 360;
				
		/************** PITCH **************/
			temp_p[0] = temp_p[1];
			if(abs(RxMessage.pitch_speed)>50)
				temp_p[1] = 1.0f *RxMessage.pitch_speed / RC_CH_MAX_RELATIVE;  // 1.0 也为步长
			else
				temp_p[1] = 0;
			temp_p[1] = SmoothFilter(temp_p[0], temp_p[1]);

			//赋值限制
			pitch->TargetAngle -= temp_p[1];
			pitch->TargetAngle = pitch->TargetAngle > Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN) ? Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN) : pitch->TargetAngle;
			pitch->TargetAngle = pitch->TargetAngle < Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP) ? Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP) : pitch->TargetAngle;	
		}
			//自瞄模式
	else if(RxMessage.controlmode == ControlMode_Aimbot){
		switch(DataRecFromJetson.SentryGimbalMode){ 
			case RotatinPatrol:{
				// 拨盘电机状态
				StirMotorStatus = StirStatus_Stop;
				if(j) {
					Yaw_last_time = xTaskGetTickCount();
					j--;
				} // 第一次更新当前时间
				// ********************  定步长 自瞄		******************** //
				if(PITCH_ANGLE>=25) Aimbot_RotatinPatrol_pitchmode = upward;
				if(PITCH_ANGLE<=6)	Aimbot_RotatinPatrol_pitchmode = downward;
				if(xTaskGetTickCount()-Yaw_last_time>100){
					if((YAW_ANGLE>80&&YAW_ANGLE<110)||(YAW_ANGLE>-100&&YAW_ANGLE<-80))
					{
						yaw->TargetAngle+=30.0f;					
					}
					else{
						yaw->TargetAngle+=10.0f;				
					}
				
				}
				Yaw_last_time = xTaskGetTickCount();
				if(Aimbot_RotatinPatrol_pitchmode==upward)
				{	
					pitch->TargetAngle-=0.20f;					
				}
			
				if(Aimbot_RotatinPatrol_pitchmode==downward)
				{		
					pitch->TargetAngle+=0.20f;	
				}
					
				while (yaw->TargetAngle >= 180)
					yaw->TargetAngle -= 360;
				while (yaw->TargetAngle < -180)
					yaw->TargetAngle += 360;				
				// ********************  130 自瞄		******************** //

//				//pitch轴巡逻范围
//				if(PITCH_ANGLE>=27) Aimbot_RotatinPatrol_pitchmode = upward;
//				if(PITCH_ANGLE<=8)	Aimbot_RotatinPatrol_pitchmode = downward;
//				if(YAW_ANGLE<-65) Aimbot_RotatinPatrol_yawmode=rightward;
//				if(YAW_ANGLE>85) Aimbot_RotatinPatrol_yawmode=leftward;
//		
//				//pitch轴巡逻方向及步长 
//				if(Aimbot_RotatinPatrol_pitchmode==upward){	
//					pitch->TargetAngle-=0.28f;					
//				}
//				if(Aimbot_RotatinPatrol_pitchmode==downward){		
//					pitch->TargetAngle+=0.28f;							
//				 }
//				//yaw轴巡逻方向及步长 
//				if(Aimbot_RotatinPatrol_yawmode==rightward)
//					yaw->TargetAngle+=0.34f;					
//				if(Aimbot_RotatinPatrol_yawmode==leftward)	
//					yaw->TargetAngle-=0.34f;	

				// ********************  360 自瞄		******************** //
				if(PITCH_ANGLE>=25) Aimbot_RotatinPatrol_pitchmode = upward;
				if(PITCH_ANGLE<=6)	Aimbot_RotatinPatrol_pitchmode = downward;
				
				if((YAW_ANGLE>80&&YAW_ANGLE<110)||(YAW_ANGLE>-100&&YAW_ANGLE<-80))
					yaw->TargetAngle+=0.64f;
				else
					yaw->TargetAngle+=0.14f;
				if(Aimbot_RotatinPatrol_pitchmode==upward)
				{	
					pitch->TargetAngle-=0.20f;					
				}
			
				if(Aimbot_RotatinPatrol_pitchmode==downward)
				{		
					pitch->TargetAngle+=0.20f;	
				}
					
				while (yaw->TargetAngle >= 180)
					yaw->TargetAngle -= 360;
				while (yaw->TargetAngle < -180)
					yaw->TargetAngle += 360;
				
//				// SHEN_WEI 云台控制到目标位置，在 Serve 之前保持这个角度
//				if(!SHEN_WEI.Is_Finished){
//					yaw->TargetAngle = -163;
//					pitch->TargetAngle = 6;
//				}
				
				break;
			}
			
			//伺服模式
			case ServoMode:
			{
//				if(KF_Versioninit == 0){
//					KF_Versioninit = 1;
//					Version_Init();
//				}
//				KF_Cal_Desire();
		
			//瞄准之后角度确立
				yaw->TargetAngle = Yaw_Desire;  //赋值Jeston处理后的数据，在Jeston部分代码里可寻得
					
				while (yaw->TargetAngle >= 180)
					yaw->TargetAngle -= 360;
				while (yaw->TargetAngle < -180)
					yaw->TargetAngle += 360;
				// 上云台不完全转动
				
//				yaw->TargetAngle = yaw->TargetAngle > 65 ? 65 : yaw->TargetAngle;
//				yaw->TargetAngle = yaw->TargetAngle < -65 ? -65 : yaw->TargetAngle;

				pitch->TargetAngle = Pitch_Desire;
					
				pitch->TargetAngle = pitch->TargetAngle > Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN) ? Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_DOWN) : pitch->TargetAngle;
				pitch->TargetAngle = pitch->TargetAngle < Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP) ? Mechanical_PITCHAngle_To_RealAngle(Mechanical_Angle_UP) : pitch->TargetAngle;	
			
				
			//卡尔曼DEBUG输出  ??????
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

				break;
			}	
		}
	}
}

/**
  * @brief  pitch轴重力补偿
  * @param  重力补偿角
  * @retval 
	* @note 当前角与零度的差值乘SIN
  */
float Gravity = -850;
int16_t PitchGravityCompensation(float Angle)
{	
  return (int16_t)(Gravity * sin((Angle-0) / 180 * Pi));
}

/**
  * @brief  平滑遥控读数
  * @param  data_last：前次遥控数据 data：当前遥控数据
  * @retval 平滑后数据
  */
float SmoothFilter(float data_last, float data){
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
    // *********************** PITCH *********************** //
	if (pitch != NULL){
		
		// ************  pitch轴位置环  ************ //
		pitch->PositionPID.Last_Error = pitch->PositionPID.Cur_Error;
		pitch->Real_Angle = PITCH_ANGLE;   // 此变量为机械角定义的宏定义
			
		pitch->PositionPID.Cur_Error = pitch->TargetAngle - pitch->Real_Angle;
		
		pitch->PositionPID.Cur_Error = pitch->PositionPID.Cur_Error > 180 ? pitch->PositionPID.Cur_Error - 360 : pitch->PositionPID.Cur_Error;
		pitch->PositionPID.Cur_Error = pitch->PositionPID.Cur_Error < -180 ? pitch->PositionPID.Cur_Error + 360 : pitch->PositionPID.Cur_Error;
		if(pitch->PositionPID.Cur_Error<0.03f&&pitch->PositionPID.Cur_Error>-0.03f) pitch->PositionPID.Cur_Error = 0;

		if ((pitch->PositionPID.Cur_Error) > -4 && (pitch->PositionPID.Cur_Error) < 4)
			pitch->PositionPID.Sum_Error += pitch->PositionPID.Cur_Error;
		else
			pitch->PositionPID.Sum_Error = 0;

		pitch->PositionPID.Sum_Error = pitch->PositionPID.Sum_Error > 25000 ? 25000 : pitch->PositionPID.Sum_Error;
		pitch->PositionPID.Sum_Error = pitch->PositionPID.Sum_Error < -25000 ? -25000 : pitch->PositionPID.Sum_Error;

		pitch->PositionPID.Output = pitch->PositionPID.Kp * pitch->PositionPID.Cur_Error + pitch->PositionPID.Ki * pitch->PositionPID.Sum_Error + pitch->PositionPID.Kd * (pitch->PositionPID.Cur_Error - pitch->PositionPID.Last_Error);
		
		pitch->PositionPID.Output = pitch->PositionPID.Output > 150 ? 150 : pitch->PositionPID.Output;
		pitch->PositionPID.Output = pitch->PositionPID.Output < -150 ? -150 : pitch->PositionPID.Output;
		pitch->TargetSpeed = pitch->PositionPID.Output;
		
		//************  pitch轴速度环  ************//
    pitch->SpeedPID.Last_Error = pitch->SpeedPID.Cur_Error;
    pitch->SpeedPID.Cur_Error = pitch->TargetSpeed + APITCH;  //速度真实值为 IMU 返回数据，+号根据两值正负确定
		pitch->SpeedPID.Sum_Error += pitch->SpeedPID.Cur_Error;

    pitch->SpeedPID.Sum_Error = pitch->SpeedPID.Sum_Error > 25000 ? 25000 : pitch->SpeedPID.Sum_Error;
    pitch->SpeedPID.Sum_Error = pitch->SpeedPID.Sum_Error < -25000 ? -25000 : pitch->SpeedPID.Sum_Error;

    pitch->SpeedPID.Output = pitch->SpeedPID.Kp * pitch->SpeedPID.Cur_Error + pitch->SpeedPID.Ki * pitch->SpeedPID.Sum_Error + pitch->SpeedPID.Kd * (pitch->SpeedPID.Cur_Error - pitch->SpeedPID.Last_Error) + PitchGravityCompensation(PITCH_ANGLE);
		
		//pitch轴输出限制幅度
    pitch->NeedCurrent = (pitch->SpeedPID.Output > 27000 ? 27000 : pitch->SpeedPID.Output);
    pitch->NeedCurrent = (pitch->SpeedPID.Output < -27000 ? -27000 : pitch->NeedCurrent);
  }
    // ****************** YAW ***************** //
  if (yaw != NULL){
		// ************  yaw轴位置环  ************ //
    yaw->PositionPID.Last_Error = yaw->PositionPID.Cur_Error;
		yaw->Real_Angle=-YAW_ANGLE;  // 此变量，上云台读取的是机械角，而下云台是IMU返回数据

    yaw->PositionPID.Cur_Error = yaw->TargetAngle + yaw->Real_Angle;

    yaw->PositionPID.Cur_Error = yaw->PositionPID.Cur_Error > 180 ? yaw->PositionPID.Cur_Error - 360 : yaw->PositionPID.Cur_Error;
    yaw->PositionPID.Cur_Error = yaw->PositionPID.Cur_Error < -180 ? yaw->PositionPID.Cur_Error + 360 : yaw->PositionPID.Cur_Error;
		if(yaw->PositionPID.Cur_Error<0.02f&&yaw->PositionPID.Cur_Error>-0.02f) yaw->PositionPID.Cur_Error = 0;
    yaw->PositionPID.Sum_Error += yaw->PositionPID.Cur_Error;

    yaw->PositionPID.Sum_Error = yaw->PositionPID.Sum_Error > 10000 ? 10000 : yaw->PositionPID.Sum_Error;
    yaw->PositionPID.Sum_Error = yaw->PositionPID.Sum_Error < -10000 ? -10000 : yaw->PositionPID.Sum_Error;

    yaw->PositionPID.Output = yaw->PositionPID.Kp * yaw->PositionPID.Cur_Error + yaw->PositionPID.Ki * yaw->PositionPID.Sum_Error + yaw->PositionPID.Kd * (yaw->PositionPID.Cur_Error - yaw->PositionPID.Last_Error);

		yaw->PositionPID.Output = yaw->PositionPID.Output > 300 ? 300 : yaw->PositionPID.Output;
		yaw->PositionPID.Output = yaw->PositionPID.Output < -300 ? -300 : yaw->PositionPID.Output;
    yaw->TargetSpeed = yaw->PositionPID.Output;

		// ************  yaw轴速度环  ************ //
		yaw->SpeedPID.Last_Error = yaw->SpeedPID.Cur_Error;
		yaw->SpeedPID.Cur_Error = yaw->TargetSpeed - AYAW;
		yaw->SpeedPID.Sum_Error += yaw->SpeedPID.Cur_Error;
	
		yaw->SpeedPID.Sum_Error = yaw->SpeedPID.Sum_Error > 2000 ? 2000 : yaw->SpeedPID.Sum_Error;
		yaw->SpeedPID.Sum_Error = yaw->SpeedPID.Sum_Error < -2000 ? -2000 : yaw->SpeedPID.Sum_Error;
	
		yaw->SpeedPID.Output = yaw->SpeedPID.Kp * yaw->SpeedPID.Cur_Error + yaw->SpeedPID.Ki * yaw->SpeedPID.Sum_Error + yaw->SpeedPID.Kd * (yaw->SpeedPID.Cur_Error - yaw->SpeedPID.Last_Error);
		//�����������
		yaw->NeedCurrent = (yaw->SpeedPID.Output > YAWCURRENTMAX ? YAWCURRENTMAX : yaw->SpeedPID.Output);
		yaw->NeedCurrent = (yaw->SpeedPID.Output < -YAWCURRENTMAX ? -YAWCURRENTMAX : yaw->NeedCurrent);
  }
	
	if(ControlMode == ControlMode_Telecontrol_DOWN){
		yaw->NeedCurrent=0;
		pitch->NeedCurrent=0;
	}
}


/**
	* @brief  上云台CAN发送
  * @param  pitch（ID 6）,yaw（ID 5）,拨盘电机输出，注意 0x2ff
  * @retval 
* @note 注意两个分开发送是因为CAN总线ID不足，一个是CAN1一个是CAN2
  */

void Gimbal_CAN_Yaw_Send(int16_t Yaw_Output)
{
	CanSend_Type Gimbalsend;
	Gimbalsend.CANx = CANSEND_1;
	Gimbalsend.stdid = 0x2ff;

	Gimbalsend.Data[0] = (uint8_t)(Yaw_Output >> 8);
	Gimbalsend.Data[1] = (uint8_t)Yaw_Output;
	Gimbalsend.Data[2] = (uint8_t)0;
	Gimbalsend.Data[3] = (uint8_t)0;
	Gimbalsend.Data[4] = (uint8_t)0;
	Gimbalsend.Data[5] = (uint8_t)0;
	Gimbalsend.Data[6] = (uint8_t)0;
	Gimbalsend.Data[7] = (uint8_t)0;
	xQueueSend(Queue_CANSend, &Gimbalsend, 3 / portTICK_RATE_MS);
}


void Gimbal_CAN_Pitch_Send(int16_t Pitch_Output)
{
	CanSend_Type Gimbalsend;
	Gimbalsend.CANx = CANSEND_2;
	Gimbalsend.stdid = 0x2ff;

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

