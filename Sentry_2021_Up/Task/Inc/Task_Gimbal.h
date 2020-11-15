#ifndef __TASKGimbalMOTOR_H
#define __TASKGimbalMOTOR_H

#include "Task_init.h"

//角度增加步进值
#define PhysicalAngleAddStep 1.0f
#define Pi 3.1415926f

//电机电调最大电流
#define RM6623CURRENTMAX 6000
#define RM6020CURRENTMAX 28000
//云台电机发送ID
#define GimbalMOTORSENDCANID 0x1FF
//云台电机校准ID
#define RM6623RESETID 0x3F0
//云台电机接收ID
#define RM6020PITCHRECEIVEID 0x205
#define RM6623YAWRECEIVEID 0x206


//6020电机结构体
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

typedef enum
{
	downward,
	upward,
}Aimbot_RotatinPatrol_PitchMode;

typedef enum
{
	leftward,
	rightward,
}Aimbot_RotatinPatrol_YawMode;

typedef struct
{
	uint16_t Last_Pitch_Me_Angle;
	uint16_t Now_Pitch_Me_Angle;
}Diff;


//云台电机机械角上下限位
#define Mechanical_Angle_UP 3980
#define Mechanical_Angle_DOWN 5300
//6369 38.95°
#define PITCHOFFSET 4700
#define YAWOFFSET   7886

#define Mechanical_YAWAngle_To_RealAngle(x) (360 * (x - YAWOFFSET) / 8191.0f)
#define Mechanical_PITCHAngle_To_RealAngle(x) (360 * (x - PITCHOFFSET) / 8191.0f/2.0f)

#define PITCH_ANGLE Mechanical_PITCHAngle_To_RealAngle(Pitch.Mechanical_Angle)//Mechanical_YAWAngle_To_RealAngle PersonalGYRO.RollAngle
//#define YAW_ANGLE PersonalGYRO.YawAngle
//#define APITCH  PersonalGYRO.Gyro_X
//#define AYAW  PersonalGYRO.Gyro_Z

#define YAW_ANGLE imu.yaw
#define AYAW 			imu.wz * 57.3f
#define APITCH 		imu.wx * 57.3f

#define YAWCURRENTMAX RM6020CURRENTMAX
#define PITCHCURRENTMAX RM6623CURRENTMAX

extern Aimbot_RotatinPatrol_PitchMode  Aimbot_RotatinPatrol_pitchmode ;
extern Aimbot_RotatinPatrol_YawMode Aimbot_RotatinPatrol_yawMode;
extern MotorType_6020 Pitch;
extern MotorType_6020 Yaw;

extern int16_t P_Angle;
extern int16_t Y_Angle;

extern int16_t P_DeltaAngle;
extern int16_t Y_DeltaAngle;

extern int16_t P_KFoutput;
extern int16_t Y_KFoutput;
	
void Gimbal_Init(void);
void Gimbal_Ctrl(void);
void Gimbal_jiaozhun(void);
void Gimbal_CAN_Send(int16_t Yaw_Output,int16_t Pitch_Output);
float SmoothFilter(float data_last,float data);
void GimbalMotor_AngleSet(MotorType_6020 *yaw, MotorType_6020 *pitch);
int16_t PitchGravityCompensation(float Angle);
void GimbalMotor_PID(MotorType_6020 *pitch, MotorType_6020 *yaw);

#endif