#ifndef __FUNC_JETSONCOMM_H
#define __FUNC_JETSONCOMM_H

#define ARM_MATH_CM4
#include "Task_init.h"
#include <arm_math.h>

#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32
#define mat_inv arm_mat_inverse_f32
#define mat arm_matrix_instance_f32

/*
    镜头右下旋转为正
    镜头右移前进为正
    哨兵向轨道分段标号增加方向运动为正
*/

#define JetsonCommReservedFrameLEN 5

#define JETSONFLAG_LEN 16

//帧头帧尾
#define JetsonCommSOF 0x66
#define JetsonCommEOF 0x88
#define CommSetUp (uint16_t)0xCCCC
#define RecordAngle (uint16_t)0xffff
#define RequestTrans (uint16_t)0xbbbb
//比赛红蓝方
#define BlueTeam (uint8_t)0xDD
#define RedTeam (uint8_t)0xEE
//发射方式
#define NoFire (uint16_t)(0x00 << 8)      //不发射
#define RunningFire (uint16_t)(0x02 << 8) //连发

//哨兵云台工作模式
#define RotatinPatrol (uint8_t)(0x01) //旋转巡逻
#define ServoMode (uint8_t)(0x04)     //伺服打击

typedef struct
{
    uint8_t SoF;
    uint8_t Seq;
    uint16_t ShootMode;     //高八位发射方式 低八位发射速度等级  (0xFFFF-记录当前角度  0xEEEE-红方  0xDDDD-蓝方  0xCCCC-通信建立  0xBBBB-请求数据发送)
    float TargetPitchAngle; //Pitch目标角度
    float TargetYawAngle;   //Yaw目标角度
    /*  哨兵专用   */
    uint8_t SentryGimbalMode;  //哨兵云台攻击模式
		uint8_t reserve[2];			//目标装甲板数字
		uint8_t EoF;
} JetsonToSTM_Struct;

typedef struct
{
    //记录读图时的角度
    float CurAngle_Pitch;
    float CurAngle_Yaw;
    //此次上一次绝对角度
    float Velocity_Pitch;
    float Velocity_Yaw;
    //是否记录过角度
    uint8_t ChangeAngle_flag;
} JetsonFlag_Struct;

typedef struct
{
    //uint8_t SoF;
    //uint8_t Seq;
    float Gimbal_Pitch;
    float Gimbal_Yaw;
		float TeamFlag;
    //uint8_t EoF;
} STMToJetson_Struct_Gyro;



extern JetsonFlag_Struct JetsonFlag[JETSONFLAG_LEN];
extern uint8_t Jetson_Seq;

extern JetsonToSTM_Struct DataRecFromJetson;
extern STMToJetson_Struct_Gyro DataSendToJetson_gyro;
extern float Pitch_Desire, Yaw_Desire;

void JetsonCommUart_Config(UART_HandleTypeDef *huart);
void JetsonCommUart_ReConfig_In_IRQHandler(UART_HandleTypeDef *huart);
void JetsonComm_Control(UART_HandleTypeDef *huart);

extern float Pitch_Desire, Yaw_Desire;
extern float Jetson_AnglePitch;
extern float Jetson_AngleYaw;
extern float Jetson_VelocityPitch;
extern float Jetson_VelocityYaw;

typedef struct
{
    float raw_value;
    float filtered_value[2];
    mat xhat;
    mat xhatminus;
    mat z;
    mat A;
    mat H;
    mat AT;
    mat HT;
    mat Q;
    mat R;
    mat P;
    mat Pminus;
    mat K;
    mat B; //
    mat u; //
} kalman_filter_t;

typedef struct
{
    float raw_value;
    float filtered_value[2];
    float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];
    float B_data[2]; //
    float *u_data;   //
} kalman_filter_init_t;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);

float *amended_kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3);

void KF_Init(void);
void KF_Cal_Desire(void);
void Version_Init(void);
#endif
