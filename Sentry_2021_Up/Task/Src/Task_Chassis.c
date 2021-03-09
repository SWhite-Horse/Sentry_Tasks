#include "Task_Chassis.h"
#include "Task_CAN.h"
#include "Task_RC.h"
#include "Task_Communication.h"
#include "Task_Measure.h"
#include "Task_Shoot.h"
#include "Task_JudgeReceive.h"
#include "Task_JetsonComm.h"


uint16_t time_cnt = 0; //判断被打之后加速运动，计时
uint16_t tim_cnt = 0; //在测距失效之后运动模式，计时

Motor3508_type Chassis_Motor[2];

/**************************************************
  * @brief  底盘任务
  * @param  unused
  * @retval void
  */
void Task_Chassis(void *parameters)
{
	Chassis_Ctrl_Init();   /*底盘参数初始化函数*/     
	TickType_t xLastWakeUpTime=xTaskGetTickCount();

	while(1)
	{		
    Chassis_Speed_Set(); /*底盘速度读取设定*/
		Chassis_PID_Ctrl(&Chassis_Motor[0]);
		Chassis_PID_Ctrl(&Chassis_Motor[1]);
	
		ChassisPowerControl();
		Chassis_CAN_Send(Chassis_Motor[0].Output,Chassis_Motor[1].Output);
		vTaskDelayUntil(&xLastWakeUpTime, 5);
	}
}


/**
  * @brief  底盘pid初始化
  * @param  None
  * @retval None
  */
void Chassis_Ctrl_Init(void)
{
  for(int i = 0;i < 2; i++)
  {
    Chassis_Motor[i].PID.Kp = 6; 
    Chassis_Motor[i].PID.Ki = 0.03; 
    Chassis_Motor[i].PID.Kd = 2;   
  }
}



/**
  * @brief  底盘电机速度闭环
  * @param  底盘结构体
  * @retval void
  */

void Chassis_PID_Ctrl(Motor3508_type *chassis)
{
  /**************  Motor  ***************/
  chassis->PID.Last_Error = chassis->PID.Cur_Error; //更新PID上次误差
  chassis->PID.Cur_Error = chassis->TargetSpeed - chassis->RealSpeed; //计算PID当前误差
  chassis->PID.Sum_Error += chassis->PID.Cur_Error; //计算PID误差积分
  //积分上限
  chassis->PID.Sum_Error = chassis->PID.Sum_Error > 15000 ? 15000 : chassis->PID.Sum_Error;
  chassis->PID.Sum_Error = chassis->PID.Sum_Error < -15000 ? -15000 : chassis->PID.Sum_Error;

  chassis->Output = (chassis->PID.Kp * chassis->PID.Cur_Error + chassis->PID.Ki * chassis->PID.Sum_Error + chassis->PID.Kd * (chassis->PID.Cur_Error - chassis->PID.Last_Error));
  
  //限制输出电流
  chassis->Output = (chassis->Output >= C620CURRENTMAX) ? C620CURRENTMAX : chassis->Output;
  chassis->Output = (chassis->Output <= -C620CURRENTMAX) ? -C620CURRENTMAX : chassis->Output;
}


/**
  * @brief  设置底盘移动速度
  * @param  None
  * @retval None
  * @note   目前是正输出向主控模块安装位置的反向行驶（记得排查）
  */

extern uint8_t get_hurted; //判断是否受到伤害
//*** 以下是光电开关使用 ***//
//int Target_speed_auto;
////*** 以下是红外使用 ***//
//uint8_t left_singal = 0; //判断自己是否到达最左边
//float left=0.35;
//uint8_t right_singal = 1; //判断自己是否到达最右边
//float right=1.7;


void Chassis_Speed_Set(void){
	
	static uint8_t left_detected = 0; //判断自己是否到达最左边  0 表示未到达
	static uint8_t right_detected = 0; //判断自己是否到达最右边

	int Target_speed;
	
	// 以下部分是判断哨兵在轨道上的位置
	if(LeftSwitch == 0 && RightSwitch==1){
		left_detected = 1;
		right_detected = 0;
	}
	else if(RightSwitch==0 && LeftSwitch==1){
		right_detected = 1;
		left_detected = 0;
	}
	else if(RightSwitch==0 && LeftSwitch==0){
		right_detected = 1;
		left_detected = 1;
	}
	
  //速度赋值(数据为正值，但是要确定方向要根据电机安装方式和PID正负综合确定，如果相反，此处取反一般即可）：
	if(RxMessage.controlmode == ControlMode_Aimbot){
		if(RxMessage.speed == 1500 || DataRecFromJetson.SentryGimbalMode == ServoMode)  // 下云台或者上云台是伺服模式时
		{
			if(get_hurted) Target_speed  = 2400;
			else Target_speed  = 300;
		}
		else
			Target_speed = 2400;
		
	}
	else // 非自瞄模式就是下云台传输数据
		Target_speed  = RxMessage.speed;
	
	//异常检测
	//作用机理是，定时反向
	if(right_detected == 1 && left_detected == 1){
		if(tim_cnt>800){
			Target_speed = -Target_speed; 
			tim_cnt=0;
		}
		else
			Target_speed = Target_speed; 
		
		if(abs(Target_speed==3000))
			tim_cnt+=2;
		else 
			tim_cnt++;
	}
		
		//判断是否受到伤害
	if(get_hurted){		
		time_cnt++;
	} 
	if(time_cnt>2000){  //10s
		get_hurted = 0;
		time_cnt = 0;
	}
		
  // 以下部分对不同位置，根据控制模式不同做速度方向确定
	if(RxMessage.controlmode == ControlMode_Telecontrol_UP||RxMessage.controlmode == ControlMode_Telecontrol_DOWN){ //遥控模式
		//遥控模式检测仅用于在两边停下来，反向则是遥控输入正负来控制
		if(left_detected == 1 && right_detected == 0){ // 左边检测到柱
			if(Target_speed < 0) Target_speed = 0; 			 //此举意义在于持续驶向柱子时会自动停止，遥控向反方向才有用
			}
		if(right_detected==1 && left_detected == 0 ){  // 右边检测到柱
			if(Target_speed > 0) Target_speed = 0; 		
			}
		}
	else if(RxMessage.controlmode == ControlMode_Aimbot){ // 自瞄模式
		if(left_detected == 1 && right_detected == 0){
			Target_speed = Target_speed;
			}
		if(right_detected==1 && left_detected == 0 ){  
			Target_speed = -Target_speed;
		}
	}
	
	//最后做对电机输出目标的赋值
	Chassis_Motor[0].TargetSpeed = -Target_speed; 
	Chassis_Motor[1].TargetSpeed = Target_speed; 
}

//void Chassis_Speed_Set(void){
//				
////		if(ControlMode == ControlMode_Telecontrol_UP) //遥控模式
////	  {
////			Target_speed =-6000 * Get_Channel_Val(&RC_ReceiveData,RC_CH0) / RC_CH_MAX_RELATIVE;
////		}
////		else if(ControlMode== ControlMode_Aimbot)
////			{
////				if(Distance<=left) {
////					right_singal=0;
////					left_singal=1;
////				}
////				if(Distance>=right) {
////					right_singal=1;
////					left_singal=0;
////				}
////				Target_speed=8000;
////				if(right_singal==0){
////					Target_speed=-Target_speed;
////				}				
////			}
////			Chassis_Motor[0].TargetSpeed = Target_speed; 
////			Chassis_Motor[1].TargetSpeed = -Target_speed; 
////	return;
//			
//	if(RxMessage.controlmode == ControlMode_Telecontrol_UP||RxMessage.controlmode == ControlMode_Telecontrol_DOWN) //遥控模式
//	{	
//		//判断是否到达左边或右边
//		if(LeftSwitch==0)
//		{
//			left_detected = 1;
//			right_detected = 0;
//		}
//		if(RightSwitch==0)
//		{
//			right_detected = 1;
//			left_detected = 0;
//		}
//		
//		Target_speed  = RxMessage.speed;
//		
//		//根据位置确定速度正负
//		if(left_detected==1)
//		{
//			if(Target_speed < 0)
//			{
//				Target_speed = -Target_speed; 
//			}			
//		}
//		if(right_detected==1)
//		{
//			if(Target_speed > 0)
//			{
//				Target_speed = Target_speed; 
//			}			
//		}
//		if(right_detected==1 && left_detected==1)
//		{
//			Target_speed = -Target_speed; 
//		}
//	}
//	else if(RxMessage.controlmode == ControlMode_Aimbot) //自瞄模式
//	{
////		if(Rxmessage.chassis_servo==2)
////		{
//			if(LeftSwitch==0 && RightSwitch==1)
//			{
//				left_detected = 1;
//				right_detected = 0;
//			}
//			else if(RightSwitch==0 && LeftSwitch==1)
//			{
//				right_detected = 1;
//				left_detected = 0;
//			}
//			else if(RightSwitch==0 && LeftSwitch==0)
//			{
//				right_detected = 1;
//				left_detected = 1;
//			}
//			
//			Target_speed_auto = RxMessage.speed;
//			
//			if(left_detected == 1 && right_detected != 1)
//			{
//				Target_speed_auto = Target_speed_auto; 
//			}
//			if(right_detected == 1 && left_detected != 1)
//			{
//				Target_speed_auto = -Target_speed_auto; 		
//			}
//			//异常检测
//			if(right_detected == 1 && left_detected == 1)
//			{
//				if(tim_cnt>800)
//				{
//					Target_speed_auto = -Target_speed_auto; 
//					tim_cnt=0;
//				}
//				else
//				{
//					Target_speed_auto = Target_speed_auto;
//				}		
//				
//				if(RxMessage.speed==8000)
//				{
//					tim_cnt+=2;
//				}
//				else 
//					tim_cnt++;
//			}
////		
//		//判断是否受到伤害
//		if(get_hurted) 
//		{		
//			time_cnt++;
//		} 
//		if(time_cnt>2000)  //10s
//		{
//			get_hurted = 0;
//			time_cnt = 0;
//		}
//		
//		Chassis_Motor[0].TargetSpeed = - Target_speed_auto; 
//		Chassis_Motor[1].TargetSpeed = Target_speed_auto; 
//		
//		return ;
//	}

////	//保护模式
////	else if(RxMessage.controlmode == ControlMode_Telecontrol_DOWN) 
////	{
////		Target_speed = 0; 
////	}
////		

//	Chassis_Motor[0].TargetSpeed = - Target_speed; 
//	Chassis_Motor[1].TargetSpeed = Target_speed; 

//			
//};

/**
  * @brief  底盘功率限制
  * @param  None
  * @retval None
  * @note	  
  */
const float WARNING_ENERGY = 200.0f; //缓冲能量
void ChassisPowerControl(void)
{
	uint16_t total_power_needed = 0;
	float temp = 0.0f;
	total_power_needed = abs(Chassis_Motor[0].Output) + abs(Chassis_Motor[1].Output);
	
	if(ext_power_heat_data.chassis_power_buffer < WARNING_ENERGY)
	{
		temp = (ext_power_heat_data.chassis_power_buffer / WARNING_ENERGY) * (ext_power_heat_data.chassis_power_buffer / WARNING_ENERGY) ;
		Chassis_Motor[0].Output = 1.0f * temp * Chassis_Motor[0].Output;
		Chassis_Motor[1].Output = 1.0f * temp * Chassis_Motor[1].Output;		
	}
}

/**
  * @brief  底盘电机数据发送  --CAN1
  * @param  底盘电机电流
  * @retval void
  * @note   Output1和Output2是底盘轮子
  */
void Chassis_CAN_Send(int16_t Output1,int16_t Output2)
{
  static CanSend_Type CANSend;  //CAN发送队列结构体
	
  CANSend.CANx = CANSEND_1; //CAN1发送
  CANSend.stdid = 0x200; //底盘电机id

  CANSend.Data[0] = (uint8_t)(Output1 >> 8);
  CANSend.Data[1] = (uint8_t)Output1;
  CANSend.Data[2] = (uint8_t)(Output2 >> 8);
  CANSend.Data[3] = (uint8_t)Output1;
  CANSend.Data[4] = 0;
  CANSend.Data[5] = 0;
  CANSend.Data[6] = 0;
  CANSend.Data[7] = 0;

  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS); //将发送数据压入队列
}

