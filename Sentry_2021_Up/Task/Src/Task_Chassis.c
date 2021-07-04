#include "Task_Chassis.h"
#include "Task_CAN.h"
#include "Task_RC.h"
#include "Task_Communication.h"
#include "Task_Measure.h"
#include "Task_Shoot.h"
#include "Task_JudgeReceive.h"
#include "Task_JetsonComm.h"
#include "time.h"


uint16_t tim_cnt;
extern uint16_t time_cnt; //�ڲ��ʧЧ֮���˶�ģʽ����ʱ
extern uint8_t get_hurted; //�ж��Ƿ��ܵ��˺�
extern int HeatStatus;
uint8_t get_ =0;
int n;
Motor3508_type Chassis_Motor[2];
Rand_Walk_struct Rand_Walk = {
	.Flag = FLAG_LEFT,
	.Measure_tick = 0,
	.Rand_Numb = 0,
	.Speedup_tick = 0
};
SHEN_WEI_struct SHEN_WEI = {
	.Location = 0,   // 0 ��ʾδ��������࣬1 �򵽴�
	.Location_Flag = 1, // 1 ��ʾ��û�е�һ�δ����������
	.Is_Finished = 1, // 0 ��ʾ��������δ��ɣ� 1 �����
	.Shoot_Count = 3, // ����ִΣ�����ÿ�������������ȷ��
	.Shoot_Flag = 1, // �����־��1 ��ʾ��������������޺���ȴ�����ޣ���ʾһ�η�����ɣ���������
	.Gimbal_Gryo = 0,   // ��̨�Ƿ�ת����λ�� 0 Ϊδ��λ
	.Start_Time = 420,     //��ģʽ����ʱ��
	.End_Time = 390,
	.Bullet_RM = 500
};

YUE_LU_struct YUE_LU = {
	.Status = 0,
};
uint8_t SHEN_WEI_Bullet = 1;

/**************************************************
  * @brief  ��������
  * @param  unused
  * @retval void
  */
void Task_Chassis(void *parameters)
{
	Chassis_Ctrl_Init();   /*���̲�����ʼ������*/     
	TickType_t xLastWakeUpTime=xTaskGetTickCount();
	while(1)
	{	
		// ���ֹ���		
		if(TxMessage.Is_gaming != Debug_status  && (((uint8_t)(ext_event_data.event_type>>10)) & 0x01)){
			YUE_LU.Status = 1;
			SHEN_WEI.Is_Finished = 1;
//			if(ext_game_state.game_progress == 0x04 && ext_game_state.stage_remain_time > SHEN_WEI.End_Time && ext_bullet_remaining.bullet_remaining_num_17mm > 412){ 
//				SHEN_WEI.Is_Finished = 0;
//				YUE_LU.Status = 0;
//			}
//			else{
//				YUE_LU.Status = 1;
//				SHEN_WEI.Is_Finished = 1;
//			}
		}
		else {
			YUE_LU.Status = 0;
			SHEN_WEI.Is_Finished = 1;
		}
		SHEN_WEI.Is_Finished = 1;
    Chassis_Speed_Set(); /*�����ٶȶ�ȡ�趨*/
		Chassis_PID_Ctrl(&Chassis_Motor[0]);
		Chassis_PID_Ctrl(&Chassis_Motor[1]);
	
		if(ext_power_heat_data.chassis_power_buffer != 0) ChassisPowerControl();
		Chassis_CAN_Send(Chassis_Motor[0].Output,Chassis_Motor[1].Output);
		vTaskDelayUntil(&xLastWakeUpTime, 5);
	}
}


/**
  * @brief  ����pid��ʼ��
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
  * @brief  ���̵���ٶȱջ�
  * @param  ���̽ṹ��
  * @retval void
  */

void Chassis_PID_Ctrl(Motor3508_type *chassis)
{
  /**************  Motor  ***************/
  chassis->PID.Last_Error = chassis->PID.Cur_Error; //����PID�ϴ����
  chassis->PID.Cur_Error = chassis->TargetSpeed - chassis->RealSpeed; //����PID��ǰ���
  chassis->PID.Sum_Error += chassis->PID.Cur_Error; //����PID������
  //��������
  chassis->PID.Sum_Error = chassis->PID.Sum_Error > 15000 ? 15000 : chassis->PID.Sum_Error;
  chassis->PID.Sum_Error = chassis->PID.Sum_Error < -15000 ? -15000 : chassis->PID.Sum_Error;

  chassis->Output = (chassis->PID.Kp * chassis->PID.Cur_Error + chassis->PID.Ki * chassis->PID.Sum_Error + chassis->PID.Kd * (chassis->PID.Cur_Error - chassis->PID.Last_Error));
  
  //�����������
  chassis->Output = (chassis->Output >= C620CURRENTMAX) ? C620CURRENTMAX : chassis->Output;
  chassis->Output = (chassis->Output <= -C620CURRENTMAX) ? -C620CURRENTMAX : chassis->Output;
}


/**
  * @brief  ���õ����ƶ��ٶ�
  * @param  None
  * @retval None
  * @note   Ŀǰ�������������ģ�鰲װλ�õķ�����ʻ���ǵ��Ų飩
  */

//*** �����ǹ�翪��ʹ�� ***//
//int Target_speed_auto;
////*** �����Ǻ���ʹ�� ***//
//uint8_t left_singal = 0; //�ж��Լ��Ƿ񵽴������
//float left=0.35;
//uint8_t right_singal = 1; //�ж��Լ��Ƿ񵽴����ұ�
//float right=1.7;
uint16_t abnormaldetection=0;
int16_t LastTarget_speed=0;

void Chassis_Speed_Set(void){
	
	static uint8_t left_detected = 0; //�ж��Լ��Ƿ񵽴������  0 ��ʾδ����
	static uint8_t right_detected = 0; //�ж��Լ��Ƿ񵽴����ұ�

	int16_t Target_speed;
	
	// ���²������ж��ڱ��ڹ���ϵ�λ��
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
	else if(RxMessage.controlmode != ControlMode_Aimbot && RightSwitch==1 && LeftSwitch==1){
		right_detected = 0;
		left_detected = 0;
	}
	
	//��һ�ε����Ҳ�״̬����
	if(!SHEN_WEI.Is_Finished && SHEN_WEI.Location_Flag && right_detected == 1){
		SHEN_WEI.Location = 1;
		SHEN_WEI.Location_Flag = 0;
	} 
	
	
	if(RxMessage.controlmode == ControlMode_Aimbot && Turn_sign && !YUE_LU.Status) {
		Turn_sign = 0;
		get_++;
		right_detected = right_detected ? 0:1 ;
		left_detected = left_detected ? 0:1;
	}
	
  //�ٶȸ�ֵ(����Ϊ��ֵ������Ҫȷ������Ҫ���ݵ����װ��ʽ��PID�����ۺ�ȷ��������෴���˴�ȡ��һ�㼴�ɣ���
	if(RxMessage.controlmode == ControlMode_Aimbot){
		if(RxMessage.speed == 1 || (DataRecFromJetson.SentryGimbalMode == ServoMode))  // ����̨��������̨���ŷ�ģʽʱ
		{			
			if(get_hurted != 3) Target_speed  = CHASSIS_NORMAL_SPEED;
			else Target_speed  = CHASSIS_NORMAL_SPEED;
		}
		else
			Target_speed = CHASSIS_NORMAL_SPEED;
		// ����SHEN_WEI ͣ�������Ҳ�
		if(!SHEN_WEI.Is_Finished && SHEN_WEI.Location) Target_speed = 0;
		if(YUE_LU.Status && left_detected == 1) Target_speed = 0;
		
	}
	else // ������ģʽ��������̨��������
		Target_speed  = RxMessage.speed;
	
//	//�쳣���
//	//���û����ǣ���ʱ����
//	if(right_detected == 1 && left_detected == 1){
//		if(tim_cnt>800){
//			Target_speed = -Target_speed; 
//			tim_cnt=0;
//		}
//		else
//			Target_speed = Target_speed; 
//		
//		if(abs(Target_speed==3000))
//			tim_cnt+=2;
//		else 
//			tim_cnt++;
//	}
		
		//�ж��Ƿ��ܵ��˺�

		
  // ���²��ֶԲ�ͬλ�ã����ݿ���ģʽ��ͬ���ٶȷ���ȷ��
	if(RxMessage.controlmode == ControlMode_Telecontrol_UP||RxMessage.controlmode == ControlMode_Telecontrol_DOWN){ //ң��ģʽ
		//ң��ģʽ��������������ͣ��������������ң����������������
		if(left_detected == 1 && right_detected == 0){ // ��߼�⵽��
			if(Target_speed < 0) Target_speed = 0; 			 //�˾��������ڳ���ʻ������ʱ���Զ�ֹͣ��ң���򷴷��������
			}
		if(right_detected==1 && left_detected == 0 ){  // �ұ߼�⵽��
			if(Target_speed > 0) Target_speed = 0; 		
			}
		}
	else if(RxMessage.controlmode == ControlMode_Aimbot){ // ����ģʽ
		if(left_detected == 1 && right_detected == 0){
			Target_speed = Target_speed;
			if(Rand_Walk.Flag){
				Rand_Walk.Rand_Numb = rand()%10+1;
				Rand_Walk.Measure_tick = 0;
				Rand_Walk.Flag = FLAG_RIGHT;
				Rand_Walk.Speedup_tick = 0;
			}
			
		}
		if(right_detected==1 && left_detected == 0 ){  
			Target_speed = -Target_speed;
			if(!Rand_Walk.Flag){
				Rand_Walk.Rand_Numb = rand()%10+1;
				Rand_Walk.Measure_tick = 0;
				Rand_Walk.Flag = FLAG_LEFT;
				Rand_Walk.Speedup_tick = 0;

			}
		}
		if(SHEN_WEI.Is_Finished && !YUE_LU.Status){
			Rand_Walk.Measure_tick++;
			if(((Rand_Walk.Rand_Numb & 1) || Rand_Walk.Rand_Numb == 2) && Rand_Walk.Speedup_tick < 70){   // 
				if(Rand_Walk.Measure_tick >= MEASURE_CIRCLE * Rand_Walk.Rand_Numb){
					Rand_Walk.Speedup_tick++;
					if(Rand_Walk.Flag) Target_speed = -CHASSIS_HIGH_SPEED;
					else Target_speed = CHASSIS_HIGH_SPEED;
				}
			}
			else if(Rand_Walk.Rand_Numb != 0){
				if(Rand_Walk.Measure_tick == MEASURE_CIRCLE * Rand_Walk.Rand_Numb){
					Rand_Walk.Flag = FLAG_LEFT ? FLAG_RIGHT : FLAG_LEFT;
					right_detected = right_detected ? 0:1 ;
					left_detected = left_detected ? 0:1;
				}
			}
		} 
	}
	// �ܴ������
	if(get_hurted==3){		
		time_cnt++;	
	}
	if(time_cnt>600){  
		get_hurted = 0;
		time_cnt = 0;
	}
//	//�쳣������
//	if(abnormaldetection<12000){
//		if(Target_speed*LastTarget_speed>0){
//			if(StirMotor.Output > 0) abnormaldetection++;
//			else abnormaldetection+=15;
//		}
//		else{
//			LastTarget_speed=Target_speed;
//			abnormaldetection=0;
//		}
//	}
//	else if(ControlMode == ControlMode_Aimbot) Target_speed=0;

	
	//������Ե�����Ŀ��ĸ�ֵ
	Chassis_Motor[0].TargetSpeed = -Target_speed; 
	Chassis_Motor[1].TargetSpeed = Target_speed; 
}


/**
  * @brief  ���̹�������
  * @param  None
  * @retval None
  * @note	  
  */
const float WARNING_ENERGY = 200.0f; //��������
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
  * @brief  �����˶��������ֹͣ��ǰ��ս
  * @param  ���̵��
  * @retval void
  * @note   Output1��Output2�ǵ�������
  */




/**
  * @brief  ���̵�����ݷ���  --CAN1
  * @param  ���̵������
  * @retval void
  * @note   Output1��Output2�ǵ�������
  */
void Chassis_CAN_Send(int16_t Output1,int16_t Output2)
{
  static CanSend_Type CANSend;  //CAN���Ͷ��нṹ��
	
  CANSend.CANx = CANSEND_1; //CAN1����
  CANSend.stdid = 0x200; //���̵��id

  CANSend.Data[0] = (uint8_t)(Output1 >> 8);
  CANSend.Data[1] = (uint8_t)Output1;
  CANSend.Data[2] = (uint8_t)(Output2 >> 8);
  CANSend.Data[3] = (uint8_t)Output1;
  CANSend.Data[4] = 0;
  CANSend.Data[5] = 0;
  CANSend.Data[6] = 0;
  CANSend.Data[7] = 0;

  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS); //����������ѹ�����
}

