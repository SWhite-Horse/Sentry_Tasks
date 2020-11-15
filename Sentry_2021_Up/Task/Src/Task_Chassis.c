#include "Task_Chassis.h"
#include "Task_CAN.h"
#include "Task_RC.h"
#include "Task_Communication.h"
#include "Task_Measure.h"
#include "Task_Shoot.h"
#include "Task_JudgeReceive.h"

uint16_t time_cnt = 0; //�жϱ���֮������˶�����ʱ
uint16_t tim_cnt = 0; //�ڲ��ʧЧ֮���˶�ģʽ����ʱ

Motor3508_type Chassis_Motor[2];

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
    Chassis_Speed_Set(); /*�����ٶȶ�ȡ�趨*/
		Chassis_PID_Ctrl(&Chassis_Motor[0]);
		Chassis_PID_Ctrl(&Chassis_Motor[1]);
	
//		ChassisPowerControl();
		Chassis_CAN_Send(Chassis_Motor[0].Output,Chassis_Motor[1].Output);
		vTaskDelayUntil(&xLastWakeUpTime, 5);
	}
}

/**��
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
  * @brief  ����pid��ʼ��
  * @param  None
  * @retval None
  */
void Chassis_Ctrl_Init(void)
{
    for(int i = 0;i < 2; i++)
    {
      Chassis_Motor[i].PID.Kp = 6; 
      Chassis_Motor[i].PID.Ki = 0;//0.03; 
      Chassis_Motor[i].PID.Kd = 0;//2;   
    }
}











/**
  * @brief  ���õ����ƶ��ٶ�
  * @param  None
  * @retval None
  * @note   
  */


uint8_t left_singal = 0; //�ж��Լ��Ƿ񵽴������
float left=0.35;
uint8_t right_singal = 1; //�ж��Լ��Ƿ񵽴����ұ�
float right=1.7;
int Target_speed;

void Chassis_Speed_Set(void){
				
		if(ControlMode == ControlMode_Telecontrol_UP) //ң��ģʽ
	  {
			Target_speed =-6000 * Get_Channel_Val(&RC_ReceiveData,RC_CH0) / RC_CH_MAX_RELATIVE;
		}
		else if(ControlMode== ControlMode_Aimbot)
			{
				if(Distance<=left) {
					right_singal=0;
					left_singal=1;
				}
				if(Distance>=right) {
					right_singal=1;
					left_singal=0;
				}
				Target_speed=8000;
				if(right_singal==0){
					Target_speed=-Target_speed;
				}				
			}
			Chassis_Motor[0].TargetSpeed = Target_speed; 
			Chassis_Motor[1].TargetSpeed = -Target_speed; 
	return;
};

///**
//  * @brief  ���̹�������
//  * @param  None
//  * @retval None
//  * @note	  
//  */
//const float WARNING_ENERGY = 200.0f; //��������
//void ChassisPowerControl(void)
//{
//	uint16_t total_power_needed = 0;
//	float temp = 0.0f;
//	total_power_needed = abs(Chassis_Motor[0].Output) + abs(Chassis_Motor[1].Output);
//	
//	if(ext_power_heat_data.chassis_power_buffer < WARNING_ENERGY)
//	{
//		temp = (ext_power_heat_data.chassis_power_buffer / WARNING_ENERGY) * (ext_power_heat_data.chassis_power_buffer / WARNING_ENERGY) ;
//		Chassis_Motor[0].Output = 1.0f * temp * Chassis_Motor[0].Output;
//		Chassis_Motor[1].Output = 1.0f * temp * Chassis_Motor[1].Output;		
//	}
//}

/**
  * @brief  ���̵�����ݷ���  --CAN1
  * @param  ���̵������
  * @retval void
  * @note   Output1��Output2�ǵ������ӣ�3��4��Ħ�������
  */
void Chassis_CAN_Send(int16_t Output1,int16_t Output2)
{
  static CanSend_Type CANSend;  //CAN���Ͷ��нṹ��


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

