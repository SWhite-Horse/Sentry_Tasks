#define __TASK_FRICMOTOR_GLOBALS
#include "Task_Shoot.h"
#include "Task_CAN.h"

Motor3508_type Motor_3508[2];

void Task_Shoot(void *parameters){
		Motor_3508_PID_Init();
		Motor_3508[0].TargetSpeed=500;
		while(1){
				Motor_3508_PID_Calculate(&Motor_3508[0]);
				Motor_3508_Send(Motor_3508[0].Output);
		}
			
};


void Motor_3508_PID_Init(void){
	
		Motor_3508[0].PID.Kp=0.5;
		Motor_3508[0].PID.Ki=0;
		Motor_3508[0].PID.Kd=0;

};

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

void Motor_3508_Send(int16_t Output)
{
  static CanSend_Type CANSend;

  CANSend.CANx = CANSEND_1;

  CANSend.stdid = 0x200;

			CANSend.Data[0] = (uint8_t)(Output >> 8);
			CANSend.Data[1] = (uint8_t)Output;
			CANSend.Data[2] = (uint8_t)0;
			CANSend.Data[3] = (uint8_t)0;
			CANSend.Data[4] = (uint8_t)0;
			CANSend.Data[5] = (uint8_t)0;
			CANSend.Data[6] = (uint8_t)0;
			CANSend.Data[7] = (uint8_t)0;

  xQueueSend(Queue_CANSend, &CANSend, 3 / portTICK_RATE_MS);
}
