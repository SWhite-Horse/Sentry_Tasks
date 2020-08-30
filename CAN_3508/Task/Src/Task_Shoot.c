#include "Task_Shoot.h"
#include "Task_CAN.h"

Motor3508_type Motor_3508[2];

void Task_Shoot(void *parameters)
{
		while(1){
			Motor_3508_Send(600);
		}
}
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
