#define __TASK_STATUSMACHINE_GLOBALS

#include "Task_StatusMachine.h"
#include "Task_RC.h"
#include "Task_Communication.h"
#include "Task_JetsonComm.h"

			
ControlMode_Enum         ControlMode;
FricStatus_Enum          FricStatus;
StirStatus_Enum          StirMotorStatus;
ShootStatus_Enum         ShootStatus;
//Aimbot_RotatinPatrol_PitchMode  Aimbot_RotatinPatrol_pitchmode ;
//Aimbot_RotatinPatrol_YawMode Aimbot_RotatinPatrol_yawMode;

/**
 * @description: ״̬������
 * @param {none} 
 * @return: void
 * @note: 
 */ 
void Task_StatusMachine(void *parameters)
{
	TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
	StatusMachine_Init();
	while(1)
	{
		 StatusMachine_Update();				
     vTaskDelayUntil(&xLastWakeUpTime, 10);
	}
}
/**
 * @description: ״̬����ʼ��
 * @param {void} 
 * @return: void
 * @note: 
 */
void StatusMachine_Init(void)
{
    //״̬��ʼ��
	  ControlMode = ControlMode_Telecontrol_DOWN;
    FricStatus = FricStatus_Stop;
    StirMotorStatus = StirStatus_Stop;
    ShootStatus = off;
}
/**
 * @description: ״̬������
 * @param {void} 
 * @return: void
 * @note: 	�Ҳ��뿪���ϣ�����ģʽ�����ϣ� �����У� �����£� ��
            �Ҳ��뿪���У�����̨ģʽ�����ϣ�Ħ����״̬�л� �����£����� ��
            �Ҳ��뿪���£�����̨ģʽ�����ϣ� �����У� �����£� ��
 */
uint8_t chassis_servo = 0;
void StatusMachine_Update(void)
{
		switch (ControlMode)
		{			
			case ControlMode_Aimbot:  //����ģʽ
			{ 
				if(DataRecFromJetson.SentryGimbalMode == ServoMode && CommStatus.CommSuccess == 1 ){
					if(TxMessage.mains_power_shooter==0){
					  FricStatus = FricStatus_Stop;
						StirMotorStatus = StirStatus_Stop;
					}
				else
					FricStatus = FricStatus_Working_High;					
				}
				
//				 FricStatus = FricStatus_Working_High;
				 break;	
			}
			
			case ControlMode_Telecontrol_DOWN:  
			{
				FricStatus = FricStatus_Stop;
				StirMotorStatus = StirStatus_Stop;	
				break;
			}		
			
			case ControlMode_Telecontrol_UP:  //ң��ģʽ
			{
         FricStatus = RxMessage.fricstatus;
         StirMotorStatus = RxMessage.stirstatus;      
				break;
			}				
		}
}
	

