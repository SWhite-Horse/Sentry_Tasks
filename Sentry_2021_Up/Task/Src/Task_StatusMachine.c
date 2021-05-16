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
 * @description: 状态机任务
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
 * @description: 状态机初始化
 * @param {void} 
 * @return: void
 * @note: 
 */
void StatusMachine_Init(void)
{
    //状态初始化
	  ControlMode = ControlMode_Telecontrol_DOWN;
    FricStatus = FricStatus_Stop;
    StirMotorStatus = StirStatus_Stop;
    ShootStatus = off;
}
/**
 * @description: 状态机更新
 * @param {void} 
 * @return: void
 * @note: 	右拨码开关上：自瞄模式：左上： ，左中： ，左下： 。
            右拨码开关中：上云台模式：左上：摩擦轮状态切换 ，左下：发射 。
            右拨码开关下：下云台模式：左上： ，左中： ，左下： 。
 */
uint8_t chassis_servo = 0;
void StatusMachine_Update(void)
{
		switch (ControlMode)
		{			
			case ControlMode_Aimbot:  //自瞄模式
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
			
			case ControlMode_Telecontrol_UP:  //遥控模式
			{
         FricStatus = RxMessage.fricstatus;
         StirMotorStatus = RxMessage.stirstatus;      
				break;
			}				
		}
}
	

