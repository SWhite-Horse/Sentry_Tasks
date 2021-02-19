#define __TASK_STATUSMACHINE_GLOBALS

#include "Task_StatusMachine.h"
#include "Task_RC.h"
#include "Task_Communication.h"
#include "Task_Gimbal.h"
#include "Task_JetsonComm.h"

			
ControlMode_Enum         ControlMode;
FricStatus_Enum          FricStatus;
StirStatus_Enum          StirMotorStatus;
ShootStatus_Enum         ShootStatus;


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
//uint8_t chassis_servo = 0;
void StatusMachine_Update(void)
{
    static uint8_t FricChange = 1;
    switch (Get_Switch_Val(&RC_ReceiveData, RC_SW_Right))
    {
    case RC_SW_MID:
    {		
        ControlMode =ControlMode_Telecontrol_UP;
				TxMessage.controlmode=ControlMode_Telecontrol_UP;
        break;
    }
    case RC_SW_DOWN:
    {
        ControlMode = ControlMode_Telecontrol_DOWN ;
				TxMessage.controlmode=ControlMode_Telecontrol_DOWN;

        break;
    }
    case RC_SW_UP:
    {		
        ControlMode =ControlMode_Aimbot;
				TxMessage.controlmode=ControlMode_Aimbot;
        break;
    }
    }

		switch (ControlMode)
		{
			
			case ControlMode_Aimbot:  //自瞄模式
			{
//				 if (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_UP)
//						chassis_servo = 1;
//				 else if(Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_MID)
//						chassis_servo = 2;
//				 else
//					  chassis_servo = 3;
				 
				 FricStatus = FricStatus_Working_High;
				 
				 break;	
			}
			
			case ControlMode_Telecontrol_DOWN:  //遥控模式
			{
				if (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_UP)
				{
					if(FricChange)
					{
            if(FricStatus == FricStatus_Stop)
							FricStatus = FricStatus_Working_Low;
            else if(FricStatus == FricStatus_Working_Low)
             FricStatus = FricStatus_Working_High;
            else if(FricStatus == FricStatus_Working_High)
							FricStatus = FricStatus_Stop;
						FricChange = 0;
          }
        }
				else if (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_DOWN)
				{  
          FricChange = 1;
          if(FricStatus != FricStatus_Stop)// &&Rxmessage.Heat<420)
          {
            StirMotorStatus = StirStatus_SpeedControl;
          }
				}
				else if (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_MID)
				{
          FricChange = 1;
          StirMotorStatus = StirStatus_Stop;
				}
				break;
			}		
			
			case ControlMode_Telecontrol_UP:
			{
				//下云台静止
				FricStatus = FricStatus_Stop;
				StirMotorStatus = StirStatus_Stop;
				
				if (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_UP)
				{
					if(FricChange)
					{
            if(TxMessage.fricstatus == FricStatus_Stop)
              TxMessage.fricstatus = FricStatus_Working_Low;
            else if(TxMessage.fricstatus == FricStatus_Working_Low)
              TxMessage.fricstatus = FricStatus_Working_High;
            else if(TxMessage.fricstatus == FricStatus_Working_High)
              TxMessage.fricstatus = FricStatus_Stop;
              FricChange = 0;
          }
				}
				else if (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_DOWN)
				{  
          FricChange = 1;
          if(TxMessage.fricstatus != FricStatus_Stop) //&& RxMessage.Heat<420 )
          {
            TxMessage.stirstatus = StirStatus_SpeedControl;
          }
				}
				else if (Get_Switch_Val(&RC_ReceiveData, RC_SW_Left) == RC_SW_MID)
				{
          FricChange = 1;
          TxMessage.stirstatus = StirStatus_Stop;
				}
				break;
			}				
		}
}
	

