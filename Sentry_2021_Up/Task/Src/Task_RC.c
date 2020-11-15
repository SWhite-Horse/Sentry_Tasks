#define __TASK_RC_GLOBALS
#include "Task_RC.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

/**
 * @description: 遥控接收任务
 * @param {unused} 
 * @return: void
 * @note: 
 */
void Task_RC(void *parameters)
{
	while(1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //等待遙控接受發送信息
    RC_Data_Update();                        //遥控数据更新
	}
}

/**
  * @brief  遥控接收使能
  * @param  UART_HandleTypeDef *huart 遥控对应的串口
  * @retval void
  * @note   
  */
void RC_Receive_Enable(UART_HandleTypeDef *huart)
{
  RC_InitConfig(); //遥控初始化
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  //开启DMA双缓冲
  HAL_DMAEx_MultiBufferStart(huart->hdmarx,(uint32_t)&(huart->Instance->DR),(uint32_t)&RCBuffer[0][0],(uint32_t)&RCBuffer[1][0],(RC_FRAME_LEN+RC_FRAME_LEN_BACK));
  __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE); //使能串口空闲中断
} 

/**
 * @description: 遥控初始化配置
 * @param {type} 
 * @return: void
 * @note: 
 */ 
void RC_InitConfig(void)
{
  RC_ReceiveData.RCFrameCounter = 0; //帧率计数
  RC_Rx_Mem = MEMORYRESET;           //串口缓存区默认值
  RC_ReceiveData.ch0 = RC_CH_VALUE_OFFSET;
  RC_ReceiveData.ch1 = RC_CH_VALUE_OFFSET;
  RC_ReceiveData.ch2 = RC_CH_VALUE_OFFSET;
  RC_ReceiveData.ch3 = RC_CH_VALUE_OFFSET;
	
  RC_ReceiveData.Switch_Left = RC_SW_UP;
  RC_ReceiveData.Switch_Right = RC_SW_UP; 

  RC_ReceiveData.Switch_Left = RC_SW_MID; //初始默认摩擦轮关闭，下云台控制模式
  RC_ReceiveData.Switch_Right = RC_SW_DOWN; 

  LastRC_ReceiveData = RC_ReceiveData; //上次收到的遥控数据
}

/**
 * @description: 遥控串口中断配置
 * @param {UART_HandleTypeDef *huart 遥控对应的串口 串口一} 
 * @return: void
 * @note: 
 */ 
void RC_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static uint8_t this_time_rx_len = 0;
    if (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE) != RESET)
  {
    //** 这两句话的根本是对USART1的两个寄存器做了限制，使得编译器不对此做优化（volatile）
    (void)USART1->SR;
    (void)USART1->DR;
		//清除空闲中断标志位
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    //关闭DMA接收，关闭才能配置等等
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    //计算本次帧长度
    this_time_rx_len = (RC_FRAME_LEN+RC_FRAME_LEN_BACK) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    //记录本次DMA内存
    if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) != RESET)
    {
      /* Current memory buffer used is Memory 1 */
      hdma_usart1_rx.Instance->CR &= (uint32_t)(~DMA_SxCR_CT);
      RC_Rx_Mem = MEMORY1;
    }
    else
    {
      /* Current memory buffer used is Memory 0 */
      hdma_usart1_rx.Instance->CR |= (uint32_t)DMA_SxCR_CT;
      RC_Rx_Mem = MEMORY0;
    }
    //如果本次帧长度与RC帧长度不等，重启遥控接收内存
    if (this_time_rx_len != RC_FRAME_LEN)
      RC_Rx_Mem = MEMORYRESET;
    //设定DMA发送的长度
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, (RC_FRAME_LEN+RC_FRAME_LEN_BACK));
    //重启DMA接收
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

    //发送任务通知给遥控任务
    vTaskNotifyGiveFromISR(TaskHandle_RC,&xHigherPriorityTaskWoken);
    /*强制FreeRTOS任务切换*/
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/**
 * @description: 遥控接收数据更新//双缓存//官方
 * @param {none} 
 * @return: void
 * @note: 
 */
void RC_Data_Update(void)
{
  
  LastRC_ReceiveData = RC_ReceiveData;
  
  if (RC_Rx_Mem == MEMORY0)
  {
    RC_ReceiveData.ch0 = ((RCBuffer[0][0] | (RCBuffer[0][1] << 8)) & 0x07ff);        //!< Channel 0
    RC_ReceiveData.ch1 = (((RCBuffer[0][1] >> 3) | (RCBuffer[0][2] << 5)) & 0x07ff); //!< Channel 1
    RC_ReceiveData.ch2 = (((RCBuffer[0][2] >> 6) | (RCBuffer[0][3] << 2) | (RCBuffer[0][4] << 10)) & 0x07ff);
    RC_ReceiveData.ch3 = (((RCBuffer[0][4] >> 1) | (RCBuffer[0][5] << 7)) & 0x07ff); //!< Channel 3

    RC_ReceiveData.Switch_Left = ((RCBuffer[0][5] >> 4) & 0x000C) >> 2; //!< Switch left
    RC_ReceiveData.Switch_Right = ((RCBuffer[0][5] >> 4) & 0x0003);     //!< Switch right
  }
  else if (RC_Rx_Mem == MEMORY1)
  {
    RC_ReceiveData.ch0 = (RCBuffer[1][0] | (RCBuffer[1][1] << 8)) & 0x07ff;        //!< Channel 0
    RC_ReceiveData.ch1 = ((RCBuffer[1][1] >> 3) | (RCBuffer[1][2] << 5)) & 0x07ff; //!< Channel 1
    RC_ReceiveData.ch2 = ((RCBuffer[1][2] >> 6) | (RCBuffer[1][3] << 2) | (RCBuffer[1][4] << 10)) & 0x07ff;
    RC_ReceiveData.ch3 = ((RCBuffer[1][4] >> 1) | (RCBuffer[1][5] << 7)) & 0x07ff; //!< Channel 3

    RC_ReceiveData.Switch_Left = ((RCBuffer[1][5] >> 4) & 0x000C) >> 2; //!< Switch left
    RC_ReceiveData.Switch_Right = ((RCBuffer[1][5] >> 4) & 0x0003);     //!< Switch right
  }
  else 
  {
    return;
  }
//帧率计数
  RC_ReceiveData.RCFrameCounter++; // 14ms 一次数据
}

/**
  * @brief  获取通道值
  * @param  通道号  RC_CH0 RC_CH1 RC_CH2 RC_CH3 
  * @retval 通道数值  (相对零点)  -660~660
  */
int16_t Get_Channel_Val(RCDecoding_Type *RC_ReceiveData,uint8_t channel_num)
{
  switch (channel_num)
  {
  case RC_CH0:
    if(abs(((*RC_ReceiveData).ch0 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch0 - RC_CH_VALUE_OFFSET);
  case RC_CH1:
    if(abs(((*RC_ReceiveData).ch1 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch1 - RC_CH_VALUE_OFFSET);
  case RC_CH2:
    if(abs(((*RC_ReceiveData).ch2 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch2 - RC_CH_VALUE_OFFSET);
  case RC_CH3:
    if(abs(((*RC_ReceiveData).ch3 - RC_CH_VALUE_OFFSET)) < 10)
      return 0;
    else
      return ((*RC_ReceiveData).ch3 - RC_CH_VALUE_OFFSET);
  default:
    return RC_CH_VALUE_OFFSET;
  }
}

/**
  * @brief  获取开关值
  * @param  RC_SW_Right  RC_SW_Left
  * @retval RC_SW_UP  RC_SW_MID   RC_SW_DOWN
  */
uint8_t Get_Switch_Val(RCDecoding_Type *RC_ReceiveData,uint8_t switch_num)
{
  switch (switch_num)
  {
  case RC_SW_Right:
    return (*RC_ReceiveData).Switch_Right;
  case RC_SW_Left:
    return (*RC_ReceiveData).Switch_Left;
  default:
    return RC_SW_MID;
  }
}


