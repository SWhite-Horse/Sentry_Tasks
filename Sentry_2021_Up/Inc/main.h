/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L1_Tx_Pin GPIO_PIN_1
#define L1_Tx_GPIO_Port GPIOE
#define L1_Rx_Pin GPIO_PIN_0
#define L1_Rx_GPIO_Port GPIOE
#define IMU_HEATING_Pin GPIO_PIN_5
#define IMU_HEATING_GPIO_Port GPIOB
#define Jet_Tx_Pin GPIO_PIN_14
#define Jet_Tx_GPIO_Port GPIOG
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define RC_RX_Pin GPIO_PIN_7
#define RC_RX_GPIO_Port GPIOB
#define Jet_Rx_Pin GPIO_PIN_9
#define Jet_Rx_GPIO_Port GPIOG
#define Power_1_Pin GPIO_PIN_2
#define Power_1_GPIO_Port GPIOH
#define Power_2_Pin GPIO_PIN_3
#define Power_2_GPIO_Port GPIOH
#define Power_3_Pin GPIO_PIN_4
#define Power_3_GPIO_Port GPIOH
#define Power_4_Pin GPIO_PIN_5
#define Power_4_GPIO_Port GPIOH
#define SPI5_NSS_Pin GPIO_PIN_6
#define SPI5_NSS_GPIO_Port GPIOF
#define RIGHT_SW_Pin GPIO_PIN_12
#define RIGHT_SW_GPIO_Port GPIOH
#define LEFT_SW_Pin GPIO_PIN_11
#define LEFT_SW_GPIO_Port GPIOH
#define JUDGE_TX_Pin GPIO_PIN_8
#define JUDGE_TX_GPIO_Port GPIOE
#define Green_LED_Pin GPIO_PIN_14
#define Green_LED_GPIO_Port GPIOF
#define JUDGE_RX_Pin GPIO_PIN_7
#define JUDGE_RX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
