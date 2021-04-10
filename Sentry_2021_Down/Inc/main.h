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
#define IMU_HEATING_Pin GPIO_PIN_5
#define IMU_HEATING_GPIO_Port GPIOB
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG
#define RC_TX_Pin GPIO_PIN_7
#define RC_TX_GPIO_Port GPIOB
#define RC_RX_Pin GPIO_PIN_6
#define RC_RX_GPIO_Port GPIOB
#define L1_Measure_Pin GPIO_PIN_9
#define L1_Measure_GPIO_Port GPIOG
#define POWER1_Pin GPIO_PIN_2
#define POWER1_GPIO_Port GPIOH
#define POWER2_Pin GPIO_PIN_3
#define POWER2_GPIO_Port GPIOH
#define POWER3_Pin GPIO_PIN_4
#define POWER3_GPIO_Port GPIOH
#define POWER4_Pin GPIO_PIN_5
#define POWER4_GPIO_Port GPIOH
#define SPI5_NSS_Pin GPIO_PIN_6
#define SPI5_NSS_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
