/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

uint8_t timer_flag;

uint8_t pb1_pressed;
uint8_t pb2_pressed;

uint8_t can1_recv_flag;

CAN_TxHeaderTypeDef pHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint8_t a, r;  // CAN mailboxes used for tx and rx

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_PWR_RPI_Pin GPIO_PIN_13
#define EN_PWR_RPI_GPIO_Port GPIOC
#define CLK_PWR_RPI_Pin GPIO_PIN_0
#define CLK_PWR_RPI_GPIO_Port GPIOC
#define SHUTDOWN_RPI_Pin GPIO_PIN_1
#define SHUTDOWN_RPI_GPIO_Port GPIOC
#define RUNNING_RPI_Pin GPIO_PIN_2
#define RUNNING_RPI_GPIO_Port GPIOC
#define DETECT_PWR_RPI_Pin GPIO_PIN_3
#define DETECT_PWR_RPI_GPIO_Port GPIOC
#define HS1_Pin GPIO_PIN_4
#define HS1_GPIO_Port GPIOC
#define HS2_Pin GPIO_PIN_5
#define HS2_GPIO_Port GPIOC
#define Volant_Enable24V_Pin GPIO_PIN_10
#define Volant_Enable24V_GPIO_Port GPIOD
#define Volant_Status_Pin GPIO_PIN_11
#define Volant_Status_GPIO_Port GPIOD
#define UART_TXDEn_Pin GPIO_PIN_12
#define UART_TXDEn_GPIO_Port GPIOD
#define FT230_RESET_Pin GPIO_PIN_13
#define FT230_RESET_GPIO_Port GPIOD
#define PB2_Pin GPIO_PIN_14
#define PB2_GPIO_Port GPIOD
#define PB2_EXTI_IRQn EXTI15_10_IRQn
#define PB1_Pin GPIO_PIN_15
#define PB1_GPIO_Port GPIOD
#define PB1_EXTI_IRQn EXTI15_10_IRQn
#define HS3_Pin GPIO_PIN_6
#define HS3_GPIO_Port GPIOC
#define HS4_Pin GPIO_PIN_7
#define HS4_GPIO_Port GPIOC
#define INT_IO1_Pin GPIO_PIN_0
#define INT_IO1_GPIO_Port GPIOD
#define INT_IO2_Pin GPIO_PIN_1
#define INT_IO2_GPIO_Port GPIOD
#define USB_PROG_EN_Pin GPIO_PIN_3
#define USB_PROG_EN_GPIO_Port GPIOD
#define USB_Enable1_Pin GPIO_PIN_4
#define USB_Enable1_GPIO_Port GPIOD
#define USB_Enable2_Pin GPIO_PIN_5
#define USB_Enable2_GPIO_Port GPIOD
#define USB_Enable3_Pin GPIO_PIN_6
#define USB_Enable3_GPIO_Port GPIOD
#define USB_Enable4_Pin GPIO_PIN_7
#define USB_Enable4_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
