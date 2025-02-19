/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef __packed struct 
{
  uint8_t header;
  uint16_t luminance[128];
  uint32_t checksum;
} VLP_packet_t;

typedef __packed struct 
{
  uint8_t header;
  uint8_t data[256];
  uint8_t syn;
  uint32_t checksum;
} comm_packet_t;

typedef struct{
	uint16_t adc_values[8];
	uint8_t adc_done[5];
	uint16_t VLP_value[128];
} checker_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc5;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc4;
extern DMA_HandleTypeDef hdma_adc5;
extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern VLP_packet_t VLP_packet;
extern comm_packet_t comm_packet;

extern uint8_t adc_done;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEL_0_96_111_Pin GPIO_PIN_13
#define SEL_0_96_111_GPIO_Port GPIOC
#define SEL_2_0_15_Pin GPIO_PIN_0
#define SEL_2_0_15_GPIO_Port GPIOC
#define SEL_3_0_15_Pin GPIO_PIN_1
#define SEL_3_0_15_GPIO_Port GPIOC
#define SEL_0_0_15_Pin GPIO_PIN_2
#define SEL_0_0_15_GPIO_Port GPIOC
#define SEL_1_0_15_Pin GPIO_PIN_3
#define SEL_1_0_15_GPIO_Port GPIOC
#define ADC_0_15_Pin GPIO_PIN_0
#define ADC_0_15_GPIO_Port GPIOA
#define ADC_48_63_Pin GPIO_PIN_1
#define ADC_48_63_GPIO_Port GPIOA
#define ADC_96_111_Pin GPIO_PIN_2
#define ADC_96_111_GPIO_Port GPIOA
#define ADC_16_31_Pin GPIO_PIN_3
#define ADC_16_31_GPIO_Port GPIOA
#define SEL_3_16_31_Pin GPIO_PIN_4
#define SEL_3_16_31_GPIO_Port GPIOA
#define SEL_2_16_31_Pin GPIO_PIN_5
#define SEL_2_16_31_GPIO_Port GPIOA
#define SEL_1_16_31_Pin GPIO_PIN_6
#define SEL_1_16_31_GPIO_Port GPIOA
#define SEL_0_16_31_Pin GPIO_PIN_7
#define SEL_0_16_31_GPIO_Port GPIOA
#define SEL_2_48_63_Pin GPIO_PIN_4
#define SEL_2_48_63_GPIO_Port GPIOC
#define SEL_3_48_63_Pin GPIO_PIN_5
#define SEL_3_48_63_GPIO_Port GPIOC
#define SEL_0_48_63_Pin GPIO_PIN_0
#define SEL_0_48_63_GPIO_Port GPIOB
#define SEL_1_48_63_Pin GPIO_PIN_1
#define SEL_1_48_63_GPIO_Port GPIOB
#define SEL_1_96_111_Pin GPIO_PIN_2
#define SEL_1_96_111_GPIO_Port GPIOB
#define SEL_2_96_111_Pin GPIO_PIN_10
#define SEL_2_96_111_GPIO_Port GPIOB
#define ADC_32_47_Pin GPIO_PIN_11
#define ADC_32_47_GPIO_Port GPIOB
#define ADC_64_79_Pin GPIO_PIN_12
#define ADC_64_79_GPIO_Port GPIOB
#define ADC_112_127_Pin GPIO_PIN_13
#define ADC_112_127_GPIO_Port GPIOB
#define SEL_3_64_79_Pin GPIO_PIN_14
#define SEL_3_64_79_GPIO_Port GPIOB
#define SEL_2_64_79_Pin GPIO_PIN_15
#define SEL_2_64_79_GPIO_Port GPIOB
#define SEL_1_64_79_Pin GPIO_PIN_6
#define SEL_1_64_79_GPIO_Port GPIOC
#define SEL_0_64_79_Pin GPIO_PIN_7
#define SEL_0_64_79_GPIO_Port GPIOC
#define SEL_2_112_127_Pin GPIO_PIN_8
#define SEL_2_112_127_GPIO_Port GPIOC
#define SEL_3_112_127_Pin GPIO_PIN_9
#define SEL_3_112_127_GPIO_Port GPIOC
#define ADC_80_95_Pin GPIO_PIN_8
#define ADC_80_95_GPIO_Port GPIOA
#define SEL_0_112_127_Pin GPIO_PIN_15
#define SEL_0_112_127_GPIO_Port GPIOA
#define SEL_1_112_127_Pin GPIO_PIN_10
#define SEL_1_112_127_GPIO_Port GPIOC
#define SEL_3_80_95_Pin GPIO_PIN_11
#define SEL_3_80_95_GPIO_Port GPIOC
#define SEL_2_80_95_Pin GPIO_PIN_12
#define SEL_2_80_95_GPIO_Port GPIOC
#define SEL_0_80_95_Pin GPIO_PIN_2
#define SEL_0_80_95_GPIO_Port GPIOD
#define SEL_1_80_95_Pin GPIO_PIN_3
#define SEL_1_80_95_GPIO_Port GPIOB
#define SEL_2_32_47_Pin GPIO_PIN_4
#define SEL_2_32_47_GPIO_Port GPIOB
#define SEL_3_32_47_Pin GPIO_PIN_5
#define SEL_3_32_47_GPIO_Port GPIOB
#define SEL_0_32_47_Pin GPIO_PIN_6
#define SEL_0_32_47_GPIO_Port GPIOB
#define SEL_1_32_47_Pin GPIO_PIN_7
#define SEL_1_32_47_GPIO_Port GPIOB
#define SEL_3_96_111_Pin GPIO_PIN_9
#define SEL_3_96_111_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
