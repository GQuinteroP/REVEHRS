/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define PB_ST_Pin GPIO_PIN_13
#define PB_ST_GPIO_Port GPIOB
#define PB_ST_EXTI_IRQn EXTI15_10_IRQn
#define STATUS_P_Pin GPIO_PIN_6
#define STATUS_P_GPIO_Port GPIOC
#define STATUS_N_Pin GPIO_PIN_7
#define STATUS_N_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define UART_DEBUG_Tx_Pin GPIO_PIN_6
#define UART_DEBUG_Tx_GPIO_Port GPIOB
#define UART_DEBUG_Rx_Pin GPIO_PIN_7
#define UART_DEBUG_Rx_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define 	ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_1     ((uint32_t)0x08000800) /* Base @ of Page 1, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_2     ((uint32_t)0x08001000) /* Base @ of Page 2, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_3     ((uint32_t)0x08001800) /* Base @ of Page 3, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_4     ((uint32_t)0x08002000) /* Base @ of Page 4, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_5     ((uint32_t)0x08002800) /* Base @ of Page 5, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_6     ((uint32_t)0x08003000) /* Base @ of Page 6, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_7     ((uint32_t)0x08003800) /* Base @ of Page 7, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_8     ((uint32_t)0x08004000) /* Base @ of Page 8, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_9     ((uint32_t)0x08004800) /* Base @ of Page 9, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_128   ((uint32_t)0x08040000) /* Base @ of Page 128, 2 Kbytes */

#define 	ADDR_FLASH_PAGE_251   ((uint32_t)0x0807d800) /* Base @ of Page 254, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_252   ((uint32_t)0x0807e000) /* Base @ of Page 254, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_253   ((uint32_t)0x0807e800) /* Base @ of Page 254, 2 Kbytes */

#define 	ADDR_FLASH_PAGE_254   ((uint32_t)0x0807f000) /* Base @ of Page 254, 2 Kbytes */
#define 	ADDR_FLASH_PAGE_255   ((uint32_t)0x0807f800) /* Base @ of Page 255, 2 Kbytes */
#define 	FLASH_ROW_SIZE          32

#define		fast_125ms 	4000
#define		padding		96
#define 	NLeqSlow	8
#define		false		0
#define		true		1
#define 	CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define 	GET_BITS(x, pos) ((x & ( 1 << pos)) >> pos)
#define 	CLR_BIT(p,n) ((p) &= ~((1) << (n)))

#define 	n_bytes_header	15//8//
#define 	n_bytes_payload	33//48//
#define		block_payloads	7//5//	//1 header + 7 payloads (246 bytes). Limited by SIM7022 since only 255 bytes can be sent in one round
#define		n_blocks_LPWA	10	//Transmit data when XX blocks are acquired
#define		block_len		(n_bytes_header + block_payloads*n_bytes_payload)
#define 	buffer_size		100
#define		max_msgs		10
#define 	buffer_lwpa_config_size 	200
#define		eeprom_size		8191	//Size in pages number
#define		n_blocks_full_buffer	(uint8_t) buffer_size*0.8	//Store data from multiples of XX	- 6 blocks takes around 3208e-5 for storing and 1322e-5 for reading.
#define		n_blocks_EEPROM	4
#define		cal_def			-11.5	//Kind of pre-calibrated value
#define		cal_def_LF		-15		//Kind of pre-calibrated value

#define 	data_buff_len		512

//DEBUG AND TESTING DEFINES
//#define 	LOW_POWER		//Enable low power mode
//#define		LOW_FREQ		//Enable processor's clock frequency reduction when no USB is detected
#define		MIC				//Work with mic signal or sine
//#define		debug_GNSS		0//Print GNSS information (0 - Only NGGA string, 1 - Time, Lat, Lon, Lock, and Sat)
//#define		debug_USB
#define		debug_LPWA
#define		debug_Leq		0//Print Leq timing, TOB_LF (0), TOB (1) and LPWA (2)

#define 	PSD_ACC_DEC
//#define 	IIR_DEC

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */