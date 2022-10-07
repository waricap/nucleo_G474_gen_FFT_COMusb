/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdbool.h"

 // #include "arm_const_structs.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
	{
		uint16_t cmd_flags;
		bool flag_ON_generation;        //	Reg_CMD_Buf[0].0 - флаг-команда Включить Генерацию
		bool flag_ON_scan_freq;         //	Reg_CMD_Buf[0].1 - флаг-команда Вкл. Скольжение по диапазону, в соответствии с задаными регистрами
        bool flag_ON_TxData_cicle;      //	Reg_CMD_Buf[0].2 - флаг-команда Вкл. долбежку передачи данных по кругу, прием при этом прекратиться, стоповать можно будет только синей кнопкой
        bool flag_ON_autoTuning_freq;   //	Reg_CMD_Buf[0].3 - флаг-команда при включеной генерации автоподстройка частоты будет работать, если включен флаг скольжения, этот флаг игнорится
        bool flag_ON_scan_time;         //	Reg_CMD_Buf[0].4 - флаг-команда на вкл генерации на одной частоте, и циклическую передачу сигналов КЛЮЧ�?, ТОК и НАПРЯЖЕН�?Е в реале, как есть из замера

		uint16_t freq_start;	//	Reg_CMD_Buf[2] - регистр стартовой частоты, 14500-40000
		uint16_t proc_pwr;		//	Reg_CMD_Buf[1] - регистр мощности,  2-98% заполнения
		uint16_t step;			//	Reg_CMD_Buf[3] - регистр step(1-25гц) перемещения частоты, при сканировании диапазона, при сканировании старт будет Reg_CMD_Buf[2], максимум = (Reg_CMD_Buf[2] + step*_N-количество_)
		uint16_t time_step;		//	Reg_CMD_Buf[4] - регистр время милисекунд, между степами (10-1000мс)
		uint16_t N_step;		//	Reg_CMD_Buf[5] - регистр N-количество степов при сканировании 4-1000
	} struct_cmd_set;

	/* ----------------------- Type definitions ---------------------------------*/
	typedef enum
	{
	    STATE_RX_INIT,              /*!< Receiver is in initial state. Приемник находится в исходном состоянии.*/
	    STATE_RX_IDLE,              /*!< Receiver is in idle state. 	Приемник находится в режиме ожидания.*/
	    STATE_RX_RCV,               /*!< Frame is beeing received. 		Frame принимается.*/
	    STATE_RX_ERROR              /*!< If the frame is invalid.  		Если frame- invalid */
	} eMBRcvState;

	typedef enum
	{
	    STATE_TX_IDLE,              /*!< Transmitter is in idle state.  Передатчик находится в режиме ожидания. */
	    STATE_TX_XMIT               /*!< Transmitter is in transfer state. Передатчик находится в состоянии передачи. */
	} eMBSndState;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MB_ADDRESS  0x07/*address*/
#define LEN_FLOAT_ARRAY_SEND_UDP 1024
#define LEN_CMD_ARRAY 8
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint16_t fun_data_time_podgotovka( uint8_t * buffer_data_time, uint16_t index_data_time_zamer );
void fun_data_scan_freq_podgotovka( uint8_t * buffer_data_scan_freq,  float  freq );
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
