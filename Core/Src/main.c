/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "arm_const_structs.h"
#include "stdbool.h"
#include "mb.h"
#include "mbrtu.h"

// #include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* ----------------------- Defines ------------------------------------------*/
#define REG_HOLDING_START           1  // при таком значении - при обращении из вне, 0-регистр это 0, 1-регистр это 1. и тд
#define REG_HOLDING_NREGS           16 // количество командных регистров
#define REG_INPUT_START             64 // стартовый номер регистра области данных
#define REG_INPUT_NREGS             65530  // длина области данных, заполняться полностью не будет, данные будут подставляться из результатов обработки замеров

// все статические переменные пусть будут глобальными, сойдет и так
volatile eMBRcvState eRcvState;
volatile eMBSndState eSndState;


/* ----------------------- Static variables ---------------------------------*/
static uint16_t   num_reg_CMD_Start = REG_HOLDING_START; // CMD == Holding_Register
static uint16_t   Reg_CMD_Buf[REG_HOLDING_NREGS]; 		// CMD == Holding_Register
	//	Reg_CMD_Buf[0] - регистр флагов-команд, приходящих для исполнения
	//		Reg_CMD_Buf[0].0 - флаг-команда Включить Генерацию
	//		Reg_CMD_Buf[0].1 - флаг-команда Вкл. Скольжение по диапазону, в соответствии с задаными регистрами
	//		Reg_CMD_Buf[0].2 - флаг-команда Вкл. долбежку передачи данных по кругу, прием при этом прекратиться, стоповать можно будет только синей кнопкой
	//		Reg_CMD_Buf[0].3 - флаг-команда при включеной генерации автоподстройка частоты будет работать, если включен флаг скольжения, этот флаг игнорится
	//		Reg_CMD_Buf[0].4 - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].5 - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].6 - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].7 - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].8 - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].9 - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].A - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].B - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].C - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].D - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].E - флаг-команда резерв пока
	//		Reg_CMD_Buf[0].F - флаг-команда резерв пока
	//	Reg_CMD_Buf[1] - регистр мощности,  2-98% заполнения
	//	Reg_CMD_Buf[2] - регистр стартовой частоты, 14500-43000
	//	Reg_CMD_Buf[3] - регистр step(1-25гц) перемещения частоты, при сканировании диапазона, при сканировании старт будет Reg_CMD_Buf[2], максимум = (Reg_CMD_Buf[2] + step*_N-количество_)
	//	Reg_CMD_Buf[4] - регистр время милисекунд, между степами (10-1000мс)
	//	Reg_CMD_Buf[5] - регистр N-количество степов при сканировании 4-1000
	//	Reg_CMD_Buf[6] - регистр резерв пока
	//	Reg_CMD_Buf[7] - регистр резерв пока
	//	Reg_CMD_Buf[8] - регистр резерв пока
	//	Reg_CMD_Buf[9] - регистр резерв пока
	//	Reg_CMD_Buf[10] - регистр резерв пока
	//	Reg_CMD_Buf[11] - регистр резерв пока
	//	Reg_CMD_Buf[12] - регистр резерв пока
	//	Reg_CMD_Buf[13] - регистр резерв пока
	//	Reg_CMD_Buf[14] - регистр резерв пока
	//	Reg_CMD_Buf[15] - регистр резерв пока
	// пример команды на исполнение: включить генерацию, скольжение, долбежку (0x0007). Мощн-2% (0х0002). F_start-20000kHz (0x4E20). Step-10hz (0x000A). T-20ms (0x0014). N-1000 (0x03E8).
	//			0x07 адрес
	//			0x10 команда на запись пакета регистров
	//			0x00 Адрес первого регистра, с которого начинать запись, Hi-байт
	//			0x00 Адрес первого регистра, с которого начинать запись, Lo-байт
	//			0x00 количество регистров, которые надо записать, Hi-байт
	//			0x06 количество регистров, которые надо записать, Lo-байт
	//			0x0C количество байтов данных (не слов а байтов) в этой посылке, -байт
	//			0x00 значение Reg_CMD_Buf[0], Hi-байт  	== включить генерацию, скольжение, долбежку (0x0007)
	//			0x07 значение Reg_CMD_Buf[0], Lo-байт
	//			0x00 значение Reg_CMD_Buf[1], Hi-байт	== Мощн-2% (0х0002)
	//			0x02 значение Reg_CMD_Buf[1], Lo-байт
	//			0x4E значение Reg_CMD_Buf[2], Hi-байт	== F_start-20000kHz (0x4E20)
	//			0x20 значение Reg_CMD_Buf[2], Lo-байт
	//			0x00 значение Reg_CMD_Buf[3], Hi-байт	== Step-10hz (0x000A)
	//			0x0A значение Reg_CMD_Buf[3], Lo-байт
	//			0x00 значение Reg_CMD_Buf[4], Hi-байт	== T-20ms (0x0014)
	//			0x14 значение Reg_CMD_Buf[4], Lo-байт
	//			0x03 значение Reg_CMD_Buf[5], Hi-байт	== N-1000 (0x03E8)
	//			0xE8 значение Reg_CMD_Buf[5], Lo-байт
	//			0xFE значение CRC16 этого пакета, Hi-байт
	//			0x8D значение CRC16 этого пакета, Lo-байт
	//
//static uint16_t   usRegInputStart = REG_INPUT_START;
//static uint16_t   usRegInputBuf[128]; //[REG_INPUT_NREGS];
uint16_t timeout_Tim6_50us =0;
uint16_t counter_Tim6_MB ;

enum
{
  TRANSFER_RX_WAIT,
  TRANSFER_RX_START,
  TRANSFER_RX_COMPLETE,
  TRANSFER_RX_ERROR,
  TRANSFER_RX_Stop,

  TRANSFER_TX_WAIT,
  TRANSFER_TX_START,
  TRANSFER_TX_COMPLETE,
  TRANSFER_TX_ERROR,
  TRANSFER_TX_Stop,

  TRANSFER_RX_COMPLETE_copy
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	// ===================== ======================================================
#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

#define LENGTH_SAMPLES 		1024
#define LENGTH_OTSOS  		(32 + LENGTH_SAMPLES/2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
UART_HandleTypeDef * adr_huart_MB;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern CRC_HandleTypeDef hcrc;

volatile	uint8_t temp_value1[8] ;
volatile	uint8_t temp_value2 ;
volatile	uint16_t temp_value16;
volatile	uint16_t * temp_adr16;

 uint32_t cnt_tim1;
 uint32_t ccr1_tim1 = 63;
 uint32_t ccr2_tim1 = 10;
 uint32_t ccr3_tim1 = 2;
 uint32_t ccr4_tim1 = 66;
 uint32_t ccr5_tim1 = 10;
 uint32_t power_procent = 20; // процент заполнения импульса накачки будет меняться от 2% до 98%
 uint32_t step_power_procent = 20; // процент заполнения импульса накачки будет меняться от 2% до 98%
 uint8_t	flag_step_power;
 uint8_t  count_press_blue_btn;
 uint8_t flag_exti_stop_TX_cicles;

 uint16_t flag_start_ADC;
 uint16_t flag_complit_ADC;
 uint16_t flag_end_FFT =0;

 volatile uint32_t count_tic_start;
 volatile uint32_t count_tic_finish;
 float32_t count_tic_float_mks;

 volatile uint32_t count_tic_adc_start;
 volatile uint32_t count_tic_adc_finish;
 float32_t count_tic_adc_float_mks;

 volatile uint32_t count_tic_FFT_start;
 volatile uint32_t count_tic_FFT_finish;
 float32_t count_tic_FFT_float_mks;

 volatile uint32_t count_tic_ALL_start;
 volatile uint32_t count_tic_ALL_finish;
 float32_t count_tic_ALL_float_mks;

 volatile uint32_t count_tic_ALL_MemToMem;
 float32_t count_tic_MemToMem_float_mks;

 GPIO_PinState status_button_blue;
 uint16_t flag_start_oscil;
 uint8_t flag_generate_OFF;
 uint8_t flag_generate_ON;


 // -------------------------------------------------------------------
 // Input and Output buffer Declarations for FFT Bin
 uint32_t zamer_adc1_2[LENGTH_SAMPLES];
 uint32_t zamer_adc_dma[LENGTH_SAMPLES];
 uint32_t zamer_tim20[LENGTH_SAMPLES];
 uint32_t adc1_int[LENGTH_SAMPLES];
 float32_t  data_adc1[LENGTH_SAMPLES*2] ; // формируем через ноль, это будет входным файлом для БПФ
 float32_t  data_adc2[LENGTH_SAMPLES*2] ; // формируем через ноль, это будет входным файлом для БПФ
 int 		size_data_adc1;
 uint16_t  adc1_Tx[LENGTH_SAMPLES] ; // массив временный, будет хранится пока не пройдет вся передача на ПК
 uint16_t  adc2_Tx[LENGTH_SAMPLES] ; // массив временный, будет хранится пока не пройдет вся передача на ПК
 float32_t  data_adc1_Tx[LENGTH_SAMPLES] ; // массив временный, будет хранится пока не пройдет вся передача на ПК
 float32_t  data_adc2_Tx[LENGTH_SAMPLES] ; // массив временный, будет хранится пока не пройдет вся передача на ПК
 float32_t  filter_adc1[LENGTH_SAMPLES] ; // отфильтрованое замеры ацп1, для вывода на дисплей
 float32_t  filter_adc2[LENGTH_SAMPLES] ;  // отфильтрованое замеры ацп2, для вывода на дисплей
 static float32_t arr1_Output_f32[LENGTH_SAMPLES*2];
 static float32_t arr2_Output_f32[LENGTH_SAMPLES*2];
  float32_t arr1_phase_Output_8_f32;
  float32_t arr2_phase_Output_8_f32;
  float32_t arr_power_Output_8_f32;
  float32_t temp_phase_f32;

 /* ------------------------------------------------------------------
 * Global variables for FFT Bin Example
 * ------------------------------------------------------------------- */
 uint32_t fftSize; // сколько магнитуд достаточно вычислить
 uint32_t ifftFlag = 0;
 uint32_t doBitReverse = 1;
 uint16_t fft_N =  512;  // [] = {256, 512, 1024, 2048, 4096}; // массив точек расчета
 uint32_t freq_tim1; //частота, которую выдает TIM1
 uint32_t freq_tim1_float; //частота, которую выдает TIM1
 float32_t F_bin;
 float32_t koeff;
 float ln_x3, ln_x4, ln_x5;
 float32_t freq_new; // расчитаное смещение частоты от частоты таймера, плюс минус там уже вложено
 int16_t	flag_napravlenia_scan = 1; // флаг направления сканировнаия по частоте, два значения +1 и -1
 float32_t old_freq_new;
 uint32_t hrtim_period_new = 6800000;

 /* Reference index at which max energy of bin ocuurs */
 uint32_t refIndex = 213, testIndex = 0;

 // ===============================================================
 //Параметры фильтра на частоту 100 KHz
 //R ≈ (2*Sin(PI*Wr))^2, где Wr = Fr/Fd, Fr = 100 KHz, Fd = 1000 KHz
 //Параметры фильтра на частоту ==freq_tim1 KHz,  полоса == F_bin
 //R ≈ (2*Sin(PI*Wr))^2, где Wr = Fr/Fd, Центральная частота полосы пропускания фильтра Fr = freq_tim1 Hz, Частота дискретизации Fd = freq_tim1 *64 Hz,  в нашем случае где Wr=1/64
 // частота выборок всегда в 64 раза выше центральной-заданой, значит параметр всегда  Wr=1/64
 // R_filter = powf((2*sinf(M_PI /64)), 2);  всегда == 0.0943743718
 float32_t R_filter = 0.009630546655; //
 float32_t L_filter =  0.001;
 //Начальные значения
 float32_t X_filter_1 = 0;
 float32_t V_filter_1 = 0;
 float32_t X_filter_2 = 0;
 float32_t V_filter_2 = 0;
 float32_t err_f[10];
 float32_t delta_F;

 uint16_t delta_power;

 uint16_t  data_I[LEN_FLOAT_ARRAY_SEND_UDP+1] ; // массив 1
 uint16_t  data_U[LEN_FLOAT_ARRAY_SEND_UDP+1] ; // массив 1
 uint16_t  data_P[LEN_FLOAT_ARRAY_SEND_UDP+1] ; // массив 1
 uint16_t  data_R[LEN_FLOAT_ARRAY_SEND_UDP+1] ; // массив 1
 uint16_t  data_Z[LEN_FLOAT_ARRAY_SEND_UDP+1] ; // массив 1
 //uint16_t  data_Q[LEN_FLOAT_ARRAY_SEND_UDP+1] ; // массив 1
 //uint16_t  data_X[LEN_FLOAT_ARRAY_SEND_UDP+1] ; // массив 1
 //uint16_t  data_Y[LEN_FLOAT_ARRAY_SEND_UDP+1] ; // массив 1
 uint16_t  index_temp;
 uint16_t  index_data_real_zamer;
 uint16_t 	flag_data_complit_for_Tx=0;
 uint16_t 	index_array_TX;
 uint16_t  cmd_array_SPI[LEN_CMD_ARRAY]; // это будет файл с командами на генерацию
 uint16_t  len_cmd_array_SPI = LEN_CMD_ARRAY *2;
 struct_cmd_set 	cmd_set={0};
 uint8_t	send_buff_cicle[85];

uint8_t  cmd_array_test[LEN_CMD_ARRAY*2];
  // uint16_t  HEAP_arr_ADC_zamer[LEN_FLOAT_ARRAY_SEND_UDP*2*8] ; // массив 1
  float32_t temp_float;
 uint16_t  len_data_TX_SPI = LEN_FLOAT_ARRAY_SEND_UDP *8;

/////////////////////////////////// USBD_StatusTypeDef status_USBcdc_TX;
 ///////////////////////////////////  USBD_StatusTypeDef status_USBcdc_RX;
 ///////////////////////////////////  extern USBD_HandleTypeDef hUsbDeviceFS;
 HAL_StatusTypeDef status_TX_LPUART;
 HAL_StatusTypeDef status_RX_LPuart;

 volatile uint32_t wTransferState_RX = TRANSFER_RX_Stop;// transfer state
 volatile uint32_t wTransferState_TX = TRANSFER_TX_Stop;// transfer state

 HAL_StatusTypeDef answer_Transmit_DMA;

 uint16_t flag_TX_tim7, flag_RX_tim7;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Mem_to_Mem_Complete(DMA_HandleTypeDef *hdma_memtomem_dma1_channel1);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	static uint32_t lock_nesting_count = 0;
	void __critical_enter(void)
	{
		__disable_irq();
		++lock_nesting_count;
	}
	void __critical_exit(void)
	{
		/* Unlock interrupts only when we are exiting the outermost nested call. */
		--lock_nesting_count;
		if (lock_nesting_count == 0) {
			__enable_irq();
		}
	}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// test for github
	// ===================== тики замеряем, время работы вычисляем
    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;// разрешаем использовать DWT
	DWT_CYCCNT = 0;// обнуляем значение
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; // включаем счётчик, здесь и один раз
					// DWT_CYCCNT = 0;// обнуляем значение
					// DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; // включаем счётчик
					// Какой-то код
					// count_tic = DWT_CYCCNT;//смотрим сколько натикало
					// count_tic_start = DWT_CYCCNT;
					//			 какой то код
					// count_tic_finish = DWT_CYCCNT - count_tic_start;//смотрим сколько натикало  - ццикл отсоса массивов данных длится 16мкс
					// count_tic_float_mks = (float)count_tic_finish * 1000000 / SystemCoreClock;
	// ===================== ======================================================
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // количество ошибок при разных скоростях передачи от G474 -> H743  MX_SPI2_Init()
  // G474_prescaler_4 - speed_42Mb - err_25,16,24
  // G474_prescaler_8 - speed_21Mb - err_14,18,16
  // G474_prescaler_16 - speed_10Mb - err_11,19,13
  // G474_prescaler_256 - speed_0.6Mb - err_16,16,24
  // 4 -много ошибок,  256 - медлено,   16 - вроде как норм

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  // TIM6 - используем для модбуса, делает прерывания через 50uS, а там библиотека ModBUS  работает
  // TIM7 - используется для барабанной передачи данных по модбусу, выдает тики 100uS, прерывания нет, в цикле смотрим CNT
  // TIM16 - используем для увеличения уменьшения мощности, выдает тики 1uS, прерывания нет, в цикле смотрим CNT
  // TIM17 - используем для сканирования по частоте, выдает тики 100uS, прерывания нет, в цикле смотрим CNT
  // �?нициализируйте стек протоколов в режиме RTU для SLAVE устройства с адресом 1 = 0x01
  adr_huart_MB = &huart1;
   eMBInit( MB_RTU, MB_ADDRESS/*address*/, 1/*LPUART1*/, adr_huart_MB->Init.BaudRate/*115200*/ , adr_huart_MB->Init.Parity/*UART_PARITY_NONE*/ );

   // Включите стек протоколов Modbus.
   eMBEnable();

	  HAL_Delay(300);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Led green

  printf("MX_xxx_Init(); \n");
  for(int i=0; i<LEN_FLOAT_ARRAY_SEND_UDP; i++)
  {
	  data_I[i] = (uint16_t) 1000 * sinf(((float)i)/64);
	  data_U[i] = (uint16_t) cosf(((float)i)/64);
	  data_P[i] = (uint16_t) 100 * sinf(((float)i)/64);
	  data_R[i] = (uint16_t) 100 * cosf(((float)i)/64);
	  data_Z[i] = (uint16_t) tanf(((float)i)/64);
	  //data_Q[i] = (uint16_t) i;
	  //data_X[i] = (uint16_t) i+1024;
	  //data_Y[i] = (uint16_t) i+4096;
  }
  //data_Q[1024] = 0;
  data_I[1024] = 1;
  data_U[1024] = 2;
  data_P[1024] = 3;
  data_R[1024] = 4;
  data_Z[1024] = 5;
  //data_X[1024] = 6;
  //data_Y[1024] = 7;

  // ============== LPUART =============== LPUART ============= LPUART ==================
  // вначале надо получить данные, пока будут нулевые, ещё нет никаких,
  // но потом прием данных будет активироваться окончанием передачи.
  // Дальше приемник не будет активирован. активация приемника происходит только 1 раз, после передачи
	  wTransferState_RX = TRANSFER_RX_START; // TRANSFER_WAIT - пока ждем приема настроечных данных
	  wTransferState_TX = TRANSFER_TX_Stop;
  // штучная передача данных замеров пойдет только тогда, когда будут приняты настройки частоты-работы �? слово окончания посылки кмд


  // не надо ничего ждать-проверять, дали команду и пошли дальше
  // while (wTransferState != TRANSFER_TXRX_COMPLETE)	// крутить цикл до тех пор, пока не пройдет правильная передача, перед началом генерации - это обязятельно
  // 	  {
  //  		status_SPI = HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *) data_I, (uint8_t *) cmd_array_SPI, len_cmd_array_SPI);
  //		if (status_SPI != HAL_OK)
	//  	  	  {  wTransferState = TRANSFER_ERROR; }
  	//  }
  // if ((wTransferState != TRANSFER_ERROR ) & (status_SPI == HAL_OK))
  // { ; } // теперь надо извлечь все надобные настройки из полученого массива   cmd_array_SPI[] , только потом мохно будет запускать генерацию
  // else
  // { default; }

  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_MASTER);

  // TIM1 - главный таймер выдает управление ключами моста
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 	// плечо №1-низ PC0
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); 	// плечо №1-верх PА7
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); 	// плечо №2-низ PС2
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); 	// плечо №2-верх PB9
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);		// запаздывание CH3 относительно CH1 - это ширина импульса накачки, мощность

  HAL_TIM_Base_Start(&htim7); // tim7 тактуется 10 MHz, по условию if (htim7.Instance->CNT >100)   будет включаться передача ModBUS_TX
  HAL_TIM_Base_Start(&htim16);// tim16 тактуется 1 MHz,  по условию if (htim16.Instance->CNT >10000)   будет плавно меняться мощность на одну ступеньку
  HAL_TIM_Base_Start(&htim17); // tim17 тактуется 10 MHz, по условию if (htim17.Instance->CNT > time_step*10)   будет работать скольжение по частоте

  // для запуска 256 замеров, от HRTIM_SCOUT / 2, будут перекрывать 4 периода
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);      // синхроимпульс для осцила, 6 тактов от таймера TIM1 отсчитывает

	HAL_ADCEx_Calibration_Start(&hadc1,  ADC_SINGLE_ENDED );
	HAL_ADCEx_Calibration_Start(&hadc2,  ADC_SINGLE_ENDED );
	uint32_t temp_length = LENGTH_SAMPLES; // 32 замера, по 16 в начале и в конце, будут отброшены
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, zamer_adc1_2, temp_length);
   // LL_ADC_REG_StopConversion(hadc1->Instance);  - это есть стоп АЦП произвольно по желанию, по тесту в любом месте
   // LL_ADC_REG_StartConversion(hadc1.Instance);  - это есть старт АЦП произвольно по желанию, если был остановлен

    size_data_adc1 = sizeof(data_adc1)/sizeof(data_adc1[0]); // делаем один раз при инициализации
	for (uint16_t ic =0; ic <size_data_adc1; ic++)//обнулим для начала
		{
			data_adc1[ic] =0;
			data_adc2[ic] =0;
		}

	  /* Select Callbacks functions called after Transfer complete and Transfer error */
	  HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_channel4, HAL_DMA_XFER_CPLT_CB_ID, Mem_to_Mem_Complete);
	  HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel4, (uint32_t)zamer_adc1_2, (uint32_t)zamer_adc_dma, LENGTH_SAMPLES);

	  //R_filter = 0.5857864376269; // для 512 замеров
	 // L_filter = 0.01;

	  freq_new = 25000;
	  old_freq_new = freq_new;
	// HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, adc1_int, 256, DAC_ALIGN_12B_R);

	  printf("INIT__xxx (); - executed  \n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  uint16_t Num=0;
	  uint16_t usCRC16_main;
  while (1)
  {
	  if (((cmd_set.cmd_flags & 0x0004) >0) | ((cmd_set.cmd_flags & 0x0010) >0))// если надо, по флагу, вот тут будет включаться циклическая долбежка
	  {
                  // TIM7 - используется для барабанной передачи данных по модбусу, выдает тики 100uS, прерывания нет, в цикле смотрим CNT
		  	  	  // 25 mS  минимальный интервал между передачами, при котором минимум ошибок CRC
              	  if (htim7.Instance->CNT >250) // tim7 тактуется 10 MHz, по условию if (htim7.Instance->CNT >100)   будет включаться передача ModBUS_TX
              	  {
              		  htim7.Instance->CNT =0;
              		  	send_buff_cicle[0] = MB_ADDRESS;
						send_buff_cicle[1] = 4;  // заглушка-обманка
						send_buff_cicle[2] = 80; // заглушка-обманка, типа длина данных
						if (eSndState != STATE_TX_XMIT)/* Activate the transmitter. */
						{
							eMBRegInputCB( &(send_buff_cicle[3]), 64, Num /*номер элемента массива замеров*/ );
							usCRC16_main = HAL_CRC_Calculate(&hcrc, ( uint32_t *)(&send_buff_cicle), 83);//смотрим сколько натикало -цикл HAL_CRC_Calculate(85char) длится 11.30мкс (1921 тика)
							send_buff_cicle[83] = ( UCHAR )( usCRC16_main & 0xFF );
							send_buff_cicle[84] = ( UCHAR )( usCRC16_main >> 8 );
							vMBPortSerialEnable( FALSE, TRUE );
							eSndState = STATE_TX_XMIT;
							if ( HAL_OK == HAL_UART_Transmit_DMA(adr_huart_MB, send_buff_cicle, 85))
							{
								if (flag_exti_stop_TX_cicles >0)
								{
									flag_exti_stop_TX_cicles =0;
									Num++;
									if (Num >999) { Num =0;}
								}
							}
						}
              	  }
	  }
	  else
	  {  eMBPoll(  ); } // Вызовите основной цикл опроса стека протоколов Modbus.




	  // power_procent - меняется от 2 проц до 98 проц, и от этого сдвигаются фронты TIM1_OUT3,  TIM1_OUT1 - constanta
	  if (htim16.Instance->CNT >1000) // tim16 тактуется 1 MHz, нужен для плавного старта-стопа генерации
	  {
		  htim16.Instance->CNT =0;
		  // синяя кнопка для стопа, если был запущен с модбуса
	      if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) // нажата кнопка
	      { 														// HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	    	  count_press_blue_btn ++;
	    	  if (count_press_blue_btn >5)
	    	  {
	    		  count_press_blue_btn=5;
	    		  cmd_set.cmd_flags =0;  // тотальный стоп
	    	  }
	      }
	      else
	      { count_press_blue_btn =0; } // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);


		  if ((cmd_set.cmd_flags & 0x0001) >0)// Reg_CMD_Buf[0].0 - флаг-команда Включить Генерацию
	  		{
	  			if (step_power_procent > cmd_set.proc_pwr /*power_procent*/) {step_power_procent --;}
	  			if (step_power_procent < cmd_set.proc_pwr /*power_procent*/) {step_power_procent ++;}
	  	  	  	  // flag_generate_ON  flag_generate_OFF - потому что onoff генерацию будем только в первой половине периода, чтобы deadtime - как надо
	  	  	  	  	cnt_tim1 = htim1.Instance->CNT;   // tim1->ARR == 127  всегда без вариантов
					if ((cnt_tim1 > 15) & (cnt_tim1 <55))
					{
						if (flag_generate_ON ==0)
							{ __HAL_TIM_MOE_ENABLE(&htim1); } // програмно восстанавливается флаг MOE для подключения выходов TIM1
						flag_generate_ON =255;
						flag_generate_OFF =0;
					}
	  		}
	  		else	// кнопка отпущена, СТОП
	  		{
				if (step_power_procent > 2)
					{step_power_procent --;}
				else
				{
					step_power_procent =2;
	  	  	  	  	  // flag_generate_ON  flag_generate_OFF - потому что onoff генерацию будем только в первой половине периода, чтобы deadtime - как надо
	  	  	  	  	  	cnt_tim1 = htim1.Instance->CNT;   // tim1->ARR == 127  всегда без вариантов
	  					if ((cnt_tim1 > 15) & (cnt_tim1 <55))
	  					{
							if (flag_generate_OFF ==0)
							{
								for(int i=0; i<100; i++)
								{
									if (HAL_OK == HAL_TIM_GenerateEvent(&htim1, TIM_EventSource_Break))
									{
										for (uint16_t ic =0; ic <size_data_adc1/2; ic++)//обнулим чтобы при следующей частоте не было лажи
											{
												data_adc1_Tx[ic] =0;
												data_adc2_Tx[ic] =0;
												adc1_Tx[ic] =0;
												adc2_Tx[ic] =0;
											}
										index_data_real_zamer =0; // подготовка для замера
										break;
									}// if (HAL_OK == HAL_TIM_GenerateEvent(&htim1, TIM_EventSource_Break))
								} // for(int i=0; i<100; i++)
							} // if (flag_generate_OFF ==0)
							flag_generate_OFF =255;
							flag_generate_ON =0;
	  					} //if ((cnt_tim1 > 15) & (cnt_tim1 <55))
				} //  else    if (step_power_procent > 2)
	  		}// if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
	  }// if (htim16.Instance->CNT >10000)

	  	  	  	  	  // ======   А ЭТО СТОП  СТОП  ====== ======   А ЭТО START  START  ======
	  	  	  	  	  // flag_generate_ON  flag_generate_OFF - потому что onoff генерацию будем только в первой половине периода, чтобы deadtime - как надо
	/*  	  	  	  	  	cnt_tim1 = htim1.Instance->CNT;   // tim1->ARR == 127  всегда без вариантов
	  					if ((cnt_tim1 > 15) & (cnt_tim1 <55))
	  					{
	  						if (flag_generate_OFF >0)
	  						{
	  							for(int i=0; i<100; i++)
	  							{
	  								if (HAL_OK == HAL_TIM_GenerateEvent(&htim1, TIM_EventSource_Break))
	  								{
	  									for (uint16_t ic =0; ic <size_data_adc1/2; ic++)//обнулим чтобы при следующей частоте не было лажи
	  										{
	  											data_adc1_Tx[ic] =0;
	  											data_adc2_Tx[ic] =0;
	  											adc1_Tx[ic] =0;
	  											adc2_Tx[ic] =0;
	  										}
	  									index_data_real_zamer =0; // подготовка для замера
	  									break;
	  								}
	  							}
	  							//while(HAL_OK != HAL_TIM_GenerateEvent(&htim1, TIM_EventSource_Break)) {;}// програмно формируется сигнал BREAK для стопа выходов TIM1
	  							flag_generate_OFF =0;
	  						}
	  						if (flag_generate_ON >0)
	  						{
	  							__HAL_TIM_MOE_ENABLE(&htim1); // програмно восстанавливается флаг MOE для подключения выходов TIM1
	  							flag_generate_ON =0;
	  						}
	  					}
	  	*/
	  					// ======   А ЭТО СТОП  СТОП  ====== ======   А ЭТО START  START  ======


	  htim1.Instance->CCR1 = ccr1_tim1; // tim1->CCR1 == 63   всегда без вариантов
	  ccr3_tim1 = (64 * step_power_procent ) / 100;
		  if(ccr3_tim1 >62) {ccr3_tim1 =62;}// 96.88%   =>   64*96.88/100=62	 deadtime фиксировано ==85 тиков от 170_МГц, Меньшая мощность будет наползать на deadtime
		  if(ccr3_tim1 <2) {ccr3_tim1 =2;}  // 3.125%   =>    64* 3.125/100=2	 2 тика от TIM1  == 90(тиков от 170_МГц при 30кГц) == 106(тиков от 170_МГц при 25кГц) == 186(тиков от 170_МГц при 14,5кГц)
	  ccr4_tim1 = 64 + ccr3_tim1;
		  if(ccr4_tim1 >125) {ccr4_tim1 =125;}
		  if(ccr4_tim1 <66) {ccr4_tim1 =66;}
	  htim1.Instance->CCR3 = ccr3_tim1; //
	  htim1.Instance->CCR4 = ccr4_tim1; //  power_procent

	  // ========================== а вот ОНО, - новая частота  ============================
	  // hrtim_period_new = 5440000000 / ((uint32_t) freq_new) / (htim1.Instance->ARR +1) ;
	  if(hrtim_period_new >1000) // == 43 kHz    hrtim_period_new=2930  == 14505 Hz
		  {  hhrtim1.Instance->sMasterRegs.MPER = hrtim_period_new; }
	  freq_tim1 = 5440000000 / (hhrtim1.Instance->sMasterRegs.MPER) / 128; // это есть выходная частота, tim1->ARR == 127  всегда ==> делитель=128
	  freq_tim1_float = (float)freq_tim1;

	  if (((cmd_set.cmd_flags & 0x0010) >0) & (index_data_real_zamer >1000))
	  	  { flag_data_complit_for_Tx = 255; }

	  if (flag_end_FFT ==0) // flag_complit_ADC ==1 означает что все замеры сделаны
	  {
		  flag_end_FFT =1;
		//count_tic_adc_finish = DWT_CYCCNT;//смотрим сколько натикало, поскольку при старте  DWT_CYCCNT==0, то все просто
		//count_tic_adc_float_mks = (float)count_tic_adc_finish * 1000000 / SystemCoreClock; // а сколько это мкс? в реале == 82-86 мкс
		// обнуляем счетчик тиков, для подсчета времени работы прг, и одновременно уже будет идти 256 замеров АЦП
			count_tic_ALL_finish = DWT_CYCCNT;
			count_tic_ALL_float_mks = (float)count_tic_ALL_finish * 1000000 / SystemCoreClock; // а сколько это мкс? в реале == 1320 мкс 512 zamerov;
		 DWT_CYCCNT =0;



//		for (uint16_t ic =32; ic < LENGTH_OTSOS; ic++)//цикл отсоса массивов данных длится 16мкс -это без фильтрации, 264 мкс с фильтрацией(256), 534мкс фильтрация(512)
		for (uint16_t ic =0; ic < LENGTH_SAMPLES; ic++)//цикл отсоса массивов данных длится 16мкс -это без фильтрации, 264 мкс с фильтрацией(256), 534мкс фильтрация(512)
			{
				//adc1_int[ic-32] = zamer_adc1_2[ic] & 0x0000FFFF;
				// ===============================================================
				//Фильтр
				//X_filter += (int)ADC1_DR;
				//V_filter -= X_filter * R_filter;
				//X_filter += V_filter - X_filter * L_filter;
				// ===============================================================
				// X_filter_1 += (float) ((uint16_t) (zamer_adc1_2[ic] & 0x0000FFFF));
				//data_adc1[2*ic] = 0.001 * (float) (zamer_adc_dma[ic] & 0x0000FFFF);
				// if (zamer_tim20[ic] >50) { X_filter_1 += 1; }
				if(flag_data_complit_for_Tx>0) { adc1_Tx[ic] = zamer_adc_dma[ic] & 0x0000FFFF; }
				X_filter_1 += (float) (zamer_adc_dma[ic] & 0x0000FFFF);
				V_filter_1 -= X_filter_1 * (R_filter);
				X_filter_1 += V_filter_1 - X_filter_1 * L_filter;
				filter_adc1[ic] = X_filter_1;
				data_adc1[2*ic] = 0.001 * X_filter_1;
				if(flag_data_complit_for_Tx>0) { data_adc1_Tx[ic] = data_adc1[2*ic]; }
				//data_adc1[2*(ic-32)] = X_filter_1;
				//data_adc1[2*(ic-32)] = (float) ((uint16_t) (zamer_adc1_2[ic] & 0x0000FFFF));
				//data_adc1[2*(ic-32+1)] =0;
				// ==================================================================
				// X_filter_2 += (float) ((uint16_t) (zamer_adc1_2[ic] >>16));
				//data_adc2[2*ic] = 0.001 * (float) (zamer_adc_dma[ic] >>16);
				if(flag_data_complit_for_Tx>0) { adc2_Tx[ic] = (zamer_adc_dma[ic] >>16); }
				X_filter_2 += (float) ((zamer_adc_dma[ic] >>16));
				V_filter_2 -= X_filter_2 * (R_filter);
				X_filter_2 += V_filter_2 - X_filter_2 * L_filter;
				filter_adc2[ic] = X_filter_2;
				data_adc2[2*ic] = 0.001 * X_filter_2;
				if(flag_data_complit_for_Tx>0) { data_adc2_Tx[ic] = data_adc2[2*ic]; }
				//data_adc2[2*(ic-32)] = X_filter_2;
				//data_adc2[2*(ic-32)] = (float) ((uint16_t) (zamer_adc1_2[ic] >>16));
				//data_adc2[2*(ic-32+1)] =0;
			}

			if(flag_data_complit_for_Tx>0)
				{
					index_data_real_zamer =0;
					flag_data_complit_for_Tx =0;
				}

//		for (uint16_t ic =0; ic < 32; ic=ic+2)//цикл отсоса массивов данных длится 16мкс -это без фильтрации, 264 мкс с фильтрацией(256), 534мкс фильтрация(512)
//			{
//				if (ic <delta_power)
//				{
//					for (uint16_t iv =0; iv < 16; iv++)
//					{ data_adc1[32*iv + ic] = 0; }
//				}
//			}

			flag_end_FFT =2; // такой флаг разрешает остановить АЦП и перекинуть массив данных, пока тут дорасчитываем до конца
							// перекид данных занимает 41мкс, и произойдет только при полном окончании N(512) замеров
			count_tic_FFT_start= DWT_CYCCNT;
		  // Process the data through the CFFT/CIFFT module
			if (fft_N ==256)
			{
			  arm_cfft_f32(&arm_cfft_sR_f32_len256, data_adc1, ifftFlag, doBitReverse);//в реале == 361мкс(256 sampl)
			  arm_cfft_f32(&arm_cfft_sR_f32_len256, data_adc2, ifftFlag, doBitReverse);
			}
			else
			{
				if (fft_N ==512)
				{
				  arm_cfft_f32(&arm_cfft_sR_f32_len512, data_adc1, ifftFlag, doBitReverse);//в реале == 618мкс(512 sampl)
				  arm_cfft_f32(&arm_cfft_sR_f32_len512, data_adc2, ifftFlag, doBitReverse);
				}
				else
				{
					if (fft_N ==1024)
					{
					  arm_cfft_f32(&arm_cfft_sR_f32_len1024, data_adc1, ifftFlag, doBitReverse);//в реале == ___мкс(1024 sampl)
					  arm_cfft_f32(&arm_cfft_sR_f32_len1024, data_adc2, ifftFlag, doBitReverse);
					}
					else
					{
						if (fft_N ==2048)
						{
						  arm_cfft_f32(&arm_cfft_sR_f32_len2048, data_adc1, ifftFlag, doBitReverse);//в реале == ___мкс(2048 sampl)
						  arm_cfft_f32(&arm_cfft_sR_f32_len2048, data_adc2, ifftFlag, doBitReverse);
						}
						else
						{
							if (fft_N ==4096)
							{
							  arm_cfft_f32(&arm_cfft_sR_f32_len4096, data_adc1, ifftFlag, doBitReverse);//в реале == 5461мкс(4096 sampl)
							  arm_cfft_f32(&arm_cfft_sR_f32_len4096, data_adc2, ifftFlag, doBitReverse);
							} // if (fft_N ==4096)
						} // if (fft_N ==2048)
					} // if (fft_N ==1024)
				} // if (fft_N ==512)
			} // if (fft_N ==256)

		  // Process the data through the Complex Magnitude Module for  calculating the magnitude at each bin
		  fftSize = fft_N /64 +10; // 10 такое количество гармоник достаточно, для массива 512 без нулей
		  //ввремя вычисления Две магнитуды, в реале == 97мкс(fftSize==256) == 37мкс(fftSize==96)
		  arm_cmplx_mag_f32(data_adc1, arr1_Output_f32, fftSize);
		  arm_cmplx_mag_f32(data_adc2, arr2_Output_f32, fftSize);
		  arr1_phase_Output_8_f32 = atan2f(data_adc1[17], data_adc1[16]);
			  //while(arr1_phase_Output_8_f32 > M_PI ) {arr1_phase_Output_8_f32 = arr1_phase_Output_8_f32 - M_PI; }
			  //while(arr1_phase_Output_8_f32 < (0-M_PI) ) {arr1_phase_Output_8_f32 = arr1_phase_Output_8_f32 + M_PI; }
		  arr2_phase_Output_8_f32 = atan2f(data_adc2[17], data_adc2[16]);
			  //while(arr2_phase_Output_8_f32 > M_PI ) {arr2_phase_Output_8_f32 = arr2_phase_Output_8_f32 - M_PI; }
			  //while(arr2_phase_Output_8_f32 < (0-M_PI) ) {arr2_phase_Output_8_f32 = arr2_phase_Output_8_f32 + M_PI; }
		  temp_phase_f32 = arr1_phase_Output_8_f32 - arr2_phase_Output_8_f32;
//			  while(temp_phase_f32 > 2*M_PI ) {temp_phase_f32 = temp_phase_f32 - M_PI; }
//			  while(temp_phase_f32 < (0-2*M_PI) ) {temp_phase_f32 = temp_phase_f32 + M_PI; }
		  arr_power_Output_8_f32 = arr1_Output_f32[8] * arr2_Output_f32[8] *arm_cos_f32(temp_phase_f32);

		  // тут будем расчитывать смещение частоты, которое нужно после FFT
		// F_bin = F *64/256 = F / 4 ;
		// F_bin = F *64/512 = F / 8 ;
		// F_bin = F *64/1024 = F / 16 ;
		// Koeff = (arr1_Output_f32[5] -arr1_Output_f32[3]) / (2*(2*arr1_Output_f32[4] -arr1_Output_f32[5] -arr1_Output_f32[3]));
		// freq_new = F_bin * N + Koeff * F_bin;   // для этого случая N==4
		  if (((cmd_set.cmd_flags & 0x0001) >0) & ((cmd_set.cmd_flags & 0x0002) ==0) & ((cmd_set.cmd_flags & 0x0008) >0)) // автоподстройка частоты
		  {
			if (fft_N ==256)
			{
				koeff = (arr1_Output_f32[5] -arr1_Output_f32[3]) / (2*(2*arr1_Output_f32[4] -arr1_Output_f32[5] -arr1_Output_f32[3]));
				freq_new = freq_tim1_float + koeff * freq_tim1_float/4;   // для этого случая N==4
			}
			else
			{
				if (fft_N ==512)
				{
					ln_x3 = 100000.f * logf((float)arr1_Output_f32[7]);
					ln_x4 = 100000.f * logf((float)arr1_Output_f32[8]);
					ln_x5 = 100000.f * logf((float)arr1_Output_f32[9]);
					// koeff = (ln_x5 -ln_x3) / (2*(2*ln_x4 -ln_x5 -ln_x3));
					koeff = (8.f + (ln_x5 -ln_x3) / 2.f / (2.f * ln_x4 -ln_x5 -ln_x3)) * 10000.f;
					freq_new = koeff * ((float)freq_tim1_float)/80000.f; // для этого случая N==8
				}
				else
				{
					if (fft_N ==1024)
					{
						koeff = (arr1_Output_f32[17] -arr1_Output_f32[15]) / (2*(2*arr1_Output_f32[16] -arr1_Output_f32[17] -arr1_Output_f32[15]));
						freq_new = freq_tim1_float + koeff * freq_tim1_float/16;   // для этого случая N==16
					}
				}
			} // else  <==  if (fft_N ==256)
		  } // if (((cmd_set.cmd_flags & 0x0001) >0) & ((cmd_set.cmd_flags & 0x0002) ==0) & ((cmd_set.cmd_flags & 0x0008) >0)) // автоподстройка частоты




		  // заполняем HEAP_arr_ADC_zamer, на отправку через SPI, прямые замеры смотреть будем
		  // заполняем HEAP_arr_ADC_zamer, на отправку через SPI, прямые замеры смотреть будем
					 //   ==========  С# будет понимать FLOAT вот так  ================
					// data_S[0] = 0;  		==HEAP_arr_ADC_zamer[2*i] HIGH
					// data_S[1] = 0;		==HEAP_arr_ADC_zamer[2*i] LOW
					// data_S[2] = 0x80;	==HEAP_arr_ADC_zamer[2*i+1] HIGH
					// data_S[3] = 0x3F;	==HEAP_arr_ADC_zamer[2*i+1] LOW
					// array_recive[1] = BitConverter.ToSingle(data_S, 0); // ==1
/*		 for (uint16_t i =0; i < LENGTH_SAMPLES; i++)
		 {
			 temp_float = (float32_t) (zamer_adc_dma[i] & 0x0000FFFF);
			 HEAP_arr_ADC_zamer[2*i] = 									*(((uint8_t *) &temp_float) + 1); //
			 HEAP_arr_ADC_zamer[2*i] = 	(HEAP_arr_ADC_zamer[2*i] <<8) +	*(((uint8_t *) &temp_float) + 0);
			 HEAP_arr_ADC_zamer[2*i+1] =  								*(((uint8_t *) &temp_float) + 3); //
			 HEAP_arr_ADC_zamer[2*i+1] =(HEAP_arr_ADC_zamer[2*i+1]<<8)+	*(((uint8_t *) &temp_float) + 2);

			 temp_float = (float32_t) ((zamer_adc_dma[i] >>16));
			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *2] = 														*(((uint8_t *) &temp_float) + 1); //
			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *2] = 	(HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *2] <<8) +	*(((uint8_t *) &temp_float) + 0);
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *2] =  														*(((uint8_t *) &temp_float) + 3); //
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *2] = 	(HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *2]<<8) +	*(((uint8_t *) &temp_float) + 2);

			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *4 ] = 														*(((uint8_t *) &filter_adc1[i]) + 1); //
			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *4 ] = 	(HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES*4] <<8) +	*(((uint8_t *) &filter_adc1[i]) + 0);
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *4 ] =  														*(((uint8_t *) &filter_adc1[i]) + 3); //
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *4 ] = 	(HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES*4]<<8) +	*(((uint8_t *) &filter_adc1[i]) + 2);

			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *6 ] = 														*(((uint8_t *) &filter_adc2[i]) + 1); //
			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *6 ] = 	(HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES*6] <<8) +	*(((uint8_t *) &filter_adc2[i]) + 0);
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *6 ] =  														*(((uint8_t *) &filter_adc2[i]) + 3); //
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *6 ] = 	(HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES*6]<<8) +	*(((uint8_t *) &filter_adc2[i]) + 2);

			 temp_float =  arr1_Output_f32[i]; // data_adc1[i];// ;(float32_t)zamer_tim20[i];
			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *8 ] = 														*(((uint8_t *) &temp_float) + 1); //
			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *8 ] = 	(HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES*8] <<8) +	*(((uint8_t *) &temp_float) + 0);
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *8 ] =  														*(((uint8_t *) &temp_float) + 3); //
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *8 ] = 	(HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES*8]<<8) +	*(((uint8_t *) &temp_float) + 2);

			 temp_float = arr2_Output_f32[i]; // data_adc2[i]; // data_adc1[i];
			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *10 ] = 														*(((uint8_t *) &temp_float) + 1); //
			 HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES *10 ] = 	(HEAP_arr_ADC_zamer[2*i + LENGTH_SAMPLES*10] <<8) +	*(((uint8_t *) &temp_float) + 0);
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *10 ] =  													*(((uint8_t *) &temp_float) + 3); //
			 HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES *10 ] = 	(HEAP_arr_ADC_zamer[2*i+1 + LENGTH_SAMPLES*10]<<8)+	*(((uint8_t *) &temp_float) + 2);
		 }
*/

			// FREQ = 5440000000 / (hhrtim1.Instance->sMasterRegs.MPER) / (htim1.Instance->ARR -1);

			//memset(data_adc1, 0, 32764); // sizeof(float32_t)*8196); //обнулим место для нового пересчета, время на это == 2700мкс, методом memset
			//memset(data_adc2, 0, 32764); // sizeof(float32_t)*8196);



		  	// size_data_adc1 = sizeof(data_adc1)/sizeof(data_adc1[0]); // делаем один раз при инициализации
			for (uint16_t ic =0; ic < size_data_adc1  ; ic++)//обнулим место для нового пересчета, время на это == 1447мкс(8196) ==200мкс(1024), методом по циклу, почти тоже самое время
				{
					data_adc1[ic] =0;
					data_adc2[ic] =0;
				}
	  } // ================================================================================== END  IF  ==========================

	  // здесь будет разборка условий перехода по частоте
	  // freq_new - вычислена выше для типа автоподдержания частоты
	  // если получаем бит сканирования по частоте,  cmd_set.cmd_flags.1
	  // ============================================  сканирование по частоте  ================================================
	  if (((cmd_set.cmd_flags & 0x0001) >0) & ((cmd_set.cmd_flags & 0x0002) >0)) // скольжение по частоте
	  {
		  if (htim17.Instance->CNT > (cmd_set.time_step *10) )
		  {
			  htim17.Instance->CNT =0;
			  old_freq_new = freq_new;
			  freq_new = freq_new + flag_napravlenia_scan * cmd_set.step;
			  if (freq_new > (cmd_set.freq_start + (cmd_set.step *(cmd_set.N_step-1))))
			  {
				  flag_napravlenia_scan = -1; // следующий степ будет в отрицательную сторону
			  }
			  if (freq_new < (cmd_set.freq_start + cmd_set.step))
			  {
				  flag_napravlenia_scan = 1; // следующий степ будет в положительную сторону
			  }
		  }
	  }
	  // ============================================  стоять на одной частоте  ================================================
	  if (((cmd_set.cmd_flags & 0x0001) >0) & ((cmd_set.cmd_flags & 0x0002) ==0) & ((cmd_set.cmd_flags & 0x0008) ==0)) // стоять на одной частоте
	  {
		  old_freq_new = freq_new;
		  freq_new = cmd_set.freq_start;
	  }
	  // ============================================  автоподстройка частоты  ================================================
	  if (((cmd_set.cmd_flags & 0x0001) >0) & ((cmd_set.cmd_flags & 0x0002) ==0) & ((cmd_set.cmd_flags & 0x0008) >0)) // автоподстройка частоты
	  {
			if (old_freq_new >0)// надо проверить, чтобы не выскакивало за пределы
				{
					if (freq_new >14500)
						{
							if (freq_new <43000)
								{ freq_new = (freq_new + old_freq_new) /2;  }
							else
								{ freq_new = 25000; }
						}
					else
					{ freq_new = 25000; }
				}
			old_freq_new = freq_new;
			err_f[9] = err_f[8];
			err_f[8] = err_f[7];
			err_f[7] = err_f[6];
			err_f[6] = err_f[5];
			err_f[5] = err_f[4];
			err_f[4] = err_f[3];
			err_f[3] = err_f[2];
			err_f[2] = err_f[1];
			err_f[1] = err_f[0];
			err_f[0] = freq_new;
			delta_F = fabsf (freq_new - (err_f[0] +err_f[1] +err_f[2] +err_f[3] +err_f[4] +err_f[5] +err_f[6] +err_f[7] +err_f[8] +err_f[9])/10);
	  }
	  // ========================== а вот ОНО, - новая частота  ============================
	  hrtim_period_new = 5440000000 / ((uint32_t) freq_new) / (htim1.Instance->ARR +1) ;

/*	  if (htim7.Instance->CNT >10000) // tim7 тактуется 10 kHz,  будет включать барабанную передачу при необходимости
	  {
		  htim7.Instance->CNT =0;

		  cmd_array_SPI[0]=257;
		  cmd_array_SPI[1]=258;
		  cmd_array_SPI[2]=259;
		  cmd_array_SPI[3]=260;
		  cmd_array_SPI[4]=261;
		  cmd_array_SPI[5]=262;
		  cmd_array_SPI[6]=263;
		  cmd_array_SPI[7]=264;

		  if (status_TX_LPUART == HAL_OK)
		  {
			  status_TX_LPUART = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *) cmd_array_SPI, len_cmd_array_SPI);
			  //status_TX_LPUART = HAL_UART_Transmit(&hlpuart1, (uint8_t *) cmd_array_SPI, len_cmd_array_SPI, 100);
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);  // Led green
			  printf("_status_TX_LPUART == HAL_OK \n");
		  }
		  else
		  {
			  if (status_TX_LPUART == HAL_BUSY) { printf("_status_TX_LPUART == HAL_BUSY \n");}
			  if (status_TX_LPUART == HAL_ERROR) { printf("_status_TX_LPUART == HAL_ERROR \n");}
			  if (status_TX_LPUART == HAL_TIMEOUT) { printf("_status_TX_LPUART == HAL_TIMEOUT \n");}
			  HAL_UART_DMAStop(&hlpuart1);
			  HAL_UART_AbortTransmit(&hlpuart1);
			  status_TX_LPUART = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *) cmd_array_SPI, len_cmd_array_SPI);
		  }
		 //status_TX_LPUART = HAL_UART_Transmit(&hlpuart1, (uint8_t *) cmd_array_SPI, len_cmd_array_SPI, 1000);
		  //USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *) HEAP_arr_ADC_zamer, 512); //LENGTH_SAMPLES*2  );
		  //status_USBcdc_TX = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
		  htim7.Instance->CNT =0;
	  }
*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ----------------------- Callback -----------------------------------------*/

/*! \defgroup modbus_registers Modbus Registers
 * \code #include "mb.h" \endcode
	Стек протоколов внутренне не выделяет никакой памяти для регистров.
	Это делает стек протоколов очень маленьким, а также пригодным для использования в низкоуровневых целях.
	Кроме того, значения не обязательно должны находиться в памяти и могут, например, храниться во флэш-памяти.
	Всякий раз, когда стеку протоколов требуется значение,
	он вызывает одну из функций обратного вызова с адресом регистра и количеством регистров для чтения в качестве аргумента.
	Затем приложение должно считывать фактические значения регистра (например, напряжение АЦП) и сохранять результат в предоставленном буфере.
	Если стек протоколов хочет обновить значение регистра, поскольку была получена функция записи регистра,
	буфер с новыми значениями регистра передается функции обратного вызова.
	Затем функция должна использовать эти значения для обновления значений регистра приложения.
 */

/*  modbus_registers
 *   Функция Callback используется, если значение входного регистра Input_Register  требуется стеком протоколов.
 *   Адрес начального регистра задается адресом usAddress, а последний регистр задается ( usAddress + usNRegs - 1 ).
 *
 * \param pucRegBuffer Буфер, в который функция обратного вызова должна записывать текущее значение регистров modbus.
 * \param usAddress Начальный адрес регистра. Input registers находятся в пределах 1 - 65535.
 * \param usNRegs Количество регистров, которые должна предоставить функция обратного вызова.
 *
 * \return Функция должна возвращать один из следующих кодов ошибок:
 *   - eMBErrorCode::MB_ENOERR Если ошибка не произошла. В этом случае отправляется обычный ответ Modbus.
 *   - eMBErrorCode::MB_ENOREG Если приложение не может предоставить значения для регистров в пределах этого диапазона.
 *   					В этом случае в качестве ответа отправляется фрейм исключения ILLEGAL DATA ADDRESS ( НЕДОПУСТ�?МЫЙ АДРЕС ДАННЫХ ).
 *   - eMBErrorCode::MB_ETIMEDOUT Если запрошенный блок регистра в данный момент недоступен
 *   								и тайм-аут ответа, 	зависящий от приложения, будет нарушен.
 *   							В этом случае в качестве ответа отправляется исключение SLAVE DEVICE BUSY ( ВЕДОМОЕ УСТРОЙСТВО ЗАНЯТО ).
 *   - eMBErrorCode::MB_EIO Если произошла неустранимая ошибка.
 *   						В этом случае в качестве ответа отправляется исключение SLAVE DEVICE FAILURE ( СБОЙ ВЕДОМОГО УСТРОЙСТВА ).
 */
eMBErrorCode    eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	// эта фун осталась как заглушка, значения в буфер  pucRegBuffer  закладываются в конце обработки замера
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT number_data_array = usNRegs; // number_data_array - это будет номер элемента массива замеров, это будет собираться пакет из 10 массивов, из каждого этот элемент
    if (number_data_array > 1024) { number_data_array =1023; } // размер всех этих массивов = 1024
    // заморачиваться с сообщением об ошибке не будем, тупо отправить в ответ последние элементы пакета массивов, и все

	// DWT_CYCCNT =0;// обнуляем значение
	// DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; // включаем счётчик
	// Какой-то код
	//			 какой то код
    		if (cmd_set.step ==0) {cmd_set.step =1;}
			 index_temp = (freq_tim1 - cmd_set.freq_start) / cmd_set.step;
			 if (index_temp > 1000) { index_temp =1000; }
			 if (index_temp < 1) { index_temp =0; }

			 number_data_array=index_temp;

			 index_data_real_zamer ++; // в ноль будет сбрасываться при окончании пересчета массивов данных, И после передачи всего пакета
			 if (index_data_real_zamer > 1023) {index_data_real_zamer =1023; }

			 // график не рисуется, здесь идет значение частоты
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   -42 ошибка от реала, формируется здесь, выявлена при проверке через SpLab , значит на прием команды -42,  на передачу инфы +42 Гц
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   -42 ошибка от реала, формируется здесь, выявлена при проверке через SpLab , значит на прием команды -42,  на передачу инфы +42 Гц
			 if ((cmd_set.cmd_flags & 0x0010) >0)
			 	 { temp_float = (float32_t) index_data_real_zamer; } // если будем долбить по кругу реальные замеры по времени
			 else
			 	 { temp_float = (float32_t) (freq_tim1 +42 ); } // в этом месте всегда будет стоять текущая частота, пересчитаная из таймера назад
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 1 график, сверху вниз
			 temp_float = (float32_t) arr1_Output_f32[8]; //(zamer_adc_dma[number_data_array] & 0x0000FFFF);
			 //*pucRegBuffer++ = 	(uint8_t)number_data_array; // *(((uint8_t *) &temp_float) + 0); //
			 //*pucRegBuffer++ =	(uint8_t)number_data_array; //*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 2 график, сверху вниз
			 temp_float = (float32_t) arr2_Output_f32[8]; // ((zamer_adc_dma[number_data_array] >>16));
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 3 график, сверху вниз
			 temp_float = arr1_phase_Output_8_f32; // filter_adc1[number_data_array];
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 4 график, сверху вниз
			 //float ugol_rad = (float)(freq_tim1 -19500)*0.002 * M_PI - M_PI;
			 //temp_float = atan2f(cosf(ugol_rad), sinf(ugol_rad));     //
			 temp_float = arr2_phase_Output_8_f32; // filter_adc2[number_data_array];
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 5 график, сверху вниз
			 temp_float = temp_phase_f32; // (float32_t) arr1_Output_f32[number_data_array];
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 6 график, сверху вниз
			 temp_float = arr_power_Output_8_f32; // (float32_t) arr1_Output_f32[number_data_array];
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 3);

			 // 7 график, adc1_Tx - реальные фактические данные АЦП-1
			if (flag_data_complit_for_Tx ==0)
			 {temp_float = (float) (adc1_Tx[index_data_real_zamer]);}
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 8 график, data_adc2_Tx - фильтрованые  данные  после АЦП-1
			if (flag_data_complit_for_Tx ==0)
			 {temp_float = data_adc1_Tx[index_data_real_zamer];}
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 9 график, adc2_Tx - реальные фактические данные АЦП-2
			if (flag_data_complit_for_Tx ==0)
				{temp_float = (float) (adc2_Tx[index_data_real_zamer]);}
			 *pucRegBuffer++ = 	*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 // 10 график, data_adc2_Tx - фильтрованые  данные  после АЦП-2
			if (flag_data_complit_for_Tx ==0)
			 {temp_float = data_adc2_Tx[index_data_real_zamer];}
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  *(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	*(((uint8_t *) &temp_float) + 3);

			 temp_float = arr_power_Output_8_f32; // (float32_t) data_Y[number_data_array];
			 *pucRegBuffer++ = 	77;//*(((uint8_t *) &temp_float) + 0); //
			 *pucRegBuffer++ =	78;//*(((uint8_t *) &temp_float) + 1);
			 *pucRegBuffer++ =  79;//*(((uint8_t *) &temp_float) + 2); //
			 *pucRegBuffer++ =	80;//*(((uint8_t *) &temp_float) + 3);


			//	count_tic_finish = DWT_CYCCNT;//смотрим сколько натикало -цикл формирования массивов данных длится 4.894мкс (832 тика)
			//	count_tic_float_mks = (float)count_tic_finish * 1000000 / SystemCoreClock;

    return eStatus;
}

/*  modbus_registers
 * Функция Callback, используемая, если значение Holding Register, считывается или записывается стеком протокола
 * Адрес начального регистра задается USAddress, а последний регистр задается  USAddress + usNRegs - 1.
 *
 * \param pucRegBuffer Если значения регистров приложения должны быть обновлены, буфер указывает на новые значения регистров.
 * 						Если стеку протоколов необходимо сохранить текущие значения функция обратного вызова должна записать их в этот буфер
 * \param usAddress адрес первого Регистра.
 * \param usNRegs количество регистров read or write.
 * \param eMode If eMBRegisterMode::MB_REG_WRITE значения регистра приложения должны быть обновлены на основе значений в буфере.
 * 												Например, это может быть в том случае, когда мастер Modbus выдал  команду * <b>WRITE SINGLE REGISTER</b>.
 *   Если  eMBRegisterMode::MB_REG_READ   приложение должно скопировать текущие значения в буфер  pucRegBuffer.
 *
 * \return Функция должна возвращать один из следующих кодов ошибок:
 *   - eMBErrorCode::MB_ENOERR Если ошибка не произошла. В этом случае отправляется обычный ответ Modbus.
 *   - eMBErrorCode::MB_ENOREG Если приложение не может предоставить значения для регистров в пределах этого диапазона.
 *   							В этом случае в качестве ответа отправляется фрейм исключения ILLEGAL DATA ADDRESS ( НЕДОПУСТ�?МЫЙ АДРЕС ДАННЫХ )
 *   - eMBErrorCode::MB_ETIMEDOUT Если запрошенный блок регистра в данный момент недоступен и тайм-аут ответа, зависящий от приложения, будет нарушен.
 *   							В этом случае в качестве ответа отправляется исключение SLAVE DEVICE BUSY ( ВЕДОМОЕ УСТРОЙСТВО ЗАНЯТО ).
 *   - eMBErrorCode::MB_EIO Если произошла неустранимая ошибка.
 *   							В этом случае в качестве ответа отправляется исключение SLAVE DEVICE FAILURE ( СБОЙ ВЕДОМОГО УСТРОЙСТВА ).
 */
eMBErrorCode    eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress,  USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    // проверка пределов, и вгон в разрешенный диапазон,
    // будет отвечать ошибочными данными, но сам дебил кто дает ошибочный запрос
    if (usNRegs ==0) 	{ usNRegs =1; }
    if (usNRegs > REG_HOLDING_NREGS)	{ usNRegs =REG_HOLDING_NREGS; }
    if (usAddress < REG_HOLDING_START)	{ usAddress = usAddress; }
    if ((usAddress + usNRegs) > (REG_HOLDING_START + REG_HOLDING_NREGS)) { usAddress = REG_HOLDING_START - REG_HOLDING_NREGS; }

        iRegIndex = ( int )( usAddress - num_reg_CMD_Start );

            /* Pass current register values to the protocol stack.
             * Передайте текущие значения регистра в стек протоколов.*/
        if (eMode == MB_REG_READ)
        {
             while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( Reg_CMD_Buf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( Reg_CMD_Buf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
        } // if (eMode == MB_REG_READ)

            /* Update current register values with new values from the  protocol stack.
             * Обновите текущие значения регистра новыми значениями из стека протоколов. */
        if (eMode == MB_REG_WRITE)
        {
            while( usNRegs > 0 )
            {
                Reg_CMD_Buf[iRegIndex] = *pucRegBuffer++ << 8;
                Reg_CMD_Buf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            cmd_set.cmd_flags	= Reg_CMD_Buf[0];	//	Reg_CMD_Buf[0] - регистр флагов-команд, приходящих для исполнения
            cmd_set.proc_pwr	= Reg_CMD_Buf[1];	//	Reg_CMD_Buf[1] - регистр мощности,  2-98% заполнения
   // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   -42 ошибка от реала, формируется здесь, выявлена при проверке через SpLab , значит на прием команды -42,  на передачу инфы +42 Гц
            cmd_set.freq_start	= Reg_CMD_Buf[2] -42;	// Reg_CMD_Buf[2] - регистр стартовой частоты, 14500-43000
            cmd_set.step 		= Reg_CMD_Buf[3];	//	Reg_CMD_Buf[3] - регистр step(1-25гц) перемещения частоты, при сканировании диапазона, при сканировании старт будет Reg_CMD_Buf[2], максимум = (Reg_CMD_Buf[2] + step*_N-количество_)
            cmd_set.time_step	= Reg_CMD_Buf[4];	//	Reg_CMD_Buf[4] - регистр время милисекунд, между степами (10-1000мс)
            cmd_set.N_step		= Reg_CMD_Buf[5];	//	Reg_CMD_Buf[5] - регистр N-количество степов при сканировании 4-1000
            Reg_CMD_Buf[6] = Reg_CMD_Buf[5];
        } // if (eMode == MB_REG_WRITE)
    return eStatus;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// для контроля окончания передачи пакета на МВ485, используем прерывание от ножки PA11--PA12--DE--MB485
	// никаких разборок с EXTI нету,  там всего одно прерывание включено,
	xMBRTUTransmitFSM();
	flag_exti_stop_TX_cicles++;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    	{
    		counter_Tim6_MB++;
    		if(counter_Tim6_MB >=timeout_Tim6_50us)
			  { xMBRTUTimerT35Expired();   }
    	}
}


int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
		{ ITM_SendChar(*ptr++); }
	return len;
}


static void Mem_to_Mem_Complete(DMA_HandleTypeDef *hdma_memtomem_dma1_channel4)
{
	 // а сколько это мкс? в реале == 41 мкс 1024 WORD передача Mem_to_Mem;
	LL_ADC_REG_StartConversion(hadc1.Instance);  //  - это есть старт АЦП произвольно по желанию, если был остановлен
	flag_end_FFT =0;
}

// count_tic_start = DWT_CYCCNT;
//			 какой то код
// count_tic_finish = DWT_CYCCNT - count_tic_start;//смотрим сколько натикало  - ццикл отсоса массивов данных длится 16мкс
// count_tic_float_mks = (float)count_tic_finish * 1000000 / SystemCoreClock;

void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// LL_ADC_REG_StopConversion (hadc1.Instance);
	HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel4, (uint32_t)zamer_adc1_2, (uint32_t)zamer_adc_dma, LENGTH_SAMPLES);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // CDCDataReceivedCallback(uint8_t * Buf, uint16_t len)
{
//	status_TX_LPUART = HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *) cmd_array_SPI, len_cmd_array_SPI);
//			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Led green
//	for (uint16_t i=0; i< len; i++)
//	{
//		HEAP_arr_ADC_zamer[i + LENGTH_SAMPLES *4 ] = Buf[i];
//	}
//	// status_USBcdc_TX = CDC_Transmit_FS((uint8_t *) HEAP_arr_ADC_zamer, 64);
	return;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// включаем прием LPUART, ждём настроечных данных
//	status_RX_LPuart = HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *) cmd_array_SPI, len_cmd_array_SPI);
//	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Led green
	return;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	uint16_t count_printf;
  __disable_irq();
  while (1)
  {
	  count_printf++;
	  if (count_printf >1000)
	  	  {
		  	printf("MAIN Error_Handler() Error_Handler_Debug\n");
		  	count_printf =0;
	  	  }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
