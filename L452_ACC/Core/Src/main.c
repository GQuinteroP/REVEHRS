/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "arm_math.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "time.h"
#include "string.h"
#include "m95p32.h"
#include "EEPROM.h"
#include "stm32l4xx_ll_rcc.h"
#include "usb_device.h"
#include "ADXL355.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for main_task */
osThreadId_t main_taskHandle;
const osThreadAttr_t main_task_attributes = {
  .name = "main_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_sampling */
osThreadId_t task_samplingHandle;
const osThreadAttr_t task_sampling_attributes = {
  .name = "task_sampling",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for usb_cmd_queue */
osMessageQueueId_t usb_cmd_queueHandle;
const osMessageQueueAttr_t usb_cmd_queue_attributes = {
  .name = "usb_cmd_queue"
};
/* Definitions for mutex_buffer */
osMutexId_t mutex_bufferHandle;
const osMutexAttr_t mutex_buffer_attributes = {
  .name = "mutex_buffer"
};
/* Definitions for sem_mem_write */
osSemaphoreId_t sem_mem_writeHandle;
const osSemaphoreAttr_t sem_mem_write_attributes = {
  .name = "sem_mem_write"
};
/* Definitions for sem_mem_read */
osSemaphoreId_t sem_mem_readHandle;
const osSemaphoreAttr_t sem_mem_read_attributes = {
  .name = "sem_mem_read"
};
/* Definitions for sem_usb */
osSemaphoreId_t sem_usbHandle;
const osSemaphoreAttr_t sem_usb_attributes = {
  .name = "sem_usb"
};
/* Definitions for sem_SPI_DMA */
osSemaphoreId_t sem_SPI_DMAHandle;
const osSemaphoreAttr_t sem_SPI_DMA_attributes = {
  .name = "sem_SPI_DMA"
};
/* Definitions for sem_processing */
osSemaphoreId_t sem_processingHandle;
const osSemaphoreAttr_t sem_processing_attributes = {
  .name = "sem_processing"
};
/* Definitions for sem_sampling */
osSemaphoreId_t sem_samplingHandle;
const osSemaphoreAttr_t sem_sampling_attributes = {
  .name = "sem_sampling"
};
/* USER CODE BEGIN PV */

/* Definitions for usb_task */
osThreadId_t usb_taskHandle;
const osThreadAttr_t usb_task_attributes = { .name = "usb_task", .stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityAboveNormal,};

/* Definitions for IoT_task */
osThreadId_t IoT_taskHandle;
const osThreadAttr_t IoT_task_attributes = { .name = "IoT_task", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal,};

/* Definitions for offline_task */
osThreadId_t offline_taskHandle;
const osThreadAttr_t offline_task_attributes = { .name = "offline_task", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal,};

/* Definitions for gnss_task */
osThreadId_t gnss_taskHandle;
const osThreadAttr_t gnss_task_attributes = { .name = "gnss_task", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal,};

const osThreadAttr_t data_send_task_attributes = { .name = "task_data_send", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal,};

const osThreadAttr_t data_store_task_attributes = { .name = "task_data_store", .stack_size = 512 * 4,
		.priority = (osPriority_t) osPriorityNormal,};

RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;
struct tm time_ref;

//Config values
uint32_t BankNumber = 0;
uint32_t Address = 0, PAGEError = 0;
__IO uint32_t MemoryProgramStatus = 0;
__IO uint64_t data64 = 0;
static FLASH_EraseInitTypeDef EraseInitStruct;
const uint64_t Data64_To_Prog[FLASH_ROW_SIZE] = {
  0x1234567890123456, 0x1111111111111111, 0x2222222222222222, 0x3333333333333333,
  0x4444444444444444, 0x5555555555555555, 0x6666666666666666, 0x7777777777777777,
  0x8888888888888888, 0x9999999999999999, 0xAAAAAAAAAAAAAAAA, 0xBBBBBBBBBBBBBBBB,
  0xCCCCCCCCCCCCCCCC, 0xDDDDDDDDDDDDDDDD, 0xEEEEEEEEEEEEEEEE, 0xFFFFFFFFFFFFFFFF,
  0x0011001100110011, 0x2233223322332233, 0x4455445544554455, 0x6677667766776677,
  0x8899889988998899, 0xAABBAABBAABBAABB, 0xCCDDCCDDCCDDCCDD, 0xEEFFEEFFEEFFEEFF,
  0x2200220022002200, 0x3311331133113311, 0x6644664466446644, 0x7755775577557755,
  0xAA88AA88AA88AA88, 0xBB99BB99BB99BB99, 0xEECCEECCEECCEECC, 0xFFDDFFDDFFDDFFDD};

//USB
uint8_t usb_cmd[buffer_size];
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t USB_plugged = 0;

uint8_t LPWGNS_config = false;
uint8_t buffer_rx[2];
uint8_t buffer_lpwa_old[buffer_lwpa_config_size] = {0};
uint16_t buffer_lpwa_len = 0;
uint16_t write_lpwa_index = 0;
uint16_t read_lpwa_index = 0;

uint8_t gps_enabled = false;		//Use gps coordinates
uint8_t lpwa_enabled = false;		//Use gps coordinates
float32_t cal = 0; //21.8
uint16_t LeqTime = 1;  	//Leq time period - '0' value indicates fast Leq (Up to 1 hour - 3600 seconds)
uint16_t RecTime = 0;    	//Recording time - '0' value indicates push button control recording
uint8_t octave = true;  //'True' or 'false' calculate octave bands

//Variables
time_t endTime;     //End time of recording
uint8_t start=false;   //Flag to keep track of measuring status
uint8_t	sampling_buffer[fast_125ms*4]; // fast_125ms samples of 32 bits (4 bytes) - 24 bits data
float32_t working_buffer[fast_125ms + 96] = {0};
float32_t aux_low_freqs[4][250];
float32_t acc_dec_buffer[fast_125ms + 96] = {0};//__attribute__((section(".sram2"))) = {0};	//TODO: Ram2 only working for less than 2000 samples
uint8_t acc_dec_ctr = 0;
uint32_t seconds =  0;

struct BUFF
{
	uint16_t bufferLength;	//Actual buffer length
	uint16_t writeIndex;	 //	Increase writeIndex position to prepare for next write
	uint16_t readIndex;
	uint8_t  bufferMain[max_msgs][buffer_size];	//Five commands of maximum 40 chars
};

struct BUFF USB_msgs;
osStatus_t Queue_put(struct BUFF *queue, uint8_t *msg_in, uint8_t msg_len);
osStatus_t Queue_get(struct BUFF *queue, uint8_t *msg_out);

#ifndef MIC
	float32_t sine_working_buffer[fast_125ms + 96] = {0};
#endif


//Measurement data
uint32_t data_ctr=0;
int32_t data_array[data_buff_len];

float32_t	input_f32 [2048]; //input para fft
float32_t	input_buf [2048]; //input para fft
float32_t	output_f32 [2048]; //2048 output crudo de fft

float32_t	output_rfft [2048]; //2048 output final de fft

float32_t rfft_angle [2048];
float32_t signo [2048];

float32_t	inputIFFT_f32 [2048];	//input para ifft

float32_t	output_signo[2048];

float32_t	inputIFFT_f32_mod [2048];	//input para ifft
float32_t	outputIFFT_f32 [2048];	// output de ifft

float32_t	output_Run [2048];	  //input para fft de tercios de octava FFT
float32_t	output_Run_mag [2048];//output de magnitud

float32_t	max_mtvv [1];	  //input para fft de tercios de octava FFT

float H [2048];	//ponderacion para las frecuencias  SE CAMIO DE 1024 a 2048
//double Hp [2048];	//ponderacion para las frecuencias
//float Hh [2048];	//banda paso alto
//float Hl [2048]; 	//banda paso bajo
//float Ht [2048];	//ponderacion pura
float f [2048];	// frecuencia de lectura
float f1;	// para Hh
float f2;	//para Hl
float f3;	//para Ht

float32_t 	RunRMS[2048];
float32_t 	RunRunRMS[2048];

float32_t Octave[1024];

const float32_t Nband[25]={

		1,// Center 	1	Hz
		3,// Center 	1.25Hz
		3,// Center 	1.6	Hz
		3,// Center 	2	Hz
		5,// Center 	2.5	Hz

		6,// Center 	3.15Hz
		7,// Center 	4	Hz
		9,// Center 	5	Hz
		12,// Center 	6.3	Hz
		15,// Center 	8	Hz

		18,// Center 	10	Hz
		23,// Center 	12.5Hz
		30,// Center 	16	Hz
		37,// Center 	20	Hz
		46,// Center 	25	Hz

		59,// Center 	31.5Hz
		73,// Center 	40	Hz
		92,// Center 	50	Hz
		117,// Center 	63	Hz
		146,// Center 	80	Hz

		184,// Center 	100	Hz
		128,// Center 	125	Hz
		0,// Center 	160	Hz
		0,// Center 	200	Hz
		0,// Center 	250	Hz
		};

const float32_t NPosition[25]={

		4,// Center 	1	Hz
		5,// Center 	1.25Hz
		6,// Center 	1.6	Hz
		8,// Center 	2	Hz
		9,// Center 	2.5	Hz

		12,// Center 	3.15Hz
		15,// Center 	4	Hz
		18,// Center 	5	Hz
		23,// Center 	6.3	Hz
		29,// Center 	8	Hz

		36,// Center 	10	Hz
		45,// Center 	12.5Hz
		57,// Center 	16	Hz
		72,// Center 	20	Hz
		90,// Center 	25	Hz

		113,// Center 	31.5Hz
		142,// Center 	40	Hz
		179,// Center 	50	Hz
		225,// Center 	63	Hz
		284,// Center 	80	Hz

		357,// Center 	100	Hz
		449,// Center 	125	Hz
		566,// Center 	160	Hz
		712,// Center 	200	Hz
		896,// Center 	250	Hz
		};

int16_t readIndex = 0;
int16_t writeIndex = 0;
int16_t bufferLength = 0;	//Number of elements in buffer

int16_t	eeprom_data_len = 0;
int16_t eeprom_read_idx = 0;
int16_t eeprom_write_idx = 0;
uint8_t eeprom_aux_buffer[M95P32_PAGESIZE];



//DEBUG
uint32_t benchmark_ctr = 0;
uint32_t benchmark_time = 0;
uint32_t tcount=1;
uint32_t prevTime = 0;
uint8_t tmp_leq[200];
uint8_t tmp_gps[200];
float32_t delta = (2*M_PI*63)/32000;
float32_t out_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM16_Init(void);
void task_main(void *argument);
void sampling_task(void *argument);

/* USER CODE BEGIN PFP */
void task_gnss(void *argument);
void kappa(const char *fmt, ...);
void stopSampling();
void restart_UART_DMA();
void buffer_show();
void loadValues();
void store_config_64(uint64_t config_data);
void green_led(uint8_t status);
void red_led(uint8_t status);
void load_config(float32_t *cal, uint8_t *gps_enabled, uint8_t *lpwa_enabled,
			uint8_t *octave, uint16_t *RecTime, uint16_t	*LeqTime);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void kappa(const char *fmt, ...)
{
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);
	int len = strlen(buffer);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, -1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	#ifdef LOW_FREQ
	  MX_GPIO_Init();
	  MX_USART1_UART_Init();

	  HAL_Delay(100);
	  SET_BIT(PWR->CR2, PWR_PVM_1);
	  HAL_Delay(500);
	  if (!HAL_IS_BIT_SET(PWR->SR2, PWR_SR2_PVMO1)) //Detect USBVDD
	  {
		  kappa("\r\n USB ON");
		  SystemClock_Config_HIGH();
		  MX_GPIO_Init();
		  MX_DMA_Init();
		  MX_RTC_Init();
		  MX_USART1_UART_Init();
		  MX_SPI1_Init();
		  MX_USB_DEVICE_Init();
	  }else
	  {
		  kappa("\r\n USB OFF");
		  CLEAR_BIT(PWR->CR2, PWR_PVM_1);
		  #ifdef PSD_ACC_DEC
		  	  SystemClock_Config_LOW();	//16 MHz
		  	  MX_GPIO_Init();
			  MX_DMA_Init();
			  MX_RTC_Init();
		  #else
		  	  SystemClock_Config_LOW_IIR();	//25 MHz
		  	  MX_GPIO_Init();
			  MX_DMA_Init();
			  MX_LPUART1_UART_Init();
			  MX_RTC_Init();
		  	  MX_SAI1_Init_Low_IIR();
		  #endif
		  MX_USART1_UART_Init();
		  MX_SPI1_Init();
	  }
	#else
	  //SystemClock_Config_HIGH();
	  MX_GPIO_Init();
	  MX_DMA_Init();
	  MX_RTC_Init();
	  MX_USART1_UART_Init();
	  MX_SPI1_Init();
	  SET_BIT(PWR->CR2, PWR_PVM_1);
	  HAL_Delay(500);
	  if (!HAL_IS_BIT_SET(PWR->SR2, PWR_SR2_PVMO1)) //Detect USBVDD
		  MX_USB_DEVICE_Init();
	#endif

  //GPIO_ConfigAN();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_SPI2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mutex_buffer */
  mutex_bufferHandle = osMutexNew(&mutex_buffer_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem_mem_write */
  sem_mem_writeHandle = osSemaphoreNew(1, 0, &sem_mem_write_attributes);

  /* creation of sem_mem_read */
  sem_mem_readHandle = osSemaphoreNew(1, 0, &sem_mem_read_attributes);

  /* creation of sem_usb */
  sem_usbHandle = osSemaphoreNew(1, 0, &sem_usb_attributes);

  /* creation of sem_SPI_DMA */
  sem_SPI_DMAHandle = osSemaphoreNew(1, 0, &sem_SPI_DMA_attributes);

  /* creation of sem_processing */
  sem_processingHandle = osSemaphoreNew(1, 0, &sem_processing_attributes);

  /* creation of sem_sampling */
  sem_samplingHandle = osSemaphoreNew(1, 0, &sem_sampling_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of usb_cmd_queue */
  usb_cmd_queueHandle = osMessageQueueNew (30, sizeof(uint8_t), &usb_cmd_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of main_task */
  main_taskHandle = osThreadNew(task_main, NULL, &main_task_attributes);

  /* creation of task_sampling */
  task_samplingHandle = osThreadNew(sampling_task, NULL, &task_sampling_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 14;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV19;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;//RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  time_ref.tm_hour = 0x00;
  time_ref.tm_min = 0x00;
  time_ref.tm_sec = 0x00;
  time_ref.tm_mday = 0x01;
  time_ref.tm_mon = 0x01;
  time_ref.tm_year = 0x22;

  /*RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  time.Hours = 0x11;
  time.Minutes = 0x55;
  time.Seconds = 00;
  date.WeekDay = RTC_WEEKDAY_TUESDAY;
  date.Year = 0x23;
  date.Month = RTC_MONTH_JUNE;
  date.Date	= 0x07;
  HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BCD);
  HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BCD);*/

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 64000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 400000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STATUS_P_Pin|STATUS_N_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB_ST_Pin */
  GPIO_InitStruct.Pin = PB_ST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PB_ST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_P_Pin */
  GPIO_InitStruct.Pin = STATUS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(STATUS_P_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_N_Pin */
  GPIO_InitStruct.Pin = STATUS_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_N_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	kappa("Alarm!\r\n");
}

time_t rtc_read(void)
{
	struct tm timeinfo;
	time_t	t;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	timeinfo.tm_mon  = rtc_date.Month -1 ;
	timeinfo.tm_mday = rtc_date.Date;
	uint16_t y = rtc_date.Year;
	timeinfo.tm_year =  (uint16_t)(y+2000-1900);
	timeinfo.tm_hour = rtc_time.Hours;
	timeinfo.tm_min  = rtc_time.Minutes;
	timeinfo.tm_sec  = rtc_time.Seconds;

	t = mktime(&timeinfo);

	return t;
}

osStatus_t Queue_put(struct BUFF *queue, uint8_t *msg_in, uint8_t msg_len)
{
	if (queue->bufferLength == max_msgs)
	{
		#ifdef DEBUG
			kappa("\r\nBuffer is full!");
		#endif
		return osError;
	}
	queue->bufferMain[queue->writeIndex][0] = msg_len;
	//memset(queue->bufferMain[queue->writeIndex], 0, buffer_size);	TODO: Make this work//Clear buffer for previous data
	memcpy(&queue->bufferMain[queue->writeIndex][1], msg_in, msg_len);

	queue->bufferLength++;	 //	Increase buffer size after writing
	queue->writeIndex++;	 //	Increase writeIndex position to prepare for next write

	// If at last index in buffer, set writeIndex back to 0
	if (queue->writeIndex == max_msgs)
		queue->writeIndex = 0;

	return osOK;
}

osStatus_t Queue_get(struct BUFF *queue, uint8_t *msg_out)
{
	if (queue->bufferLength == 0)
	{
		#ifdef DEBUG
			kappa("\r\nBuffer is empty!");
		#endif
		return osError;
	}

	memcpy(msg_out, queue->bufferMain[queue->readIndex], queue->bufferMain[queue->readIndex][0]+1);
	queue->bufferLength--;	 //	Decrease buffer size after reading
	queue->readIndex++;	 //	Increase readIndex position to prepare for next read

	// If at last index in buffer, set readIndex back to 0
	if (queue->readIndex == max_msgs)
		queue->readIndex = 0;

	return osOK;
}

osStatus_t eeprom_write(uint8_t *data)
{
	if (eeprom_data_len == eeprom_size)
	{
		kappa("\r\nEEPROM is full!");
		return osError;
	}
	WRITE_ENABLE();
	if(Page_Write(data, eeprom_write_idx*M95P32_PAGESIZE, block_len*2)!=M95_OK)
	{
		kappa("\r\neeprom_write ERROR!");
		return osError;
	}
	//vTaskDelay(2);
	WRITE_DISABLE();

	eeprom_data_len++;	 //	Increase buffer size after writing
	eeprom_write_idx++;	 //	Increase writeIndex position to prepare for next write

	// If at last index in buffer, set writeIndex back to 0
	if (eeprom_write_idx == eeprom_size)
		eeprom_write_idx = 0;

	return osOK;
}

osStatus_t eeprom_read(uint8_t *data_out)
{
	if (eeprom_data_len == 0)
	{
		//kappa("\r\nEEPROM is empty!");
		return osError;
	}

	if(Single_Read(data_out, eeprom_read_idx*M95P32_PAGESIZE, block_len*2)!=M95_OK)
	{
		//kappa("\r\neeprom_read ERROR!");
		return osError;
	}

	eeprom_data_len--;	 //	Decrease buffer size after reading
	eeprom_read_idx++;	 //	Increase readIndex position to prepare for next read

	// If at last index in buffer, set readIndex back to 0
	if (eeprom_read_idx == eeprom_data_len)
		eeprom_read_idx = 0;

	return osOK;
}

void task_usb(void *argument)
{
	/* USER CODE BEGIN task_usb */
	uint16_t data_len = 0, tout = 100;
	uint64_t p;

	uint64_t config_val = 0;

	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(sem_usbHandle, osWaitForever);
		#ifdef debug_USB
			//kappa("\r\ncmd0:%02x%02x%02x", usb_cmd[0], usb_cmd[1], usb_cmd[2]);
		#endif

		kappa("\r\nQueue out [%d]: ", USB_msgs.bufferLength);
		while(USB_msgs.bufferLength>0)
		{
			Queue_get(&USB_msgs, usb_cmd);
			for(int ii=0;ii<usb_cmd[0];ii++)
				kappa("%c", usb_cmd[ii+1]);
		}

		switch(usb_cmd[1])
		{
			case 0xFF:	//Validate PC connection sending the random bytes back
				CDC_Transmit_FS(&usb_cmd[1], usb_cmd[0]);
				#ifdef debug_USB
					kappa("\r\nConnected!");
				#endif
			break;

			case 0x01:	//Get EEPROM data length
				data_len = get_eeprom_data_len();
				#ifdef debug_USB
					kappa("\r\nData_len: %d", data_len);
				#endif
				CDC_Transmit_FS((uint8_t*)&data_len, 2);
			break;

			case 0x02: //Read EEPROM data
				memcpy(&data_len, &usb_cmd[2], 2);
				Single_Read(eeprom_aux_buffer, data_len*M95P32_PAGESIZE, block_len*2);
				#ifdef debug_USB
					kappa("\r\nSel data: %d", data_len);
					kappa("\r\nRead: ");
					for(int ii=0;ii<block_len*2;ii++)
						kappa("%02x", eeprom_aux_buffer[ii]);
				#endif
				CDC_Transmit_FS(eeprom_aux_buffer, block_len*2);
			break;

			case 0x03:	//Enter LPWA configuration
				memcpy(&tout, &usb_cmd[2], 2);

				#ifdef debug_USB
					kappa("\r\nIoT Config! (Tout=%d)", tout);
				#endif

			break;

			case 0x04:	//Get configuration value from flash memory
				p = *(uint64_t *)ADDR_FLASH_PAGE_252;
				#ifdef debug_USB
					kappa("\r\nConfig. value:%lx - %lx",(uint32_t)(p>>32), (uint32_t)(p&0xFFFFFFFF));
				#endif
				CDC_Transmit_FS((uint8_t*)&p, 8);
			break;

			case 0x05:	//Receive 64bit configuration data and store it in flash

				memcpy(&config_val, &usb_cmd[2], 8);

				#ifdef debug_USB
					kappa("\r\nConfig_val: %d", config_val);
				#endif
				store_config_64(config_val);
				vTaskDelay(100);
				load_config(&cal, &gps_enabled, &lpwa_enabled, &octave, &RecTime, &LeqTime);
				#ifdef debug_USB
					kappa("\r\nCal: %f, gps:%d, lwpa:%d, octave:%d, Rec:%d, Leq:%d", cal, gps_enabled, lpwa_enabled, octave, RecTime, LeqTime);
				#endif
			break;

			case 0x06:
				time.Hours = usb_cmd[2];
				time.Minutes = usb_cmd[3];
				time.Seconds = usb_cmd[4];
				date.WeekDay = usb_cmd[5];
				date.Year = usb_cmd[6];
				date.Month = usb_cmd[7];
				date.Date = usb_cmd[8];
				HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BCD);
				HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BCD);

				#ifdef debug_USB
					vTaskDelay(100);
					HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
					sprintf((char *)currTimeBuff, "%02d/%02d/%02d %02d:%02d:%02d %04lu/%04lu", rtc_date.Date, rtc_date.Month, rtc_date.Year,
								rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, rtc_time.SubSeconds, rtc_time.SecondFraction);
					kappa("\r\n%s", currTimeBuff);
					kappa("\r\nDatetime Configuration EXIT!");
				#endif
			break;

			case 0x07:
				//TODO: Return data to validate erase ok
				WRITE_ENABLE();
				Chip_Erase();
				WRITE_DISABLE();
				#ifdef debug_USB
					kappa("\r\nChip_Erase");
				#endif
			break;

			case 0x08:

			break;

			default:
				kappa("\r\nUnknown command!");
			break;
		}
	}
	/* USER CODE END task_usb */
}

void buffer_show()
{
	int16_t idx = readIndex;
	//kappa("\r\n\r\nBuffer: ");
	for(int jj=0;jj<bufferLength;jj++)
	{
		//kappa("\r\nBlock %d: ",jj);
		for(int ii=0;ii<block_len;ii++)
			//kappa("%02x", leqs_buffer_main[jj][ii]);
		idx++;
		if(idx == buffer_size)
			idx=0;
	}
	//kappa("\r\n");
}

void store_config_64(uint64_t config_data)
{
	uint8_t ctr = 0;
	HAL_StatusTypeDef stat = HAL_OK;
	if (HAL_FLASH_Unlock()!= HAL_OK)
			kappa("\r\nError unlocking memory...");
	else
		kappa("\r\nMEMORY UNLOCKED!");
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	//EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.Page	= 252;
	EraseInitStruct.Banks = FLASH_BANK_1;
	vTaskDelay(150);
	stat = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) ;
	while ((stat!= HAL_OK) & (ctr<3))
	{
		kappa("\r\nError clearing memory...");
		ctr++;
		vTaskDelay(150);
		stat = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) ;
	}
	if (stat==HAL_OK)
		kappa("\r\nMEMORY ERASED!");

	vTaskDelay(150);
	ctr=0;
	stat = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ADDR_FLASH_PAGE_252,  config_data);
	while ( (stat != HAL_OK) & (ctr<3))
	{
		kappa("\r\nError writing config (%d)...", stat);
		ctr++;
		vTaskDelay(150);
		stat = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ADDR_FLASH_PAGE_252,  config_data);
	}

	if (stat==HAL_OK)
		kappa("\r\nCONFIG STORED!");

	HAL_FLASH_Lock();

	kappa("\r\nStore_config_64 finished!");
}

void store_config(float32_t cal_loc, uint8_t gps_loc, uint8_t lpwa_loc,
		uint8_t octave_loc, uint16_t RecTime_loc, uint16_t LeqTime_loc)
{
	uint64_t data_out = ((*(uint64_t *)&cal_loc)<<32);
	uint32_t tmp = (gps_loc<<31) | ((lpwa_loc&0x1)<<30) | ((octave_loc&0x1)<<29)
				| ((RecTime_loc&0x1FFF)<<16) | ((LeqTime_loc&0xFFF)<<4);
	data_out |=tmp;
	store_config_64(data_out);
	////kappa("\r\nstore_config Done!");
}

void load_config(float32_t *cal_loc, uint8_t *gps_loc, uint8_t *lpwa_loc,
			uint8_t *octave_loc, uint16_t *RecTime_loc, uint16_t	*LeqTime_loc)
{
	Address =ADDR_FLASH_PAGE_252;
	uint64_t *p = (uint64_t *)ADDR_FLASH_PAGE_252;
	////kappa("\r\nload_config: %lx - %lx",(uint32_t)((*p)>>32), (uint32_t)((*p)&0xFFFFFFFF));
	uint32_t pt1 = (*p)>>32;
	uint32_t pt2 = (uint32_t)((*p)&0xFFFFFFFF);
	*cal_loc = *(float32_t*)&pt1;
	*gps_loc = (pt2>>31);
	*lpwa_loc = (pt2>>30)&0x01;
	*octave_loc = (pt2>>29)&0x01;
	*RecTime_loc = ((pt2>>16)&0x1FFF);
	*LeqTime_loc = ((pt2>>4)&0xFFF);
	////kappa("\r\nload_config Done!");
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	kappa("\r\nWakeup timer");
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_RTCEx_DeactivateWakeUpTimer(hrtc);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	osSemaphoreRelease(sem_SPI_DMAHandle);

	//TODO: IMPLEMENT SPI port identification
	//osSemaphoreRelease(sem_mem_writeHandle);
	//kappa("\r\nSPI Tx!");
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	osSemaphoreRelease(sem_SPI_DMAHandle);

	//TODO: IMPLEMENT SPI port identification
	//osSemaphoreRelease(sem_mem_readHandle);
	//kappa("\r\nSPI Rx!");
}

uint32_t ADXL355_SPI_Read_DMA(uint8_t ui8address) {

	HAL_StatusTypeDef status;
	uint8_t recieveData;
	uint8_t txData;

	txData = (ui8address << 1) | 1 ;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); //ON

	status = HAL_SPI_Transmit_DMA(&hspi2, &txData, 1);
	osSemaphoreAcquire(sem_SPI_DMAHandle, osWaitForever);

	status = HAL_SPI_Receive_DMA(&hspi2, &recieveData, 1);
	osSemaphoreAcquire(sem_SPI_DMAHandle, osWaitForever);

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); //OFF

	if (status == HAL_OK)
		return recieveData;
	else
		printf("\r\nError Reading: Invalid HAL_STATUS\r\n");

	return 255;
}

uint32_t ADXL355_SPI_Read(uint8_t ui8address) {

	HAL_StatusTypeDef status;
	uint8_t recieveData;
	uint8_t txData;

	txData = (ui8address << 1) | 1 ;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); //ON

	status = HAL_SPI_Transmit (&hspi2, &txData, 1, 100);
	status = HAL_SPI_Receive (&hspi2, &recieveData, 1, 100);

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); //OFF

	if (status == HAL_OK)
		return recieveData;
	else
		kappa("\r\nError Reading: Invalid HAL_STATUS\r\n");

	return 255;
}

/*
 * @param : ui8address - unsigned 8 bit integer that represents the address we will write to
 * @param : ui8Data - unsigned 8 bit integer that represents the data we will write into the corresponding address
 * @param : enMode - regarding how many bytes of data you will write, i chose to not really use this and hardcoded a
 *                   1 for '1 byte' in every write I ever used
 *
 * This function is a callback for the accelerometer write
 */
void ADXL355_SPI_Write(uint8_t ui8address, uint8_t ui8Data, enWriteData enMode) {

	HAL_StatusTypeDef status;
	uint8_t address;

	address = ((ui8address << 1) & 0xFE);

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); //ON

	status = HAL_SPI_Transmit (&hspi2, &address, 1, 100);
	status = HAL_SPI_Transmit (&hspi2, &ui8Data, 1, 100);

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); //OFF

	if (status != HAL_OK)
		printf("\r\nError writing: Invalid HAL STATUS\r\n");
}

int32_t ADXL355_SPI_READ_ACC(uint8_t axis)	//Axis = 0x00 - x, 0x01 - y, 0x02 - z
{
	uint32_t acc_val_raw3 = 0, acc_val_raw2 = 0, acc_val_raw1 = 0;
	int32_t acc_val = 0;

	switch(axis)
	{
		case 0x00:
			acc_val_raw3 = ADXL355_SPI_Read_DMA(XDATA3);
			acc_val_raw2 = ADXL355_SPI_Read_DMA(XDATA2);
			acc_val_raw1 = ADXL355_SPI_Read_DMA(XDATA1);
		break;

		case 0x01:
			acc_val_raw3 = ADXL355_SPI_Read_DMA(YDATA3);
			acc_val_raw2 = ADXL355_SPI_Read_DMA(YDATA2);
			acc_val_raw1 = ADXL355_SPI_Read_DMA(YDATA1);
		break;

		case 0x02:
			acc_val_raw3 = ADXL355_SPI_Read_DMA(ZDATA3);
			acc_val_raw2 = ADXL355_SPI_Read_DMA(ZDATA2);
			acc_val_raw1 = ADXL355_SPI_Read_DMA(ZDATA1);

			/*acc_val_raw3 = ADXL355_SPI_Read(ZDATA3);
			acc_val_raw2 = ADXL355_SPI_Read(ZDATA2);
			acc_val_raw1 = ADXL355_SPI_Read(ZDATA1);*/
		break;
	}

	acc_val = ADXL355_Acceleration_Data_Conversion(
							((acc_val_raw3<<16) & 0x00FF0000) |
									((acc_val_raw2<<8) & 0x0000FF00) |
											(acc_val_raw1 & 0x000000FF));
	return acc_val;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		data_ctr = 0;

		HAL_TIM_Base_Start_IT(&htim16);
		kappa("Iniciado\r\n");
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//kappa("Timer\r\n");
	if(data_ctr < data_buff_len)
	{
		osSemaphoreRelease(sem_samplingHandle);

	}else
	{
		HAL_TIM_Base_Stop_IT(&htim16);
		osSemaphoreRelease(sem_processingHandle);
	}
}

void postprocessing()
{
	float acc_val_flt;
	float	output_INT ; //envio de datos
	float	output_Final ; //envío de datos
	float	Octavas_Final ; //envío de datos
	arm_rfft_fast_instance_f32 S;	//for FFT
	arm_rfft_fast_instance_f32 K;	// for InverseFFT
	arm_rfft_fast_instance_f32 M;	// for third-octave FFT

	for(int ii=0;ii<data_buff_len;ii++)
	{


		//input_buf[ii] = 1000*data_array[ii] / 26214.4;
		input_f32[ii]= 1000*((data_array[ii] / 26214.4)-9.722);
		#ifdef debug
			acc_val_flt = 1000*((data_array[ii] / 26214.4)-9.722);
			kappa("%d.%d\r\n", (int) acc_val_flt, (int) (fmodf(fabs(acc_val_flt), 1.0)*1));
		#endif
	}

	//// FFT ////
	arm_rfft_fast_init_f32(&S,2048); //Initialize the rFFT

	arm_rfft_fast_f32(&S, input_f32, output_f32, 0); //rFFT
	//arm_cmplx_mag_f32(output_f32, output_rfft, 1024);//1024 /magnitud porq tiene parte imaginaria

	////////////


	 ////// Sign from the input
		int sig=0;
		while (sig<2048)
	   {

			if(output_f32[sig]>=0)
			{
				signo[sig]=1;
				signo[sig+1]=1;
			}
			if(output_f32[sig]<0)
			{
				signo[sig]=-1;
				signo[sig+1]=-1;
			}

		 sig=sig+2;  	//incremento
	  //////////////////
	   }

	 ////// Angle (between real and imaginary data)
		int cc=0;
		int aa=0;
		while (cc<2048 && aa < 1024)
	   {
		rfft_angle[aa]=atan(output_f32[cc+1]/output_f32[cc]);
		cc=cc+2;  	//incremento
		aa++;
	  //////////////////
	   }

		//Modulo FFT (ocupo aqui el mismo array despeus de haber extraido el angulo y eel signo)
		arm_cmplx_mag_f32(output_f32, output_f32, 1024);//1024 /magnitud porq tiene parte imaginaria


		int q=0;  		 	//inicialización del contador
		while (q<1024) 	//condicion  (para array de volyaje j=4096)///////////
		   {
	///////////////PONDERACION ///////////////////
		 //ya que los 2048 datos de array representan 256Hz
		 //por tanto el dato 0 esta a 0.25Hz
		 //          el dato 1 esta a 0.50Hz, etc

		//Ponderacion: H= PasoAlto * PasoBajo * PondFrecPura
		//Ponderacion: H= Hh * Hl * Ht

		//FRECUENCIA para ponderacion
		f[q]=q*0.125; //0.125 // 1024/125 = 8 //calculo de la frecuencia de cada muestra

		///PASO ALTO///
			//f1=10^-0.1Hz
		f1= 0.79432823472428150206591828283639; //pow(10,-0.1)
		//Hh= sqrt((pow(f,4)/(pow(f,4)+pow(f1,4)));
		//Hh[q]= sqrt(pow(f[q],4)/(pow(f[q],4)+pow(f1,4)));

		///PASO BAJO///
		f2=100;
		//Hl= sqrt((pow(f2,4)/(pow(f,4)+pow(f2,4)));
		//Hl[q]= sqrt(pow(f2,4)/(pow(f[q],4)+pow(f2,4)));

		///PONDERACION PURA///
		f3=5.6841051104248334203172772633041; //   formula: 1/(0.028*(2*M_PI));
		// Hh= sqrt((pow(f3,2)/(pow(f,2)+pow(f3,2)));
		//Ht[q]= sqrt(pow(f3,2)/(pow(f[q],2)+pow(f3,2)));

		///FUNCION DE TRASNFERENCIA///
			///Transformar a dB, (20log(x))///
		//H[q]= (Hh[q])*(Hl[q])*(Ht[q]);     /// realizo en un solo paso el calculo
		H[q]= (sqrt(pow(f[q],4)/(pow(f[q],4)+pow(f1,4))))*(sqrt(pow(f2,4)/(pow(f[q],4)+pow(f2,4))))
			   *(sqrt(pow(f3,2)/(pow(f[q],2)+pow(f3,2))));
		//se convierte a Decibelios
		//Hp[q]= 20*log10(H[q]);   // se debe usar log10, porque log es LogaritNatural

		//Introducir ala salida compleja de FFT en ingreso de InverseFFT
		//y se multiplica por la ponderacio en frecuencia

		//inputIFFT_f32[q]=output_f32[q] * H[q];
		//inputIFFT_f32_mod[q]=output_rfft[q];
		output_f32[q]=(output_f32[q]*H[q])/1024;	//
		//output_f32[q]=(output_f32[q]*H[q])/1024;	//

			q++;  			//incremento

	//////////////////
		   }

	////// Complete FFT of 2048 despues de la ponderación

	//		int ww=0;
	//		while (ww<1024)
	//	   {
	//			inputIFFT_f32_mod[1024+ww]=inputIFFT_f32_mod[1023-ww];
	//	  	ww++;  	//increment

	//////////////////
	//	   }

	////// Calculation of real e imaginary part  (base on module & angle)

			int www=0;
			int aaa=0;
			while (www<2048 && aaa < 1024)
				 {
				#ifdef debug
					output_INT=output_f32[aaa];
					kappa("%d.%d\r\n", (int) output_INT, (int) (fmodf(fabs(output_INT), 1.0)*1000));
				#endif
				inputIFFT_f32[www]=(cos(rfft_angle[aaa]))*(output_f32[aaa]);
				//inputIFFT_f32[www]=(cos(rfft_angle[aaa]))*(inputIFFT_f32_mod[aaa]);
				inputIFFT_f32[www+1]=(sin(rfft_angle[aaa]))*(output_f32[aaa]);
				//inputIFFT_f32[www+1]=(sin(rfft_angle[aaa]))*(inputIFFT_f32_mod[aaa]);

				www=www+2;  	//increment
				aaa++;
	//////////////////
		   }

	//// AÑADIR SIGNO
			int sig2=0;
			while (sig2<2048)
		   {
			input_f32[sig2]= inputIFFT_f32[sig2]*signo[sig2];//se ocupa array anterior
			//output_signo[sig2]= inputIFFT_f32[sig2]*signo[sig2];

				sig2++;  	//incremento
		  //////////////////
		   }

	/////INVERSE FFT ///
	arm_rfft_fast_init_f32(&K,2048); ///Initialize the riFFT
	arm_rfft_fast_f32(&K, input_f32, outputIFFT_f32, 1); //ingreso del output complejo de FFT en la funcion IFFT


	#ifdef debug
		for(int iiii=0;iiii<2048;iiii++)
		{
		output_Final=1000*outputIFFT_f32[iiii];
		kappa("%d.%d\r\n", (int) output_Final, (int) (fmodf(fabs(output_Final), 1.0)*1000));
		}
	#endif

	//arm_rfft_fast_f32(&K, output_signo, outputIFFT_f32, 1); //ingreso del output complejo de FFT en la funcion IFFT
	////////////////////

	///////////////////// BUFFER  input RMS
//			int w=0;
//			while (w<2048)
//			   {
//				MovRmsInput[w]=1000*outputIFFT_f32[w];
//			  	w++;  			//incremento

	//////////////////
//			   }

	//////////////// RMS ////////////////

	   // Calculate RMS values
		for (int i = 0; i < 2048; i++)
		{						 // 256 es el numero de muestras en 1 segundo
								 // velocidad del RMS movil 0,125sec
			if (i <= (2048-32)) // 32 muestras (0,125sec)total menos intervalo de RMS
			{
				float suma_cuadrada = 0.0;
				for (int j = i; j < i+32; j++) // 32 samples = 0,125sec
				{
					suma_cuadrada += outputIFFT_f32[j] * outputIFFT_f32[j];
					}
					RunRMS[i] = sqrt(suma_cuadrada / 32); // 32 samples = 0,125sec
					//RunRunRMS[i] = sqrt(suma_cuadrada / 32);
						}
						if (i > (2048-32)) 	// 32 samples = 0,125sec
						{
							float suma_cuadrada = 0.0;
							for (int j = i; j < 2048; j++)
					 {
					suma_cuadrada += outputIFFT_f32[j] * outputIFFT_f32[j];
				}
				 RunRMS[i] = sqrt(suma_cuadrada / (2048 - i));
				 //RunRunRMS[i] = sqrt(suma_cuadrada / (2048 - i));
			 }
		}

		//// FFT para Tercios de octava////

		arm_rfft_fast_init_f32(&M,2048); //Initialize the rFFT

		arm_rfft_fast_f32(&M, RunRMS, output_Run, 0); //rFFT
		arm_cmplx_mag_f32(output_Run, output_Run_mag, 1024);//1024 /magnitud

		//////////////// OCTAVE ////////////////
		for (int i = 0; i < 24; i++)
		{
			// índice inicial y la cantidad de elementos para sumar
			int indice_inicial = NPosition[i];
			int elementos_a_sumar = Nband[i];

			// suma de los elementos
			float suma_amp = 0;
			for (int j = 0; j < elementos_a_sumar; j++)
			{
				suma_amp += output_Run_mag[indice_inicial + j];
			}

		// Almacenar la suma (Altura x Base)(Amplitude x Bandwidth)(amplitud del bin de frecuwncia)
		Octave[i] = suma_amp*Nband[i]*0.125; //sumAmplitud*NumeroBandas*AnchoFreq
		}


		//Extraccion y envio de MTVV
		for(int iiii=0;iiii<1;iiii++) // solo 1 dato
		{
			//Max value of spectrum after Running RMS
			arm_max_f32(output_Run_mag, 2048, max_mtvv, output_Run_mag);
			#ifdef debug
				Octavas_Final=max_mtvv[iiii];	//ocupo el mismo array que ocupare en las octavas mas adelante
				kappa("%d.%d\r MTVV\n", (int) Octavas_Final, (int) (fmodf(fabs(Octavas_Final), 1.0)*1));
			#endif
			//kappa("%d.%d\r\n", (int) output_INT, (int) (fmodf(fabs(output_INT), 1.0)*1000));
		}
		#ifdef debug
			//Envio de las 20 Octavas
			for(int iiii=0;iiii<20;iiii++) //para transmitir las 20 bandas (para llegar a 80Hz que interesa)
			{
				Octavas_Final=Octave[iiii];
				kappa("%d.%d\r\n", (int) Octavas_Final, (int) (fmodf(fabs(Octavas_Final), 1.0)*1000));
				//kappa("%d.%d\r\n", (int) output_INT, (int) (fmodf(fabs(output_INT), 1.0)*1000));
			}
		#endif
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_main */
/**
  * @brief  Function implementing the main_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_main */
void task_main(void *argument)
{
  /* USER CODE BEGIN 5 */

	kappa("\r\n****STARTING...!****");

	/* Infinite loop */
	//HAL_LPTIM_Counter_Start_IT(&hlptim1, 797);	//DEBUG - 0.01 ms
	//TODO:Find out why this helps to start with low current consumption
	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
	{
		kappa("\r\nUSB (%d, %d)",  hUsbDeviceFS.ep0_state, hUsbDeviceFS.dev_state);
		usb_taskHandle = osThreadNew(task_usb, NULL, &usb_task_attributes);
		USB_plugged = 1;

	} else {

	}

	vTaskDelay(1000);
	kappa("Program started!\r\n");
	ADXL355_Init();
	vTaskDelay(100);
	ADXL355_Start_Sensor();
	kappa("\r\nSYSTEM READY!\r\n");

	for (;;)
	{
		osSemaphoreAcquire(sem_processingHandle, osWaitForever);
		kappa("\r\nProcessing data!");

		postprocessing();

		kappa("\r\nDone!\r\n");
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		//HAL_PWREx_EnterSTOP2Mode(PWR_LOWPOWERREGULATOR_ON, 2);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_sampling_task */
/**
* @brief Function implementing the task_sampling thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sampling_task */
void sampling_task(void *argument)
{
  /* USER CODE BEGIN sampling_task */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(sem_samplingHandle, osWaitForever);
	  HAL_GPIO_TogglePin(STATUS_P_GPIO_Port, STATUS_P_Pin);
	  data_array[data_ctr] = ADXL355_SPI_READ_ACC(2);
	  data_ctr++;
  }
  /* USER CODE END sampling_task */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
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