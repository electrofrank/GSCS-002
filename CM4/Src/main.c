/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for Cortex M4 Auxiliary Core of EOS GNC
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include <retarget.h>
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/udp.h"
#include "app_ethernet.h"
#include <string.h>
#include "hx711.h"
#include "spp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  8)   /* Size of array aADCxConvertedData[] to be used with DMA */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

LPTIM_HandleTypeDef hlptim1;
LPTIM_HandleTypeDef hlptim2;

OPAMP_HandleTypeDef hopamp2;

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId Sens_Acq_TaskHandle;
osThreadId CommandRX_TaskHandle;
osThreadId TelemetryTX_TasHandle;
osThreadId ActuatorsControHandle;
osSemaphoreId InitSemaphoreHandle;
/* USER CODE BEGIN PV */


///////////  SENSORS GLOBAL VARIABLES /////////////////////////////
//Load Cell variable
hx711_t LOAD_CELL1; //LC1
hx711_t LOAD_CELL2; //LC2
hx711_t LOAD_CELL3; //LC3
hx711_t LOAD_CELL4; //LC4

float thrust[4];

float total_thrust;

#define SAMPLE_TIME 10 //ms

extern int load_cells_tare_flag = 0;

//CURRENT READING FROM IGNITION SMART SWITCH
float Ign_curr = 0;

//FLOW METERS
int cnt1=0,prev_cnt1=0,dcnt1=0,totalcnt1 = 0;
int cnt2,prev_cnt2,dcnt2 = 0,totalcnt2 = 0;
float totalvolume1 = 0;
float totalvolume2 = 0;
float vol_flow1 = 0; //MF1
float vol_flow2 = 0; //MF2
float prev_totalvolume1 = 0;
float prev_totalvolume2 = 0;

//TMC turbine flow meter
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0;

/* Measure Frequency */
#define TIMCLOCK   200000000
#define PRESCALER  19999

float frequency = 0;


//Analog sensors variables
ALIGN_32BYTES(static uint16_t ADC_ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);

#define PRESS_LOX_SENSITIVIY  3750// bar/A
#define PRESS_KERO_SENSITIVITY 3750

float pressure_LOX = 0; //PS1
float pressure_KERO = 0; //PS2
float pressure_CHAMBER = 0; //PS3

//Status Variables
int Valve1_Status,Valve2_Status;


/////////// NETWORK INTERFACE VARIABLES ////////////////////
struct netif gnetif;
ip_addr_t ipaddr;
ip_addr_t netmask;
ip_addr_t gw;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_LPTIM2_Init(void);
static void MX_OPAMP2_Init(void);
void StartDefaultTask(void const * argument);
void StartSens_Acq_Task(void const * argument);
void Start_CommandRX_Task(void const * argument);
void Start_TelemetryTX_Task(void const * argument);
void Start_ActuatorsControl_Task(void const * argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */

	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* Activate HSEM notification for Cortex-M4*/
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	/*
	 Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
	 perform system initialization (system clock config, external memory configuration.. )
	 */
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE,
	PWR_D2_DOMAIN);
	/* Clear HSEM flag */
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_LPTIM1_Init();
  MX_LPTIM2_Init();
  MX_OPAMP2_Init();
  /* USER CODE BEGIN 2 */

	MX_USART3_UART_Init(); //init uart3 also on CM4

	RetargetInit(&huart3);

	while (HAL_HSEM_IsSemTaken(HSEM_ID_0) == 0) {
		HAL_Delay(100);
	}

	printf("GSCS-002 CM4 Started\n");

	// Calibrate The ADC On Power-Up For Better Accuracy
	printf("GSCS-002 CM4 - Starting ADC1 calibration\n");

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
	ADC_SINGLE_ENDED);

	printf("GSCS-002 CM4 - ADC1 calibration done\n");

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of InitSemaphore */
  osSemaphoreDef(InitSemaphore);
  InitSemaphoreHandle = osSemaphoreCreate(osSemaphore(InitSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Sens_Acq_Task */
  osThreadDef(Sens_Acq_Task, StartSens_Acq_Task, osPriorityIdle, 0, 256);
  Sens_Acq_TaskHandle = osThreadCreate(osThread(Sens_Acq_Task), NULL);

  /* definition and creation of CommandRX_Task */
  osThreadDef(CommandRX_Task, Start_CommandRX_Task, osPriorityIdle, 0, 512);
  CommandRX_TaskHandle = osThreadCreate(osThread(CommandRX_Task), NULL);

  /* definition and creation of TelemetryTX_Tas */
  osThreadDef(TelemetryTX_Tas, Start_TelemetryTX_Task, osPriorityIdle, 0, 256);
  TelemetryTX_TasHandle = osThreadCreate(osThread(TelemetryTX_Tas), NULL);

  /* definition and creation of ActuatorsContro */
  osThreadDef(ActuatorsContro, Start_ActuatorsControl_Task, osPriorityIdle, 0, 128);
  ActuatorsControHandle = osThreadCreate(osThread(ActuatorsContro), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//handled by the callback, here i can operate on the buffer to parse the commands

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//after initialization this task can be removed
	}
  /* USER CODE END 3 */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim1.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief LPTIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM2_Init(void)
{

  /* USER CODE BEGIN LPTIM2_Init 0 */

  /* USER CODE END LPTIM2_Init 0 */

  /* USER CODE BEGIN LPTIM2_Init 1 */

  /* USER CODE END LPTIM2_Init 1 */
  hlptim2.Instance = LPTIM2;
  hlptim2.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim2.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim2.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim2.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim2.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim2.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim2.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  hlptim2.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim2.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM2_Init 2 */

  /* USER CODE END LPTIM2_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 200;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 200 - 1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535 - 1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 20000 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LOW_SIDE_SW_1_Pin|LOW_SIDE_SW_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOW_SIDE_SW_4_GPIO_Port, LOW_SIDE_SW_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LC4_CK_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LC3_CK_GPIO_Port, LC3_CK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LC1_CK_GPIO_Port, LC1_CK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, IGN_OUT_Pin|LOW_SIDE_SW_3_Pin|LC2_CK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LOW_SIDE_SW_1_Pin LOW_SIDE_SW_2_Pin */
  GPIO_InitStruct.Pin = LOW_SIDE_SW_1_Pin|LOW_SIDE_SW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LC2_DIN_Pin */
  GPIO_InitStruct.Pin = LC2_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LC2_DIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LOW_SIDE_SW_4_Pin */
  GPIO_InitStruct.Pin = LOW_SIDE_SW_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOW_SIDE_SW_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LC4_CK_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LC4_CK_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LC4_DIN_Pin LC3_DIN_Pin */
  GPIO_InitStruct.Pin = LC4_DIN_Pin|LC3_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LC3_CK_Pin */
  GPIO_InitStruct.Pin = LC3_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LC3_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LC1_CK_Pin */
  GPIO_InitStruct.Pin = LC1_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LC1_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LC1_DIN_Pin */
  GPIO_InitStruct.Pin = LC1_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LC1_DIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IGN_OUT_Pin LOW_SIDE_SW_3_Pin LC2_CK_Pin */
  GPIO_InitStruct.Pin = IGN_OUT_Pin|LOW_SIDE_SW_3_Pin|LC2_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Netif_Config(void) {

	/* Create tcp_ip stack thread */
	tcpip_init(NULL, NULL);

	IP_ADDR4(&ipaddr, 192, 168, 1, 104); //GNC Board IP
	IP_ADDR4(&netmask, 255, 255, 255, 0);
	IP_ADDR4(&gw, 192, 168, 1, 1);
	/* add the network interface */
	netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init,
			&ethernet_input);

	/*  Registers the default network interface. */
	netif_set_default(&gnetif);

	printf("GSCS-002 CM4 - Network Interface Configured\n");

	ethernet_link_status_updated(&gnetif);

#if LWIP_NETIF_LINK_CALLBACK
	netif_set_link_callback(&gnetif, ethernet_link_status_updated);

	osThreadDef(EthLink, ethernet_link_thread, osPriorityNormal, 0,
			configMINIMAL_STACK_SIZE *2);
	osThreadCreate(osThread(EthLink), &gnetif);
#endif

}

// UDP functions over lwIP


void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim15, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim15) < us)
		;  // wait for the counter to reach the us input in the parameter
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

}

void init_loadcells() {
	hx711_init(&LOAD_CELL1, LC1_CK_GPIO_Port, LC1_CK_Pin, LC1_DIN_GPIO_Port, LC1_DIN_Pin);
	hx711_init(&LOAD_CELL2, LC2_CK_GPIO_Port, LC2_CK_Pin, LC2_DIN_GPIO_Port, LC2_DIN_Pin);
	hx711_init(&LOAD_CELL3, LC3_CK_GPIO_Port, LC3_CK_Pin, LC3_DIN_GPIO_Port, LC3_DIN_Pin);
	hx711_init(&LOAD_CELL4, LC4_CK_GPIO_Port, LC4_CK_Pin, LC4_DIN_GPIO_Port, LC4_DIN_Pin);


	hx711_coef_set(&LOAD_CELL1, -45); // LC1
	hx711_coef_set(&LOAD_CELL2, -44.5 ); // set calibration coefficent
	hx711_coef_set(&LOAD_CELL3, -44.5); // set calibration coefficent
	hx711_coef_set(&LOAD_CELL4, -44.5); // set calibration coefficent

	hx711_tare(&LOAD_CELL1, 10); //read offset
	hx711_tare(&LOAD_CELL2, 10); //read offset
	hx711_tare(&LOAD_CELL3, 10); //read offset
	hx711_tare(&LOAD_CELL4, 10); //read offset
}

void compute_flow() {

	prev_cnt1 = cnt1;
	prev_cnt2 = cnt2;

	cnt1 = LPTIM1->CNT; //

	cnt2 = LPTIM2->CNT; //kerosene flow meter timer input

	//2.5mL/P

	dcnt1 = cnt1 - prev_cnt1;
	dcnt2 = cnt2 - prev_cnt2;

	totalcnt1 += dcnt1;
	totalcnt2 += dcnt2;

	prev_totalvolume1 = totalvolume1;
	prev_totalvolume2 = totalvolume2;


	totalvolume1 = ((totalcnt1 *(1/324.71))); //in Liters (LOX)

	totalvolume2 = ((totalcnt2*(1/800.111))); //in Liters (KEROSENE)


	vol_flow1 = (totalvolume1 - prev_totalvolume1)*(1000.0/SAMPLE_TIME);

	vol_flow2 = (totalvolume2 - prev_totalvolume2)*(1000.0/SAMPLE_TIME);


	printf("%.3f Liters - %.3f L/s \N",totalvolume1,vol_flow1);

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (Is_First_Captured==0) // if the first rising edge is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
		}

		else   // If the first rising edge is captured, now we will capture the second edge
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffffffff - IC_Val1) + IC_Val2;
			}

			float refClock = TIMCLOCK/(PRESCALER);

			frequency = refClock/Difference;

			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_First_Captured = 0; // set it back to false
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  /* USER CODE BEGIN 5 */
	printf("\nGSCS_002 CM4 - Main Task Started\n");

	/* Infinite loop */

	Netif_Config();

	udpClient_connect();

	//other tASK SHOULD WAIT THIS POINT, relase the semaphore

	/* Infinite loop */
	for (;;) {
		osDelay(100);

	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSens_Acq_Task */
/**
 * @brief Function implementing the Sens_Acq_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSens_Acq_Task */
void StartSens_Acq_Task(void const * argument)
{
  /* USER CODE BEGIN StartSens_Acq_Task */

	osDelay(500); //Thermocouple Power-up Time 200 ms, HX711 400 ms

	printf("\nDAQ Task Started\n");

	HAL_TIM_Base_Start(&htim15); //timer used for us delay

	HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);


	if(HAL_LPTIM_Counter_Start(&hlptim1,0XFFFF) != HAL_OK) printf("Error Starting LPTIM1\n");
	if(HAL_LPTIM_Counter_Start(&hlptim2,0XFFFF) != HAL_OK) printf("Error Starting LPTIM2\n");

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_ConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE) != HAL_OK) {
		printf("GSCS-002 CM4 - ADC1 DMA error\n");
		Error_Handler();
		}


	init_loadcells();

	/* Infinite loop */
	for (;;) {

		osDelay(SAMPLE_TIME);

		thrust[0] = hx711_weight(&LOAD_CELL1,1);
		thrust[1] = hx711_weight(&LOAD_CELL2,1);
		thrust[2] = hx711_weight(&LOAD_CELL3,1);
		thrust[3] = hx711_weight(&LOAD_CELL4,1);

		total_thrust = thrust[0]+thrust[1]+thrust[2]+thrust[3];

		compute_flow();

		pressure_LOX = (((ADC_ConvertedData[2]*(3.3/65535))/165.0) - 0.004)*PRESS_LOX_SENSITIVIY;

		pressure_KERO = (((ADC_ConvertedData[4]*(3.3/65535))/165.0) - 0.004)*PRESS_KERO_SENSITIVITY;

		pressure_CHAMBER = (ADC_ConvertedData[0]*(3.3/65535))/165.0;

		//LOX_FLOW = ADC_ConvertedData[5]*(3.3/65535);

		Ign_curr = ((ADC_ConvertedData[3]*(3.3/65535))/400)*21000;// (Vadc/Rsense)*Kis

		Valve1_Status = HAL_GPIO_ReadPin(GPIOE,LOW_SIDE_SW_2_Pin);
		Valve2_Status = HAL_GPIO_ReadPin(GPIOE,LOW_SIDE_SW_1_Pin);

		//printf("%6.1f %6.1f %6.1f %6.1f - %6.1f\n",thrust[0],thrust[1],thrust[2],thrust[3],total_thrust);
		//printf("%d\n",ADC_ConvertedData[0]);
		//printf("%.1f\n",Ign_curr);
		//printf(" Pressure KERO: %2.1f - Pressure LOX: %2.1f \n",pressure_KERO,pressure_LOX);
		//printf("%.1f\n",LOX_FLOW);
		//printf("%4.2f \n",frequency);


	}
  /* USER CODE END StartSens_Acq_Task */
}

/* USER CODE BEGIN Header_Start_CommandRX_Task */
/**
 * @brief Function implementing the CommandRX_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_CommandRX_Task */
void Start_CommandRX_Task(void const * argument)
{
  /* USER CODE BEGIN Start_CommandRX_Task */


	//Before entering the loop the ground station must be initialized and sent a udp initialization command
	/* Infinite loop */
	osDelay(5000);
	printf("\nTC Task Started\n");


	for (;;) {
		osDelay(100);

		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

		unpack_SPP_TC();


	}
  /* USER CODE END Start_CommandRX_Task */
}

/* USER CODE BEGIN Header_Start_TelemetryTX_Task */
/**
 * @brief Function implementing the TelemetryTX_Tas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_TelemetryTX_Task */
void Start_TelemetryTX_Task(void const * argument)
{
  /* USER CODE BEGIN Start_TelemetryTX_Task */
	/* Infinite loop */
	printf("\nTM Task Started\n"); //should start after DAQ task

	primaryHdr TM_hdr;
	space_packet TM_pkt;

	//define SPP Packet Header structure
	TM_hdr.APID = 5; //application id
	TM_hdr.pkt_ver = 0;
	TM_hdr.secHdrFlag = 0;
	TM_hdr.type = 0;
	TM_hdr.data_len = SPP_BYTE_DATA; //bytes in sec. header + data minus one (0 + 6) - 1
	TM_hdr.seqFlag = 3;

    //put the header into the packet
	TM_pkt.pHdr = TM_hdr;

	unsigned int ccsds_counter = 0;

	myfloat SPP_IGN_curr;

	myfloat SPP_Thrust1;
	myfloat SPP_Thrust2;
	myfloat SPP_Thrust3;
	myfloat SPP_Thrust4;

	myfloat SPP_ThrustTOT;

	myfloat SPP_MF1;
	myfloat SPP_MF2;

	myfloat SPP_PS_KERO;
	myfloat SPP_PS_LOX;
	myfloat SPP_PS_CHAMBER;


	for (;;) {

		SPP_IGN_curr.f = Ign_curr;

		SPP_Thrust1.f = thrust[0];
		SPP_Thrust2.f = thrust[1];
		SPP_Thrust3.f = thrust[2];
		SPP_Thrust4.f = thrust[3];

		SPP_ThrustTOT.f = total_thrust;

		SPP_MF1.f = vol_flow1;
		SPP_MF2.f = vol_flow2;

		SPP_PS_KERO.f = pressure_KERO;
		SPP_PS_LOX.f = pressure_LOX;
		SPP_PS_CHAMBER.f = pressure_CHAMBER;

		TM_pkt.data[0] = (SPP_IGN_curr.raw.sign << 7)| (SPP_IGN_curr.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[1] = (SPP_IGN_curr.raw.exponent << 7)	| (SPP_IGN_curr.raw.mantissa >> 16);
		TM_pkt.data[2] = (SPP_IGN_curr.raw.mantissa >> 8);
		TM_pkt.data[3] = (SPP_IGN_curr.raw.mantissa);

		TM_pkt.data[4] = (SPP_Thrust1.raw.sign << 7)| (SPP_Thrust1.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[5] = (SPP_Thrust1.raw.exponent << 7)	| (SPP_Thrust1.raw.mantissa >> 16);
		TM_pkt.data[6] = (SPP_Thrust1.raw.mantissa >> 8);
		TM_pkt.data[7] = (SPP_Thrust1.raw.mantissa);

		TM_pkt.data[8] = (SPP_Thrust2.raw.sign << 7)| (SPP_Thrust2.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[9] = (SPP_Thrust2.raw.exponent << 7)	| (SPP_Thrust2.raw.mantissa >> 16);
		TM_pkt.data[10] = (SPP_Thrust2.raw.mantissa >> 8);
		TM_pkt.data[11] = (SPP_Thrust2.raw.mantissa);

		TM_pkt.data[12] = (SPP_Thrust3.raw.sign << 7)| (SPP_Thrust3.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[13] = (SPP_Thrust3.raw.exponent << 7)	| (SPP_Thrust3.raw.mantissa >> 16);
		TM_pkt.data[14] = (SPP_Thrust3.raw.mantissa >> 8);
		TM_pkt.data[15] = (SPP_Thrust3.raw.mantissa);

		TM_pkt.data[16] = (SPP_Thrust4.raw.sign << 7)| (SPP_Thrust4.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[17] = (SPP_Thrust4.raw.exponent << 7)	| (SPP_Thrust4.raw.mantissa >> 16);
		TM_pkt.data[18] = (SPP_Thrust4.raw.mantissa >> 8);
		TM_pkt.data[19] = (SPP_Thrust4.raw.mantissa);

		TM_pkt.data[20] = (SPP_MF1.raw.sign << 7)| (SPP_MF1.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[21] = (SPP_MF1.raw.exponent << 7)	| (SPP_MF1.raw.mantissa >> 16);
		TM_pkt.data[22] = (SPP_MF1.raw.mantissa >> 8);
		TM_pkt.data[23] = (SPP_MF1.raw.mantissa);

		TM_pkt.data[24] = (SPP_MF2.raw.sign << 7)| (SPP_MF2.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[25] = (SPP_MF2.raw.exponent << 7)	| (SPP_MF2.raw.mantissa >> 16);
		TM_pkt.data[26] = (SPP_MF2.raw.mantissa >> 8);
		TM_pkt.data[27] = (SPP_MF2.raw.mantissa);


		TM_pkt.data[28] = (SPP_PS_KERO.raw.sign << 7)| (SPP_PS_KERO.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[29] = (SPP_PS_KERO.raw.exponent << 7)	| (SPP_PS_KERO.raw.mantissa >> 16);
		TM_pkt.data[30] = (SPP_PS_KERO.raw.mantissa >> 8);
		TM_pkt.data[31] = (SPP_PS_KERO.raw.mantissa);

		TM_pkt.data[32] = (SPP_PS_LOX.raw.sign << 7)| (SPP_PS_LOX.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[33] = (SPP_PS_LOX.raw.exponent << 7)	| (SPP_PS_LOX.raw.mantissa >> 16);
		TM_pkt.data[34] = (SPP_PS_LOX.raw.mantissa >> 8);
		TM_pkt.data[35] = (SPP_PS_LOX.raw.mantissa);

		TM_pkt.data[36] = (SPP_PS_CHAMBER.raw.sign << 7)| (SPP_PS_CHAMBER.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[37] = (SPP_PS_CHAMBER.raw.exponent << 7)	| (SPP_PS_CHAMBER.raw.mantissa >> 16);
		TM_pkt.data[38] = (SPP_PS_CHAMBER.raw.mantissa >> 8);
		TM_pkt.data[39] = (SPP_PS_CHAMBER.raw.mantissa);

		TM_pkt.data[40] = Valve1_Status >> 24; //& (vartest.raw.exponent >> 1);
		TM_pkt.data[41] = Valve1_Status >> 16;
		TM_pkt.data[42] = Valve1_Status >> 8;
		TM_pkt.data[43] = Valve1_Status;

		TM_pkt.data[44] = Valve2_Status >> 24; //& (vartest.raw.exponent >> 1);
		TM_pkt.data[45] = Valve2_Status >> 16;
		TM_pkt.data[46] = Valve2_Status >> 8;
		TM_pkt.data[47] = Valve2_Status;

		TM_pkt.data[48] = (SPP_ThrustTOT.raw.sign << 7)| (SPP_ThrustTOT.raw.exponent >> 1); //& (vartest.raw.exponent >> 1);
		TM_pkt.data[49] = (SPP_ThrustTOT.raw.exponent << 7)	| (SPP_ThrustTOT.raw.mantissa >> 16);
		TM_pkt.data[50] = (SPP_ThrustTOT.raw.mantissa >> 8);
		TM_pkt.data[51] = (SPP_ThrustTOT.raw.mantissa);


		TM_pkt.pHdr.seqCount = ccsds_counter;

		pack_SPP_TM(TM_pkt);

		udpClient_send_spp();

		ccsds_counter++;

		osDelay(SAMPLE_TIME); // 1 kHz UDP Telemetry


	}
  /* USER CODE END Start_TelemetryTX_Task */
}

/* USER CODE BEGIN Header_Start_ActuatorsControl_Task */
/**
 * @brief Function implementing the ActuatorsContro thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_ActuatorsControl_Task */
void Start_ActuatorsControl_Task(void const * argument)
{
  /* USER CODE BEGIN Start_ActuatorsControl_Task */

	osDelay(1000); //start after TC -> DAQ -> TM -> ACT

	//printf("\nActuator Control Task Started\n");

	for (;;) {

		osDelay(100);

	}
  /* USER CODE END Start_ActuatorsControl_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printf("Error handler called\n");
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
