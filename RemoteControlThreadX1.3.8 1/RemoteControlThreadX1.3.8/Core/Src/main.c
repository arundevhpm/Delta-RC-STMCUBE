/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @brief   BLE application with BLE core
 *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                    ##### IMPORTANT NOTE #####
  ==============================================================================

  This application requests having the stm32wb5x_BLE_Stack_fw.bin binary
  flashed on the Wireless Coprocessor.
  If it is not the case, you need to use STM32CubeProgrammer to load the appropriate
  binary.

  All available binaries are located under following directory:
  /Projects/STM32_Copro_Wireless_Binaries

  Refer to UM2237 to learn how to use/install STM32CubeProgrammer.
  Refer to /Projects/STM32_Copro_Wireless_Binaries/ReleaseNote.html for the
  detailed procedure to change the Wireless Coprocessor binary.

  @endverbatim
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "tim.h"
#include "adc.h"
#include "Periodic.h"
#include "dbg_trace.h"
#include "i2c.h"
#include "tagtype5_wrapper.h"
#include "nfc07a1_nfctag.h"
#include "iwdg.h"

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
IPCC_HandleTypeDef hipcc;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_lpuart1_tx;
DMA_HandleTypeDef hdma_usart1_tx;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef hadc1;
extern uint8_t BattDataAvailablefg;
extern uint32_t BattData;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
//static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RF_Init(void);
void MX_RTC_Init(void);
static void MX_IPCC_Init(void);
static void MX_RNG_Init(void);
void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
extern IWDG_HandleTypeDef hiwdg;
static void Init_NFC( void )
{
  PWM_Stop(LED_Blue);
  NFC07A1_NFCTAG_Init(0);
  NFC07A1_NFCTAG_SetRFDisable_Dyn(0);

//  BSP_GPO_Init();
  NFC07A1_GPO_Init();
  NFC07A1_NFCTAG_ConfigIT(0,0x82);

  /* Write CC file content */
  CCFileStruct.MagicNumber  = NFCT5_MAGICNUMBER_E1_CCFILE;
  CCFileStruct.Version      = NFCT5_VERSION_V1_0;
  CCFileStruct.MemorySize   = 0x40; // 4kbits
  CCFileStruct.TT5Tag       =  1;

   NfcTag_SelectProtocol(NFCTAG_TYPE5);
   NfcType5_TT5Init();
   NDEF_ClearNDEF();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint8_t PowerSource;
HAL_StatusTypeDef modifyIWDG()
{
    FLASH_OBProgramInitTypeDef OB;
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBGetConfig(&OB);

    // check if
    if ((OB.UserConfig & FLASH_OPTR_IWDG_STOP) || // IWDG_STOP is set      or
        (OB.UserConfig & FLASH_OPTR_IWDG_STDBY))  // IWDG_STDBY is set
    {
        OB.OptionType = OPTIONBYTE_USER;
        OB.UserType = OB_USER_IWDG_STOP | OB_USER_IWDG_STDBY;
        OB.UserConfig = OB_IWDG_STOP_FREEZE | OB_IWDG_STDBY_FREEZE;

        if (HAL_FLASHEx_OBProgram(&OB) != HAL_OK)
        {
            HAL_FLASH_OB_Lock();
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }

        HAL_FLASH_OB_Launch();

        /* We should not make it past the Launch, so lock
         * flash memory and return an error from function
         */
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
    return HAL_OK;
}
int main(void)
{
  /* USER CODE BEGIN 1 */
	GPIO_PinState PinState;
	  ADC_ChannelConfTypeDef sConfig = {0};
	  HAL_StatusTypeDef halResult;
	  int i;
	  int batteryconnectedcount=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  modifyIWDG();
  MX_IWDG_Init();
  PinState = HAL_GPIO_ReadPin(ST_STATUS_POWER_GPIO_Port,ST_STATUS_POWER_Pin);
  if(PinState ==GPIO_PIN_SET)
  {
	  PowerSource = USB_POWER;

  }
  else
  {
	  PowerSource = BATTERY_POWER;

	  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
	  MX_APPE_Config();
	  /* IPCC initialisation */
	  MX_IPCC_Init();
	  MX_RF_Init();
	  MX_RNG_Init();
	  Init_NFC();
	  /* Init code for STM32_WPAN */
	  /* Containing all application initialization could be run before kernel launching */
	  MX_APPE_Init();
  }

	  HW_TS_Init(hw_ts_InitMode_Full, &hrtc); /**< Initialize the TimerServer */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
#if(CFG_DEBUG_TRACE != 0)
  DbgTraceInit();
#endif
  //-------------------------------USB and Battery Poka Yoke------------------------------------------
  HAL_GPIO_WritePin(ADC_VREF_ENABLE_GPIO_Port,ADC_VREF_ENABLE_Pin,GPIO_PIN_SET);
  float Vref = 2500.00;
  uint32_t ADCResolution = 4095; // 12 bit
  float ActualBatteryVtg = 0;
  float stepSize = Vref / ADCResolution;
  PWM_Start(LED_Green);
  HAL_Delay(1000);
  PWM_Stop(LED_Green);
  HAL_IWDG_Refresh(&hiwdg);
	if( PowerSource == USB_POWER)
	{
		//  sConfig.Channel = ADC_CHANNEL_11;//Double battery
		sConfig.Channel = ADC_CHANNEL_12;//Single battery
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		Error_Handler();
		}
		for(i=0;i<5;i++)
		{
			halResult = HAL_ADC_Start_IT(&hadc1);
			if (halResult != HAL_OK)
			{
			  Error_Handler();
			}
			while (BattDataAvailablefg==0);
			BattDataAvailablefg=0;
			BattData = HAL_ADC_GetValue(&hadc1);
			halResult = HAL_ADC_Stop_IT(&hadc1);
			// ADCVrefPin.set(hal::IHalGpioOut::PinState::Reset);
			ActualBatteryVtg = (float)BattData * stepSize;
			APP_DBG_MSG("  Actual Battery Voltage : %f\r\n", ActualBatteryVtg);

			if (ActualBatteryVtg > 1100)
			{
			   batteryconnectedcount++;
			}
			HAL_Delay(100);
		}
		if(batteryconnectedcount > 3)
		{
			PWM_Start(LED_Red);

			while(1){
				  HAL_IWDG_Refresh(&hiwdg);
				  HAL_Delay(1000);
			};
		}
		batteryconnectedcount=0;
		sConfig.Channel = ADC_CHANNEL_11;//Double battery
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		for(i=0;i<5;i++)
		{
			halResult = HAL_ADC_Start_IT(&hadc1);
			if (halResult != HAL_OK)
			{
			  Error_Handler();
			}
			while (BattDataAvailablefg==0);
			BattDataAvailablefg=0;
			BattData = HAL_ADC_GetValue(&hadc1);
			halResult = HAL_ADC_Stop_IT(&hadc1);
			// ADCVrefPin.set(hal::IHalGpioOut::PinState::Reset);
			ActualBatteryVtg = (float)BattData * stepSize * 2;
			APP_DBG_MSG("  Actual Battery Voltage : %f\r\n", ActualBatteryVtg);

			if (ActualBatteryVtg > 500)
			{
			   batteryconnectedcount++;
			}
			HAL_Delay(100);
		}

		if(batteryconnectedcount > 3)
		{
			PWM_Start(LED_Red);
			while(1){
				  HAL_IWDG_Refresh(&hiwdg);
				  HAL_Delay(1000);
			};
		}
	}
    InitRows(GPIO_PIN_RESET);
//    PWM_Start(Buzzer);
//    HAL_Delay(500);
//    PWM_Stop(Buzzer);
//    LowPower_Enter();
//
//	LowPower_Exit();
//    PWR_EnterStopMode();
//    PWR_ExitStopMode();
//    PWM_Start(Buzzer);
//    HAL_Delay(500);
//    PWM_Stop(Buzzer);
//    while(1)
//    {
//
//    }


  /* Launching ThreadX kernel */
  MX_ThreadX_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
    /* USER CODE END WHILE */
    /* MX_APPE_Process(); */
    /* tx_application_define is the entry point for threadx OS */
    /* while loop could be totally removed */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
//                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
//  RCC_OscInitStruct.PLL.PLLN = 12;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV24;
//  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
//  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI1 | RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV24;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
      Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
 void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  /* 15 priority is reserved for Cortex-M4 Pend SV Interrupt  handler (used for ThreadX Idle loop) */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  /* 15 priority is reserved for Cortex-M4 Pend SV Interrupt  handler (used for ThreadX Idle loop) */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
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
	while(1)
	{
		  HAL_IWDG_Refresh(&hiwdg);
		  HAL_Delay(1000);

	}
  /* User can add his own implementation to report the HAL error return state */

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
