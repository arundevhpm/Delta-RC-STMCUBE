/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
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
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"
#include "tx_api.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32wbxx_nucleo.h"
/* USER CODE END Includes */

/* Large memory allocations to give margin to add extra code */
#define DEMO_STACK_SIZE_LARGE      (2048)
#define DEMO_STACK_SIZE_REDUCED    (2048)
#define DEMO_STACK_SIZE_REDUCED1    (1024)
#define DEMO_BYTE_POOL_SIZE        (24576+4096)//(9120 *2)
#define DEMO_BLOCK_POOL_SIZE       (100)
#define IDLE_THREAD_STACK	(512)
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
void   MX_LPUART1_UART_Init(void);
void   MX_USART1_UART_Init(void);
void MX_RTC_Init(void);


/* USER CODE BEGIN Private defines */
#define ALM_Blue_Pin GPIO_PIN_2
#define ALM_Blue_GPIO_Port GPIOA
#define ALM_Green_Pin GPIO_PIN_1
#define ALM_Green_GPIO_Port GPIOA
#define ALM_Red_Pin GPIO_PIN_0
#define ALM_Red_GPIO_Port GPIOA
#define KB_OUT4_ST_Pin GPIO_PIN_3
#define KB_OUT4_ST_GPIO_Port GPIOC
#define KB_OUT3_ST_Pin GPIO_PIN_2
#define KB_OUT3_ST_GPIO_Port GPIOC
#define KB_OUT2_ST_Pin GPIO_PIN_1
#define KB_OUT2_ST_GPIO_Port GPIOC
#define KB_OUT1_ST_Pin GPIO_PIN_0
#define KB_OUT1_ST_GPIO_Port GPIOC
#define STM_BOOT_PIN GPIO_PIN_3
#define STM_BOOT_GPIO_Port GPIOH
//#define NFC_GPO_RES_Pin GPIO_PIN_9
//#define NFC_GPO_RES_GPIO_Port GPIOB
//#define NFC_GPO_RES_EXTI_IRQn EXTI9_5_IRQn
//#define ST_I2C_SCL_Pin GPIO_PIN_8
//#define ST_I2C_SCL_GPIO_Port GPIOB
//#define ST_I2C_SDA_Pin GPIO_PIN_7
//#define ST_I2C_SDA_GPIO_Port GPIOB
#define STM_JTAG_NJRST_Pin GPIO_PIN_4
#define STM_JTAG_NJRST_GPIO_Port GPIOB
#define STM_JTAG_TDO_Pin GPIO_PIN_3
#define STM_JTAG_TDO_GPIO_Port GPIOB
#define ST_STATUS_POWER_Pin GPIO_PIN_12
#define ST_STATUS_POWER_GPIO_Port GPIOC
#define STM_JTAG_TMS_SWDIO_Pin GPIO_PIN_13
#define STM_JTAG_TMS_SWDIO_GPIO_Port GPIOA
#define STM_JTAG_TMS_SWCLK_Pin GPIO_PIN_14
#define STM_JTAG_TMS_SWCLK_GPIO_Port GPIOA
#define STM_JTAG_TDI_Pin GPIO_PIN_15
#define STM_JTAG_TDI_GPIO_Port GPIOA
#define ST_USB_DP_Pin GPIO_PIN_12
#define ST_USB_DP_GPIO_Port GPIOA
#define ST_USB_DM_Pin GPIO_PIN_11
#define ST_USB_DM_GPIO_Port GPIOA
#define KB_IN3_ST_Pin GPIO_PIN_6
#define KB_IN3_ST_GPIO_Port GPIOC
#define KB_IN3_ST_EXTI_IRQn EXTI9_5_IRQn
#define ADC_VREF_ENABLE_Pin GPIO_PIN_14
#define ADC_VREF_ENABLE_GPIO_Port GPIOB
#define ADC_GND_RD_Pin GPIO_PIN_15
#define ADC_GND_RD_GPIO_Port GPIOB
#define KB_IN2_ST_Pin GPIO_PIN_5
#define KB_IN2_ST_GPIO_Port GPIOC
#define KB_IN2_ST_EXTI_IRQn EXTI9_5_IRQn
#define KB_IN1_ST_Pin GPIO_PIN_4
#define KB_IN1_ST_GPIO_Port GPIOC
#define KB_IN1_ST_EXTI_IRQn EXTI4_IRQn
#define STM_PWM_RET_Pin GPIO_PIN_9
#define STM_PWM_RET_GPIO_Port GPIOA
#define V_BATT_CELL1_ST_Pin GPIO_PIN_7
#define V_BATT_CELL1_ST_GPIO_Port GPIOA
#define STM_ADC_VBATT_Pin GPIO_PIN_6
#define STM_ADC_VBATT_GPIO_Port GPIOA
#define STM_PWM_BEEP_Pin GPIO_PIN_3
#define STM_PWM_BEEP_GPIO_Port GPIOA

#define Remote
#ifdef Remote
#define ST_I2C_SCL_Pin 				GPIO_PIN_8
#define ST_I2C_SCL_GPIO_Port 		GPIOB
#define ST_I2C_SDA_Pin 				GPIO_PIN_7//GPIO_PIN_9
#define ST_I2C_SDA_GPIO_Port 		GPIOB
#define NFC_GPO_RES_Pin 			GPIO_PIN_9 //GPIO_PIN_9
#define NFC_GPO_RES_GPIO_Port 		GPIOB	//GPIOB
#define NFC_GPO_RES_EXTI_IRQn 		EXTI9_5_IRQn
#else
#define ST_I2C_SCL_Pin 				GPIO_PIN_8
#define ST_I2C_SCL_GPIO_Port 		GPIOB
#define ST_I2C_SDA_Pin 				GPIO_PIN_9
#define ST_I2C_SDA_GPIO_Port 		GPIOB
#define NFC_GPO_RES_Pin 			GPIO_PIN_6
#define NFC_GPO_RES_GPIO_Port 		GPIOA
#define NFC_GPO_RES_EXTI_IRQn 		EXTI9_5_IRQn
#endif

/********************************************************************************

     In this example TIM2 input clock (TIM2CLK)  is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
      TIM2CLK = PCLK1
      PCLK1 = HCLK
      => TIM2CLK = HCLK = SystemCoreClock
      SystemCoreClock is set to 32 MHz
    To get TIM2 counter clock equal to 1000000 Hz, the Prescaler is computed as following:
    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    Prescaler = (SystemCoreClock /1000000) - 1
//--------------------------------------
		Update rate = TIM2 counter clock / (Period + 1) = 100 Hz,
		So the TIM2 generates an interrupt each 10 ms

		Update rate = TIM2 counter clock / (Period + 1) = 50 Hz,
		So the TIM2 generates an interrupt each 5 ms

		Update rate = TIM2 counter clock / (Period + 1) = 2 KHz,PWM is generated with 2KHz Freq
		So the TIM2 generates an interrupt each 500 ms

		Update rate = TIM2 counter clock / (Period + 1) = 4 KHz,PWM is generated with 2KHz Freq
		So the TIM2 generates an interrupt each 250 ms
*/
#define PRESCALER_VALUE (uint32_t)(((SystemCoreClock) / (1000000)) - 1)

#define PERIOD_VALUE (500 - 1)
#define PULSE_VALUE_BUZZ (uint32_t)((250-1) / 2)  //for 4KHz
#define PULSE_VALUE1 (uint32_t)(PERIOD_VALUE / 2)          /* Capture Compare 1 Value  */
#define PULSE_VALUE2 (uint32_t)(PERIOD_VALUE * 37.5 / 100) // Capture Compare 2 Value
#define PULSE_VALUE3 (uint32_t)(PERIOD_VALUE / 4)          // Capture Compare 3 Value
#define PULSE_VALUE4 (uint32_t)(PERIOD_VALUE * 12.5 / 100) // Capture Compare 4 Value


extern uint8_t PowerSource;

enum PowerSrc
{
	BATTERY_POWER =0,
	USB_POWER,
};
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
