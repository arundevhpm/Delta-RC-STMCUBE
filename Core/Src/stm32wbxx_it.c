/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wbxx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32wbxx_it.h"
#include "Periodic.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 // Addresses for fault status registers
#define CFSR   (*((volatile uint32_t*)0xE000ED28)) // Configurable Fault Status Register
#define HFSR   (*((volatile uint32_t*)0xE000ED2C)) // Hard Fault Status Register
#define DFSR   (*((volatile uint32_t*)0xE000ED30)) // Debug Fault Status Register
#define MMAR   (*((volatile uint32_t*)0xE000ED34)) // Memory Management Fault Address Register
#define BFAR   (*((volatile uint32_t*)0xE000ED38))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern IPCC_HandleTypeDef hipcc;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim17;
extern ADC_HandleTypeDef hadc1;
extern PCD_HandleTypeDef hpcd_USB_FS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
// HardFault handler C function that extracts stack frame values
void HardFault_Handler_C(uint32_t *stack) {
    uint32_t stacked_r0 = stack[0];
    uint32_t stacked_r1 = stack[1];
    uint32_t stacked_r2 = stack[2];
    uint32_t stacked_r3 = stack[3];
    uint32_t stacked_r12 = stack[4];
    uint32_t stacked_lr = stack[5];
    uint32_t stacked_pc = stack[6];
    uint32_t stacked_psr = stack[7];

    printf("Stacked registers at HardFault:\n");
    printf("R0  = 0x%08lx\n", stacked_r0);
    printf("R1  = 0x%08lx\n", stacked_r1);
    printf("R2  = 0x%08lx\n", stacked_r2);
    printf("R3  = 0x%08lx\n", stacked_r3);
    printf("R12 = 0x%08lx\n", stacked_r12);
    printf("LR  = 0x%08lx\n", stacked_lr);   // Link Register
    printf("PC  = 0x%08lx\n", stacked_pc);   // Program Counter
    printf("PSR = 0x%08lx\n", stacked_psr);  // Program Status Register

    while (1); // Infinite loop to halt the system for debugging
}
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
    // Print fault status register values for debugging
    printf("HardFault detected!\n");
    printf("CFSR: 0x%08lx\n", CFSR);
    printf("HFSR: 0x%08lx\n", HFSR);
    printf("DFSR: 0x%08lx\n", DFSR);
    printf("MMAR: 0x%08lx\n", MMAR);
    printf("BFAR: 0x%08lx\n", BFAR);

    // Capture stack frame information
    __asm volatile (
        "TST lr, #4 \n"                      // Test EXC_RETURN to determine stack pointer
        "ITE EQ \n"                          // If-Then-Else block
        "MRSEQ r0, msp \n"                   // Load Main Stack Pointer if in Handler mode
        "MRSNE r0, psp \n"                   // Load Process Stack Pointer if in Thread mode
        "B HardFault_Handler_C \n"           // Branch to C handler
    );
  /* USER CODE END HardFault_IRQn 0 */
//  while (1)
//  {
//    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
//    /* USER CODE END W1_HardFault_IRQn 0 */
//  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32WBxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wbxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC wake-up interrupt through EXTI line 19.
  */
uint8_t LedStatus=0;
void RTC_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_WKUP_IRQn 0 */

  /* USER CODE END RTC_WKUP_IRQn 0 */
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_WKUP_IRQn 1 */
//  if(LedStatus==0)
//  {
//	  LedStatus=1;
//	  PWM_Stop( LED_Green);
//  }
//  else
//  {
//	  LedStatus=0;
//	  PWM_Start( LED_Green);
//  }
  /* USER CODE END RTC_WKUP_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_lpuart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 global interrupt.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */

  /* USER CODE END LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */

  /* USER CODE END LPUART1_IRQn 1 */
}

/**
  * @brief This function handles IPCC RX occupied interrupt.
  */
void IPCC_C1_RX_IRQHandler(void)
{
  /* USER CODE BEGIN IPCC_C1_RX_IRQn 0 */

  /* USER CODE END IPCC_C1_RX_IRQn 0 */
  HAL_IPCC_RX_IRQHandler(&hipcc);
  /* USER CODE BEGIN IPCC_C1_RX_IRQn 1 */

  /* USER CODE END IPCC_C1_RX_IRQn 1 */
}

/**
  * @brief This function handles IPCC TX free interrupt.
  */
void IPCC_C1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN IPCC_C1_TX_IRQn 0 */

  /* USER CODE END IPCC_C1_TX_IRQn 0 */
  HAL_IPCC_TX_IRQHandler(&hipcc);
  /* USER CODE BEGIN IPCC_C1_TX_IRQn 1 */

  /* USER CODE END IPCC_C1_TX_IRQn 1 */
}

/**
  * @brief This function handles HSEM global interrupt.
  */
void HSEM_IRQHandler(void)
{
  /* USER CODE BEGIN HSEM_IRQn 0 */

  /* USER CODE END HSEM_IRQn 0 */
  HAL_HSEM_IRQHandler();
  /* USER CODE BEGIN HSEM_IRQn 1 */

  /* USER CODE END HSEM_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel4 global interrupt.
  */
void DMA2_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel4_IRQn 0 */

  /* USER CODE END DMA2_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Channel4_IRQn 1 */

  /* USER CODE END DMA2_Channel4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
 * @brief  This function handles External line
 *         interrupt request.
 * @param  None
 * @retval None
 */
//void PUSH_BUTTON_SW1_EXTI_IRQHandler(void)
//{
// // HAL_GPIO_EXTI_IRQHandler(BUTTON_SW1_PIN);
//}

/**
 * @brief  This function handles External line
 *         interrupt request.
 * @param  None
 * @retval None
 */
//void PUSH_BUTTON_SW2_EXTI_IRQHandler(void)
//{
// // HAL_GPIO_EXTI_IRQHandler(BUTTON_SW2_PIN);
//}

/**
 * @brief  This function handles External line
 *         interrupt request.
 * @param  None
 * @retval None
 */
//void PUSH_BUTTON_SW3_EXTI_IRQHandler(void)
//{
// // HAL_GPIO_EXTI_IRQHandler(BUTTON_SW3_PIN);
//}
void EXTI3_IRQHandler(void)
{
	  HAL_GPIO_EXTI_IRQHandler(STM_BOOT_PIN);
}
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KB_IN1_ST_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KB_IN2_ST_Pin);
  HAL_GPIO_EXTI_IRQHandler(KB_IN3_ST_Pin);
  HAL_GPIO_EXTI_IRQHandler(NFC_GPO_RES_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}
/**
  * @brief This function handles USB low priority interrupt, USB wake-up interrupt through EXTI line 28.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/* USER CODE END 1 */
