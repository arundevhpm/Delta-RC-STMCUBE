/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#define ST25_DISCOVERY_I2C1_SPEED   (1000000)
#define ST25_DISCOVERY_I2Cx_TIMEOUT (100)
#define ST25DV_I2C_WRITE_TIMEOUT    (320)
static void BSP_I2C1_MspInit( void );
static void BSP_I2C1_MspDeInit( void );
static uint8_t BSP_I2C1_IsNacked( void );
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */


	  /* USER CODE END I2C1_Init 1 */
	  hi2c1.Instance = I2C1;
	  hi2c1.Init.Timing = 0x00303D5B;
	  hi2c1.Init.OwnAddress1 = 0;
	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.OwnAddress2 = 0;
	  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure Analogue filter
	  */
	  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure Digital filter
	  */
	  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN I2C1_Init 2 */


	  /* USER CODE END I2C1_Init 2 */


}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    /* Reset I2Cx */
    __HAL_RCC_I2C1_FORCE_RESET( );
    __HAL_RCC_I2C1_RELEASE_RESET( );

    /* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = ST_I2C_SCL_Pin|ST_I2C_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(ST_I2C_SCL_GPIO_Port, ST_I2C_SCL_Pin);

    HAL_GPIO_DeInit(ST_I2C_SDA_GPIO_Port, ST_I2C_SDA_Pin);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
int32_t BSP_I2C1_Init( void )
{
  HAL_StatusTypeDef ret_val = HAL_OK;

  if( HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET )
  {
    /* I2C2 peripheral configuration */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00000E14;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    /* Init the I2C */
    BSP_I2C1_MspInit( );
    ret_val = HAL_I2C_Init( &hi2c1 );

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      // to do: call proper function
      //Error_Handler();
      while(1);
    }
    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
      // to do: call proper function
      //Error_Handler();
      while(1);
    }
    /** I2C Enable Fast Mode Plus
    */
    HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);

  }

  return (int32_t)ret_val;
}

int32_t BSP_I2C1_DeInit( void )
{
  /* DeInit the I2C */
  BSP_I2C1_MspDeInit( );
  HAL_I2C_DeInit( &hi2c1 );

  /* ReInit the I2C */
  // BSP_I2C1_Init( );

  return 0;
}


/**
  * @brief  Write data in a register of the device through the bus
  * @param  pData : pointer to the data to write
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to write
  * @param  Size : Size in bytes of the value to be written
  * @retval HAL status
  */

int32_t BSP_I2C1_MemWrite( const uint16_t DevAddr, const uint16_t TarAddr, const uint8_t * const pData,
                                              const uint16_t Size )
{
  int32_t     pollstatus;
  uint8_t     *pbuffer = (uint8_t *)pData;
  int32_t     ret;
  uint32_t    tickstart;

  if ((pbuffer == NULL) && (Size == 0U)) {
    ret = (int32_t)HAL_I2C_Master_Transmit(&hi2c1, (uint8_t)DevAddr, pbuffer, Size, ST25_DISCOVERY_I2Cx_TIMEOUT);
  }
  else {
    ret = (int32_t)HAL_I2C_Mem_Write( &hi2c1, DevAddr, TarAddr, I2C_MEMADD_SIZE_16BIT, pbuffer, Size,
                            ST25_DISCOVERY_I2Cx_TIMEOUT );
  }
  if( ret == 0 )
  {
	/* Poll until EEPROM is available */
	tickstart = HAL_GetTick( );
	/* Wait until ST25DV is ready or timeout occurs */
	do
	{
	  pollstatus = BSP_I2C1_IsDeviceReady( DevAddr, 1 );
	} while( ( (HAL_GetTick() - tickstart) < ST25DV_I2C_WRITE_TIMEOUT) && (pollstatus != 0) );

	if( pollstatus != 0 )
	{
	  ret = 4;
	}
  }
  else
  {
	/* Check if Write was NACK */
	if( BSP_I2C1_IsNacked() )
	{
	  ret = -102;
	}
  }
  return ret;
}

/**
  * @brief  Read the value of a register of the device through the bus.
  * @param  pData : pointer to store read data
  * @param  DevAddr : Target device address
  * @param  TarAddr : I2C data memory address to read
  * @param  Size : Size in bytes of the value to be read
  * @retval HAL status.
  */
int32_t BSP_I2C1_MemRead( const uint16_t DevAddr, const uint16_t TarAddr, uint8_t * const pData,
                                             const uint16_t Size )
{
  uint8_t *pbuffer = (uint8_t *)pData;
  int32_t ret;

  /* I2C Timeout: (transfer size in bytes) * (bits per bytes) * (extra delay) / (I2C speed) */
  uint32_t timeout = (Size * 8 * 1000 * 2) / ST25_DISCOVERY_I2C1_SPEED;
  if( timeout < ST25_DISCOVERY_I2Cx_TIMEOUT )
  {
    timeout = ST25_DISCOVERY_I2Cx_TIMEOUT;
  }

  if ((pbuffer == NULL) && (Size == 0U)) {
    ret = (int32_t)HAL_I2C_Master_Receive(&hi2c1, DevAddr, pbuffer, Size, timeout);
  }
  else {
    ret = (int32_t)HAL_I2C_Mem_Read( &hi2c1, DevAddr, TarAddr, I2C_MEMADD_SIZE_16BIT, pbuffer, Size, timeout );
  }

  return ret;
}

/**
  * @brief  Read the value of a register of the device through the bus.
  * @param  pData : pointer to store read data
  * @param  DevAddr : the device address on bus
  * @param  Size : Size in bytes of the value to be read
  * @retval HAL status
  */
HAL_StatusTypeDef BSP_I2C1_Read( const uint8_t DevAddr, uint8_t * const pData, const uint16_t Size )
{
  uint8_t *pbuffer = (uint8_t *)pData;
  HAL_StatusTypeDef ret;

  ret = HAL_I2C_Master_Receive( &hi2c1, DevAddr, pbuffer, Size, ST25_DISCOVERY_I2Cx_TIMEOUT );

  return ret;
}

/**
* @brief  Checks if NACK was received from I2C Slave
* @param  None
* @retval 0 ACK, 1 NACK
*/
static uint8_t BSP_I2C1_IsNacked( void )
{
  if( hi2c1.ErrorCode == HAL_I2C_ERROR_AF )
  {
    return 1;
  }
  return 0;
}

/**
* @brief  Checks if target device is ready for communication
* @param  DevAddr : Target device address
* @param  Trials : Number of trials
* @retval HAL status
*/
int32_t BSP_I2C1_IsDeviceReady( const uint16_t DevAddr, const uint32_t Trials )
{
  return (int32_t)HAL_I2C_IsDeviceReady( &hi2c1, DevAddr, Trials, ST25_DISCOVERY_I2Cx_TIMEOUT );
}

/**
  * @brief  I2C MSP Initialization
  *         This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  *           - NVIC configuration for DMA interrupt request enable
  * @param  None
  * @return None
  */
static void BSP_I2C1_MspInit( void )
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable I2Cx clock */
  __HAL_RCC_I2C1_CLK_ENABLE( );
  /* USER CODE BEGIN I2C1_MspInit 0 */
    RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /* Reset I2Cx */
    __HAL_RCC_I2C1_FORCE_RESET( );
    __HAL_RCC_I2C1_RELEASE_RESET( );

    /* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE( );

    GPIO_InitStruct.Pin = ST_I2C_SCL_Pin|ST_I2C_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(ST_I2C_SCL_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
}

/**
  * @brief  I2C MSP DeInitialization
  * @param  None
  * @return None
  */
static void BSP_I2C1_MspDeInit( void )
{

  HAL_GPIO_DeInit( ST_I2C_SCL_GPIO_Port, ST_I2C_SCL_Pin );

  HAL_GPIO_DeInit( ST_I2C_SDA_GPIO_Port, ST_I2C_SDA_Pin );

    /* Disable I2Cx clock */
  __HAL_RCC_I2C1_CLK_DISABLE( );
}


/* USER CODE END 1 */
