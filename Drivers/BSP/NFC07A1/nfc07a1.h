/**
  ******************************************************************************
  * @file    nfc07a1.h
  * @author  MMY Application Team
  * @brief   This file contains definitions for the x_nucleo_nfc07a1.c
  *          board specific functions.
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT 2021 STMicroelectronics, all rights reserved
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef NFC07A1_H
#define NFC07A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "nfc07a1_conf.h"

#include "st25dvxxkc.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup X_NUCLEO_NFC07A1
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup X_NUCLEO_NFC07A1_Exported_Types
  * @{
  */
/**
 * @brief  NFC07A1 Led enumerator definition
 */
typedef enum 
{
  GREEN_LED = 0,
  BLUE_LED,
  YELLOW_LED
}NFC07A1_Led_E;

/**
 * @brief  NFC07A1 Ack Nack enumerator definition
 */
typedef enum 
{
  I2CANSW_ACK = 0,
  I2CANSW_NACK
}NFC07A1_I2CANSW_E;

/**
 * @brief  NFC07A1 Led structure definition
 */
typedef struct
{
  uint16_t          NFC07A1_LED_PIN;
  GPIO_TypeDef *    NFC07A1_LED_PIN_PORT;
}NFC07A1_Led_TypeDef;

/**
  * @}
  */
/* Exported constants --------------------------------------------------------*/
#define NFC07A1_INIT_CLK_GREEN_LED()        NFC07A1_INIT_CLK_LED1_RFD( );
#define NFC07A1_INIT_CLK_BLUE_LED()         NFC07A1_INIT_CLK_LED2_RFD( );
#define NFC07A1_INIT_CLK_YELLOW_LED()       NFC07A1_INIT_CLK_LED3_RFD( );


/* External variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/** @defgroup NFC07A1_NUCLEO_Exported_Functions
  * @{
  */
int32_t NFC07A1_LED_Init(NFC07A1_Led_E led);
int32_t NFC07A1_LED_DeInit(NFC07A1_Led_E led);
int32_t NFC07A1_LED_On(const NFC07A1_Led_E led);
int32_t NFC07A1_LED_Off(const NFC07A1_Led_E led);
int32_t NFC07A1_LED_Toggle(const NFC07A1_Led_E led);
int32_t NFC07A1_GPO_Init(void);
int32_t NFC07A1_GPO_DeInit(void);
int32_t NFC07A1_GPO_ReadPin(void);
int32_t NFC07A1_LPD_Init(void);
int32_t NFC07A1_LPD_DeInit(void);
int32_t NFC07A1_LPD_ReadPin(void);
int32_t NFC07A1_LPD_On(void);
int32_t NFC07A1_LPD_Off(void);
int32_t NFC07A1_LPD_Toggle(void);
void NFC07A1_GPO_IRQHandler(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 

#ifdef __cplusplus
  }
#endif

#endif /* NFC07A1_H */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
