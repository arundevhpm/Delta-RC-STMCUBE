/**
 ******************************************************************************
 * @file    nfc07a1_conf_template.h
 * @author  MMY Application Team
 * @brief   This file contains definitions for the NFC7 components bus interfaces
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
#ifndef NFC07A1_CONF_H
#define NFC07A1_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo_bus.h"
#include "stm32l4xx_nucleo_errno.h"

#include "stm32l4xx_hal_exti.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define NFC07A1_I2C_Init         BSP_I2C1_Init
#define NFC07A1_I2C_DeInit       BSP_I2C1_DeInit
#define NFC07A1_I2C_ReadReg16    BSP_I2C1_ReadReg16
#define NFC07A1_I2C_WriteReg16   BSP_I2C1_WriteReg16
#define NFC07A1_I2C_Recv         BSP_I2C1_Recv
#define NFC07A1_I2C_IsReady      BSP_I2C1_IsReady

#define NFC07A1_GetTick          HAL_GetTick

#define NFC07A1_LPD_PIN_PORT GPIOA
#define NFC07A1_LPD_PIN GPIO_PIN_8
#define NFC07A1_GPO_PIN_PORT GPIOA
#define NFC07A1_GPO_PIN GPIO_PIN_6
#define NFC07A1_NFCTAG_GPO_EXTI_LINE EXTI_LINE_6
#define NFC07A1_GPO_EXTI EXTI9_5_IRQn
#define H_EXTI_6 GPO_EXTI
#define NFC07A1_LED1_PIN_PORT GPIOB
#define NFC07A1_LED1_PIN GPIO_PIN_4
#define NFC07A1_LED2_PIN_PORT GPIOB
#define NFC07A1_LED2_PIN GPIO_PIN_5
#define NFC07A1_LED3_PIN_PORT GPIOA
#define NFC07A1_LED3_PIN GPIO_PIN_10

#define NFC07A1_NFCTAG_INSTANCE         (0)

#define NFC07A1_NFCTAG_GPO_PRIORITY     (0)

/* Exported variables --------------------------------------------------------*/
extern EXTI_HandleTypeDef GPO_EXTI;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* NFC07A1_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

