/**
 ******************************************************************************
 * @file    nfc07a1_conf.h
 * @author  SRA Application Team
 * @version V0.0.1
 * @date    26-Nov-2018
 * @brief   This file contains definitions for the NFC7 components bus interfaces
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NFC07A1_CONF_H__
#define __NFC07A1_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif
 
 
#include "stm32wbxx_hal.h"
#include "i2c.h"
//#include "stm32wbxx_nucleo.h"

#define NFC07A1_I2C_Init         BSP_I2C1_Init
#define NFC07A1_I2C_DeInit       BSP_I2C1_DeInit
#define NFC07A1_I2C_ReadReg16    BSP_I2C1_MemRead
#define NFC07A1_I2C_WriteReg16   BSP_I2C1_MemWrite
#define NFC07A1_I2C_Recv         BSP_I2C1_Read
#define NFC07A1_I2C_IsReady      BSP_I2C1_IsDeviceReady

#define NFC07A1_LPD_PIN_PORT GPIOA
#define NFC07A1_LPD_PIN GPIO_PIN_8

#define NFC07A1_GPO_PIN_PORT NFC_GPO_RES_GPIO_Port //GPIOA
#define NFC07A1_GPO_PIN NFC_GPO_RES_Pin //GPIO_PIN_6
#ifdef Remote
#define NFC07A1_NFCTAG_GPO_EXTI_LINE 	EXTI_LINE_9
#else
#define NFC07A1_NFCTAG_GPO_EXTI_LINE 	EXTI_LINE_6
#endif
#define NFC07A1_GPO_EXTI NFC_GPO_RES_EXTI_IRQn //EXTI9_5_IRQn
//#define NFC07A1_LED1_PIN_PORT GPIOB
//#define NFC07A1_LED1_PIN GPIO_PIN_4
//#define NFC07A1_LED2_PIN_PORT GPIOB
//#define NFC07A1_LED2_PIN GPIO_PIN_5
//#define NFC07A1_LED3_PIN_PORT GPIOA
//#define NFC07A1_LED3_PIN GPIO_PIN_10

#define NFC07A1_NFCTAG_INSTANCE         (0)

#define NFC07A1_NFCTAG_GPO_PRIORITY     (0)

#define BSP_ERROR_NONE (0)

#ifdef __cplusplus
}
#endif

#endif /* __NFC07A1_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

