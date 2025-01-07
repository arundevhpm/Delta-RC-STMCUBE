/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    ux_device_keyboard.c
 * @author  MCD Application Team
 * @brief   USBX Device HID Keyboard applicative source file
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
#include "ux_device_keyboard.h"
#include "KBD.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_HID* hid_keyboard;



extern TX_QUEUE KBD_Queue;
extern uint8_t* KBD_Queue_DestPtr;
extern uint8_t KeyDestSequenceBuffer[MaxKBDMsgSize];
extern keyboard_report_t keyboard_report;
//extern keyboard_report_t* keyboard_report_ptr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_HID_Keyboard_Activate
  *         This function is called when insertion of a HID Keyboard device.
  * @param  hid_instance: Pointer to the hid class instance.
  * @retval none
  */


VOID USBD_HID_Keyboard_Activate(VOID *hid_instance)
{
  /* USER CODE BEGIN USBD_HID_Keyboard_Activate */
    UX_PARAMETER_NOT_USED(hid_instance);
    hid_keyboard = (UX_SLAVE_CLASS_HID*)hid_instance;
  /* USER CODE END USBD_HID_Keyboard_Activate */

  return;
}

/**
  * @brief  USBD_HID_Keyboard_Deactivate
  *         This function is called when extraction of a HID Keyboard device.
  * @param  hid_instance: Pointer to the hid class instance.
  * @retval none
  */
VOID USBD_HID_Keyboard_Deactivate(VOID *hid_instance)
{
  /* USER CODE BEGIN USBD_HID_Keyboard_Deactivate */
    UX_PARAMETER_NOT_USED(hid_instance);
    hid_keyboard = UX_NULL;
  /* USER CODE END USBD_HID_Keyboard_Deactivate */

  return;
}

/**
  * @brief  USBD_HID_Keyboard_SetReport
  *         This function is invoked when the host sends a HID SET_REPORT
  *         to the application over Endpoint 0.
  * @param  hid_instance: Pointer to the hid class instance.
  * @param  hid_event: Pointer to structure of the hid event.
  * @retval status
  */
UINT USBD_HID_Keyboard_SetReport(UX_SLAVE_CLASS_HID *hid_instance,
                                 UX_SLAVE_CLASS_HID_EVENT *hid_event)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_HID_Keyboard_SetReport */
    UX_PARAMETER_NOT_USED(hid_instance);
    UX_PARAMETER_NOT_USED(hid_event);
  /* USER CODE END USBD_HID_Keyboard_SetReport */

  return status;
}

/**
  * @brief  USBD_HID_Keyboard_GetReport
  *         This function is invoked when host is requesting event through
  *         control GET_REPORT request.
  * @param  hid_instance: Pointer to the hid class instance.
  * @param  hid_event: Pointer to structure of the hid event.
  * @retval status
  */
UINT USBD_HID_Keyboard_GetReport(UX_SLAVE_CLASS_HID *hid_instance,
                                 UX_SLAVE_CLASS_HID_EVENT *hid_event)
{
  UINT status = UX_SUCCESS;

  /* USER CODE BEGIN USBD_HID_Keyboard_GetReport */
    UX_PARAMETER_NOT_USED(hid_instance);
    UX_PARAMETER_NOT_USED(hid_event);
  /* USER CODE END USBD_HID_Keyboard_GetReport */

  return status;
}

/* USER CODE BEGIN 1 */
VOID usbx_hid_thread_entry(ULONG thread_input)
{
	uint32_t status;
  UX_SLAVE_DEVICE *device;
  UX_SLAVE_CLASS_HID_EVENT hid_event;

  UX_PARAMETER_NOT_USED(thread_input);
  int result = 0xff;
  TX_QUEUE* KBD_Queue_ptr;
  KBD_Queue_DestPtr = KeyDestSequenceBuffer;
  KBD_Queue_ptr = &KBD_Queue;

  device = &_ux_system_slave->ux_system_slave_device;

  ux_utility_memory_set(&hid_event, 0, sizeof(UX_SLAVE_CLASS_HID_EVENT));

  while (1)
  {
    /* Check if the device state already configured */
//    if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (hid_keyboard != UX_NULL))
//    {
//      /* sleep for 10ms */
//      tx_thread_sleep(MS_TO_TICK(10));
//
//      /* Check if user button is pressed */
//
//    }
//    else
//    {
//      /* Sleep thread for 10ms */
//      tx_thread_sleep(MS_TO_TICK(10));
//    }

	  result = tx_queue_receive(KBD_Queue_ptr,KeyDestSequenceBuffer,1);
	  if(result == TX_SUCCESS)
	  {
			memset(&keyboard_report,0,sizeof(keyboard_report));

			hid_event.ux_device_class_hid_event_length = 8;
			memcpy(hid_event.ux_device_class_hid_event_buffer, KBD_Queue_DestPtr, hid_event.ux_device_class_hid_event_length);
			hid_event.ux_device_class_hid_event_report_id=0x01;
			status =ux_device_class_hid_event_set(hid_keyboard, &hid_event);


	  }
  }
}
/* USER CODE END 1 */
