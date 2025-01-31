/**
  ******************************************************************************
  * @file    hids_app.c
  * @author  MCD Application Team
  * @brief   Human Interface Device Service Application
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



/* Includes ------------------------------------------------------------------*/
#include "app_common.h"

#include "dbg_trace.h"
#include "app_ble.h"
#include "ble.h"
#include "hids_app.h"
//#include "stm32_seq.h"
#include <time.h>
#include "hids_menu.h"
#include "tx_api.h"
#include "main.h"
#include "KBD.h"
/* Private typedef -----------------------------------------------------------*/
#if((BLE_CFG_HIDS_INPUT_REPORT_NB != 0) || (BLE_CFG_HIDS_KEYBOARD_DEVICE != 0) || (BLE_CFG_HIDS_MOUSE_DEVICE != 0))
typedef struct
{
#if(BLE_CFG_HIDS_INPUT_REPORT_NB != 0)
  uint8_t ReportNotificationEnabled[BLE_CFG_HIDS_INPUT_REPORT_NB];
#endif
#if(BLE_CFG_HIDS_KEYBOARD_DEVICE != 0)
  uint8_t KeyboardInputNotificationEnabled;
#endif
#if(BLE_CFG_HIDS_MOUSE_DEVICE != 0)
  uint8_t MouseInputNotificationEnabled;
#endif
} HIDSAPP_Context_t;
#endif

//typedef struct
//{
//  uint8_t buttons;
//  int8_t x;
//  int8_t y;
//  int8_t wheel;
//} mouse_report_t;

extern TX_QUEUE KBD_Queue;
extern uint8_t* KBD_Queue_DestPtr;
extern uint8_t KeyDestSequenceBuffer[MaxKBDMsgSize];
extern keyboard_report_t keyboard_report;
extern keyboard_report_t* keyboard_report_ptr;

#define KEY_A 0x04 // Keyboard a and A

#define KEYBOARD_REPORT_SIZE     65
/* Private defines -----------------------------------------------------------*/
#define MOUSE_REPORT_SIZE       52

#define ENABLED         1
#define DISABLED        0

#define BOUNCE_THRESHOLD                20U
#define BUTTON_PRESSED                  GPIO_PIN_RESET
  
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t report_keyboard[KEYBOARD_REPORT_SIZE] =
{
  0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x01,                    //
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x6F,                    //   LOGICAL_MAXIMUM (111)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x6F,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION




};

static uint8_t report_mouse[MOUSE_REPORT_SIZE] =
{
  0x05, 0x01,         /* USAGE_PAGE (Generic Desktop) */
  0x09, 0x02,         /* USAGE (Mouse) */
  0xa1, 0x01,         /* COLLECTION (Application) */
  0x09, 0x01,         /*   USAGE (Pointer) */
  0xa1, 0x00,         /*   COLLECTION (Physical) */
  0x05, 0x09,         /*     USAGE_PAGE (Button) */
  0x19, 0x01,         /*     USAGE_MINIMUM (Button 1) */
  0x29, 0x03,         /*     USAGE_MAXIMUM (Button 3) */
  0x15, 0x00,         /*     LOGICAL_MINIMUM (0) */
  0x25, 0x01,         /*     LOGICAL_MAXIMUM (1) */
  0x95, 0x03,         /*     REPORT_COUNT (3) */
  0x75, 0x01,         /*     REPORT_SIZE (1) */
  0x81, 0x02,         /*     INPUT (Data,Var,Abs) */
  0x95, 0x01,         /*     REPORT_COUNT (1) */
  0x75, 0x05,         /*     REPORT_SIZE (5) */
  0x81, 0x03,         /*     INPUT (Cnst,Var,Abs) */
  0x05, 0x01,         /*     USAGE_PAGE (Generic Desktop) */
  0x09, 0x30,         /*     USAGE (X) */
  0x09, 0x31,         /*     USAGE (Y) */
  0x09, 0x38,         /*     USAGE (Wheel) */
  0x15, 0x81,         /*     LOGICAL_MINIMUM (-127) */
  0x25, 0x7f,         /*     LOGICAL_MAXIMUM (127) */
  0x75, 0x08,         /*     REPORT_SIZE (8) */
  0x95, 0x03,         /*     REPORT_COUNT (3) */
  0x81, 0x06,         /*     INPUT (Data,Var,Rel) */
  0xc0,               /*   END_COLLECTION (Physical) */
  0xc0,               /* END_COLLECTION (Application) */
};


/**
 * START of Section BLE_APP_CONTEXT
 */

#if((BLE_CFG_HIDS_INPUT_REPORT_NB != 0) || (BLE_CFG_HIDS_KEYBOARD_DEVICE != 0) || (BLE_CFG_HIDS_MOUSE_DEVICE != 0))
HIDSAPP_Context_t HIDSAPP_Context[BLE_CFG_HIDS_NUMBER];
#endif

/**
 * END of Section BLE_APP_CONTEXT
 */

/* Global variables ----------------------------------------------------------*/
static TX_THREAD thread_HIDProcess;
TX_SEMAPHORE sem_HIDProcessSignal;

/* Private function prototypes -----------------------------------------------*/
static void thread_HIDProcess_entry(ULONG argument);
void Get_KBD_Buffer(uint8_t* BuffPtr);
/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
void HIDS_Notification(HIDS_App_Notification_evt_t *pNotification)
{
  switch(pNotification->HIDS_Evt_Opcode)
  {

#if(BLE_CFG_HIDS_INPUT_REPORT_NB != 0)
    case HIDS_REPORT_NOTIFICATION_ENABLED:
      {
        BLE_DBG_APP_MSG("HIDS_REPORT_NOTIFICATION_ENABLED\n");
        HIDSAPP_Context[pNotification->Index].ReportNotificationEnabled[pNotification->Index] = ENABLED;
      }
      break;

    case HIDS_REPORT_NOTIFICATION_DISABLED:
      {
        BLE_DBG_APP_MSG("HIDS_REPORT_NOTIFICATION_DISABLED\n");
        HIDSAPP_Context[pNotification->Index].ReportNotificationEnabled[pNotification->Index] = DISABLED;
      }
      break;
#endif
      
#if(BLE_CFG_HIDS_KEYBOARD_DEVICE != 0)
    case HIDS_KEYB_INPUT_NOTIFY_ENABLED:
      {
        BLE_DBG_APP_MSG("HIDS_KEYB_INPUT_NOTIFY_ENABLED\n");
        HIDSAPP_Context[pNotification->Index].KeyboardInputNotificationEnabled = ENABLED;
      }
      break;

    case HIDS_KEYB_INPUT_NOTIFY_DISABLED:
      {
        BLE_DBG_APP_MSG("HIDS_KEYB_INPUT_NOTIFY_DISABLED\n");
        HIDSAPP_Context[pNotification->Index].KeyboardInputNotificationEnabled = DISABLED;
      }
      break;
#endif

#if(BLE_CFG_HIDS_MOUSE_DEVICE != 0)          
    case HIDS_MOUSE_INPUT_NOTIFY_ENABLED:
      {
        BLE_DBG_APP_MSG("HIDS_MOUSE_INPUT_NOTIFY_ENABLED\n");
        HIDSAPP_Context[pNotification->Index].MouseInputNotificationEnabled = ENABLED;
      }
      break;

    case HIDS_MOUSE_INPUT_NOTIFY_DISABLED:
      {
        BLE_DBG_APP_MSG("HIDS_MOUSE_INPUT_NOTIFY_DISABLED\n");
        HIDSAPP_Context[pNotification->Index].MouseInputNotificationEnabled = DISABLED;
      }
      break;
#endif

//#if(BLE_CFG_HIDS_OUTPUT_REPORT_NB != 0)
#if(BLE_CFG_HIDS_REPORT_CHAR != 0)
    case HIDS_OUTPUT_REPORT:
      {
        uint8_t i;
        
        BLE_DBG_APP_MSG("HIDS_OUTPUT_REPORT\n");
        BLE_DBG_HIDS_MSG("HID Instance %d Report %d \n", 
                          pNotification->Instance,
                          pNotification->Index); 
    
        for(i = 0; i < pNotification->ReportLength; i++)
          BLE_DBG_HIDS_MSG("Report[%d] 0x%X \n",
                           i,
                           pNotification->pReport[i]);
      }
      break;
#endif
    
#if((BLE_CFG_HIDS_MOUSE_DEVICE != 0) && (BLE_CFG_HIDS_MOUSE_INPUT_WRITE != 0))
    case HIDS_MOUSE_INPUT_REPORT:
      {
        uint8_t i;
        
        BLE_DBG_APP_MSG("HIDS_MOUSE_INPUT_REPORT\n");
        BLE_DBG_HIDS_MSG("HID Instance %d Report %d \n", 
                          pNotification->Instance,
                          pNotification->Index); 
    
        for(i = 0; i < pNotification->ReportLength; i++)
          BLE_DBG_HIDS_MSG("Report[%d] 0x%X \n",
                           i,
                           pNotification->pReport[i]);
      }
      break;
#endif
      
#if((BLE_CFG_HIDS_KEYBOARD_DEVICE != 0) && (BLE_CFG_HIDS_KEYBOARD_INPUT_WRITE != 0))
    case HIDS_KEYBOARD_INPUT_REPORT:
      {
        uint8_t i;
        
        BLE_DBG_APP_MSG("HIDS_KEYBOARD_INPUT_REPORT\n");
        BLE_DBG_HIDS_MSG("HID Instance %d Report %d \n", 
                          pNotification->Instance,
                          pNotification->Index); 
    
        for(i = 0; i < pNotification->ReportLength; i++)
          BLE_DBG_HIDS_MSG("Report[%d] 0x%X \n",
                           i,
                           pNotification->pReport[i]);
      }
      break;

    case HIDS_KEYBOARD_OUTPUT_REPORT:
      {
        uint8_t i;
        
        BLE_DBG_APP_MSG("HIDS_KEYBOARD_OUTPUT_REPORT\n");
        BLE_DBG_HIDS_MSG("HID Instance %d Report %d \n", 
                          pNotification->Instance,
                          pNotification->Index); 
    
        for(i = 0; i < pNotification->ReportLength; i++)
          BLE_DBG_HIDS_MSG("Report[%d] 0x%X \n",
                           i,
                           pNotification->pReport[i]);
      }
      break;
#endif
      
    default:
      break;
  }

  return;
}


void HIDSAPP_Init(TX_BYTE_POOL* p_byte_pool)
{
	CHAR* p_pointer;
  tBleStatus result = BLE_STATUS_INVALID_PARAMS;

//  UTIL_SEQ_RegTask( 1<< CFG_TASK_HID_UPDATE_REQ_ID, UTIL_SEQ_RFU, HIDSAPP_Profile_UpdateChar );
  tx_semaphore_create(&sem_HIDProcessSignal, "sem_HIDProcessSignal", 0);
  tx_byte_allocate(p_byte_pool, (VOID**) &p_pointer, DEMO_STACK_SIZE_REDUCED, TX_NO_WAIT);
  tx_thread_create(&thread_HIDProcess,
                   "thread_HIDProcess",
                   thread_HIDProcess_entry,
                   0,
                   p_pointer,
                   DEMO_STACK_SIZE_REDUCED,
                   16,
                   16,
                   TX_NO_TIME_SLICE,
                   TX_AUTO_START);
/*
  result = HIDS_Update_Char(REPORT_MAP_CHAR_UUID, 
                            0, 
                            0, 
                            MOUSE_REPORT_SIZE,
                            (uint8_t *)&report_mouse);
 */
  result = HIDS_Update_Char(REPORT_MAP_CHAR_UUID,
                            0,
                            0,
                            KEYBOARD_REPORT_SIZE,
                            (uint8_t *)&report_keyboard);

  if( result == BLE_STATUS_SUCCESS )
  {
    BLE_DBG_APP_MSG("Report Map Successfully Sent\n");
  }
  else 
  {
    BLE_DBG_APP_MSG("Sending of Report Map Failed error 0x%X\n", result);
  }
}

static void thread_HIDProcess_entry(ULONG argument)
{
  UNUSED(argument);
  int i;

  int result = 0xff;
  TX_QUEUE* KBD_Queue_ptr;
  KBD_Queue_DestPtr = KeyDestSequenceBuffer;
  KBD_Queue_ptr = &KBD_Queue;
  for(;;)
  {
//    tx_semaphore_get(&sem_HIDProcessSignal, TX_WAIT_FOREVER);

	  result = tx_queue_receive(KBD_Queue_ptr,KeyDestSequenceBuffer,1);
	  if(result == TX_SUCCESS)
	  {
			memset(&keyboard_report,0,sizeof(keyboard_report));
			keyboard_report_ptr = &keyboard_report.modifier;
//			 KBD_Queue_DestPtr = KeyDestSequenceBuffer;
			memcpy(keyboard_report_ptr, KBD_Queue_DestPtr, sizeof(keyboard_report_t)-1);
			keyboard_report.reportID = 0x01;
		  result = HIDS_Update_Char(REPORT_CHAR_UUID,
		                                0,
		                                0,
		                                sizeof(keyboard_report_t),
		                                (uint8_t *)& keyboard_report);
//			KBD_Queue_DestPtr = &KeyDestSequenceBuffer;

	  }

  }
}

/**
 * @brief  Alert Notification Application service update characteristic
 * @param  None
 * @retval None
 */
void Get_KBD_Buffer(uint8_t* BuffPtr)
{
//	memset((unsigned char*)&keyboard_report,0,sizeof(keyboard_report_t));
//	memcpy((unsigned char*)&keyboard_report, BuffPtr, sizeof(keyboard_report_t));
}


void HIDSAPP_Profile_UpdateChar(void)
{
/*  uint8_t action_type; */
//  keyboard_report_t keyboard_report={0};

//  if(BUTTON_Flag!=0)
//  {
//    if( BUTTON_Flag == 1)
//    {
//      keyboard_report.KEY1=KEY_A; //pressed
//      keyboard_report.modifier=0x02;
//     // keyboard_report.modifier=KEY_MOD_LMETA;
//    }
//
// //   keyboard_report.reportID=0x01;
//
//
//    tBleStatus result = BLE_STATUS_INVALID_PARAMS;
//
//    result = HIDS_Update_Char(REPORT_CHAR_UUID,
//                              0,
//                              0,
//                              sizeof(keyboard_report_t),
//                              (uint8_t *)& keyboard_report);
//
//    if( result == BLE_STATUS_SUCCESS )
//    {
//      BLE_DBG_APP_MSG("Keyboard Report 0x%x %d %d %d Successfully Sent\n",
//                       keyboard_report.modifier,
//                       keyboard_report.KEY1,
//                       keyboard_report.KEY2,
//                       keyboard_report.KEY3);
//    }
//    else
//    {
//      BLE_DBG_APP_MSG("Sending of Keyboard Report Failed error 0x%X\n", result);
//    }
//
//    if(BUTTON_Flag == 0xAA)
//      BUTTON_Flag = 0;
//    if(BUTTON_Flag != 0)
//    {
//      /* Key Release */
//      BUTTON_Flag = 0xAA;
//      tx_semaphore_put(&sem_HIDProcessSignal);
////      UTIL_SEQ_SetTask( 1<<CFG_TASK_HID_UPDATE_REQ_ID, CFG_SCH_PRIO_0);
//    }
//  }
}
//void APP_BLE_Key_Button1_Action(void)
//{
//	/*
//  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
//
//  ret = aci_gap_clear_security_db();
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("==>> aci_gap_clear_security_db - Fail, result: %d \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("==>> aci_gap_clear_security_db - Success\n");
//  }
//  */
//	  uint32_t t0 = 0,t1 = 1;
//
//	  t0 = HAL_GetTick(); /* SW1 press timing */
//
//	  while(BSP_PB_GetState(BUTTON_SW1) == BUTTON_PRESSED);
//	  t1 = HAL_GetTick(); /* SW1 release timing */
//
//	  if((t1 - t0) > BOUNCE_THRESHOLD)
//	  {
//	    /* Button 1 short press action */
//	   // UTIL_SEQ_SetTask(1<<CFG_TASK_HID_UPDATE_REQ_ID, CFG_SCH_PRIO_0);
//		  tx_semaphore_put(&sem_HIDProcessSignal);
//	  }
//}
//void APP_BLE_Key_Button3_Action(void)
//{
//	  uint32_t t0 = 0,t1 = 1;
//
//	  t0 = HAL_GetTick(); /* SW1 press timing */
//
//	  while(BSP_PB_GetState(BUTTON_SW1) == BUTTON_PRESSED);
//	  t1 = HAL_GetTick(); /* SW1 release timing */
//
//	  if((t1 - t0) > BOUNCE_THRESHOLD)
//	  {
//	    /* Button 1 short press action */
//	   // UTIL_SEQ_SetTask(1<<CFG_TASK_HID_UPDATE_REQ_ID, CFG_SCH_PRIO_0);
//		  tx_semaphore_put(&sem_HIDProcessSignal);
//	  }
//}

/*
void HIDSAPP_Profile_UpdateChar(void)
{
  uint8_t action_type;
  mouse_report_t mouse_report;
  
  HIDS_Menu(&action_type, (uint8_t *)&mouse_report);
    
  if(action_type == 1)
  {
    tBleStatus result = BLE_STATUS_INVALID_PARAMS;

    result = HIDS_Update_Char(REPORT_CHAR_UUID, 
                              0, 
                              0, 
                              sizeof(mouse_report_t),
                              (uint8_t *)& mouse_report);

    if( result == BLE_STATUS_SUCCESS )
    {
      BLE_DBG_APP_MSG("Mouse Report 0x%x %d %d %d Successfully Sent\n",
                       mouse_report.buttons,
                       mouse_report.x,
                       mouse_report.y,
                       mouse_report.wheel);
    }
    else 
    {
      BLE_DBG_APP_MSG("Sending of Mouse Report Failed error 0x%X\n", result);
    }
  }
}
*/

