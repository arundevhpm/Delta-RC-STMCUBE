/**
  ******************************************************************************
 * @file    bas_app.c
 * @author  MCD Application Team
 * @brief   Battery Service Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#include "bas_app.h"
#include "ble.h"
//#include "stm32_seq.h"

#include "app_ble.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint8_t  Level;
//	float Level;
  uint8_t   TimerLevel_Id;
} BSAAPP_Context_t;


/* Private defines -----------------------------------------------------------*/
#define BASAPP_DEFAULT_BAT_LEVEL       100  /**100% */
#define BASAPP_DEFAULT_BAT_LEVEL_CHG   10000//(4880000/CFG_TS_TICK_VAL)  /**< 5s *///5000000
//#define BAS_UPDATE_TIME		(200/CFG_TS_TICK_VAL)		//5Sec

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

BSAAPP_Context_t BASAPP_Context[BLE_CFG_BAS_NUMBER];

/**
 * END of Section BLE_APP_CONTEXT
 */

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//static void BASAPP_UpdateLevel( void );
static void thread_BASProcess_entry(ULONG argument);

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
static TX_THREAD thread_BASProcess;
 TX_SEMAPHORE sem_BASProcessSignal;
/* Public functions ----------------------------------------------------------*/
extern uint16_t BattValInPercentage;
void BAS_Notification(BAS_Notification_evt_t *pNotification)
{
  switch(pNotification->BAS_Evt_Opcode)
  {
    case BAS_LEVEL_NOT_ENABLED_EVT:
      {
        HW_TS_Stop(BASAPP_Context[pNotification->ServiceInstance].TimerLevel_Id);
        HW_TS_Start(BASAPP_Context[pNotification->ServiceInstance].TimerLevel_Id, BASAPP_DEFAULT_BAT_LEVEL_CHG);
      }
      break;

    case BAS_LEVEL_NOT_DISABLED_EVT:
      {
        HW_TS_Stop(BASAPP_Context[pNotification->ServiceInstance].TimerLevel_Id);
      }
      break;

    case BAS_LEVEL_READ_EVT:
      {
        if(BASAPP_Context[pNotification->ServiceInstance].Level > 0)
          BASAPP_Context[pNotification->ServiceInstance].Level -= 1;
        else
          BASAPP_Context[pNotification->ServiceInstance].Level = BASAPP_DEFAULT_BAT_LEVEL;
        BAS_Update_Char(BATTERY_LEVEL_CHAR_UUID,
                        pNotification->ServiceInstance,
                        (uint8_t *)&BASAPP_Context[pNotification->ServiceInstance].Level);
      }
      break;

    default:
      break;
  }

  return;
}

void BASAPP_Init(TX_BYTE_POOL* p_byte_pool)
{
	CHAR* p_pointer;
	 uint8_t index;
  //UTIL_SEQ_RegTask( 1<< CFG_TASK_BAS_LEVEL_REQ_ID, UTIL_SEQ_RFU, BASAPP_Level );
	  tx_semaphore_create(&sem_BASProcessSignal, "sem_BASProcessSignal", 0);
	  tx_byte_allocate(p_byte_pool, (VOID**) &p_pointer, DEMO_STACK_SIZE_REDUCED, TX_NO_WAIT);
	  tx_thread_create(&thread_BASProcess,
	                   "thread_BASProcess",
	                   thread_BASProcess_entry,
	                   0,
	                   p_pointer,
	                   DEMO_STACK_SIZE_REDUCED,
	                   16,
	                   16,
	                   TX_NO_TIME_SLICE,
	                   TX_AUTO_START);

  
  /**
   * Initialize Level
   */
	for(index = 0; index < BLE_CFG_BAS_NUMBER; index++)
	{
		BASAPP_Context[index].Level = BASAPP_DEFAULT_BAT_LEVEL;
		BAS_Update_Char(BATTERY_LEVEL_CHAR_UUID, index, (uint8_t *)&BASAPP_Context[index].Level);


  /**
   * Create timer for Battery Level
   */
//		HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BASAPP_Context[index].TimerLevel_Id), hw_ts_Repeated, BASAPP_UpdateLevel);
//		HW_TS_Start(BASAPP_Context[index].TimerLevel_Id, BASAPP_DEFAULT_BAT_LEVEL_CHG);
	}

  return;
}
static void thread_BASProcess_entry(ULONG argument)
{
  UNUSED(argument);
  uint8_t index;

  for(;;)
  {
    tx_semaphore_get(&sem_BASProcessSignal, TX_WAIT_FOREVER);
	  BASAPP_Level( );
//	  tx_thread_sleep(10);
  }
}

void BASAPP_Level(void)
{
  uint8_t index;
  
  for(index = 0; index < BLE_CFG_BAS_NUMBER; index++)
  {
//    if(BASAPP_Context[index].Level > 0)
//      BASAPP_Context[index].Level = 1;
//    else
//      BASAPP_Context[index].Level = BASAPP_DEFAULT_BAT_LEVEL;
	  BASAPP_Context[index].Level = BattValInPercentage;

    BAS_Update_Char(BATTERY_LEVEL_CHAR_UUID, index, (uint8_t *)&BASAPP_Context[index].Level);
  }

  return;
}
//static void BASAPP_UpdateLevel( void )
//{
//  /**
//   * Notifying a new measure is available
//   */
//  tx_semaphore_put(&sem_BASProcessSignal);
///* USER CODE BEGIN HrMeas */
//
///* USER CODE END HrMeas */
//
//  return;
//}

//static void BASAPP_UpdateLevel( void )
//{
//  /**
//   * The code shall be executed in the background as aci command may be sent
//   * The background is the only place where the application can make sure a new aci command
//   * is not sent if there is a pending one
//   */
//  UTIL_SEQ_SetTask( 1<<CFG_TASK_BAS_LEVEL_REQ_ID, CFG_SCH_PRIO_0);
//
//  return;
//}


