/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_ble.c
  * @author  MCD Application Team
  * @brief   BLE Application
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

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "tx_api.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"

//#include "hrs_app.h"
#include "dis_app.h"
#include "limits.h"

#include "hids_app.h"
#include "tx_user.h"
#include "p2p_server_app.h"
#include "Periodic.h"
#include "bas_app.h"
#include "ias_app.h"
#include "lls_app.h"
#include "tps_app.h"
#include "nfc07a1_nfctag.h"
#include "lib_NDEF_Bluetooth.h"
#include "lib_wrapper.h"
#include "nfc07a1.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
#define OOB_SECURE 	1
#define USE_RANDOM_ADDR	1
/**
 * security parameters structure
 */
extern PulsePattern BLEConnectLED;
extern TX_SEMAPHORE sem_P2PSProcessSignal;
extern EventGrp Ueventsgrp;
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;
  /**
   * Flag to tell whether OOB data has
   * to be used during the pairing process
   */
  uint8_t OOB_Data_Present;

  /**
   * OOB data to be used in the pairing process if
   * OOB_Data_Present is set to TRUE
   */
  uint8_t OOB_Data[16];
  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
  /* USER CODE BEGIN tSecurityParams*/

  /* USER CODE END tSecurityParams */
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{
  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle[CFG_MAX_CONNECTION];

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];
  /* USER CODE BEGIN BleGlobalContext_t*/

  /* USER CODE END BleGlobalContext_t */
}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status[CFG_MAX_CONNECTION];

  /**
   * ID of the Advertising Timeout
   */
  uint8_t Advertising_mgr_timer_Id;
  /* USER CODE BEGIN PTD_1*/
  uint8_t SwitchOffGPIO_timer_Id;
  /* USER CODE END PTD_1 */
}BleApplicationContext_t;

/* USER CODE BEGIN PTD */
#if (CFG_LPM_SUPPORTED == 1)
typedef struct
{
  uint32_t LpTXTimeLeftOnEntry;
  uint8_t LpTXTimerThreadx_Id;
} LpTXTimerContext_t;
#endif
/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define BD_ADDR_SIZE_LOCAL    6

/* USER CODE BEGIN PD */
#define LED_ON_TIMEOUT                 (0.005*1000*1000/CFG_TS_TICK_VAL) /**< 5ms */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t a_MBdAddr[BD_ADDR_SIZE_LOCAL] =
{
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
};

static uint8_t a_BdAddrUdn[BD_ADDR_SIZE_LOCAL];

/**
 *   Identity root key used to derive IRK and DHK(Legacy)
 */
static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IR;

/**
 * Encryption root key used to derive LTK(Legacy) and CSRK
 */
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ER;

/**
 * These are the two tags used to manage a power failure during OTA
 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
 * The MagicKeywordvalue is checked in the ble_ota application
 */
PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29 ;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

static BleApplicationContext_t BleApplicationContext;
static uint16_t AdvIntervalMin, AdvIntervalMax;
static const char *name = "Philips RC 1.3.8";
//static const char a_LocalName[] = {AD_TYPE_COMPLETE_LOCAL_NAME ,'H','R','S','T','X'};
static const char a_LocalName[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'P','h','i','l','i','p','s','R','C'};//,'1','.','3','.','3'};//,' ','C','o','n','t','r','o','l',' ','1','.','3','.','0'};//'S','T','M','3','2','W','B','X'};//
uint8_t a_ManufData[14] = {sizeof(a_ManufData)-1,
                           AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                           0x01/*SKD version */,
                           0x00 /* Generic*/,
                           0x00 /* GROUP A Feature  */,
                           0x00 /* GROUP A Feature */,
                           0x00 /* GROUP B Feature */,
                           0x00 /* GROUP B Feature */,
                           0x00, /* BLE MAC start -MSB */
                           0x00,
                           0x00,
                           0x00,
                           0x00,
                           0x00, /* BLE MAC stop */
                          };

/* USER CODE BEGIN PV */
#if (CFG_LPM_SUPPORTED == 1)
static LpTXTimerContext_t LpTXTimerContext;
#endif
/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/
static TX_MUTEX     mtx_hci;
static TX_SEMAPHORE sem_hci;
static TX_THREAD    thread_HciUserEvtProcess;
static TX_SEMAPHORE sem_HciUserEvtProcessSignal;
static TX_THREAD    thread_AdvUpdateProcess;
static TX_THREAD    thread_Disconnection;
static TX_SEMAPHORE sem_AdvUpdateProcessSignal;

TX_SEMAPHORE sem_HIDUnpairProcessSignal;

/* Private function prototypes -----------------------------------------------*/
static void thread_AdvUpdateProcess_entry(ULONG thread_input);
static void thread_Disconnection_entry(ULONG thread_input);

static void thread_HciUserEvtProcess_entry(ULONG thread_input);
static void BLE_UserEvtRx(void* p_Payload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t Status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress(void);
static void Adv_Request(APP_BLE_ConnStatus_t NewStatus);
static void Add_Advertisment_Service_UUID(uint16_t servUUID);
static void Adv_Mgr(void);
static void Adv_Update(void);
static void runOOB(void);
//static void Switch_OFF_GPIO(void);
/* USER CODE BEGIN PFP */
#if (CFG_LPM_SUPPORTED == 1)
static void APP_BLE_Threadx_LpTimerCb(void);
#endif
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init(TX_BYTE_POOL* p_byte_pool)
{
	 uint8_t index;
  SHCI_CmdStatus_t status;
  /* USER CODE BEGIN APP_BLE_Init_1 */

  /* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
     CFG_BLE_NUM_GATT_ATTRIBUTES,
     CFG_BLE_NUM_GATT_SERVICES,
     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
     CFG_BLE_NUM_LINK,
     CFG_BLE_DATA_LENGTH_EXTENSION,
     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
     CFG_BLE_MBLOCK_COUNT,
     CFG_BLE_MAX_ATT_MTU,
     CFG_BLE_PERIPHERAL_SCA,
     CFG_BLE_CENTRAL_SCA,
     CFG_BLE_LS_SOURCE,
     CFG_BLE_MAX_CONN_EVENT_LENGTH,
     CFG_BLE_HSE_STARTUP_TIME,
     CFG_BLE_VITERBI_MODE,
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG,
     CFG_BLE_MAX_ADV_SET_NBR,
     CFG_BLE_MAX_ADV_DATA_LEN,
     CFG_BLE_TX_PATH_COMPENS,
     CFG_BLE_RX_PATH_COMPENS,
     CFG_BLE_CORE_VERSION,
     CFG_BLE_OPTIONS_EXT
    }
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init();

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  CHAR* p_pointer;
  tx_mutex_create(&mtx_hci, "mtx_hci", TX_NO_INHERIT);
  tx_semaphore_create(&sem_hci, "sem_hci", 0);
  tx_semaphore_create(&sem_HciUserEvtProcessSignal, "sem_HciUserEvtProcessSignal", 0);
  tx_byte_allocate(p_byte_pool, (VOID**) &p_pointer, DEMO_STACK_SIZE_LARGE+2048, TX_NO_WAIT);
  tx_thread_create(&thread_HciUserEvtProcess,
                   "thread_HciUserEvtProcess",
                   thread_HciUserEvtProcess_entry,
                   0,
                   p_pointer,
                   DEMO_STACK_SIZE_LARGE+2048,
                   16,
                   16,
                   TX_NO_TIME_SLICE,
                   TX_AUTO_START);

  /**
   * Starts the BLE Stack on CPU2
   */
  status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
  if (status != SHCI_Success)
  {
    APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
    /* if you are here, maybe CPU2 doesn't contain STM32WB_Copro_Wireless_Binaries, see Release_Notes.html */
    Error_Handler();
  }
  else
  {
    APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
  }

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * Initialization of the BLE App Context
   */
//  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
//  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;
  /**
   * Initialization of the BLE App Context
   */
  for(index = 0; index < CFG_MAX_CONNECTION; index++)
  {
    BleApplicationContext.Device_Connection_Status[index] = APP_BLE_IDLE;
    BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = 0xFFFF;
  }
  /**
   * From here, all initialization are BLE application specific
   */
  tx_semaphore_create(&sem_AdvUpdateProcessSignal, "sem_AdvUpdateProcessSignal", 0);
  tx_byte_allocate(p_byte_pool, (VOID**) &p_pointer, DEMO_STACK_SIZE_REDUCED, TX_NO_WAIT);
  tx_thread_create(&thread_AdvUpdateProcess,
                   "thread_AdvUpdateProcess",
                   thread_AdvUpdateProcess_entry,
                   0,
                   p_pointer,
                   DEMO_STACK_SIZE_REDUCED,
                   16,
                   16,
                   TX_NO_TIME_SLICE,
                   TX_AUTO_START);
  tx_semaphore_create(&sem_HIDUnpairProcessSignal, "sem_HIDUnpairProcessSignal", 0);

  tx_byte_allocate(p_byte_pool, (VOID**) &p_pointer, DEMO_STACK_SIZE_REDUCED1, TX_NO_WAIT);
  tx_thread_create(&thread_Disconnection,
                   "thread_Disconnection",
				   thread_Disconnection_entry,
                   0,
                   p_pointer,
				   DEMO_STACK_SIZE_REDUCED1,
                   16,
                   16,
                   TX_NO_TIME_SLICE,
                   TX_AUTO_START);

  /**
   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
   */
#if (BLE_CFG_OTA_REBOOT_CHAR != 0)
  a_ManufData[sizeof(a_ManufData)-8] = CFG_FEATURE_OTA_REBOOT;
#endif /* BLE_CFG_OTA_REBOOT_CHAR != 0 */

  /**
   * Initialize DIS Application
   */
  DISAPP_Init();

  /**
   * Initialize HRS Application
   */
  //HRSAPP_Init(p_byte_pool);

  /* USER CODE BEGIN APP_BLE_Init_3 */
  /**
   * Initialize BAS Application
   */
  BASAPP_Init(p_byte_pool);
  /**
   * Initialize Human Interface Device Service
   */
  HIDSAPP_Init(p_byte_pool);
  /* USER CODE END APP_BLE_Init_3 */
  /**
   * Initialize P2P Server Application
   */
  P2PS_APP_Init(p_byte_pool);

  /**
   * Initialize Immediate Alert Service
   */
  IASAPP_Init();

  /**
   * Initialize Link loss Service
   */
  LLSAPP_Init();

  /**
   * Initialize Tx Power Service
   */
  TPSAPP_Init();
  /**
   * Create timer to handle the connection state machine
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Mgr);
  /**
    * Create timer to handle the Led Switch OFF
    */
//   HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.SwitchOffGPIO_timer_Id), hw_ts_SingleShot, Switch_OFF_GPIO);

#if (CFG_LPM_SUPPORTED == 1)
  /**
   * Create low power timer to handle the user ThreadX timers
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(LpTXTimerContext.LpTXTimerThreadx_Id), hw_ts_SingleShot, APP_BLE_Threadx_LpTimerCb);
#endif
  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_16_BIT_SERV_UUID;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;
 // Add_Advertisment_Service_UUID(HEART_RATE_SERVICE_UUID);
  Add_Advertisment_Service_UUID(DEVICE_INFORMATION_SERVICE_UUID);
  Add_Advertisment_Service_UUID(BATTERY_SERVICE_UUID);
  Add_Advertisment_Service_UUID(HUMAN_INTERFACE_DEVICE_SERVICE_UUID);

  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /**
   * Start to Advertise to be connected by Collector
   */
  Adv_Request(APP_BLE_FAST_ADV);

  /* USER CODE BEGIN APP_BLE_Init_2 */

  /* USER CODE END APP_BLE_Init_2 */

  return;
}

//SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
//{
//  hci_event_pckt    *p_event_pckt;
//  evt_le_meta_event *p_meta_evt;
//  evt_blecore_aci   *p_blecore_evt;
//  uint8_t           Tx_phy, Rx_phy;
//  tBleStatus        ret = BLE_STATUS_INVALID_PARAMS;
//  hci_le_connection_complete_event_rp0        *p_connection_complete_event;
//  hci_disconnection_complete_event_rp0        *p_disconnection_complete_event;
//  hci_le_phy_update_complete_event_rp0        *p_evt_le_phy_update_complete;
//  hci_le_enhanced_connection_complete_event_rp0 *p_enhanced_connection_complete_event;
//  uint8_t index;
//
//#if (CFG_DEBUG_APP_TRACE != 0)
//  hci_le_connection_update_complete_event_rp0 *p_connection_update_complete_event;
//#endif /* CFG_DEBUG_APP_TRACE != 0 */
//
//  /* USER CODE BEGIN SVCCTL_App_Notification */
//
//  /* USER CODE END SVCCTL_App_Notification */
//
//  p_event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) p_Pckt)->data;
//
//  switch (p_event_pckt->evt)
//  {
//    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
//    {
//      p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) p_event_pckt->data;
//
//      APP_DBG_MSG("HCI_DISCONNECTION_COMPLETE_EVT_CODE for connection handle 0x%x\n",
//                  p_disconnection_complete_event->Connection_Handle);
//      /* Find index of the handle deconnected */
//      index = 0;
//      while((index < CFG_MAX_CONNECTION) &&
//          (BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] != p_disconnection_complete_event->Connection_Handle))
//      {
//        index++;
//      }
//
//      if(index < CFG_MAX_CONNECTION)
//      {
//        APP_DBG_MSG("Index of the handle deconnected: %d\n", index);
//        BleApplicationContext.Device_Connection_Status[index] = APP_BLE_IDLE;
//        BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] =
//            0xFFFF;
//      }
//      else
//      {
//        APP_DBG_MSG("No index found for the handle discconnected !\n");
//      }
//
//      /* USER CODE BEGIN EVT_DISCONN_COMPLETE_1 */
////#ifndef DISABLE_HR
////      HRS_App_Notification_evt_t stopHRNotif = {.HRS_Evt_Opcode = HRS_NOTIFICATION_DISABLED};
////      HRS_Notification(&stopHRNotif);
////#endif
//      HIDS_App_Notification_evt_t stopHIDNotif = {.HIDS_Evt_Opcode = HIDS_REPORT_NOTIFICATION_DISABLED};
//      HIDS_Notification(&stopHIDNotif);
//      /* USER CODE END EVT_DISCONN_COMPLETE_1 */
//
//      /* restart advertising */
//      Adv_Request(APP_BLE_FAST_ADV);
//
//      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */
//#ifdef NFC_PAIRING
//#ifdef USE_RANDOM_ADDR
//    runOOB();
//#endif
//#endif
//      /* USER CODE END EVT_DISCONN_COMPLETE */
//      break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */
//    }
//
//    case HCI_LE_META_EVT_CODE:
//    {
//      p_meta_evt = (evt_le_meta_event*) p_event_pckt->data;
//      /* USER CODE BEGIN EVT_LE_META_EVENT */
//
//      /* USER CODE END EVT_LE_META_EVENT */
//      switch (p_meta_evt->subevent)
//      {
//      case HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_SUBEVT_CODE:
//        /* the Key pair has changed, update OOB data */
//#ifdef NFC_PAIRING
//        runOOB();
//#endif
//        break;
//      case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
////Megha stop red+blue ,start blue only per 5 sec
//      	 if(BLEConnectLED.PatternStatus != START)
//      	Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
//
//#if (CFG_DEBUG_APP_TRACE != 0)
//        p_connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) p_meta_evt->data;
//        APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
//        APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
//                     p_connection_update_complete_event->Conn_Interval*1.25,
//                     p_connection_update_complete_event->Conn_Latency,
//                     p_connection_update_complete_event->Supervision_Timeout*10);
//#endif /* CFG_DEBUG_APP_TRACE != 0 */
//
//        /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */
//
//        /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
//        break;
//      case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
//      {
//        hci_le_connection_complete_event_rp0 * connection_complete_event;
//        //Megha stop red+blue ,start blue only per 5 sec
//                	 if(BLEConnectLED.PatternStatus != START)
//                	Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
//        /**
//         * The connection is done, there is no need anymore to schedule the LP ADV
//         */
//        connection_complete_event = (hci_le_connection_complete_event_rp0 *)p_meta_evt->data;
//        HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);
//
//
//        APP_DBG_MSG("EVT_LE_CONN_COMPLETE for connection handle 0x%x\n",
//                    connection_complete_event->Connection_Handle);
//
//        /* Find index of a connection not in HID_IDLE, HID_CONNECTED_SERVER or HID_CONNECTED_CLIENT state */
//        index = 0;
//        while((index < CFG_MAX_CONNECTION) &&
//            ((BleApplicationContext.Device_Connection_Status[index] == APP_BLE_IDLE) ||
//                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_SERVER) ||
//                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_CLIENT)))
//        {
//          index++;
//        }
//
//        if(index < CFG_MAX_CONNECTION)
//        {
//          APP_DBG_MSG("First index in state %d: %d\n",
//                      BleApplicationContext.Device_Connection_Status[index],
//                      index);
//          if(BleApplicationContext.Device_Connection_Status[index] == APP_BLE_LP_CONNECTING)
//            /* Connection as client */
//            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_CLIENT;
//          else
//            /* Connection as server */
//            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_SERVER;
//          BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = connection_complete_event->Connection_Handle;
//
//        }
//        else
//        {
//          APP_DBG_MSG("No stored connection in state different than HID_IDLE, HID_CONNECTED_CLIENT and HID_CONNECTED_SERVER!\n");
//        }
//      }
//      break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */
////        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
//////Megha stop red+blue ,start blue only per 5 sec
////        	 if(BLEConnectLED.PatternStatus != START)
////        	Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
////
////#if (CFG_DEBUG_APP_TRACE != 0)
////          p_connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) p_meta_evt->data;
////          APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
////          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
////                       p_connection_update_complete_event->Conn_Interval*1.25,
////                       p_connection_update_complete_event->Conn_Latency,
////                       p_connection_update_complete_event->Supervision_Timeout*10);
////#endif /* CFG_DEBUG_APP_TRACE != 0 */
////
////          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */
////          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);
////
////
////                   APP_DBG_MSG("EVT_LE_CONN_COMPLETE for connection handle 0x%x\n",
////                		   p_connection_update_complete_event->Connection_Handle);
////
////                   /* Find index of a connection not in HID_IDLE, HID_CONNECTED_SERVER or HID_CONNECTED_CLIENT state */
////                   index = 0;
////                   while((index < CFG_MAX_CONNECTION) &&
////                       ((BleApplicationContext.Device_Connection_Status[index] == APP_BLE_IDLE) ||
////                           (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_SERVER) ||
////                           (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_CLIENT)))
////                   {
////                     index++;
////                   }
////
////                   if(index < CFG_MAX_CONNECTION)
////                   {
////                     APP_DBG_MSG("First index in state %d: %d\n",
////                                 BleApplicationContext.Device_Connection_Status[index],
////                                 index);
////                     if(BleApplicationContext.Device_Connection_Status[index] == APP_BLE_LP_CONNECTING)
////                       /* Connection as client */
////                       BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_CLIENT;
////                     else
////                       /* Connection as server */
////                       BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_SERVER;
////                     BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = p_connection_update_complete_event->Connection_Handle;
////
////                   }
////                   else
////                   {
////                     APP_DBG_MSG("No stored connection in state different than HID_IDLE, HID_CONNECTED_CLIENT and HID_CONNECTED_SERVER!\n");
////                   }
////          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
////          break;
//
//
//
////        case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
////
////          p_evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)p_meta_evt->data;
////          APP_DBG_MSG("==>> HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE - ");
////          if (p_evt_le_phy_update_complete->Status == 0)
////          {
////            APP_DBG_MSG("status ok \n");
//////            Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
////          }
////          else
////          {
////            APP_DBG_MSG("status nok \n");
////          }
////
////          ret = hci_le_read_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, &Tx_phy, &Rx_phy);
////          if (ret != BLE_STATUS_SUCCESS)
////          {
////            APP_DBG_MSG("==>> hci_le_read_phy : fail\n\r");
////          }
////          else
////          {
////            APP_DBG_MSG("==>> hci_le_read_phy - Success \n");
////
////            if ((Tx_phy == TX_2M) && (Rx_phy == RX_2M))
////            {
////              APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n\r", Tx_phy, Rx_phy);
////            }
////            else
////            {
////              APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n\r", Tx_phy, Rx_phy);
////            }
////          }
////          /* USER CODE BEGIN EVT_LE_PHY_UPDATE_COMPLETE */
////
////          /* USER CODE END EVT_LE_PHY_UPDATE_COMPLETE */
////          break;
//
////        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
////        {
////          p_connection_complete_event = (hci_le_connection_complete_event_rp0 *) p_meta_evt->data;
////          /**
////           * The connection is done, there is no need anymore to schedule the LP ADV
////           */
////
////          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);
////
////          APP_DBG_MSG(">>== HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE - Connection handle: 0x%x\n", p_connection_complete_event->Connection_Handle);
////          APP_DBG_MSG("     - Connection established with Central: @:%02x:%02x:%02x:%02x:%02x:%02x\n",
////                      p_connection_complete_event->Peer_Address[5],
////                      p_connection_complete_event->Peer_Address[4],
////                      p_connection_complete_event->Peer_Address[3],
////                      p_connection_complete_event->Peer_Address[2],
////                      p_connection_complete_event->Peer_Address[1],
////                      p_connection_complete_event->Peer_Address[0]);
////          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
////                      p_connection_complete_event->Conn_Interval*1.25,
////                      p_connection_complete_event->Conn_Latency,
////                      p_connection_complete_event->Supervision_Timeout*10
////                     );
////
////          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
////          {
////            /* Connection as client */
////            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
////          }
////          else
////          {
////            /* Connection as server */
////            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
////          }
////          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = p_connection_complete_event->Connection_Handle;
////          /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */
////
////          /* USER CODE END HCI_EVT_LE_CONN_COMPLETE */
////          break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */
////        }
////        case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
////        {
////          p_enhanced_connection_complete_event = (hci_le_enhanced_connection_complete_event_rp0 *) p_meta_evt->data;
////
////          /**
////           * The connection is done, there is no need anymore to schedule the LP ADV
////           */
////
////          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);
////
////          APP_DBG_MSG(">>== HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE - Connection handle: 0x%x\n", p_enhanced_connection_complete_event->Connection_Handle);
////          APP_DBG_MSG("     - Connection established with Central: @:%02x:%02x:%02x:%02x:%02x:%02x\n",
////                      p_enhanced_connection_complete_event->Peer_Address[5],
////                      p_enhanced_connection_complete_event->Peer_Address[4],
////                      p_enhanced_connection_complete_event->Peer_Address[3],
////                      p_enhanced_connection_complete_event->Peer_Address[2],
////                      p_enhanced_connection_complete_event->Peer_Address[1],
////                      p_enhanced_connection_complete_event->Peer_Address[0]);
////          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
////                      p_enhanced_connection_complete_event->Conn_Interval*1.25,
////                      p_enhanced_connection_complete_event->Conn_Latency,
////                      p_enhanced_connection_complete_event->Supervision_Timeout*10
////                     );
////          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
////          {
////            /* Connection as client */
////            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
////          }
////          else
////          {
////            /* Connection as server */
////            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
////          }
////          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = p_enhanced_connection_complete_event->Connection_Handle;
////          /* USER CODE BEGIN HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
////
////          /* USER CODE END HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
////          break; /* HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
////        }
//
////      case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
////      {
////        hci_le_connection_complete_event_rp0 * connection_complete_event;
////
////        /**
////         * The connection is done, there is no need anymore to schedule the LP ADV
////         */
////        connection_complete_event = (hci_le_connection_complete_event_rp0 *)p_meta_evt->data;
////        HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);
////
////        APP_DBG_MSG("HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE for connection handle 0x%x\n",
////                    connection_complete_event->Connection_Handle);
////
////        /* Find index of a connection not in HID_IDLE, HID_CONNECTED_SERVER or HID_CONNECTED_CLIENT state */
////        index = 0;
////        while((index < CFG_MAX_CONNECTION) &&
////            ((BleApplicationContext.Device_Connection_Status[index] == APP_BLE_IDLE) ||
////                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_SERVER) ||
////                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_CLIENT)))
////        {
////          index++;
////        }
////
////        if(index < CFG_MAX_CONNECTION)
////        {
////          APP_DBG_MSG("First index in state %d: %d\n",
////                      BleApplicationContext.Device_Connection_Status[index],
////                      index);
////          if(BleApplicationContext.Device_Connection_Status[index] == APP_BLE_LP_CONNECTING)
////            /* Connection as client */
////            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_CLIENT;
////          else
////            /* Connection as server */
////            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_SERVER;
////          BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = connection_complete_event->Connection_Handle;
////          //Megha stop red+blue ,start blue only per 5 sec
////          if(BLEConnectLED.PatternStatus != START)
////                  	Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
////        }
////        else
////        {
////          APP_DBG_MSG("No stored connection in state different than HID_IDLE, HID_CONNECTED_CLIENT and HID_CONNECTED_SERVER!\n");
////        }
////      }
////      break; /* HCI_HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */
//
////      case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
////      {
////    	  //Megha stop red+blue ,start blue only per 5 sec
////    	  if(BLEConnectLED.PatternStatus != START)
////    	          	Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
////        hci_le_enhanced_connection_complete_event_rp0 *p_enhanced_connection_complete_event;
////        p_enhanced_connection_complete_event = (hci_le_enhanced_connection_complete_event_rp0 *) p_meta_evt->data;
////
////        /**
////         * The connection is done, there is no need anymore to schedule the LP ADV
////         */
////        HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);
////
////        APP_DBG_MSG(">>== HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE - Connection handle: 0x%x\n", p_enhanced_connection_complete_event->Connection_Handle);
////        APP_DBG_MSG("     - Connection established with Central: @:%02x:%02x:%02x:%02x:%02x:%02x\n",
////                    p_enhanced_connection_complete_event->Peer_Address[5],
////                    p_enhanced_connection_complete_event->Peer_Address[4],
////                    p_enhanced_connection_complete_event->Peer_Address[3],
////                    p_enhanced_connection_complete_event->Peer_Address[2],
////                    p_enhanced_connection_complete_event->Peer_Address[1],
////                    p_enhanced_connection_complete_event->Peer_Address[0]);
////        APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
////                    p_enhanced_connection_complete_event->Conn_Interval*1.25,
////                    p_enhanced_connection_complete_event->Conn_Latency,
////                    p_enhanced_connection_complete_event->Supervision_Timeout*10
////                   );
////         /* Find index of a connection not in HID_IDLE, HID_CONNECTED_SERVER or HID_CONNECTED_CLIENT state */
////        index = 0;
////        while((index < CFG_MAX_CONNECTION) &&
////            ((BleApplicationContext.Device_Connection_Status[index] == APP_BLE_IDLE) ||
////                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_SERVER) ||
////                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_CLIENT)))
////        {
////          index++;
////        }
////
////        if(index < CFG_MAX_CONNECTION)
////        {
////          APP_DBG_MSG("First index in state %d: %d\n",
////                      BleApplicationContext.Device_Connection_Status[index],
////                      index);
////          if(BleApplicationContext.Device_Connection_Status[index] == APP_BLE_LP_CONNECTING)
////            /* Connection as client */
////            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_CLIENT;
////          else
////            /* Connection as server */
////            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_SERVER;
////          BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = p_enhanced_connection_complete_event->Connection_Handle;
////        }
////        else
////        {
////          APP_DBG_MSG("No stored connection in state different than HID_IDLE, HID_CONNECTED_CLIENT and HID_CONNECTED_SERVER!\n");
////        }
////        /* USER CODE BEGIN HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
////
////        /* USER CODE END HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
////        break; /* HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
////      }
//
//
//
//        default:
//          /* USER CODE BEGIN SUBEVENT_DEFAULT */
//
//          /* USER CODE END SUBEVENT_DEFAULT */
//          break;
//      }
//    }
//
//      /* USER CODE BEGIN META_EVT */
//
//      /* USER CODE END META_EVT */
//      break; /* HCI_LE_META_EVT_CODE */
//
//
////    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
////      p_blecore_evt = (evt_blecore_aci*) p_event_pckt->data;
////      /* USER CODE BEGIN EVT_VENDOR */
////
////      /* USER CODE END EVT_VENDOR */
////      switch (p_blecore_evt->ecode)
////      {
////        /* USER CODE BEGIN ecode */
////        aci_gap_pairing_complete_event_rp0 *pairing_complete;
////
////        case ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE:
////          APP_DBG_MSG(">>== ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE \n");
////          break; /* ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE */
////
////        case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:
////          APP_DBG_MSG(">>== ACI_GAP_PASS_KEY_REQ_VSEVT_CODE \n");
////
////          ret = aci_gap_pass_key_resp(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,123456);
////          if (ret != BLE_STATUS_SUCCESS)
////          {
////            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Fail, reason: 0x%x\n", ret);
////          }
////          else
////          {
////            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Success \n");
////          }
////          break; /* ACI_GAP_PASS_KEY_REQ_VSEVT_CODE */
////
////        case ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE:
////          APP_DBG_MSG(">>== ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE\n");
////          break; /* ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE */
////
////        case ACI_GAP_PERIPHERAL_SECURITY_INITIATED_VSEVT_CODE:
////          APP_DBG_MSG("==>> ACI_GAP_PERIPHERAL_SECURITY_INITIATED_VSEVT_CODE \n");
////          break; /* ACI_GAP_PERIPHERAL_SECURITY_INITIATED_VSEVT_CODE */
////
////        case ACI_GAP_BOND_LOST_VSEVT_CODE:
////          APP_DBG_MSG("==>> ACI_GAP_BOND_LOST_VSEVT_CODE \n");
////          ret = aci_gap_allow_rebond(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
////          if (ret != BLE_STATUS_SUCCESS)
////          {
////            APP_DBG_MSG("==>> aci_gap_allow_rebond : Fail, reason: 0x%x\n", ret);
////          }
////          else
////          {
////            APP_DBG_MSG("==>> aci_gap_allow_rebond : Success \n");
////          }
////          break; /* ACI_GAP_BOND_LOST_VSEVT_CODE */
////
////        case ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE:
////          APP_DBG_MSG(">>== ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE \n");
////          break; /* ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE */
////
////        case (ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE):
////          APP_DBG_MSG(">>== ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE\n");
////          break; /* ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE */
////
////        case (ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE):
////          APP_DBG_MSG(">>== ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE\n");
////          APP_DBG_MSG("     - numeric_value = %ld\n",
////                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
////          APP_DBG_MSG("     - Hex_value = %lx\n",
////                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
////          ret = aci_gap_numeric_comparison_value_confirm_yesno(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, YES); /* CONFIRM_YES = 1 */
////          if (ret != BLE_STATUS_SUCCESS)
////          {
////            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Fail, reason: 0x%x\n", ret);
////          }
////          else
////          {
////            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Success \n");
////          }
////          break;
////
////        case (ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE):
////        {
////          pairing_complete = (aci_gap_pairing_complete_event_rp0*)p_blecore_evt->data;
////
////          APP_DBG_MSG(">>== ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE\n");
////          if (pairing_complete->Status == 0)
////          {
////            APP_DBG_MSG("     - Pairing Success\n");
////          }
////          else
////          {
////            APP_DBG_MSG("     - Pairing KO \n     - Status: 0x%x\n     - Reason: 0x%x\n",pairing_complete->Status, pairing_complete->Reason);
////          }
////          APP_DBG_MSG("\n");
////        }
////          break;
////        /* USER CODE END ecode */
////
////        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
////          APP_DBG_MSG(">>== ACI_GAP_PROC_COMPLETE_VSEVT_CODE \r");
////          /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */
////
////          /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
////          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */
////
////        /* USER CODE BEGIN BLUE_EVT */
////
////        /* USER CODE END BLUE_EVT */
////      }
////      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */
//
//      /* USER CODE BEGIN EVENT_PCKT */
//
//      /* USER CODE END EVENT_PCKT */
//
//    default:
//      /* USER CODE BEGIN ECODE_DEFAULT*/
//
//      /* USER CODE END ECODE_DEFAULT*/
//      break;
//  }
//
//  return (SVCCTL_UserEvtFlowEnable);
//}
SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
{
  hci_event_pckt    *p_event_pckt;
  evt_le_meta_event *p_meta_evt;
  evt_blecore_aci   *p_blecore_evt;
  uint8_t           Tx_phy, Rx_phy;
  tBleStatus        ret = BLE_STATUS_INVALID_PARAMS;
  hci_le_connection_complete_event_rp0        *p_connection_complete_event;
  hci_disconnection_complete_event_rp0        *p_disconnection_complete_event;
  hci_le_phy_update_complete_event_rp0        *p_evt_le_phy_update_complete;
  hci_le_enhanced_connection_complete_event_rp0 *p_enhanced_connection_complete_event;
  uint8_t index;

#if (CFG_DEBUG_APP_TRACE != 0)
  hci_le_connection_update_complete_event_rp0 *p_connection_update_complete_event;
#endif /* CFG_DEBUG_APP_TRACE != 0 */

  /* USER CODE BEGIN SVCCTL_App_Notification */

  /* USER CODE END SVCCTL_App_Notification */

  p_event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) p_Pckt)->data;

  switch (p_event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) p_event_pckt->data;

      APP_DBG_MSG("HCI_DISCONNECTION_COMPLETE_EVT_CODE for connection handle 0x%x\n",
                  p_disconnection_complete_event->Connection_Handle);
      /* Find index of the handle deconnected */
      index = 0;
      while((index < CFG_MAX_CONNECTION) &&
          (BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] != p_disconnection_complete_event->Connection_Handle))
      {
        index++;
      }

      if(index < CFG_MAX_CONNECTION)
      {
        APP_DBG_MSG("Index of the handle deconnected: %d\n", index);
        BleApplicationContext.Device_Connection_Status[index] = APP_BLE_IDLE;
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] =
            0xFFFF;
      }
      else
      {
        APP_DBG_MSG("No index found for the handle discconnected !\n");
      }

      /* USER CODE BEGIN EVT_DISCONN_COMPLETE_1 */
      HIDS_App_Notification_evt_t stopHIDNotif = {.HIDS_Evt_Opcode = HIDS_REPORT_NOTIFICATION_DISABLED};
      HIDS_Notification(&stopHIDNotif);
#ifdef USE_RANDOM_ADDR
    runOOB();
#endif
      /* USER CODE END EVT_DISCONN_COMPLETE_1 */

      /* restart advertising */
      Adv_Request(APP_BLE_FAST_ADV);

      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */

      /* USER CODE END EVT_DISCONN_COMPLETE */
      break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */
    }

    case HCI_LE_META_EVT_CODE:
    {
      p_meta_evt = (evt_le_meta_event*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (p_meta_evt->subevent)
      {
      case HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_SUBEVT_CODE:
        /* the Key pair has changed, update OOB data */
        runOOB();
        break;
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
//Megha stop red+blue ,start blue only per 5 sec
        	 if(BLEConnectLED.PatternStatus != START)
        	Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;

#if (CFG_DEBUG_APP_TRACE != 0)
          p_connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) p_meta_evt->data;
          APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                       p_connection_update_complete_event->Conn_Interval*1.25,
                       p_connection_update_complete_event->Conn_Latency,
                       p_connection_update_complete_event->Supervision_Timeout*10);
#endif /* CFG_DEBUG_APP_TRACE != 0 */

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;



        case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:

          p_evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)p_meta_evt->data;
          APP_DBG_MSG("==>> HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE - ");
          if (p_evt_le_phy_update_complete->Status == 0)
          {
            APP_DBG_MSG("status ok \n");
//            Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
          }
          else
          {
            APP_DBG_MSG("status nok \n");
          }

          ret = hci_le_read_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, &Tx_phy, &Rx_phy);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> hci_le_read_phy : fail\n\r");
          }
          else
          {
            APP_DBG_MSG("==>> hci_le_read_phy - Success \n");

            if ((Tx_phy == TX_2M) && (Rx_phy == RX_2M))
            {
              APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n\r", Tx_phy, Rx_phy);
            }
            else
            {
              APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n\r", Tx_phy, Rx_phy);
            }
          }
          /* USER CODE BEGIN EVT_LE_PHY_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_PHY_UPDATE_COMPLETE */
          break;

      case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
      {
        hci_le_connection_complete_event_rp0 * connection_complete_event;

        /**
         * The connection is done, there is no need anymore to schedule the LP ADV
         */
        connection_complete_event = (hci_le_connection_complete_event_rp0 *)p_meta_evt->data;
        HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

        APP_DBG_MSG("HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE for connection handle 0x%x\n",
                    connection_complete_event->Connection_Handle);

        /* Find index of a connection not in HID_IDLE, HID_CONNECTED_SERVER or HID_CONNECTED_CLIENT state */
        index = 0;
        while((index < CFG_MAX_CONNECTION) &&
            ((BleApplicationContext.Device_Connection_Status[index] == APP_BLE_IDLE) ||
                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_SERVER) ||
                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_CLIENT)))
        {
          index++;
        }

        if(index < CFG_MAX_CONNECTION)
        {
          APP_DBG_MSG("First index in state %d: %d\n",
                      BleApplicationContext.Device_Connection_Status[index],
                      index);
          if(BleApplicationContext.Device_Connection_Status[index] == APP_BLE_LP_CONNECTING)
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_CLIENT;
          else
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_SERVER;
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = connection_complete_event->Connection_Handle;
          //Megha stop red+blue ,start blue only per 5 sec
          if(BLEConnectLED.PatternStatus != START)
                  	Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
        }
        else
        {
          APP_DBG_MSG("No stored connection in state different than HID_IDLE, HID_CONNECTED_CLIENT and HID_CONNECTED_SERVER!\n");
        }
      }
      break; /* HCI_HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */

      case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
      {
    	  //Megha stop red+blue ,start blue only per 5 sec
    	  if(BLEConnectLED.PatternStatus != START)
    	          	Ueventsgrp.EventFlag_bits.BleConnectEvent = 1;
        hci_le_enhanced_connection_complete_event_rp0 *p_enhanced_connection_complete_event;
        p_enhanced_connection_complete_event = (hci_le_enhanced_connection_complete_event_rp0 *) p_meta_evt->data;

        /**
         * The connection is done, there is no need anymore to schedule the LP ADV
         */
        HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

        APP_DBG_MSG(">>== HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE - Connection handle: 0x%x\n", p_enhanced_connection_complete_event->Connection_Handle);
        APP_DBG_MSG("     - Connection established with Central: @:%02x:%02x:%02x:%02x:%02x:%02x\n",
                    p_enhanced_connection_complete_event->Peer_Address[5],
                    p_enhanced_connection_complete_event->Peer_Address[4],
                    p_enhanced_connection_complete_event->Peer_Address[3],
                    p_enhanced_connection_complete_event->Peer_Address[2],
                    p_enhanced_connection_complete_event->Peer_Address[1],
                    p_enhanced_connection_complete_event->Peer_Address[0]);
        APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                    p_enhanced_connection_complete_event->Conn_Interval*1.25,
                    p_enhanced_connection_complete_event->Conn_Latency,
                    p_enhanced_connection_complete_event->Supervision_Timeout*10
                   );
         /* Find index of a connection not in HID_IDLE, HID_CONNECTED_SERVER or HID_CONNECTED_CLIENT state */
        index = 0;
        while((index < CFG_MAX_CONNECTION) &&
            ((BleApplicationContext.Device_Connection_Status[index] == APP_BLE_IDLE) ||
                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_SERVER) ||
                (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_CONNECTED_CLIENT)))
        {
          index++;
        }

        if(index < CFG_MAX_CONNECTION)
        {
          APP_DBG_MSG("First index in state %d: %d\n",
                      BleApplicationContext.Device_Connection_Status[index],
                      index);
          if(BleApplicationContext.Device_Connection_Status[index] == APP_BLE_LP_CONNECTING)
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_CLIENT;
          else
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status[index] = APP_BLE_CONNECTED_SERVER;
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = p_enhanced_connection_complete_event->Connection_Handle;
        }
        else
        {
          APP_DBG_MSG("No stored connection in state different than HID_IDLE, HID_CONNECTED_CLIENT and HID_CONNECTED_SERVER!\n");
        }
        /* USER CODE BEGIN HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */

        /* USER CODE END HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
        break; /* HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE */
      }



        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }
    }

      /* USER CODE BEGIN META_EVT */

      /* USER CODE END META_EVT */
      break; /* HCI_LE_META_EVT_CODE */


//    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
//      p_blecore_evt = (evt_blecore_aci*) p_event_pckt->data;
//      /* USER CODE BEGIN EVT_VENDOR */
//
//      /* USER CODE END EVT_VENDOR */
//      switch (p_blecore_evt->ecode)
//      {
//        /* USER CODE BEGIN ecode */
//        aci_gap_pairing_complete_event_rp0 *pairing_complete;
//
//        case ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE:
//          APP_DBG_MSG(">>== ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE \n");
//          break; /* ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE */
//
//        case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:
//          APP_DBG_MSG(">>== ACI_GAP_PASS_KEY_REQ_VSEVT_CODE \n");
//
//          ret = aci_gap_pass_key_resp(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,123456);
//          if (ret != BLE_STATUS_SUCCESS)
//          {
//            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Fail, reason: 0x%x\n", ret);
//          }
//          else
//          {
//            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Success \n");
//          }
//          break; /* ACI_GAP_PASS_KEY_REQ_VSEVT_CODE */
//
//        case ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE:
//          APP_DBG_MSG(">>== ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE\n");
//          break; /* ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE */
//
//        case ACI_GAP_PERIPHERAL_SECURITY_INITIATED_VSEVT_CODE:
//          APP_DBG_MSG("==>> ACI_GAP_PERIPHERAL_SECURITY_INITIATED_VSEVT_CODE \n");
//          break; /* ACI_GAP_PERIPHERAL_SECURITY_INITIATED_VSEVT_CODE */
//
//        case ACI_GAP_BOND_LOST_VSEVT_CODE:
//          APP_DBG_MSG("==>> ACI_GAP_BOND_LOST_VSEVT_CODE \n");
//          ret = aci_gap_allow_rebond(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
//          if (ret != BLE_STATUS_SUCCESS)
//          {
//            APP_DBG_MSG("==>> aci_gap_allow_rebond : Fail, reason: 0x%x\n", ret);
//          }
//          else
//          {
//            APP_DBG_MSG("==>> aci_gap_allow_rebond : Success \n");
//          }
//          break; /* ACI_GAP_BOND_LOST_VSEVT_CODE */
//
//        case ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE:
//          APP_DBG_MSG(">>== ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE \n");
//          break; /* ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE */
//
//        case (ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE):
//          APP_DBG_MSG(">>== ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE\n");
//          break; /* ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE */
//
//        case (ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE):
//          APP_DBG_MSG(">>== ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE\n");
//          APP_DBG_MSG("     - numeric_value = %ld\n",
//                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
//          APP_DBG_MSG("     - Hex_value = %lx\n",
//                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
//          ret = aci_gap_numeric_comparison_value_confirm_yesno(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, YES); /* CONFIRM_YES = 1 */
//          if (ret != BLE_STATUS_SUCCESS)
//          {
//            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Fail, reason: 0x%x\n", ret);
//          }
//          else
//          {
//            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Success \n");
//          }
//          break;
//
//        case (ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE):
//        {
//          pairing_complete = (aci_gap_pairing_complete_event_rp0*)p_blecore_evt->data;
//
//          APP_DBG_MSG(">>== ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE\n");
//          if (pairing_complete->Status == 0)
//          {
//            APP_DBG_MSG("     - Pairing Success\n");
//          }
//          else
//          {
//            APP_DBG_MSG("     - Pairing KO \n     - Status: 0x%x\n     - Reason: 0x%x\n",pairing_complete->Status, pairing_complete->Reason);
//          }
//          APP_DBG_MSG("\n");
//        }
//          break;
//        /* USER CODE END ecode */
//
//        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
//          APP_DBG_MSG(">>== ACI_GAP_PROC_COMPLETE_VSEVT_CODE \r");
//          /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */
//
//          /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
//          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */
//
//        /* USER CODE BEGIN BLUE_EVT */
//
//        /* USER CODE END BLUE_EVT */
//      }
//      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

    default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

//APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
//{
//  return BleApplicationContext.Device_Connection_Status;
//}

/* USER CODE BEGIN FD*/
/*
void APP_BLE_Key_Button1_Action(void)
{

  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  ret = aci_gap_clear_security_db();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("==>> aci_gap_clear_security_db - Fail, result: %d \n", ret);
  }
  else
  {
    APP_DBG_MSG("==>> aci_gap_clear_security_db - Success\n");
  }


}

void APP_BLE_Key_Button2_Action(void)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  
  ret = aci_gap_slave_security_req(BleApplicationContext.BleApplicationContext_legacy.connectionHandle); 

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("==>> aci_gap_slave_security_req() Fail , result: %d \n", ret);
  }
  else
  {
    APP_DBG_MSG("===>> aci_gap_slave_security_req - Success\n");
  }
}
*/
//void APP_BLE_Key_Button3_Action(void)
//{
//  uint8_t TX_PHY, RX_PHY;
//  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
//
//  ret = hci_le_read_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,&TX_PHY,&RX_PHY);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("==>> hci_le_read_phy - Fail \n\r");
//  }
//  else
//  {
//    APP_DBG_MSG("==>> hci_le_read_phy - Success\n");
//    APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n", TX_PHY, RX_PHY);
//    if ((TX_PHY == TX_2M) && (RX_PHY == RX_2M))
//    {
//      APP_DBG_MSG("==>> hci_le_set_phy PHY Param  TX= %d, RX= %d - ", TX_1M, RX_1M);
//      ret = hci_le_set_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,ALL_PHYS_PREFERENCE,TX_1M,RX_1M,0);
//      if (ret != BLE_STATUS_SUCCESS)
//      {
//        APP_DBG_MSG("Fail\n\r");
//      }
//      else
//      {
//        APP_DBG_MSG("Success\n\r");
//      }
//    }
//    else
//    {
//      APP_DBG_MSG("==>> hci_le_set_phy PHY Param  TX= %d, RX= %d - ", TX_2M_PREFERRED, RX_2M_PREFERRED);
//      ret = hci_le_set_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED,0);
//      if (ret != BLE_STATUS_SUCCESS)
//      {
//        APP_DBG_MSG("Fail\n\r");
//      }
//      else
//      {
//        APP_DBG_MSG("Success\n\r");
//      }
//    }
//  }
//}



#if (CFG_LPM_SUPPORTED == 1)

static void APP_BLE_Threadx_LpTimerCb( void )
{
  /**
  * Nothing to be done
  */
  return;
}

/**
 * @brief  Request to start a low power timer
 *
 * @param  tx_low_power_next_expiration: Number of ThreadX ticks
 * @retval None
 */
void APP_BLE_ThreadX_Low_Power_Setup(ULONG tx_low_power_next_expiration)
{
  uint64_t time;
  /* Timer was already created, here we need to start it */
  /* By default, see  tx_initialize_low_level.S, each tick is 10 ms */
  /* This function should be very similar to LpTimerStart used in freertos_port.c */
  /* Converts the number of FreeRTOS ticks into hw timer tick */
  if (tx_low_power_next_expiration > (ULLONG_MAX / 1e12)) /* Prevent overflow in else statement */
  {
    time = 0xFFFF0000; /* Maximum value equal to 24 days */
  }
  else
  {
    /* The result always fits in uint32_t and is always less than 0xFFFF0000 */
    time = tx_low_power_next_expiration * 1000000000000ULL;
    time = (uint64_t)( time /  ( CFG_TS_TICK_VAL_PS * TX_TIMER_TICK_PER_SECOND ));
  }

  HW_TS_Start(LpTXTimerContext.LpTXTimerThreadx_Id, (uint32_t)time);

  /**
   * There might be other timers already running in the timer server that may elapse
   * before this one.
   * Store how long before the next event so that on wakeup, it will be possible to calculate
   * how long the tick has been suppressed
   */
  LpTXTimerContext.LpTXTimeLeftOnEntry = HW_TS_RTC_ReadLeftTicksToCount( );

  return;
}
/**
 * @brief  Read how long the tick has been suppressed
 *
 * @param  None
 * @retval The number of tick rate (FreeRTOS tick)
 */
unsigned long APP_BLE_Threadx_Low_Power_Adjust_Ticks(void)
{
  uint64_t val_ticks, time_ps;
  uint32_t LpTimeLeftOnExit;

  LpTimeLeftOnExit = HW_TS_RTC_ReadLeftTicksToCount();
  /* This cannot overflow. Max result is ~ 1.6e13 */
  time_ps = (uint64_t)((CFG_TS_TICK_VAL_PS) * (uint64_t)(LpTXTimerContext.LpTXTimeLeftOnEntry - LpTimeLeftOnExit));

  /* time_ps can be less than 1 RTOS tick in following situations
   * a) MCU didn't go to STOP2 due to wake-up unrelated to Timer Server or woke up from STOP2 very shortly after.
   *    Advancing RTOS clock by 1 ThreadX tick doesn't hurt in this case.
   * b) APP_BLE_ThreadX_Low_Power_Setup(tx_low_power_next_expiration) was called with xExpectedIdleTime = 2 which is minimum value defined by configEXPECTED_IDLE_TIME_BEFORE_SLEEP.
   *    The xExpectedIdleTime is decremented by one RTOS tick to wake-up in advance.
   *    Ex: RTOS tick is 1ms, the timer Server wakes the MCU in ~977 us. RTOS clock should be advanced by 1 ms.
   * */
  if(time_ps <= (1e12 / TX_TIMER_TICK_PER_SECOND)) /* time_ps < RTOS tick */
  {
    val_ticks = 1;
  }
  else
  {
    /* Convert pS time into OS ticks */
    val_ticks = time_ps * TX_TIMER_TICK_PER_SECOND; /* This cannot overflow. Max result is ~ 1.6e16 */
    val_ticks = (uint64_t)(val_ticks / (1e12)); /* The result always fits in uint32_t */
  }

  /**
   * The system may have been out from another reason than the timer
   * Stop the timer after the elapsed time is calculated other wise, HW_TS_RTC_ReadLeftTicksToCount()
   * may return 0xFFFF ( TIMER LIST EMPTY )
   * It does not hurt stopping a timer that exists but is not running.
   */
  HW_TS_Stop(LpTXTimerContext.LpTXTimerThreadx_Id);

  return (unsigned long)val_ticks;
}
#endif
/* USER CODE END FD*/

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init(void)
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}
/** This function enables the OOB and update the NFC tag content accordingly*/
static void runOOB(void)
{
  uint8_t at = 0;
  uint8_t add[6] = {0,0,0,0,0,0};
  uint8_t len = 0;
  uint8_t rand[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  uint8_t hash[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  ST25DVxxKC_PASSWD_t default_password = {.MsbPasswd=0, .LsbPasswd=0};
  ST25DVxxKC_RF_PROT_ZONE_t rf_write_protect = {.PasswdCtrl = ST25DVXXKC_PROT_PASSWD1 , .RWprotection =ST25DVXXKC_WRITE_PROT};

  NFC07A1_NFCTAG_SetRFDisable_Dyn(0);
  NFC07A1_NFCTAG_PresentI2CPassword(0,default_password);
  NFC07A1_NFCTAG_WriteRFZxSS(0, ST25DVXXKC_PROT_ZONE1,  rf_write_protect );

  /* this dummy call is required to enable OOB */
#ifdef OOB_SECURE
   int status = aci_gap_set_oob_data(0,0,add, 0, 0, rand);
#endif

   Ndef_Bluetooth_OOB_t NdefBle = {

#ifdef OOB_SECURE
                                    .LeOptionalMask = (NDEF_BLUETOOTH_OPTION(BLUETOOTH_EIR_LE_SECURE_CONNECTIONS_CONFIRMATION_VALUE) |
                                                       NDEF_BLUETOOTH_OPTION(BLUETOOTH_EIR_LE_SECURE_CONNECTIONS_RANDOM_VALUE)),
#endif
                                    .OptionalMask = (NDEF_BLUETOOTH_OPTION(BLUETOOTH_EIR_COMPLETE_LOCAL_NAME) |
                                                     NDEF_BLUETOOTH_OPTION(BLUETOOTH_EIR_SECURITY_MANAGER_TK_VALUE)),
                                    .Type = NDEF_BLUETOOTH_BLE,
                                    .Role = NDEF_BLE_ROLE_PERIPH_ONLY,
#ifndef USE_RANDOM_ADDR
                                    .DeviceAddressType = NDEF_BLE_PUBLIC_ADDRESS_TYPE
#else
                                    .DeviceAddressType = NDEF_BLE_RANDOM_ADDRESS_TYPE
#endif
                                  };
    strcpy(NdefBle.LocalName,name);


#ifndef OOB_SECURE
 memcpy(NdefBle.DeviceAddress,a_BdAddrUdn,sizeof(NdefBle.DeviceAddress));
  for(int i = 5; i >= 0; i --)
    NdefBle.DeviceAddress[i] = a_BdAddrUdn[5-i];
#else
   status = aci_gap_get_oob_data(1, &at,add,&len, rand);
    status = aci_gap_get_oob_data(2, &at,add,&len, hash);

  for(int i = 5; i >= 0; i --)
    NdefBle.DeviceAddress[i] = add[5-i];
  for(int i = 15; i >= 0; i --)
  {
    NdefBle.SimplePairingRandomizer[i] = rand[15-i];
    NdefBle.SimplePairingHash[i] = hash[15-i];
  }
#endif

   NDEF_ClearNDEF();
   NDEF_AppendBluetoothOOB(&NdefBle, NULL);

  NFC07A1_NFCTAG_ResetRFDisable_Dyn(0);


}
static void Ble_Hci_Gap_Gatt_Init(void){

  uint8_t role;
  uint8_t index;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *bd_addr;
  uint32_t srd_bd_addr[2];
  uint16_t appearance[1] = {BLE_CFG_GAP_APPEARANCE }; /* Generic Heart Rate Sensor */
  int status = 0;

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  hci_reset();

  /**
   * Write the BD Address
   */

  bd_addr = BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) bd_addr);

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
  srd_bd_addr[1] =  0x0000ED6E;
  srd_bd_addr[0] =  LL_FLASH_GetUDN( );
  aci_hal_write_config_data( CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr );

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)CFG_BLE_IR );

   /**
   * Write Encryption root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)CFG_BLE_ER );

  /**
   * Set TX Power to 0dBm.
   */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /**
   * Initialize GATT interface
   */
  aci_gatt_init();

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif

  if (role > 0)
  {

    aci_gap_init(role, 0,
    		strlen(name),
                 &gap_service_handle, &gap_dev_name_char_handle, &gap_appearance_char_handle);

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n");
  }

  uint8_t preferred[] = {0x20,0x00,0x30,0x00,0x00,0x00,0xF4,0x01};
  uint16_t preferred_conn_handle = 0x000A;
    status = aci_gatt_update_char_value(gap_service_handle,
                                      preferred_conn_handle,
                                      0,
                                      8,//sizeof(preferred),
                                      preferred);


  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION_NOT_REQUIRED;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.OOB_Data_Present = 0;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = 8;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = 16;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = 1;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = 111111;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = 1;
  for (index = 0; index < 16; index++)
  {
    BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.OOB_Data[index] = (uint8_t) index;
  }

// for native pairing -> Fixed_Pin = 111111
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = 0;


  status = 0xFF;

  status = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
#ifdef OOB_SECURE
                                         0x2,
#else
                                         0,
#endif
                                         0,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
#ifndef USE_RANDOM_ADDR
                                         0
#else
                                         1
#endif
  );


#ifdef OOB_SECURE
  uint8_t ALL_EVENTS[8]={0x9F,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
  hci_le_set_event_mask(ALL_EVENTS);
  status = hci_le_read_local_p256_public_key();
#else
  runOOB();
#endif
  /**
   * Initialize whitelist
   */
   if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
   {
     aci_gap_configure_whitelist();
   }

   (void)status;    // Avoid warnings
}
//static void Ble_Hci_Gap_Gatt_Init(void)
//{
//  uint8_t role;
//  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
//  const uint8_t *p_bd_addr;
//
//#if (CFG_BLE_ADDRESS_TYPE != GAP_PUBLIC_ADDR)
//  uint32_t a_srd_bd_addr[2] = {0,0};
//#endif
//  uint16_t a_appearance[1] = {BLE_CFG_GAP_APPEARANCE};
//  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
//  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init*/
//
//  /* USER CODE END Ble_Hci_Gap_Gatt_Init*/
//
//  APP_DBG_MSG("==>> Start Ble_Hci_Gap_Gatt_Init function\n");
//
//  /**
//   * Initialize HCI layer
//   */
//  /*HCI Reset to synchronise BLE Stack*/
//  ret = hci_reset();
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : hci_reset command, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: hci_reset command\n");
//  }
//
//  /**
//   * Write the BD Address
//   */
//  p_bd_addr = BleGetBdAddress();
//  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, (uint8_t*) p_bd_addr);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_PUBADDR_OFFSET, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_PUBADDR_OFFSET\n");
//    APP_DBG_MSG("  Public Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n",p_bd_addr[5],p_bd_addr[4],p_bd_addr[3],p_bd_addr[2],p_bd_addr[1],p_bd_addr[0]);
//  }
//
//#if (CFG_BLE_ADDRESS_TYPE == GAP_PUBLIC_ADDR)
//  /* BLE MAC in ADV Packet */
//  a_ManufData[ sizeof(a_ManufData)-6] = p_bd_addr[5];
//  a_ManufData[ sizeof(a_ManufData)-5] = p_bd_addr[4];
//  a_ManufData[ sizeof(a_ManufData)-4] = p_bd_addr[3];
//  a_ManufData[ sizeof(a_ManufData)-3] = p_bd_addr[2];
//  a_ManufData[ sizeof(a_ManufData)-2] = p_bd_addr[1];
//  a_ManufData[ sizeof(a_ManufData)-1] = p_bd_addr[0];
//#endif /* CFG_BLE_ADDRESS_TYPE == GAP_PUBLIC_ADDR */
//
//  /**
//   * Static random Address
//   * The two upper bits shall be set to 1
//   * The lowest 32bits is read from the UDN to differentiate between devices
//   * The RNG may be used to provide a random number on each power on
//   */
//#if (CFG_IDENTITY_ADDRESS == GAP_STATIC_RANDOM_ADDR)
//#if defined(CFG_STATIC_RANDOM_ADDRESS)
//  a_srd_bd_addr[0] = CFG_STATIC_RANDOM_ADDRESS & 0xFFFFFFFF;
//  a_srd_bd_addr[1] = (uint32_t)((uint64_t)CFG_STATIC_RANDOM_ADDRESS >> 32);
//  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
//#else
//  /* Get RNG semaphore */
//  while(LL_HSEM_1StepLock(HSEM, CFG_HW_RNG_SEMID));
//
//  /* Enable RNG */
//  __HAL_RNG_ENABLE(&hrng);
//
//  /* Enable HSI48 oscillator */
//  LL_RCC_HSI48_Enable();
//  /* Wait until HSI48 is ready */
//  while(! LL_RCC_HSI48_IsReady());
//
//  if (HAL_RNG_GenerateRandomNumber(&hrng, &a_srd_bd_addr[1]) != HAL_OK)
//  {
//    /* Random number generation error */
//    Error_Handler();
//  }
//  if (HAL_RNG_GenerateRandomNumber(&hrng, &a_srd_bd_addr[0]) != HAL_OK)
//  {
//    /* Random number generation error */
//    Error_Handler();
//  }
//  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
//
//  /* Disable HSI48 oscillator */
//  LL_RCC_HSI48_Disable();
//
//  /* Disable RNG */
//  __HAL_RNG_DISABLE(&hrng);
//
//  /* Release RNG semaphore */
//  LL_HSEM_ReleaseLock(HSEM, CFG_HW_RNG_SEMID, 0);
//#endif /* CFG_STATIC_RANDOM_ADDRESS */
//#endif
//
//#if (CFG_BLE_ADDRESS_TYPE == GAP_STATIC_RANDOM_ADDR)
//  /* BLE MAC in ADV Packet */
//  a_ManufData[ sizeof(a_ManufData)-6] = a_srd_bd_addr[1] >> 8 ;
//  a_ManufData[ sizeof(a_ManufData)-5] = a_srd_bd_addr[1];
//  a_ManufData[ sizeof(a_ManufData)-4] = a_srd_bd_addr[0] >> 24;
//  a_ManufData[ sizeof(a_ManufData)-3] = a_srd_bd_addr[0] >> 16;
//  a_ManufData[ sizeof(a_ManufData)-2] = a_srd_bd_addr[0] >> 8;
//  a_ManufData[ sizeof(a_ManufData)-1] = a_srd_bd_addr[0];
//#endif
//
//#if (CFG_BLE_ADDRESS_TYPE != GAP_PUBLIC_ADDR)
//  ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)a_srd_bd_addr);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET\n");
//    APP_DBG_MSG("  Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n", (uint8_t)(a_srd_bd_addr[1] >> 8),
//                                                                               (uint8_t)(a_srd_bd_addr[1]),
//                                                                               (uint8_t)(a_srd_bd_addr[0] >> 24),
//                                                                               (uint8_t)(a_srd_bd_addr[0] >> 16),
//                                                                               (uint8_t)(a_srd_bd_addr[0] >> 8),
//                                                                               (uint8_t)(a_srd_bd_addr[0]));
//  }
//#endif /* CFG_BLE_ADDRESS_TYPE != GAP_PUBLIC_ADDR */
//
//  /**
//   * Write Identity root key used to derive IRK and DHK(Legacy)
//   */
//  ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)a_BLE_CfgIrValue);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET\n");
//  }
//
//  /**
//   * Write Encryption root key used to derive LTK and CSRK
//   */
//  ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)a_BLE_CfgErValue);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET\n");
//  }
//
//  /**
//   * Set TX Power.
//   */
//  ret = aci_hal_set_tx_power_level(1, CFG_TX_POWER);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n");
//  }
//
//  /**
//   * Initialize GATT interface
//   */
//  ret = aci_gatt_init();
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : aci_gatt_init command, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: aci_gatt_init command\n");
//  }
//
//  /**
//   * Initialize GAP interface
//   */
//  role = 0;
//
//#if (BLE_CFG_PERIPHERAL == 1)
//  role |= GAP_PERIPHERAL_ROLE;
//#endif /* BLE_CFG_PERIPHERAL == 1 */
//
//#if (BLE_CFG_CENTRAL == 1)
//  role |= GAP_CENTRAL_ROLE;
//#endif /* BLE_CFG_CENTRAL == 1 */
//
///* USER CODE BEGIN Role_Mngt*/
//
///* USER CODE END Role_Mngt */
//
//  if (role > 0)
//  {
//    const char *name = "HID";// HRSTM";
//    ret = aci_gap_init(role,
//                       CFG_PRIVACY,
//                       APPBLE_GAP_DEVICE_NAME_LENGTH,
//                       &gap_service_handle,
//                       &gap_dev_name_char_handle,
//                       &gap_appearance_char_handle);
//
//    if (ret != BLE_STATUS_SUCCESS)
//    {
//      APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%x \n", ret);
//    }
//    else
//    {
//      APP_DBG_MSG("  Success: aci_gap_init command\n");
//    }
//
//    ret = aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name);
//    if (ret != BLE_STATUS_SUCCESS)
//    {
//      BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
//    }
//    else
//    {
//      BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
//    }
//  }
//
//  ret = aci_gatt_update_char_value(gap_service_handle,
//                                   gap_appearance_char_handle,
//                                   0,
//                                   2,
//                                   (uint8_t *)&a_appearance);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Appearance\n");
//  }
//  else
//  {
//    BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Appearance\n");
//  }
//
//  /**
//   * Initialize Default PHY
//   */
////  ret = hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED);
////  if (ret != BLE_STATUS_SUCCESS)
////  {
////    APP_DBG_MSG("  Fail   : hci_le_set_default_phy command, result: 0x%x \n", ret);
////  }
////  else
////  {
////    APP_DBG_MSG("  Success: hci_le_set_default_phy command\n");
////  }
//
//  /**
//   * Initialize IO capability
//   */
//  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
//  ret = aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n");
//  }
//
//  /**
//   * Initialize authentication
//   */
//  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
//  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
//  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
//  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
//  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
//  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;
//  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init_1*/
//
//  /* USER CODE END Ble_Hci_Gap_Gatt_Init_1*/
//
//  ret = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
//                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
//                                               CFG_SC_SUPPORT,
//                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
//                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
//                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
//                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
//                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
//                                               CFG_IDENTITY_ADDRESS);
//
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    APP_DBG_MSG("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%x \n", ret);
//  }
//  else
//  {
//    APP_DBG_MSG("  Success: aci_gap_set_authentication_requirement command\n");
//  }
//
//  /**
//   * Initialize whitelist
//   */
//  if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
//  {
//    ret = aci_gap_configure_whitelist();
//    if (ret != BLE_STATUS_SUCCESS)
//    {
//      APP_DBG_MSG("  Fail   : aci_gap_configure_whitelist command, result: 0x%x \n", ret);
//    }
//    else
//    {
//      APP_DBG_MSG("  Success: aci_gap_configure_whitelist command\n");
//    }
//  }
//  APP_DBG_MSG("==>> End Ble_Hci_Gap_Gatt_Init function\n\r");
//}
//
static void Adv_Request(APP_BLE_ConnStatus_t New_Status)
{
	  uint8_t index;
	  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
	  uint16_t Min_Inter, Max_Inter;

	  index = 0;
	  if(New_Status == APP_BLE_FAST_ADV)
	  {
	    Min_Inter = AdvIntervalMin;
	    Max_Inter = AdvIntervalMax;

	    /* Find first index of connection in ADVERTISE or IDLE mode */
	    while((index < CFG_MAX_CONNECTION) &&
	        (BleApplicationContext.Device_Connection_Status[index] != APP_BLE_IDLE))
	    {
	      index++;
	    }
	  }
	  else
	  {
	    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
	    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;

	    /* Find first index of connection in Fast ADVERTISE mode */
	    while((index < CFG_MAX_CONNECTION) &&
	        (BleApplicationContext.Device_Connection_Status[index] != APP_BLE_FAST_ADV))
	    {
	      index++;
	    }
	  }

	  if(index < CFG_MAX_CONNECTION)
	  {
	    /**
	     * Stop the timer, it will be restarted for a new shot
	     * It does not hurt if the timer was not running
	     */
	    HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

	    APP_DBG_MSG("First index in %d state: %d\n",
	                BleApplicationContext.Device_Connection_Status[index],
	                index);

	    if((New_Status == APP_BLE_LP_ADV) &&
	        ((BleApplicationContext.Device_Connection_Status[index] == APP_BLE_FAST_ADV) ||
	            (BleApplicationContext.Device_Connection_Status[index] == APP_BLE_LP_ADV)))
	    {
	      /* Connection in ADVERTISE mode have to stop the current advertising */
	      ret = aci_gap_set_non_discoverable();
	      if(ret == BLE_STATUS_SUCCESS)
	      {
	        APP_DBG_MSG("Successfully Stopped Advertising at index: %d\n", index);
	      }
	      else
	      {
	        APP_DBG_MSG("Stop Advertising Failed at index: %d, result: %d \n", index, ret);
	      }
	    }

	    BleApplicationContext.Device_Connection_Status[index] = New_Status;
	    /* Start Fast or Low Power Advertising */
	    ret = aci_gap_set_discoverable(ADV_IND,
	                                   Min_Inter,
	                                   Max_Inter,
	#ifndef USE_RANDOM_ADDR
									   GAP_PUBLIC_ADDR,
	#else
									   GAP_STATIC_RANDOM_ADDR,
	#endif
	                                   NO_WHITE_LIST_USE, /* use white list */
	                                   sizeof(a_LocalName),
	                                   (uint8_t*)&a_LocalName,
	                                   BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen,
	                                   BleApplicationContext.BleApplicationContext_legacy.advtServUUID,
	                                   80,
	                                   100);
	    if(ret == BLE_STATUS_SUCCESS)
	    {
// Megha start red+blue pattern

	    	Ueventsgrp.EventFlag_bits.ReadyToPairEvent = 1;
	      if(New_Status == APP_BLE_FAST_ADV)
	      {
	        APP_DBG_MSG("Successfully Start Fast Advertising at index: %d\n", index);
	      }
	      else
	      {
	        APP_DBG_MSG("Successfully Start Low Power Advertising at index: %d\n", index);
	      }
	    }
	    else
	    {
	      if(New_Status == APP_BLE_FAST_ADV)
	      {
	        APP_DBG_MSG("Start Fast Advertising Failed at index: %d, result: %d \n", index, ret);
	      }
	      else
	      {
	        APP_DBG_MSG("Start Low Power Advertising Failed at index: %d, result: %d \n", index, ret);
	      }
	    }
	    if(New_Status == APP_BLE_FAST_ADV)
	    {
	      /* Fast advertising during FAST_ADV_TIMEOUT */
	      HW_TS_Start(BleApplicationContext.Advertising_mgr_timer_Id, FAST_ADV_TIMEOUT);
	    }
	  }
	  else
	    APP_DBG_MSG("No index in HID_IDLE state !\n");

	  return;
}

const uint8_t* BleGetBdAddress(void)
{
  uint8_t *p_otp_addr;
  const uint8_t *p_bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if (udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

    /**
     * Public Address with the ST company ID
     * bit[47:24] : 24bits (OUI) equal to the company ID
     * bit[23:16] : Device ID.
     * bit[15:0] : The last 16bits from the UDN
     * Note: In order to use the Public Address in a final product, a dedicated
     * 24bits company ID (OUI) shall be bought.
     */
    a_BdAddrUdn[0] = (uint8_t)(udn & 0x000000FF);
    a_BdAddrUdn[1] = (uint8_t)((udn & 0x0000FF00) >> 8);
    a_BdAddrUdn[2] = (uint8_t)device_id;
    a_BdAddrUdn[3] = (uint8_t)(company_id & 0x000000FF);
    a_BdAddrUdn[4] = (uint8_t)((company_id & 0x0000FF00) >> 8);
    a_BdAddrUdn[5] = (uint8_t)((company_id & 0x00FF0000) >> 16);

    p_bd_addr = (const uint8_t *)a_BdAddrUdn;
  }
  else
  {
    p_otp_addr = OTP_Read(0);
    if (p_otp_addr)
    {
      p_bd_addr = ((OTP_ID0_t*)p_otp_addr)->bd_address;
    }
    else
    {
      p_bd_addr = a_MBdAddr;
    }
  }

  return p_bd_addr;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTION */

/* USER CODE END FD_LOCAL_FUNCTION */

/*************************************************************
 *
 *SPECIFIC FUNCTIONS
 *
 *************************************************************/
static void Add_Advertisment_Service_UUID(uint16_t servUUID)
{
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] =
      (uint8_t) (servUUID & 0xFF);
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] =
      (uint8_t) (servUUID >> 8) & 0xFF;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;

  return;
}
//static void Switch_OFF_GPIO(void)
//{
//  /**
//   * The code shall be executed in the background as an aci command may be sent
//   * The background is the only place where the application can make sure a new aci command
//   * is not sent if there is a pending one
//   */
//   tx_semaphore_put(&sem_P2PSProcessSignal);
//
//  return;
//}
static void Adv_Mgr(void)
{
  /**
   * The code shall be executed in the background as an aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
   tx_semaphore_put(&sem_AdvUpdateProcessSignal);

  return;
}

static void thread_AdvUpdateProcess_entry(ULONG argument)
{
  UNUSED(argument);

  for (;;){
            tx_semaphore_get(&sem_AdvUpdateProcessSignal, TX_WAIT_FOREVER);
            Adv_Update();
          }
}
static void thread_Disconnection_entry(ULONG argument)
{
  UNUSED(argument);
  tBleStatus result;
  uint8_t index;
  for (;;){


	  tx_semaphore_get(&sem_HIDUnpairProcessSignal,TX_WAIT_FOREVER);
			  for(index = 0; index < BLE_CFG_HIDS_NUMBER; index++)
			  {
				UNUSED(result);

//				result =  hci_le_remove_device_from_filter_accept_list( CFG_IDENTITY_ADDRESS,a_MBdAddr);
				result = aci_gap_terminate(BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index],
										   HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE);
//		        BleApplicationContext.Device_Connection_Status[index] = APP_BLE_IDLE;
//		        BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] =
//		            0xFFFF;
				result =  aci_gap_remove_bonded_device( CFG_IDENTITY_ADDRESS,a_MBdAddr);
				result = aci_gap_clear_security_db();

				APP_DBG_MSG("Disconnection: result = %d\n", result);

//				APP_DBG_MSG(": result = %d\n", result);
			  }
			  Adv_Request(APP_BLE_FAST_ADV);
			 // tx_thread_sleep(10);
          }
}


static void Adv_Update(void)
{
  Adv_Request(APP_BLE_LP_ADV);

  return;
}

static void thread_HciUserEvtProcess_entry(ULONG argument)
{
  UNUSED(argument);

  for (;;) {
             tx_semaphore_get(&sem_HciUserEvtProcessSignal, TX_WAIT_FOREVER);
             hci_user_evt_proc();
           }
}

/* USER CODE BEGIN FD_SPECIFIC_FUNCTIONS */

/* USER CODE END FD_SPECIFIC_FUNCTIONS */
/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* pdata)
{
  UNUSED(pdata);
  tx_semaphore_put(&sem_HciUserEvtProcessSignal);
  
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UNUSED(flag);
  tx_semaphore_put(&sem_hci);
  
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UNUSED(timeout);
  tx_semaphore_get(&sem_hci, TX_WAIT_FOREVER);
  
  return;
}

static void BLE_UserEvtRx(void *p_Payload)
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *p_param;

  p_param = (tHCI_UserEvtRxParam *)p_Payload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(p_param->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    p_param->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    p_param->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t Status)
{
  switch (Status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      tx_mutex_get(&mtx_hci, TX_WAIT_FOREVER);
      /* USER CODE BEGIN HCI_TL_CmdBusy */

      /* USER CODE END HCI_TL_CmdBusy */
      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      tx_mutex_put(&mtx_hci);
      /* USER CODE BEGIN HCI_TL_CmdAvailable */

      /* USER CODE END HCI_TL_CmdAvailable */
      break;

    default:
      /* USER CODE BEGIN default */

      /* USER CODE END default */
      break;
  }

  return;
}

void SVCCTL_ResumeUserEventFlow(void)
{
  hci_resume_flow();

  return;
}
//void APP_BLE_Key_Button2_Action(void)
//{
////  UTIL_SEQ_SetTask(1<<CFG_TASK_HID_DISC_REQ_ID, CFG_SCH_PRIO_0);
//	tx_semaphore_put(&sem_HIDDisconnectProcessSignal);
//}
/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */
