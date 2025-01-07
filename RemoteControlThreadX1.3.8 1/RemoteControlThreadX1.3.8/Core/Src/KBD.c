
#include"KBD.h"
#include "tx_queue.h"
#include "periodic.h"
#include "main.h"
#include "iwdg.h"
//static void  process_KBD(void);
//extern TX_SEMAPHORE SleepMode_semaphore;

extern TX_SEMAPHORE sem_HIDUnpairProcessSignal;
extern  EventGrp Ueventsgrp;
extern RTC_HandleTypeDef hrtc;
uint8_t KBD_mgr_timer_Id;
int KeyData;
uint8_t BackKeyPressedFg;
uint32_t BackKeyPressCounter;
uint8_t BootmodeKeyPressedFg;
uint32_t BootmodeKeyPressCounter;
const uint32_t LongKeyPressTime = 5000; // 5sec
//const uint16_t KeyDebounceTime =50;    // 30=30msec
uint16_t KeyDebounceTime=0;
const uint8_t ContinueKeyPressdebounceTime =200;
const uint32_t BootModeKeyPressDebounceTime =5000 ;
const uint32_t BootModeReleaseTime = 200;
const uint32_t KeyStateIdleTime =(30 * 1000);
uint8_t  CurrentKeyState;
uint8_t KeyStateIdleFg=0;
//--------------------------------------------------------------------------
TX_QUEUE KBD_Queue;

uint8_t KeySrcSequenceBuffer[MaxKBDMsgSize];// __attribute__((aligned(32)));
uint8_t KeyDestSequenceBuffer[MaxKBDMsgSize];// __attribute__((aligned(32)));
#define KBDQueueSize 20
uint8_t* KBDQueueBuffPtr;
uint8_t* KBD_Queue_SrcPtr;
uint8_t* KBD_Queue_DestPtr;
#define KBDQBUFFSIZE  (KBDQueueSize * MaxKBDMsgSize)
uint8_t KBDQueueBuffer[KBDQBUFFSIZE];// __attribute__((aligned(32)));
//KBDQueuePtr = KBDQueueBuffer;
//---------------------------------------------------------------------------

void KBD_Mgr(void);
void JumpToBootloader(void);

const uint8_t HIDKeypad_NUMPAD_ENTER = 0x58;
const uint8_t HIDKeypad_NUMPAD_LEFT =0x50;
const uint8_t HIDKeypad_NUMPAD_RIGHT =0x4F;
const uint8_t HIDKeypad_NUMPAD_UP =0x52;
const uint8_t HIDKeypad_NUMPAD_DOWN =0x51;
const uint8_t HIDKeypad_BACKSPACE =0x29;       //{0x2A};
const uint8_t HIDKeypad_NBPSTART_F1 =0x3A;     //{0x70};
const uint8_t HIDKeypad_ZEROPRESSURE_F2 =0x3B; //{0x71};
const uint8_t HIDKeypad_ALARMPAUSE_F3 =0x3C;   //{0x72};
const uint8_t HIDKeypad_ALARM_ACK_F24 =0x73;
const uint8_t HIDKeypad_F20 =0x6F;
const uint8_t HIDKeypad_F21 =0x70;



 keyboard_report_t keyboard_report;
 keyboard_report_t* keyboard_report_ptr;
const keymatrix[NRROWS][NRCOLS]={
	{0x70,0x51,0x71},
	{0x4F,0x58,0x50},
	{0x72,0x52,0x2A},
	{0x73,0x00,0x00},
};
Key key_dummy = {
    0,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        // RESERVED_ZERO,
        // RESERVED_ZERO,
        // RESERVED_ZERO,
        // RESERVED_ZERO,

    },
};
Key key_dummy1 = {
    0,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        // RESERVED_ZERO,
        // RESERVED_ZERO,
        // RESERVED_ZERO,
        // RESERVED_ZERO,

    },
};
Key key_NBPSTART = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_NBPSTART_F1,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    } // namespace application
    ,
};
Key key_DOWN = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {

        NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_NUMPAD_DOWN,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },
};
Key key_ZEROPRESSURE = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_ZEROPRESSURE_F2,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },
};
Key key_RIGHT = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_NUMPAD_RIGHT,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },
};
Key key_ENTER = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_NUMPAD_ENTER,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },
};
Key key_LEFT = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_NUMPAD_LEFT,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },
};
Key key_PAUSE = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        NO_MODIFIER, // NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_ALARMPAUSE_F3,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },
};

Key key_UP = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_NUMPAD_UP,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },
};

Key key_BACK = {
    2,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        NO_MODIFIER,
        RESERVED_ZERO,
        HIDKeypad_BACKSPACE,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },
};
Key key_ACK = {
    4,
    RESERVED_ZERO,
    RESERVED_ZERO,
    RESERVED_ZERO,
    {
        0x41, //  Left Ctrl + Right Alt
        RESERVED_ZERO,
        HIDKeypad_F20,
        RESERVED_ZERO,

        0x12, // Right ctrl +Left Shift,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

        0x24, // Left Alt + Right Shift,
        RESERVED_ZERO,
        HIDKeypad_F21,
        RESERVED_ZERO,

        NO_MODIFIER,
        RESERVED_ZERO,
        RESERVED_ZERO,
        RESERVED_ZERO,

    },

};

Keymap keymap = {
    NUM_KEYS,
    {
        &key_dummy,
        &key_ZEROPRESSURE,
#ifdef EP2
		&key_dummy1,
#else
		&key_ACK,
#endif

        &key_UP,
        &key_ENTER,
        &key_LEFT,
        &key_RIGHT,
        &key_BACK,
        &key_DOWN,
        &key_PAUSE,
        &key_NBPSTART,
#ifdef EP2
		&key_ACK,
#endif
    }};
void SetRow(uint8_t RowIdx,GPIO_PinState pinState)
{
	switch(RowIdx)
	{
	case ROW1:
		 HAL_GPIO_WritePin(KB_OUT1_ST_GPIO_Port, KB_OUT1_ST_Pin, pinState);
		 break;
	case ROW2:
		 HAL_GPIO_WritePin(KB_OUT2_ST_GPIO_Port, KB_OUT2_ST_Pin, pinState);
		 break;
	case ROW3:
		 HAL_GPIO_WritePin(KB_OUT3_ST_GPIO_Port, KB_OUT3_ST_Pin, pinState);
		 break;
	case ROW4:
		 HAL_GPIO_WritePin(KB_OUT4_ST_GPIO_Port, KB_OUT4_ST_Pin, pinState);
		 break;
	}

}
GPIO_PinState ReadColumn(uint8_t ColIdx)
{
	GPIO_PinState pinState;

	switch(ColIdx)
	{
	case COLUMN1:
		pinState = HAL_GPIO_ReadPin(KB_IN1_ST_GPIO_Port, KB_IN1_ST_Pin);
		 break;
	case COLUMN2:
		pinState = HAL_GPIO_ReadPin(KB_IN2_ST_GPIO_Port, KB_IN2_ST_Pin);
		 break;
	case COLUMN3:
		pinState = HAL_GPIO_ReadPin(KB_IN3_ST_GPIO_Port, KB_IN3_ST_Pin);
		 break;

	}
}
void InitRows(GPIO_PinState pinState)
{
	HAL_GPIO_WritePin(KB_OUT1_ST_GPIO_Port, KB_OUT1_ST_Pin, pinState);
	HAL_GPIO_WritePin(KB_OUT2_ST_GPIO_Port, KB_OUT2_ST_Pin, pinState);
	HAL_GPIO_WritePin(KB_OUT3_ST_GPIO_Port, KB_OUT3_ST_Pin, pinState);
	HAL_GPIO_WritePin(KB_OUT4_ST_GPIO_Port, KB_OUT4_ST_Pin, pinState);


}
uint8_t ScanKey(void)
{
    uint8_t ColIdx = 0;
    uint8_t RowIdx = 0;
    uint8_t keyCount;
    uint8_t KeyOffset = 0;
    GPIO_PinState CurrentState;
    union
    {
        uint32_t RowInt;
        uint8_t RowArr[4];

    } Row;
    union
    {
        uint32_t ColInt;
        uint8_t ColArr[4];

    } Col;
    Row.RowInt = 0;
    Col.ColInt = 0;
    KeyData = 0;
    keyCount = 0;
    for (RowIdx = 0; RowIdx < NRROWS; RowIdx++)
    {

    	SetRow(RowIdx,GPIO_PIN_RESET);
 //       keypadnew::SetRow(RowIdx, hal::IHalGpioOut::PinState::Reset);

    	Thread_delay(1);
        for (ColIdx = 0; ColIdx < NRCOLS; ColIdx++)
        {

            CurrentState = ReadColumn(ColIdx);
            if (CurrentState == GPIO_PIN_RESET)
            {
                Row.RowArr[keyCount] = RowIdx;
                Col.ColArr[keyCount] = ColIdx;
                keyCount++;
            }
        }
        SetRow(RowIdx,GPIO_PIN_SET);
    }
    if (keyCount == 1)
    {
        //            KeyOffset = (Row.RowArr[0] * NRCOLS_) + Col.ColArr[0] + 1;
        KeyOffset = (Col.ColArr[0] * NRROWS) + Row.RowArr[0] + 1;
        if (KeyOffset >= KEY_BOOTMODE)
            KeyOffset = 0;

        KeyData = KeyOffset;
//        return KeyData;
    }
#ifdef EP2
//check for boot key and enter key pressed
    CurrentState = HAL_GPIO_ReadPin(STM_BOOT_GPIO_Port, STM_BOOT_PIN);
    if(CurrentState == GPIO_PIN_SET)
    {
    	if(keyCount == 0)
    	{
			KeyData = KEY_PAUSE;
			return KeyData;
    	}
    	else if(keyCount==1)
    	{
			KeyData = KEY_BOOTMODE;
			return KeyData;
    	}
    }
#else

    if (keyCount == 2)
    // 0x 0   2   0   0             0x  0  0  1  0    for enter  and back keys saved row and column ids to respective place
    //    R4  R3  R2  R1               C4  C3 C2 C1
    //     0    2   0   0
    //      3   0   0   0               0   0   1   0
    {
        if ((Row.RowInt == 0x300) && (Col.ColInt == 0x02)) // Alm Pause and enter
        {
            KeyData = KEY_BOOTMODE;
        }
    }
#endif

    return KeyData;

}
void LowPower_Enter(void)
{
        // tx_low_power_enter();
        GPIO_InitTypeDef GPIO_InitStructure;
        //    if (modifyIWDG() != HAL_OK)
        //        return;

        /* USER CODE BEGIN  App_ThreadX_LowPower_Enter */

        /* GPIO Ports Clock Enable */
        // __HAL_RCC_GPIOA_CLK_ENABLE();
        //   __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();

        /* Set all GPIO in analog state to reduce power consumption,                */
        /*   except GPIOC to keep user button interrupt enabled                     */
        /* Note: Debug using ST-Link is not possible during the execution of this   */
        /*       example because communication between ST-link and the device       */
        /*       under test is done through UART. All GPIO pins are disabled (set   */
        /*       to analog input mode) including  UART I/O pins.                    */
        GPIO_InitStructure.Pin = GPIO_PIN_All;
        GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStructure.Pull = GPIO_NOPULL;


        HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
        HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
        HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

        /* GPIO Ports Clock Disable */

        __HAL_RCC_GPIOD_CLK_DISABLE();
        __HAL_RCC_GPIOE_CLK_DISABLE();
        __HAL_RCC_GPIOH_CLK_DISABLE();
        HAL_RTC_DeInit(&hrtc);
        /* Enter to the stop mode */
        HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
        /* USER CODE END  App_ThreadX_LowPower_Enter */

        // HAL_RCC_DeInit();
        // SystemClock_Config();

        // MX_GPIO_Init();

        // /* Hold the CPU2 and its allocated peripherals after wakeup from Stop 2 Mode */
        // HAL_PWREx_HoldCore(PWR_CORE_CPU2);
    }
void LowPower_Exit(void)
{
//     HAL_RCC_DeInit();
    HAL_Init();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    SystemClock_Config();
    PeriphCommonClock_Config();
    MX_RTC_Init();
    MX_GPIO_Init();

//    MX_IWDG_Init();
    //          MX_USB_PCD_Init();
    /* Hold the CPU2 and its allocated peripherals after wakeup from Stop 2 Mode */
    HAL_PWREx_HoldCore(PWR_CORE_CPU2);
}
void thread_KeypressEvtProcess_entry(ULONG thread_input)
{

  UNUSED(thread_input);
  static uint8_t KeyCode = 0;

  int tempcntr = 0;

  static int KeyPressFg = 0;
  static unsigned long CurrentTick = 0, PrevTick = 0, KeyPressedTick = 0,KeyTaskTick=0;
  static int CurrentKey = 0, PrevKey = 0;
  int result = 0xff;
  int i;
  TX_QUEUE* KBD_Queue_ptr;
  Key* keyStruct;
//----------------------------------------------
  KBDQueueBuffPtr = KBDQueueBuffer;
  KBD_Queue_ptr = &KBD_Queue;
  KBD_Queue_ptr->tx_queue_id =TX_QUEUE_ID;
  KBD_Queue_ptr->tx_queue_message_size = MaxKBDMsgSize;
  KBD_Queue_ptr->tx_queue_capacity = KBDQBUFFSIZE;
  KBD_Queue_ptr->tx_queue_start=KBDQueueBuffer;
//  KBD_Queue_ptr->tx_queue_write = KeySrcSequenceBuffer;
//  KBD_Queue_ptr->tx_queue_read = KeyDestSequenceBuffer;
  result = tx_queue_create(KBD_Queue_ptr, "KBD_Queue", MaxKBDMsgSize,  KBD_Queue_ptr->tx_queue_start, KBDQBUFFSIZE);
 // feAssert(result == TX_SUCCESS);
  if(result != TX_SUCCESS)
  {
	  Error_Handler();
  }
  KBD_Queue_SrcPtr = KeySrcSequenceBuffer;
  KBD_Queue_DestPtr = KeyDestSequenceBuffer;
 //--------------------------------------------
  HW_TS_Start(KBD_mgr_timer_Id, KBD_SCAN_TIME);
  CurrentKeyState = KEYSTATE_IDLE;
  InitRows(GPIO_PIN_RESET);
  CurrentTick = KeyTaskTick =tx_time_get();
  while (1) {
        tx_semaphore_get(&semaphore_keypress_evt, TX_WAIT_FOREVER);
        HAL_IWDG_Refresh(&hiwdg);
        CurrentTick = tx_time_get();
        switch(CurrentKeyState)
        {
        case KEYSTATE_IDLE:
			if(((CurrentTick - PrevTick) >= ((KeyStateIdleTime) * TX_TIMER_TICKS_PER_SECOND) / 1000) && (KeyStateIdleFg==0))
			{
				KeyStateIdleFg=1;

			}
            CurrentKey = ScanKey();
            if (CurrentKey != 0)
            {
            	if(KeyStateIdleFg==1)
            	{
            		if(CurrentKey==KEY_ENTER)
            		{
                        PrevKey = CurrentKey;
                        // CurrentT1Tick = GPTimer1_.getTimer1Tick();
                        KeyPressedTick = CurrentTick; //= SYSTIME;
                        KeyDebounceTime = IDLESTATE_DEBOUNCE;
                        CurrentKeyState = KEYPRESSED;

            		}
            	}
            	else
            	{
                    PrevKey = CurrentKey;
                    // CurrentT1Tick = GPTimer1_.getTimer1Tick();
                    KeyPressedTick = CurrentTick; //= SYSTIME;
                    KeyDebounceTime =NORMAL_DEBOUNCE;
                    CurrentKeyState = KEYPRESSED;
            	}

            }
        	break;
        case KEYPRESSED:
            if ((CurrentTick - KeyPressedTick) >= ((KeyDebounceTime) * TX_TIMER_TICKS_PER_SECOND) / 1000)
            {
                CurrentKey = ScanKey();
                if (CurrentKey == PrevKey)
                {
                    if(KeyStateIdleFg==1)
                    {
                		if(CurrentKey==KEY_ENTER)
                		{
                			KeyStateIdleFg=0;
                		}
                    }
                    KeyPressedTick = CurrentTick;
                    KeyCode = CurrentKey;
                    CurrentKey = 0;
                    PrevKey = 0;
                    CurrentKeyState = KEYRELEASED;

                    if (KeyCode < KEY_BOOTMODE)
                    {
//                        if (cfgProd::ConfigurationRC::USBConnectedFg == 1)
                        {
                            memcpy(&keyStruct, &keymap.keys[KeyCode], sizeof(Key));
                            KBD_Queue_ptr = &KBD_Queue;
                            KBD_Queue_SrcPtr = KeySrcSequenceBuffer;
                            for (i = 0; i < keyStruct->SequenceSize; i++)
                            {
                            	memset(KeySrcSequenceBuffer,0,sizeof(KeySrcSequenceBuffer));
                                memcpy(&KeySrcSequenceBuffer, &keyStruct->Scancodes[i], sizeof(keyStruct->Scancodes[i]));
                                result = tx_queue_send(KBD_Queue_ptr, KBD_Queue_SrcPtr, 1);
                                if(result != TX_SUCCESS)
                                {
                                	Error_Handler();
                                }
                            }
                        }
                    }
                    if (KeyCode == KEY_BACK)
                    {
                    	BackKeyPressedFg = 1;
                        BackKeyPressCounter = 0;
                    }
                    else if ((KeyCode == KEY_PAUSE )|| // KEY_ENTER
                             (KeyCode == KEY_ENTER) || (KeyCode == KEY_BOOTMODE))
                    {
                        BootmodeKeyPressedFg = 1;
                        BootmodeKeyPressCounter = 0;
                    }

                }
                else
                {
                    CurrentKeyState = KEYSTATE_IDLE;
                }
            }
        	break;
        case KEYRELEASED:
        	CurrentKey = ScanKey();
            if ((CurrentKey == KEY_BACK) && (BackKeyPressedFg == 1))
            {
                BackKeyPressCounter++;

                if ((CurrentTick - KeyPressedTick) >= ((LongKeyPressTime) * TX_TIMER_TICKS_PER_SECOND) / 1000)
                {
                    // set disconnect event
                    //--------------------------------------------------------------------------------
//                    eventsq.SrcPtr = reinterpret_cast<uint8_t*>(&eventsqueue.Src_EventGrp);
//                    eventsq.DestPtr = reinterpret_cast<uint8_t*>(&eventsqueue.Dest_EventGrp);
//                    // memset(&(eventsqueue.EventGrp*)eventsqueue.Src_EventGrp, 0, sizeof(eventsqueue.Src_EventGrp));
//                    eventsqueue.Src_EventGrp.EventFlag_int |= (static_cast<uint32_t>(EventsQueue::EVENTSFLAG::BLEDISCONNECT));
//                    result = eventsq.push(osal::Wait);
//
//                    eventsqueue.Src_EventGrp.EventFlag_int &= !(static_cast<uint32_t>(EventsQueue::EVENTSFLAG::BLEDISCONNECT));
                	//EventFg |= BLEDISCONNECT;
                	if(BLEStatus == CONNECTED)
                	{
						Ueventsgrp.EventFlag_bits.BleUnpairEvent = 1;
						tx_semaphore_put(&sem_HIDUnpairProcessSignal);
                	}
                    //-----------------------------------------------------------------------------------
                    BackKeyPressCounter = 0;
                    BackKeyPressedFg = 0;
                }
            }
            else if (BootmodeKeyPressedFg == 1)
            {
                BootmodeKeyPressCounter++;


                if (CurrentKey == KEY_BOOTMODE)
                {

                    if ((CurrentTick - KeyPressedTick) >= (BootModeKeyPressDebounceTime * TX_TIMER_TICKS_PER_SECOND) / 1000)
                    {

                    	PWM_Start(LED_Red);
                        BootmodeKeyPressCounter = 0;
                        BootmodeKeyPressedFg = 0;
                        JumpToBootloader();
                    }
                }

                else if (BootmodeKeyPressCounter > (BootModeReleaseTime * TX_TIMER_TICKS_PER_SECOND) / 1000)
                {
                    BootmodeKeyPressedFg = 0;
                    BootmodeKeyPressCounter = 0;
                }
                else if (CurrentKey == 0)
                {
                    CurrentKeyState = KEYSTATE_IDLE;
                    BackKeyPressedFg = 0;
                    BootmodeKeyPressedFg = 0;
                    BackKeyPressCounter = 0;
                    BootmodeKeyPressCounter = 0;
                    PrevTick = CurrentTick;
                }
                //                    halTimer2.stop(hal::HalTimer::Buzzer);
            }
            else if ((CurrentKey == KEY_DOWN) || (CurrentKey == KEY_UP) || (CurrentKey == KEY_LEFT) || (CurrentKey == KEY_RIGHT))
            {
                if ((CurrentTick - KeyPressedTick) >= ((ContinueKeyPressdebounceTime) * TX_TIMER_TICKS_PER_SECOND) / 1000)
                {
                    KeyPressedTick = CurrentTick;
                    KeyCode = CurrentKey;
                    if (KeyCode < 11)
                    {
//                        if (cfgProd::ConfigurationRC::USBConnectedFg == 1)
                        {
                            memcpy(&keyStruct, &keymap.keys[KeyCode], sizeof(Key));
                            KBD_Queue_ptr = &KBD_Queue;
                            KBD_Queue_SrcPtr = KeySrcSequenceBuffer;
                            for (i = 0; i < keyStruct->SequenceSize; i++)
                            {
                            	memset(KeySrcSequenceBuffer,0,sizeof(KeySrcSequenceBuffer));
                                memcpy(&KeySrcSequenceBuffer, &keyStruct->Scancodes[i], sizeof(keyStruct->Scancodes[i]));
                                result = tx_queue_send(KBD_Queue_ptr, KBD_Queue_SrcPtr, 1);

                                if(result != TX_SUCCESS)
                                {
                                	Error_Handler();
                                }
                            }
                        }
                    }
                }
            }
        	else if (CurrentKey == 0)
			{
				CurrentKeyState = KEYSTATE_IDLE;
				PrevTick = CurrentTick;
			}
        	break;

        }


//        process_KBD();
    }
}
//void process_KBD(void)
//{
//
//}
void JumpToBootloader(void)
{
    uint32_t i = 0;
    void (*SysMemBootJump)(void);

    /* Set a vector addressed with STM32 Microcontrollers names */
    /* Each vector position contains an address to the boot loader entry point */

    volatile uint32_t BootAddr[33];

    BootAddr[C0] = 0x1FFF0000;
    BootAddr[F030x8] = 0x1FFFEC00;
    BootAddr[F030xC] = 0x1FFFD800;
    BootAddr[F03xx] = 0x1FFFEC00;
    BootAddr[F05] = 0x1FFFEC00;
    BootAddr[F07] = 0x1FFFC800;
    BootAddr[F09] = 0x1FFFD800;
    BootAddr[F10xx] = 0x1FFFF000;
    BootAddr[F105] = 0x1FFFB000;
    BootAddr[F107] = 0x1FFFB000;
    BootAddr[F10XL] = 0x1FFFE000;
    BootAddr[F2] = 0x1FFF0000;
    BootAddr[F3] = 0x1FFFD800;
    BootAddr[F4] = 0x1FFF0000;
    BootAddr[F7] = 0x1FF00000;
    BootAddr[G0] = 0x1FFF0000;
    BootAddr[G4] = 0x1FFF0000;
    BootAddr[H503] = 0x0BF87000;
    BootAddr[H563] = 0x0BF97000;
    BootAddr[H573] = 0x0BF97000;
    BootAddr[H7x] = 0x1FF09800;
    BootAddr[H7A] = 0x1FF0A800;
    BootAddr[H7B] = 0x1FF0A000;
    BootAddr[L0] = 0x1FF00000;
    BootAddr[L1] = 0x1FF00000;
    BootAddr[L4] = 0x1FFF0000;
    BootAddr[L5] = 0x0BF90000;
    BootAddr[WBA] = 0x0BF88000;
    BootAddr[WBX] = 0x1FFF0000;
    BootAddr[WL] = 0x1FFF0000;
    BootAddr[U5] = 0x0BF90000;

    /* Disable all interrupts */
    __disable_irq();

    /* Disable Systick timer */
    SysTick->CTRL = 0;

    /* Set the clock to the default state */
    HAL_RCC_DeInit();

    /* Clear Interrupt Enable Register & Interrupt Pending Register */
    for (i = 0; i < 5; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* Re-enable all interrupts */
    __enable_irq();

    /* Set up the jump to boot loader address + 4 */
    SysMemBootJump = (void (*)(void))(*((uint32_t*)((BootAddr[MCU] + 4))));

    /* Set the main stack pointer to the boot loader stack */
    __set_MSP(*(uint32_t*)BootAddr[MCU]);

    /* Call the function to jump to boot loader location */
    SysMemBootJump();

    /* Jump is done successfully */
    while (1)
    {
        /* Code should never reach this loop */
    }
}
void Thread_delay(unsigned long durationInMs)
{
    unsigned long initial_time = tx_time_get();
    while ((tx_time_get() - initial_time) < durationInMs)
        ;
}
void KBD_Mgr(void)
{
	tx_semaphore_put(&semaphore_keypress_evt);
}

