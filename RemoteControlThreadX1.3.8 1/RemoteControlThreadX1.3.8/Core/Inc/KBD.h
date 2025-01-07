#include "app_common.h"
#include "main.h"
#include "app_conf.h"

#define NORMAL_DEBOUNCE	50
#define IDLESTATE_DEBOUNCE	1000
#define NRROWS			4
#define NRCOLS			3
#ifdef EP2
	#define NUM_KEYS		11
#else
	#define NUM_KEYS		10
#endif
#define RESERVED_ZERO 	0
#define NO_MODIFIER 	0
#define KBD_SCAN_TIME               (50*1000/CFG_TS_TICK_VAL)//50 msec

#define MaxKBDMsgSize  2 //8  bytes

#define MCU WBX
enum
{
    C0,
    F030x8,
    F030xC,
    F03xx,
    F05,
    F07,
    F09,
    F10xx,
    F105,
    F107,
    F10XL,
    F2,
    F3,
    F4,
    F7,
    G0,
    G4,
    H503,
    H563,
    H573,
    H7x,
    H7A,
    H7B,
    L0,
    L1,
    L4,
    L5,
    WBA,
    WBX,
    WL,
    U5
};



        enum  ROWPINS
        {
            ROW1 = 0,
            ROW2,
            ROW3,
            ROW4
        };
        enum  COLUMNPINS
        {
            COLUMN1 = 0,
            COLUMN2,
            COLUMN3
        };
        enum  KEYMAPPING
        {
            KEY_DUMMY = 0,
            KEY_ZEROPRESSURE,

#ifdef EP2
			KEY_DUMMY1,
#else
            KEY_ACK,
#endif
            KEY_UP,
            KEY_ENTER,
            KEY_LEFT,
            KEY_RIGHT,
            KEY_BACK,
            KEY_DOWN,
            KEY_PAUSE,
            KEY_NBPSTART,
#ifdef EP2
			KEY_ACK,
			KEY_BOOTMODE,
#else
			KEY_BOOTMODE,

#endif


        };
        enum  KEY_EVENTS
        {
            KEY_IDLE_EVENT = 0,
            KEYPRESS_EVENT,
            KEY_FORCE_RELEASE_EVENT

        };
        enum  KEYSTATE
        {
            KEYSTATE_IDLE = 0,
            KEYPRESSED,
            KEYRELEASED,
            DOUBLEKEYPRESSED,
            KEYFORCERELEASE
        };
        typedef struct
        {
            uint8_t modifier;
            uint8_t reserved;
            uint8_t keycode1;
            uint8_t keycode2;
        }Scancode;
        typedef struct
        {
            uint8_t SequenceSize;
            uint8_t reserved1;
            uint8_t reserved2;
            uint8_t reserved3;
            Scancode Scancodes[4];
        }Key;
        typedef struct
        {
            uint16_t size;
            Key* keys[NUM_KEYS + 1];
        } Keymap;

        typedef struct
        {
         uint8_t reportID;
          uint8_t modifier;
          int8_t OEM;
          int8_t KEY1;
          int8_t KEY2;
          int8_t KEY3;
          int8_t KEY4;
          int8_t KEY5;
          int8_t KEY6;
        } keyboard_report_t;

extern TX_SEMAPHORE semaphore_keypress_evt;

extern void thread_KeypressEvtProcess_entry(ULONG thread_input);
