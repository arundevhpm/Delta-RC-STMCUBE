/*
 * Periodic.h
 *
 *  Created on: Sep 10, 2024
 *      Author: 320198856
 */
#include "app_common.h"
#ifndef INC_PERIODIC_H_
#define INC_PERIODIC_H_

#define LED_Red		0x00000000
#define LED_Green 	0x00000004
#define LED_Blue 	0x00000008
#define Buzzer 		0x0000000C
#define LED_Yellow	1 	//Red+Green
#define LED_Cyan	2	//Green+Blue
#define LED_Purple	3 	//Red+Blue


#define PERIODIC_SCAN_TIME			(10*1000/CFG_TS_TICK_VAL)//10 msec
        enum  LEDPATTERN
        {
            STOP = 0,
            START,
        };
        typedef struct
        {
            uint16_t BlinkTime;
            uint8_t BlinkPulses;
            uint8_t PulseStatus;
            uint8_t PatternStatus;
            uint8_t PulseCntr;
            uint8_t PulseStartFg;
            uint8_t Dummy;
        } PulsePattern;

        enum  EVENTSFLAG
        {
            BLEDISCONNECT 		= 0x00,
            BUZZERBEEPEVENT		,
            CRITIALBATTEVENT,
            LOWBATTEVENT,
			READYTOPAIREVENT,
			BLECONNECTEVENT,
			BLEUNPAIREVENT,
        };
        enum  BLESTATUS
        {
        	READYTOPAIR=0,
			CONNECTED ,

        };
        typedef union
        {
            struct
            {
                uint32_t BLEDisconnectEvent : 1;
                uint32_t BuzzBeepEvent : 1;
                uint32_t CriticalBattEvent : 1;
                uint32_t LowBattEvent : 1;
                uint32_t ReadyToPairEvent : 1;
                uint32_t BleConnectEvent : 1;
                uint32_t BleUnpairEvent : 1;
                uint32_t bit8 : 1;
                uint8_t Dummy_char;
                uint16_t Dummy_int;

            } EventFlag_bits;
            uint32_t EventFlag_int;
        } EventGrp;

extern void PWM_Start(uint32_t Channel);
extern void PWM_Stop(uint32_t Channel);
extern uint8_t BLEStatus;
#endif /* INC_PERIODIC_H_ */
