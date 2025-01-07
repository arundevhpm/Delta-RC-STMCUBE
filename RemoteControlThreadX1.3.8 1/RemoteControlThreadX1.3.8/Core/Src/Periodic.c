/*
 * Periodic.c
 *
 *  Created on: Sep 10, 2024
 *      Author: 320198856
 */
#include <stdint.h>
#include "Periodic.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "dbg_trace.h"
#include "iwdg.h"
extern TX_SEMAPHORE semaphore_Periodic_evt;
extern ADC_HandleTypeDef hadc1;
extern  TX_SEMAPHORE sem_BASProcessSignal;

uint8_t Periodic_mgr_timer_Id;;
const uint8_t BLEDisconnectBlinkPulses =3;
const uint8_t BLEUnpairBlinkPulses =3;
const uint8_t BuzzerBeepPulses=15;
const uint8_t LowBattPulses=1;
uint16_t BattData_;
const uint16_t OneSecondPeriod=1000/10;
const uint16_t TenMsPeriod=10/10;       // 1000 =1sec
uint16_t BatteryReadPeriod= 60; // 5 sec
const uint16_t BuzzerOnPeriod=60;
const uint16_t LowBattOnPeriod=60;




// 60*3 sec
const uint16_t BLEDisconnectBlinkTimeCntr=50; // for counting in 10 msec
const uint16_t BLEUnpairBlinkTimeCntr=50;
const uint16_t BuzzBeepPulseTimeCntr=100;//1 Pulse of 1 sec
const uint16_t LowBattPulseTimeCntr=50;
const uint16_t LowBattTimeCntr=3000;
const uint16_t ReadyToPairPulseTimeCntr = 100;//toggle every 1 sec
const uint16_t BLEConnectPulseOFFTimeCntr = 500;//pulse off for 5 sec
const uint16_t BLEConnectPulseONTimeCntr = 50;//pulkse ON for 500 msec
const uint16_t BuzzerContinueTimeCntr=3000;//30 sec
//const uint8_t LowBattContinueTimeCntr=3000;//continue low battery pulse after 30 sec
uint8_t LowBattFg;
uint8_t CriticalBattFg;
uint32_t EventFg = 0;
uint32_t BattData;
uint8_t BattDataAvailablefg;
EventGrp Ueventsgrp;
PulsePattern BLEUnpairLED;
PulsePattern BuzzerBeep;
PulsePattern LowBatt;
PulsePattern ReadyToPair;
PulsePattern BLEConnectLED;
PulsePattern BLEDisconnectLED;
uint8_t rssi_val=0;
uint8_t rssi_val_arr[4];
uint8_t BattValInPercentage;
void PWM_Start(uint32_t Channel);
uint8_t BLEStatus=0;
uint16_t BuzzerTimeCntr=0;

uint8_t LedOnFg=0;
void thread_PeriodicEvtProcess_entry(ULONG thread_input)
{

  UNUSED(thread_input);
	GPIO_PinState PinState;
  static int CurrentTick = 0, PrevTick = 0;
  uint8_t PowerOnFg = 0;
  //static int Test = 0;
  int result = 0xff;
  //int onetimebatteryreadfg ;
  int /*battreadcount = 0, singlebatteryconnectedcount = 0,*/ LowBatteryCount = 0;

  int OneSecondCounter = 0;
  int Tenmseccntr = 0;
  int Onemseccntr=0;
  float Vref = 2500.00;
  uint32_t ADCResolution = 4095; // 12 bit
  float ActualBatteryVtg = 0;
  float stepSize = Vref / ADCResolution;
  ADC_ChannelConfTypeDef sConfig = {0};
  HAL_StatusTypeDef halResult;
  HW_TS_Start(Periodic_mgr_timer_Id, PERIODIC_SCAN_TIME);
//  HAL_GPIO_WritePin(ADC_VREF_ENABLE_GPIO_Port,ADC_VREF_ENABLE_Pin,GPIO_PIN_SET);
//  PWM_Start(LED_Green);
  CurrentTick = PrevTick = tx_time_get();
//  if (PowerSource == BATTERY_POWER)
//  {
//	  BatteryReadPeriod =10;
//  }
//  else if(PowerSource == USB_POWER)
//  {
//	  BatteryReadPeriod =1;
////	  onetimebatteryreadfg=1;
//  }
//
//  while(1)
//  {
//	  tx_semaphore_get(&semaphore_Periodic_evt, TX_WAIT_FOREVER);
//	  Onemseccntr++;
//
//      if ((Onemseccntr % OneSecondPeriod) == 0)
//      {
//    	  if(PowerOnFg==0)
//    	  {
//    		  PowerOnFg=1;
//    		  PWM_Stop( LED_Red);
//    	  }
//    	  else
//    	  {
//    		  PowerOnFg=0;
//    		  PWM_Start( LED_Red);
//    	  }
//      }
//  }
  while(1)
  {
	  tx_semaphore_get(&semaphore_Periodic_evt, TX_WAIT_FOREVER);
	  HAL_IWDG_Refresh(&hiwdg);
//	  CurrentTick = tx_time_get();
//      if ((CurrentTick - PrevTick) >= ((TenMsPeriod*10) * TX_TIMER_TICKS_PER_SECOND) / 1000)
//      {
    	 Onemseccntr++;
	  if((Ueventsgrp.EventFlag_int != 0))//&&(PowerOnFg==1))
	  {
		  if((Ueventsgrp.EventFlag_int & (0x0001 << BLEUNPAIREVENT)) != 0)
		  {
			  if (BLEUnpairLED.PatternStatus != START)
			  {
				  Ueventsgrp.EventFlag_bits.BleUnpairEvent = 0;
				  BLEUnpairLED.BlinkPulses =(BLEUnpairBlinkPulses * 2) - 1;
				  BLEUnpairLED.PulseCntr = BLEUnpairLED.BlinkPulses;
				  BLEUnpairLED.BlinkTime = BLEUnpairBlinkTimeCntr ; // 1000= in one sec number of pulses
				  BLEUnpairLED.PatternStatus = START;
				  BLEUnpairLED.PulseStartFg = 1;
				  PWM_Start(LED_Blue);
//                  PWM_Start(Buzzer);
			  }

		  }

		  if((Ueventsgrp.EventFlag_int & (0x0001 << BLEDISCONNECT)) != 0)
		  {
              if (BLEDisconnectLED.PatternStatus != START)
              {
				  Ueventsgrp.EventFlag_bits.BLEDisconnectEvent = 0;
				  PWM_Start(LED_Yellow);
				  PWM_Stop(LED_Blue);//LED_Blue);
				  BLEDisconnectLED.PatternStatus =START;
				  BLEDisconnectLED.BlinkTime = BLEConnectPulseONTimeCntr ;
				  BLEDisconnectLED.PulseStartFg = 1;
//                  PWM_Start(Buzzer);
              }
		  }
		  if ((Ueventsgrp.EventFlag_int & (0x0001 << LOWBATTEVENT)) != 0 )
		  {
			  if (LowBatt.PatternStatus != START)
			  {
				  Ueventsgrp.EventFlag_bits.LowBattEvent = 0;
				  PWM_Start(LED_Red);
				  // PWMT2.start(hal::HalTimer::Buzzer);
				  LowBatt.BlinkPulses = (LowBattPulses * 2) - 1;
				  LowBatt.PulseCntr = LowBatt.BlinkPulses;
				  LowBatt.BlinkTime = LowBattPulseTimeCntr ; // (3000 / periodicapplication.BuzzerBeep.BlinkPulses); // 1000= in one sec number of pulses
				  LowBatt.PatternStatus = START;
				  LowBatt.PulseStartFg = 1;
				  BLEConnectLED.PatternStatus = STOP;
			  }
		  }
		   if((Ueventsgrp.EventFlag_int & (0x0001 << BUZZERBEEPEVENT)) != 0)
		  {
			   Ueventsgrp.EventFlag_bits.BuzzBeepEvent = 0;
			  if (BuzzerBeep.PatternStatus != START)
			  {

				  PWM_Start(Buzzer);
				  // PWMT2.start(hal::HalTimer::Buzzer);
				  BuzzerBeep.BlinkPulses = (BuzzerBeepPulses * 2) - 1;
				  BuzzerBeep.PulseCntr = BuzzerBeep.BlinkPulses;
				  BuzzerBeep.BlinkTime = BuzzBeepPulseTimeCntr ; // (3000 / periodicapplication.BuzzerBeep.BlinkPulses); // 1000= in one sec number of pulses
				  BuzzerBeep.PatternStatus = START;
				  BuzzerBeep.PulseStartFg = 1;
			  }
			  else
			  {
				  BuzzerBeep.PatternStatus = STOP;
				  BuzzerTimeCntr=0;
				  PWM_Stop(Buzzer);
			  }

		  }

		   if((Ueventsgrp.EventFlag_int & (0x0001 << READYTOPAIREVENT) ) != 0)
		  {
			  if (ReadyToPair.PatternStatus != START)
			  {
				  Ueventsgrp.EventFlag_bits.ReadyToPairEvent = 0;
				  PWM_Start(LED_Blue);
				  PWM_Stop( LED_Red);
				  // PWMT2.start(hal::HalTimer::Buzzer);
//				  ReadyToPair.BlinkPulses = (BuzzerBeepPulses * 2) - 1;
				  ReadyToPair.PulseCntr = ReadyToPair.BlinkPulses;
				  ReadyToPair.BlinkTime = ReadyToPairPulseTimeCntr ; // (3000 / periodicapplication.BuzzerBeep.BlinkPulses); // 1000= in one sec number of pulses
				  ReadyToPair.PatternStatus = START;
				  ReadyToPair.PulseStartFg = 1;
				  BLEConnectLED.PatternStatus = STOP; //stop BLE connect LED pattern
				  BLEConnectLED.PulseStartFg = 0;
				  BLEStatus = READYTOPAIR;
			  }

		  }
		   if((Ueventsgrp.EventFlag_int & (0x0001 << BLECONNECTEVENT)) != 0)
		  {

				  if (BLEConnectLED.PatternStatus != START)
				  {
					  PWM_Stop(LED_Yellow);
					  PWM_Stop( LED_Red);
					  Ueventsgrp.EventFlag_bits.BleConnectEvent = 0;
					  //PWM_Stop(LED_Red);

					  PWM_Start(LED_Blue);//LED_Blue);
					  BLEConnectLED.PatternStatus =START;
					  BLEConnectLED.BlinkTime = BLEConnectPulseONTimeCntr ;
					  BLEConnectLED.PulseStartFg = 1;
					  ReadyToPair.PatternStatus=STOP;  //stop ready to pair blinking pattern and start BLE connection blinking pattern
					  ReadyToPair.PulseStartFg = 0;
					  BLEStatus = CONNECTED;

				  }

		  }


	  }
	  if((Onemseccntr % TenMsPeriod)==0 )
	  {
		  Tenmseccntr++;
          if (BLEDisconnectLED.PatternStatus == START)
          {
        	  BLEDisconnectLED.BlinkTime--;

              if (BLEDisconnectLED.BlinkTime == 0)
              {
            	   //(3000 / periodicapplication.BuzzerBeep.BlinkPulses);
            	  BLEDisconnectLED.PulseCntr--;
                  // periodicapplication.BuzzerBeep.BlinkPulses--;
                  if (BLEDisconnectLED.PulseStartFg == 1)
                  {
                	  BLEDisconnectLED.BlinkTime = BLEConnectPulseOFFTimeCntr;//for 1 sec
                	  BLEDisconnectLED.PulseStartFg = 0;

                	  PWM_Stop(LED_Yellow);
                  }
                  else
                  {
                	  BLEDisconnectLED.BlinkTime = BLEConnectPulseONTimeCntr;
                	  BLEDisconnectLED.PulseStartFg = 1;

                      PWM_Start(LED_Yellow);//LED_Blue);
                  }

//                     Test++;
              }
          }
          if (BLEUnpairLED.PatternStatus == START)
          {
              if (BLEUnpairLED.PulseCntr > 0)
              {
            	  BLEUnpairLED.BlinkTime--;

                  if (BLEUnpairLED.BlinkTime == 0)
                  {
                	  BLEUnpairLED.BlinkTime = BLEUnpairBlinkTimeCntr ;

                      if (BLEUnpairLED.PulseStartFg == 1)
                      {
                    	  BLEUnpairLED.PulseStartFg = 0;
                          PWM_Stop( LED_Blue);
                      }
                      else
                      {
                    	  BLEUnpairLED.PulseStartFg = 1;
                          PWM_Start(LED_Blue);

                      }
                      BLEUnpairLED.PulseCntr--;
                  }
              }
              else
              {
            	  BLEUnpairLED.PulseStartFg = 0;
            	  BLEUnpairLED.PatternStatus = STOP;
              }
          }
          if (LowBatt.PatternStatus == START)
          {
//              if (LowBatt.PulseCntr > 0)
              {
                  LowBatt.BlinkTime--;

                  if (LowBatt.BlinkTime == 0)
                  {

//                      LowBatt.PulseCntr--;
                      if (LowBatt.PulseStartFg == 1)
                      {
                    	  LowBatt.BlinkTime = LowBattTimeCntr ; //30 sec time counter                         LowBatt.PulseStartFg = 0;

                    	  PWM_Stop( LED_Red);
                          LowBatt.PulseStartFg=0;
                      }
                      else
                      {
                    	  LowBatt.BlinkTime = LowBattPulseTimeCntr ; //(3000 / periodicapplication.BuzzerBeep.BlinkPulses);
//                    	  LowBatt.PulseCntr = LowBatt.BlinkPulses;
                    	  LowBatt.PulseStartFg = 1;
                          PWM_Start(LED_Red);
                      }

 //                     Test++;
                  }
              }
//              else
//              {
//                  LowBatt.PatternStatus = STOP;
//                  LowBatt.PulseStartFg = 0;
//              }
          }
          if (BuzzerBeep.PatternStatus == START)
          {
        	  BuzzerTimeCntr++;
              if (BuzzerBeep.PulseCntr > 0)
              {
            	  BuzzerBeep.BlinkTime--;

                  if (BuzzerBeep.BlinkTime == 0)
                  {
                	  BuzzerBeep.BlinkTime = BuzzBeepPulseTimeCntr;//(3000 / periodicapplication.BuzzerBeep.BlinkPulses);
                	  BuzzerBeep.PulseCntr--;
                      // periodicapplication.BuzzerBeep.BlinkPulses--;
                      if (BuzzerBeep.PulseStartFg == 1)
                      {


                    	  PWM_Stop( Buzzer);
                    	  BuzzerBeep.PulseStartFg = 0;


                      }
                      else
                      {

                    	  PWM_Start(Buzzer);
                    	  BuzzerBeep.PulseStartFg = 1;

                      }

 //                     Test++;
                  }
              }
              else
              {
//            	  BuzzerBeep.PatternStatus = STOP;
            	  BuzzerBeep.BlinkTime = 300;//BuzzBeepPulseTimeCntr
            	  BuzzerBeep.PulseStartFg = 0;
              }
              if(BuzzerTimeCntr >=BuzzerContinueTimeCntr )
              {
            	  BuzzerBeep.PatternStatus = STOP;
            	  BuzzerTimeCntr=0;
              }
          }
//-dont copy the below case,it is different
          if (ReadyToPair.PatternStatus == START)
          {
              //if (ReadyToPair.PulseCntr > 0)
              {
            	  ReadyToPair.BlinkTime--;

                  if (ReadyToPair.BlinkTime == 0)
                  {
                	   //(3000 / periodicapplication.BuzzerBeep.BlinkPulses);
                	  ReadyToPair.PulseCntr--;
                      // periodicapplication.BuzzerBeep.BlinkPulses--;
                      if (ReadyToPair.PulseStartFg == 1)
                      {
                    	  ReadyToPair.BlinkTime = ReadyToPairPulseTimeCntr;// OFF for 1 sec
                    	  ReadyToPair.PulseStartFg = 0;

                    	  PWM_Stop( LED_Blue);
                          PWM_Start(LED_Red);
                      }
                      else
                      {
                    	  ReadyToPair.BlinkTime = ReadyToPairPulseTimeCntr;//ON for 200 msec
                    	  ReadyToPair.PulseStartFg = 1;
                          PWM_Start(LED_Blue);
                          PWM_Stop(LED_Red);
                      }

 //                     Test++;
                  }
              }
//              else
//              {
//            	  ReadyToPair.PatternStatus = STOP;
//            	  ReadyToPair.PulseStartFg = 0;
//              }
          }
			else
			{
			  ReadyToPair.PatternStatus = STOP;
			  ReadyToPair.PulseStartFg = 0;
			}
          if (BLEConnectLED.PatternStatus == START)
          {
              //if (ReadyToPair.PatternStatus == STOP)
              {
            	  BLEConnectLED.BlinkTime--;

                  if (BLEConnectLED.BlinkTime == 0)
                  {
                	   //(3000 / periodicapplication.BuzzerBeep.BlinkPulses);
                	  BLEConnectLED.PulseCntr--;
                      // periodicapplication.BuzzerBeep.BlinkPulses--;
                      if (BLEConnectLED.PulseStartFg == 1)
                      {
                    	  BLEConnectLED.BlinkTime = BLEConnectPulseOFFTimeCntr;//for 1 sec
                    	  BLEConnectLED.PulseStartFg = 0;

                    	  PWM_Stop(LED_Blue);
                      }
                      else
                      {
                    	  BLEConnectLED.BlinkTime = BLEConnectPulseONTimeCntr;
                    	  BLEConnectLED.PulseStartFg = 1;

                          PWM_Start(LED_Blue);//LED_Blue);
                      }

 //                     Test++;
                  }
              }

          }
	  }

      if ((Onemseccntr % OneSecondPeriod) == 0)
      {
    	  OneSecondCounter++;
//    	  if(PowerOnFg==0)
//    	  {
//    		  PowerOnFg=1;
//    		  PWM_Stop( LED_Green);
//    	  }
    		if( PowerSource == BATTERY_POWER)
    		{
    			  PinState = HAL_GPIO_ReadPin(ST_STATUS_POWER_GPIO_Port,ST_STATUS_POWER_Pin);
    			  if(PinState ==GPIO_PIN_SET)
    			  {
    					PWM_Start(LED_Red);

    					while(1){
    						  HAL_IWDG_Refresh(&hiwdg);
    						  HAL_Delay(1000);
    					};
    			  }
    		}
          if(BLEStatus == CONNECTED)
          {
          	aci_hal_read_rssi(&rssi_val);//to read rssi value
 //         	aci_hal_read_raw_rssi(&rssi_val_arr);
          	APP_DBG_MSG("RSSI value = %d\n",rssi_val );

          }
          if ((OneSecondCounter % BatteryReadPeriod) == 0) //(powerChkpin.get() == hal::IHalGpioIn::PinState::Reset))
          {
              // read battery periodically per minute
              // ADCVrefPin.set(hal::IHalGpioOut::PinState::Set);
//              ADCBatt.selectADCChannel(static_cast<uint8_t>(hal::HalAdc::ADCChannel::DoubleBatteryChannel)); // SingleBatteryChannel
              // ADCBatt.selectADCChannel(static_cast<uint8_t>(hal::HalAdc::ADCChannel::SingleBatteryChannel)); //
        	  sConfig.Channel = ADC_CHANNEL_11;//Double battery
//        	  sConfig.Channel = ADC_CHANNEL_12;//Single battery
              sConfig.Rank = ADC_REGULAR_RANK_1;
              sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
              sConfig.SingleDiff = ADC_SINGLE_ENDED;
              sConfig.OffsetNumber = ADC_OFFSET_NONE;
              sConfig.Offset = 0;
          	 if(PowerSource == BATTERY_POWER)		//enable this comment for production
          	 {
              if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
              {
                  Error_Handler();
              }


              halResult = HAL_ADC_Start_IT(&hadc1);
              if (halResult != HAL_OK)
              {
                  Error_Handler();
              }
         	 }

          }
      }
      if (BattDataAvailablefg == 1)
      {

    	  BattData = HAL_ADC_GetValue(&hadc1);

          BattDataAvailablefg = 0;

          //ADCBatt.stopAdcConv();
          halResult = HAL_ADC_Stop_IT(&hadc1);
          // ADCVrefPin.set(hal::IHalGpioOut::PinState::Reset);
          ActualBatteryVtg = (float)BattData * stepSize * 2;
          APP_DBG_MSG("  Actual Battery Voltage : %f\r\n", ActualBatteryVtg);
          BattValInPercentage = /*(float)*/((ActualBatteryVtg-1950)/(3000-1950))*100;//voltage range is from 1.95v to 3.0v
          APP_DBG_MSG("  Actual Battery Voltage in percentage : %d\r\n", BattValInPercentage);
          tx_semaphore_put(&sem_BASProcessSignal);
          //               SEGGER_RTT_printf(0, "Actual Battery voltage %d mV\r\n", ActualBatteryVtg);
//           if ((onetimebatteryreadfg == 1)&&(PowerSource == USB_POWER))
//           {
//               battreadcount++;
//               if (ActualBatteryVtg > 1100)
//               {
//                   singlebatteryconnectedcount++;
//               }
//               if (battreadcount >= 10)
//               {
//                   onetimebatteryreadfg = 0;
//                   BatteryReadPeriod=60;
//               }
//               else if (singlebatteryconnectedcount > 5)
//               {
//                   // disable watchdog
//                    while (1)
//                    {
//                    	if(LedOnFg==0)
//                    	{
//                    		PWM_Start(LED_Red);
//                    		LedOnFg=1;
//                    	}
//                    	else
//                    	{
//                    		LedOnFg=0;
//                    		PWM_Stop(LED_Red);
//                    	}
//                        Thread_delay(1000);
////                        halIWdt.reset();
//                    }
//               }
//           }
//           else
        	if (ActualBatteryVtg < 2000)
           {
              LowBatteryCount++;
              if ((LowBatteryCount >= 2) && (CriticalBattFg == 0))
              {
                  CriticalBattFg = 1;

                  PWM_Stop(LED_Blue);
                  PWM_Stop(LED_Green);
//                  PWM_Stop(LED_Yellow);
//                  PWM_Stop(LED_Cyan);
                  PWM_Start(LED_Red);
                  while (1)
                  {
//stop the system working and shut down
    				  HAL_IWDG_Refresh(&hiwdg);
    				  HAL_Delay(1000);
                  }
              }
          }
          else if (ActualBatteryVtg < 2300) // && (periodicapplication.LowBattFg == 0))
          {

              if((BLEConnectLED.PatternStatus==START)&&(LowBattFg == 0))
              {
            	  LowBattFg = 1;
            	  Ueventsgrp.EventFlag_bits.LowBattEvent = 1;
              }
          }
          else
          {
              LowBatteryCount = 0;
              LowBattFg=0;
              PWM_Stop( LED_Red);
              if(LowBatt.PatternStatus == START)
              LowBatt.PatternStatus = STOP;
          }
          //-----------------------------------------------------------
          // dbgq.SrcPtr = dbgqueue.DbgSrcBuffer.data();
          // std::fill(std::begin(dbgqueue.DbgSrcBuffer), std::end(dbgqueue.DbgSrcBuffer), 0);
          // //                sprintf(Dbg_log, "Battery Data is %d   \r\n", periodicapplication.BattData_);
          // sprintf(Dbg_log, "Battery Data is %.4f   \r\n", ActualBatteryVtg);
          // memcpy(&dbgqueue.DbgSrcBuffer, Dbg_log, sizeof(dbgqueue.DbgSrcBuffer));
          // result = dbgq.push(osal::Wait);
          // feAssert(result == TX_SUCCESS);
          //-----------------------------------------------------------
      }

	}
}

void Periodic_Mgr(void)
{
	tx_semaphore_put(&semaphore_Periodic_evt);
}
void PWM_Stop(uint32_t Channel)
{
	switch(Channel)
	{
		case LED_Red:
		case LED_Green:
		case LED_Blue:
		case Buzzer:
			HAL_TIM_PWM_Stop(&htim2, Channel);
			break;
		case LED_Yellow:
			HAL_TIM_PWM_Stop(&htim2, LED_Red);
			HAL_TIM_PWM_Stop(&htim2, LED_Green);
			break;
		case LED_Cyan:
			HAL_TIM_PWM_Stop(&htim2, LED_Blue);
			HAL_TIM_PWM_Stop(&htim2, LED_Green);
			break;
		case LED_Purple:
			HAL_TIM_PWM_Stop(&htim2, LED_Blue);
			HAL_TIM_PWM_Stop(&htim2, LED_Red);
			break;
		default:
			break;
	}

}
void PWM_Start(uint32_t Channel)
{
	TIM_OC_InitTypeDef sConfigOC;
	switch(Channel)

	{
	case LED_Red:

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = PULSE_VALUE1;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        // if (cfgProd::ConfigurationRC::USBConnectedFg == 1)
        {
            if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, Channel) != HAL_OK)
            {
                Error_Handler();
            }
            HAL_TIM_PWM_Start(&htim2, Channel);
        }
		break;
	case LED_Green:
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = PULSE_VALUE1;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        // if (cfgProd::ConfigurationRC::USBConnectedFg == 1)
        {
            if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, Channel) != HAL_OK)
            {
                Error_Handler();
            }
            HAL_TIM_PWM_Start(&htim2, Channel);
        }
		break;
	case LED_Blue:
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = PULSE_VALUE1;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        // if (cfgProd::ConfigurationRC::USBConnectedFg == 1)
        {
            if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, Channel) != HAL_OK)
            {
                Error_Handler();
            }
            HAL_TIM_PWM_Start(&htim2, Channel);
        }
		break;
	case Buzzer:
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = PULSE_VALUE_BUZZ;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, Channel) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_TIM_PWM_Start(&htim2, Channel);
		break;
	case LED_Yellow:
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = PULSE_VALUE1;//PERIOD_VALUE;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, LED_Red ) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC,   LED_Green) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_TIM_PWM_Start(&htim2, LED_Red );
        HAL_TIM_PWM_Start(&htim2, LED_Green);
		break;
	case LED_Cyan:
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = PULSE_VALUE1;//PERIOD_VALUE;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, LED_Blue ) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC,   LED_Green) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_TIM_PWM_Start(&htim2, LED_Blue );
        HAL_TIM_PWM_Start(&htim2, LED_Green);
        break;
	case LED_Purple:
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = PULSE_VALUE1;//PERIOD_VALUE;
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, LED_Blue ) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC,   LED_Red) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_TIM_PWM_Start(&htim2, LED_Blue );
        HAL_TIM_PWM_Start(&htim2, LED_Red);
        break;
	default:
		break;
	}

}
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  BattDataAvailablefg = 1;

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
}
