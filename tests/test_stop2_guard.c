/* Extracted production PreIdle/Idle/PostIdle plus board prepare function.
   A command timeout and a BUSY transfer must never lead to STOP2; retries and
   the nested CPU2 command-wait fallback remain functional. */
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#define CFG_LPM_SUPPORTED 1
#define UTIL_LPM_STOPMODE 1U
#define HAL_OK 0U
#define HAL_BUSY 2U
#define GPIO_PIN_SET 1U
#define LED_NET_BLUE_GPIO_Port 0U
#define LED_NET_BLUE_Pin 1U
#define LED_NET_RED_Pin 2U
typedef unsigned HAL_StatusTypeDef;
static uint8_t Stop2SensorsPrepared,Stop2SensorsReady,SequencerWaitDepth;
static unsigned flash_result,entries,sleeps,wakes,restores,parked;
static unsigned mode=UTIL_LPM_STOPMODE;
static unsigned UTIL_LPM_GetMode(void){return mode;}
static void DigitalMicrophone_PrepareStop2(void){}
static void AnalogMicrophone_Stop(void){}
static void BSP_SENSOR_PWR_Off(void){}
static unsigned Vibrometer_IsReady(void){return 1;}
static unsigned Vibrometer_Stop(void){return 0;}
static unsigned Magnetometer_EnterLowPower(void){return 0;}
static unsigned Thermometer_EnterLowPower(void){return 0;}
static void DT_ParkSensorBuses(void){parked=1;}
static HAL_StatusTypeDef QspiNor_PrepareStop2(void){return flash_result;}
static void DT_Log(const char *text){(void)text;}
static void HAL_GPIO_WritePin(unsigned port,unsigned pins,unsigned value){(void)port;(void)pins;(void)value;}
static void PWR_EnterSleepMode(void){sleeps++;}
static void PWR_ExitSleepMode(void){}
static void UTIL_LPM_EnterLowPower(void){assert(Stop2SensorsReady);entries++;}
static void PWR_CompleteStopModeWakeup(void){wakes++;}
static void Custom_APP_ResumeFromStop2(void){parked=0;restores++;}
#include "stop2-under-test.inc"
static void idle(void){UTIL_SEQ_PreIdle();UTIL_SEQ_Idle();UTIL_SEQ_PostIdle();}
int main(void){
  flash_result=HAL_BUSY;idle();assert(entries==0&&sleeps==0&&!parked&&restores==1);
  flash_result=1;idle();assert(entries==0&&sleeps==0&&!parked&&restores==2);
  flash_result=HAL_OK;idle();assert(entries==1&&!parked&&restores==3);
  /* A task can cancel STOP2 after preparation; PostIdle must still restore buses. */
  UTIL_SEQ_PreIdle();mode=0;UTIL_SEQ_PostIdle();assert(!parked&&restores==4);
  mode=UTIL_LPM_STOPMODE;SequencerWaitDepth=1;idle();
  assert(entries==1&&sleeps==1&&restores==4);
  assert(wakes==5);
  puts("PASS: STOP2 veto on BUSY/error, retry success, canceled idle bus restoration, nested CPU2 wait.");
}
