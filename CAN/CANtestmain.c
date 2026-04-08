/* CANtestmain.c
 * Jonathan Valvano
 * December 11, 2025
 * Derived from timx_timer_mode_periodic_sleep_LP_MSPM0G3507_nortos_ticlang
 *              gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang
 *              dac12_fixed_voltage_vref_internal_LP_MSPM0G3507_nortos_ticlang
 *              dac12_fifo_timer_event_LP_MSPM0G3507_nortos_ticlang
 */


#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/Timer.h"
#include "../RTOS_Labs_common/CAN.h"

uint32_t TheValue=0;
uint32_t YourID=253;
uint32_t RecvID,RecDLC;
uint8_t RecvData[8];
uint16_t TheDLC=4;
uint8_t TheData[8];
int main(void){ 
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  CAN_Init(); // Clock_Init80MHz and  LaunchPad_Init must preced CAN_Init

// enable interrupts after everything is initialized   
  CAN_EnableInterrupts(1);
  __enable_irq();
  while(1){
    if(CAN_GetMailNonBlock(&RecvID,&RecDLC,RecvData)){
      GPIOB->DOUTTGL31_0 = RED;        
    }
    if((GPIOB->DIN31_0&(1<<21))==0){
      GPIOB->DOUTTGL31_0 = GREEN;     
      for(int i=0; i<TheDLC; i++){
        TheData[i] = TheValue++;
      }
      CAN_Send(YourID,TheDLC,TheData);
      YourID = (YourID+1)&0x3FF; // 0 to 1023
      TheDLC = (TheDLC+1)%9;     // 0 to 8
      while((GPIOB->DIN31_0&(1<<21))==0){
        Clock_Delay(100000);
      }
    }
  }
}


