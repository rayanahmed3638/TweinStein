/* PeriodicSysTickInts.c
 * Jonathan Valvano
 * July 19, 2025
 * Derived from timx_timer_mode_periodic_sleep_LP_MSPM0G3507_nortos_ticlang
 *              gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang
 *
 */

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S1 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S2 negative logic switch,  index 48 in IOMUX PINCM table
// DOUTTGL31_0 is used so the code is thread-safe.

uint32_t Count;
void SysTick_IntArm(uint32_t period, uint32_t priority){
  SysTick->CTRL  = 0x00;      // disable during initialization
  SysTick->LOAD  = period-1;  // set reload register
  //The ARM Cortex-M0+ only implements the most significant 2 bits of each 8-bit priority field (giving the 4 priority levels).
  SCB->SHP[1]    = (SCB->SHP[1]&(~0xC0000000))|priority<<30;    // set priority (bits 31,30)
  SysTick->VAL   = 0;         // clear count, cause reload
  SysTick->CTRL  = 0x07;      // Enable SysTick IRQ and SysTick Timer
}
#define PA12INDEX 33

int main(void){
  __disable_irq();
  LaunchPad_Init();
//  Clock_Init(OSCFREQ32MHZ);
  Clock_Init40MHz();
//  Clock_Init80MHz(0);
  Count =  0;
//  SysTick_IntArm(16000000,2); //32M/16M = 2Hz
  SysTick_IntArm(400000,2); //40M/400k = 100Hz
//  SysTick_IntArm(800000,2); //80M/800k = 100Hz
 __enable_irq();
  while(1){
    GPIOA->DOUTTGL31_0 = RED1;
  }
}

void SysTick_Handler(void){
  GPIOB->DOUTTGL31_0 = GREEN; // toggle PB27
  GPIOB->DOUTTGL31_0 = GREEN; // toggle PB27
  Count++;
  GPIOB->DOUTTGL31_0 = GREEN; // toggle PB27
}


