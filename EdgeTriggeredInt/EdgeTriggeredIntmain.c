/* EdgeTriggeredIntmain.c
 * Interrupt from Switch S3, output to LEDs
 * Jonathan Valvano
 * July 19, 2025
 * Derived from gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang
 *              gpio_input_capture_LP_MSPM0G3507_nortos_ticlang
 */
// Brief description of the program:
// An interrupt is invoked on the falling edge of S2 (PB21).
// When S2 is touched (falling edge) the red LED is toggled.
// The green LED is toggled in the main program.
// DOUTTGL31_0 is used so the code is thread-safe.


#include <ti/devices/msp/msp.h>
#include "../inc/EdgeTriggered.h"
#include "../inc/LaunchPad.h"

//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S1 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S2 negative logic switch,  index 48 in IOMUX PINCM table
// Insert J5: Connects PB22 to blue LED2
// Insert J6: Connects PB26 to red LED2
// Insert J7: Connects PB27 to green LED2
uint32_t Count;
int main(void){
  __disable_irq();
  EdgeTriggered_Init();
  Count = 0;
  __enable_irq();
  while(1){
    GPIOB->DOUTTGL31_0 = GREEN; // toggle PB27
  }
}
// do not need to use IIDX when there is a single interrupt source
void GROUP1_IRQHandler(void){
  Count++; // number of touches
  GPIOB->DOUTTGL31_0 = RED; // toggle PB26
  GPIOB->CPU_INT.ICLR = 0x00200000;   // clear bit 21
}
// use IIDX when there are multiple interrupt sources
/*
void GROUP1_IRQHandler(void){
  uint32_t stat;
  while((stat = GPIOB->CPU_INT.IIDX)!=0){
    if(stat == 22){ // PB21
      Count++; // number of touches
      GPIOB->DOUTTGL31_0 = RED; // toggle PB26
    }
  }
}
*/


