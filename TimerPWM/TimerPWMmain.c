/* TimerPWMmain.c
 * Jonathan Valvano
 * November 3, 2025
 * Derived from timx_timer_mode_pwm_edge_sleep_LP_MSPM0G3507_nortos_ticlang
 *              gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang
 *
 */


#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/PWM.h"
// PA12 is CCP0 PWM output
// PA13 is CCP1 PWM output
//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S1 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S2 negative logic switch,  index 48 in IOMUX PINCM table
int main(void){
  uint32_t sw2,lasts2;
  uint32_t Duty,Period,Change;
  Clock_Init40MHz();
  LaunchPad_Init();
//  PWM_Init(PWMUSELFCLK,0,1000,250,750);    // 32.768 Hz
//  Duty = 100;
//  Period = 1000;
//  Change = 100;
  PWM_Init(PWMUSEBUSCLK,19,10000,2500,7500); // 100Hz
  Duty = 1000;
  Period = 10000;
  Change = 1000;
  lasts2 = (~(GPIOB->DIN31_0)) & S2;
  while(1){
    Clock_Delay(1000000); // debounce switch
    sw2 = (~(GPIOB->DIN31_0)) & S2;
    if(sw2 && (lasts2==0)){ // touch s2
      Duty = Duty+Change;
      if(Duty >= Period){
        Duty = Change;
      }
      PWM_SetDuty(Duty,Period-Duty);
    }
    lasts2 = sw2;
  }
}



