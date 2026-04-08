/* ADCTimertriggerTestmain.c
 * Jonathan Valvano
 * May 23, 2025
 * Derived from adc12_single_conversion_vref_internal_LP_MSPM0G3507_nortos_ticlang
 *              adc12_single_conversion_LP_MSPM0G3507_nortos_ticlang
 *              adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang
 */

// ****note to students****
// the data sheet says the ADC does not work when clock is 80 MHz
// however, the ADC seems to work on my boards at 80 MHz
// I suggest you try 80MHz, but if it doesn't work, switch to 40MHz

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../RTOS_Labs_common/ADCTimer.h"

volatile uint32_t Data0,Data1,Flag0,Flag1,Timems0,TimeSec0,Timems1,TimeSec1;
void ADC0_IRQHandler(void){ // 1000 Hz
  ADC0->ULLMEM.CPU_INT.ICLR = 0x00000100;
  Data0 = ADC0->ULLMEM.MEMRES[0];
  Flag0 = 1;
  Timems0++;
  if((Timems0%1000)==0){
    TimeSec0++;
    GPIOB->DOUT31_0 ^= RED;
  }
  ADC0->ULLMEM.CTL0 |= 1; // restart
}

uint16_t volt0,volt1; // voltage in mV
int main1(void){
  __disable_irq();
  Clock_Init80MHz(0);
  Timems0 = TimeSec0 = 0;
  Flag0 = 0;
  LaunchPad_Init();

  ADC0_TimerG0_Init(7,ADCVREF_VDDA,5000,8,1); 
  __enable_irq();
  while(1){    /* toggle on sample */
    if(Flag0){
     Flag0 = 0;
     volt0 = (Data0*3300)>>12;
     GPIOA->DOUT31_0 ^= RED1;
    }
  }
}
void ADC1_IRQHandler(void){ // 100 Hz
  ADC1->ULLMEM.CPU_INT.ICLR = 0x00000100;
  Data1 = ADC1->ULLMEM.MEMRES[0];
  Flag1 = 1;
  Timems1++;
  if((Timems1%100)==0){
    TimeSec1++;
    GPIOB->DOUT31_0 ^= BLUE;
  }
  ADC1->ULLMEM.CTL0 |= 1; // restart
}
int main(void){
  __disable_irq();
  Clock_Init80MHz(0);
  Timems0 = TimeSec0 = 0;
  Flag0 = 0;
  Timems1 = TimeSec1 = 0;
  Flag1 = 0; 
  LaunchPad_Init();
  ADC0_TimerG0_Init(7,ADCVREF_VDDA,5000,8,1); 

//ADC1 channel 5, PB18, slidepot
  ADC1_TimerG8_Init(5,ADCVREF_VDDA,50000,8,0); 
  __enable_irq();
  while(1){    /* toggle on sample */
    if(Flag0){
     Flag0 = 0;
     volt0 = (Data0*3300)>>12;
     GPIOA->DOUT31_0 ^= RED1;
    }
    if(Flag1){
     Flag1 = 0;
     volt1 = (Data1*3300)>>12;
     GPIOA->DOUT31_0 ^= RED1;
    }
  }
}

