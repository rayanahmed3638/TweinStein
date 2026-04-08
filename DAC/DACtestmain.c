/* DACtestmain.c
 * Jonathan Valvano
 * November 30, 2022
 * Derived from timx_timer_mode_periodic_sleep_LP_MSPM0G3507_nortos_ticlang
 *              gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang
 *              dac12_fixed_voltage_vref_internal_LP_MSPM0G3507_nortos_ticlang
 *              dac12_fifo_timer_event_LP_MSPM0G3507_nortos_ticlang
 */


#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/Timer.h"
#include "../inc/DAC.h"
//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S2 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S3 negative logic switch,  index 48 in IOMUX PINCM table

// PA15 J3.30  DACout
uint32_t Data;
int main1(void){ // static test
 // __disable_irq();
//  Clock_Init40MHz();
  Clock_Init(OSCFREQ32MHZ);
  Data=2048;
  LaunchPad_Init();
  DAC_Init();
  while(1){
    Data = Data+1;
    if(Data > 4095) Data = 0;
    DAC_Out(Data);
    Clock_Delay1ms(1); // about 1ms
  }
}

const uint16_t Wave[64] = {
  2048,2249,2447,2642,2831,3013,3185,3347,3495,3630,3750,
  3853,3939,4007,4056,4085,4095,4085,4056,4007,3939,3853,
  3750,3630,3495,3347,3185,3013,2831,2642,2447,2249,2048,
  1847,1649,1454,1265,1083,911,749,601,466,346,243,
  157,89,40,11,1,11,40,89,157,243,346,
  466,601,749,911,1083,1265,1454,1649,1847
};
uint32_t Index;
int main(void){ // dynamic test
  __disable_irq();
 // Clock_Init(OSCFREQ32MHZ);
 // Clock_Init40MHz();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  Index = 0;

  DAC_Init();
 // TimerG0_IntArm(5,32,2); // interrupt frequency = 32MHz/5/32 = 200kHz
 // TimerG0_IntArm(5,20,2); // interrupt frequency = 20MHz/5/20 = 200kHz
  TimerG0_IntArm(5,40,2); // interrupt frequency = 40MHz/5/40 = 200kHz
  // wave frequency = interrupt frequency/table size= 200kHz/64= 3.125kHz
  __enable_irq();
  while(1){
     __WFI();
  }
}
// 200kHz/64 = 3.125kHz
void TIMG0_IRQHandler(void){
  if((TIMG0->CPU_INT.IIDX) == 1){ // this will acknowledge
    GPIOB->DOUTTGL31_0 = GREEN;
    Index = Index+1;
    if(Index > 63) Index = 0;
    DAC_Out(Wave[Index]);
  }
}



