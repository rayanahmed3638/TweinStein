/* GP2Y0ATestmain.c
 * Jonathan Valvano
 * November 30, 2025
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)

 * Derived from adc12_single_conversion_vref_internal_LP_MSPM0G3507_nortos_ticlang
 *              adc12_single_conversion_LP_MSPM0G3507_nortos_ticlang
 */
// IR analog distance sensors
//   30 cm GP2Y0A41SK0F or 80 cm long range GP2Y0A21YK0F 
//   PA26 Right  ADC0_1
//   PA22 Left   ADC0_7
//   PA24 Center ADC0_3
//   PA27 Extra  ADC0_0

// ****note to students****
// the data sheet says the ADC does not work when clock is 80 MHz
// however, the ADC seems to work on my boards at 80 MHz
// I suggest you try 80MHz, but if it doesn't work, switch to 40MHz

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/ADC.h"
//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S2 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S3 negative logic switch,  index 48 in IOMUX PINCM table
uint32_t Right,Left,Center;
uint16_t volt; // voltage in mV
int main1(void){
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ADC0_Init(3,ADCVREF_VDDA); // center (analog)
  while(1){    /* toggle on sample */
    Clock_Delay(10000000);
    Center = ADC0_In();
    volt = (Center*3300)>>12;
    GPIOA->DOUT31_0 ^= RED1;
  }
}
int main2(void){
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ADC_InitDual(ADC0,1,7,ADCVREF_VDDA); //Right and Left
  while(1){    /* toggle on sample */
    Clock_Delay(10000000);
    ADC_InDual(ADC0,&Right,&Left);

    GPIOA->DOUT31_0 ^= RED1;
  }
}

int main(void){ // main3, test three sensors
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ADC_InitTriple(ADC0,1,3,7,ADCVREF_VDDA); //Right Center, and Left
  
  while(1){    /* toggle on sample */
    Clock_Delay(10000000);
    ADC_InTriple(ADC0,&Right,&Center,&Left);
    GPIOA->DOUT31_0 ^= RED1;
  }
}


uint32_t elapsed,n;
#define SIZE 16
uint32_t Histogram[SIZE]; // probability mass function
int main4(void){uint32_t start,end,i;
  int32_t d,Data;
  Clock_Init80MHz(0);
  
  LaunchPad_Init();

  ADC0_InitAve(3,0); // center (analog)
  SysTick->CTRL = 0;           // 1) disable SysTick during setup
  SysTick->LOAD = 0xFFFFFF;    // 2) max
  SysTick->VAL = 0;            // 3) any write to current clears it
  SysTick->CTRL = 0x00000005;  // 4) enable SysTick with core clock
  while(1){    /* toggle on sample */
    for(n=0; n<=7; n++){
      ADC0_InitAve(7,n); //center (analog)
      Center = ADC0_In();
      Clock_Delay(1000);
      start = SysTick->VAL;
      Center = ADC0_In();
      end = SysTick->VAL;
      elapsed = ((start-end)&0xFFFFFF)-13;
      for(i=0; i<SIZE; i++) Histogram[i] = 0; // clear
      Center = 0;
      for(i=0; i<1000; i++){
        Clock_Delay(1000);
        Center += ADC0_In();
      }
      Center = Center/1000;
      for(i=0; i<1000; i++){
        Clock_Delay(1000);
        Data = ADC0_In();
        if(Data<Center-SIZE/2){
          Histogram[0]++;
        }else if(Data>=Center+SIZE/2){
          Histogram[SIZE-1]++;
        }else{
          d = Data-Center+SIZE/2;
          Histogram[d]++;
        }
      }
    GPIOA->DOUT31_0 ^= RED1;
    }
  }
}
