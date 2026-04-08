/* TExaSTestmain.c
 * Jonathan Valvano
 * August 8, 2023
 * Derived
 * Derived from adc12_single_conversion_vref_internal_LP_MSPM0G3507_nortos_ticlang
 *              adc12_single_conversion_LP_MSPM0G3507_nortos_ticlang
 */
// Educational BoosterPack MKII (BOOSTXL-EDUMKII)
// Analog MKII  Joystick
// J1.5 joystick Select button (digital) PA26
// J1.2 joystick horizontal (X) (analog) PA25_ADC0.2
// J3.26 joystick vertical (Y) (analog)  PA18_ADC1.3

// MKII accelerometer
// J3.23 accelerometer X (analog)    PB19__ADC1.6
// J3.24 accelerometer Y (analog)    PA22__ADC0.7
// J3.25 accelerometer Z (analog)    PB18__ADC1.5

// PB24 ADC0 channel 5 J1.6 MKII microphone in

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/TExaS.h"
//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S2 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S3 negative logic switch,  index 48 in IOMUX PINCM table


int main1(void){ // example scope using MKII booster
  __disable_irq();
 // Clock_Init80MHz(0);
 // Clock_Init40MHz();
  Clock_Init(OSCFREQ32MHZ);
  LaunchPad_Init();
  TExaS_Init(ADC0,5,0); //microphone
  __enable_irq();
  while(1){
   // Clock_Delay(120000000);
   // GPIOA->DOUT31_0 ^= RED1;
  }
}

int main(void){ // example logic analyzer usage
  __disable_irq();
  Clock_Init80MHz(0);
 // Clock_Init40MHz();
  //Clock_Init(OSCFREQ32MHZ);
  LaunchPad_Init();
  GPIOB->DOUTSET31_0 = (BLUE | RED );
  TExaS_Init(0,0,&TExaS_LaunchPadLogic); // see TExaS.h for pin selection
  __enable_irq();
  while(1){
    for(int i=0;i<10;i++){
      Clock_Delay(Clock_Freq()/100); // 10kHz
      GPIOB->DOUTTGL31_0 = (BLUE | RED | GREEN); // clear LED2
    }
    GPIOA->DOUTTGL31_0 = RED1;
  }
}


