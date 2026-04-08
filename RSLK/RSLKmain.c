/* RSLKmain.c
 * Jonathan Valvano
 * v2.0.1
 * July 19, 2025
 *
 */

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/PWM1.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/SSD1306.h"
#include "../inc/ADC.h"
#include "../inc/InputCapture.h"

uint32_t Duty,Period,Change;
// Motor
//   PB4  Motor_PWML=PB.4/TIMA1_C0;
//   PB1  Motor_PWMR=PB.1/TIMA1_C1;
//   PB0  Motor_DIR_L  1 means forward, 0 means backward
//   PB16 Motor_DIR_R, ***change from 2.0.0***
// tachometer
//   PB8  ELA  TA0_C0
//   PB7  ERB  GPIO input TG8_C1/TG6_C1/TA1_C0N
//   PB6  ELB  GPIO input TG8_C0/TG6_C0
//   PB12 ERA  TA0_C1
// Negative Logic Bumper switches
//   PA27 Left, Bump 0, ***change from 2.0.0***
//   PB15 Center Left, Bump 1, ***change from 2.0.0***
//   PA28 Center Right, Bump 2
//   PA31 Right, Bump 3
// SSD1306 I2C OLED
//   PB2  SCL I2C clock
//   PB3  SDA I2C data
// IR analog distance sensors
//   PA26 Right ADC0_1
//   PB24 Center ADC0_5 (remove J9)
//   PA22 Left ADC0_3
// TF Luna TOF distance sensor
//   TM4C MSPM0
//   PB1  PA8 TxD microcontroller sensor RxD pin 2
//   PB0  PA9 RxD microcontroller sensor TxD pin 3
// LaunchPad pins
//   PA0  red LED1
//   PB22 BLUE LED2
//   PB26 RED LED2
//   PB27 GREEN LED2
//   PA18 S1 positive logic switch
//   PB21 S2 negative logic switch
// Remove +5V jumper from debugger to target
// Remove J9: Disconnects thermistor from PB24
// Remove J16 J17 J18: Disconnects light sensor from the microcontroller PA22 PA27 PA26
// Insert J5: Connects PB22 to blue LED2
// Insert J6: Connects PB26 to red LED2<
// Insert J7: Connects PB27 to green LED2
uint32_t bump;
int main0(void){ // use main0 to test bump switches
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  Bump_Init();
   __enable_irq();
  while(1){
    bump = Bump_In();
  }
}
int main1(void){ // use main1 to test low level motor
  uint32_t sw2,lasts2;
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
   __enable_irq();
//  PWM_Init(PWMUSELFCLK,0,1000,250,750);    // 32.768 Hz
//  Duty = 100;
//  Period = 1000;
//  Change = 100;
  PWM1_Init(PWMUSEBUSCLK,39,10000,2500,7500); // 100Hz
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
      PWM1_SetDuty(Duty,Period-Duty);
    }
    lasts2 = sw2;
  }
}

int main2(void){ // use main2 to test motors and OLED
  uint32_t sw2,lasts2;

  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  SSD1306_OutString("RSLK 2");
  Bump_Init();
   __enable_irq();
//  PWM_Init(PWMUSELFCLK,0,1000,250,750);    // 32.768 Hz
//  Duty = 100;
//  Period = 1000;
//  Change = 100;
  Motor_Init(); // 100Hz
  Duty = 1000;
  Period = 10000;
  Change = 1000;
  lasts2 = (~(GPIOB->DIN31_0)) & S2;
  SSD1306_SetCursor(0,1);
  SSD1306_OutString("Period= ");
  SSD1306_OutUDec(Period);
  SSD1306_SetCursor(0,2);
  SSD1306_OutString("Duty= ");
  SSD1306_SetCursor(6,2);
  SSD1306_OutUDec(Duty);
  while(1){
    Clock_Delay(100000); // debounce switch
    if(Bump_In()){
      Motor_Forward(4,4);
      while(Bump_In()){};
      Motor_Forward(Duty,Duty);
    }
    sw2 = (~(GPIOB->DIN31_0)) & S2;
    if(sw2 && (lasts2==0)){ // touch s2
      Duty = Duty+Change;
      if(Duty >= Period){ // 10 20 30 ... 90%
        Duty = Change;
      }
      Motor_Forward(Duty,Duty);
      SSD1306_SetCursor(6,2);
      SSD1306_OutUDec(Duty);
    }
    lasts2 = sw2;
  }
}
uint32_t Left,Center,Right;
uint32_t Bump;
int main3(void){ // use main3 to test OLED and IR sensors
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  SSD1306_OutString("RSLK 2");
  SSD1306_SetCursor(0,1);
  SSD1306_OutString("Left  =");
  SSD1306_SetCursor(0,2);
  SSD1306_OutString("Center=");
  SSD1306_SetCursor(0,3);
  SSD1306_OutString("Right =");
  SSD1306_SetCursor(0,4);
  SSD1306_OutString("Bump  =");
  //PA26 IR analog distance sensor, Right ADC0_1
  //PB24 IR analog distance sensor, Center ADC0_5
  //PA22 IR analog distance sensor, Left ADC0_3
  //ADC0_Init(5,ADCVREF_VDDA);
  ADC_InitTriple(ADC0,1,5,3,ADCVREF_VDDA);
  Bump_Init();
   __enable_irq();

  while(1){
    Clock_Delay(100000); // slow down
    Bump = Bump_In();
    ADC_InTriple(ADC0,&Right,&Center,&Left);
    SSD1306_SetCursor(7,1);
    SSD1306_OutUDec(Left);
    SSD1306_SetCursor(7,2);
    SSD1306_OutUDec(Center);
    SSD1306_SetCursor(7,3);
    SSD1306_OutUDec(Right);
    SSD1306_SetCursor(7,4);
    SSD1306_OutUHex7(Bump);
  }
}
uint32_t Count0=0,Count1=0,Time0,Time1,Last0,Last1,Period0,Period1;
uint32_t Data0[8],Data1[8];
int main(void){ // use main5 to test motors and tach
  uint32_t sw2,lasts2;

  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  SSD1306_OutString("RSLK 2");
  SSD1306_SetCursor(0,1);
  SSD1306_OutString("Left = ");
  SSD1306_SetCursor(0,2);
  SSD1306_OutString("Right= ");
  SSD1306_SetCursor(0,3);
  SSD1306_OutString("Duty = ");
  SSD1306_SetCursor(0,4);
  SSD1306_OutString("Bump  =");
  Bump_Init();
   __enable_irq();
  Motor_Init(); // 100Hz
  InputCapture_InitDual(INPUTCAPTUREUSEBUSCLK,79,1); // 1us resolution
  Last0 = 0xFFFFFFFF;  Last1 = 0xFFFFFFFF;
  Count0=0; Count1=0;
  Duty = 500;
  Period = 10000;
  Change = 1000;
  lasts2 = (~(GPIOB->DIN31_0)) & S2;
  while(1){
    SSD1306_SetCursor(7,1);
    SSD1306_OutUDec(Period0);
    SSD1306_SetCursor(7,2);
    SSD1306_OutUDec(Period1);
    SSD1306_SetCursor(7,3);
    SSD1306_OutUDec(Duty);
    SSD1306_SetCursor(0,5);
    SSD1306_OutUHex32(Bump_In());
    if(Bump_In()){
      Motor_Forward(4,4); // stop
      SSD1306_SetCursor(0,5);
      SSD1306_OutUHex32(Bump_In());
      while(Bump_In()){Clock_Delay(80000);};
      while(Bump_In()==0){Clock_Delay(80000);};
      while(Bump_In()){Clock_Delay(80000);};
      Motor_Forward(Duty,Duty); // turn left
    }
    sw2 = (~(GPIOB->DIN31_0)) & S2;
    if(sw2 && (lasts2==0)){ // touch s2
      Duty = Duty+Change;
      if(Duty >= Period/2){ // 10 20 30 ... 90%
        Duty = 500;
      }
      Motor_Forward(Duty,Duty); // turn left
    }
    lasts2 = sw2;
  }
}

void TIMA0_IRQHandler(void){
  uint32_t iidx = TIMA0->CPU_INT.IIDX;// this will acknowledge
  if(iidx == 5){ // 5 means capture CCD0, PB8  ELA  TA0_C0
    Time0 = TIMA0->COUNTERREGS.CC_01[0]; // time now
    Period0  = (Last0-Time0)&0xFFFF; // elapsed time since last
    Last0 = Time0;
    Data0[Count0&0x07] = Period0;
    Count0++;
  }
  if(iidx == 6){ // 6 means capture CCD1=PB12 ERA  TA0_C1
    Time1 = TIMA0->COUNTERREGS.CC_01[1]; // time now
    Period1  = (Last1-Time1)&0xFFFF; // elapsed time since last
    Last1 = Time1;
    Data1[Count1&0x07] = Period1;
    Count1++;
  }
}


