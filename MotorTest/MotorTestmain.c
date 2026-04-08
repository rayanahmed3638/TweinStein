/* MotorTestmain.c
 * Jonathan Valvano
 * Feb 10, 2026
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)


PINCM  Pin   Mode    Timer     Usage
17     PB4   Mode4   TIMA1_C0  Left Motor,  IN2, PWM low for forward, GPIO high for reverse 
13     PB1   Mode4   TIMA1_C1  Left Motor,  IN1, GPIO high for forward, PWM low for reverse  

25     PB8   Mode4   TIMA0_C0  Right Motor, IN2, PWM low for forward, GPIO high for reverse   
26     PB9   Mode4   TIMA0_C1  Right Motor, IN1, GPIO high for forward, PWM low for reverse 

23     PB6   Mode7   TIMG6_C0  Steering Servo PWM high 20ms period,   
 */


#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/SSD1306.h"
#include "../RTOS_Labs_common/PWMA0.h"
#include "../RTOS_Labs_common/PWMA1.h"
#include "../RTOS_Labs_common/PWMG6.h"
//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S1 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S2 negative logic switch,  index 48 in IOMUX PINCM table
// On reset
//   Release S2 to RunForward
//   Touch S2 to RunBackward
// while running
//   touch S2 to change motor duty cycle
//   touch S1 to change servo duty cycle
uint32_t Duty;
#define MOTORPERIOD 10000 // 200Hz
#define MOTORCHANGE 1000  // 10%
#define MOTORMIN 1000     // 10%
#define MOTORMAX 9000     // 90%
uint32_t ServoDuty; // 2000,2250,2500,2750,3000,3250,3500,3750,4000
#define SERVOMIN 2000      // 1ms
#define SERVOMAX 4000      // 2ms
#define SERVOINIT 3000     // 1.5ms
#define SERVOPERIOD 40000  // 20ms
#define SERVOCHANGE 250    // 0.125ms
void SSD1306_Display(void){
  SSD1306_SetCursor(0,3);
  if(Duty == 0){
    SSD1306_OutString("Motor break         ");
    SSD1306_SetCursor(0,4);
    SSD1306_OutString("                    ");
  }else{
    SSD1306_OutString("Motor Period=       ");
    SSD1306_SetCursor(13,3);SSD1306_OutUDec(MOTORPERIOD);
    SSD1306_SetCursor(0,4);
    SSD1306_OutString("Motor Duty =        ");
    SSD1306_SetCursor(13,4);SSD1306_OutUDec(Duty);
  }
  SSD1306_SetCursor(0,5);
  SSD1306_OutString("Servo Period=       ");
  SSD1306_SetCursor(13,5);SSD1306_OutUDec(SERVOPERIOD);
  SSD1306_SetCursor(0,6);
  SSD1306_OutString("Servo Duty =        ");
  SSD1306_SetCursor(13,6);SSD1306_OutUDec(ServoDuty);
}
    
// scope on PB1 PB4, run without motors
// PB4 is Duty, PB1 is Period-Duty
int main0(void){   uint32_t sw2,lasts2;
  Clock_Init80MHz(0);
  LaunchPad_Init();
  PWMA1_Init(PWMUSEBUSCLK,39,MOTORPERIOD,2500,7500); // 200Hz
  PWMA1_Coast(); // low low, sleep mode
  Duty = MOTORMIN;
  lasts2 = (~(GPIOB->DIN31_0)) & S2;
  while(1){    
    Clock_Delay(1000000); // debounce switch
    sw2 = (~(GPIOB->DIN31_0)) & S2;
    if(sw2 && (lasts2==0)){ // touch s2
      Duty = Duty+MOTORCHANGE;
      if(Duty > MOTORMAX){
        Duty = MOTORMIN;
      }
      SSD1306_Display();
      PWMA1_SetDuty(Duty,MOTORPERIOD-Duty);
    }
    lasts2 = sw2;
    
  }
}
// scope on PB1 PB4, spins with left motor forward
// PB4 is Duty (time low), PB1 is high
// scope on PB8,PB9, spins with right motor forward
// PB8 is Duty (time low), PB9 is high
// PB6 1ms to 2ms pulse high, 20ms period
int RunForward(void){   uint32_t sw2,lasts2;
  uint32_t sw1,lasts1;
  PWMA0_Init(PWMUSEBUSCLK,39,MOTORPERIOD,2500,7500); // 200Hz
  PWMA0_Break(); // high, high, break mode
  PWMA1_Init(PWMUSEBUSCLK,39,MOTORPERIOD,2500,7500); // 200Hz
  PWMA1_Break(); // high, high, break mode
  Duty = MOTORMIN;
  lasts2 = (~(GPIOB->DIN31_0)) & S2;
  while(1){    
    Clock_Delay(1000000); // debounce switch
    sw2 = (~(GPIOB->DIN31_0)) & S2;
    sw1 = GPIOA->DIN31_0 & S1;
    if(sw2 && (lasts2==0)){ // touch s2
      Duty = Duty+MOTORCHANGE;
      if(Duty > MOTORMAX){
        Duty = MOTORMIN;
      }
      SSD1306_Display();
      PWMA0_Backward(Duty);
      PWMA1_Forward(Duty);
    }
    if(sw1 && (lasts1==0)){ // touch s1
      ServoDuty = ServoDuty+SERVOCHANGE;
      if(ServoDuty > SERVOMAX){ // 2000,2250,2500,2750,3000,3250,3500,3750,4000
        ServoDuty = SERVOMIN;
      }
      SSD1306_Display();
      PWMG6_SetDuty(ServoDuty);
    }
    lasts2 = sw2;
    lasts1 = sw1;
  }
}

// scope on PB1 PB4, spins with left motor backward
// PB4 is high , PB1 is Duty (time low)
// scope on PB8 PB9, spins with right motor backward
// PB8 is high , PB9 is Duty (time low)
// PB6 1ms to 2ms pulse high, 20ms period
int RunBackward(void){   uint32_t sw2,lasts2;
uint32_t sw1,lasts1;
  PWMA0_Init(PWMUSEBUSCLK,39,MOTORPERIOD,2500,7500); // 200Hz
  PWMA0_Break(); // high, high, break mode
  PWMA1_Init(PWMUSEBUSCLK,39,MOTORPERIOD,2500,7500); // 200Hz
  PWMA1_Break(); // high, high, break mode
  Duty = MOTORMIN;
  lasts2 = (~(GPIOB->DIN31_0)) & S2;
  while(1){    
    Clock_Delay(1000000); // debounce switch
    sw1 = GPIOA->DIN31_0 & S1;
    sw2 = (~(GPIOB->DIN31_0)) & S2;
    if(sw2 && (lasts2==0)){ // touch s2
      Duty = Duty+MOTORCHANGE;
      if(Duty > MOTORMAX){
        Duty = MOTORMIN;
      }
      SSD1306_Display();
      PWMA0_Forward(Duty);
      PWMA1_Backward(Duty);   
   }
    if(sw1 && (lasts1==0)){ // touch s1
      ServoDuty = ServoDuty+SERVOCHANGE;
      if(ServoDuty > SERVOMAX){ // 2000,2250,2500,2750,3000,3250,3500,3750,4000
        ServoDuty = SERVOMIN;
      }
      SSD1306_Display();
      PWMG6_SetDuty(ServoDuty);
    }
    lasts2 = sw2;
    lasts1 = sw1;
  }
}



// PB6 0.9ms to 2.1ms pulse high, 20ms period
// 1ms is 2000
// 2ms is 4000
int main10(void){   uint32_t sw2,lasts2;
  ServoDuty = SERVOINIT;    // 1.5ms
  PWMG6_Init(PWMUSEBUSCLK,39,SERVOPERIOD,SERVOINIT); // 50Hz, 1.5ms
  lasts2 = (~(GPIOB->DIN31_0)) & S2;
  while(1){    
    Clock_Delay(1000000); // debounce switch
    sw2 = (~(GPIOB->DIN31_0)) & S2;
    if(sw2 && (lasts2==0)){ // touch s2
      ServoDuty = ServoDuty+SERVOCHANGE;
      if(ServoDuty > SERVOMAX){ // 2000,2250,2500,2750,3000,3250,3500,3750,4000
        ServoDuty = SERVOMIN;
      }
      SSD1306_Display();
      PWMG6_SetDuty(ServoDuty);
    }
    lasts2 = sw2;
    
  }
}
int main(void){
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  uint32_t  sw2 = (~(GPIOB->DIN31_0)) & S2;
  SSD1306_SetCursor(0,0);
  SSD1306_OutString("ECE445M motor test\n");  
  ServoDuty = SERVOINIT;    // 1.5ms
// period is 20ms
// change is 0.125ms
  PWMG6_Init(PWMUSEBUSCLK,39,SERVOPERIOD,SERVOINIT); // 50Hz, 1.5ms

  SSD1306_Display();
  SSD1306_SetCursor(0,1);
   SSD1306_OutString("Forward test\n");  
  RunForward(); // forward
  if(sw2){
    
    SSD1306_OutString("Backward test\n");  
    RunBackward(); // back
  }
 
  while(1){};
}
int main67(void){
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  
  // Initialize servo
  ServoDuty = SERVOINIT;
  PWMG6_Init(PWMUSEBUSCLK,39,SERVOPERIOD,3000);
  
  // Initialize BOTH motor PWMs
  PWMA0_Init(PWMUSEBUSCLK,39,MOTORPERIOD,2500,7500);
  PWMA1_Init(PWMUSEBUSCLK,39,MOTORPERIOD,2500,7500);
  
  // Set a fixed duty cycle and run forward
  Duty = 3000;  // 30% duty cycle - start low for safety
  PWMA0_Forward(Duty);  // Right motor
  PWMA1_Forward(Duty);  // Left motor
  
  SSD1306_SetCursor(0,0);
  SSD1306_OutString("Motor Test Running");
  
  // Just spin forever
  while(1){
    // Motors should be running now
  }
}