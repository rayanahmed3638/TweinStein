//*****************************************************************************
// user.c
// Runs on MSPM0
// An example user program that initializes the simple operating system
//   Schedule three independent threads using preemptive round robin  
//   Each thread rapidly toggles a pin on Port B and increments its counter 
//   TIMESLICE is how long each thread runs

// Jonathan Valvano
// June 8, 2025

/* 
 Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include "osSimple.h"
#include "../inc/Clock.h"
#include "../inc/TExaS.h"
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#define TIMESLICE  (TIME_2MS)    // thread switch time in system time units
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
uint32_t Count0;   // number of times Task0 loops
uint32_t Count1;   // number of times Task1 loops
uint32_t Count2;   // number of times Task2 loops
uint32_t Count3;   // number of times Task3 loops

void Task0(void){
  Count0 = 0;
  for(;;){
    Count0++;
    GPIOA->DOUTTGL31_0 = 0x01;      // toggle PA0
    Clock_Delay(1000);
   }
}
void Task1(void){
  Count1 = 0;
  for(;;){
    Count1++;
    GPIOA->DOUTTGL31_0 = 0x02;      // toggle PA1
    Clock_Delay(1000);
  }
}
void Task2(void){
  Count2 = 0;
  for(;;){
    Count2++;
    GPIOA->DOUTTGL31_0 = 0x04;      // toggle PA2
    Clock_Delay(1000);
  }
}
void Task3(void){
  Count3 = 0;
  for(;;){
    Count3++;
    GPIOA->DOUTTGL31_0 = 0x08;      // toggle PA3
    Clock_Delay(1000);
  }
}
void PortA_Init(void){
 IOMUX->SECCFG.PINCM[PA0INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PA1INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PA2INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PA3INDEX] = (uint32_t) 0x00000081;
  GPIOA->DOE31_0 |= 0x0F;
}
int main(void){
  OS_DisableInterrupts();
  LaunchPad_Init(); 
  Clock_Init80MHz(0);         // set processor clock to 80 MHz
  TExaS_Init(0,0,TExaS_PA60Logic);
  OS_Init();           // initialize OS
  PortA_Init();
  OS_AddThreads(&Task0, &Task1, &Task2, &Task3);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}
