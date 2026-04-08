// os.c
// Runs on LM4F120/TM4C123
// A very simple real time operating system with minimal features.
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

#include <ti/devices/msp/msp.h>
#include "../inc/Clock.h"




#define STACKSIZE 100
uint32_t *tcbs[4];  
uint32_t RunI;  // index of currently running thread (0,1,2,3)
uint32_t Stacks[4][STACKSIZE]; 
void OS_AddThreads(void(*task0)(void), void(*task1)(void),
                 void(*task2)(void), void(*task3)(void)){ 
  tcbs[0] = &Stacks[0][STACKSIZE-12]; // thread stack pointer 
  Stacks[0][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[0][STACKSIZE-2] = (int32_t)(task0); // PC
  tcbs[1] = &Stacks[1][STACKSIZE-12]; // thread stack pointer 
  Stacks[1][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[1][STACKSIZE-2] = (int32_t)(task1); // PC
  tcbs[2] = &Stacks[2][STACKSIZE-12]; // thread stack pointer 
  Stacks[2][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[2][STACKSIZE-2] = (int32_t)(task2); // PC
  tcbs[3] = &Stacks[3][STACKSIZE-12]; // thread stack pointer 
  Stacks[3][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[3][STACKSIZE-2] = (int32_t)(task3); // PC
  RunI = 0;       // thread 0 will run first
}   


// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// input:  none
// output: none
void OS_Init(void){

  SysTick->CTRL = 0;         // disable SysTick during setup
  SysTick->VAL= 0;      // any write to current clears it
  SCB->SHP[1]    = (SCB->SHP[1]&(~0xC0000000))|(3<<30);  // priority 3
}




///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice){
  SysTick->LOAD = theTimeSlice - 1; // reload value
  SysTick->CTRL = 0x00000007; // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
