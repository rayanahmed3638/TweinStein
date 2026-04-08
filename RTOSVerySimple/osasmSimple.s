/* ****************************************************************************
; osasmSimple.s: low-level OS commands, written in assembly                       *
; Runs on MSPM0
; A very simple real time operating system with minimal features.
; Jonathan Valvano
; June 12, 2025

;
;Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu
;    You may use, edit, run or distribute this file
;    as long as the above copyright notice remains
; THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
; OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; For more information about my classes, my research, and my books, see
; http://users.ece.utexas.edu/~valvano/
; 
*/

         .data
        .align 2
// Declare global variables here if needed
// with the .space assembly directive

        .text
        .thumb
        .align 2

        .global  RunI            // currently running thread
        .global  tcbs            // array of TCBs
        .global  OS_DisableInterrupts
        .global  OS_EnableInterrupts
        .global  StartOS
        .global  SysTick_Handler


OS_DisableInterrupts:
        CPSID   I
        BX      LR


OS_EnableInterrupts:
        CPSIE   I
        BX      LR

.type SysTick_Handler, %function
SysTick_Handler:       // 1) Saves R0-R3,R12,LR,PC,PSR
    CPSID   I          // 2) Prevent interrupt during switch
    PUSH    {R4-R7}    // 3) Save remaining regs r4-r7
    LDR     R0,=RunI   // 4) R0=pointer to RunI, old thread
    LDR     R1,[R0]    //    R1 = RunI
    LSLS    R2,R1,#2   // R2 = RunI*4
    LDR     R3,=tcbs   // points to tcbs
    ADDS    R4,R3,R2   // points to tcbs[RunI]
    MOV     R5,SP
    STR     R5,[R4]    // 5) Save SP into TCB
    ADDS    R1,#1      // 6) R1 = RunI+1
    MOVS    R6,#3
    ANDS    R1,R6      // wrap 4 to 0
    STR     R1,[R0]    // RunI = R1
    LSLS    R2,R1,#2   // R2 = RunI*4
    ADDS    R4,R3,R2   // points to tcbs[RunI]
    LDR     R5,[R4]    // 7) new thread SP  SP = tcbs[RunI]
    MOV     SP,R5
    POP     {R4-R7}    // 8) restore regs r4-r7
    CPSIE   I          // 9) tasks run with interrupts enabled
    BX   LR            // 10) restore R0-R3,R12,LR,PC,PSR



StartOS:
    LDR     R0, =RunI  // 4) R0=pointer to RunI, old thread
    LDR     R1, [R0]   //    R1 = RunI
    LSLS    R2,R1,#2   // R2 = RunI*4
    LDR     R3,=tcbs   // points to tcbs
    ADD     R3,R2      // points to tcbs[RunI]
    LDR     R5, [R3]   // new thread SP  SP = tcbs[RunI]
    MOV     SP,R5
    POP     {R4-R7}    // restore regs r4-7
    POP     {R0-R3}    // restore regs r0-3
    ADD     SP,#4      // discard R12 
    ADD     SP,#4      // discard LR from initial stack
    POP     {R0}       // start location
    ADD     SP,#4      // discard PSR
    CPSIE   I          // Enable interrupts at processor level
    BX      R0         // start first thread


    .end
