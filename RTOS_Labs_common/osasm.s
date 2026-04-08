/* ***************************************************************************
; osasm.s: low-level OS commands, written in assembly                       
; derived from uCOS-II                                                     
;****************************************************************************
;OS Lab2/3/4/5/6 
;Students will implement these functions as part of ECE445M Lab
;Starter to labs 1,2,3,4
; Jonathan Valvano
; December 21, 2025
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
     .global  RunPt            // currently running thread
     .global  NextThreadPt     // next thread to run, set by schedule
     .global  Scheduler
     .global  SysTick_Handler 
     .global  TimeMs
     .global  OS_MsTime
     .global  OS_ClearMsTime
     .global  Scheduler
     .global  ReadyLists
     .global  RunPts
     .global  StartOS
     .global  ContextSwitch2
     .global  PendSV_Handler
     .global  OSDisableInterrupts
     .global  OSEnableInterrupts 
     .global  SVC_Handler
     .global  StartCritical
     .global  EndCritical 

     .include "../inc/msp.s" // For profiling
     .macro TogglePB22
        LDR R0, =GPIOB_DOUTTGL31_0
        LDR R1, =(1 << 22)
        STR R1, [R0]
     .endm

     .equ POINTERSIZE, 4 // bytes
     .equ TCB_NEXT, 4 // offset within struct

OSDisableInterrupts:
        CPSID   I
        BX      LR

OSEnableInterrupts:
        CPSIE   I
        BX      LR

// *********** StartCritical ************************
//make a copy of previous I bit, disable interrupts
// inputs :  none
// outputs : R0=previous I bit
StartCritical:
        MRS    R0, PRIMASK  // save old status
        CPSID  I           // mask all (except faults)
        BX     LR

// *********** EndCritical ************************
// using the copy of previous I bit, restore I bit to previous value
// inputs :  R0=previous I bit
// outputs : none
EndCritical:
        MSR    PRIMASK, R0
        BX     LR



// ******** OS_ClearMsTime ************
// sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works
OS_ClearMsTime:
        PUSH {LR}
        BL StartCritical // Save I bit
        LDR R2, =TimeMs
        MOVS R1, #0
        STR R1, [R2]
        BL EndCritical  // Restore I bit
        LDR R0, =10000  //period
        MOVS R1, 4      //prescale
        MOVS R2, 1      //priority
        BL TimerG8_IntArm //40MHz Clock - will arm @ 1kHz
        POP {PC}

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
OS_MsTime:
        LDR R0, =TimeMs
        LDR R0, [R0]
        BX LR
        
// Scheduler handles the algorithm for switching threads
Scheduler:
        MOVS R0, #0 // priorityReady
        LDR R1, =ReadyLists
readyListLoop:  // Traverse ReadyLists array looking for highest priority thread that is ready
        LDR R2, [R1, R0]
        CMP R2, #0
        BNE readyFound
        ADDS R0, R0, #POINTERSIZE // Keep looking if NULL, will break if no ready threads
        B readyListLoop
readyFound:
        LDR R1, =RunPts
        ADDS R1, R0 // R1 = RunPts + priorityReady Pointer to TCB pointer
        LDR R0, [R1] // R0 = RunPts[priorityReady] Pointer to actual TCB
        LDR R0, [R0, #TCB_NEXT] // R0 = RunPts[priorityReady]->next
        LDR R2, =RunPt
        STR R0, [R2] // RunPt = RunPts[priorityReady]->next
        STR R0, [R1] // RunPts[priorityReady] = RunPt
        BX LR

StartOS:
// put your code here
        LDR R0, =RunPt // Pointer to currently running thread
        LDR R1, [R0]   // R1 = value of RunPt
        LDR R2, [R1]   // new thread SP
        MOV SP, R2     // Switch stack pointer
        POP {R4-R7}    // Restore R4-R7
        POP {R0-R3}    // Restore R0-R3
        ADD SP, SP, #8 // Discard R12 and LR
        POP {R0}       // start location
        ADD SP, SP, #4 // Discard PSR
        CPSIE I        // Enable interrupts
        BX R0          // Start first thread


OSStartHang:
    B       OSStartHang        // Should never get here


    

/* *******************************************************************************************************
;                                         HANDLE PendSV EXCEPTION
;                                     void PendSV_Handler(void)
;
; Note(s) : 1) PendSV is used to cause a context switch.  This is a recommended method for performing
;              context switches with Cortex-M.  This is because the Cortex-M auto-saves half of the
;              processor context on any exception, and restores same on return from exception.  So only
;              saving of R4-R7 is required and fixing up the stack pointers.  Using the PendSV exception
;              this way means that context saving and restoring is identical whether it is initiated from
;              a thread or occurs due to an interrupt or exception.
;
;           2) Pseudo-code is:
;              a) disable interrupts so context switch is atomic
;              b) Push remaining R4-R7 on stack;
;              c) Save the SP in its TCB, 
;              d) Get next thread to run from the OS.c
;              e) Load SP of next thread into SP 
;              f) Pop R4-R7 from new stack;
;              g) enable interrupts 
;              i) Perform exception return which will restore remaining context.
;
;           3) On entry into PendSV handler:
;              a) The following have been saved on the process stack (by processor):
;                 xPSR, PC, LR, R12, R0-R3
;              b) RunPt         points to the tcb of the task to suspend
;              c) NextThreadPt  points to the tcb of the task to resume
;
;           4) Since PendSV is set to lowest priority in the system, we
;              know that it will only be run when no other exception or interrupt is active, and
;              therefore safe to assume that context being switched out was a user thread and not an ISR.
;
;           5) We will assume (hope) the compiler does not use R8-R11
;
;********************************************************************************************************/
.type PendSV_Handler, %function
// Only ISR running at priority 3 guarantees it interrupts only main/foreground code
PendSV_Handler:
// put your code here
        TogglePB22 // For profiling
        TogglePB22 // For profiling

        CPSID I   // Prevent interrupt during switch
        PUSH {R4-R7}
        LDR R4, =RunPt
        LDR R1, [R4]
        MOV R2, SP
        STR R2, [R1] // Save SP into old thread's TCB
        MOV R5, LR // Preserve LR when calling scheduler
        BL Scheduler // Call scheduler to pick the next thread
        MOV LR, R5 // Restore LR
        LDR R1, [R4] // R1 = RunPt
        LDR R2, [R1] // SP of new thread
        MOV SP, R2
        POP {R4-R7}
        CPSIE I   // Re-enable interrupts

        TogglePB22 // For profiling
   

        BX      LR                 // Exception return will restore remaining context   
    

/* *******************************************************************************************************
;                                         HANDLE SVC EXCEPTION
;                                     void SVC_Handler(void)
;
; Note(s) : SVC is a software-triggered exception to make OS kernel calls from user land. 
;           The function ID to call is encoded in the instruction itself, the location of which can be
;           found relative to the return address saved on the stack on exception entry.
;           Function-call paramters in R0..R3 are also auto-saved on stack on exception entry.
;
;********************************************************************************************************/

.type OS_Id, %function
.type OS_Kill, %function
.type OS_Sleep, %function
.type OS_Time, %function
.type OS_AddThread, %function
.type ST7735_Message, %function
        .global    OS_Id
        .global    OS_Kill
        .global    OS_Sleep
        .global    OS_Time
        .global    OS_AddThread
        .global    ST7735_Message
.type SVC_Handler, %function
SVC_Handler:
    PUSH    {R4-R5,LR}

    POP     {R4-R5,PC}
SVCJumpTable:
    .long       OS_Id
    .long       OS_Kill
    .long       OS_Sleep
    .long       OS_Time
    .long       OS_AddThread    
    .long       ST7735_Message


    .end

