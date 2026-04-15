// *************os.c**************
// ECE445M Labs 1, 2, 3, 4, 5 and 6
// Starter to labs 1,2,3,4,5,6
// high level OS functions
// Students will implement these functions as part of Lab
// Runs on MSPM0
// Jonathan W. Valvano 
// January 10, 2026, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include <ti/devices/msp/msp.h>
#include "file.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Timer.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/RTOS_UART.h"
#include "../RTOS_Labs_common/SPI.h"
#include "../RTOS_Labs_common/ST7735_SDC.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Labs_common/heap.h"
#include "bump.h"

// Hardware interrupt priorities
//   Priority 0: Periodic threads 
//   Priority 1: Input/output interrupts like UART and edge triggered 
//   Priority 2: 1000 Hz periodic event to implement OS_MsTime and sleep using TimerG8
//   Priority 2: SysTick for running the scheduler
//   Priority 3: PendSV for context switch 

// *****************Timers******************
// SysTick for running the scheduler
// Use TimerG0 is used for SDC timeout
// Use TimerG7 for background periodic threads
// Use TimerG8 is interrupts at 1000Hz to implement OS_MsTime, and sleeping
// Use TimerG12 for 32-bit OS_Time, free running (no interrupts)
// Use TimerA0 for PWM outputs to motors
// Use TimerA1 for PWM outputs to motors
// Use TimerG6 for Lab 1 and then for PWM to servo steering

/* Improvements to make (ask Devin)
    * Re-write all simple C functions in assembly
    * Create "kernel requests" that will handle critical sections in PendSV
    * Maybe look into using SVC?
*/

void OSDisableInterrupts(void);
void OSEnableInterrupts(void);
long StartCritical(void);
void EndCritical(long);
#define  OSCRITICAL_ENTER() { sr=StartCritical(); }
#define  OSCRITICAL_EXIT()  { EndCritical(sr); }

uint32_t TimeMs; // in ms
uint32_t SysTickStart,SysTickElapsed; // execution time of SysTick_Handler in 12.5ns units

TCB_t tcbs[MAXNUMTHREADS];
int32_t Stacks[MAXNUMTHREADS][STACKSIZE];
TCB_t* RunPt; // Can't be static since osasm.s needs it
TCB_t* ReadyLists[NUMPRIORITIES]; // Linked list heads for each priority
TCB_t* RunPts[NUMPRIORITIES]; // Keeping track of last run thread for each priority
TCB_t* Sleeping; // Linked list of all sleeping threads
TCB_t* Free; // Linked list of all free TCBs (makes addThread faster)
Event_t PeriodicTasks[MAXEVENTS];
Mailbox_t OS_Mailbox;
Fifo_t OS_Fifo;

uint8_t crashed;

void OS_ClearMsTime(void); // implemented in osasm.s

uint32_t OS_MsTime(void); // implemented in osasm.s

void StartOS(void); // implemented in osasm.s

#define bump_right_pin  (1<<31)
#define bump_left_pin   (1<<27)
#define bump_center_pin (1<<28)
#define bump_pins (bump_right_pin | bump_center_pin | bump_left_pin) // for bump swithces

#define TogglePB20() (GPIOB->DOUTTGL31_0 = (1<<20))

//------------------------------------------------------------------------------
//  Systick Interrupt Handler
//  SysTick interrupt happens every 2 ms
// used for preemptive foreground thread switch
// ------------------------------------------------------------------------------
void SysTick_Handler(void) {
  SysTickStart = OS_Time();
  SCB->ICSR = 0x10000000; // Invoke PendSV for context switch
  SysTickElapsed = OS_Time() - SysTickStart;
} // end SysTick_Handler

// Scheduler handles the algorithm for switching threads
void Scheduler(void); // implemented in osasm.s

uint32_t OS_LockScheduler(void){
 uint32_t old = SysTick->CTRL;
  SysTick->CTRL= 5;
  return old;
}
void OS_UnLockScheduler(uint32_t previous){
  SysTick->CTRL = previous;
}


//
//@details  Initialize operating system, disable interrupts until OS_Launch.
//Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
// Interrupts not yet enabled.
 // @param  none
 // @return none
 //@brief  Initialize OS
//
void OS_Init(void){
  // put Lab 2 (and beyond) solution here

  // Initialize all tcbs as free
  for (int threadID = 0; threadID < MAXNUMTHREADS; threadID++){
    tcbs[threadID].state = FREE;
    if (threadID == MAXNUMTHREADS-1){
      tcbs[threadID].next = NULL;
    }
    else{
      tcbs[threadID].next = &tcbs[threadID+1];
    }
    // set thread ID
    tcbs[threadID].id = threadID;
  }

  // Start Free linked list
  Free = &tcbs[0];

  // Initialize all ready lists as empty and last ran as NULL
  for (int priority = 0; priority < NUMPRIORITIES; priority++){
    ReadyLists[priority] = NULL; 
    RunPts[priority] = NULL;
  }

  // Initialize RunPt to NULL, indicating no threads yet
  RunPt = NULL;

  // Initialize sleeping to NULL (blocked lists are per-semaphore, initialized in OS_InitSemaphore)
  Sleeping = NULL;

  // Initialize periodic threads to NULL
  for (int i = 0; i < MAXEVENTS; i++){
    PeriodicTasks[i].task = NULL;
  }

  // Initalize MsTime
  OS_ClearMsTime();

  // Initialize UART
  UART_Init(1);

  // Initialize time
  TimerG12_Init();

  //Enable Interrupts occurs at OS_LaunchOS
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4_t *semaPt, int32_t value){
  semaPt->Value = value;
  semaPt->blockedList = NULL;
  semaPt->blockedTail = NULL;
}


// Instead of completely disabling interrupts, maybe just prevent scheduler (PRIMASK)

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4_t *semaPt){long sr;
  sr = StartCritical();
  --(semaPt->Value);
  if (semaPt->Value < 0){ // Block this thread
    TCB_t* blockedThread = RunPt;
    blockedThread->blocked = semaPt;

    // Unlink from ready list
    if (blockedThread->next == blockedThread){ // only thread in its priority
      RunPts[blockedThread->priority] = NULL;
      ReadyLists[blockedThread->priority] = NULL;
    } else{
      RunPts[blockedThread->priority] = blockedThread->prev;
      ReadyLists[blockedThread->priority] = blockedThread->next;
    }
    blockedThread->prev->next = blockedThread->next;
    blockedThread->next->prev = blockedThread->prev;

    // Enqueue at tail of this semaphore's blocked list
    blockedThread->next = NULL;
    if (semaPt->blockedList == NULL){ // First waiter on this semaphore
      blockedThread->prev = NULL;
      semaPt->blockedList = blockedThread;
    } else {
      semaPt->blockedTail->next = blockedThread;
      blockedThread->prev = semaPt->blockedTail;
    }
    semaPt->blockedTail = blockedThread;

    OS_Suspend();
  }
  EndCritical(sr);
}


// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4_t *semaPt){long sr;
  sr = StartCritical();
  ++(semaPt->Value);
  if (semaPt->Value <= 0){ // wakeup oldest waiting thread
    TCB_t* wakeupThread = semaPt->blockedList;

    // Dequeue from head of this semaphore's blocked list
    semaPt->blockedList = wakeupThread->next;
    if (semaPt->blockedList != NULL){
      semaPt->blockedList->prev = NULL;
    } else {
      semaPt->blockedTail = NULL;
    }
    wakeupThread->blocked = NULL;

    // Link to ready list
    if (ReadyLists[wakeupThread->priority] == NULL){ // Handle empty list
      ReadyLists[wakeupThread->priority] = wakeupThread;
      wakeupThread->next = wakeupThread;
      wakeupThread->prev = wakeupThread;
      RunPts[wakeupThread->priority] = wakeupThread;
    } else { // Normal reinsertion: insert just before RunPts (runs last in this round)
      RunPts[wakeupThread->priority]->prev->next = wakeupThread;
      wakeupThread->prev = RunPts[wakeupThread->priority]->prev;
      RunPts[wakeupThread->priority]->prev = wakeupThread;
      wakeupThread->next = RunPts[wakeupThread->priority];
    }

    if (RunPt->priority > wakeupThread->priority){
      OS_Suspend(); // Context switch if we're waking up a higher priority thread
    }
  }
  EndCritical(sr);
}

// ******** OS_bWait ************ Could be done in assembly (store only)
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4_t *semaPt){
  long sr = StartCritical();
  if (semaPt->Value == 0){ // Resource taken, block this thread
    TCB_t* blockedThread = RunPt;
    blockedThread->blocked = semaPt;

    // Unlink from ready list
    if (blockedThread->next == blockedThread){ // only thread in its priority
      RunPts[blockedThread->priority] = NULL;
      ReadyLists[blockedThread->priority] = NULL;
    }
    else{
      RunPts[blockedThread->priority] = blockedThread->prev;
      ReadyLists[blockedThread->priority] = blockedThread->next;
    }
    blockedThread->prev->next = blockedThread->next;
    blockedThread->next->prev = blockedThread->prev;

    // Enqueue at tail of this semaphore's blocked list
    blockedThread->next = NULL;
    if (semaPt->blockedList == NULL){ // First waiter on this semaphore
      blockedThread->prev = NULL;
      semaPt->blockedList = blockedThread;
    }
    else{
      semaPt->blockedTail->next = blockedThread;
      blockedThread->prev = semaPt->blockedTail;
    }
    semaPt->blockedTail = blockedThread;

    OS_Suspend(); // Returns here when OS_bSignal wakes this thread
  }
  semaPt->Value = 0; // Acquire: mark semaphore as taken
  EndCritical(sr);
}

// ******** OS_bSignal ************ Could be done in assembly (store only)
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4_t *semaPt){
  long sr = StartCritical();
  if (semaPt->blockedList != NULL){ // Someone is waiting on this semaphore
    TCB_t* wakeupThread = semaPt->blockedList; // oldest waiter

    // Dequeue from head of this semaphore's blocked list
    semaPt->blockedList = wakeupThread->next;
    if (semaPt->blockedList != NULL){
      semaPt->blockedList->prev = NULL;
    }
    else{
      semaPt->blockedTail = NULL;
    }
    wakeupThread->blocked = NULL;

    // Link to ready list
    if (ReadyLists[wakeupThread->priority] == NULL){ // Handle empty list
      ReadyLists[wakeupThread->priority] = wakeupThread;
      wakeupThread->next = wakeupThread;
      wakeupThread->prev = wakeupThread;
      RunPts[wakeupThread->priority] = wakeupThread;
    }
    else{ // Normal reinsertion
      RunPts[wakeupThread->priority]->prev->next = wakeupThread;
      wakeupThread->prev = RunPts[wakeupThread->priority]->prev;
      RunPts[wakeupThread->priority]->prev = wakeupThread;
      wakeupThread->next = RunPts[wakeupThread->priority];
    }

    if (RunPt->priority > wakeupThread->priority){
      OS_Suspend(); // Context switch if we're waking up a higher priority thread
    }
  }
  else{
    semaPt->Value = 1; //release the resource
  }
  EndCritical(sr);
}



// ******** OS_AddThread *************** 
// add a foreground thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 255 is the lowest
// Priorities are relative to other foreground threads
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
// In Lab 4, the stackSize can be 128, 256, or 512 bytes

int OS_AddProcessThread(void(*task)(void), 
   uint32_t stackSize, uint32_t priority, uint32_t pid){
	   return 0;
   }

static void inline setInitialStack(int threadID){
  tcbs[threadID].sp = &Stacks[threadID][STACKSIZE-12]; // Thread stack pointer
  Stacks[threadID][STACKSIZE-1] =  0x01000000; // Thumb bit
  Stacks[threadID][STACKSIZE-3] =  0x14141414; // R14
  Stacks[threadID][STACKSIZE-4] =  0x12121212; // R12
  Stacks[threadID][STACKSIZE-5] =  0x33333333; // R3
  Stacks[threadID][STACKSIZE-6] =  0x22222222; // R2
  Stacks[threadID][STACKSIZE-7] =  0x11111111; // R1
  Stacks[threadID][STACKSIZE-8] =  0x00000000; // R0
  Stacks[threadID][STACKSIZE-9] =  0x77777777; // R7
  Stacks[threadID][STACKSIZE-10] = 0x66666666; // R6
  Stacks[threadID][STACKSIZE-11] = 0x55555555; // R5
  Stacks[threadID][STACKSIZE-12] = 0x44444444; // R4
}

int OS_AddThread(void(*task)(void), 
   uint32_t stackSize, uint32_t priority){ 

  // put Lab 2 (and beyond) solution here
  if (priority >= NUMPRIORITIES){
    return 0; // Not a valid priority
  }
  
  int threadID = 0;
  long sr = StartCritical();
  // Find the first free tcb (critical section)
  if (Free != NULL){
    threadID = Free->id;
    Free = Free->next;
  }
  else{
    return 0; // No free tcb
  }
  EndCritical(sr);

  // Mark thread as active
  tcbs[threadID].state = ACTIVE;

  // Mark thread as not sleeping
  tcbs[threadID].sleep = 0;

  // Mark thread as not blocked
  tcbs[threadID].blocked = NULL;

  // set thread priority
  tcbs[threadID].priority = priority;

  // Initialize the stack for this thread
  setInitialStack(threadID);

  // Set PC to point to task
  Stacks[threadID][STACKSIZE-2] = (int32_t)task;

  // Add to ready list (critical section)
  sr = StartCritical();
  if (ReadyLists[priority] == NULL){ // Handle case of empty ready list for that priority
    ReadyLists[priority] = &tcbs[threadID];
    tcbs[threadID].next = &tcbs[threadID];
    tcbs[threadID].prev = &tcbs[threadID];
    RunPts[priority] = &tcbs[threadID];
  }
  else{ // Link new tcb into its proper linked list
    tcbs[threadID].next = RunPts[priority];
    tcbs[threadID].prev = RunPts[priority]->prev;
    tcbs[threadID].prev->next = &tcbs[threadID];
    RunPts[priority]->prev = &tcbs[threadID];
  }

  if (RunPt == NULL){ // Handle case of first ever thread
    RunPt = &tcbs[threadID];
  }

  EndCritical(sr);

  return 1;
}

// ******** OS_AddProcess *************** 
// add a process with foregound thread to the scheduler
// Inputs: pointer to process text (code) segment, entry point at top
//         pointer to process data segment
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this process can not be added
// This function will be needed for Lab 5
// In Labs 2-4, this function can be ignored
int OS_AddProcess(void *text, void *data, uint32_t stackSize, uint32_t priority){ 
  
  return 0;
}


int OS_LoadProgram(char *name, uint32_t priority){
  
  return 0;
}



// ******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  // put Lab 2 (and beyond) solution here
  return RunPt->id;
}



uint32_t lcm2(uint32_t n1,uint32_t n2){
  uint32_t n;
  if(n1 > n2){
    n = n1;
  }else{
    n = n2;
  }
  while( ((n % n1) != 0) || ((n % n2) != 0) ){
    n++;
  }
  return n;
}

uint32_t lcm3(uint32_t n1,uint32_t n2,uint32_t n3){
  return lcm2(lcm2(n1,n2),n3);
}
uint32_t lcm4(uint32_t n1,uint32_t n2,uint32_t n3,uint32_t n4){
  return lcm2(lcm2(n1,n2),lcm2(n3,n4));
}
uint32_t lcm5(uint32_t n1,uint32_t n2,uint32_t n3,uint32_t n4,uint32_t n5){
  return lcm2(lcm2(n1,n2),lcm3(n3,n4,n5));
}

//******** OS_AddPeriodicThread *************** 
// Add a background periodic thread
// typically this function receives the highest priority
// Inputs: task is pointer to a void/void background function
//         period in ms
//         priority 0 is the highest, 3 is the lowest
// Priorities are relative to other background periodic threads
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the resolution of period
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 3, this command will be called 0 to 4 times
// In labs 3-7, there will be 0 to 4 background periodic threads, and this priority field 
//           determines the relative priority of these threads
// For Lab 3, it ok to make reasonable limits to reduce the complexity. E.g.,
//  - You can assume there are 0 to 4 background periodic threads
//  - You can assume the priorities are sequential 0,1,2,3,4
//  - You can assume a maximum thread execution time, e.g., 50us
//  - You can assume a slowest period, e.g., 50ms
//  - You can limit possible periods, e.g., 1,2,4,5,10,20,25,50ms
//  - You can assume (E0/T0)+(E1/T1)+(E2/T2)+(E3/T3) is much less than 1 

static uint8_t numRegistered = 0; // unique tasks added by user
uint8_t numPeriodic = 0;          // total schedule entries (set by builder)
uint8_t ThisTask = 0;

// ******** OS_BuildTimeline ************
// Rebuild the aperiodic schedule from registered periodic tasks.
// Each task gets a sub-ms offset = priority * 100us within its ms slot.
// With max 4 tasks at offsets 0, 100, 200, 300us, no two tasks ever
// collide, so jitter is limited to ISR entry latency (~30 cycles).
// Fills PeriodicTasks[] with the expanded timeline and sets numPeriodic.
// This routine was written by Claude Opus 4.6
#define TASK_SPACING_US 200
static void OS_BuildTimeline(void){
  if (numRegistered == 0) return;

  // Save registration data to stack before overwriting the array
  Event_t reg[4];
  for (int i = 0; i < numRegistered; i++){
    reg[i] = PeriodicTasks[i];
  }

  // Compute LCM of all registered periods (in ms)
  uint32_t lcm = reg[0].period;
  for (int i = 1; i < numRegistered; i++){
    lcm = lcm2(lcm, reg[i].period);
  }

  uint32_t count = 0;
  uint32_t prevTime = 0;
  uint32_t firstTime = 0;

  // Generate schedule entries in sorted time order:
  //   outer loop over ms ticks ensures ascending time
  //   inner loop over priority 0..3 ensures higher priority fires first
  for (uint32_t ms = 0; ms < lcm; ms++){
    for (uint32_t p = 0; p <= 3; p++){
      // Find the registered task at this priority
      int idx = -1;
      for (int i = 0; i < numRegistered; i++){
        if (reg[i].priority == p){ idx = i; break; }
      }
      if (idx < 0) continue;

      // Does this task fire on this ms tick?
      if (ms % reg[idx].period != 0) continue;

      if (count >= MAXEVENTS) return; // overflow guard

      uint32_t time_us = ms * 1000 + p * TASK_SPACING_US;

      PeriodicTasks[count].task = reg[idx].task;
      PeriodicTasks[count].priority = reg[idx].priority;
      PeriodicTasks[count].period = reg[idx].period;

      // Set timeToNext for the previous entry (delay from prev to this)
      if (count > 0){
        PeriodicTasks[count - 1].timeToNext = time_us - prevTime;
      } else {
        firstTime = time_us;
      }

      prevTime = time_us;
      count++;
    }
  }

  // Last entry wraps to first entry of the next LCM cycle
  if (count > 0){
    uint32_t lcm_us = lcm * 1000;
    PeriodicTasks[count - 1].timeToNext = lcm_us - prevTime + firstTime;
  }

  numPeriodic = count;
  ThisTask = 0;
}

int OS_AddPeriodicThread(void(*task)(void),
   uint32_t period, uint32_t priority){
  // put Lab 2 (and beyond) solution here

  if (priority > 3 || period == 0 || numRegistered >= 4){
    return 0;
  }

  if (numRegistered == 0){
    // Intialize Timer G7 for periodic thread management
    TimerG7_IntArm(10000, 80, 0); // High priority
  }

  // Store registration info (builder reads from first numRegistered entries)
  PeriodicTasks[numRegistered].task = task;
  PeriodicTasks[numRegistered].period = period;
  PeriodicTasks[numRegistered].priority = priority;
  numRegistered++;

  // Rebuild schedule with the new task included
  OS_BuildTimeline();
  return 1;
}

void TIMG7_IRQHandler(void){
  if((TIMG7->CPU_INT.IIDX) == 1){ // this will acknowledge
    (*PeriodicTasks[ThisTask].task)();   // execute user task
    ThisTask = ThisTask+1;
    if (ThisTask == numPeriodic){
      ThisTask = 0; // wrap around
    }
    TIMG7->COUNTERREGS.LOAD  = PeriodicTasks[ThisTask].timeToNext-1;    // set reload register
  }
}  

void TIMG8_IRQHandler(void){
  if((TIMG8->CPU_INT.IIDX) == 1){ // this will acknowledge
    ++TimeMs;
    // Decrement all TCB sleep counters
    TCB_t* thread = Sleeping;
    while (thread != NULL){
      TCB_t* nextSleeper = thread->next; // Need to save in case of re-linkage
      --(thread->sleep);

      if (thread->sleep == 0){ // Time to wake up
        // Unlink from sleeping list
        if (thread->prev != NULL){
          thread->prev->next = thread->next;
        }
        if (thread->next != NULL){
          thread->next->prev = thread->prev;
        }
      
        // Link to ready list
        if (ReadyLists[thread->priority] == NULL){ // Handle empty list
          ReadyLists[thread->priority] = thread;
          thread->next = thread;
          thread->prev = thread;
          RunPts[thread->priority] = thread;
        }
        else{ // Normal reinsertion
          RunPts[thread->priority]->prev->next = thread;
          thread->prev = RunPts[thread->priority]->prev;
          RunPts[thread->priority]->prev = thread;
          thread->next = RunPts[thread->priority];
        }

        if (thread == Sleeping){
          Sleeping = nextSleeper;
        }
        
        if (RunPt->priority > thread->priority){
          OS_Suspend(); // Context switch if we're waking up a higher priority thread
        }
      }
      thread = nextSleeper;
    }
  }
}





//----------------------------------------------------------------------------
//  Edge triggered Interrupt Handler
//  Rising edge of PA18 (S1) 
//  Falling edge of PB21 (S2)
//  Falling edge of PA27 (bump)
//  Falling edge of PA28 (bump)
//  Falling edge of PA31 (bump)
//----------------------------------------------------------------------------
void (*S2Task)(void) = NULL;
void (*PA28Task)(void) = NULL;
void (*BumpTask)(bump_status) = NULL;
uint32_t BumpStatus; // WiFi logging global in RTOS_MotorBoard.c
void GROUP1_IRQHandler(void){
  // write this
  TogglePB20();
  if(GPIOA->CPU_INT.RIS&(1<<18)){ // PA18
    GPIOA->CPU_INT.ICLR = 1<<18;
    
  }
  uint32_t flags = GPIOA->CPU_INT.RIS;
  if(flags & bump_pins){
    bump_status status = bump_none;
    if(flags & bump_left_pin)   status |= bump_left;
    if(flags & bump_center_pin) status |= bump_center;
    if(flags & bump_right_pin)  status |= bump_right;
    
    GPIOA->CPU_INT.ICLR = (flags & bump_pins);
    BumpStatus = (uint32_t)status;

    if(BumpTask != NULL){
      BumpTask(status);
    } else {
      crashed =1;
      bump_collision();
    
    }
  }
  //if(GPIOA->CPU_INT.RIS&(1<<28)){ // PA28
    //GPIOA->CPU_INT.ICLR = 1<<28;
  // PA28Task();
   
  //}
  if(GPIOB->CPU_INT.RIS&(1<<21)){ // PB21
    GPIOB->CPU_INT.ICLR = 1<<21;
    //GPIOB->DOUTTGL31_0 = (1<<20); // For profiling
    S2Task(); // Assume task WILL be initialized
  }
}
// ******** OS_AddS1Task *************** 
// add a background task to run whenever the S1 (PA18) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 1 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// Because of the pin conflict with TFLuna, this command will not be called 
int OS_AddS1Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
  
  return 0; // replace this line with solution
};


// ******** OS_AddS2Task *************** 
// add a background task to run whenever the S2 (PB21) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 1 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function will be called 0 or 1 times
// In lab 3, this function will be called will be called 0 or 1 times
// In lab 3, there will be many background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddS2Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here

  if (priority > 1){
    return 0;
  }

  // From EdgeTriggered.c
  GPIOB->POLARITY31_16 |= 0x00000800;     // falling
  GPIOB->CPU_INT.ICLR = 0x00200000;   // clear bit 21
  GPIOB->CPU_INT.IMASK = 0x00200000;  // arm PB21
  NVIC->IP[0] = (NVIC->IP[0]&(~0x0000FF00))|priority<<14;    // set priority (bits 15,14) IRQ 1
  NVIC->ISER[0] = 1 << 1; // Group1 interrupt

  S2Task = task;
  
  return 1;
}

// ******** OS_AddPA28Task *************** 
// add a background task to run whenever the bump (PA28) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 1 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 3, this command will be called 0 or 1 times
// In lab 2, not implemented
// In lab 3, there will be many background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPA28Task(void(*task)(void), uint32_t priority){
  // put Lab 3 (and beyond) solution here
  if (priority > 1){
    return 0;
  }

  // Initialize PA28
  IOMUX->SECCFG.PINCM[PA28INDEX] = (uint32_t) 0x00060081; // input, pull up

  // From EdgeTriggered.c
  GPIOA->POLARITY31_16 |= (1<<25);     // falling
  GPIOA->CPU_INT.ICLR = (1 << 28);   // clear bit 28
  GPIOA->CPU_INT.IMASK = (1<< 28); // Arm PA28
  // These were probably already done by S2 task but we'll just do it again
  NVIC->IP[0] = (NVIC->IP[0]&(~0x0000FF00))|priority<<14;    // set priority (bits 15,14) IRQ 1
  NVIC->ISER[0] = 1 << 1; // Group1 interrupt

  PA28Task = task;
  
  return 1;
};



// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  // put Lab 2 (and beyond) solution here

  TCB_t* thread = RunPt;
  
  thread->sleep = sleepTime; //set time (in ms) to sleep for
  if (sleepTime == 0){ // just suspend
    OS_Suspend();
  }
  else{
    long sr = StartCritical();
    // Handle case where this is the only thread in that priority
    if (thread->next == thread){
      RunPts[thread->priority] = NULL;
      ReadyLists[thread->priority] = NULL;
    }
    else{
      RunPts[thread->priority] = thread->prev;
      ReadyLists[thread->priority] = thread->next;
    }

    // Unlink from ready list
    thread->prev->next = thread->next;
    thread->next->prev = thread->prev;

    // Link to sleeping list
    if (Sleeping != NULL){
      Sleeping->prev = thread;
    }

    thread->next = Sleeping;
    thread->prev = NULL;
    Sleeping = thread;

    OS_Suspend(); //suspend current thread
    EndCritical(sr);
  }
} 

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  // put Lab 2 (and beyond) solution here

  long sr = StartCritical();
  TCB_t* threadKilled = RunPt;
  // Handle case where this is the only thread in that priority
  if (threadKilled->next == threadKilled){
    RunPts[threadKilled->priority] = NULL;
    ReadyLists[threadKilled->priority] = NULL;
  }
  else{
    RunPts[threadKilled->priority] = threadKilled->prev; // Set this for scheduler
    ReadyLists[threadKilled->priority] = threadKilled->next; // Doesn't really matter as long as it doesn't point to killed thread
  }

  // Unlink from ready list
  threadKilled->prev->next = threadKilled->next;
  threadKilled->next->prev = threadKilled->prev;
  
  threadKilled->state = FREE; //label killed thread as free
  // Add to free list
  threadKilled->next = Free;
  Free = threadKilled;
  
  OS_Suspend(); // Will clear SysTick and pend PendSV

  EndCritical(sr);


  for(;;){};        // can not return (must return in Lab 5 since called from SVC_hander)
   
}; 



// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  // put Lab 2 (and beyond) solution here
  SysTick->VAL = 0; // Clear Systick
  SCB->ICSR = 0x10000000; // Invoke PendSV

};
  



// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128,256
void OS_Fifo_Init(uint32_t size){
  // put Lab 2 (and beyond) solution here
  
  OS_Fifo.PutI = OS_Fifo.GetI = 0; // Initialize put and get indices
  OS_InitSemaphore(&OS_Fifo.CurrentSize, 0); // Semaphore used to synchronize threads
  OS_Fifo.MaxSize = size;
  OS_Fifo.LostData = 0;
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
  // put Lab 2 (and beyond) solution here
  
  if (OS_Fifo.CurrentSize.Value == OS_Fifo.MaxSize){
    OS_Fifo.LostData++;
    return 0; // Not saved, full
  }
  else{
    OS_Fifo.data[OS_Fifo.PutI] = data; // Save data
    OS_Fifo.PutI = (OS_Fifo.PutI + 1) % OS_Fifo.MaxSize; // PutI wraps around
    OS_Signal(&OS_Fifo.CurrentSize);
    return 1; // Success
  }

}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){ uint32_t data;
  // put Lab 2 (and beyond) solution here
   
   OS_Wait(&OS_Fifo.CurrentSize); // Spin if empty
   data = OS_Fifo.data[OS_Fifo.GetI]; // Get data
   OS_Fifo.GetI = (OS_Fifo.GetI + 1) % OS_Fifo.MaxSize; // Wraps around
   return data;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  // put Lab 2 (and beyond) solution here
  return OS_Fifo.CurrentSize.Value;
}
// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
  // put Lab 2 (and beyond) solution here
  OS_Mailbox.data = 0;
  OS_Mailbox.free.Value = 1;
  OS_Mailbox.dataValid.Value = 0;
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data){
  // put Lab 2 (and beyond) solution here
  OS_bWait(&OS_Mailbox.free);
  OS_Mailbox.data = data;
  OS_bSignal(&OS_Mailbox.dataValid);
};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void){
  // put Lab 2 (and beyond) solution here
   OS_bWait(&OS_Mailbox.dataValid);
   uint32_t data = OS_Mailbox.data;
   OS_bSignal(&OS_Mailbox.free);
   return data;
};

// ******** OS_Time ************
// return the system time, counting up 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void){
  // put Lab 2 (and beyond) solution here
    return ~(TIMG12->COUNTERREGS.CTR);
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
  // put Lab 2 (and beyond) solution here
    return stop - start;
};



// ******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
// make PendSV priority 3, and SysTick priority 2
void OS_Launch(uint32_t theTimeSlice){
  // put Lab 2 (and beyond) solution here
   SysTick->CTRL = 0; // Disable SysTick during setup
   SysTick->VAL = 0; // Clear
   SCB->SHP[1] = (SCB->SHP[1] & (~0xC0000000)) | (2<<30) | (3<<22); // Priority 2 for SysTick, 3 for PendSV
   SysTick->LOAD = theTimeSlice - 1; // reload value
   SysTick->CTRL = 0x7; // Re-enable, core clock and interrupt arm
   StartOS();
}
