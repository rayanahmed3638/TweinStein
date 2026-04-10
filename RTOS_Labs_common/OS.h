/**
 * @file      OS.h
 * @brief     Real Time Operating System for Labs 1, 2, 3, 4 and 5
 * @details   ECE445M
 * @version   V1.0
 * @author    Valvano
 * @copyright Copyright 2026 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      January 10, 2026

 ******************************************************************************/

#ifndef __OS_H
#define __OS_H  1
#include <stdint.h>
/**
 * \brief Set which lab is active so I can use the same OS.c for all Labs
 */ 
#define LAB1 0
#define LAB2 0
#define LAB3 0
#define LAB4 0
#define LAB5 0
#define LAB6 1
/**
 * \brief Times assuming a 80 MHz
 */      
#define TIME_1MS    80000          
#define TIME_2MS    (2*TIME_1MS)  
#define TIME_500US  (TIME_1MS/2)  
#define TIME_250US  (TIME_1MS/4)  

// Thread constants
#define MAXNUMTHREADS 10
#define STACKSIZE 256
#define NUMPRIORITIES 8
#define MAXEVENTS 200

// Fifo size
#define MAXFIFOSIZE 256

// Forward declaration so Sema4 can hold TCB pointers
typedef struct TCB TCB_t;

/**
 * \brief Semaphore structure. Feel free to change the type of semaphore, there are lots of good solutions
 */
struct  Sema4{
  int32_t Value;       // >0 means free, otherwise means busy
  TCB_t*  blockedList; // head of per-semaphore blocked queue (oldest waiter)
  TCB_t*  blockedTail; // tail of per-semaphore blocked queue (newest waiter)
};
typedef struct Sema4 Sema4_t;

// TCB state
typedef enum {FREE, ACTIVE} State_t;
// Thread Control Block structure
struct TCB{
  int32_t*      sp;
  TCB_t*        next;
  TCB_t*        prev;
  State_t       state;
  uint8_t       id;
  uint32_t      sleep;
  uint32_t      priority;
  Sema4_t*      blocked;
};

// Periodic thread structure
typedef struct Event Event_t;
struct Event{
  void (*task)(void);
  uint32_t timeToNext;
  uint32_t period;
  uint32_t priority;
};

// Mailbox structure
typedef struct Mailbox{
  uint32_t data;
  Sema4_t  free;
  Sema4_t  dataValid;
} Mailbox_t;

// Fifo structure
typedef struct Fifo{
  uint32_t PutI;
  uint32_t GetI;
  uint32_t data[MAXFIFOSIZE];
  Sema4_t CurrentSize;
  uint32_t MaxSize;
  uint32_t LostData;
} Fifo_t;

/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */
void OS_Init(void); 

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4_t *semaPt, int32_t value); 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4_t *semaPt); 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4_t *semaPt); 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4_t *semaPt); 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4_t *semaPt); 

//******** OS_AddThread *************** 
// add a foreground thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 255 is the lowest
// Priorities are relative to other foreground threads
// Outputs: 1 if successful, 0 if this thread can not be added
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
// In Lab 4, the stackSize can be 128, 256, or 512 bytes
int OS_AddThread(void(*task)(void), 
   uint32_t stackSize, uint32_t priority);

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void);

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
//  - You can assume the priorities are sequential 0,1,2,3
//  - You can assume a maximum thread execution time, e.g., 50us
//  - You can assume a slowest period, e.g., 50ms
//  - You can limit possible periods, e.g., 1,2,4,5,10,20,25,50ms
//  - You can assume (E0/T0)+(E1/T1)+(E2/T2)+(E3/T3) is much less than 1 
int OS_AddPeriodicThread(void(*task)(void), 
   uint32_t period, uint32_t priority);

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
// In lab 2, the priority field can be ignored
// In labs 3-7, there will be many background threads, and this priority field 
//           determines the relative priority of these threads
int OS_AddS1Task(void(*task)(void), uint32_t priority);

// ******** OS_AddS2Task *************** 
// add a background task to run whenever the S2 (PB21) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 1 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In labs 3-7, this command will be called will be called 0 or 1 times
// In labs 3-7, there will be many background threads, and this priority field 
//           determines the relative priority of these threads
int OS_AddS2Task(void(*task)(void), uint32_t priority);

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
int OS_AddPA28Task(void(*task)(void), uint32_t priority);

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime); 

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void); 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void);

// temporarily prevent foreground thread switch (but allow background interrupts)
uint32_t OS_LockScheduler(void);
// resume foreground thread switching
void OS_UnLockScheduler(uint32_t previous);
 
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(uint32_t size);

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data);  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void);

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void);

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void);

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data);

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void);

// ******** OS_Time ************
// return the system time counting up
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void);

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop);

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void);

// ******** OS_MsTime ************
// reads the current time in msec 
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void);

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice);

// Program has these fields
struct Program{
  uint32_t StartOffset; // offset to starting location
  uint32_t CodeSize;    // size of code segment
  uint32_t StackSize;   // size of stack segment
  uint32_t DataSize;    // size of data segment (globals)
  char Name[8];         // ASCII string
};
typedef struct Program Program_t;

// Load program from disk and launch process
//  open file for reading
//  read StartOffset,CodeSize,StackSize,DataSize,Name
//  Allocate spaces in RAM for data, stack, and code segments 
//  Reads the object code from file into the code segment
//  Closes the file
//  Add program as a process, create a thread for it, and execute
//    SP => stack segment
//    R7 => data segment
//    PC => code segment (entry point at first location)
// Inputs: name is the name of the file on SDC
//         priority is the thread priority
// Output: 1 success, 0 failure
int OS_LoadProgram(char *name, uint32_t priority);


#endif
