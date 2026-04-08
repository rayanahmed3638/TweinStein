/* aPeriodicIntmain.c
 * Jonathan Valvano
 * December 19, 2025
 * Derived from timx_timer_mode_periodic_sleep_LP_MSPM0G3507_nortos_ticlang
 *              gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang
 *
 */

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#define NUMTASKS 3
#define RUNLENGTH 1000     // 7 sec
#define PERIOD (80*7000)   // 1+2+4ms
uint32_t Count0,Count1,Count2;
//---------------------User debugging-----------------------
int32_t MaxJitter[NUMTASKS];             // largest time jitter between interrupts in 10 usec
#define JITTERSIZE 32
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram[NUMTASKS][JITTERSIZE]={0,};
// Unused sensor board pins, made outputs for debugging
// Jumper J14 select PA9
// Jumper J15 select PA16
void Logic_Init(void){
  IOMUX->SECCFG.PINCM[PA8INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PA9INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PA16INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PB4INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PB1INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PB20INDEX] = (uint32_t) 0x00000081;
  GPIOA->DOE31_0 |= (1<<8)|(1<<9)|(1<<16);
  GPIOB->DOE31_0 |= (1<<4)|(1<<1)|(1<<20);
}
uint32_t TheTime(void){
  return ~(TIMG12->COUNTERREGS.CTR);
};
void Task0(void){ 
  GPIOA->DOUTTGL31_0 = (1<<8); // PA8
uint32_t thisTime,jitter;
static uint32_t LastTime;      // time at previous execution, 12.5 ns
  thisTime = TheTime();          // current time, 12.5 ns
  Count0++;
  if(Count0>2){      // ignore timing of first couple interrupt
    uint32_t diff = thisTime-LastTime;  // down count
    if(diff>PERIOD){ // should be 1ms (12.5ns units)
      jitter = (diff-PERIOD);  // in 12.5ns 
    }else{
      jitter = (PERIOD-diff);  // in 12.5ns 
    }
    if(jitter > MaxJitter[0]){
      MaxJitter[0] = jitter; // in 12.5ns 
    }       // jitter should be 0
    if(jitter >= JitterSize){
      jitter = JitterSize-1;
    }
    JitterHistogram[0][jitter]++; 
  }
  LastTime = thisTime;
}
void Task1(void){ 
  GPIOA->DOUTTGL31_0 = (1<<9); // PA9
uint32_t thisTime,jitter;
static uint32_t LastTime;      // time at previous execution, 12.5 ns
  thisTime = TheTime();          // current time, 12.5 ns
  Count1++;
  if(Count1>2){      // ignore timing of first couple interrupt
    uint32_t diff = thisTime-LastTime;  // down count
    if(diff>PERIOD){ // should be 1ms (12.5ns units)
      jitter = (diff-PERIOD);  // in 12.5ns 
    }else{
      jitter = (PERIOD-diff);  // in 12.5ns 
    }
    if(jitter > MaxJitter[1]){
      MaxJitter[1] = jitter; // in 12.5ns 
    }       // jitter should be 0
    if(jitter >= JitterSize){
      jitter = JitterSize-1;
    }
    JitterHistogram[1][jitter]++; 
  }
  LastTime = thisTime;
}
void Task2(void){
  GPIOA->DOUTTGL31_0 = (1<<16); // PA16
uint32_t thisTime,jitter;
static uint32_t LastTime;      // time at previous execution, 12.5 ns
  thisTime = TheTime();          // current time, 12.5 ns
  Count2++;
  if(Count2>2){      // ignore timing of first couple interrupt
    uint32_t diff = thisTime-LastTime;  // down count
    if(diff>PERIOD){ // should be 1ms (12.5ns units)
      jitter = (diff-PERIOD);  // in 12.5ns 
    }else{
      jitter = (PERIOD-diff);  // in 12.5ns 
    }
    if(jitter > MaxJitter[0]){
      MaxJitter[2] = jitter; // in 12.5ns 
    }       // jitter should be 0
    if(jitter >= JitterSize){
      jitter = JitterSize-1;
    }
    JitterHistogram[2][jitter]++; 
  }
  LastTime = thisTime;
}
void TogglePB4(void){ GPIOB->DOUTTGL31_0 = (1<<4);}
void TogglePB1(void){ GPIOB->DOUTTGL31_0 = (1<<1);}
void TogglePB20(void){GPIOB->DOUTTGL31_0 = (1<<20);}

struct event{
  void(*task)(void);    // this task to run
  uint32_t timeToNext;  // time for this task
};
typedef const struct event event_t;
uint32_t ThisTask;
event_t Tasks[NUMTASKS]={
  {Task0,  4000}, // Toggle PA8 then wait 4ms
  {Task1,  2000}, // Toggle PA9 then wait 2ms
  {Task2,  1000}  // Toggle PA16 then wait 1ms
};


void Jitter_Init(void){
  for(int j=0; j<NUMTASKS; j++){
    for(int i=0;i<JitterSize;i++){
      JitterHistogram[j][i] = 0;
    }
    MaxJitter[j] = 0;
  }
}
// Power Domain PD1
// initialize A0/A1/G6/G7/G12 for aperiodic interrupt
// for 32MHz bus clock, Timer clock is 32MHz
// for 40MHz bus clock, Timer clock is MCLK 40MHz
// for 80MHz bus clock, Timer clock is MCLK 80MHz
// frequency = TimerClock/prescale/period
// initialize G7 for periodic interrupt
// frequency = TimerClock/prescale/period
void TimerG7_IntArm(uint32_t prescale, uint32_t priority){
  TIMG7->GPRCM.RSTCTL = (uint32_t)0xB1000003;
  TIMG7->GPRCM.PWREN = (uint32_t)0x26000001;
  Clock_Delay(24); // time for TimerG7 to power up
  TIMG7->CLKSEL = 0x08; // bus clock
  TIMG7->CLKDIV = 0x00; // divide by 1
  TIMG7->COMMONREGS.CPS = prescale-1;     // divide by prescale
  TIMG7->COUNTERREGS.LOAD  = 80000-1;     // set reload register, will be reset during each ISR
  TIMG7->COUNTERREGS.CTRCTL = 0x02;
    // bits 5-4 CM =0, down
    // bits 3-1 REPEAT =001, continue
    // bit 0 EN enable (0 for disable, 1 for enable)
  TIMG7->CPU_INT.IMASK |= 1; // zero event mask
  TIMG7->COMMONREGS.CCLKCTL = 1;
  NVIC->ISER[0] = 1 << 20; // TIMG7 interrupt
  NVIC->IP[5] = (NVIC->IP[5]&(~0x000000FF))|(priority<<6);    // set priority (bits 7,6) IRQ 20
  TIMG7->COUNTERREGS.CTRCTL |= 0x01;
}

// G12 for profiling
void TimerG12_Init(void){
  TIMG12->GPRCM.RSTCTL = (uint32_t)0xB1000003;
  TIMG12->GPRCM.PWREN = (uint32_t)0x26000001;
  Clock_Delay(24);     // time for TimerG8 to power up
  TIMG12->CLKSEL = 0x08; // bus clock
  TIMG12->CLKDIV = 0; // divide by 1
//  TIMG8->COMMONREGS.CPS = prescale-1;    // divide by prescale,
  TIMG12->COUNTERREGS.LOAD  = 0xFFFFFFFF;     // set reload register
  TIMG12->COUNTERREGS.CTRCTL = 0x02;
    // bits 5-4 CM =0, down
    // bits 3-1 REPEAT =001, continue
    // bit 0 EN enable (0 for disable, 1 for enable)
  TIMG12->COMMONREGS.CCLKCTL = 1;
  TIMG12->COUNTERREGS.CTRCTL |= 0x01;
}

int main(void){
  __disable_irq();
  Clock_Init80MHz(0);
  Count0=Count1=Count2=0;
  ThisTask = 0;
  LaunchPad_Init();
  Logic_Init();
  Jitter_Init();
  TimerG12_Init();   // free running timer for OS_Time
  TimerG7_IntArm(80,2); // 80MHz/80 = 1MHz, or 1us period
  __enable_irq();
  while(1){
   TogglePB1();
  }
}

void TIMG7_IRQHandler(void){
  if((TIMG7->CPU_INT.IIDX) == 1){ // this will acknowledge
    (*Tasks[ThisTask].task)();   // execute user task
    ThisTask = (ThisTask+1)%NUMTASKS;
    TIMG7->COUNTERREGS.LOAD  = Tasks[ThisTask].timeToNext-1;    // set reload register
  }
}




