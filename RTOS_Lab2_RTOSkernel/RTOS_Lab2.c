/* RTOS_Lab2.c
 * Jonathan Valvano
 * December 227, 2025
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)
 */

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../inc/Clock.h"
#include "../RTOS_Labs_common/ST7735_SDC.h"
#include "../RTOS_Labs_common/RTOS_UART.h"
#include "../RTOS_Labs_common/Interpreter.h"
#include "../RTOS_Labs_common/IRDistance.h"
#include "../RTOS_Labs_common/LPF.h"
#include "../RTOS_Labs_common/DFT16.h"
#include "../RTOS_Labs_common/TFLuna2.h"
#include "../RTOS_Labs_common/OS.h"
#include <stdio.h>
// PA10 is UART0 Tx    index 20 in IOMUX PINCM table
// PA11 is UART0 Rx    index 21 in IOMUX PINCM table
// Insert jumper J21: Connects PA10 to XDS_UART
// Insert jumper J22: Connects PA11 to XDS_UART
// Insert jumper J14 SW1 to select PA9
// Insert jumper J15 SW2 to select PA16
// Remove jumps J16,J17,J18: disconnect light sensor
//  PA0 is red LED1,   index 0 in IOMUX PINCM table, negative logic
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S1 positive logic switch,  conflict with TFLuna1, so S1 will not be used
// PB21 is S2 negative logic switch,  used for aperiodic task
// IR analog distance sensors
//   30 cm GP2Y0A41SK0F or 80 cm long range GP2Y0A21YK0F 
//   PA26 Right  ADC0_1
//   PA24 Center ADC0_3, used in Labs 1,2,3,4
//   PA22 Left   ADC0_7
//   PA27 Extra  ADC0_0

// RTOS sensor board supported three TF-Luna sensors
//    Serial TxD: PA17 is UART1 Tx (MSPM0 to TFLuna1)
//    Serial RxD: PA18 is UART1 Rx (TFLuna1 to MSPM0), conflict with LaunchPad S1
//    Serial TxD: PB17 is UART2 Tx (MSPM0 to TFLuna2), used in Labs 1,2,3,4
//    Serial RxD: PB18 is UART2 Rx (TFLuna2 to MSPM0), used in Labs 1,2,3,4
//    Serial TxD: PB12 is UART3 Tx (MSPM0 to TFLuna3), 
//    Serial RxD: PB13 is UART3 Rx (TFLuna3 to MSPM0), shared with LD19 Lidar 
//UART3 is shared between LD19 and TFLuna3 (can have either but not both)

uint32_t NumCreated;   // number of foreground threads created
// 10-sec finite time experiment duration 
Sema4_t LCDFree;  // SDC and LCD sharing

//---------------------User debugging-----------------------
// Performance Measurements 
int32_t MaxJitter;             // largest time jitter between interrupts in 12.5ns
#define JITTERSIZE 256
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram[JITTERSIZE]={0,};
void Jitter_Init(void){
  for(int i=0;i<JitterSize;i++){
    JitterHistogram[i] = 0;
  }
  MaxJitter = 0;
}
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
#define TogglePA8() (GPIOA->DOUTTGL31_0 = (1<<8))
#define TogglePA9() (GPIOA->DOUTTGL31_0 = (1<<9))
#define TogglePA16() (GPIOA->DOUTTGL31_0 = (1<<16))
#define TogglePB4() (GPIOB->DOUTTGL31_0 = (1<<4))
#define TogglePB1() (GPIOB->DOUTTGL31_0 = (1<<1))
#define TogglePB20() (GPIOB->DOUTTGL31_0 = (1<<20))

uint32_t Checks; // number of times virus checking has run
uint32_t ChecksWork; // number of checks in 10 second
//------------------Task 1--------------------------------
// Fixed bandwidth
// real-time sampling ADC0 channel 3, using software start trigger
// 60-Hz notch high-Q, IIR filter, assuming fs=1000 Hz
// y(n) = (256x(n) -476x(n-1) + 256x(n-2) + 471y(n-1)-251y(n-2))/256 (1k sampling)
#define PERIOD TIME_1MS      // DAS 1kHz sampling period in system time units
#define FS 1000              // DAS sampling
#define RUNLENGTH (10000)     // display results and quit when FilterWork==RUNLENGTH
uint32_t FilterOutput,Distance;

uint32_t FilterWork;
//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 1000 times/sec
// samples PA24 Center ADC0_3, calculates Distance
// inputs:  none
// outputs: none
void DAS(void){ 
  uint32_t input;  
  static uint32_t LastTime;      // time at previous ADC sample, 12.5 ns
  uint32_t thisTime;             // time at current ADC sample, 12.5 ns
  uint32_t jitter;               // time between measured and expected, 12.5 ns
  TogglePA8();                   // toggle PA8
  input = ADC0_In();             // channel 3 set when calling ADC0_Init
  TogglePA8();                   // toggle PA8
  thisTime = OS_Time();          // current time, 12.5 ns
  FilterOutput = Filter(input);
  Distance = IRDistance_Convert(FilterOutput,0); // in mm
  if(FilterWork < RUNLENGTH){    // finite time run
    FilterWork++;        // calculation finished
    if(FilterWork>1){    // ignore timing of first interrupt
      uint32_t diff = OS_TimeDifference(LastTime,thisTime);
      if(diff>PERIOD){
        jitter = (diff-PERIOD);  // in 12.5 ns
      }else{
        jitter = (PERIOD-diff);  // in 12.5 ns
      }
      if(jitter > MaxJitter){
        MaxJitter = jitter; // in 12.5 ns
      }       // jitter should be 0
      if(jitter >= JitterSize){
        jitter = JitterSize-1;
      }
      JitterHistogram[jitter]++; 
    }
    ChecksWork = Checks;
    LastTime = thisTime;
  }
  TogglePA8();    // toggle PA8
}
//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// I/O Bound
// background thread executes with S2 button
// S2 negative logic switch on PB21
// one foreground task created with each button push
// foreground treads run for about 500ms and dies
uint32_t DataLost;     // data sent by Producer, but not received by Consumer

// ***********ButtonWork*************
void ButtonWork(void){
  uint32_t myId = OS_Id(); 
  ST7735_Message(1,0,"myID        =",myId); 
  OS_Sleep(500);       // set this to sleep for 500msec
  ST7735_Message(1,1,"Distance(mm)=",Distance);
  ST7735_Message(1,2,"Checks      =",Checks);
  ST7735_Message(1,3,"DataLost    =",DataLost);
  ST7735_Message(1,4,"Jitter (cyc)=",MaxJitter);
  ST7735_Message(1,5,"NumCreated  =",NumCreated);
  OS_Kill();  // done, OS does not return from a Kill
} 

//************S2Push*************
// Called when S2 Button PB21 pushed
// Adds another foreground task
// background threads execute once and return
void S2Push(void){
  TogglePA9();        // toggle PA9
  TogglePA9();        // toggle PA9
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,0)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
  TogglePA9();        // toggle PA9
}
//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------
// I/O Bound
// hardware-triggered TFLuna distance sampling at 100Hz
// Producer runs as part of UART2 ISR
// Producer uses fifo to transmit 100 distance samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement
uint32_t DataLost;        // data sent by Producer, but not received by Consumer
uint32_t Distance2;       // mm
int32_t x[16],ReX[16],ImX[16];           // input and output arrays for FFT

//******** Producer *************** 
// The Producer in this lab will be called from the UART2 ISR
// The TFLuna2 samples distance at about 100 Hz
// sends data to the consumer, runs periodically at 100Hz
void Producer(uint32_t data){  uint32_t dist2; // mm
  if(FilterWork < RUNLENGTH){   // finite time run
    TogglePA16();        // toggle PA16
    dist2 = Median5((int32_t) data);
    TogglePA16();        // toggle PA16
    if(OS_Fifo_Put(dist2) == 0){ // send to consumer
      DataLost++;
    } 
    TogglePA16();        // toggle PA16
  } 
}

//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
void Display(void); 
void Consumer(void){ 
  uint32_t data,DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
  uint32_t t;                  // time in 2.5 ms
  LPF_Init7(500,7);
  TFLuna2_Init(&Producer);
  TFLuna2_Format_Standard_mm(); // format in mm
  TFLuna2_Frame_Rate();         // 100 samples/sec
  TFLuna2_SaveSettings();  // save format and rate
  TFLuna2_System_Reset();  // start measurements
  NumCreated += OS_AddThread(&Display,128,0); 
  while(FilterWork < RUNLENGTH) { 
    for(t = 0; t < 16; t++){   // collect 64 ADC samples
      data = OS_Fifo_Get();    // get from producer, mm
      x[t] = data;             // real part is 0 to 4095, imaginary part is 0
    }
    TogglePB4();        // toggle PB4
    DFT16(x,ReX,ImX);   // complex FFT of last 16 distance values
    TogglePB4();        // toggle PB4
    DCcomponent = ReX[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    OS_MailBox_Send(DCcomponent); // called every 10ms*16 = 160ms
  }
  OS_Kill();  // done
}

//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
  uint32_t data,voltage,distance;
  uint32_t myId = OS_Id();
  ST7735_Message(0,1,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
  while(FilterWork < RUNLENGTH) { 
    TogglePB1();        // toggle PB1
    data = OS_MailBox_Recv();
    voltage = 3000*data/4095;   // calibrate your device so voltage is in mV
    distance = IRDistance_Convert(data,1); // you will calibrate this in Lab 6
    TogglePB1();        // toggle PB1
    ST7735_Message(0,2,"v(mV) =",voltage);  
    ST7735_Message(0,3,"d(mm) =",distance);  
    TogglePB1();        // toggle PB1
 } 
  ST7735_Message(0,4,"Num samples =",FilterWork);  
  OS_Kill();  // done
} 

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
// CPU Bound
// foreground thread that runs without waiting or sleeping
// it executes a virus detector 
uint32_t Check(uint32_t start, uint32_t end){
  uint32_t sum=0;
  uint32_t *pt; pt = (uint32_t *)start;
  while((uint32_t)pt < end){
    sum += *pt++;
  }
  return sum;
}
//******** Virus Detector *************** 
// foreground thread, performs a checksum of all ROM
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
uint32_t Checksum;             // sum of data stored in ROM
uint32_t ChecksumOriginal;     // sum of data stored in ROM
uint32_t ChecksumErrors;
void VirusDetector(void){ 
  Checks = ChecksumErrors = 0;
  ChecksumOriginal = Check(0,0x20000);
  while(1) { 
    TogglePB20();        // toggle PB20
    Checksum = Check(0,0x20000);
    Checks++;
    if(Checksum !=  ChecksumOriginal){
      ChecksumErrors++; 
    }    
  }
}

//--------------end of Task 4-----------------------------

//------------------Task 5--------------------------------
// I/O Bound
// UART0 background ISR performs serial input/output
// Two software fifos are used to pass I/O data to foreground
// The interpreter runs as a foreground thread
// The UART0 driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// The UART0 ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// Similarly, the transmit channel waits on a semaphore in the foreground
// and the UART0 ISR signals this semaphore (TxRoomLeft) when getting data from fifo

//******** Interpreter *************** 
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
void Interpreter(void);    // just a prototype, link to your interpreter
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumCreated, MaxJitter, DataLost, FilterWork, Check
      
// 2) print debugging parameters 
//    i.e., Checks, ChecksumErrors

// Call these from your interpreter
void Lab2(void){int i;
  UART_OutString("\r\nLab 2 performance data");
  UART_OutString("\r\nFilterWork    = "); UART_OutUDec(FilterWork);
  UART_OutString("\r\nNumCreated    = "); UART_OutUDec(NumCreated);
  UART_OutString("\r\nChecksWork    = "); UART_OutUDec(ChecksWork);
  UART_OutString("\r\nDataLost      = "); UART_OutUDec(DataLost); 
  UART_OutString("\r\nMaxJitter(cyc)= "); UART_OutUDec(MaxJitter); 
}
void Lab2_device(int device){int i;
  UART_OutString("\r\nLab 2 performance data");
  ST7735_Message(device, 0, "FilterWork    = ", FilterWork);
  ST7735_Message(device, 1, "NumCreated    = ", NumCreated);
  ST7735_Message(device, 2, "ChecksWork    = ", ChecksWork);
  ST7735_Message(device, 3, "DataLost      = ", DataLost);
  ST7735_Message(device, 4, "MaxJitter(cyc)= ", MaxJitter);

  UART_OutString("\r\nFilterWork    = "); UART_OutUDec(FilterWork);
  UART_OutString("\r\nNumCreated    = "); UART_OutUDec(NumCreated);
  UART_OutString("\r\nChecksWork    = "); UART_OutUDec(ChecksWork);
  UART_OutString("\r\nDataLost      = "); UART_OutUDec(DataLost); 
  UART_OutString("\r\nMaxJitter(cyc)= "); UART_OutUDec(MaxJitter); 
}
void DFT(void){ int i;  int32_t real,imag,mag;
  UART_OutString("\r\nLab 2/3/4 DFT data");
  UART_OutString("\r\nInput,  Output Real, Output Imaginary, Magnitude");
  for(i=0; i<8; i++){
    real = ReX[i];
    imag = ImX[i];    
    mag = sqrt2(real*real+imag*imag);
    UART_OutString("\r\n"); UART_OutUDec(x[i]); UART_OutChar(' '); UART_OutSDec(real); UART_OutChar(' '); UART_OutSDec(imag);
    UART_OutChar(' '); UART_OutSDec(mag);
  }
}
void Jitter(void){ int i;
  UART_OutString("\r\nLab 2 Real-time sampling jitter (12.5ns)");
  UART_OutString("\r\nTime,  Frequency");
  for(i=0; i<JitterSize; i++){
    if(JitterHistogram[i]){ // skip blanks
      UART_OutString("\r\n"); UART_OutUDec3(i);  UART_OutUDec5(JitterHistogram[i]);
    }
  }
  UART_OutString("\r\nMaxJitter(12.5ns) = "); UART_OutUDec(MaxJitter);
}
//--------------end of Task 5-----------------------------


//*******************final user main DEMONTRATE THIS TO TA**********
int realmain(void){     // realmain
  OS_Init();        // initialize, disable interrupts
  Logic_Init();
  DataLost = 0;     // lost data between producer and consumer
  FilterWork = 0;
  Jitter_Init();
  OS_InitSemaphore(&LCDFree, 1);

  // initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init(64);    // ***note*** 4 is not big enough*****

  // hardware init
  ADC0_Init(3,ADCVREF_VDDA);  // PA24 Center ADC0_3, sampling in DAS() 
	
  // attach background tasks
  OS_AddS2Task(&S2Push,1);
  OS_AddPeriodicThread(&DAS,PERIOD/80000,0); // 1 kHz real time sampling of ADC0_3

	// create initial foreground threads
  NumCreated = 0;
  NumCreated += OS_AddThread(&Consumer,128,0); 
  NumCreated += OS_AddThread(&Interpreter,128,0); 
  NumCreated += OS_AddThread(&VirusDetector,128,0);
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 
//*******************Initial TEST**********
// This is the simplest configuration, test this first, (Lab 2 part 1)
// run this with 
// no UART interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
uint32_t Count0;   // number of times thread0 loops
uint32_t Count1;   // number of times thread1 loops
uint32_t Count2;   // number of times thread2 loops
uint32_t Count3;   // number of times thread3 loops
uint32_t Count4;   // number of times thread4 loops
uint32_t Count5;   // number of times thread5 loops
void Thread0(void){
  Count0 = 0;          
  for(;;){
    TogglePA8();        // toggle PA8
    Count0++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread1(void){
  Count1 = 0;          
  for(;;){
    TogglePA9();        // toggle PA9
    Count1++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    TogglePA16();        // toggle PA16
    Count2++;
    OS_Suspend();      // cooperative multitasking
  }
}

int Testmain1(void){  // Testmain1
  OS_Init();          // initialize, disable interrupts
  Logic_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread0,128,0); 
  NumCreated += OS_AddThread(&Thread1,128,0); 
  NumCreated += OS_AddThread(&Thread2,128,0); 
  // Count0 Count1 Count2 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 2 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
void Thread0b(void){
  Count0 = 0;          
  for(;;){
    TogglePA8();        // toggle PA8
    Count0++;
  }
}
void Thread1b(void){
  Count1 = 0;          
  for(;;){
    TogglePA9();        // toggle PA9
    Count1++;
  }
}
void Thread2b(void){
  Count2 = 0;          
  for(;;){
    TogglePA16();        // toggle PA16
    Count2++;
  }
}

int Testmain2(void){  // Testmain2
  OS_Init();          // initialize, disable interrupts
  Logic_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread0b,128,0); 
  NumCreated += OS_AddThread(&Thread1b,128,0); 
  NumCreated += OS_AddThread(&Thread2b,128,0); 
  // Count0 Count1 Count2 should be equal on average
  // counts are much larger than Testmain1
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Third TEST**********
// Once the second test runs, test this (Lab 2 part 2)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
// tests AddThread, Sleep and Kill
// Logic analyzer pattern repeats approximately every 5ms
//   PA9 toggles once
//   PA16 toggles many times for about 2ms
//   PA8 toggles 42 times
//   PA16 toggles many times for about 3ms
void Thread0c(void){ int i;
  Count0 = 0;          
  for(i=0; i<42; i++){
    TogglePA8();        // toggle PA8
    Count0++;
  }
  OS_Kill();
  Count0 = 0;
}
void Thread1c(void){
  Count1 = 0;          
  for(;;){
    TogglePA9();        // toggle PA9
    Count1++;
    NumCreated += OS_AddThread(&Thread0c,128,0); 
    OS_Sleep(5);
  }
}
void Thread2c(void){
  Count2 = 0;          
  for(;;){
    TogglePA16();        // toggle PA16
    Count2++;
  }
}

int Testmain3(void){  // Testmain3
  OS_Init();          // initialize, disable interrupts
  Logic_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1c,128,0); 
  NumCreated += OS_AddThread(&Thread2c,128,0); 
// Count2 should be larger than Count1, Count0 should be 42
// NumCreated should slowly increase creating 1000s of threads (which kill)
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fourth TEST**********
// Once the third test runs, test this (Lab 2 part 2)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// PB21 edge triggered interrupts, falling edge because S2 is negative logic
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4_t Readyd;        // set in background
int Lost;
void BackgroundThread1d(void){   // called at 1000 Hz
  Count1++;
  TogglePA8();        // toggle PA8
  OS_Signal(&Readyd);
}
void Thread5d(void){
  for(;;){
    OS_Wait(&Readyd);
    TogglePA9();        // toggle PA9
    Count5++;   // Count2 + Count5 should equal Count1 
    Lost = Count1-Count5-Count2;
  }
}
void Thread2d(void){
  OS_InitSemaphore(&Readyd,0);
  Count1 = 0;    // number of times signal is called      
  Count2 = 0;    
  Count5 = 0;    // Count2 + Count5 should equal Count1  
  NumCreated += OS_AddThread(&Thread5d,128,0); 
  OS_AddPeriodicThread(&BackgroundThread1d,TIME_1MS/80000,0); 
  for(;;){
    OS_Wait(&Readyd);
    TogglePA16();        // toggle PA16
    Count2++;   // Count2 + Count5 should equal Count1
  }
}
void Thread3d(void){
  Count3 = 0;          
  for(;;){
    TogglePB4();        // toggle PB4
    Count3++;
  }
}
void Thread4d(void){ int i;
  for(i=0;i<64;i++){
    Count4++;
    TogglePB1();        // toggle PB1
    OS_Sleep(10);
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5d(void){   // called when S2 button pushed
  NumCreated += OS_AddThread(&Thread4d,128,0); 
  Count0++;
  TogglePB20();        // toggle PB20
}
      
int Testmain4(void){   // Testmain4
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
  Logic_Init();
  // Count2 + Count5 should equal Count1
  // Count0 increases by 1 every time S2 is pressed
  // Thread4d runs once making Count4 64 (the line Count4=0 should NOT run)
  // Count4 increases by 64 every time S2 is pressed
  NumCreated = 0 ;
  OS_AddS2Task(&BackgroundThread5d,1);
  NumCreated += OS_AddThread(&Thread2d,128,0); 
  NumCreated += OS_AddThread(&Thread3d,128,0); 
  NumCreated += OS_AddThread(&Thread4d,128,0); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fifth TEST**********
// Once the fourth test runs, run this example (Lab 2 part 2)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// S2 switch interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4_t Readye;        // set in background
void BackgroundThread1e(void){   // called at 1000 Hz
static int i=0;
  i++;
  if(i==50){
    i = 0;         //every 50 ms
    Count1++;
    OS_bSignal(&Readye);
  }
}
void Thread2e(void){
  OS_InitSemaphore(&Readye,0);
  Count1 = 0;          
  Count2 = 0;          
  for(;;){
    OS_bWait(&Readye);
    Count2++;     // Count2 should be equal to Count1
  }
}
void Thread3e(void){
  Count3 = 0;          
  for(;;){
    Count3++;     // Count3 should be large
  }
}
void Thread4e(void){ int i;
  for(i=0;i<640;i++){
    Count4++;     // Count4 should increase on button press
    OS_Sleep(1);
  }
  OS_Kill();
}
void BackgroundThread5e(void){   // called when S2 button pushed
  Count0++;
  NumCreated += OS_AddThread(&Thread4e,128,0); 
}

int Testmain5(void){   // Testmain5
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
  // Count1 should exactly equal Count2
  // Count3 should be very large
  // Thread4e runs once making Count4 640 
  // Count0 increments by 1, and Count4 increases by 640 every time S2 is pressed
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThread1e,PERIOD/80000,0); 
  OS_AddS2Task(&BackgroundThread5e,1);
  NumCreated += OS_AddThread(&Thread2e,128,0); 
  NumCreated += OS_AddThread(&Thread3e,128,0); 
  NumCreated += OS_AddThread(&Thread4e,128,0); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Test of S2 task latency *******************
void Thread1S2(){
  while(1){
    TogglePA8();
    OS_Sleep(1);
  }
}

void Thread2S2(){
  while(1){
    TogglePA9();
  }
}

void ThreadPeriodic(){
  TogglePA16();
}

void FunTask(){
  for (int i = 0; i < 42; i++){
    TogglePB4();
  }
  OS_Kill();
}

void ThreadS2(){ // Called when S2 pressed
  TogglePB1();
  NumCreated += OS_AddThread(&FunTask, 128, 0);
}

int TestmainS2(){
  OS_Init();
  Logic_Init();
  NumCreated = 0;
  OS_AddS2Task(&ThreadS2, 1);
  OS_AddPeriodicThread(&ThreadPeriodic, 1, 0);
  NumCreated += OS_AddThread(&Thread1S2, 128, 0);
  NumCreated += OS_AddThread(&Thread2S2, 128, 0);
  OS_Launch(TIME_2MS);
  return 0; // Should never return
}

//*******************Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed 
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// S1 not needed, 
// S2 not needed
// logic analyzer on PB22 for systick interrupt (in your OS)
//                on PA8 to measure context switch time
void ThreadCS(void){    // only thread running
  while(1){
    TogglePA8();        // toggle PA8
  }
}
int TestmainCS(void){       // TestmainCS
  Logic_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&ThreadCS,128,0); 
  OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here
  return 0;               // this never executes
}

//*******************FIFO TEST**********
// FIFO test
// Count1 should exactly equal Count2
// Count3 should be very large
// Error8 should be 0
// DataLost should be 0
// Timer interrupts, with period established by OS_AddPeriodicThread
uint32_t OtherCount1;
uint32_t Expected8; // last data read+1
uint32_t Error8;
void ConsumerThreadFIFO(void){        
  Count2 = 0;          
  for(;;){
    OtherCount1 = OS_Fifo_Get();
    if(OtherCount1 != Expected8){
      Error8++;
    }
    Expected8 = OtherCount1+1; // should be sequential
    Count2++;     
  }
}
void FillerThreadFIFO(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void BackgroundThreadFIFOProducer(void){   // called periodically
  if(OS_Fifo_Put(Count1) == 0){ // send to consumer
    DataLost++;
  }
  Count1++;
}

int TestmainFIFO(void){   // TestmainFIFO
  Count1 = 0;     DataLost = 0;  
  Expected8 = 0;  Error8 = 0;
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThreadFIFOProducer,PERIOD/80000,0); 
  OS_Fifo_Init(16);
  NumCreated += OS_AddThread(&ConsumerThreadFIFO,128,0); 
  NumCreated += OS_AddThread(&FillerThreadFIFO,128,0); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Trampoline for selecting which main to execute**********
int main(void) { 			// main 
  __disable_irq();
  Clock_Init80MHz(0); // no clock out to pin
  LaunchPad_Init();   // LaunchPad_Init must be called once and before other I/O initializations
  realmain();
}


