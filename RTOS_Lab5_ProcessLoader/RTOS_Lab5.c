/* RTOS_Lab5.c
 * Jonathan Valvano
 * January 10, 2026
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)
 */
// You need to run Testmain2 before Testmain3 Testmain5 Testmain6 and realmain
// Test Testmain2 formats the disk and loads to programs Blinky and Prog2

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
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Labs_common/heap.h"
#include <stdio.h>
// PA10 is UART0 Tx    index 20 in IOMUX PINCM table
// PA11 is UART0 Rx    index 21 in IOMUX PINCM table
// Insert jumper J25: Connects PA10 to XDS_UART
// Insert jumper J26: Connects PA11 to XDS_UART
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

// **** OS must run disk_timerproc();  at 1000Hz, every 1ms *****
uint32_t Running;      // true while robot is running
uint32_t NumCreated;   // number of foreground threads created
uint32_t NumProcessCreated;   // number of foreground processes created

extern Program_t ProgramBlock;
extern Program_t ProgramBlock2;

//---------------------User debugging-----------------------

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
#define SetPA8() (GPIOA->DOUTSET31_0 = (1<<8))
#define ClrPA8() (GPIOA->DOUTCLR31_0 = (1<<8))
#define TogglePA9() (GPIOA->DOUTTGL31_0 = (1<<9))
#define SetPA9() (GPIOA->DOUTSET31_0 = (1<<9))
#define ClrPA9() (GPIOA->DOUTCLR31_0 = (1<<9))
#define TogglePA16() (GPIOA->DOUTTGL31_0 = (1<<16))
#define TogglePB4() (GPIOB->DOUTTGL31_0 = (1<<4))
#define SetPB4() (GPIOB->DOUTSET31_0 = (1<<4))
#define ClrPB4() (GPIOB->DOUTCLR31_0 = (1<<4))
#define TogglePB1() (GPIOB->DOUTTGL31_0 = (1<<1))
#define TogglePB20() (GPIOB->DOUTTGL31_0 = (1<<20))

uint32_t Checks; // number of times virus checking has run
uint32_t ChecksWork; // number of checks in 10 second
//------------------Task 1--------------------------------
// real-time sampling ADC0 channel 3, using software start trigger
// 60-Hz notch high-Q, IIR filter, assuming fs=1000 Hz
// y(n) = (256x(n) -476x(n-1) + 256x(n-2) + 471y(n-1)-251y(n-2))/256 (1k sampling)
#define PERIOD TIME_1MS      // DAS 1kHz sampling period in system time units
#define FS 1000              // DAS sampling
#define RUNLENGTH (10000)     // 10 seconds, display results and quit when FilterWork==RUNLENGTH
uint32_t FilterOutput,Distance;
Sema4_t LCDFree;  // SDC and LCD sharing

uint32_t FilterWork;
uint32_t MaxJitter3;  
#define JITTERSIZE3 512
uint32_t const JitterSize3=JITTERSIZE3;
uint32_t JitterHistogram3[JITTERSIZE3]={0,};
void Jitter3_Init(void){
  for(int i=0;i<JitterSize3;i++){
    JitterHistogram3[i] = 0;
  }
  MaxJitter3 = 0;
}
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
  if(Running){    // finite time run
    FilterWork++;        // calculation finished
    if(FilterWork>2){    // ignore timing of first interrupt
      uint32_t diff = OS_TimeDifference(LastTime,thisTime);
      if(diff>PERIOD){
        jitter = (diff-PERIOD);  // in 12.5 ns
      }else{
        jitter = (PERIOD-diff);  // in 12.5 ns
      }
      if(jitter > MaxJitter3){
        MaxJitter3 = jitter; // in 12.5 ns
      }       // jitter should be 0    
      JitterHistogram3[jitter]++; 
    }
    ChecksWork = Checks;
    LastTime = thisTime;
  }
  TogglePA8();    // toggle PA8
}
//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with PA28 button
// PA28 negative logic switch 
// one foreground task created with each button push
// foreground tread outputs a message and dies
uint32_t DataLost;     // data sent by Producer, but not received by Consumer

// ***********PA28Push*************
int ArmCrash=1;
void HandleCrash(void){
  TogglePA9();
  TogglePA9();
  uint32_t myId = OS_Id(); 
  ST7735_Message(1,0,"myID        =",myId); 
  ST7735_Message(1,1,"*Crash*,  t= ",OS_MsTime());
  ArmCrash=1;
  TogglePA9();
  OS_Kill();
} 
void PA28Push(void){ // real time task
  if(ArmCrash){
    ArmCrash = 0; // debounce
    NumCreated += OS_AddThread(&HandleCrash,128,1);  // test eDisk
  }
} 

//------------------Task 3--------------------------------
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
void Producer(uint32_t data){ 
  if(Running){           // finite time run
    TogglePA16();        // toggle PA16
    Distance2 = Median5((int32_t) data);
    TogglePA16();        // toggle PA16
    if(OS_Fifo_Put(Distance2) == 0){ // send to consumer
      DataLost++;
    } 
    TogglePA16();        // toggle PA16
  } 
}

void Display(void); 

// Describe the error with text on the LCD and then stall. 
// If you are getting disk errors, rerun Testmain1 Testmain2 Testmain3
void diskError(char *errtype, int32_t code){
  OS_bSignal(&LCDFree);
  ST7735_DrawString(0, 1, "Err:  ", ST7735_LIGHTGREY);
  ST7735_DrawString(5, 1, errtype, ST7735_LIGHTGREY);
  ST7735_DrawString(0, 2, "Code: ", ST7735_LIGHTGREY);
  ST7735_SetCursor(6, 2);
  ST7735_SetTextColor(ST7735_RED);
  ST7735_OutUDec(code);
  Running = 0;
  OS_Kill();
}
void StartFileDump(char *pt){
  OS_bWait(&LCDFree);
  eFile_Create(pt); // ignore error if file already exists
  if(eFile_WOpen(pt))  diskError("eFile_WOpen",0);
  if(eFile_WriteString("time(s)\tdist(mm)\tdist(mm)\n\r"))  diskError("eFile_WriteString",0);
  OS_bSignal(&LCDFree);
}
void EndFileDump(){
  OS_bWait(&LCDFree);
  if(eFile_WClose())            diskError("eFile_WClose",0);
  OS_bSignal(&LCDFree);
}

void FileDump(uint32_t data, uint32_t data2){
  SetPB4();
  OS_bWait(&LCDFree);
  eFile_WriteUFix2(OS_MsTime()/10); eFile_Write('\t');
  eFile_WriteUDec(data); eFile_Write('\t');
  eFile_WriteUDec(data2); eFile_WriteString("\n\r");
  OS_bSignal(&LCDFree);
  ClrPB4();
}
//******** Robot *************** 
// foreground Consumer thread, accepts data from producer
// inputs:  none
// outputs: none
char FileName[8]="robot0";

void Robot(void){   
  DataLost = 0;       // new run with no lost data 
  FilterWork = 0;
  Running = 1;
  Jitter3_Init();

  OS_ClearMsTime();    
  OS_Fifo_Init(256);
  NumCreated += OS_AddThread(&Display,128,0); 
  UART_OutString("Robot running...");
  StartFileDump(FileName);

  while(FilterWork < RUNLENGTH) { 
    uint32_t data;      // in mm, from TFLuna
    uint32_t sum=0;
    for(int t = 0; t < 16; t++){   // collect 16 TFLuna samples
      data = OS_Fifo_Get();    // get from producer, mm
      x[t] = data;
      sum += data;             // average
    }
    Distance2 = sum>>4;  // in mm
    FileDump(Distance,Distance2);
    OS_MailBox_Send(Distance2); // called every 10ms*16 = 160ms
  }
  EndFileDump();
  UART_OutString("done.\n\r>");
  FileName[5] = (FileName[5]+1)&0xF7; // 0 to 7
  Running = 0;             // robot no longer running
  OS_Kill();
}
 //************S2Push*************
// Called when S2 Button pushed, fall of PB21
// Adds another Robot foreground task
// background threads execute once and return
void S2Push(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&Robot,128,1);  // test eDisk
  }
}
//--------------end of Task 2-----------------------------
 
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
  uint32_t distance;
  uint32_t myId = OS_Id();
  ST7735_Message(1,1,"myId = ",myId);   // top half used for Display
  ST7735_Message(1,2,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
  while(Running) { 
    TogglePB1();        // toggle PB1
    distance = OS_MailBox_Recv();
// you will calibrate this in Lab 6
    TogglePB1();        // toggle PB1
    ST7735_Message(1,3,"Time(ms) =",OS_MsTime());  
    ST7735_Message(1,4,"work  =",FilterWork);  
    ST7735_Message(1,5,"d(mm) =",distance);  
    TogglePB1();        // toggle PB1
 } 
  OS_Kill();  // done
} 

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
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
//    i.e., NumCreated, MaxJitter, DataLost, FilterWork, Checks
      
// 2) print debugging parameters 
//    i.e., Checks, ChecksumErrors

// Call these from your interpreter
void Lab5(void){  heap_stats_t heap;
  UART_OutString("\r\nLab 5 performance data");
  UART_OutString("\r\nNumCreated   = "); UART_OutUDec(NumCreated);
  UART_OutString("\r\nNumProcesses = "); UART_OutUDec(NumProcessCreated);

  if(!Heap_Stats(&heap)){
    UART_OutString("\r\nHeap size    = "); UART_OutUDec(heap.size); UART_OutString(" bytes");
    UART_OutString("\r\nHeap used    = "); UART_OutUDec(heap.used); UART_OutString(" bytes");
    UART_OutString("\r\nHeap free    = "); UART_OutUDec(heap.free); UART_OutString(" bytes");
    UART_OutString("\r\nHeap waste   = "); UART_OutUDec(heap.size - heap.used - heap.free); UART_OutString(" bytes");
  }
}
void DFT(void){ int i;  int32_t real,imag,mag;
  UART_OutString("\r\nLab 2/3 DFT data");
  UART_OutString("\r\nInput,  Output Real, Output Imaginary, Magnitude");
  for(i=0; i<8; i++){
    real = ReX[i];
    imag = ImX[i];    
    mag = sqrt2(real*real+imag*imag);
    UART_OutString("\r\n"); UART_OutUDec(x[i]); UART_OutChar(' '); UART_OutSDec(real); UART_OutChar(' '); UART_OutSDec(imag);
    UART_OutChar(' '); UART_OutSDec(mag);
  }
}

//--------------end of Task 5-----------------------------
void ProcessLoadBlinky(void){
  UART_OutString("\n\rECE445M Lab 5 Load Blinky\n\r");
  OS_bWait(&LCDFree);
  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);
  OS_bSignal(&LCDFree);
  NumProcessCreated += OS_LoadProgram("Blinky",2);
  NumProcessCreated += OS_LoadProgram("Prog2",1);

  UART_OutString("\n\rOS_LoadProgram Blinky ok\n\r>");
  OS_Kill();
}
//*******************final user main DEMONTRATE THIS TO TA**********
int realmain(void){     // realmain
  OS_Init();        // initialize, disable interrupts

  Logic_Init();
  DataLost = 0;     // lost data between producer and consumer
  FilterWork = 0;
  Jitter3_Init();
  // initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init(256);    // ***note*** 4 is not big enough*****

  // hardware init
  ADC0_Init(3,ADCVREF_VDDA);  // PA24 Center ADC0_3, sampling in DAS() 
	OS_InitSemaphore(&LCDFree, 1);
  // attach background tasks
  OS_AddS2Task(&S2Push,1);      // fall of PB21
  OS_AddPA28Task(&PA28Push,1);  // fall of PA28
  OS_AddPeriodicThread(&DAS,PERIOD/80000,0); // 1 kHz real time sampling of ADC0_3
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
  
	// create initial foreground threads
  NumCreated = 0;
  NumCreated += OS_AddThread(&Interpreter,128,1); 
  NumCreated += OS_AddThread(&ProcessLoadBlinky,128,2);
  NumCreated += OS_AddThread(&VirusDetector,128,2);
  NumProcessCreated = 0;
  LPF_Init7(500,7);
  TFLuna2_Init(&Producer);
  TFLuna2_Format_Standard_mm(); // format in mm
  TFLuna2_Frame_Rate();         // 100 samples/sec
  TFLuna2_SaveSettings();  // save format and rate
  TFLuna2_System_Reset();  // start measurements

 // if(eFile_Init())              diskError("eFile_Init",0); 
 // if(eFile_Format())            diskError("eFile_Format",0); 
 // if(eFile_Mount())             diskError("eFile_Mount",0);
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 

uint32_t M=1;
uint32_t Random32(void){
  M = 1664525*M+1013904223;
  return M;
}
// 0 to 31
uint32_t Random5(void){
  return (Random32()>>27);
}
// 0 to 127
uint32_t Random7(void){
  return (Random32()>>25);
}
// 0 to 255
uint8_t Random8(void){
  return (Random32()>>24);
}

//*****************Test project 1*************************
// Syntax check of Program structure
unsigned char buffer[512];  // don't put on stack

void DumpProgramInformation(void){
uint32_t start,code,stack,data,number;
char *name; char *pt;
  pt = (char*)&ProgramBlock;
  start = ProgramBlock.StartOffset;
  code = ProgramBlock.CodeSize;
  stack = ProgramBlock.StackSize;
  data = ProgramBlock.DataSize;
  name = ProgramBlock.Name;
  UART_OutString("ECE445M Spring 2026 testmain1");
  UART_OutString("Name="); UART_OutString(name);  UART_OutString("\n\r");
  UART_OutString("Start offset="); UART_OutUDec(start);  UART_OutString("\n\r");
  UART_OutString("Code size="); UART_OutUDec(code);  UART_OutString("\n\r");
  UART_OutString("Stack size="); UART_OutUDec(stack);  UART_OutString("\n\r");
  UART_OutString("Data size="); UART_OutUDec(data);  UART_OutString("\n\r");
  for(int i=0; i<code;i++){
    number = *pt++;
    UART_OutUHex2(number);
    if((i%10)==9) UART_OutString("\n\r");
  }
  OS_Kill();
}
int Testmain1(void){   // Testmain1
  OS_Init();           // initialize, disable interrupts
  Logic_Init();
  Running = 0; 
  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&DumpProgramInformation,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


//*****************Test project 2*************************
// Test of saving object files 
// Warning: this reformats the disk, all existing data will be lost
void PrintDirectory(void){ char *name; unsigned long size; 
  unsigned int num;
  unsigned long total;
  num = 0;
  total = 0;
  UART_OutString("\n\r");
  if(eFile_DOpen(""))           diskError("eFile_DOpen",0);
  while(!eFile_DirNext(&name, &size)){
    UART_OutString("Filename = "); UART_OutString(name); UART_OutString("  ");
    UART_OutString("Size (bytes)= "); UART_OutUDec(size); UART_OutString("\n\r");
    total = total+size;
    num++;    
  }
  UART_OutString("Number of Files = "); UART_OutUDec(num); UART_OutString("\n\r");
  UART_OutString("Number of Bytes = "); UART_OutUDec(total); UART_OutString("\n\r");
  if(eFile_DClose())            diskError("eFile_DClose",0);
}
void SaveProgram(char *name, uint32_t codesize, char *pt ){int i; 
  if(eFile_Create(name))     diskError("eFile_Create",0);
  if(eFile_WOpen(name))      diskError("eFile_WOpen",0);
  for(i=0; i<codesize; i++){
    eFile_Write(*pt);
    pt++;
  }
  if(eFile_WClose())            diskError("eFile_WClose",0);
}
void PrintCode(char *name){int i; char data; int status;
  if(eFile_ROpen(name))      diskError("eFile_ROpen",0);
  i=0;
  do{
    status = eFile_ReadNext(&data);
    if(status == 0) {
      UART_OutUHex2(data);
      if((i%10)==9) UART_OutString("\n\r");
      i++;
    }
  }while(status==0);
  UART_OutString("\n\r");
  if(eFile_RClose())  diskError("eFile_RClose",0);
}

void PrintObjectCode(char *name){int i; char data; int status;
  uint32_t startoffset,codesize,stacksize,datasize;
  if(eFile_ROpen(name))      diskError("eFile_ROpen",0);
  if(eFileReadNextWord(&startoffset)) diskError("eFileReadNextWord",0); 
  if(startoffset<20){
    UART_OutString("\n\rBad start offset = ");UART_OutString(name);
    UART_OutString("\n\r");
    if(eFile_RClose())  diskError("eFile_RClose",0);
    return;
  }  
  if(eFileReadNextWord(&codesize))  diskError("eFileReadNextWord",0); 
  if(eFileReadNextWord(&stacksize)) diskError("eFileReadNextWord",0); 
  if(eFileReadNextWord(&datasize) ) diskError("eFileReadNextWord",0); 

  UART_OutString("\n\rFilename = "); 
  i=16; int endofName=1;
  while(i<startoffset){
    status = eFile_ReadNext(&data);
    if(status == 0) {
      if(data){
        if(endofName) UART_OutChar(data);
      }
      else endofName=0;
    }
    i++;
  }
  UART_OutString("\n\r StartOffset= "); UART_OutUDec(startoffset); 
  UART_OutString("\n\r CodeSize   = "); UART_OutUDec(codesize); 
  UART_OutString("\n\r StackSize  = "); UART_OutUDec(stacksize); 
  UART_OutString("\n\r DataSize   = "); UART_OutUDec(datasize); UART_OutString("\n\r");
  do{
    status = eFile_ReadNext(&data);
    if(status == 0) {
      UART_OutUHex2(data);
      if(((i+startoffset)&0x0F)==0x0F) UART_OutString("\n\r");
      i++;
    }
  }while(status==0);
  UART_OutString("\n\r");
  if(eFile_RClose())  diskError("eFile_RClose",0);
}
void TestSaveProgram(void){   uint32_t code; char *pt; int i; char data; int status;
  UART_OutString("\n\rECE445M Lab 5 save program\n\r");
  ST7735_DrawString(0, 1, "Save program     ", ST7735_WHITE);

  // simple test of eFile
  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Format())            diskError("eFile_Format",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);
  PrintDirectory();
  SaveProgram(ProgramBlock.Name, ProgramBlock.CodeSize, (char*) &ProgramBlock);
  SaveProgram(ProgramBlock2.Name, ProgramBlock2.CodeSize, (char*) &ProgramBlock2);
  PrintDirectory();
 // PrintCode(ProgramBlock.Name);
  PrintObjectCode(ProgramBlock.Name);
  PrintObjectCode(ProgramBlock2.Name);
  if(eFile_Unmount())           diskError("eFile_Unmount",0);
  UART_OutString("\n\rSuccessful test\n\r");
  ST7735_DrawString(0, 1, "eFile successful", ST7735_YELLOW);
  Running=0; // launch again
  OS_Kill();
}

void StartFileTest(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&TestSaveProgram,128,1);  // test eFile
  }
}

int Testmain2(void){   // Testmain2 
  OS_Init();           // initialize, disable interrupts
  Logic_Init();
  Running = 1; 

  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
  OS_AddS2Task(&StartFileTest,1);
  OS_AddPA28Task(&StartFileTest,1);
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&TestSaveProgram,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*****************Test project 3*************************
// Test of loading object files 
// Testmain2 must have been successfully run (it creates Blinky on disk)
// Warning: this reformats the disk, all existing data will be lost
char Buffer3[256]; // big enough for Blinky
uint32_t Count3;
void LoadObjectCode(char *name, char* buffer){int i; char data; int status;
  OS_bWait(&LCDFree);
  if(eFile_ROpen(name))      diskError("eFile_ROpen",0);
  i=0;
  do{
    status = eFile_ReadNext(&data);
    Buffer3[i] = data;
    i++;
  }while((status==0)&&(i<256));
  if(eFile_RClose())  diskError("eFile_RClose",0);
  OS_bSignal(&LCDFree);
}
void Chaos3(void){
  ST7735_Message(1,0,"Chaos",3); 
  while(1){
    for(int l=1; l<5; l++){
      ST7735_Message(1,l,"n =",Random8());  
    }
    OS_Sleep(100);
  }
}
void FileTest3(void){
  UART_OutString("\n\rECE445M Lab 5 load program\n\r");
  OS_bWait(&LCDFree);
  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);
  OS_bSignal(&LCDFree);
  LoadObjectCode("Blinky",Buffer3);
  Count3++;
  UART_OutString("LoadObjectCode ok, Count3=");UART_OutUDec(Count3); UART_OutString("\n\r");
  ST7735_Message(0,1,"Count3=",Count3);  
  Running=0; // launch again
  OS_Kill();
}
void StartFileTest3(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&FileTest3,128,1);  // test eFile
  }
}

int Testmain3(void){   // Testmain3 
  OS_Init();           // initialize, disable interrupts
  Logic_Init();
  Running = 1; 
	OS_InitSemaphore(&LCDFree, 1);
  Count3 = 0;
  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
  OS_AddS2Task(&StartFileTest3,1);
  OS_AddPA28Task(&StartFileTest3,1);
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&FileTest3,128,1);  
  NumCreated += OS_AddThread(&Chaos3,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


//*****************Test project 4*************************
// Heap test, allocate and deallocate memory
void heapError(char* errtype, char* v,uint32_t n){
  UART_OutString("\n\rErr: ");    UART_OutString(errtype);
  UART_OutString(" heap error "); UART_OutString(v);
  UART_OutUDec(n);                UART_OutString("\n\r"); 
  OS_Kill();
}
heap_stats_t stats;
void heapStats(void){
  if(Heap_Stats(&stats))  heapError("Heap_Stats","",0);
  ST7735_Message(1,0,"Heap size  =",stats.size); 
  ST7735_Message(1,1,"Heap used  =",stats.used);  
  ST7735_Message(1,2,"Heap free  =",stats.free);
  ST7735_Message(1,3,"Heap waste =",stats.size - stats.used - stats.free);
  UART_OutString("\n\rHeap size  ="); UART_OutUDec(stats.size);
  UART_OutString("\n\rHeap used  ="); UART_OutUDec(stats.used);
  UART_OutString("\n\rHeap free  ="); UART_OutUDec(stats.free);
  UART_OutString("\n\rHeap waste ="); UART_OutUDec(stats.size - stats.used - stats.free);
}
int16_t* ptr;  // Global so easier to see with the debugger
int16_t* p1;   // Proper style would be to make these variables local
int16_t* p2;
int16_t* p3;
uint8_t* q1;
uint8_t* q2;
uint8_t* q3;
uint8_t* q4;
uint8_t* q5;
uint8_t* q6;
int16_t maxBlockSize;
uint8_t* bigBlock;
void TestHeap(void){  int16_t i;  
  ST7735_DrawString(0, 0, "Heap test            ", ST7735_WHITE);
  UART_OutString("\n\rECE445M, Lab 5 Heap Test");
  if(Heap_Init())         heapError("Heap_Init","",0);

  ptr = Heap_Malloc(sizeof(int16_t));
  if(!ptr)                heapError("Heap_Malloc","ptr",0);
  *ptr = 0x1111;

  if(Heap_Free(ptr))      heapError("Heap_Free","ptr",0);

  ptr = Heap_Malloc(1);
  if(!ptr)                heapError("Heap_Malloc","ptr",1);

  if(Heap_Free(ptr))      heapError("Heap_Free","ptr",1);

  p1 = (int16_t*) Heap_Malloc(1 * sizeof(int16_t));
  if(!p1)                 heapError("Heap_Malloc","p",1);
  p2 = (int16_t*) Heap_Malloc(2 * sizeof(int16_t));
  if(!p2)                 heapError("Heap_Malloc","p",2);
  p3 = (int16_t*) Heap_Malloc(3 * sizeof(int16_t));
  if(!p3)                 heapError("Heap_Malloc","p",3);
  p1[0] = 0xAAAA;
  p2[0] = 0xBBBB;
  p2[1] = 0xBBBB;
  p3[0] = 0xCCCC;
  p3[1] = 0xCCCC;
  p3[2] = 0xCCCC;
  heapStats();

  if(Heap_Free(p1))       heapError("Heap_Free","p",1);
  if(Heap_Free(p3))       heapError("Heap_Free","p",3);

  if(Heap_Free(p2))       heapError("Heap_Free","p",2);
  heapStats();

  for(i = 0; i <= (stats.size / sizeof(int32_t)); i++){
    ptr = Heap_Malloc(sizeof(int16_t));
    if(!ptr) break;
  }
  if(ptr)                 heapError("Heap_Malloc","i",i);
  heapStats();
  
  UART_OutString("\n\rRealloc test\n\r");
  if(Heap_Init())         heapError("Heap_Init","",1);
  q1 = Heap_Malloc(1);
  if(!q1)                 heapError("Heap_Malloc","q",1);
  q2 = Heap_Malloc(2);
  if(!q2)                 heapError("Heap_Malloc","q",2);
  q3 = Heap_Malloc(3);
  if(!q3)                 heapError("Heap_Malloc","q",3);
  q4 = Heap_Malloc(4);
  if(!q4)                 heapError("Heap_Malloc","q",4);
  q5 = Heap_Malloc(5);
  if(!q5)                 heapError("Heap_Malloc","q",5);

  *q1 = 0xDD;
  q6 = Heap_Realloc(q1, 6);
  heapStats();

  for(i = 0; i < 6; i++){
    q6[i] = 0xEE;
  }
  q1 = Heap_Realloc(q6, 2);
  heapStats();

  UART_OutString("\n\rLarge block test");
  if(Heap_Init())         heapError("Heap_Init","",2);
  heapStats();
  maxBlockSize = stats.free;
  bigBlock = Heap_Malloc(maxBlockSize);
  for(i = 0; i < maxBlockSize; i++){
    bigBlock[i] = 0xFF;
  }
  heapStats();
  if(Heap_Free(bigBlock)) heapError("Heap_Free","bigBlock",0);

  bigBlock = Heap_Calloc(maxBlockSize);
  if(!bigBlock)           heapError("Heap_Calloc","bigBlock",0);
  if(*bigBlock)           heapError("Zero initialization","bigBlock",0);
  heapStats();

  if(Heap_Free(bigBlock)) heapError("Heap_Free","bigBlock",0);
  heapStats();
  
  UART_OutString("\n\rSuccessful heap test\n\r");
  ST7735_DrawString(0, 0, "Heap test successful", ST7735_YELLOW);
  OS_Kill();
}

void StartHeapTest4(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&TestHeap,128,1)){
      NumCreated++;
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

int Testmain4(void){   // Testmain4
  OS_Init();           // initialize, disable interrupts
  Logic_Init();

	OS_InitSemaphore(&LCDFree, 1);

  // attach background tasks
  OS_AddS2Task(&StartHeapTest4,1);
  OS_AddPA28Task(&StartHeapTest4,1);
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&TestHeap,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}

//*****************Test project 5*************************
// Testmain2 must have been successfully run (it creates Blinky on disk)
// Load Blinky from disk
// Allocate space for data, stack code segments 
// Add Blinky as a process and execute
// Blinky does no OS calls and never returns

void ProcessLoadTest5(void){
  UART_OutString("\n\rECE445M Lab 5 load program\n\r");
  OS_bWait(&LCDFree);
  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);
  OS_bSignal(&LCDFree);
  NumProcessCreated += OS_LoadProgram("Blinky",2);
  UART_OutString("\n\rOS_LoadProgram ok\n\r");
  OS_Kill();
}

int Testmain5(void){   // Testmain5 
  OS_Init();           // initialize, disable interrupts
  Logic_Init();
   
	OS_InitSemaphore(&LCDFree, 1);
  
  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
 // OS_AddS2Task(&ProcessLoadTest5,1);
 // OS_AddPA28Task(&ProcessLoadTest5,1);
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&ProcessLoadTest5,128,1);  
  NumCreated += OS_AddThread(&Chaos3,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 
  NumProcessCreated = 0;

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*****************Test project 6*************************
// Testmain2 must have been successfully run (it creates Blinky and Prog2 on disk)
// Load Blinky from disk
// Allocate space for data, stack code segments 
// Add Blinky as a process and execute
// Blinky does no OS calls and never returns

void ProcessLoadTest6(void){
  UART_OutString("\n\rECE445M Lab 5 testmain6\n\r");
  OS_bWait(&LCDFree);
  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);
  OS_bSignal(&LCDFree);
  NumProcessCreated += OS_LoadProgram("Blinky",2);
  UART_OutString("\n\rOS_LoadProgram Blinky ok\n\r");
  OS_Kill();
}
void ProcessLoadProg2(void){
  NumProcessCreated += OS_LoadProgram("Prog2",1);
}

int Testmain6(void){   // Testmain6 
  OS_Init();           // initialize, disable interrupts
  Logic_Init();
   
	OS_InitSemaphore(&LCDFree, 1);
  
  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
  OS_AddS2Task(&ProcessLoadProg2,1);
  OS_AddPA28Task(&ProcessLoadProg2,1);
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&ProcessLoadTest6,128,1);  
  NumCreated += OS_AddThread(&Chaos3,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 
  NumProcessCreated = 0;

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


