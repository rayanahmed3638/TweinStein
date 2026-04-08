/* RTOS_Lab1.c
 * Jonathan Valvano
 * December 27, 2025
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)
 */

#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/LaunchPad.h"
#include "../inc/ADC.h"
#include "../inc/DAC.h"
#include "../inc/Clock.h"
#include "../inc/Timer.h"
#include "../RTOS_Labs_common/RTOS_UART.h"
#include "../RTOS_Labs_common/SPI.h"
#include "../RTOS_Labs_common/ST7735_SDC.h"
#include "../RTOS_Labs_common/Interpreter.h"
#include "../RTOS_Labs_common/IRDistance.h"
#include "../RTOS_Labs_common/TFLuna2.h"
#include "../RTOS_Labs_common/LPF.h"
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

// Logic analyzer pins
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

// ******************Task1*******************
// Sample PA24 Center ADC0_3, calculate distance from analog IR sensor
int32_t ADCdata,FilterOutput,Distance;
uint32_t Index;
#define BUFSIZE 100
uint32_t DataBuf[BUFSIZE]; // 0 to 4095 assuming constant analog input
int32_t TheSignal,TheNoise,SNR,sum,sumsq;
  // TheSignal is the average of BUFSIZE Distance samples
  // TheNoise is the standard deviation of the BUFSIZE Distance samples (0.01)
  // SNR = signal/noise 
// periodic task, runs at 10Hz in background
void TIMG6_IRQHandler(void){
  if((TIMG6->CPU_INT.IIDX) == 1){ // this will acknowledge
    TogglePA8();          // toggle PA8
    ADCdata = ADC0_In();  // channel set when calling ADC0_Init
    TogglePA8();          // toggle PA8
    // FilterOutput = ADCdata; // no filter
    FilterOutput = Median(ADCdata); // 3-wide median filter
    // FilterOutput = LPF_Calc7(ADCdata); // 16-wide average filter
    Distance = IRDistance_Convert(FilterOutput,0); // in mm
    DataBuf[Index] = Distance;
    sum = sum+Distance;
    Index++;        // calculation finished
    if(Index>=BUFSIZE){
      Index = 0;
      TheSignal = sum/BUFSIZE;       // units 1
      sumsq = 0;
      for(int i=0; i<BUFSIZE; i++){int32_t v;
        v = 100*(DataBuf[i]-TheSignal);
        sumsq = sumsq+v*v;
      }
      TheNoise = sqrt2(sumsq/BUFSIZE);   // units 0.01
      SNR = (100*TheSignal)/TheNoise;    // units 1
      sum = 0;  
    }
    TogglePA8();          // toggle PA8
   }
}

// ******************Task2*******************
// TFLuna2, calculate distance from TOF sensor
// Runs about 100 Hz using UART1 interrupts
uint32_t Distance2; // mm
uint32_t Count2;
uint32_t Index2;
#define BUFSIZE2 100
uint32_t DataBuf2[BUFSIZE2]; // distance in mm
int32_t TheSignal2,TheNoise2,SNR2,sum2,sumsq2;
void Background2(uint32_t d){
  TogglePA9();          // toggle PA9
  TogglePA9();          // toggle PA9
  Distance2 = d;
  Count2++;
  DataBuf2[Index2] = Distance2;
  sum2 = sum2+Distance2;
  Index2++;        // calculation finished
  if(Index2 >= BUFSIZE2){
    Index2 = 0;
    TheSignal2 = sum2/BUFSIZE2;       // units 1
    sumsq2 = 0;
    for(int i=0; i<BUFSIZE2; i++){int32_t v;
      v = 100*(DataBuf2[i]-TheSignal2);
      sumsq2 = sumsq2+v*v;
    }
    TheNoise2 = sqrt2(sumsq2/BUFSIZE2);   // units 0.01
    SNR2 = (100*TheSignal2)/TheNoise2;  // units 1
    sum2 = 0;   
  }
  TogglePA9();          // toggle PA9
}

int32_t ADCdata3,Voltage3;
uint32_t Index3;
#define BUFSIZE3 100
uint32_t DataBuf3[BUFSIZE3]; // 0 to 4095 assuming constant analog input
int32_t TheSignal3,TheNoise3,SNR3,sum3,sumsq3;
  // TheSignal3 is the average of BUFSIZE3 Voltage3 samples
  // TheNoise3 is the standard deviation of the BUFSIZE3 ADC samples (0.01)
  // SNR3 = signal/noise 
  // periodic task
// runs at 100Hz in background
void TIMA0_IRQHandler(void){
  if((TIMA0->CPU_INT.IIDX) == 1){ // this will acknowledge
    TogglePA16();          // toggle PA16
    ADCdata3 = ADC1_In();  // channel set when calling ADC1_Init
    TogglePA16();          // toggle PA16
    Voltage3 = (3300*ADCdata3)>>12; // in mV
    DataBuf3[Index3] = Voltage3;
    sum3 = sum3+Voltage3;
    Index3++;        // calculation finished
    if(Index3>=BUFSIZE3){
      Index3 = 0;
      TheSignal3 = sum3/BUFSIZE3;       // units 1
      sumsq3 = 0;
      for(int i=0; i<BUFSIZE3; i++){int32_t v;
        v = 100*(DataBuf3[i]-TheSignal3);
        sumsq3 = sumsq3+v*v;
      }
      TheNoise3 = sqrt2(sumsq3/BUFSIZE2);   // units 0.01
      SNR3 = (100*TheSignal3)/TheNoise3;    // units 1
      sum3 = 0;   
    }
    TogglePA16();          // toggle PA16
  }
}

void Lab1_Results(uint32_t d){ // call this from your intepreter
	UART_OutString("\r\nLab 1 performance data");
  UART_OutString("\r\nDistance= "); UART_OutUDec(TheSignal);
  UART_OutString("\r\nTheNoise= "); UART_OutUDec(TheNoise);
  UART_OutString("\r\nSNR=      "); UART_OutUDec(SNR);
  UART_OutString("\r\nDistince2="); UART_OutUDec(TheSignal2);
  UART_OutString("\r\nTheNoise2="); UART_OutUDec(TheNoise2);
  UART_OutString("\r\nSNR2=     "); UART_OutUDec(SNR2);
  UART_OutString("\r\nVoltage3= "); UART_OutUDec(TheSignal3);
  UART_OutString("\r\nTheNoise3="); UART_OutUDec(TheNoise3);
  UART_OutString("\r\nSNR3=     "); UART_OutUDec(SNR3);
  UART_OutString("\r\nTime(s)=  "); UART_OutUDec(OS_MsTime()/1000);
	ST7735_Message(d,0,"ADC      = ",ADCdata);
	ST7735_Message(d,1,"Distance = ",TheSignal);
	ST7735_Message(d,2,"SNR      = ",SNR);
	ST7735_Message(d,3,"Destance2= ",TheSignal2);
	ST7735_Message(d,4,"SNR2     = ",SNR2);
	ST7735_Message(d,5,"DAC      = ",TheSignal3);
	ST7735_Message(d,6,"SNR3     = ",SNR3);
	ST7735_Message(d,7,"Time(s)  = ",OS_MsTime()/1000);
}
int main(void){
  __disable_irq();
  Clock_Init80MHz(0); // no clock out to pin
  LaunchPad_Init();
  Logic_Init();
  ADC0_Init(3,ADCVREF_VDDA);  // PA24 Center ADC0_3
  LPF_Init7(2048,16);
  DAC_Init();
  DAC_Out(2048); // 1.25V
  ADC1_Init(0,ADCVREF_VDDA);  // PA15 ADC1_0, also DACout
  TimerG6_IntArm(50000,80,2); // 80MHz/80/50000 = 10Hz
  TimerA0_IntArm(10000,80,2); // 80MHz/80/10000 = 100Hz
  OS_ClearMsTime();    // start a periodic interrupt to maintain time
  ST7735_InitR(INITR_BLACKTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("RTOS Lab 1\nSpring 2026\n");
  UART_Init(1); // hardware priority 1
  LPF_Init7(500,7);
  TFLuna2_Init(&Background2);
  TFLuna2_Format_Standard_mm(); // format in mm
  TFLuna2_Frame_Rate();         // 100 samples/sec
  TFLuna2_SaveSettings();  // save format and rate
  TFLuna2_System_Reset();  // start measurements
  __enable_irq();
  UART_OutString("ECE445M Lab 1\n\rSpring 2026\n\r");
  Interpreter();     // finally, launch interpreter, should never return
  while(1){

  }
}

// Wait for S2 press and release (debounced)
void WaitForS2(void){
  while(LaunchPad_InS2() == 0){}   // Wait for press
  Clock_Delay1ms(20);              // Debounce
  while(LaunchPad_InS2() != 0){}   // Wait for release
  Clock_Delay1ms(20);              // Debounce
}

// main2: Comprehensive tests on ST7735_Message
int main2(){
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ST7735_InitR(INITR_BLACKTAB);
  ST7735_FillScreen(ST7735_BLACK);

  // Test 1: Integer boundary values
  ST7735_SetCursor(0, 0);
  ST7735_OutString("=== Boundary Tests ===");
  ST7735_SetCursor(0, 2);
  ST7735_OutString("Press S2 to start");
  WaitForS2();
  ST7735_FillScreen(ST7735_BLACK);

  ST7735_Message(0, 0, "INT_MAX:", 2147483647);
  ST7735_Message(0, 1, "INT_MIN:", -2147483647);
  ST7735_Message(0, 2, "Zero:     ", 0);
  ST7735_Message(0, 3, "Neg one:  ", -1);
  ST7735_Message(0, 4, "Pos one:  ", 1);
  ST7735_Message(0, 5, "Large neg:", -999999999);
  ST7735_Message(0, 6, "Large pos:", 999999999);
  ST7735_Message(0, 7, "Mid value:", 12345678);
  ST7735_SetCursor(0, 15);
  ST7735_OutString("S2 for next test");
  WaitForS2();

  // Test 2: All lines on both devices
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("=== All Lines Test ===");
  ST7735_SetCursor(0, 1);
  ST7735_OutString("S2 to continue");
  WaitForS2();
  ST7735_FillScreen(ST7735_BLACK);

  for(int d = 0; d < 2; d++){
    for(int l = 0; l < 8; l++){
      ST7735_Message(d, l, "Dev Line: ", d*10 + l);
    }
  }
  WaitForS2();

  // Test 3: Overwrite same line - check for artifacts
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("=== Overwrite Test ===");
  ST7735_SetCursor(0, 1);
  ST7735_OutString("Watch line 3");
  ST7735_SetCursor(0, 2);
  ST7735_OutString("S2 cycles values");
  WaitForS2();

  // Large number then small - should clear old digits
  ST7735_Message(0, 3, "Value: ", 999999999);
  WaitForS2();
  ST7735_Message(0, 3, "Value: ", 5);
  WaitForS2();
  ST7735_Message(0, 3, "Value: ", -888888888);
  WaitForS2();
  ST7735_Message(0, 3, "Value: ", 0);
  WaitForS2();
  ST7735_Message(0, 3, "Value: ", -1);
  WaitForS2();

  // Test 4: Rapid updates on same line
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("=== Rapid Update ===");
  ST7735_SetCursor(0, 1);
  ST7735_OutString("Counting 0-99");
  ST7735_SetCursor(0, 2);
  ST7735_OutString("S2 to start");
  WaitForS2();
  for(int i = 0; i < 100; i++){
    ST7735_Message(0, 3, "Count: ", i);
    Clock_Delay1ms(50);
  }
  ST7735_SetCursor(0, 5);
  ST7735_OutString("S2 for next test");
  WaitForS2();

  // Test 5: String length variations
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("=== String Tests ===");
  ST7735_SetCursor(0, 1);
  ST7735_OutString("S2 to continue");
  WaitForS2();
  ST7735_FillScreen(ST7735_BLACK);

  ST7735_Message(0, 0, "", 42);                    // Empty string
  ST7735_Message(0, 1, "A", 1);                    // Single char
  ST7735_Message(0, 2, "Short:", 2);               // Normal
  ST7735_Message(0, 3, "Medium string: ", 3);      // Medium
  ST7735_Message(0, 4, "This is a longer string", 4);  // Long
  ST7735_SetCursor(0, 15);
  ST7735_OutString("S2 for next test");
  WaitForS2();

  // Test 6: Positive/negative pairs
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("=== Pos/Neg Pairs ===");
  ST7735_SetCursor(0, 1);
  ST7735_OutString("S2 to continue");
  WaitForS2();
  ST7735_FillScreen(ST7735_BLACK);

  int32_t testValues[] = {0, 1, 10, 100, 1000, 10000, 100000, 1000000};
  for(int i = 0; i < 8; i++){
    ST7735_Message(0, i, "Pos: ", testValues[i]);
    ST7735_Message(1, i, "Neg: ", -testValues[i]);
  }
  WaitForS2();

  // Test 7: Out-of-bounds parameters
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("=== Bounds Test ===");
  ST7735_SetCursor(0, 1);
  ST7735_OutString("Next screen\nshould be blank");
  WaitForS2();

  ST7735_FillScreen(ST7735_BLACK);
  ST7735_Message(2, 0, "Bad device:", 123);   // device out of bounds
  ST7735_Message(0, 8, "Bad line:", 456);     // line out of bounds
  ST7735_Message(0, 255, "Max line:", 789);   // extreme line value
  ST7735_SetCursor(0, 15);
  ST7735_OutString("S2 to finish");

  WaitForS2();

  // Final screen
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("=== Tests Done ===");
  ST7735_SetCursor(0, 2);
  ST7735_OutString("Verify visually:");
  ST7735_SetCursor(0, 3);
  ST7735_OutString("- No artifacts");
  ST7735_SetCursor(0, 4);
  ST7735_OutString("- Correct values");
  ST7735_SetCursor(0, 5);
  ST7735_OutString("- Lines cleared");

  while(1){
    
  }
}

// main3 performs simple tests on OS_ClearMsTime and OS_MsTime
int main3(){
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ST7735_InitR(INITR_BLACKTAB);
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("OS Time Testing");

  // Simple 1 millisecond test
  OS_ClearMsTime();
  Clock_Delay1ms(1);
  ST7735_Message(0, 1, "After 1 ms: ", OS_MsTime());

  // 100 millisecond test
  OS_ClearMsTime();
  Clock_Delay1ms(100);
  ST7735_Message(0, 2, "After 100 ms: ", OS_MsTime());

  // 1 second test
  OS_ClearMsTime();
  Clock_Delay1ms(1000);
  ST7735_Message(0, 3, "After 1 s: ", OS_MsTime());

  // 10 second test
  OS_ClearMsTime();
  Clock_Delay1ms(10000);
  ST7735_Message(0, 4, "After 10 s: ", OS_MsTime());

  // 30 second test
  OS_ClearMsTime();
  Clock_Delay1ms(30000);
  ST7735_Message(0, 5, "After 30 s: ", OS_MsTime());

  // Reset test                                                                                                                                                                                                                                                                                              
  OS_ClearMsTime();                                                                                                                                                                                                                                                                                             
  ST7735_Message(0, 6, "After reset: ", OS_MsTime());                                               

  ST7735_SetCursor(0, 15);
  ST7735_OutString("Testing complete.");

  while (1) {
    
  }
}