/* Testermain.c
 * Jonathan Valvano
 * November 3, 2025
 * Derived from uart_rw_multibyte_fifo_poll_LP_MSPM0G3507_nortos_ticlang
 */


#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/UART.h"
#include <stdio.h>

// PA10 is UART0 Tx    index 20 in IOMUX PINCM table
// PA11 is UART0 Rx    index 21 in IOMUX PINCM table
//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S1 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S2 negative logic switch,  index 48 in IOMUX PINCM table
// Remove J16 J17 J18: Disconnects light sensor from the microcontroller PA22 PA27 PA26
// J9: Remove J9: Disconnects thermistor from PB24
// J14: Connect PB23 to SW1
// J15: Connect PA16 to SW2
// J4: Connects PA0 to red LED
// Insert jumper J25: Connects PA10 to XDS_UART
// Insert jumper J26: Connects PA11 to XDS_UART
#define debug 0
struct pin{
  char name[8];
  GPIO_Regs *port; // GPIOA or GPIOB
  uint32_t pincm;  // IOMUX->SECCFG.PINCM[xxx]
  uint32_t mask;   // selects bit
};
typedef const struct pin pin_t;

pin_t GroupA[6]={
{"PB3",  GPIOB, PB3INDEX,  1<<3},
{"PB2",  GPIOB, PB2INDEX,  1<<2},
{"PA27", GPIOA, PA27INDEX, 1<<27},
{"PB6",  GPIOB, PB6INDEX,  1<<6},
{"PB0",  GPIOB, PB0INDEX,  1<<0},
{"PB16", GPIOB, PB16INDEX, 1<<16}
};
pin_t GroupB[8]={
{"PB8",  GPIOB, PB8INDEX,  1<<8},
{"PB7",  GPIOB, PB7INDEX,  1<<7},
{"PA25", GPIOA, PA25INDEX, 1<<25},
{"PB9",  GPIOB, PB9INDEX,  1<<9},
{"PA28", GPIOA, PA28INDEX, 1<<28},
{"PB12", GPIOB, PB12INDEX, 1<<12},
{"PA8",  GPIOA, PA8INDEX,  1<<8},
{"PB23", GPIOB, PB23INDEX, 1<<23}
};
pin_t GroupC[3]={
//{"PA10", GPIOA, PA10INDEX, 1<<10}, // UART
{"PB13", GPIOB, PB13INDEX, 1<<13},
{"PB20", GPIOB, PB20INDEX, 1<<20},
{"PA31", GPIOA, PA31INDEX, 1<<31}
};

pin_t GroupD[5]={
{"PA12", GPIOA, PA12INDEX, 1<<12},
//{"PA11", GPIOA, PA11INDEX, 1<<11}, // UART
{"PA18", GPIOA, PA18INDEX, 1<<18},
{"PB18", GPIOB, PB18INDEX, 1<<18},
{"PA22", GPIOA, PA22INDEX, 1<<22},
{"PB19", GPIOB, PB19INDEX, 1<<19}
};

pin_t GroupE[6]={
{"PA26", GPIOA, PA26INDEX, 1<<26},
{"PB24", GPIOB, PB24INDEX, 1<<24},
{"PA16", GPIOA, PA16INDEX, 1<<16},
{"PA17", GPIOA, PA17INDEX, 1<<17},
{"PA24", GPIOA, PA24INDEX, 1<<24},
{"PB17", GPIOB, PB17INDEX, 1<<17},
};
pin_t GroupF[5]={
{"PA13", GPIOA, PA13INDEX, 1<<13},
{"PB1",  GPIOB, PB1INDEX,  1<<1},
{"PB4",  GPIOB, PB4INDEX,  1<<4},
{"PA15", GPIOA, PA15INDEX, 1<<15},
{"PB15", GPIOB, PB15INDEX, 1<<15}
};
uint32_t ErrCount[8];
void Test(pin_t G[], uint32_t size){ uint32_t in,err;
  err = 0;
  for(int i=0; i<8; i++) ErrCount[i]=0;
  UART_OutString("\n\rTesting pins: ");
  for(int i=0; i<size-1;i++){
    UART_OutString((char*)G[i].name);
    UART_OutChar(',');
  }
  UART_OutString((char*)G[size-1].name);
  for(int i=0; i<size; i++){
#if debug
    UART_OutString("\n\rMake "); UART_OutString((char*)G[i].name); UART_OutString(" an output, others are input.");
#endif
    IOMUX->SECCFG.PINCM[G[i].pincm]  = 0x00040081; // input and output
    G[i].port->DOE31_0 |= G[i].mask;
    for(int j=0; j<size; j++){
      if(i != j){
        IOMUX->SECCFG.PINCM[G[j].pincm]  = 0x00040081; // input
        G[j].port->DOE31_0 &= ~G[j].mask;
      }
    }
    Clock_Delay(80000);
    G[i].port->DOUTSET31_0 = G[i].mask; // output high
    Clock_Delay(80000);
    for(int j=0; j<size; j++){
      if(i != j){
        in = G[j].port->DIN31_0&G[j].mask;
        if(in == 0){
          err++;
          ErrCount[i]++; ErrCount[j]++;
          UART_OutString("\n\r*** ");
          UART_OutString((char*)G[i].name); UART_OutString(" is high, but ");
          UART_OutString((char*)G[j].name); UART_OutString(" is low!");
        }
      }
    }
    G[i].port->DOUTCLR31_0 = G[i].mask; // output low
    Clock_Delay(80000);
    for(int j=0; j<size; j++){
      if(i != j){
        in = G[j].port->DIN31_0&G[j].mask;
        if(in != 0){
          err++;
          ErrCount[i]++; ErrCount[j]++;
          UART_OutString("\n\r*** ");
          UART_OutString((char*)G[i].name); UART_OutString(" is low, but ");
          UART_OutString((char*)G[j].name); UART_OutString(" is high!");
        }
      }
    }
  }

  if(err == 0){
      UART_OutString(" Passed!");
  }else{
    UART_OutString("\n\rFailed\n\r");
    for(int i=0; i<size; i++){
      if(ErrCount[i] > 2){
        UART_OutString((char*)G[i].name);
        UART_OutString(" had ");
        UART_OutUDec(ErrCount[i]);
        UART_OutString(" errors");
      }
    }
  }
}



int main(void){
  Clock_Init80MHz(0);
  LaunchPad_Init();
  UART_InitPrintf();
  UART_OutString("\n\rLP-MSPM0G3507 tester\n\rValvano, Dec 12, 2023");
  Test(GroupA,6);
  Test(GroupB,8);
  Test(GroupC,3);
  Test(GroupD,5);
  Test(GroupE,6);
  Test(GroupF,5);
  UART_OutString("\n\rRed LED1 should be flashing");
  UART_OutString("\n\rRed/Green/Blue LED should be flash Red if S2, Green if S3, Blue if both\n\r");
  while(1){
    LaunchPad_LED1on();
    if(LaunchPad_InS1()&&(LaunchPad_InS2()==0)){
      LaunchPad_LED(RED);
    }else if((LaunchPad_InS1()==0)&&(LaunchPad_InS2())){
      LaunchPad_LED(GREEN);
    }else if((LaunchPad_InS1())&&(LaunchPad_InS2())){
      LaunchPad_LED(BLUE);
    }else{
      LaunchPad_LEDoff();
    }
    Clock_Delay1ms(500);
    LaunchPad_LED1off();
    LaunchPad_LEDoff();
    Clock_Delay1ms(500);
  }
}


uint32_t in1,in2;
void TestPB23(void){
 // IOMUX->SECCFG.PINCM[PB23INDEX]  = 0x00040081; // output
 // IOMUX->SECCFG.PINCM[PB8INDEX]  = 0x00000081; // input
  IOMUX->SECCFG.PINCM[PB23INDEX]  = 0x00000081; // output
  IOMUX->SECCFG.PINCM[PB8INDEX]  = 0x00040081; // input
  IOMUX->SECCFG.PINCM[PB7INDEX]  = 0x00040081;
  IOMUX->SECCFG.PINCM[PA25INDEX]  = 0x00040081;
  IOMUX->SECCFG.PINCM[PB9INDEX]  = 0x00040081;
  IOMUX->SECCFG.PINCM[PA28INDEX]  = 0x00040081;
  IOMUX->SECCFG.PINCM[PB12INDEX]  = 0x00040081;
  IOMUX->SECCFG.PINCM[PA8INDEX]  = 0x00040081;
  for(int j=0; j<7; j++){
    GroupB[j].port->DOE31_0 &= ~GroupB[j].mask;
  }
  GPIOB->DOE31_0 |= 1<<23;

  while(1){
    GPIOB->DOUTSET31_0 = 1<<23;
    Clock_Delay(80000);
    in1 = GPIOB->DIN31_0;
    GPIOB->DOUTCLR31_0 = 1<<23;
    Clock_Delay(160000);
    in2 = GPIOB->DIN31_0;
  }
}

