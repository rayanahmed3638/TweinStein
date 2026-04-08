// esp8266.c
// Driver for ESP8266 module to act as a WiFi client or server
// Currently restricted to one incoming or outgoing connection at a time
//
// Steven Prickett (steven.prickett@gmail.com)
// Modified version by Dung Nguyen, Wally Guzman
// Modified by Jonathan Valvano, March 28, 2017
// Consolidated by Andreas Gerstlauer, April 6, 2020 
// Converted to MSPM0G3507 UART1 by Jonathan Valvano, Jan 19, 2026
// Added MSPM0G3507 UART2 by Jonathan Valvano, Jan 26, 2026
/* 
  THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
  OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
  VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
  OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*/

// NOTE: see ESP8266 files in datasheets folder

/* Hardware connections
 Vcc is a separate regulated 3.3V supply with at least 215mA
 /------------------------------\
 |              chip      1   8 |
 | Ant                    2   7 |
 | enna       processor   3   6 |
 |                        4   5 |
 \------------------------------/ 
 Connects MSPM0 
    UART1 on (PA17/PA18) or UART2 on (PB17/PB18)
    Reset on PA25
    Ok to not access PB19 because of the internal pullup in ESP8266
 ESP8266    MSPM0     Motor board version 7
  1 URxD    PA17      UART1 out of MSPM0, into ESP8266 115200 baud
  2 GPIO0             +3.3V for normal operation (ground to flash)
  3 GPIO2   PB19      GPIO, high/float on startup, has internal pullup, can be used for I/O
  4 GND     Gnd       GND (70mA)
  5 UTxD    PA18      UART out of ESP8266, UART1 into MSPM0 115200 baud
  6 Ch_PD             chip select, 10k resistor to 3.3V
  7 Reset   PA25      MSPM0 GPIO output, can issue output low to cause hardware reset
  8 Vcc               regulated 3.3V supply with at least 70mA

 ESP8266    MSPM0     Motor board version 7.1
  1 URxD    PB17      UART2 out of MSPM0, into ESP8266 115200 baud
  2 GPIO0             +3.3V for normal operation (ground to flash)
  3 GPIO2   PB19      GPIO, high/float on startup, has internal pullup, can be used for I/O
  4 GND     Gnd       GND (70mA)
  5 UTxD    PB18      UART out of ESP8266, UART2 into MSPM0 115200 baud
  6 Ch_PD             chip select, 10k resistor to 3.3V
  7 Reset   PA25      MSPM0 GPIO output, can issue output low to cause hardware reset
  8 Vcc               regulated 3.3V supply with at least 70mA
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ti/devices/msp/msp.h>
#include "../inc/UART.h"
#include "../inc/Clock.h"
#include "../inc/FIFO.h"
#include "../RTOS_Labs_common/esp8266.h"
#include "../RTOS_Labs_common/WifiSettings.h"  // access point parameters
#include "../RTOS_Labs_common/OS.h"
#include "../inc/LaunchPad.h"

/*
===========================================================
==========          CONSTANTS                    ==========
===========================================================
*/

#define ESP8266_RST   (1<<25)  // PA25

//#define ESP8266_UART     1   // Motor board v7,  UART1: U1Tx PA17, U1Rx PA18
#define ESP8266_UART     2  // Motor board v7.1, UART2: U2Tx PB17, U2Rx PB18
#if ESP8266_UART==1
#define ESP8266_RX PA18INDEX
#define ESP8266_TX PA17INDEX
#else
#define ESP8266_RX PB18INDEX
#define ESP8266_TX PB17INDEX
#endif
// defined in CriticalSection.s
long StartCritical(void);
void EndCritical(long);

void OS_InitSemaphore(Sema4_t *semaPt, int32_t value);
void OS_Signal(Sema4_t *semaPt);
void OS_Wait(Sema4_t *semaPt); 


#define MAXTRY 1              // number of attempts to send command

// ESP responses
static const char ESP8266_READY_RESPONSE[] = "\r\nready\r\n";
static const char ESP8266_OK_RESPONSE[] = "\r\nOK\r\n";
static const char ESP8266_ERROR_RESPONSE[] = "\r\nERROR\r\n";
static const char ESP8266_FAIL_RESPONSE[] = "\r\nFAIL\r\n";
static const char ESP8266_SENDOK_RESPONSE[] = "\r\nSEND OK\r\n";

/*
=============================================================
==========            GLOBAL VARIABLES             ==========
=============================================================
*/

// Globals for UART driver
bool ESP8266_EchoResponse = false;
bool ESP8266_EchoCommand = false;
void UART_OutCharNonBlock(char data){
  if((TxFifo_Size() >= (TXFIFOSIZE-1))) return;
  UART_OutChar(data);
}
// UART receive control/data & transmit FIFOs (see FIFO.h)
uint32_t LostData1;
// prototypes for private functions
void     ESP8266TxFifo_Init(void);
uint32_t ESP8266TxFifo_Put(char data);
uint32_t ESP8266TxFifo_Get(char *datapt);
uint32_t ESP8266TxFifo_Size(void);
void     ESP8266RxFifo_Init(void);
uint32_t ESP8266RxFifo_Put(char data);
uint32_t ESP8266RxFifo_Get(char *datapt);
uint32_t ESP8266RxFifo_Size(void);
void     ESP8266Rx0Fifo_Init(void);
uint32_t ESP8266Rx0Fifo_Put(char data);
uint32_t ESP8266Rx0Fifo_Get(char *datapt);
uint32_t ESP8266Rx0Fifo_Size(void);

#define FIFOSIZE    1024      // size of the FIFOs (must be power of 2)
#define FIFOSUCCESS 1         // return value on success
#define FIFOFAIL    0         // return value on failure

// ESP8266 state
uint16_t ESP8266_Server = 0;   // server port, if any   
uint16_t ESP8266_ConnectionMux = 0;
uint16_t ESP8266_Segment = 0;  // last segment ID for buffered send

/*
=======================================================================
==========              helper FUNCTIONS                     ==========
=======================================================================
*/

// make letter lower-case
char lc(char letter){
  if((letter>='A')&&(letter<='Z')) letter |= 0x20;
  return letter;
}
// ***** Receive buffer ***
#define RECBUFMAX 16
char ReceiveSearch[16];
uint32_t ReceiveIndex=0;
uint32_t ReceiveSize;
uint32_t ReceiveMode;
void ESP8266_StartReceiveSearch(char *search){
  ReceiveIndex = 0;
  int i=0;
  while(search[i]){
    ReceiveSearch[i] = search[i];
    i++;
  }
  ReceiveSize = i;
  ReceiveMode = 1; // searching
}
char * ESP8266_GetReceiveBuffer(void){
  if(ReceiveMode==3){
    return ReceiveSearch;
  }
  return 0;
}

const char ReceiveDataSearchString[]="+IPD,";
static uint32_t ReceiveDataSearchIndex = 0;
static uint32_t ReceiveDataState = 1;     // 0 to disable filtering
static uint32_t ReceiveDataStream = 0;    // connection ID for received data
static uint32_t ReceiveDataLen = 0;       // receive data packet remaining size
volatile uint32_t ESP8266_DataAvailable = 0;  // received data (about to be) available
volatile uint32_t ESP8266_DataLoss = 0;       // lost data (for debugging)

//-------------------ReceiveDataFilter -------------------
// State machine to filter out received data stream from UART Rx input
// Inputs: character to check 
// Outputs: true if data was filtered out, false otherwise
bool ReceiveDataFilter(char letter){
  switch(ReceiveDataState) {  // Filter FSM
    case 4:   // filter out data and put it into the right receive FIFO
      if(ReceiveDataLen) {
        switch(ReceiveDataStream) {
          case 0: if(ESP8266Rx0Fifo_Put(letter) == FIFOFAIL){  // overflow, data loss
                    ESP8266_DataAvailable--;
                    ESP8266_DataLoss++;
                  } 
                  break;  
          // only one connection currently supported
        }          
        ReceiveDataLen--;
        return true;
      }
      ReceiveDataState = 1;   // restart    
      // fall through to start searching again if data is done
    case 1:  // Look for +IPD
      if (letter == ReceiveDataSearchString[ReceiveDataSearchIndex]){ // match letter?
        ReceiveDataSearchIndex++;
        if(ReceiveDataSearchString[ReceiveDataSearchIndex] == 0){ // end of match string?
          if(ESP8266_ConnectionMux) {
            ReceiveDataState = 2;
          } else {
            ReceiveDataState = 3;
          }
          ReceiveDataSearchIndex = 0;
          ReceiveDataStream = 0;
          ReceiveDataLen = 0;
        }
      } else {
        ReceiveDataSearchIndex = 0; // start over
      }
      return false;
    case 2:  // look for connection ID (separated by comma)
      if(letter >= '0' && letter <= '9') {
        ReceiveDataStream = ReceiveDataStream * 10 + (letter - '0');  // add digit to ID    
      } else if (letter == ',') {
        ReceiveDataState = 3;   // ID completed, move on
      } else {
        ReceiveDataStream = 0;
        ReceiveDataState = 1;   // error, start over
      }
      return false;
    case 3:   // look for receive data size (separated by colon)
      if(letter >= '0' && letter <= '9') {
        ReceiveDataLen = ReceiveDataLen * 10 + (letter - '0');  // add digit to length    
      } else if(letter == ':') {
        ReceiveDataState = 4;   // size complete, move on
        ESP8266_DataAvailable += ReceiveDataLen; // mark as becoming available
      } else {
        ReceiveDataStream = 0;
        ReceiveDataLen = 0;
        ReceiveDataState = 1;   // error, start over
      }
      return false;
  } 
  return false;
}

/*
======================================================================
==========         UART and private FUNCTIONS               ==========
======================================================================
*/

// Preprocessor magic to construct UARTx_ identifiers
#define STR(x)  #x
#define CONCAT(x,y,z) x ## y ## z

#define UART_STR(uart)  "UART" STR(uart) 
#define UART_NAME(uart,identifier)  CONCAT(UART,uart,identifier)

#define UART_ESP8266(identifier) UART_NAME(ESP8266_UART,identifier)


// power Domain PD0
// for 80MHz bus clock, UART1/UART2 clocks are ULPCLK 40MHz

//------------------- ESP8266InitUART-------------------
// Intializes uart needed to communicate with esp8266
// Configure UART for 115200bps operation
// Inputs: RX and/or TX echo for debugging
// Outputs: none
void ESP8266_InitUART(int rx_echo, int tx_echo){ 
  ESP8266TxFifo_Init();
  ESP8266RxFifo_Init();
  ESP8266Rx0Fifo_Init();  
  ESP8266_EchoResponse = rx_echo;
  ESP8266_EchoCommand = tx_echo;
#if ESP8266_UART==1
    // do not reset or activate PortA, already done in LaunchPad_Init
    // RSTCLR to GPIOA and UART1 peripherals
    //   bits 31-24 unlock key 0xB1
    //   bit 1 is Clear reset sticky bit
    //   bit 0 is reset gpio port
 // GPIOA->GPRCM.RSTCTL = (uint32_t)0xB1000003; // called previously
  UART1->GPRCM.RSTCTL = 0xB1000003;
    // Enable power to GPIOA and UART1 peripherals
    // PWREN
    //   bits 31-24 unlock key 0x26
    //   bit 0 is Enable Power
 // GPIOA->GPRCM.PWREN = (uint32_t)0x26000001; // called previously
  UART1->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for uart to power up

 // the following code selects which pins to use
  IOMUX->SECCFG.PINCM[ESP8266_RX]  = 0x00040082;
  //bit 18 INENA input enable
  //bit 7  PC connected
  //bits 5-0=2 for UART1_Rx

  // configure  alternate UART1 transmit function
  IOMUX->SECCFG.PINCM[ESP8266_TX]  = 0x00000082;
  //bit 7  PC connected
  //bits 5-0=2 for UART1_Tx
  
  UART1->CLKSEL = 0x08; // bus clock
  UART1->CLKDIV = 0x00; // no divide
  UART1->CTL0 &= ~0x01; // disable UART1
  UART1->CTL0 = 0x00020018;
   // bit  17    FEN=1    enable FIFO
   // bits 16-15 HSE=00   16x oversampling
   // bit  14    CTSEN=0  no CTS hardware
   // bit  13    RTSEN=0  no RTS hardware
   // bit  12    RTS=0    not RTS
   // bits 10-8  MODE=000 normal
   // bits 6-4   TXE=001  enable TxD
   // bit  3     RXE=1    enable TxD
   // bit  2     LBE=0    no loop back
   // bit  0     ENABLE   0 is disable, 1 to enable
  if(Clock_Freq() == 40000000){
      // 20000000/16 = 1,250,000 Hz
     // Baud = 115200
      //    1,250,000/115200 = 10.8506944444
      //   divider = 10+54/64 = 10.84375
    UART1->IBRD = 10;
    UART1->FBRD = 54; // baud =1,250,000/10.84375 = 115,273.77
  }else if (Clock_Freq() == 32000000){
    // 32000000/16 = 2,000,000
     // Baud = 115200
      //    2,000,000/115200 = 17.3611111
      //   divider = 21+23/64 = 17.359375
    UART1->IBRD = 17;
    UART1->FBRD = 23; // 115,211.52
  }else if (Clock_Freq() == 80000000){
     // 40000000/16 = 2,500,000 Hz
     // Baud = 115200
      //    2,500,000/115200 = 21.701388
      //   divider = 21+45/64 = 21.703125
    UART1->IBRD = 21;
    UART1->FBRD = 45; // baud =2,500,000/21.703125 = 115,191
  }else return;
  UART1->LCRH = 0x00000030;
   // bits 5-4 WLEN=11 8 bits
   // bit  3   STP2=0  1 stop
   // bit  2   EPS=0   parity select
   // bit  1   PEN=0   no parity
   // bit  0   BRK=0   no break
  UART1->CPU_INT.IMASK = 0x0C01;
  // bit 11 TXINT yes
  // bit 10 RXINT yes
  // bit 0  Receive timeout, yes
  UART1->IFLS = 0x0422;
  // bits 11-8 RXTOSEL receiver timeout select 4 (0xF highest)
  // bits 6-4  RXIFLSEL 2 is greater than or equal to half
  // bits 2-0  TXIFLSEL 2 is less than or equal to half

  UART1->CTL0 |= 0x01; // enable UART1
#else
    // do not reset or activate PortB, already done in LaunchPad_Init
    // RSTCLR to GPIOB and UART2 peripherals
    //   bits 31-24 unlock key 0xB1
    //   bit 1 is Clear reset sticky bit
    //   bit 0 is reset gpio port
 // GPIOB->GPRCM.RSTCTL = (uint32_t)0xB1000003; // called previously
  UART2->GPRCM.RSTCTL = 0xB1000003;
    // Enable power to GPIOB and UART2 peripherals
    // PWREN
    //   bits 31-24 unlock key 0x26
    //   bit 0 is Enable Power
 // GPIOB->GPRCM.PWREN = (uint32_t)0x26000001; // called previously
  UART2->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for uart to power up

 // the following code selects which pins to use
  IOMUX->SECCFG.PINCM[ESP8266_RX]  = 0x00040082;
  //bit 18 INENA input enable
  //bit 7  PC connected
  //bits 5-0=2 for UART2_Rx

  // configure  alternate UART2 transmit function
  IOMUX->SECCFG.PINCM[ESP8266_TX]  = 0x00000082;
  //bit 7  PC connected
  //bits 5-0=2 for UART2_Tx
  
  UART2->CLKSEL = 0x08; // bus clock
  UART2->CLKDIV = 0x00; // no divide
  UART2->CTL0 &= ~0x01; // disable UART2
  UART2->CTL0 = 0x00020018;
   // bit  17    FEN=1    enable FIFO
   // bits 16-15 HSE=00   16x oversampling
   // bit  14    CTSEN=0  no CTS hardware
   // bit  13    RTSEN=0  no RTS hardware
   // bit  12    RTS=0    not RTS
   // bits 10-8  MODE=000 normal
   // bits 6-4   TXE=001  enable TxD
   // bit  3     RXE=1    enable TxD
   // bit  2     LBE=0    no loop back
   // bit  0     ENABLE   0 is disable, 1 to enable
  if(Clock_Freq() == 40000000){
      // 20000000/16 = 1,250,000 Hz
     // Baud = 115200
      //    1,250,000/115200 = 10.8506944444
      //   divider = 10+54/64 = 10.84375
    UART2->IBRD = 10;
    UART2->FBRD = 54; // baud =1,250,000/10.84375 = 115,273.77
  }else if (Clock_Freq() == 32000000){
    // 32000000/16 = 2,000,000
     // Baud = 115200
      //    2,000,000/115200 = 17.3611111
      //   divider = 21+23/64 = 17.359375
    UART2->IBRD = 17;
    UART2->FBRD = 23; // 115,211.52
  }else if (Clock_Freq() == 80000000){
     // 40000000/16 = 2,500,000 Hz
     // Baud = 115200
      //    2,500,000/115200 = 21.701388
      //   divider = 21+45/64 = 21.703125
    UART2->IBRD = 21;
    UART2->FBRD = 45; // baud =2,500,000/21.703125 = 115,191
  }else return;
  UART2->LCRH = 0x00000030;
   // bits 5-4 WLEN=11 8 bits
   // bit  3   STP2=0  1 stop
   // bit  2   EPS=0   parity select
   // bit  1   PEN=0   no parity
   // bit  0   BRK=0   no break
  UART2->CPU_INT.IMASK = 0x0C01;
  // bit 11 TXINT yes
  // bit 10 RXINT yes
  // bit 0  Receive timeout, yes
  UART2->IFLS = 0x0422;
  // bits 11-8 RXTOSEL receiver timeout select 4 (0xF highest)
  // bits 6-4  RXIFLSEL 2 is greater than or equal to half
  // bits 2-0  TXIFLSEL 2 is less than or equal to half

  UART2->CTL0 |= 0x01; // enable UART2
#endif  

}

//--------ESP8266_EnableInterrupt--------
// Enables uart interrupt
// Inputs: none
// Outputs: none
void ESP8266_EnableInterrupt(void){
#if ESP8266_UART==1
  NVIC->ICPR[0] = 1<<13; // UART1 is IRQ 13
  NVIC->ISER[0] = 1<<13;
  NVIC->IP[3] = (NVIC->IP[3]&(~0x0000FF00))|(1<<14);    // set priority (bits 15,14) IRQ 13
#else
  NVIC->ICPR[0] = 1<<14; // UART2 is IRQ 14
  NVIC->ISER[0] = 1<<14;
  NVIC->IP[3] = (NVIC->IP[3]&(~0x00FF0000))|(1<<22);    // set priority (bits 23,22) IRQ 14
#endif    
}

//--------ESP8266_DisableInterrupt--------
// Disables uart interrupt
// Inputs: none
// Outputs: none
void ESP8266_DisableInterrupt(void){
#if ESP8266_UART==1
  NVIC->ICER[0] = 1<<13; // UART1 is IRQ 13
#else
  NVIC->ICER[0] = 1<<14; // UART2 is IRQ 14
#endif    
}

//----------ESP8266BufferToTx----------
// Copies TX buffer (software defined FIFO) to uart
// Inputs: none
// Outputs:none
void static ESP8266BufferToTx(void){
  char letter;
#if ESP8266_UART==1
  while(((UART1->STAT&0x80) == 0) && (ESP8266TxFifo_Size() > 0)){
    ESP8266TxFifo_Get(&letter);
    if(ESP8266_EchoCommand){
      UART_OutCharNonBlock(letter); // echo
    }
    UART1->TXDATA = letter;
  }
#else
  while(((UART2->STAT&0x80) == 0) && (ESP8266TxFifo_Size() > 0)){
    ESP8266TxFifo_Get(&letter);
    if(ESP8266_EchoCommand){
      UART_OutCharNonBlock(letter); // echo
    }
    UART2->TXDATA = letter;
  }
#endif    
}


//----------ESP8266RxToBuffer----------
// Copies uart fifo to RX buffer (software defined FIFO)
// Inputs: none
// Outputs:none
void static ESP8266RxToBuffer(void){
  char letter;
#if ESP8266_UART==1
  while(((UART1->STAT&0x04) == 0)&&((ESP8266RxFifo_Size() < (FIFOSIZE - 1)))){
    letter = UART1->RXDATA;
    if(ESP8266_EchoResponse){
      UART_OutCharNonBlock(letter); // echo
      if(ReceiveBufferIndex<RECBUFMAX){
        ReceiveBuffer[ReceiveBufferIndex] = letter;
        ReceiveBufferIndex++;
        ReceiveBuffer[ReceiveBufferIndex] = 0;
      }
    }
    if(!ReceiveDataFilter(letter)) {
      ESP8266RxFifo_Put(letter);
    }
  }
#else
  while(((UART2->STAT&0x04) == 0)&&((ESP8266RxFifo_Size() < (FIFOSIZE - 1)))){
    letter = UART2->RXDATA;
    if(ESP8266_EchoResponse){
      UART_OutCharNonBlock(letter); // echo
      if(ReceiveMode==1){// searching
        if(ReceiveSearch[ReceiveIndex] == letter){
          ReceiveIndex++;
          if(ReceiveIndex == ReceiveSize){
            ReceiveMode=2;
          }
        }else{
          ReceiveIndex = 0; // no match
        }
      }else{
        if(ReceiveMode==2){ // found
          if(letter>='A'){
            ReceiveSearch[ReceiveIndex] = letter;
            ReceiveIndex++;
          }else{
            ReceiveMode = 3; // done
            ReceiveSearch[ReceiveIndex] = 0;
          }
        }
      }
    }
    if(!ReceiveDataFilter(letter)) {
      ESP8266RxFifo_Put(letter);
    }
  }
#endif
}

//----------UART_Handler----------
// At least one of three things has happened:
// hardware TX FIFO goes from 3 to 2 or less items
// hardware RX FIFO goes from 1 to 2 or more items
// UART receiver has timed out
#if ESP8266_UART==1
void UART1_IRQHandler(void){ uint32_t status;
  status = UART1->CPU_INT.IIDX; // reading clears bit in RIS
  if(status == 0x01){   // 0x01 receive timeout
    ESP8266RxToBuffer();
  }else if(status == 0x0B){ // 0x0B receive
    ESP8266RxToBuffer();
  }else if(status == 0x0C){ // 0x0C transmit
    ESP8266BufferToTx();
    if(ESP8266TxFifo_Size() == 0){             // software TX FIFO is empty
      UART1->CPU_INT.IMASK &= ~0x0800;    // disable TX FIFO interrupt
    }
  }
}
#else
void UART2_IRQHandler(void){ uint32_t status;
  status = UART2->CPU_INT.IIDX; // reading clears bit in RIS
  if(status == 0x01){   // 0x01 receive timeout
    ESP8266RxToBuffer();
  }else if(status == 0x0B){ // 0x0B receive
    ESP8266RxToBuffer();
  }else if(status == 0x0C){ // 0x0C transmit
    ESP8266BufferToTx();
    if(ESP8266TxFifo_Size() == 0){             // software TX FIFO is empty
      UART2->CPU_INT.IMASK &= ~0x0800;    // disable TX FIFO interrupt
    }
  }
}
#endif


//--------ESP8266_OutChar--------
// Prints a character to the esp8226 via uart
// Inputs: character to transmit
// Outputs: none
void ESP8266_OutChar(char data){
  ESP8266TxFifo_Put(data); // blocks via semaphore if FIFO full
#if ESP8266_UART==1
  UART1->CPU_INT.IMASK &= ~0x0800;   // disarm TX FIFO interrupt
  ESP8266BufferToTx();
  UART1->CPU_INT.IMASK |= 0x0800;    // rearm TX FIFO interrupt
#else
  UART2->CPU_INT.IMASK &= ~0x0800;   // disarm TX FIFO interrupt
  ESP8266BufferToTx();
  UART2->CPU_INT.IMASK |= 0x0800;    // rearm TX FIFO interrupt
#endif
}

//--------ESP8266_InChar--------
// Read a character from the esp8226 via uart
// Inputs: none
// Outputs: character received
char ESP8266_InChar(void){ char letter;
  ESP8266RxFifo_Get(&letter); // blocks via semaphore if FIFO empty
  return(letter);
}

//---------ESP8266_SendCommand-----
// Sends a string to the esp8266 module
// Inputs: string to send (null-terminated)
// Outputs: none
void ESP8266_SendCommand(const char* command){
  int index = 0;
  while(command[index] != 0){
    ESP8266_OutChar(command[index++]);
  }
}

//---------ESP8266_WaitForResponse-----
// Busy-wait until response found
// Inputs: Success or failure strings to search for
// Outputs: 1 on success, 0 on failure
int ESP8266_WaitForResponse(const char *success, const char* failure) {
  char d;
  const char *s = success;
  const char *f = failure;
  while((!s || *s) && (!f || *f)) {   // end of search string reached?
    d = ESP8266_InChar(); 
    if(s && (d == *s)) {
      s++;
    } else {
      s = success;  // start over
      if(s && (d == *s)) s++;  // match first char?
    }
    if(f && (d == *f)) {
      f++;
    } else {
      f = failure;  // start over
      if(f && (d == *f)) f++;  // match first char? 
    }
  }
  if(failure && !(*f)) return FAILURE;
  return SUCCESS;  
}

/*
=======================================================================
==========          ESP8266 PUBLIC FUNCTIONS                 ==========
=======================================================================
*/

//-------------------ESP8266_Init --------------
// Initializes the module
// Inputs: RX and/or TX echo for debugging
// Outputs: 1 for success, 0 for failure (no ESP detected)
int ESP8266_Init(int rx_echo, int tx_echo){ char c; const char* s; uint32_t timer = 1;
  // Disable interrupt during initialization
  ESP8266_DisableInterrupt();

  // Initialize UART to communicate with ESP
  ESP8266_InitUART(rx_echo, tx_echo);
  
  // Initialize reset port on PA25
  IOMUX->SECCFG.PINCM[PA25INDEX]  = (uint32_t) 0x00000081;
  GPIOA->DOE31_0 |= (1<<25);

  // Hardware reset
  GPIOA->DOUTCLR31_0 = (1<<25); // reset low
  Clock_Delay1ms(10);
  GPIOA->DOUTSET31_0 = (1<<25); // reset high
  
  // Wait for ready status with timeout
  // Use low-level UART communication, interrupts disabled
  s = ESP8266_READY_RESPONSE;
  timer = 5000000;  // around a 1-2s total timeout
  while(*s) {
#if ESP8266_UART==1
    while(timer && ((UART1->STAT&0x04) == 0x04)) { timer--; }
    if(!timer) break;  
    c = (char)UART1->RXDATA;
#else
    while(timer && ((UART2->STAT&0x04) == 0x04)) { timer--; }
    if(!timer) break;  
    c = (char)UART2->RXDATA;
#endif
    if(rx_echo) UART_OutCharNonBlock(c); // echo, requires UART0 to be operational, will print garbage
    if(c == *s) {
      s++;
    } else {
      s = ESP8266_READY_RESPONSE;
      if(c == *s) s++;
    }
  }
  
  // Finally enable interrupt
  ESP8266_EnableInterrupt();
  
  if(!timer) return FAILURE;
  return SUCCESS;
}

//-------------------ESP8266_Connect --------------
// Bring interface up and connect to Wifi
// Inputs: enable debug output
// Outputs: 1 on success, 0 on failure
int ESP8266_Connect(int verbose){
  if(ESP8266_Reset()==FAILURE) return FAILURE; 
    
  if(verbose)  // debug output: MAC address oF ESP8266
    ESP8266_GetMACAddress(); 

#if SOFTAP
  if(ESP8266_SoftAccessPoint(SSID_NAME,PASSKEY)==FAILURE) return FAILURE; 
  if(ESP8266_SetWifiMode(ESP8266_WIFI_MODE_AP)==FAILURE) return FAILURE; 
#else
  if(ESP8266_SetWifiMode(ESP8266_WIFI_MODE_CLIENT)==FAILURE) return FAILURE; 

  if(verbose)  // debug output: see APs in area
    ESP8266_ListAccessPoints(); 
  
  if(ESP8266_JoinAccessPoint(SSID_NAME,PASSKEY)==FAILURE) return FAILURE; 
#endif

  if(verbose)  // debug output: our IP address
    ESP8266_GetIPAddress();
  
  return SUCCESS;
}

//-------------------ESP8266_StartServer --------------
// Start server on specific port
// Inputs: port and server timeout
// Outputs: 1 on success, 0 on failure
int ESP8266_StartServer(uint16_t port, uint16_t timeout){
  if(ESP8266_SetConnectionMux(1)==FAILURE) return FAILURE;
  if(ESP8266_EnableServer(port)==FAILURE) return FAILURE;
  if(ESP8266_SetServerTimeout(timeout)==FAILURE) return FAILURE; 
  return SUCCESS;
}

//-------------------ESP8266_StopServer --------------
// Stop server and set to single-client mode
// Inputs: none
// Outputs: 1 on success, 0 on failure
int ESP8266_StopServer(void){
  if(ESP8266_DisableServer()==FAILURE) return FAILURE;
  if(ESP8266_SetConnectionMux(0)==FAILURE) return FAILURE;
  return SUCCESS;
}

//----------ESP8266_Reset------------
// Soft resets the esp8266 module
// Input:  none
// Output: 1 if success, 0 if fail
int ESP8266_Reset(void){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+RST\r\n");
    if(ESP8266_WaitForResponse(ESP8266_READY_RESPONSE,0)) return SUCCESS;
    try--;
  }
  return FAILURE; 
}

//---------ESP8266_Restore-----
// Restore the ESP8266 module to default values
// Inputs: none
// Outputs: 1 if success, 0 if fail
int ESP8266_Restore(void) { int try=MAXTRY;
  while(try){
  	ESP8266_SendCommand("AT+RESTORE\r\n");
    if(ESP8266_WaitForResponse(ESP8266_READY_RESPONSE,0)) return SUCCESS;
    try--;
  }
  return FAILURE; 
}


//---------ESP8266_GetVersionNumber----------
// Get status
// Input: none
// Output: 1 if success, 0 if fail 
int ESP8266_GetVersionNumber(void){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+GMR\r\n");
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) return SUCCESS;
    try--;
  }
  return FAILURE; // fail
}

//---------ESP8266_GetMACAddress----------
// Get MAC address
// Input: none
// Output: 1 if success, 0 if fail 
int ESP8266_GetMACAddress(void){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+CIPSTAMAC?\r\n");
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) return SUCCESS;
    try--;
  }
  return FAILURE; // fail
}

//---------ESP8266_SetWifiMode----------
// Configures the esp8266 to operate as a wifi client, access point, or both
// Input: mode accepts ESP8266_WIFI_MODE constants
// Output: 1 if success, 0 if fail 
int ESP8266_SetWifiMode(uint8_t mode){ int try=MAXTRY;
  char TXBuffer[32];
  while(try){
    sprintf(TXBuffer, "AT+CWMODE=%d\r\n", mode);
    ESP8266_SendCommand(TXBuffer);
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) return SUCCESS;
    try--;
  }
  return FAILURE; 
}
 
//---------ESP8266_SetConnectionMux----------
// Enables the esp8266 connection mux, required for starting tcp server
// Input: 0 (single) or 1 (multiple)
// Output: 1 if success, 0 if fail 
int ESP8266_SetConnectionMux(uint8_t enabled){ int try=MAXTRY;
  //char TXBuffer[32];
  while(try){
    if(enabled){
    //sprintf(TXBuffer, "AT+CIPMUX=%d\r\n", enabled);
    ESP8266_SendCommand("AT+CIPMUX=1\r\n");
    }else{
    ESP8266_SendCommand("AT+CIPMUX=0\r\n");

    }
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) {
      ESP8266_ConnectionMux = enabled;
      return SUCCESS;
    }
    try--;
  }
  return FAILURE;
}

//---------ESP8266_ListAccessPoints----------
// Lists available wifi access points
// Input: none
// Output: 1 if success, 0 if fail 
int ESP8266_ListAccessPoints(void){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+CWLAP\r\n");
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_ERROR_RESPONSE)) return SUCCESS;
    try--;
  }
  return FAILURE;
}

//----------ESP8266_JoinAccessPoint------------
// Joins a wifi access point using specified ssid and password
// Input:  SSID and PASSWORD
// Output: 1 if success, 0 if fail
int ESP8266_JoinAccessPoint(const char* ssid, const char* password){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+CWJAP=\"");
    ESP8266_SendCommand(ssid);
    ESP8266_SendCommand("\",\"");
    ESP8266_SendCommand(password);
    ESP8266_SendCommand("\"\r\n");    
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_FAIL_RESPONSE)) return SUCCESS;
    try--;
  }
  return FAILURE; 
}

// ----------ESP8266_QuitAccessPoint-------------
// Disconnects from currently connected wifi access point
// Inputs: none
// Outputs: 1 if success, 0 if fail 
int ESP8266_QuitAccessPoint(void){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+CWQAP\r\n");
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) return SUCCESS;
    try--;
  }
  return FAILURE; 
}

//----------ESP8266_ConfigureAccessPoint------------
// Configures esp8266 wifi soft access point settings
// Use this function only when in AP mode (and not in client mode)
// Input:  SSID, Password, channel, security
// Output: 1 if success, 0 if fail
int ESP8266_ConfigureAccessPoint(const char* ssid, const char* password, uint8_t channel, uint8_t encryptMode){
  int try=MAXTRY;
  char TXBuffer[32];
  while(try){
    ESP8266_SendCommand("AT+CWSAP=\"");
    ESP8266_SendCommand(ssid);
    ESP8266_SendCommand("\",\"");
    ESP8266_SendCommand(password);
    sprintf(TXBuffer, "\",%d,%d\r\n", channel, encryptMode);
    ESP8266_SendCommand(TXBuffer);
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_ERROR_RESPONSE)) return SUCCESS;
    try--;
  }
  return FAILURE;
}

//---------ESP8266_GetIPAddress----------
// Get local IP address
// Input: none
// Output: 1 if success, 0 if fail 
int ESP8266_GetIPAddress(void){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+CIFSR\r\n");   
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_ERROR_RESPONSE)) return SUCCESS;
    try--;
  }
  return FAILURE; 
}

//---------ESP8266_SetSSLClientConfiguration----------
// Set SSL client configuration
// Requires certificates to be flashed into the ESP firmware
// Inputs: enable/disable client/server certificate checks
// output: 1 if success, 0 if fail 
int ESP8266_SetSSLClientConfiguration(int verifyClient, int verifyServer){ int try=MAXTRY;
  char TXBuffer[32];
  while(try){
    sprintf(TXBuffer, "AT+CIPSSLCCONF=%d\r\n", (verifyClient? 1 : 0) + (verifyServer? 2 : 0));
    ESP8266_SendCommand(TXBuffer);
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_ERROR_RESPONSE)) return SUCCESS;
    try--;
  }
  return FAILURE;
}

//---------ESP8266_SetSSLBufferSize----------
// Set SSL buffer size
// Inputs: buffer size between 2048 and 4096
// output: 1 if success, 0 if fail 
int ESP8266_SetSSLBufferSize(uint16_t bufferSize){ int try=MAXTRY;
  char TXBuffer[32];
  while(try){
    sprintf(TXBuffer, "AT+CIPSSLSIZE=%d\r\n", bufferSize);
    ESP8266_SendCommand(TXBuffer);
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_ERROR_RESPONSE)) return SUCCESS;
    try--;
  }
  return FAILURE;
}

//---------ESP8266_MakeTCPConnection----------
// Establish TCP or SSL connection
// The ESP only seems to have limited SSL support, does not work with all servers
// Inputs: IP address or web page as a string, port, and keepalive time (0 if none)
// output: 1 if success, 0 if fail 
int ESP8266_MakeTCPConnection(char *IPaddress, uint16_t port, uint16_t keepalive, int ssl){ int try=MAXTRY;
  char TXBuffer[32];
  while(try){
    if(ssl){
      ESP8266_SendCommand("AT+CIPSTART=\"SSL\",\"");
    } else {
      ESP8266_SendCommand("AT+CIPSTART=\"TCP\",\"");
    }
    ESP8266_SendCommand(IPaddress);
    if(keepalive) {
      sprintf(TXBuffer, "\",%d,%d\r\n", port, keepalive);
    } else {
      sprintf(TXBuffer, "\",%d\r\n", port);
    }
    ESP8266_SendCommand(TXBuffer);   // open and connect to a socket
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_ERROR_RESPONSE)) return SUCCESS;
    try--;
  }
  return FAILURE; 
}

//---------ESP8266_Send----------
// Send a string to server 
// Input: payload to send
// Output: 1 if success, 0 if fail 
int ESP8266_Send(const char* fetch){
  char TXBuffer[32];
  if(ESP8266_ConnectionMux) {
    sprintf(TXBuffer, "AT+CIPSEND=%d,%d\r\n", 0, strlen(fetch));
  } else {
    sprintf(TXBuffer, "AT+CIPSEND=%d\r\n", strlen(fetch));
  }
  ESP8266_SendCommand(TXBuffer);  
  if(!ESP8266_WaitForResponse(">",ESP8266_ERROR_RESPONSE)) return FAILURE;
  ESP8266_SendCommand(fetch);
  if(ESP8266_WaitForResponse(ESP8266_SENDOK_RESPONSE,ESP8266_ERROR_RESPONSE)) return SUCCESS;
  return FAILURE; 
}

//---------ESP8266_SendBuffered----------
// Send a string to server using ESP TCP-send buffer
// Input: payload to send
// Output: 1 if success, 0 if fail 
int ESP8266_SendBuffered(const char* fetch){
  char TXBuffer[32];
  if(ESP8266_ConnectionMux) {
    sprintf(TXBuffer, "AT+CIPSENDBUF=%d,%d\r\n", 0, strlen(fetch));
  } else {
    sprintf(TXBuffer, "AT+CIPSENDBUF=%d\r\n", strlen(fetch));
  }
  ESP8266_SendCommand(TXBuffer);  
  if(!ESP8266_WaitForResponse(">",ESP8266_ERROR_RESPONSE)) return FAILURE;
  ESP8266_Segment++;
  ESP8266_SendCommand(fetch);
  sprintf(TXBuffer, "Recv %d bytes", strlen(fetch));
  if(ESP8266_WaitForResponse(TXBuffer,ESP8266_ERROR_RESPONSE)) return SUCCESS;
  return FAILURE; 
}

//---------ESP8266_SendBufferedStatus----------
// Check status of last buffered segment
// Input: none
// Output: 1 if success, 0 if fail 
int ESP8266_SendBufferedStatus(void){
  char OKBuffer[16];
  char FailBuffer[24];
  if(ESP8266_ConnectionMux) {
    sprintf(OKBuffer, "\n%d,%d,SEND OK\r\n", 0, ESP8266_Segment);
    sprintf(FailBuffer, "\n%d,%d,SEND FAIL\r\n", 0, ESP8266_Segment);
  } else {
    sprintf(OKBuffer, "\n%d,SEND OK\r\n", ESP8266_Segment);
    sprintf(FailBuffer, "\n%d,SEND FAIL\r\n", ESP8266_Segment);
  }
  if(ESP8266_WaitForResponse(OKBuffer,FailBuffer)) return SUCCESS;
  return FAILURE; 
}

//---------ESP8266_Receive----------
// Receive a string from server 
// Reads from data input until end of line or max length is reached
// Input: buffer and max length
// Output: 1 and null-terminated string if success, 0 if fail (disconnected)
int ESP8266_Receive(char* fetch, uint32_t max){ long sr; const char* s;
  char letter;
  while(max > 1) {
    if(ESP8266Rx0Fifo_Size() || ESP8266_DataAvailable) { // data (about to be) available?
      ESP8266Rx0Fifo_Get(&letter); // blocks via semaphore if FIFO empty
      // ESP8266_DisableInterrupt();  // critical section
      sr = StartCritical();  
      if(ESP8266_DataAvailable) ESP8266_DataAvailable--;
      EndCritical(sr); 
      // ESP8266_EnableInterrupt();         
      if(letter == '\r') continue;
      if(letter == '\n') break;
      *fetch = letter;
      fetch++;
      max--;
    } else {  // wait for next packet or connection close
      if(ESP8266_ConnectionMux) {
        s = "0,CLOSED";
      } else {
        s = "CLOSED";
      }
      if(!ESP8266_WaitForResponse(ReceiveDataSearchString,s)){
        *fetch = 0;       // connection closed
        return FAILURE;
      }
      while(ESP8266_InChar() != ':') {}  // wait for DataAvailable to be updated
    }        
  }
  *fetch = 0;  // terminate with null character
  return SUCCESS;
}

//---------ESP8266_CloseTCPConnection----------
// Close TCP connection 
// Input: none
// Output: 1 if success, 0 if fail 
int ESP8266_CloseTCPConnection(void){ int try=MAXTRY;
  while(try){
    if(ESP8266_ConnectionMux) {
      ESP8266_SendCommand("AT+CIPCLOSE=0\r\n");
    } else {
      ESP8266_SendCommand("AT+CIPCLOSE\r\n");
    }      
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_ERROR_RESPONSE)) {
      ESP8266Rx0Fifo_Init();  // Discard any data
      return SUCCESS;
    }
    try--;
  }
  ESP8266Rx0Fifo_Init();  // Discard any data
  return FAILURE;
}

//---------ESP8266_SetDataTransmissionMode----------
// Set data transmission passthrough mode
// Input: 0 not data mode, 1 data mode; return "Link is builded" 
// Output: 1 if success, 0 if fail 
int ESP8266_SetDataTransmissionMode(uint8_t mode){ int try=MAXTRY;
  char TXBuffer[32];
  while(try){
    sprintf(TXBuffer, "AT+CIPMODE=%d\r\n", mode);
    ESP8266_SendCommand(TXBuffer);
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) return SUCCESS;
    try--;
  }
  return FAILURE;
}

//---------ESP8266_GetStatus----------
// Get network connection status
// Input: none
// Output: 1 if success, 0 if fail 
int ESP8266_GetStatus(void){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+CIPSTATUS\r\n");
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) return SUCCESS;
    try--;
  }
  return FAILURE;
}

// --------ESP8266_EnableServer------------------
// Enables tcp server on specified port
// Inputs: port number
// Outputs: 1 if success, 0 if fail
int ESP8266_EnableServer(uint16_t port){  int try=MAXTRY;
  char TXBuffer[32];
  while(try){
    sprintf(TXBuffer, "AT+CIPSERVER=1,%d\r\n", port);
    ESP8266_SendCommand(TXBuffer);
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) {
      ESP8266_Server = port;
      return SUCCESS;
    }
    try--;
  }
  return FAILURE;
}

// ----------ESP8266_SetServerTimeout--------------
// Set connection timeout for tcp server, 0-28800 seconds
// Inputs: timeout parameter
// Outputs: 1 if success, 0 if fail
int ESP8266_SetServerTimeout(uint16_t timeout){ int try=MAXTRY;
  char TXBuffer[32];
  while(try){
    sprintf(TXBuffer, "AT+CIPSTO=%d\r\n", timeout);
    ESP8266_SendCommand(TXBuffer);
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,ESP8266_ERROR_RESPONSE)) return SUCCESS;
    try--;
  }
  return FAILURE;
}

// ----------ESP8266_WaitForConnection--------------
// Wait for incoming connection on server
// must ensure that no other ESP calls are done while waiting
// this should really be done in the background interrupt handler
// using a mailbox to communicate with this function
// Inputs: none
// Outputs: 1 if success, 0 if fail
int ESP8266_WaitForConnection(void){
  if(!ESP8266_ConnectionMux) return FAILURE;
  if(!ESP8266_Server) return FAILURE;
  if(ESP8266_WaitForResponse("0,CONNECT\r\n",0)) return SUCCESS;
  return FAILURE;
}

//---------ESP8266_DisableServer----------
// Disables tcp server
// Input: none 
// Output: 1 if success, 0 if fail 
int ESP8266_DisableServer(void){ int try=MAXTRY;
  while(try){
    ESP8266_SendCommand("AT+CIPSERVER=0\r\n"); 
    if(ESP8266_WaitForResponse(ESP8266_OK_RESPONSE,0)) {
      ESP8266_Server = 0;
      return SUCCESS;
    }
    try--;
  }
  return FAILURE;
}

// Declare state variables for FiFo
//        size, buffer, put and get indexes
// Lab 6 change: Added semaphores for OS synchronization.
//   TxFifo: foreground threads Put (block if full), ISR Gets (signals space available)
//   RxFifo: ISR Puts (signals data available), foreground threads Get (block if empty)
//   Rx0Fifo: ISR Puts (signals data available), foreground threads Get (block if empty)
int32_t static ESP8266TxPutI; // Index to put new
int32_t static ESP8266TxGetI; // Index of oldest
char static ESP8266TxFifo[FIFOSIZE];
Sema4_t ESP8266TxSpaceAvailable; // Lab 6: counting semaphore for free space

// *********** ESP8266TxFifo_Init**********
// Initializes a software ESP8266TxFIFO of a
// fixed size and sets up indexes for
// put and get operations
// Lab 6 change: Also initializes TxSpaceAvailable semaphore to FIFOSIZE-1
void ESP8266TxFifo_Init(void){
  ESP8266TxPutI = ESP8266TxGetI = 0;
  OS_InitSemaphore(&ESP8266TxSpaceAvailable, FIFOSIZE-1);
}

// *********** ESP8266TxFifo_Put**********
// Adds an element to the ESP8266TxFIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, TxFIFO is full
// Lab 6 change: Now blocks via OS_Wait if FIFO is full (called by foreground thread)
uint32_t ESP8266TxFifo_Put(char data){
  OS_Wait(&ESP8266TxSpaceAvailable);
  ESP8266TxFifo[ESP8266TxPutI] = data;
  ESP8266TxPutI = (ESP8266TxPutI+1)&(FIFOSIZE-1);
  return SUCCESS;
}

// *********** ESP8266TxFifo_Get**********
// Gets an element from the ESP8266TxFIFO
// Input: pointer to empty 8-bit variable
// Output: If the ESP8266TxFIFO is empty return 0
//         If the ESP8266TxFIFO has data, remove it, and put in  *datapt, return 1
// Lab 6 change: Calls OS_Signal after removing element to wake blocked Put callers (called by ISR)
uint32_t ESP8266TxFifo_Get(char *datapt){
  if(ESP8266TxGetI == ESP8266TxPutI){
    return FAILURE;
  }
  *datapt = ESP8266TxFifo[ESP8266TxGetI];
  ESP8266TxGetI = (ESP8266TxGetI+1)&(FIFOSIZE-1);
  OS_Signal(&ESP8266TxSpaceAvailable);
  return SUCCESS;
}

//------------ESP8266TxFifo_Size------------
// Returns how much data available for reading from Tx1 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t ESP8266TxFifo_Size(void){
  return ((ESP8266TxPutI - ESP8266TxGetI)&(FIFOSIZE-1));
}


int32_t static ESP8266RxPutI; // Index to put new
int32_t static ESP8266RxGetI; // Index of oldest
char static ESP8266RxFifo[FIFOSIZE];
Sema4_t ESP8266RxDataAvailable; // Lab 6: counting semaphore for available data

// *********** ESP8266RxFifo_Init**********
// Initializes a software RxFIFO of a
// fixed size and sets up indexes for
// put and get operations
// Lab 6 change: Also initializes RxDataAvailable semaphore to 0
void ESP8266RxFifo_Init(void){
  ESP8266RxPutI = ESP8266RxGetI = 0;
  OS_InitSemaphore(&ESP8266RxDataAvailable, 0);
}

// *********** ESP8266RxFifo_Put**********
// Adds an element to the ESP8266RxFIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, RxFIFO is full
// Lab 6 change: Calls OS_Signal after adding element to wake blocked Get callers (called by ISR)
uint32_t ESP8266RxFifo_Put(char data){
  if(((ESP8266RxPutI+1)&(FIFOSIZE-1)) == ESP8266RxGetI){
    return FAILURE;
  }
  ESP8266RxFifo[ESP8266RxPutI] = data;
  ESP8266RxPutI = (ESP8266RxPutI+1)&(FIFOSIZE-1);
  OS_Signal(&ESP8266RxDataAvailable);
  return SUCCESS;
}

// *********** ESP8266RxFifo_Get**********
// Gets an element from the ESP8266RxFIFO
// Input: pointer to empty 8-bit variable
// Output: If the ESP8266RxFIFO is empty return 0
//         If the ESP8266RxFIFO has data, remove it, and  put in  *datapt, return 1
// Lab 6 change: Now blocks via OS_Wait if FIFO is empty (called by foreground thread)
uint32_t ESP8266RxFifo_Get(char *datapt){
  OS_Wait(&ESP8266RxDataAvailable);
  *datapt = ESP8266RxFifo[ESP8266RxGetI];
  ESP8266RxGetI = (ESP8266RxGetI+1)&(FIFOSIZE-1);
  return SUCCESS;
}
//------------ESP8266RxFifo_Size------------
// Returns how much data available for reading from Rx FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t ESP8266RxFifo_Size(void){
  return ((ESP8266RxPutI - ESP8266RxGetI)&(FIFOSIZE-1));
}


int32_t static ESP8266Rx0PutI; // Index to put new
int32_t static ESP8266Rx0GetI; // Index of oldest
char static ESP8266Rx0Fifo[FIFOSIZE];
Sema4_t ESP8266Rx0DataAvailable; // Lab 6: counting semaphore for available data

// *********** ESP8266Rx0Fifo_Init**********
// Initializes a software RxFIFO of a
// fixed size and sets up indexes for
// put and get operations
// Lab 6 change: Also initializes Rx0DataAvailable semaphore to 0
void ESP8266Rx0Fifo_Init(void){
  ESP8266Rx0PutI = ESP8266Rx0GetI = 0;
  OS_InitSemaphore(&ESP8266Rx0DataAvailable, 0);
}

// *********** ESP8266Rx0Fifo_Put**********
// Adds an element to the ESP8266Rx0FIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, RxFIFO is full
// Lab 6 change: Calls OS_Signal after adding element to wake blocked Get callers (called by ISR)
uint32_t ESP8266Rx0Fifo_Put(char data){
  if(((ESP8266Rx0PutI+1)&(FIFOSIZE-1)) == ESP8266Rx0GetI){
    return FAILURE;
  }
  ESP8266Rx0Fifo[ESP8266Rx0PutI] = data;
  ESP8266Rx0PutI = (ESP8266Rx0PutI+1)&(FIFOSIZE-1);
  OS_Signal(&ESP8266Rx0DataAvailable);
  return SUCCESS;
}

// *********** ESP8266Rx0Fifo_Get**********
// Gets an element from the ESP8266Rx0FIFO
// Input: pointer to empty 8-bit variable
// Output: If the ESP8266Rx0FIFO is empty return 0
//         If the ESP8266Rx0FIFO has data, remove it, and  put in  *datapt, return 1
// Lab 6 change: Now blocks via OS_Wait if FIFO is empty (called by foreground thread)
uint32_t ESP8266Rx0Fifo_Get(char *datapt){
  OS_Wait(&ESP8266Rx0DataAvailable);
  *datapt = ESP8266Rx0Fifo[ESP8266Rx0GetI];
  ESP8266Rx0GetI = (ESP8266Rx0GetI+1)&(FIFOSIZE-1);
  return SUCCESS;
}
//------------ESP8266Rx0Fifo_Size------------
// Returns how much data available for reading from Rx0 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t ESP8266Rx0Fifo_Size(void){
  return ((ESP8266Rx0PutI - ESP8266Rx0GetI)&(FIFOSIZE-1));
}


