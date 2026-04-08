/* TFLuna1.c
 * Jonathan Valvano
 * Date: 10/21/2025
   TF Luna TOF distance sensor
 */


#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/TFLuna1.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"

// There are four possible sets of UART1 pins that can be used
// Select the TFLUNA1_RX/TFLUNA1_TX for the UART1 pins to use
// overhead of TFLuna will be 17us/measurement at 100 measurements/sec

// SJ-PM-TF-Luna+A01 on RSLK2 
// Pin
// 1    Red  5V
// 2    Serial TxD: PA8 is UART1 Tx (MSPM0 to TFLuna1)
// 3    Serial RxD: PA9 is UART1 Rx (TFLuna1 to MSPM0) to use PA9, set jumper J14
// 4    black ground
//#define TFLUNA1_RX PA9INDEX
//#define TFLUNA1_TX PA8INDEX


// SJ-PM-TF-Luna+A01 on RTOS sensor board 
// Pin
// 1    Red  5V
// 2    Serial TxD: PA17 is UART1 Tx (MSPM0 to TFLuna1)
// 3    Serial RxD: PA18 is UART1 Rx (TFLuna1 to MSPM0)
// 4    black ground
#define TFLUNA1_RX PA18INDEX
#define TFLUNA1_TX PA17INDEX

// SJ-PM-TF-Luna+A01 
// Pin
// 1    Red  5V
// 2    Serial TxD: PB4 is UART1 Tx (MSPM0 to TFLuna1)
// 3    Serial RxD: PB5 is UART1 Rx (TFLuna1 to MSPM0)
// 4    black ground
//#define TFLUNA1_RX PB5INDEX
//#define TFLUNA1_TX PB4INDEX

// SJ-PM-TF-Luna+A01 
// Pin
// 1    Red  5V
// 2    Serial TxD: PB6 is UART1 Tx (MSPM0 to TFLuna1)
// 3    Serial RxD: PB7 is UART1 Rx (TFLuna1 to MSPM0)
// 4    black ground
//#define TFLUNA1_RX PB7INDEX
//#define TFLUNA1_TX PB6INDEX

uint32_t LostData1;
#define FIFO1_SIZE 16  // usable size is 15
// prototypes for private functions
void     Tx1Fifo_Init(void);
uint32_t Tx1Fifo_Put(uint8_t data);
uint32_t Tx1Fifo_Get(uint8_t *datapt);
void     Rx1Fifo_Init(void);
uint32_t Rx1Fifo_Put(uint8_t data);
uint32_t Rx1Fifo_Get(uint8_t *datapt);
int TFLuna1Index;
// 0 for looking for two 59s
// 2-8 filling the TFLunaDataMessage with 9-byte message
void (*TFLuna1Function)(uint32_t); // data in mm
uint8_t TFLuna1LastByte;
uint32_t TFLuna1Distance; // in mm
uint8_t TFLuna1DataMessage[12]; // 9 byte fixed size message
int BadCheckSum1; // errors

// defined in TFLunaCommon
extern const uint8_t ObtainFirmware[4];
// expected response is 0x5A,0x07,0x01,a,b,c,SU version c.b.a
extern const uint8_t System_Reset[4];
// expected response is 0x5A,0x05,0x02,0x00, 0x60 for success
// expected response is 0x5A,0x05,0x02,0x01, 0x61 for failed
extern const uint8_t Frame_Rate[6];
extern const uint8_t Trigger[4];
extern const uint8_t Format_Standard_cm[5];
extern const uint8_t Format_Pixhawk[5];
extern const uint8_t Format_Standard_mm[5];
extern const uint8_t Output_Enable[5];
extern const uint8_t Output_Disable[5];
extern const uint8_t SaveSettings[4];

// power Domain PD0
// for 80MHz bus clock, UART clock is ULPCLK 40MHz
//------------TFLuna_Init------------
// Initialize the UART1 for 115,200 baud rate (assuming 80 MHz clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: function 0 for debug, user function for real time
// Output: none
void TFLuna1_Init(void (*function)(uint32_t)){
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
  IOMUX->SECCFG.PINCM[TFLUNA1_RX]  = 0x00040082;
  //bit 18 INENA input enable
  //bit 7  PC connected
  //bits 5-0=2 for UART1_Rx

  // configure  alternate UART1 transmit function
  IOMUX->SECCFG.PINCM[TFLUNA1_TX]  = 0x00000082;
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
  TFLuna1Function = function;
  TFLuna1Index = 0; // looking for two 59s
  TFLuna1LastByte = 0;
  TFLuna1DataMessage[0] = 0x59;
  TFLuna1DataMessage[1] = 0x59;
  BadCheckSum1 = 0;
  Tx1Fifo_Init();
  Rx1Fifo_Init();
  LostData1 = 0;
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
  NVIC->ICPR[0] = 1<<13; // UART1 is IRQ 13
  NVIC->ISER[0] = 1<<13;
  NVIC->IP[3] = (NVIC->IP[3]&(~0x0000FF00))|(1<<14);    // set priority (bits 15,14) IRQ 13
  UART1->CTL0 |= 0x01; // enable UART1
}


// copy from hardware RX FIFO to software RX FIFO
// stop when hardware RX FIFO is empty or software RX FIFO is full
void static copyHardwareToSoftware1(void){
  uint8_t letter;
  if(TFLuna1Function==0){ // raw data mode
    while(((UART1->STAT&0x04) == 0)&&(TFLuna1_InStatus() < (FIFO1_SIZE - 1))){
    letter = UART1->RXDATA;
    Rx1Fifo_Put(letter);
    }
  }else{
    while((UART1->STAT&0x04)==0){
      letter = UART1->RXDATA;
      if(TFLuna1Index == 0){
        if((letter == 0x59)&&(TFLuna1LastByte == 0x59)){
          TFLuna1Index = 2; // looking for message
        }
        TFLuna1LastByte = letter;
      }else{
        TFLuna1DataMessage[TFLuna1Index] = letter;
        TFLuna1Index++;
        if(TFLuna1Index == 9){
          TFLuna1LastByte = 0; // get ready for next message
          TFLuna1Index = 0;
          uint8_t check=0;
          for(int i=0;i<8;i++){
            check += TFLuna1DataMessage[i];
          }
          if(check == TFLuna1DataMessage[8]){
            TFLuna1Distance = TFLuna1DataMessage[3]*256+TFLuna1DataMessage[2];
            // call back
            (*TFLuna1Function)(TFLuna1Distance);     // pass distance back to higher level
          }else{
            BadCheckSum1++; // error
          }
        }
      }        
    }
  }
}

//------------TFLuna1_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
uint8_t TFLuna1_InChar(void){uint32_t status;
  uint8_t letter;
  do{
    status = Rx1Fifo_Get(&letter);
  }while(status==0);
  return(letter);
}
// copy from software TX FIFO to hardware TX FIFO
// stop when software TX FIFO is empty or hardware TX FIFO is full
void static copySoftwareToHardware1(void){
  uint8_t letter;
  while(((UART1->STAT&0x80) == 0) && (TFLuna1_OutStatus() > 0)){
    Tx1Fifo_Get(&letter);
    UART1->TXDATA = letter;
  }
}
//------------UART1_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void TFLuna1_OutChar(uint8_t data){
    while(Tx1Fifo_Put(data) == 0){};
    UART1->CPU_INT.IMASK &= ~0x0800;   // disarm TX FIFO interrupt
    copySoftwareToHardware1();
    UART1->CPU_INT.IMASK |= 0x0800;    // rearm TX FIFO interrupt
  }
//------------TFLuna_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void TFLuna1_OutString(uint8_t *pt){
  while(*pt){
    TFLuna1_OutChar(*pt);
    pt++;
  }
}


//------------TFLuna1_SendMessage------------
// Output message, msg[1] is length
// Input: pointer to message to be transferred
// Output: none
void TFLuna1_SendMessage(const uint8_t msg[]){
  for(int i=0; i<msg[1]; i++){
    TFLuna1_OutChar(msg[i]);
  }
}
void TFLuna1_Format_Standard_mm(void){
  TFLuna1_SendMessage(Format_Standard_mm);
}
void TFLuna1_Format_Standard_cm(void){
  TFLuna1_SendMessage(Format_Standard_cm);
}
void TFLuna1_Format_Pixhawk(void){
  TFLuna1_SendMessage(Format_Pixhawk);
}
void TFLuna1_Output_Enable(void){
  TFLuna1_SendMessage(Output_Enable);
}
void TFLuna1_Output_Disable(void){
  TFLuna1_SendMessage(Output_Disable);
}
void TFLuna1_Frame_Rate(void){
  TFLuna1_SendMessage(Frame_Rate);
}
void TFLuna1_SaveSettings(void){
  TFLuna1_SendMessage(SaveSettings);
}
void TFLuna1_System_Reset(void){
  TFLuna1_SendMessage(System_Reset);
}

void UART1_IRQHandler(void){ uint32_t status;
  status = UART1->CPU_INT.IIDX; // reading clears bit in RIS
  if(status == 0x01){   // 0x01 receive timeout
    copyHardwareToSoftware1();
  }else if(status == 0x0B){ // 0x0B receive
    copyHardwareToSoftware1();
  }else if(status == 0x0C){ // 0x0C transmit
    copySoftwareToHardware1();
    if(TFLuna1_OutStatus() == 0){             // software TX FIFO is empty
      UART1->CPU_INT.IMASK &= ~0x0800;    // disable TX FIFO interrupt
    }
  }
}



// Declare state variables for FiFo
//        size, buffer, put and get indexes
int32_t static Tx1PutI; // Index to put new
int32_t static Tx1GetI; // Index of oldest
uint8_t static Tx1Fifo[FIFO1_SIZE];

// *********** Tx1Fifo_Init**********
// Initializes a software Tx1FIFO of a
// fixed size and sets up indexes for
// put and get operations
void Tx1Fifo_Init(void){
  Tx1PutI = Tx1GetI = 0;
}

// *********** Tx1Fifo_Put**********
// Adds an element to the Tx1FIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, TxFIFO is full
uint32_t Tx1Fifo_Put(uint8_t data){
  if(((Tx1PutI+1)&(FIFO1_SIZE-1)) == Tx1GetI){
    return 0;
  }
  Tx1Fifo[Tx1PutI] = data;
  Tx1PutI = (Tx1PutI+1)&(FIFO1_SIZE-1);
  return 1; // success

}

// *********** Tx1Fifo_Get**********
// Gets an element from the Tx1FIFO
// Input: pointer to empty 8-bit variable
// Output: If the Tx1FIFO is empty return 0
//         If the Tx1FIFO has data, remove it, and put in  *datapt, return 1
uint32_t Tx1Fifo_Get(uint8_t *datapt){
  if(Tx1GetI == Tx1PutI){
    return 0;
  }
  *datapt = Tx1Fifo[Tx1GetI];
  Tx1GetI = (Tx1GetI+1)&(FIFO1_SIZE-1);
  return 1; // success
}
//------------TFLuna1_OutStatus------------
// Returns how much data available for reading from Tx1 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t TFLuna1_OutStatus(void){  
 return ((Tx1PutI - Tx1GetI)&(FIFO1_SIZE-1));  
}
int32_t static Rx1PutI; // Index to put new
int32_t static Rx1GetI; // Index of oldest
uint8_t static Rx1Fifo[FIFO1_SIZE];

// *********** Rx1Fifo_Init**********
// Initializes a software RxFIFO of a
// fixed size and sets up indexes for
// put and get operations
void Rx1Fifo_Init(void){
  Rx1PutI = Rx1GetI = 0;
}

// *********** Rx1Fifo_Put**********
// Adds an element to the Rx1FIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, RxFIFO is full
uint32_t Rx1Fifo_Put(uint8_t data){
  if(((Rx1PutI+1)&(FIFO1_SIZE-1)) == Rx1GetI){
    return 0;
  }
  Rx1Fifo[Rx1PutI] = data;
  Rx1PutI = (Rx1PutI+1)&(FIFO1_SIZE-1);
  return 1;
}

// *********** Rx1Fifo_Get**********
// Gets an element from the Rx1FIFO
// Input: pointer to empty 8-bit variable
// Output: If the Rx1FIFO is empty return 0
//         If the Rx1FIFO has data, remove it, and  put in  *datapt, return 1
uint32_t  Rx1Fifo_Get(uint8_t *datapt){
  if(Rx1GetI == Rx1PutI){
    return 0;
  }
  *datapt = Rx1Fifo[Rx1GetI];
  Rx1GetI = (Rx1GetI+1)&(FIFO1_SIZE-1);
  return 1;
}
//------------TFLuna1_InStatus------------
// Returns how much data available for reading from Rx1 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t TFLuna1_InStatus(void){  
 return ((Rx1PutI - Rx1GetI)&(FIFO1_SIZE-1));  
}

