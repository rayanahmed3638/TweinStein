/* TFLuna2.c
 * Jonathan Valvano
 * Date: 12/12/2025
   TF Luna TOF distance sensor
 */


#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/TFLuna2.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"

// There are four possible sets of UART2 pins that can be used
// Select the TFLUNA2_RX/TFLUNA2_TX for the UART2 pins to use
// overhead of TFLuna will be 17us/measurement at 100 measurements/sec

// SJ-PM-TF-Luna+A01 on RTOS sensor board 
// Pin
// 1    Red  5V
// 2    Serial TxD: PB17 is UART2 Tx (MSPM0 to TFLuna2)
// 3    Serial RxD: PB18 is UART2 Rx (TFLuna2 to MSPM0)
// 4    black ground
#define TFLUNA2_RX PB18INDEX
#define TFLUNA2_TX PB17INDEX

// SJ-PM-TF-Luna+A01 
// Pin
// 1    Red  5V
// 2    Serial TxD: PA21 is UART2 Tx (MSPM0 to TFLuna2)
// 3    Serial RxD: PA22 is UART2 Rx (TFLuna2 to MSPM0) 
// 4    black ground
//#define TFLUNA2_RX PA22INDEX
//#define TFLUNA2_TX PA21INDEX

// SJ-PM-TF-Luna+A01 
// Pin
// 1    Red  5V
// 2    Serial TxD: PA23 is UART2 Tx (MSPM0 to TFLuna2)
// 3    Serial RxD: PA24 is UART2 Rx (TFLuna2 to MSPM0)
// 4    black ground
//#define TFLUNA2_RX PA24INDEX
//#define TFLUNA2_TX PA23INDEX

// SJ-PM-TF-Luna+A01 
// Pin
// 1    Red  5V
// 2    Serial TxD: PB15 is UART2 Tx (MSPM0 to TFLuna2)
// 3    Serial RxD: PB16 is UART2 Rx (TFLuna2 to MSPM0)
// 4    black ground
//#define TFLUNA2_RX PB16INDEX
//#define TFLUNA2_TX PB15INDEX

uint32_t LostData2;
#define FIFO2_SIZE 16  // usable size is 15
// prototypes for private functions
void     Tx2Fifo_Init(void);
uint32_t Tx2Fifo_Put(uint8_t data);
uint32_t Tx2Fifo_Get(uint8_t *datapt);
void     Rx2Fifo_Init(void);
uint32_t Rx2Fifo_Put(uint8_t data);
uint32_t Rx2Fifo_Get(uint8_t *datapt);
int TFLuna2Index;
// 0 for looking for two 59s
// 2-8 filling the TFLunaDataMessage with 9-byte message
void (*TFLuna2Function)(uint32_t); // data in mm
uint8_t TFLuna2LastByte;
uint32_t TFLuna2Distance; // in mm
uint8_t TFLuna2DataMessage[12]; // 9 byte fixed size message
int BadCheckSum2; // errors

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
// Initialize the UART2 for 115,200 baud rate (assuming 80 MHz clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: function 0 for debug, user function for real time
// Output: none
void TFLuna2_Init(void (*function)(uint32_t)){
    // do not reset or activate PortA, already done in LaunchPad_Init
    // RSTCLR to GPIOA and UART2 peripherals
    //   bits 31-24 unlock key 0xB1
    //   bit 1 is Clear reset sticky bit
    //   bit 0 is reset gpio port
 // GPIOA->GPRCM.RSTCTL = (uint32_t)0xB1000003; // called previously
  UART2->GPRCM.RSTCTL = 0xB1000003;
    // Enable power to GPIOA and UART2 peripherals
    // PWREN
    //   bits 31-24 unlock key 0x26
    //   bit 0 is Enable Power
 // GPIOA->GPRCM.PWREN = (uint32_t)0x26000001; // called previously
  UART2->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for uart to power up

 // the following code selects which pins to use
  IOMUX->SECCFG.PINCM[TFLUNA2_RX]  = 0x00040082;
  //bit 18 INENA input enable
  //bit 7  PC connected
  //bits 5-0=2 for UART2_Rx

  // configure  alternate UART2 transmit function
  IOMUX->SECCFG.PINCM[TFLUNA2_TX]  = 0x00000082;
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
  TFLuna2Function = function;
  TFLuna2Index = 0; // looking for two 59s
  TFLuna2LastByte = 0;
  TFLuna2DataMessage[0] = 0x59;
  TFLuna2DataMessage[1] = 0x59;
  BadCheckSum2 = 0;
  Tx2Fifo_Init();
  Rx2Fifo_Init();
  LostData2 = 0;
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
  NVIC->ICPR[0] = 1<<14; // UART2 is IRQ 13
  NVIC->ISER[0] = 1<<14;
  NVIC->IP[3] = (NVIC->IP[3]&(~0x00FF0000))|(1<<22);    // set priority (bits 23-22) IRQ 14
  UART2->CTL0 |= 0x01; // enable UART2
}


// copy from hardware RX FIFO to software RX FIFO
// stop when hardware RX FIFO is empty or software RX FIFO is full
void static copyHardwareToSoftware2(void){
  uint8_t letter;
  if(TFLuna2Function==0){ // raw data mode
    while(((UART2->STAT&0x04) == 0)&&(TFLuna2_InStatus() < (FIFO2_SIZE - 1))){
    letter = UART2->RXDATA;
    Rx2Fifo_Put(letter);
    }
  }else{
    while((UART2->STAT&0x04)==0){
      letter = UART2->RXDATA;
      if(TFLuna2Index == 0){
        if((letter == 0x59)&&(TFLuna2LastByte == 0x59)){
          TFLuna2Index = 2; // looking for message
        }
        TFLuna2LastByte = letter;
      }else{
        TFLuna2DataMessage[TFLuna2Index] = letter;
        TFLuna2Index++;
        if(TFLuna2Index == 9){
          TFLuna2LastByte = 0; // get ready for next message
          TFLuna2Index = 0;
          uint8_t check=0;
          for(int i=0;i<8;i++){
            check += TFLuna2DataMessage[i];
          }
          if(check == TFLuna2DataMessage[8]){
            TFLuna2Distance = TFLuna2DataMessage[3]*256+TFLuna2DataMessage[2];
            // call back
            (*TFLuna2Function)(TFLuna2Distance);     // pass distance back to higher level
          }else{
            BadCheckSum2++; // error
          }
        }
      }        
    }
  }
}

//------------TFLuna2_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
uint8_t TFLuna2_InChar(void){uint32_t status;
  uint8_t letter;
  do{
    status = Rx2Fifo_Get(&letter);
  }while(status==0);
  return(letter);
}
// copy from software TX FIFO to hardware TX FIFO
// stop when software TX FIFO is empty or hardware TX FIFO is full
void static copySoftwareToHardware2(void){
  uint8_t letter;
  while(((UART2->STAT&0x80) == 0) && (TFLuna2_OutStatus() > 0)){
    Tx2Fifo_Get(&letter);
    UART2->TXDATA = letter;
  }
}
//------------UART2_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void TFLuna2_OutChar(uint8_t data){
    while(Tx2Fifo_Put(data) == 0){};
    UART2->CPU_INT.IMASK &= ~0x0800;   // disarm TX FIFO interrupt
    copySoftwareToHardware2();
    UART2->CPU_INT.IMASK |= 0x0800;    // rearm TX FIFO interrupt
  }
//------------TFLuna_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void TFLuna2_OutString(uint8_t *pt){
  while(*pt){
    TFLuna2_OutChar(*pt);
    pt++;
  }
}


//------------TFLuna2_SendMessage------------
// Output message, msg[1] is length
// Input: pointer to message to be transferred
// Output: none
void TFLuna2_SendMessage(const uint8_t msg[]){
  for(int i=0; i<msg[1]; i++){
    TFLuna2_OutChar(msg[i]);
  }
}
void TFLuna2_Format_Standard_mm(void){
  TFLuna2_SendMessage(Format_Standard_mm);
}
void TFLuna2_Format_Standard_cm(void){
  TFLuna2_SendMessage(Format_Standard_cm);
}
void TFLuna2_Format_Pixhawk(void){
  TFLuna2_SendMessage(Format_Pixhawk);
}
void TFLuna2_Output_Enable(void){
  TFLuna2_SendMessage(Output_Enable);
}
void TFLuna2_Output_Disable(void){
  TFLuna2_SendMessage(Output_Disable);
}
void TFLuna2_Frame_Rate(void){
  TFLuna2_SendMessage(Frame_Rate);
}
void TFLuna2_SaveSettings(void){
  TFLuna2_SendMessage(SaveSettings);
}
void TFLuna2_System_Reset(void){
  TFLuna2_SendMessage(System_Reset);
}

void UART2_IRQHandler(void){ uint32_t status;
  status = UART2->CPU_INT.IIDX; // reading clears bit in RIS
  if(status == 0x01){   // 0x01 receive timeout
    copyHardwareToSoftware2();
  }else if(status == 0x0B){ // 0x0B receive
    copyHardwareToSoftware2();
  }else if(status == 0x0C){ // 0x0C transmit
    copySoftwareToHardware2();
    if(TFLuna2_OutStatus() == 0){             // software TX FIFO is empty
      UART2->CPU_INT.IMASK &= ~0x0800;    // disable TX FIFO interrupt
    }
  }
}



// Declare state variables for FiFo
//        size, buffer, put and get indexes
int32_t static Tx2PutI; // Index to put new
int32_t static Tx2GetI; // Index of oldest
uint8_t static Tx2Fifo[FIFO2_SIZE];

// *********** Tx2Fifo_Init**********
// Initializes a software Tx2FIFO of a
// fixed size and sets up indexes for
// put and get operations
void Tx2Fifo_Init(void){
  Tx2PutI = Tx2GetI = 0;
}

// *********** Tx2Fifo_Put**********
// Adds an element to the Tx2FIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, TxFIFO is full
uint32_t Tx2Fifo_Put(uint8_t data){
  if(((Tx2PutI+1)&(FIFO2_SIZE-1)) == Tx2GetI){
    return 0;
  }
  Tx2Fifo[Tx2PutI] = data;
  Tx2PutI = (Tx2PutI+1)&(FIFO2_SIZE-1);
  return 1; // success

}

// *********** Tx2Fifo_Get**********
// Gets an element from the Tx2FIFO
// Input: pointer to empty 8-bit variable
// Output: If the Tx2FIFO is empty return 0
//         If the Tx2FIFO has data, remove it, and put in  *datapt, return 1
uint32_t Tx2Fifo_Get(uint8_t *datapt){
  if(Tx2GetI == Tx2PutI){
    return 0;
  }
  *datapt = Tx2Fifo[Tx2GetI];
  Tx2GetI = (Tx2GetI+1)&(FIFO2_SIZE-1);
  return 1; // success
}
//------------TFLuna2_OutStatus------------
// Returns how much data available for reading from Tx2 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t TFLuna2_OutStatus(void){  
 return ((Tx2PutI - Tx2GetI)&(FIFO2_SIZE-1));  
}
int32_t static Rx2PutI; // Index to put new
int32_t static Rx2GetI; // Index of oldest
uint8_t static Rx2Fifo[FIFO2_SIZE];

// *********** Rx2Fifo_Init**********
// Initializes a software RxFIFO of a
// fixed size and sets up indexes for
// put and get operations
void Rx2Fifo_Init(void){
  Rx2PutI = Rx2GetI = 0;
}

// *********** Rx2Fifo_Put**********
// Adds an element to the Rx2FIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, RxFIFO is full
uint32_t Rx2Fifo_Put(uint8_t data){
  if(((Rx2PutI+1)&(FIFO2_SIZE-1)) == Rx2GetI){
    return 0;
  }
  Rx2Fifo[Rx2PutI] = data;
  Rx2PutI = (Rx2PutI+1)&(FIFO2_SIZE-1);
  return 1;
}

// *********** Rx2Fifo_Get**********
// Gets an element from the Rx2FIFO
// Input: pointer to empty 8-bit variable
// Output: If the Rx2FIFO is empty return 0
//         If the Rx2FIFO has data, remove it, and  put in  *datapt, return 1
uint32_t  Rx2Fifo_Get(uint8_t *datapt){
  if(Rx2GetI == Rx2PutI){
    return 0;
  }
  *datapt = Rx2Fifo[Rx2GetI];
  Rx2GetI = (Rx2GetI+1)&(FIFO2_SIZE-1);
  return 1;
}
//------------TFLuna2_InStatus------------
// Returns how much data available for reading from Rx2 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t TFLuna2_InStatus(void){  
 return ((Rx2PutI - Rx2GetI)&(FIFO2_SIZE-1));  
}

