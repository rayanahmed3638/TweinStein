/* TFLuna3.c
 * Jonathan Valvano
 * Date: 10/23/2025
   TF Luna TOF distance sensor
 */


#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/TFLuna3.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"

// There are four possible sets of UART3 pins that can be used
// Select the TFLuna3_RX/TFLuna3_TX for the UART3 pins to use
// overhead of TFLuna will be 17us/measurement at 100 measurements/sec
// UART3 is shared between LD19 and TFLuna3 (can have either but not both)


// SJ-PM-TF-Luna+A01 on RTOS sensor board 
// Pin
// 1    Red  5V
// 2    Serial TxD: PB12 is UART3 Tx (MSPM0 to TFLuna3)
// 3    Serial RxD: PB13 is UART3 Rx (TFLuna3 to MSPM0)
// 4    black ground
#define TFLUNA3_RX PB13INDEX
#define TFLUNA3_TX PB12INDEX

// Can use PA13 and PA14, but different PINCH values

// SJ-PM-TF-Luna+A01 
// Pin
// 1    Red  5V
// 2    Serial TxD: PA26 is UART3 Tx (MSPM0 to TFLuna3)
// 3    Serial RxD: PA25 is UART3 Rx (TFLuna3 to MSPM0)
// 4    black ground
//#define TFLUNA3_RX PA25INDEX
//#define TFLUNA3_TX PA26INDEX

// SJ-PM-TF-Luna+A01 
// Pin
// 1    Red  5V
// 2    Serial TxD: PB2 is UART3 Tx (MSPM0 to TFLuna3)
// 3    Serial RxD: PB3 is UART3 Rx (TFLuna3 to MSPM0)
// 4    black ground
//#define TFLUNA3_RX PB3INDEX
//#define TFLUNA3_TX PB2INDEX

uint32_t LostData3;
#define FIFO3_SIZE 16  // usable size is 15
// prototypes for private functions
void     Tx3Fifo_Init(void);
uint32_t Tx3Fifo_Put(uint8_t data);
uint32_t Tx3Fifo_Get(uint8_t *datapt);
void     Rx3Fifo_Init(void);
uint32_t Rx3Fifo_Put(uint8_t data);
uint32_t Rx3Fifo_Get(uint8_t *datapt);
int TFLuna3Index;
// 0 for looking for two 59s
// 2-8 filling the TFLunaDataMessage with 9-byte message
void (*TFLuna3Function)(uint32_t); // data in mm
uint8_t TFLuna3LastByte;
uint32_t TFLuna3Distance; // in mm
uint8_t TFLuna3DataMessage[12]; // 9 byte fixed size message
int BadCheckSum3; // errors


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

// power Domain PD1
// for 32MHz bus clock, UART clock is 32MHz
// for 40MHz bus clock, UART clock is MCLK 40MHz
// for 80MHz bus clock, UART clock is MCLK 80MHz
//------------TFLuna_Init------------
// Initialize the UART3 for 115,200 baud rate (assuming 80 MHz clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: function 0 for debug, user function for real time
// Output: none
void TFLuna3_Init(void (*function)(uint32_t)){
    // do not reset or activate PortA, already done in LaunchPad_Init
    // RSTCLR to GPIOA and UART3 peripherals
    //   bits 31-24 unlock key 0xB1
    //   bit 1 is Clear reset sticky bit
    //   bit 0 is reset gpio port
 // GPIOA->GPRCM.RSTCTL = (uint32_t)0xB1000003; // called previously
  UART3->GPRCM.RSTCTL = 0xB1000003;
    // Enable power to GPIOA and UART3 peripherals
    // PWREN
    //   bits 31-24 unlock key 0x26
    //   bit 0 is Enable Power
 // GPIOA->GPRCM.PWREN = (uint32_t)0x26000001; // called previously
  UART3->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for uart to power up

 // the following code selects which pins to use
  IOMUX->SECCFG.PINCM[TFLUNA3_RX]  = 0x00040082;
  //bit 18 INENA input enable
  //bit 7  PC connected
  //bits 5-0=2 for UART3_Rx

  // configure  alternate UART3 transmit function
  IOMUX->SECCFG.PINCM[TFLUNA3_TX]  = 0x00000082;
  //bit 7  PC connected
  //bits 5-0=2 for UART3_Tx
  
  UART3->CLKSEL = 0x08; // bus clock
  UART3->CLKDIV = 0x00; // no divide
  UART3->CTL0 &= ~0x01; // disable UART3
  UART3->CTL0 = 0x00020018;
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
      // 40000000/16 = 2,500,000 Hz
     // Baud = 115200
      //    2,500,000/115200 = 21.701388888
      //   divider = 21+45/64 = 21.703125
    UART3->IBRD = 21;
    UART3->FBRD = 45; // baud =2,500,000/10.84375 = 115,273.77
  }else if (Clock_Freq() == 32000000){
    // 32000000/16 = 2,000,000
     // Baud = 115200
      //    2,000,000/115200 = 17.3611111
      //   divider = 21+23/64 = 17.359375
    UART3->IBRD = 17;
    UART3->FBRD = 23; // 115,211.52
  }else if (Clock_Freq() == 80000000){
     // 80000000/16 = 5,000,000 Hz
     // Baud = 115200
      //    5,000,000/115200 = 43.4027777
      //   divider = 43+26/64 = 43.40625
    UART3->IBRD = 43;
    UART3->FBRD = 26; // baud =5,000,000/43.40625 = 115,190.78
  }else return;
  TFLuna3Function = function;
  TFLuna3Index = 0; // looking for two 59s
  TFLuna3LastByte = 0;
  TFLuna3DataMessage[0] = 0x59;
  TFLuna3DataMessage[1] = 0x59;
  BadCheckSum3 = 0;
  Tx3Fifo_Init();
  Rx3Fifo_Init();
  LostData3 = 0;
  UART3->LCRH = 0x00000030;
   // bits 5-4 WLEN=11 8 bits
   // bit  3   STP2=0  1 stop
   // bit  2   EPS=0   parity select
   // bit  1   PEN=0   no parity
   // bit  0   BRK=0   no break
  UART3->CPU_INT.IMASK = 0x0C01;
  // bit 11 TXINT
  // bit 10 RXINT yes
  // bit 0  Receive timeout, yes
  UART3->IFLS = 0x0422;
  // bits 11-8 RXTOSEL receiver timeout select 4 (0xF highest)
  // bits 6-4  RXIFLSEL 2 is greater than or equal to half
  // bits 2-0  TXIFLSEL 2 is less than or equal to half
  NVIC->ICPR[0] = 1<<3; // UART3 is IRQ 3
  NVIC->ISER[0] = 1<<3;
  NVIC->IP[0] = (NVIC->IP[0]&(~0xFF000000))|(1<<30);    // set priority (bits 31-30) IRQ 3
  UART3->CTL0 |= 0x01; // enable UART3
}


// copy from hardware RX FIFO to software RX FIFO
// stop when hardware RX FIFO is empty or software RX FIFO is full
void static copyHardwareToSoftware3(void){
  uint8_t letter;
  if(TFLuna3Function==0){ // raw data mode
    while(((UART3->STAT&0x04) == 0)&&(TFLuna3_InStatus() < (FIFO3_SIZE - 1))){
    letter = UART3->RXDATA;
    Rx3Fifo_Put(letter);
    }
  }else{
    while((UART3->STAT&0x04)==0){
      letter = UART3->RXDATA;
      if(TFLuna3Index == 0){
        if((letter == 0x59)&&(TFLuna3LastByte == 0x59)){
          TFLuna3Index = 2; // looking for message
        }
        TFLuna3LastByte = letter;
      }else{
        TFLuna3DataMessage[TFLuna3Index] = letter;
        TFLuna3Index++;
        if(TFLuna3Index == 9){
          TFLuna3LastByte = 0; // get ready for next message
          TFLuna3Index = 0;
          uint8_t check=0;
          for(int i=0;i<8;i++){
            check += TFLuna3DataMessage[i];
          }
          if(check == TFLuna3DataMessage[8]){
            TFLuna3Distance = TFLuna3DataMessage[3]*256+TFLuna3DataMessage[2];
            // call back
            (*TFLuna3Function)(TFLuna3Distance);     // pass distance back to higher level
          }else{
            BadCheckSum3++; // error
          }
        }
      }        
    }
  }
}

//------------TFLuna3_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
uint8_t TFLuna3_InChar(void){uint32_t status;
  uint8_t letter;
  do{
    status = Rx3Fifo_Get(&letter);
  }while(status==0);
  return(letter);
}
// copy from software TX FIFO to hardware TX FIFO
// stop when software TX FIFO is empty or hardware TX FIFO is full
void static copySoftwareToHardware3(void){
  uint8_t letter;
  while(((UART3->STAT&0x80) == 0) && (TFLuna3_OutStatus() > 0)){
    Tx3Fifo_Get(&letter);
    UART3->TXDATA = letter;
  }
}
//------------UART3_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void TFLuna3_OutChar(uint8_t data){
    while(Tx3Fifo_Put(data) == 0){};
    UART3->CPU_INT.IMASK &= ~0x0800;   // disarm TX FIFO interrupt
    copySoftwareToHardware3();
    UART3->CPU_INT.IMASK |= 0x0800;    // rearm TX FIFO interrupt
  }
//------------TFLuna_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void TFLuna3_OutString(uint8_t *pt){
  while(*pt){
    TFLuna3_OutChar(*pt);
    pt++;
  }
}


//------------TFLuna3_SendMessage------------
// Output message, msg[1] is length
// Input: pointer to message to be transferred
// Output: none
void TFLuna3_SendMessage(const uint8_t msg[]){
  for(int i=0; i<msg[1]; i++){
    TFLuna3_OutChar(msg[i]);
  }
}
void TFLuna3_Format_Standard_mm(void){
  TFLuna3_SendMessage(Format_Standard_mm);
}
void TFLuna3_Format_Standard_cm(void){
  TFLuna3_SendMessage(Format_Standard_cm);
}
void TFLuna3_Format_Pixhawk(void){
  TFLuna3_SendMessage(Format_Pixhawk);
}
void TFLuna3_Output_Enable(void){
  TFLuna3_SendMessage(Output_Enable);
}
void TFLuna3_Output_Disable(void){
  TFLuna3_SendMessage(Output_Disable);
}
void TFLuna3_Frame_Rate(void){
  TFLuna3_SendMessage(Frame_Rate);
}
void TFLuna3_SaveSettings(void){
  TFLuna3_SendMessage(SaveSettings);
}
void TFLuna3_System_Reset(void){
  TFLuna3_SendMessage(System_Reset);
}

void UART3_IRQHandler(void){ uint32_t status;
  status = UART3->CPU_INT.IIDX; // reading clears bit in RIS
  if(status == 0x01){   // 0x01 receive timeout
    copyHardwareToSoftware3();
  }else if(status == 0x0B){ // 0x0B receive
    copyHardwareToSoftware3();
  }else if(status == 0x0C){ // 0x0C transmit
    copySoftwareToHardware3();
    if(TFLuna3_OutStatus() == 0){             // software TX FIFO is empty
      UART3->CPU_INT.IMASK &= ~0x0800;    // disable TX FIFO interrupt
    }
  }
}



// Declare state variables for FiFo
//        size, buffer, put and get indexes
int32_t static Tx3PutI; // Index to put new
int32_t static Tx3GetI; // Index of oldest
uint8_t static Tx3Fifo[FIFO3_SIZE];

// *********** Tx3Fifo_Init**********
// Initializes a software Tx3FIFO of a
// fixed size and sets up indexes for
// put and get operations
void Tx3Fifo_Init(void){
  Tx3PutI = Tx3GetI = 0;
}

// *********** Tx3Fifo_Put**********
// Adds an element to the Tx3FIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, TxFIFO is full
uint32_t Tx3Fifo_Put(uint8_t data){
  if(((Tx3PutI+1)&(FIFO3_SIZE-1)) == Tx3GetI){
    return 0;
  }
  Tx3Fifo[Tx3PutI] = data;
  Tx3PutI = (Tx3PutI+1)&(FIFO3_SIZE-1);
  return 1; // success

}

// *********** Tx3Fifo_Get**********
// Gets an element from the Tx3FIFO
// Input: pointer to empty 8-bit variable
// Output: If the Tx3FIFO is empty return 0
//         If the Tx3FIFO has data, remove it, and put in  *datapt, return 1
uint32_t Tx3Fifo_Get(uint8_t *datapt){
  if(Tx3GetI == Tx3PutI){
    return 0;
  }
  *datapt = Tx3Fifo[Tx3GetI];
  Tx3GetI = (Tx3GetI+1)&(FIFO3_SIZE-1);
  return 1; // success
}
//------------TFLuna3_OutStatus------------
// Returns how much data available for reading from Tx3 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t TFLuna3_OutStatus(void){  
 return ((Tx3PutI - Tx3GetI)&(FIFO3_SIZE-1));  
}
int32_t static Rx3PutI; // Index to put new
int32_t static Rx3GetI; // Index of oldest
uint8_t static Rx3Fifo[FIFO3_SIZE];

// *********** Rx3Fifo_Init**********
// Initializes a software RxFIFO of a
// fixed size and sets up indexes for
// put and get operations
void Rx3Fifo_Init(void){
  Rx3PutI = Rx3GetI = 0;
}

// *********** Rx3Fifo_Put**********
// Adds an element to the Rx3FIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, RxFIFO is full
uint32_t Rx3Fifo_Put(uint8_t data){
  if(((Rx3PutI+1)&(FIFO3_SIZE-1)) == Rx3GetI){
    return 0;
  }
  Rx3Fifo[Rx3PutI] = data;
  Rx3PutI = (Rx3PutI+1)&(FIFO3_SIZE-1);
  return 1;
}

// *********** Rx3Fifo_Get**********
// Gets an element from the Rx3FIFO
// Input: pointer to empty 8-bit variable
// Output: If the Rx3FIFO is empty return 0
//         If the Rx3FIFO has data, remove it, and  put in  *datapt, return 1
uint32_t  Rx3Fifo_Get(uint8_t *datapt){
  if(Rx3GetI == Rx3PutI){
    return 0;
  }
  *datapt = Rx3Fifo[Rx3GetI];
  Rx3GetI = (Rx3GetI+1)&(FIFO3_SIZE-1);
  return 1;
}
//------------TFLuna3_InStatus------------
// Returns how much data available for reading from Rx3 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t TFLuna3_InStatus(void){  
 return ((Rx3PutI - Rx3GetI)&(FIFO3_SIZE-1));  
}

