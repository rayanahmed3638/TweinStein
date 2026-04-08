/* LD19.c
 * Jonathan Valvano
 * Date: Nov 5, 2025
   LiDAR Sensor LD19 distance sensor

   The LD19 consists mainly of 
    a laser ranging core, 
    a wireless power transmission unit,  
    a  wireless  communication  unit, 
    an  angle  measuring  unit,  
    a  motor  drive  unit  and  
    a  mechanical housing
  Range 20mm to 12m
  Sampling rate 10 Hz
  Accuracy: 45mm
  Standard deviation 10mm
  Resolution: 15mm
  Angular resolution: 0.8deg (540 measurements per rotation)
 */


#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/LD19.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"

// There are four possible UART3 Rx pins that could be used
// Sensor board uses PB13 (UART3) and PB12=0 (GPIO output)

// LiDAR Sensor LD19 on RTOS sensor board 
// Pin LD19 MSPM0
// 1    Tx   PB13 RxD:  is UART3 Rx (LD19 to MSPM0) baud=230400 bps
// 2    PWM  PB12 GPIO write a 0 to run fastest (MSPM0 to LD19)
// 3    GND  ground
// 4    P5V  5V
// UART3 is shared between LD19 and TFLuna3 (can have either but not both)

uint32_t LostDataLD19;
#define LD19_SIZE 64  // usable size is 63
#define LD19_MESSAGESIZE 47 // LD19 packet is 47 bytes, assuming length remains 12
// prototypes for private functions

void     LD19Fifo_Init(void);
uint32_t LD19Fifo_Put(uint8_t data);
uint32_t LD19Fifo_Get(uint8_t *datapt);
int LD19Index;
uint8_t LD19LastByte;
uint32_t StartAngle,EndAngle,Speed,Distance[12],Intensity[12],Previous;
// 0 for looking for 54 2C
// 2-47 filling the TFLunaDataMessage with 47-byte message
//int LD19Mode; // 0 for FIFO and 1 for 540 array
uint8_t LD19DataMessage[64]; // 47 should be enough
//uint16_t Starts[256],Stops[256],Times[256],II=0;
// number of angles LD19NUM 540
//uint16_t Distances[540];
uint16_t *Distances;
int BadCnt=0; // index error
int BadDistance=0; // too low
uint32_t BadIntensity;
/*
 Scope Measurements Continuous packets
    Baud = 230400 bps
    Packet Length = 47 bytes, 2.034ms long (470bits/230400bps = 2.039ms)
    Packet period = 2.447ms
    Packet frequency = 409 packets/sec
    Packet payload = 12 measurements
    Measurement frequency =12 measurements/2.447ms = 4904measurements/sec
    540 measurements per rotation
    4904 measurements/sec / 540 measurements/rotation = 9.08 rotations/sec
 Overhead:
    Interrupts every three frames, 130.4us (30bits/230400 bps)
    15 interrupts take 3.55us (overhead 2.7%)
    1 interrupt takes 13.4us (overhead 10%) 
    average overhead = (15*2.7%+10%)/16 = 3.2%
 Header：The length is 1 byte, and the value is fixed at 0x54, 
     indicating the beginning of the data packet;
 VerLen：The length is 1 byte, the upper three bits indicate the packet type, 
    which is currently fixed at 1, and the lower five bits indicate the number of
    measurement points in a packet, which is currently fixed at 12, so the byte
    value is fixed at 0x2C
 Speed：The length is 2 bytes (little endian), the unit is degrees per second, indicating the
    speed of the lidar, e.g.,  2152 degrees per second;
 Start angle: The length is 2 bytes (little endian), and the unit is 0.01 degrees, indicating
    the starting angle of the data packet point; e.g., 32427, or 324.27 degrees
 Data：Indicates measurement data, a measurement data length is 3 bytes, 12 measurements
    Distance: 2 bytes (little endian) in mm, e.g., 1234 means 1234mm
    Intensity: 1 byte, the typical value of the signal strength value is around 200
 End angle: The length is 2 bytes (little endian), and the unit is 0.01 degrees, indicating
    the end angle of the data packet point; e.g., 33470, or 334.7 degrees;
 Timestamp：The  length  is  2  bytes,  the  unit  is  milliseconds,  and  the maximum  is  30000. 
    When  it  reaches  30000,  it  will  be  counted  again
 CRC check：The length is 1 byte, obtained from the verification of all the previous  data  except  itself.
*/
// UART3 is in power Domain PD1
// for 32MHz bus clock, UART3 clock is 32MHz
// for 40MHz bus clock, UART3 clock is MCLK 40MHz
// for 80MHz bus clock, UART3 clock is MCLK 80MHz
//------------LD19_Init------------
// Initialize the UART3 for 230400 baud rate (assuming 80 MHz clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: function 0 for debug, 1 for real time
// Output: none
void LD19_Init(uint16_t *d){
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

 // the following code selects PB13 to use UART3
  IOMUX->SECCFG.PINCM[PB13INDEX]  = 0x00040082;
  //bit 18 INENA input enable
  //bit 7  PC connected
  //bits 5-0=2 for UART3_Rx

  // configure PB12 to be GPIO output
  IOMUX->SECCFG.PINCM[PB12INDEX]  = 0x00000081;
  //bit 7  PC connected
  //bits 5-0=1 for GPIO
  GPIOB->DOE31_0 |= (1<<12); // enable output
  GPIOB->DOUTCLR31_0 = (1<<12); // PWM=0, fastest sampling
  
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
     // Baud = 230400
      //    2,500,000/230400 = 10.8506944444
      //   divider = 10+54/64 = 10.84375
    UART3->IBRD = 10;
    UART3->FBRD = 54; // baud =2,500,000/10.84375 = 230,547.55
  }else if (Clock_Freq() == 32000000){
    // 32000000/16 = 2,000,000
     // Baud = 230400
      //    2,000,000/230400 = 8.6805555
      //   divider = 8+43/64 = 8.671875
    UART3->IBRD = 8;
    UART3->FBRD = 43; // 230,630.63
  }else if (Clock_Freq() == 80000000){
     // 80000000/16 = 5,000,000 Hz
     // Baud = 230400
      //    5,000,000/230400 = 21.701388
      //   divider = 21+45/64 = 21.703125
    UART3->IBRD = 21;
    UART3->FBRD = 45; // baud =5,000,000/21.703125 = 230,381.569
  }else return;
  //LD19Mode = mode;
  Distances = d;
  LD19Index = 0; // looking for 0x54 0x2C
  LD19LastByte = 0;
  LD19DataMessage[0] = 0x54;
  LD19DataMessage[1] = 0x2C;
  LD19Fifo_Init();
  LostDataLD19 = 0;
  UART3->LCRH = 0x00000030;
   // bits 5-4 WLEN=11 8 bits
   // bit  3   STP2=0  1 stop
   // bit  2   EPS=0   parity select
   // bit  1   PEN=0   no parity
   // bit  0   BRK=0   no break
  UART3->CPU_INT.IMASK = 0x0401;
  // bit 11 TXINT no
  // bit 10 RXINT yes
  // bit 0  Receive timeout, yes
  UART3->IFLS = 0x0232;
  // bits 11-8 RXTOSEL receiver timeout select 2 (0xF highest)
  // bits 6-4  RXIFLSEL 3 is greater than or equal to 3/4 full (three elements)
  // bits 2-0  TXIFLSEL 2 is less than or equal to half
  NVIC->ICPR[0] = 1<<3; // UART3 is IRQ 3
  NVIC->ISER[0] = 1<<3;
  NVIC->IP[0] = (NVIC->IP[0]&(~0xFF000000))|(1<<30);    // set priority (bits 31-30) IRQ 3
  UART3->CTL0 |= 0x01; // enable UART3
}
// pointer to most recent array of 540 measurements
//uint16_t *LD19_GetDistances(void){
//  return Distances;
//}
// copy from hardware RX FIFO to software LD19Fifo
// stop when hardware RX FIFO is empty 
// data are lost if software LD19Fifo is full
void static copyHardwareToSoftwareLD19(void){
  uint8_t letter;
  if(Distances==0){ // raw data mode
    while((UART3->STAT&0x04) == 0){// empty RX hardware FIFO
      letter = UART3->RXDATA;
      if((LD19Fifo_Put(letter))==0){
	      LostDataLD19++;
      }
    }
  }else{
    while((UART3->STAT&0x04)==0){ // empty RX hardware FIFO
      letter = UART3->RXDATA;
      if(LD19Index == 0){ // looking for sequence 0x54,0x2C
        if((letter == 0x2C)&&(LD19LastByte == 0x54)){
          LD19Index = 2; // looking for message
        }
        LD19LastByte = letter;
      }else{
        LD19DataMessage[LD19Index] = letter;
        LD19Index++;
        if(LD19Index == LD19_MESSAGESIZE){
    //      GPIOB->DOUTTGL31_0 = BLUE; // PB22

        //  Speed = LD19DataMessage[2]+(LD19DataMessage[3]<<8);
          StartAngle = LD19DataMessage[4]+(LD19DataMessage[5]<<8);
          //EndAngle = LD19DataMessage[42]+(LD19DataMessage[43]<<8);
          // StartAngle goes from 0 to 35999
          // index goes      from 0 to 540-1 (539()
          //uint32_t index = (540*StartAngle)/36000; // 0 to 539
          uint32_t findex = (983*StartAngle)>>16;
          if(findex>=540) {
            findex = 540-1;
            BadCnt++; // never happens
          }
          for(int i=0;i<12;i++){
            uint16_t d = LD19DataMessage[6+3*i]+(LD19DataMessage[7+3*i]<<8);
            if(d < 10){
              BadDistance++; // happens a lot, might be low intensity
              BadIntensity = LD19DataMessage[8+3*i];
              if(findex==0){
                d = Distances[539];
                if(d > 10){
                  Distances[findex+i] = d;
                }
              }else{
                d = Distances[findex+i-1];
                if(d > 10){
                  Distances[findex+i] = Distances[findex+i-1];
                }

              }
            }else{
              Distances[findex+i] = d;
            }
            
         //   if(findex > 270){
  //            GPIOA->DOUTSET31_0 = 1;
         // GPIOB->DOUTSET31_0 = BLUE; // PB22
         //   }
          //  if(findex < 200){
//              GPIOA->DOUTCLR31_0 = 1;
         // GPIOB->DOUTCLR31_0 = BLUE; // PB22

         //   }
            
            //            Distance[i] = d;
         //   Intensity[i] = LD19DataMessage[8+3*i];
          }

          /*
          Starts[II] = StartAngle;
          Stops[II] = EndAngle;
          Times[II] =  LD19DataMessage[44]+(LD19DataMessage[45]<<8);
          II = (II+1)&0xFF;
          */
          LD19LastByte = 0; // get ready for next message
          LD19Index = 0;
        }
      }        
    }
  }
}

//------------LD19_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
uint8_t LD19_InChar(void){uint32_t status;
  uint8_t letter;
  do{
    status = LD19Fifo_Get(&letter);
  }while(status==0);
  return(letter);
}



void UART3_IRQHandler(void){ uint32_t status;
  status = UART3->CPU_INT.IIDX; // reading clears bit in RIS
 // GPIOA->DOUTTGL31_0 = 1;

     GPIOB->DOUTSET31_0 = BLUE; // PB22
  if(status == 0x01){   // 0x01 receive timeout
    copyHardwareToSoftwareLD19();
  }else if(status == 0x0B){ // 0x0B receive
    copyHardwareToSoftwareLD19();
  }
     GPIOB->DOUTCLR31_0 = BLUE; // PB22
}



// Declare state variables for FiFo
//        size, buffer, put and get indexes

int32_t static LD19PutI; // Index to put new
int32_t static LD19GetI; // Index of oldest
uint8_t static LD19Fifo[LD19_SIZE];

// *********** LD19Fifo_Init**********
// Initializes a software LD19Fifo of a
// fixed size and sets up indexes for
// put and get operations
void LD19Fifo_Init(void){
  LD19PutI = LD19GetI = 0;
}

// *********** LD19Fifo_Put**********
// Adds an element to the LD19FIFO
// Input: data is character to be inserted
// Output: 1 for success, data properly saved
//         0 for failure, LD19Fifo is full
uint32_t LD19Fifo_Put(uint8_t data){
  if(((LD19PutI+1)&(LD19_SIZE-1)) == LD19GetI){
    return 0;
  }
  LD19Fifo[LD19PutI] = data;
  LD19PutI = (LD19PutI+1)&(LD19_SIZE-1);
  return 1;
}

// *********** LD19Fifo_Get**********
// Gets an element from the LD19FIFO
// Input: pointer to empty 8-bit variable
// Output: If the LD19FIFO is empty return 0
//         If the LD19FIFO has data, remove it, and  put in  *datapt, return 1
uint32_t LD19Fifo_Get(uint8_t *datapt){
  if(LD19GetI == LD19PutI){
    return 0;
  }
  *datapt = LD19Fifo[LD19GetI];
  LD19GetI = (LD19GetI+1)&(LD19_SIZE-1);
  return 1;
}

//------------LD19_InStatus------------
// Returns how much data available for reading from LD19 FIFO
// Input: none
// Output: number of elements in receive FIFO
uint32_t LD19_InStatus(void){  
 return ((LD19PutI - LD19GetI)&(LD19_SIZE-1));  
}

