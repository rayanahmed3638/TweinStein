// CAN.c
// Runs on MSPM0G3507
// Use FDCAN0 to communicate on CAN bus PA12 and PA13

// Jonathan Valvano
// December 11, 2025
// derived from can_to_uart_bridge_LP_MSPM0G3507_nortos_ticlang
/* 
 Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// Use TCAN1057AVDRQ1 (not TCAN1057A-Q1, the version with pin 5 nc)
//  Pin1 TXD  ---- CAN_Tx PA12 FD-CAN module 0 transmit
//  Pin2 Vss  ---- ground
//  Pin3 VCC  ---- +5V with 0.1uF cap to ground
//  Pin4 RXD  ---- CAN_Rx PA13 FD-CAN module 0 receive (0 to 3.3V)
//  Pin5 VIO  ---- +3.3V (digital interface supply)
//  Pin6 CANL ---- to other CANL on network 
//  Pin7 CANH ---- to other CANH on network 
//  Pin8 RS   ---- ground, Slope-Control Input (maximum slew rate)
// 120 ohm across CANH, CANL on both ends of network
#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "../inc/Clock.h"
#include "../RTOS_Labs_common/CAN.h"
#include "../RTOS_Labs_common/LaunchPad.h"
#include "../RTOS_Labs_common/OS.h"
uint32_t CAN_PID,CAN_CREL; // version

Sema4_t CanDataAvailable;

__STATIC_INLINE uint32_t READ_REG32_RAW(uint32_t addr){
  uint32_t regVal = *(volatile uint32_t *) addr;
  return (regVal);
}
#define CAN_DEBUG 0
uint32_t objSize4[8] = {4, 5, 6, 7, 8, 10, 14, 18};

typedef struct {
    /*! Rx FIFO number
     *   One of @ref DL_MCAN_RX_FIFO_NUM
     */
    uint32_t num;
    /*! Rx FIFO Fill Level */
    uint32_t fillLvl;
    /*! Rx FIFO Get Index */
    uint32_t getIdx;
    /*! Rx FIFO Put Index */
    uint32_t putIdx;
    /*! Rx FIFO Full
     *   0 = Rx FIFO not full
     *   1 = Rx FIFO full
     */
    uint32_t fifoFull;
    /*! Rx FIFO Message Lost */
    uint32_t msgLost;
} MCAN_RxFIFOStatus_t;

/**
 *  @brief  Structure for MCAN Rx Buffer element.
*/
#if CAN_DEBUG
typedef struct {
// Identifier 
    uint32_t id;
// Remote Transmission Request
// 0 = Received frame is a data frame
// 1 = Received frame is a remote frame
    uint32_t rtr;
// Extended Identifier
// 0 = 11-bit standard identifier
// 1 = 29-bit extended identifier
    uint32_t xtd;
// Error State Indicator
//   0 = Transmitting node is error active
//   1 = Transmitting node is error passive
    uint32_t esi;
// Rx Timestamp 
    uint32_t rxts;
//Data Length Code
//   0-8  = CAN + CAN FD: received frame has 0-8 data bytes
    uint32_t dlc;
// Bit Rat Switching
//  0 = Frame received without bit rate switching
//  1 = Frame received with bit rate switching
    uint32_t brs;
// FD Format
//   0 = Standard frame format
//   1 = CAN FD frame format (new DLC-coding and CRC)
    uint32_t fdf;
// Filter Index 
    uint32_t fidx;
// Accepted Non-matching Frame
//  0 = Received frame matching filter index FIDX
//   1 = Received frame did not match any Rx filter element
    uint32_t anmf;
// Data bytes.
//   Only first dlc number of bytes are valid.
    uint8_t data[8];
} MCAN_RxBufElement_t;
MCAN_RxBufElement_t rxMsg4;
#endif

// CAN message 
typedef struct {
    uint32_t id; // identification of the message
    uint8_t dlc; // Data Length Code
    uint8_t data[8]; // Data bytes limited to 8 bytes
} CANmsg_t;
#define CANFIFOSIZE 8		  
CANmsg_t CANFIFO[CANFIFOSIZE]; // must be a power of two
uint32_t CANGetI = 0;
uint32_t CANPutI = 0;
volatile uint32_t CanInterruptLine1Status=0;


// SYSPLLCLK1 80MHz
// bit rate 1Mbps
void CAN_Init(void){uint32_t regVal;
// assumes Port A has been reset and enabled previously by LaunchPad_Init
  CANFD0->MCANSS.RSTCTL = 0xB1000003; // reset CAN
  CANFD0->MCANSS.PWREN = 0x26000001;  // enable power
  Clock_Delay(200); // time for CAN to power up  
	IOMUX->SECCFG.PINCM[PA12INDEX] = (uint32_t) 0x00000085; // PA12 CAN Tx
  IOMUX->SECCFG.PINCM[PA13INDEX] = (uint32_t) 0x00040086; // PA13 CAN Rx
  Clock_Delay(1000); // time for CAN to power up  
// Enables the MCAN functional module clock
  CANFD0->MCANSS.TI_WRAPPER.MSP.MCANSS_CLKEN = 1; 
 // Clock_Delay(24); // time for CAN to power up
	 
// Clock CAN from HFCLK
 // SYSCTL->SOCLOCK.GENCLKCFG = (SYSCTL->SOCLOCK.GENCLKCFG&~0x00000100);
  SYSCTL->SOCLOCK.GENCLKCFG |= 0x00000100; // SYSPLLCLK1
//   bit 8 0 CANCLK is HFCLK, 1 SYSPLLCLK1
    
//  Clock divide ratio specification. Enables configuring clock divide
//  settings for the MCAN functional clock input to the MCAN-SS.
//  0h (R/W) = Divides input clock by 1
//  1h (R/W) = Divides input clock by 2
//  2h (R/W) = Divides input clock by 4
//  3h (R/W) = Divides input clock by 1
  CANFD0->MCANSS.TI_WRAPPER.MSP.MCANSS_CLKDIV = 1; // divide by 2
//  Clock_Delay(100); // time for CAN to power up
// Get MCANSS Revision ID. (optional FYI only) 
  CAN_PID = CANFD0->MCANSS.TI_WRAPPER.PROCESSORS.MCANSS_REGS.MCANSS_PID;
//  31-30 SCHEME R 1h PID Register Scheme
//  27-16 MODULE_ID R 8E0h Module Identification Number
//  10-8 MAJOR R 1h Major Revision of the MCAN Subsystem
//  5-0 MINOR R 1h Minor Revision of the MCAN Subsystem

  CAN_CREL = CANFD0->MCANSS.MCAN.MCAN_CREL; // core release (optional FYI only)
//  31-28 REL R 3h Core Release. One digit, BCD-coded.
//  27-24 STEP R 2h Step of Core Release. One digit, BCD-coded.
//  23-20 SUBSTEP R 3h Sub-Step of Core Release. One digit, BCD-coded.
//  19-16 YEAR R 8h Time Stamp Year. One digit, BCD-coded.
//  15-8  MON R 6h Time Stamp Month. Two digits, BCD-coded.
//   7-0  DAY R 8h Time Stamp Day. Two digits, BCD-coded.

// Wait for Memory initialization to be completed. 
//  bit 1 MEM_INIT_DONE R 0h Memory Initialization Done.
//     0 Message RAM initialization is in progress
//     1 Message RAM is initialized for use
  while((CANFD0->MCANSS.TI_WRAPPER.PROCESSORS.MCANSS_REGS.MCANSS_STAT&0x02)==0){};
   
// Put MCAN in SW initialization mode. 
// MCAN_CCCR INIT R/W 1h Initialization
// 0 Normal Operation
// 1 Initialization is started
//  Note: Due to the synchronization mechanism between the two clock
//  domains, there may be a delay until the value written to INIT can
//  be read back. Therefore the programmer has to assure that the
//  previous value written to INIT has been accepted by reading INIT
//  before setting INIT to a new value
  CANFD0->MCANSS.MCAN.MCAN_CCCR |= 0x01; // SW initialization mode

  while((CANFD0->MCANSS.MCAN.MCAN_CCCR&0x01)==0){};
// Wait while MCAN is not in SW initialization mode 
 
// Initialize MCAN module

// Configure MCAN wakeup and clock stop controls 
  regVal = CANFD0->MCANSS.TI_WRAPPER.PROCESSORS.MCANSS_REGS.MCANSS_CTRL;
  regVal |= (1<<4); // wkupReqEnable
  regVal |= (1<<5); // autoWkupEnable
  regVal |= (1<<3); // DBGSUSP_FREE emulationEnable
  CANFD0->MCANSS.TI_WRAPPER.PROCESSORS.MCANSS_REGS.MCANSS_CTRL = regVal;
 
  CANFD0->MCANSS.MCAN.MCAN_CCCR |= 0x02; //ProtectedRegAccessUnlock
  
// Configure MCAN mode(FD vs Classic CAN operation) and controls 
  regVal = CANFD0->MCANSS.MCAN.MCAN_CCCR;
  regVal |= (1<<8);   // fdMode, Flexible Datarate Operation Enable
  regVal |= (1<<9);   // brsEnable, Bit Rate Switch Enable
  regVal &= ~(1<<14); // txpEnable=false, Transmit Pause
  regVal &= ~(1<<13); // efbi=false, Edge Filtering during Bus Integration
  regVal &= ~(1<<12); // pxhddisable=0, Protocol Exception Handling Disable
  regVal &= ~(1<<6);  // darEnable=0, Disable Automatic Retransmission
  CANFD0->MCANSS.MCAN.MCAN_CCCR = regVal;

// Configure Transceiver Delay Compensation 
  CANFD0->MCANSS.MCAN.MCAN_TDCR = (6<<8)+ 10; //tdco=6,tdcf=10
  
// Configure MSG RAM watchdog counter preload value *
  CANFD0->MCANSS.MCAN.MCAN_RWD = 255; // WDC=wdcPreload=255
   
// Enable/Disable Transceiver Delay Compensation 
  CANFD0->MCANSS.MCAN.MCAN_DBTP |= (1<<23); // Transmitter Delay Compensation
  
// Configure Bit timings
  CANFD0->MCANSS.MCAN.MCAN_NBTP = // MCAN Nominal Bit Timing and  Prescaler Register
     (31<<25)  // nomSynchJumpWidth Nominal (Re)Synchronization Jump Width
    |(0<<16)   // nomRatePrescalar Nominal Bit Rate Prescaler
    |(126<<8)  // nomTimeSeg1Nominal Time Segment Before Sample Point
    |(31<<0);  // nomTimeSeg2 Nominal Time Segment After Sample Point.
 
  CANFD0->MCANSS.MCAN.MCAN_DBTP = // MCAN Data Bit Timing and Prescaler  Register
     (1<<23)  // TDC =1 delay compensation   
	  |(14<<8)  // dataTimeSeg1
	  |(3<<4)   // dataTimeSeg2
	  |(3<<0);  // dataSynchJumpWidth

// Configure Message RAM Sections 
// start end  num  size 
//   0    0    0    0      Standard ID Filter List (not used)
//   0    0    0    0      Extended ID Filter List (not used)
// 448  ???    2    7(64)  Tx Buffers (only uses the first buffer)
// 164  172   10    2      Tx EventFIFOs (not used)
//   8  368    5    7(64)  RxFifo0 (used)
// 368  448    5    0(8)   RxFifo1 (not used)
// 600  672    1    7(64)  Rx Buffer (not used) 

// Configure Message Filters 
  CANFD0->MCANSS.MCAN.MCAN_SIDFC = 0; // MCAN Standard ID Filter Configuration
   // No standard Message ID filter
   // No Filter List Standard Start Address
   
  CANFD0->MCANSS.MCAN.MCAN_XIDFC = 0;
  // no Filter List Extended Start Address
  // No No extended Message ID filter

// Configure Rx FIFO 0 section 
  //  if (0U != msgRAMConfigParams->rxFIFO0size) {
  CANFD0->MCANSS.MCAN.MCAN_RXF0C = 
    (0<<31) // F0OM 0 FIFO 0 blocking mode
   |(0<<24) // F0WM 0 Watermark interrupt disabled
   |(5<<16) // F0S  Rx FIFO 0 number of elements (means you can queue 5 received messages)
   |(2<<2); // F0SA Rx FIFO 0 Start Address
			
// Configure Rx FIFO0 elements size */
  CANFD0->MCANSS.MCAN.MCAN_RXESC = (CANFD0->MCANSS.MCAN.MCAN_RXESC&(~0x07))|7;
//  F0DS Rx FIFO0 Data Field Size, 111 64 byte data field

// Configure Rx FIFO 1 section (not used)
  CANFD0->MCANSS.MCAN.MCAN_RXF1C = 
    (0<<31)   // F1OM 0 FIFO 0 blocking mode
   |(3<<24)   // F1WM 3 Level for Rx FIFO 1 watermark interrupt
   |(5<<16)   // F1S  Rx FIFO 1 Size
   |(368<<2); // F1SA Rx FIFO 1 Start Address
			
// Configure Rx FIFO1 elements size (not used)
  CANFD0->MCANSS.MCAN.MCAN_RXESC = (CANFD0->MCANSS.MCAN.MCAN_RXESC&(~0x70))|0;
//  F1DS Rx FIFO 1 Data Field Size 000 8-byte data field
  
// Configure Rx Buffer Start Address (not used)
  CANFD0->MCANSS.MCAN.MCAN_RXBC = (150<<2); // Rx Buffer Start Address

// Configure Rx Buffer elements size (not used)
  CANFD0->MCANSS.MCAN.MCAN_RXESC = (CANFD0->MCANSS.MCAN.MCAN_RXESC&(~0x700))|0x700;
//  RBDS Rx Buffer Data Field Size 111 64 byte data field
		
// Configure Tx Event FIFO section 
  CANFD0->MCANSS.MCAN.MCAN_TXEFC =
    (0<<24)   // EFWM Event FIFO Watermark 0 Watermark interrupt disabled
   |(2<<16)   // EFS Event FIFO Size  Number of Tx Event FIFO elements
   |(41<<2);  // EFSA Event FIFO Start Address.
	
// TX buffer configuration (uses first buffer at address 448)
  CANFD0->MCANSS.MCAN.MCAN_TXBC =
    112<<2     // TBSA txStartAddr
    | (2<<16)  // TDTB txBufNum, 2 buffers (uses first one)
    | (10<<24) // TFQS txFIFOSize, 10 fifos
    | (0<<30); // TFQM, 0 Tx FIFO operation

  CANFD0->MCANSS.MCAN.MCAN_TXESC = 7; // TBDS, 64 byte data field  txBufElemSize

// Set Extended ID Mask (does not use extended ID)
   CANFD0->MCANSS.MCAN.MCAN_XIDAM = 0x1FFFFFFF;

// set access lock
  CANFD0->MCANSS.MCAN.MCAN_CCCR &= ~0x02; // Access lock   
// Put MCAN into normal mode mode, stop SW initialization mode
  CANFD0->MCANSS.MCAN.MCAN_CCCR &= ~0x01; 

  while((CANFD0->MCANSS.MCAN.MCAN_CCCR&0x01)==1){}; // wait for normal mode

  // Initialize semaphore for read
  OS_InitSemaphore(&CanDataAvailable, 0);
}

void CAN_EnableInterrupts(uint32_t priority){
// Enable MCAN module Interrupts 
  CANFD0->MCANSS.MCAN.MCAN_IE |= 0x3FEF1201;	
  // MCAN_IE Field Descriptions
  // 29 ARAE  1 Access to Reserved Address Enable
  // 28 PEDE  1 Protocol Error in Data Phase Enable
  // 27 PEAE  1 Protocol Error in Arbitration Phase Enable
  // 26 WDIE  1 Watchdog Interrupt Enable
  // 25 BOE   1 Bus_Off Status Enable
  // 24 EWE   1 Warning Status Enable
  // 23 EPE   1 Error Passive Enable
  // 22 ELOE  1 Error Logging Overflow Enable
  // 21 BEUE  1 Bit Error Uncorrected Enable
  // 20 BECE  0 Bit Error Corrected Enable
  // 19 DRXE  1 Message Stored to Dedicated Rx Buffer Enable
  // 18 TOOE  1 Timeout Occurred Enable
  // 17 MRAFE 1 Message RAM Access Failure Enable
  // 16 TSWE  1 Timestamp Wraparound Enable
  // 15 TEFLE 0 Tx Event FIFO Element Lost Enable
  // 14 TEFFE 0 Tx Event FIFO Full Enable
  // 13 TEFWE 0 Tx Event FIFO Watermark Reached Enable
  // 12 TEFNE 1 Tx Event FIFO New Entry Enable
  // 11 TFEE  0 Tx FIFO Empty Enable
  // 10 TCFE  0 Transmission Cancellation Finished Enable
  //  9 TCE   1 Transmission Completed Enable
  //  8 HPME  0 High Priority Message Enable
  //  7 RF1LE 0 Rx FIFO 1 Message Lost Enable
  //  6 RF1FE 0 Rx FIFO 1 Full Enable
  //  5 RF1WE 0 Rx FIFO 1 Watermark Reached Enable 
  //  4 RF1NE 0 Rx FIFO 1 New Message Enable
  //  3 RF0LE 0 Rx FIFO 0 Message Lost Enable
  //  2 RF0FE 0 Rx FIFO 0 Full Enable
  //  1 RF0WE 0 Rx FIFO 0 Watermark Reached Enable
  //  0 RF0NE 1 Rx FIFO 0 New Message Enable  (only one used)

  CANFD0->MCANSS.MCAN.MCAN_ILS |= 0x3FEFFFFF;  // select all Line 1
  CANFD0->MCANSS.MCAN.MCAN_ILE = 0x02; // line 1 enable

// Enable MSPM0 MCAN interrupt 
  CANFD0->MCANSS.TI_WRAPPER.MSP.CPU_INT.ICLR = 0x02;   // line 1 clear
  CANFD0->MCANSS.TI_WRAPPER.MSP.CPU_INT.IMASK |= 0x02; // line 1 enable
  NVIC->ICPR[0] = (1 << 6); // NVIC_ClearPendingIRQ(6)
  NVIC->ISER[0] = (1 << 6); // NVIC_EnableIRQ(6)
  NVIC->IP[1] = (NVIC->IP[1]& (~0x00FF0000))|(priority<<22);	// 23 - 22 CANFD0_IRQHandler
}	

__STATIC_INLINE void WRITE_REG32_RAW(uint32_t addr, uint32_t value){
    *(volatile uint32_t *) addr = value;
    return;
}

// 0 if failure
// 1 if ok
int CAN_Send(uint32_t id, uint32_t dlc, uint8_t *data){
  uint16_t count;
  uint32_t regVal, loopCnt;
  uint32_t elemAddr;
  if(id > 2047) return 1;
  if(dlc > 8) return 1;

// always uses buffer 0
  elemAddr = CANFD0->MCANSS.MCAN.MCAN_TXBC&0xFFFC; // bits 15:2, 112*4= 448

  regVal =    (((uint32_t)(id << 18))  |   // 11-bit ID 28:18, 
               ((uint32_t)(0  << 29 )) |   // rtr=0 Transmit data frame
               ((uint32_t)(0  << 30 )) |   // xtd=0 11-bit standard identifier
               ((uint32_t)(0  << 31 )));   // esi=0 ESI bit in CAN FD format depends only on error passive flag
  WRITE_REG32_RAW(((uint32_t)CANFD0 + (uint32_t) elemAddr), regVal); //448,0, 0x40508000=0
  elemAddr += 4U;

  regVal =   (((uint32_t)(dlc << 16 )) |   // dlc = data length (0to 8) 
              ((uint32_t)(1   << 20 )) |   // brs=1, CAN FD frames transmitted with bit rate switching
              ((uint32_t)(1   << 21 )) |   // fdf=1, Frame transmitted in CAN FD format
              ((uint32_t)(1   << 23 )) |   // efc=1, Store Tx events 
              ((uint32_t)(0xAAU<< 24 )));  // mm = Message Marker
  WRITE_REG32_RAW(((uint32_t) CANFD0 + (uint32_t) elemAddr), regVal);
  elemAddr += 4U;

  loopCnt = 0U;
// Framing words out of the payload bytes and writing it to message RAM 
  while ((4U <= (dlc - loopCnt)) && (0U != (dlc - loopCnt))){
    regVal =       ((uint32_t) data[loopCnt] |
                   ((uint32_t) data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t) data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t) data[(loopCnt + 3U)] << 24U));
    WRITE_REG32_RAW(((uint32_t) CANFD0 + (uint32_t) elemAddr), regVal);
    elemAddr += 4U;
    loopCnt += 4U;
  }
// Framing a word out of remaining payload bytes and writing it to message RAM 
  if (0U < (dlc - loopCnt)){
    regVal =       ((uint32_t) data[loopCnt] |
                   ((uint32_t) data[(loopCnt + 1U)] << 8U) |
                   ((uint32_t) data[(loopCnt + 2U)] << 16U) |
                   ((uint32_t) data[(loopCnt + 3U)] << 24U));
    WRITE_REG32_RAW(((uint32_t) CANFD0 + (uint32_t) elemAddr), regVal);
  }

  regVal = CANFD0->MCANSS.MCAN.MCAN_TXBAR;
  regVal |= ((uint32_t) 1U << 0); // buffer 0
        /*
         * For writing to TXBAR CCE bit should be '0'. This need not be
         * reverted because for other qualified writes this is locked state
         * and can't be written.
         */
  CANFD0->MCANSS.MCAN.MCAN_CCCR &= ~0x02; //Configuration Change Enable
  CANFD0->MCANSS.MCAN.MCAN_TXBAR = regVal;
  return 1; // success
}

void getCANRxFIFOStatus(MCAN_RxFIFOStatus_t *fifoStatus){
    uint32_t regVal;
  regVal = CANFD0->MCANSS.MCAN.MCAN_RXF0S;
// 25 RF0L message lost	0	
// 24 F0F 1 means full	0
// 21-16 F0PI Rx Putindex 2
// 13-8  F0GI Rx Getindex 1
// 6-0   F0FL Fill level 1
  fifoStatus->fillLvl  = regVal&0x7F;       // HW_GET_FIELD(regVal, MCAN_RXF0S_F0FL);
  fifoStatus->getIdx   = (regVal>>8)&0x3F;  // HW_GET_FIELD(regVal, MCAN_RXF0S_F0GI);
  fifoStatus->putIdx   = (regVal>>16)&0x3F; // HW_GET_FIELD(regVal, MCAN_RXF0S_F0PI);
  fifoStatus->fifoFull = (regVal>>24)&0x01; // HW_GET_FIELD(regVal, MCAN_RXF0S_F0F);
  fifoStatus->msgLost  = (regVal>>25)&0x01; // HW_GET_FIELD(regVal, MCAN_RXF0S_RF0L);
}


void ReadCANMsgRam(void){
uint32_t startAddr = 0U, elemSize = 0U, elemAddr = 0U;
uint32_t idx = 0U;
uint32_t regVal, loopCnt;
  startAddr = CANFD0->MCANSS.MCAN.MCAN_RXF0C&0xFFFC;		// F0SA bits 15-2	
  elemSize = CANFD0->MCANSS.MCAN.MCAN_RXESC&0x07; // F0DS bits 2-0
  idx = (CANFD0->MCANSS.MCAN.MCAN_RXF0S>>8)&0x3F; // F0GI, get index is bits 13-0
  elemSize  = objSize4[elemSize];
  elemSize *= 4U;
  elemAddr = startAddr + (elemSize * idx);
//  ReadCANMsgRamRaw(elemAddr);
  regVal    = READ_REG32_RAW(((uint32_t) CANFD0 + (uint32_t) elemAddr));

// ID is stored in ID[28:18] 
  CANFIFO[CANPutI].id = (regVal&0x1FFFFFFF) >> 18;
#if CAN_DEBUG
  rxMsg4.id  = regVal&0x1FFFFFFF; // 28:0, 29-bit id, 28:18 have standard id
  rxMsg4.rtr = (regVal>>29)&0x01; // RTR bit 29 (should be 0)
  rxMsg4.xtd = (regVal>>30)&0x01; // XTD bit 30 (should be 0)
  rxMsg4.esi = (regVal>>31)&0x01; // ESI bit 31 (should be 0)
#endif
  elemAddr += 4U;
  regVal = READ_REG32_RAW(((uint32_t) CANFD0 + (uint32_t) elemAddr));
  CANFIFO[CANPutI].dlc = (regVal>>16)&0x0F;  // DLC data length, bits 19:16
#if CAN_DEBUG
  rxMsg4.rxts = (regVal>>0)&0x0FF;  // RXTS time stamp, bits 15:0
  rxMsg4.dlc  = (regVal>>16)&0x0F;  // DLC data length, bits 19:16
  rxMsg4.brs  = (regVal>>20)&0x01;  // BRS, baud rate shift, bit 20
  rxMsg4.fdf  = (regVal>>21)&0x01;  // FDF, FD format, bit 21, should be 1, CAN FD frame format
  rxMsg4.fidx = (regVal>>24)&0x7F;  // FIDX filter index, bits 30:24 
  rxMsg4.anmf = (regVal>>31)&0x01;  // ANMF bit 31, 0 means matching filter FIDX, 1 means no matching
#endif
  elemAddr += 4U;
// first four, only DLC bytes are valid        
  regVal = READ_REG32_RAW(((uint32_t) CANFD0 + (uint32_t) elemAddr));
  CANFIFO[CANPutI].data[0] = (uint8_t)(regVal & 0x000000FFU);
  CANFIFO[CANPutI].data[1] = (uint8_t)((regVal & 0x0000FF00U) >> 8U);
  CANFIFO[CANPutI].data[2] = (uint8_t)((regVal & 0x00FF0000U) >> 16U);
  CANFIFO[CANPutI].data[3] = (uint8_t)((regVal & 0xFF000000U) >> 24U);
  elemAddr += 4U;
// second four, only DLC bytes are valid 
  regVal = READ_REG32_RAW(((uint32_t) CANFD0 + (uint32_t) elemAddr));
  CANFIFO[CANPutI].data[4] = (uint8_t)(regVal & 0x000000FFU);
  CANFIFO[CANPutI].data[5] = (uint8_t)((regVal & 0x0000FF00U) >> 8U);
  CANFIFO[CANPutI].data[6] = (uint8_t)((regVal & 0x00FF0000U) >> 16U);
  CANFIFO[CANPutI].data[7] = (uint8_t)((regVal & 0xFF000000U) >> 24U);
#if CAN_DEBUG
  for(int i=0; i<rxMsg4.dlc; i++){
    rxMsg4.data[i] = CANFIFO[CANPutI].data[i]; 
  }
#endif
  if(((CANPutI+1)&(CANFIFOSIZE-1)) != CANGetI){
    CANPutI = (CANPutI+1)&(CANFIFOSIZE-1);
  }
}

bool readCANRxMsg(void){
  static MCAN_RxFIFOStatus_t rxFS;
  rxFS.fillLvl = 0;
  if ((CanInterruptLine1Status & 0x01) == 0x01){ // true
	// should have bit 0, RF0N new message in RxFifo0
    rxFS.num = 0 ; //DL_MCAN_RX_FIFO_NUM_0; // 0
    while ((rxFS.fillLvl) == 0) {
      getCANRxFIFOStatus(&rxFS);
    }
    ReadCANMsgRam();
    CANFD0->MCANSS.MCAN.MCAN_RXF0A = rxFS.getIdx; // ack the fifo
    CanInterruptLine1Status &= ~0x01; //(MCAN_IR_RF0N_MASK);
    return true;
  } else{
    return false;
  }
}


void CANFD0_IRQHandler(void){
bool processFlag;
uint16_t count;
uint32_t dlc; 
// IIDX = interrupt index status
// 00h = No interrupt pending.
// 1h = MCAN Interrupt Line 0 interrupt pending.
// 2h = MCAN Interrupt Line 1 interrupt pending.
// 3h = Message RAM SEC (Single Error Correction) interrupt pending.  
// 4h = Message RAM DED (Double Error Detection) interrupt pending.  
// 5h = External Timestamp Counter Overflow interrupt pending.
// 6h = Clock Stop Wake Up interrupt pending.

  switch (CANFD0->MCANSS.TI_WRAPPER.MSP.CPU_INT.IIDX){
    case 2: // DL_MCAN_IIDX_LINE1:
// Check MCAN interrupts fired during TX/RX of CAN package 
      CanInterruptLine1Status |= CANFD0->MCANSS.MCAN.MCAN_IR;
			// should have bit 0, RF0N new message in RxFifo0
      CANFD0->MCANSS.MCAN.MCAN_IR = CanInterruptLine1Status; //intrmask=1, MCAN_IR
      CANFD0->MCANSS.TI_WRAPPER.PROCESSORS.MCANSS_REGS.MCANSS_EOI = 2; // eoi=2, line 1 cleared
      processFlag = readCANRxMsg();
      if(processFlag == true){
        GPIOA->DOUTTGL31_0  = 1;
        OS_Signal(&CanDataAvailable);

      }
      break;
    default:
      break;
  }
}



// Returns true if receive data is available
//         false if no receive data ready
int CAN_CheckMail(void){
  return (CANGetI !=  CANPutI);
}
// if receive data is ready, gets the data and returns true
// if no receive data is ready, returns false
int CAN_GetMailNonBlock(uint32_t *id, uint32_t *dlc, uint8_t *data){
  if(CANGetI != CANPutI){
    *id  = CANFIFO[CANGetI].id;
    *dlc = CANFIFO[CANGetI].dlc;
    for(int i=0; i<*dlc; i++){
      data[i] = CANFIFO[CANGetI].data[i];
    }
    CANGetI = (CANGetI+1)&(CANFIFOSIZE-1);
    return 1;
  }
  return 0;
}
// if receive data is ready, gets the data 
// if no receive data is ready, it waits until it is ready
void CAN_GetMail(uint32_t *id, uint32_t *dlc, uint8_t *data){
  OS_Wait(&CanDataAvailable); // Use semaphore instead of spinning
  //while(CANGetI == CANPutI){};
  *id  = CANFIFO[CANGetI].id;
  *dlc = CANFIFO[CANGetI].dlc;
  for(int i=0; i<*dlc; i++){
    data[i] = CANFIFO[CANGetI].data[i];
  }
  CANGetI = (CANGetI+1)&(CANFIFOSIZE-1);
}

// 0 if failure
// 1 if ok
int CAN_SendCommand(uint32_t id, CanCommand_t* command){
  int status = CAN_Send(id, 8, (uint8_t*)command); // returns 1 if OK
  if (status == 1){
    return 1;
  }
  return 0;
}

void CAN_ReadCommand(CanCommand_t* command){
  uint32_t id, dlc;
  CAN_GetMail(&id, &dlc, (uint8_t*)command);
}
