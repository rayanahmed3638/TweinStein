/* UART2.c
 * Your name
 * Data:
 * PA22 UART2 Rx from other microcontroller PA8 UART1 Tx<br>
 */

#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/UART2.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
uint32_t LostData2;

// power Domain PD0
// for 80MHz bus clock, UART clock is ULPCLK 40MHz
// initialize UART2 for 2000 baud rate
// no transmit, interrupt on receive timeout
void UART2_Init(void){
    // do not reset or activate PortA, already done in LaunchPad_Init
    // RSTCLR to GPIOA and UART2 peripherals
    //   bits 31-24 unlock key 0xB1
    //   bit 1 is Clear reset sticky bit
    //   bit 0 is reset gpio port
 // GPIOA->GPRCM.RSTCTL = (uint32_t)0xB1000003; // called previously
  UART2->GPRCM.RSTCTL = 0xB1000003;
    // Enable power to GPIOA and UART1 peripherals
    // PWREN
    //   bits 31-24 unlock key 0x26
    //   bit 0 is Enable Power
 // GPIOA->GPRCM.PWREN = (uint32_t)0x26000001; // called previously
  UART2->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for uart to power up
 // configure PA22 PA8 as alternate UART2 function
  IOMUX->SECCFG.PINCM[PA22INDEX]  = 0x00040082;
  //bit 18 INENA input enable
  //bit 7  PC connected
  //bits 5-0=2 for UART1_Rx
  Fifo1_Init();
  LostData2 = 0;
  UART2->CLKSEL = 0x08; // bus clock
  UART2->CLKDIV = 0x00; // no divide
  UART2->CTL0 &= ~0x01; // disable UART1
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
     // Baud = 2000
      //   1,250,000/2000 = 625
      //   divider = 625
    UART2->IBRD = 625;
    UART2->FBRD = 0; // baud =1,250,000/625 = 2000
  }else if (Clock_Freq() == 32000000){
    // 32000000/16 = 2,000,000
     // Baud = 2000
      //   2,000,000/2000 = 1000
      //   divider = 1000
    UART2->IBRD = 1000;
    UART2->FBRD = 0;
  }else if (Clock_Freq() == 80000000){
     // 40000000/16 = 2,500,000 Hz
     // Baud = 2000
      //    2,500,000/2000 = 1250
      //   divider = 1250
    UART2->IBRD = 1250;
    UART2->FBRD = 0; // baud =2,500,000/1250 = 2000
  }else return;
  UART2->LCRH = 0x00000030;
   // bits 5-4 WLEN=11 8 bits
   // bit  3   STP2=0  1 stop
   // bit  2   EPS=0   parity select
   // bit  1   PEN=0   no parity
   // bit  0   BRK=0   no break
  UART2->CPU_INT.IMASK = 0x0001;
  // bit 11 TXINT=0 no transmit interrupt
  // bit 10 RXINT=0 no receive interrupt
  // bit 0  Receive timeout
  UART2->IFLS = 0x0422;
  // bits 11-8 RXTOSEL receiver timeout select 4 (0xF highest)
  // bits 6-4  RXIFLSEL 2 is greater than or equal to half
  // bits 2-0  TXIFLSEL 2 is less than or equal to half
  NVIC->ICPR[0] = 1<<14; // UART2 is IRQ 14
  NVIC->ISER[0] = 1<<14;
  NVIC->IP[3] = (NVIC->IP[3]&(~0x00FF0000))|(2<<22);  // set priority (bits 23-22) IRQ 14
  UART2->CTL0 |= 0x01; // enable UART2
}
//------------UART2_InChar------------
// Get new serial port receive data from FIFO1
// Input: none
// Output: Return 0 if the FIFO1 is empty
//         Return nonzero data from the FIFO1 if available
char UART2_InChar(void){
  return Fifo1_Get();
}


void UART2_IRQHandler(void){ uint32_t status; char letter;
  status = UART2->CPU_INT.IIDX; // reading clears bit in RIS
  if(status == 0x01){   // 0x01 receive timeout
    GPIOB->DOUTTGL31_0 = BLUE; // toggle PB22 (minimally intrusive debugging)
    GPIOB->DOUTTGL31_0 = BLUE; // toggle PB22 (minimally intrusive debugging)
    while((UART2->STAT&0x04) == 0){
      letter = UART2->RXDATA;
      if(Fifo1_Put(letter)==0){
        LostData2++;
      }
    }
    GPIOB->DOUTTGL31_0 = BLUE; // toggle PB22 (minimally intrusive debugging)
  }
}
