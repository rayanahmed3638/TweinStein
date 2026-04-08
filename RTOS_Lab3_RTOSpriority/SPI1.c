/* SPI1.c
 * Jonathan Valvano
 * December 27, 2025
 * Derived from uart_rw_multibyte_fifo_poll_LP_MSPM0G3507_nortos_ticlang
 */

// hardware connections, ECE445M RTOS sensor board
// **********Adafruit ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10)     to +3.3 V
// MISO (pin 9)           to SPI1 POCI: PB7
// SCK (pin 8)            to SPI1 SCLK: PB9
// MOSI (pin 7)           to SPI1 PICO: PB8
// TFT_CS (pin 6)         to GPIO:  PB6
// CARD_CS (pin 5)        to PB0 (GPIO)
// Data/Command (pin 4)   to PB16 (GPIO), high for data, low for command
// RESET (pin 3)          to PB15 (GPIO)
// VCC (pin 2)            to +3.3 V
// Gnd (pin 1)            to ground

// **********HiLetgo ST7735 TFT and SDC (SDC not tested)*******************
// ST7735
// LED-   (pin 16) TFT, to ground
// LED+   (pin 15) TFT, to +3.3 V
// SD_CS  (pin 14) SDC, to PB0 chip select
// MOSI   (pin 13) SDC, to PB8 MOSI
// MISO   (pin 12) SDC, to PB7 MISO
// SCK    (pin 11) SDC, to serial clock
// CS     (pin 10) TFT, to PB6  GPIO
// SCL    (pin 9)  TFT, to PB9  SPI1 SCLK
// SDA    (pin 8)  TFT, to PB8  MOSI SPI1 PICO
// A0     (pin 7)  TFT, to PB16 Data/Command, high for data, low for command
// RESET  (pin 6)  TFT, to PB15 reset (GPIO), low to reset
// NC     (pins 3,4,5)
// VCC    (pin 2)       to +3.3 V
// GND    (pin 1)       to ground

#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/SPI1.h"
#include "../inc/Clock.h"
#include "../inc/Timer.h"
#include "../inc/LaunchPad.h"

/*
// calls Clock_Freq to get bus clock
// initialize SPI for 8 MHz baud clock
// busy-wait synchronization
// SPI0,SPI1 in power domain PD1 SysClk equals bus CPU clock
void SPI_Init(void){uint32_t busfreq =  Clock_Freq();
    // assumes GPIOA and GPIOB are reset and powered previously
    // RSTCLR to SPI1 peripherals
    //   bits 31-24 unlock key 0xB1
    //   bit 1 is Clear reset sticky bit
    //   bit 0 is reset gpio port
  SPI1->GPRCM.RSTCTL = 0xB1000003;
    // Enable power to SPI1 peripherals
    // PWREN
    //   bits 31-24 unlock key 0x26
    //   bit 0 is Enable Power
  SPI1->GPRCM.PWREN = 0x26000001;
  // configure PB9 PB6 PB8 as alternate SPI1 function
  IOMUX->SECCFG.PINCM[PB9INDEX]  = 0x00000083;  // SPI1 SCLK
  IOMUX->SECCFG.PINCM[PB6INDEX]  = 0x00000083;  // SPI1 CS0
  IOMUX->SECCFG.PINCM[PB8INDEX]  = 0x00000083;  // SPI1 PICO
  IOMUX->SECCFG.PINCM[PB15INDEX] = 0x00000081;  // GPIO output, LCD !RST
  IOMUX->SECCFG.PINCM[PB16INDEX] = 0x00000081;  // GPIO output, LCD RS
// **  IOMUX->SECCFG.PINCM[PA13INDEX] = 0x00000081;  // GPIO output, LCD RS
  Clock_Delay(24); // time for gpio to power up
// **  GPIOA->DOE31_0 |= 1<<13;    // PA13 is LCD RS
  GPIOB->DOE31_0 |= 1<<16;    // PA13 is LCD RS
  GPIOB->DOE31_0 |= 1<<15;    // PB15 is LCD !RST
  GPIOB->DOUTSET31_0 = 1<<16; // RS=1
// **  GPIOA->DOUTSET31_0 = 1<<13; // RS=1
  GPIOB->DOUTSET31_0 = 1<<15; // !RST = 1
  SPI1->CLKSEL = 8; // SYSCLK
// bit 3 SYSCLK
// bit 2 MFCLK
// bit 1 LFCLK
   SPI1->CLKDIV = 0; // divide by 1
// bits 2-0 n (0 to 7), divide by n+1
//Set the bit rate clock divider to generate the serial output clock
//     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
//     8,000,000 = (16,000,000)/((0 + 1) * 2)
//     8,000,000 = (32,000,000)/((1 + 1) * 2)
//     6,666,667 = (40,000,000)/((2 + 1) * 2)
//    10,000,000 = (40,000,000)/((1 + 1) * 2)
//     8,000,000 = (80,000,000)/((4 + 1) * 2)
//     8,000,000 = (Clock_Freq)/((m + 1) * 2)
//     m = (Clock_Freq/16000000) - 1
   if(busfreq <= 16000000){
     SPI1->CLKCTL = 0; // frequency= busfreq/2
   }else if(busfreq == 40000000){
     SPI1->CLKCTL = 1; // frequency= 10MHz
    // SPI1->CLKCTL = 2; // frequency= 6.66MHz
   }else{
     SPI1->CLKCTL = busfreq/16000000 -1; // 8 MHz
   }
   SPI1->CTL0 = 0x0027;
// bit 14 CSCLR=0 not cleared
// bits 13-12 CSSEL=0 CS0
// bit 9 SPH = 0
// bit 8 SPO = 0
// bits 6-5 FRF = 01 (4 wire)
// bits 4-0 n=7, data size is n+1 (8bit data)
  SPI1->CTL1 = 0x0015;
// bits 29-24 RXTIMEOUT=0
// bits 23-16 REPEATX=0 disabled
// bits 15-12 CDMODE=0 manual
// bit 11 CDENABLE=0 CS3
// bit 7-5 =0 no parity
// bit 4=1 MSB first
// bit 3=0 POD (not used, not peripheral)
// bit 2=1 CP controller mode
// bit 1=0 LBM disable loop back
// bit 0=1 enable SPI
  SPI_Reset();
}


//---------SPI_OutData------------
// Output 8-bit data to SPI port
// Input: data is an 8-bit data to be transferred
// Output: none
void SPI_OutData(char data){
  while((SPI1->STAT&0x02) == 0x00){}; // spin if TxFifo full
  GPIOA->DOUTSET31_0 = 1<<13;         // RS=PA13=1 for data
  SPI1->TXDATA = data;
}
 //---------SPI_OutCommand------------
 // Output 8-bit command to SPI port
 // Input: data is an 8-bit data to be transferred
 // Output: none
 void SPI_OutCommand(char command){
   while((SPI1->STAT&0x10) == 0x10){}; // spin if SPI busy
   GPIOA->DOUTCLR31_0 = 1<<13;         // RS=PA13=0 for command
   SPI1->TXDATA = command;
   while((SPI1->STAT&0x10) == 0x10){}; // spin if SPI busy
 }
 */

 //---------SPI1_Reset------------
 // Reset LCD
 // Input: none
 // Output: none
 // at 48 MHz
 void SPI1_Reset(void){
  // GPIOB->DOUTSET31_0 = 1<<15; // PB15=!RST=1
   TFT_RST_HIGH();
   Clock_Delay1ms(500);        // 500ms (calibrated with logic analyzer)
   //GPIOB->DOUTCLR31_0 = 1<<15; // PB15=!RST=0
   TFT_RST_LOW();
   Clock_Delay1ms(500);        // 500ms
   //GPIOB->DOUTSET31_0 = 1<<15; // PB15=!RST=1
   TFT_RST_HIGH();
   Clock_Delay1ms(500);        // 500ms
 }

 //********SPI1_Init*****************
// Initialize SPI1 interface to SDC and TFT
// inputs: none
// outputs: none
// outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
// 99 for   400,000 bps slow mode, used during initialization
// 4  for 8,000,000 bps fast mode, used during disk I/O
// Version for both SDC and TFT
int LCDresetFlag;
void SPI1_Init(void){
  TimerG0_IntArm(1000,40,1);            // initialize TimerG0 for 1 ms interrupts
  CS_Init();                            // initialize whichever GPIO pin is CS for the SD card
  uint32_t busfreq =  Clock_Freq();
      // assumes GPIOA and GPIOB are reset and powered previously
      // RSTCLR to SPI1 peripherals
      //   bits 31-24 unlock key 0xB1
      //   bit 1 is Clear reset sticky bit
      //   bit 0 is reset gpio port
  SPI1->GPRCM.RSTCTL = 0xB1000003;
      // Enable power to SPI1 peripherals
      // PWREN
      //   bits 31-24 unlock key 0x26
      //   bit 0 is Enable Power
  SPI1->GPRCM.PWREN = 0x26000001;
    // configure PB9 PB6 PB8 as alternate SPI1 function
  IOMUX->SECCFG.PINCM[PB9INDEX]  = 0x00000083;  // SPI1 SCLK
  //  IOMUX->SECCFG.PINCM[PB6INDEX]  = 0x00000083;  // SPI1 CS0
  IOMUX->SECCFG.PINCM[PB8INDEX]  = 0x00000083;  // SPI1 PICO
  IOMUX->SECCFG.PINCM[TFT_RST_INDEX] = 0x00000081;  // GPIO output, LCD !RST
  IOMUX->SECCFG.PINCM[TFT_DC_INDEX] = 0x00000081;  // GPIO output, LCD D/C RS
//  IOMUX->SECCFG.PINCM[PB15INDEX] = 0x00000081;  // GPIO output, LCD !RST
//  IOMUX->SECCFG.PINCM[PB16INDEX] = 0x00000081;  // GPIO output, LCD RS
// ** IOMUX->SECCFG.PINCM[PA13INDEX] = 0x00000081;  // GPIO output, LCD RS
  IOMUX->SECCFG.PINCM[PB7INDEX]  = 0x00040083;  // SPI1 POCI
  IOMUX->SECCFG.PINCM[TFT_CS_INDEX]  = 0x00000081;  // PB6 is regular GPIO
//  IOMUX->SECCFG.PINCM[PB6INDEX]  = 0x00000081;  // PB6 is regular GPIO
  Clock_Delay(24); // time for gpio to power up
  TFT_DC->DOE31_0 |= TFT_DC_PIN;    // PB16 is LCD RS
//  GPIOB->DOE31_0 |= 1<<16;    // PB16 is LCD RS
// **   GPIOA->DOE31_0 |= 1<<13;    // PA13 is LCD RS
  TFT_RST->DOE31_0 |= TFT_RST_PIN; 
  //GPIOB->DOE31_0 |= 1<<15;    // PB15 is LCD !RST
//  GPIOB->DOUTSET31_0 = 1<<16; // RS=1
  TFT_DC_HIGH();
// **  GPIOA->DOUTSET31_0 = 1<<13; // RS=1
 // GPIOB->DOUTSET31_0 = 1<<15; // !RST = 1
  TFT_RST_HIGH();
  TFT_CS->DOE31_0 |= TFT_CS_PIN;     // PB6 is regular GPIO
  //GPIOB->DOE31_0 |= 1<<6;     // PB6 is regular GPIO
  SPI1->CLKSEL = 8; // SYSCLK
  // bit 3 SYSCLK
  // bit 2 MFCLK
  // bit 1 LFCLK
  SPI1->CLKDIV = 0; // divide by 1
  // bits 2-0 n (0 to 7), divide by n+1
  //Set the bit rate clock divider to generate the serial output clock
  //     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
  //     8,000,000 = (16,000,000)/((0 + 1) * 2)
  //     8,000,000 = (32,000,000)/((1 + 1) * 2)
  //     6,666,667 = (40,000,000)/((2 + 1) * 2)
  //    10,000,000 = (40,000,000)/((1 + 1) * 2)
  //     8,000,000 = (80,000,000)/((4 + 1) * 2)
  //     8,000,000 = (Clock_Freq)/((m + 1) * 2)
  //     m = (Clock_Freq/16000000) - 1
  if(busfreq <= 16000000){
    SPI1->CLKCTL = 0; // frequency= busfreq/2
  }else if(busfreq == 40000000){
    SPI1->CLKCTL = 1; // frequency= 10MHz
     // SPI1->CLKCTL = 2; // frequency= 6.66MHz
  }else{
    SPI1->CLKCTL = busfreq/16000000 -1; // 8 MHz
   }
   SPI1->CTL0 = 0x0007;
  // bit 14 CSCLR=0 not cleared
  // bits 13-12 CSSEL=0 CS0
  // bit 9 SPH = 0
  // bit 8 SPO = 0
  // bits 6-5 FRF = 00 (3 wire)
  // bits 4-0 n=7, data size is n+1 (8bit data)
    SPI1->CTL1 = 0x0015;
  // bits 29-24 RXTIMEOUT=0
  // bits 23-16 REPEATX=0 disabled
  // bits 15-12 CDMODE=0 manual
  // bit 11 CDENABLE=0 CS3
  // bit 7-5 =0 no parity
  // bit 4=1 MSB first
  // bit 3=0 POD (not used, not peripheral)
  // bit 2=1 CP controller mode
  // bit 1=0 LBM disable loop back
  // bit 0=1 enable SPI
  TFT_CS_HIGH();             // disable LCD
  if(LCDresetFlag) return;
  LCDresetFlag = 1;
  SPI1_Reset();


}

// SDC CS initialization
void CS_Init(void){
  IOMUX->SECCFG.PINCM[SDC_CS_INDEX]  = (uint32_t) 0x00000081;
 // IOMUX->SECCFG.PINCM[PB0INDEX]  = (uint32_t) 0x00000081;
// **   IOMUX->SECCFG.PINCM[PA12INDEX]  = (uint32_t) 0x00000081;
  SDC_CS->DOE31_0 |= SDC_CS_PIN;
  SDC_CS_HIGH(); // PB0=1
}

/* STAT register
 * 4 BUSY 0h = SPI is in idle mode. 1h = SPI is currently transmitting and/or receiving data, or transmit FIFO is not empty.
   3 RNF Receive FIFO not full 0h = Receive FIFO is full. 1h = Receive FIFO is not full.
   2 RFE Receive FIFO empty. 0h = Receive FIFO is not empty. 1h = Receive FIFO is empty.
   1 TNF Transmit FIFO not full 0h = Transmit FIFO is full. 1h = Transmit FIFO is not full.
   0 TFE Transmit FIFO empty. 0h = Transmit FIFO is not empty. 1h = Transmit FIFO is empty.*/
   
//---------TFT_OutData------------
// Output 8-bit data to SPI port
// Input: data is an 8-bit data to be transferred
// Output: none
void TFT_OutData(char data){char response;
 // while((SPI1->STAT&0x02) == 0x00){}; // spin if TxFifo full
  while((SPI1->STAT&0x10) == 0x10){}; // spin if SPI busy
  SDC_CS_HIGH(); // PB0 high
  TFT_CS_LOW();  // PB6 low
 // GPIOB->DOUTSET31_0 = 1<<16;         // RS=PB16=1 for data
  TFT_DC_HIGH();  // RS=PB16=1 for data
// **  GPIOA->DOUTSET31_0 = 1<<13;         // RS=PA13=1 for data
  SPI1->TXDATA = data;
  while((SPI1->STAT&0x04) == 0x04){}; // spin SPI RxFifo empty
  response = SPI1->RXDATA; // has no meaning, flush
  TFT_CS_HIGH();  // PB6 high
}
 //---------TFT_OutCommand------------
 // Output 8-bit command to SPI port
 // Input: data is an 8-bit data to be transferred
 // Output: none
 void TFT_OutCommand(char command){char response;
   while((SPI1->STAT&0x10) == 0x10){}; // spin if SPI busy
   SDC_CS_HIGH(); // PB0 high
   TFT_CS_LOW();  // PB6 low
   //GPIOB->DOUTCLR31_0 = 1<<16;         // RS=PB16=0 for command
   TFT_DC_LOW(); // RS=PB16=0 for command
// **   GPIOA->DOUTCLR31_0 = 1<<13;         // RS=PA13=0 for command
   SPI1->TXDATA = command;
   while((SPI1->STAT&0x10) == 0x10){}; // spin if SPI busy
   response = SPI1->RXDATA; // has no meaning, flush
   TFT_CS_HIGH();  // PB6 high
 }