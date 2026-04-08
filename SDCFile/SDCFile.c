/* SDCFile
 * Jonathan Valvano
 * December 27, 2025
 * Derived
 * Derived from spi_controller_multibyte_fifo_poll_LP_MSPM0G3507_nortos_ticlang
 *              spi_controller_command_data_control_LP_MSPM0G3507_nortos_ticlang
 *              spi_peripheral_register_format_LP_MSPM0G3507_nortos_ticlang
 */


#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../RTOS_Labs_common/SPI.h"
#include "../RTOS_Labs_common/ST7735_SDC.h"
#include <stdlib.h>
#include <stdio.h>
#include "integer.h"
#include "diskio.h"
#include "ff.h"
#include "../inc/DAC.h"
#include "../inc/Timer.h"
#include "../inc/TExaS.h"

static FATFS g_sFatFs;
FIL Handle,Handle2;
FRESULT MountFresult;
FRESULT Fresult;
DRESULT Result;
DSTATUS InitStatus;
unsigned char buffer[512];
#define MAXBLOCKS 100
// Describe the error with text on the LCD and then stall.  If
// the error was caused by a mistake in configuring SPI1, then
// the LCD will probably not work.
void diskError(char *errtype, int32_t code, int32_t block){
  ST7735_DrawString(0, 0, "Err: ", ST7735_Color565(255, 0, 0));
  ST7735_DrawString(5, 0, errtype, ST7735_Color565(255, 0, 0));
  ST7735_DrawString(0, 1, "Code:     ", ST7735_Color565(255, 0, 0));
  ST7735_SetCursor(6, 1);
  ST7735_SetTextColor(ST7735_Color565(255, 0, 0));
  ST7735_OutUDec(code);
  ST7735_DrawString(0, 2, "Block:", ST7735_Color565(255, 0, 0));
  ST7735_SetCursor(7, 2);
  ST7735_OutUDec(block);
  while(1){};
}
#define FILETESTSIZE 10000
void test(void);
int main(void){ // main1 simple test
  UINT successfulreads, successfulwrites;
  uint8_t c, x, y;
  uint32_t n,d;
  // simple test of file system
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  __enable_irq();
  InitStatus = disk_initialize(0);  // initialize disk
  ST7735_InitR(INITR_BLACKTAB);  //SPI HiLetgo ST7735R
//  ST7735_InitR(INITR_REDTAB);    //SPI AdaFruit ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("File system test\n");
  ST7735_SetCursor(0, 1);
  ST7735_OutString("Mounting\n");
  if(InitStatus) diskError("disk_initialize", InitStatus, 0);
  MountFresult = f_mount(&g_sFatFs, "", 0);
  if(MountFresult){
    ST7735_DrawString(0, 2, "f_mount error", ST7735_Color565(0, 0, 255));
    while(1){};
  }
  ST7735_SetCursor(0, 2);
  ST7735_OutString("Opening for write\n");
  c = 0;
  x = 0;
  y = 10;
  n = 1;    // seed
  Fresult = f_open(&Handle2, "test.txt", FA_CREATE_ALWAYS|FA_WRITE);
  if(Fresult){
    ST7735_DrawString(0, 3, "test error", ST7735_Color565(0, 0, 255));
    while(1){};
  } else{
    for(int i=0; i<FILETESTSIZE; i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      c = ((n>>24)%10)+'0'; // random digit 0 to 9
      Fresult = f_write(&Handle2, &c, 1, &successfulwrites);
      if((successfulwrites != 1) || (Fresult != FR_OK)){
        ST7735_DrawString(0, 0, "f_write error", ST7735_Color565(0, 0, 255));
        while(1){};
      }
    }
    Fresult = f_close(&Handle2);
    if(Fresult){
      ST7735_DrawString(0, 3, "file2 f_close error", ST7735_Color565(0, 0, 255));
      while(1){};
    }
  }
  ST7735_SetCursor(0, 3);
  ST7735_OutString("Opening for read\n");

  n = 1;  // reseed, start over to get the same sequence
  Fresult = f_open(&Handle, "test.txt", FA_READ);
  if(Fresult == FR_OK){
    ST7735_DrawString(0, 4, "Opened test.txt", ST7735_Color565(0, 0, 255));
    for(int i=0; i<FILETESTSIZE; i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      d = ((n>>24)%10)+'0'; // expected random digit 0 to 9
      Fresult = f_read(&Handle, &c, 1, &successfulreads);
      if((successfulreads == 1) && (Fresult == FR_OK) && (c == d)){
        ST7735_DrawChar(x, y, c, ST7735_Color565(255, 255, 0), 0, 1);
        x = x + 6;
        if(x > 122){
          x = 0;                          // start over on the next line
          y = y + 10;
        }
        if(y > 150){
          y = 10;                         // the screen is full
        }
      } else{
        ST7735_DrawString(0, 4, "f_read error", ST7735_Color565(0, 0, 255));
        while(1){};
      }

    }
  } else{
    ST7735_DrawString(0, 4, "Error test.txt (  )", ST7735_Color565(255, 0, 0));
    ST7735_SetCursor(20, 0);
    ST7735_SetTextColor(ST7735_Color565(255, 0, 0));
    ST7735_OutUDec((uint32_t)Fresult);
    while(1){};
  }
  ST7735_SetCursor(0, 4);
  ST7735_OutString("file test passed\n");
  Fresult = f_close(&Handle);
  while(1){};
}

const char inFilename[] = "test.txt";   // 8 characters or fewer
const char outFilename[] = "out.txt";   // 8 characters or fewer

int main2(void){ // main2 longer test
  UINT successfulreads, successfulwrites;
  uint8_t c, x, y;
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  __enable_irq();
  InitStatus = disk_initialize(0);  // initialize disk
  ST7735_InitR(INITR_REDTAB); //SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("File system test\n");
  if(InitStatus) diskError("disk_initialize", InitStatus, 0);
  ST7735_SetCursor(0, 1);
  ST7735_OutString("Mounting\n");
  MountFresult = f_mount(&g_sFatFs, "", 0);
  if(MountFresult){
    ST7735_DrawString(0, 0, "f_mount error", ST7735_Color565(0, 0, 255));
    while(1){};
  }
  ST7735_SetCursor(0, 2);
  ST7735_OutString("Opening test.txt\n");
  Fresult = f_open(&Handle, inFilename, FA_READ);
  if(Fresult == FR_OK){
    ST7735_DrawString(0, 3, "Opened ", ST7735_Color565(0, 255, 0));
    ST7735_DrawString(7, 3, (char *)inFilename, ST7735_Color565(0, 255, 0));
    // get a character in 'c' and the number of successful reads in 'successfulreads'
    Fresult = f_read(&Handle, &c, 1, &successfulreads);
    x = 0;                              // start in the first column
    y = 10;                             // start in the second row
    while((Fresult == FR_OK) && (successfulreads == 1) && (y <= 130)){
      if(c == '\n'){
        x = 0;                          // go to the first column (this seems implied)
        y = y + 10;                     // go to the next row
      } else if(c == '\r'){
        x = 0;                          // go to the first column
      } else{                           // the character is printable, so print it
        ST7735_DrawChar(x, y, c, ST7735_Color565(255, 255, 255), 0, 1);
        x = x + 6;                      // go to the next column
        if(x > 122){                    // reached the right edge of the screen
          x = 0;                        // go to the first column
          y = y + 10;                   // go to the next row
        }
      }
      // get the next character in 'c'
      Fresult = f_read(&Handle, &c, 1, &successfulreads);
    }
    // close the file
    Fresult = f_close(&Handle);
  } else{
    // print the error code
    ST7735_DrawString(0, 0, "Error          (  )", ST7735_Color565(255, 0, 0));
    ST7735_DrawString(6, 0, (char *)inFilename, ST7735_Color565(255, 0, 0));
    ST7735_SetCursor(16, 0);
    ST7735_SetTextColor(ST7735_Color565(255, 0, 0));
    ST7735_OutUDec((uint32_t)Fresult);
  }
  ST7735_SetCursor(0, 3);
  ST7735_OutString("Opening out.txt\n");

  // open the file to be written
  // Options:
  // FA_CREATE_NEW    - Creates a new file, only if it does not already exist.  If file already exists, the function fails.
  // FA_CREATE_ALWAYS - Creates a new file, always.  If file already exists, it is over-written.
  // FA_OPEN_ALWAYS   - Opens a file, always.  If file does not exist, the function creates a file.
  // FA_OPEN_EXISTING - Opens a file, only if it exists.  If the file does not exist, the function fails.
  Fresult = f_open(&Handle, outFilename, FA_WRITE|FA_OPEN_ALWAYS);
  if(Fresult == FR_OK){
    ST7735_DrawString(0, 14, "Opened ", ST7735_Color565(0, 255, 0));
    ST7735_DrawString(7, 14, (char *)outFilename, ST7735_Color565(0, 255, 0));
    // jump to the end of the file
    Fresult = f_lseek(&Handle, Handle.fsize);
    // write a message and get the number of successful writes in 'successfulwrites'
    Fresult = f_write(&Handle, "Another successful write.\r\n", 27, &successfulwrites);
    if(Fresult == FR_OK){
      // print the number of successful writes
      // expect: third parameter of f_write()
      ST7735_DrawString(0, 15, "Writes:    @", ST7735_Color565(0, 255, 0));
      ST7735_SetCursor(8, 15);
      ST7735_SetTextColor(ST7735_Color565(255, 255, 255));
      ST7735_OutUDec((uint32_t)successfulwrites);
      ST7735_SetCursor(13, 15);
      // print the byte offset from the start of the file where the writes started
      // expect: (third parameter of f_write())*(number of times this program has been run before)
      ST7735_OutUDec((uint32_t)(Handle.fptr - successfulwrites));
    } else{
      // print the error code
      ST7735_DrawString(0, 15, "f_write() error (  )", ST7735_Color565(255, 0, 0));
      ST7735_SetCursor(17, 15);
      ST7735_SetTextColor(ST7735_Color565(255, 0, 0));
      ST7735_OutUDec((uint32_t)Fresult);
    }
    // close the file
    Fresult = f_close(&Handle);
  } else{
    // print the error code
    ST7735_DrawString(0, 14, "Error          (  )", ST7735_Color565(255, 0, 0));
    ST7735_DrawString(6, 14, (char *)outFilename, ST7735_Color565(255, 0, 0));
    ST7735_SetCursor(16, 14);
    ST7735_SetTextColor(ST7735_Color565(255, 0, 0));
    ST7735_OutUDec((uint32_t)Fresult);
  }
  while(1){};
}

//*******streams 16 bit data from SDC to 12-bit DAC************
#define BUFSIZE 256
const uint16_t Wave[BUFSIZE] = {
  2048,2249,2447,2642,2831,3013,3185,3347,3495,3630,3750,
  3853,3939,4007,4056,4085,4095,4085,4056,4007,3939,3853,
  3750,3630,3495,3347,3185,3013,2831,2642,2447,2249,2048,
  1847,1649,1454,1265,1083,911,749,601,466,346,243,
  157,89,40,11,1,11,40,89,157,243,346,
  466,601,749,911,1083,1265,1454,1649,1847,
  2048,2249,2447,2642,2831,3013,3185,3347,3495,3630,3750,
  3853,3939,4007,4056,4085,4095,4085,4056,4007,3939,3853,
  3750,3630,3495,3347,3185,3013,2831,2642,2447,2249,2048,
  1847,1649,1454,1265,1083,911,749,601,466,346,243,
  157,89,40,11,1,11,40,89,157,243,346,
  466,601,749,911,1083,1265,1454,1649,1847,
  2048,2249,2447,2642,2831,3013,3185,3347,3495,3630,3750,
  3853,3939,4007,4056,4085,4095,4085,4056,4007,3939,3853,
  3750,3630,3495,3347,3185,3013,2831,2642,2447,2249,2048,
  1847,1649,1454,1265,1083,911,749,601,466,346,243,
  157,89,40,11,1,11,40,89,157,243,346,
  466,601,749,911,1083,1265,1454,1649,1847,
  2048,2249,2447,2642,2831,3013,3185,3347,3495,3630,3750,
  3853,3939,4007,4056,4085,4095,4085,4056,4007,3939,3853,
  3750,3630,3495,3347,3185,3013,2831,2642,2447,2249,2048,
  1847,1649,1454,1265,1083,911,749,601,466,346,243,
  157,89,40,11,1,11,40,89,157,243,346,
  466,601,749,911,1083,1265,1454,1649,1847};
uint16_t SoundBuf[BUFSIZE];
uint16_t SoundBuf2[BUFSIZE];
uint32_t Count;
uint16_t *front; // buffer being output to DAC
uint16_t *back;  // buffer being loaded from SDC
int flag; // 1 means need data into back
#define NUMBUF 128
uint32_t BufCount; // 0 to NUMBUF-1
uint32_t IdleTime=0;
// file will be NUMBUF by BUFSIZE uint16_t binary values
int main3(void){ // run main3 to create a sound file
  UINT successfulreads, successfulwrites;
  uint8_t c, x, y;
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  __enable_irq();
  InitStatus = disk_initialize(0);  // initialize disk
  ST7735_InitR(INITR_REDTAB);   // SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("Create sound.bin\n");
  if(InitStatus) diskError("disk_initialize", InitStatus, 0);
  ST7735_SetCursor(0, 1);
  ST7735_OutString("Mounting\n");
  MountFresult = f_mount(&g_sFatFs, "", 0);
  if(MountFresult){
    ST7735_DrawString(0, 0, "f_mount error", ST7735_Color565(0, 0, 255));
    while(1){};
  }
  ST7735_SetCursor(0, 2);
  ST7735_OutString("Opening sound.bin\n");
  Fresult = f_open(&Handle2, "sound.bin", FA_CREATE_ALWAYS|FA_WRITE);
  // Options:
  // FA_CREATE_NEW    - Creates a new file, only if it does not already exist.  If file already exists, the function fails.
  // FA_CREATE_ALWAYS - Creates a new file, always.  If file already exists, it is over-written.
  // FA_OPEN_ALWAYS   - Opens a file, always.  If file does not exist, the function creates a file.
  // FA_OPEN_EXISTING - Opens a file, only if it exists.  If the file does not exist, the function fails.
  if(Fresult){
    ST7735_DrawString(0, 3, "sound.bin error", ST7735_Color565(0, 0, 255));
    while(1){};
  }
  ST7735_DrawString(0, 3, "Opened sound.bin", ST7735_Color565(0, 255, 0));
  for(int i=0;i<NUMBUF;i++){
    Fresult = f_write(&Handle2, Wave, BUFSIZE*2, &successfulwrites);
    if((successfulwrites != BUFSIZE*2) || (Fresult != FR_OK)){
      ST7735_DrawString(0, 4, "f_write error", ST7735_Color565(0, 0, 255));
      while(1){};
    }
  }
  // close the file
  Fresult = f_close(&Handle2);
  ST7735_DrawString(0, 4, "Closed sound.bin", ST7735_Color565(0, 255, 0));
  // open the file to be read
  Fresult = f_open(&Handle, "sound.bin", FA_READ);
  if(Fresult == FR_OK){
    ST7735_DrawString(0, 5, "Opened sound.bin", ST7735_Color565(0, 255, 0));
    for(int i=0;i<NUMBUF;i++){
      Fresult = f_read(&Handle, SoundBuf, BUFSIZE*2, &successfulreads);
      if(Fresult){
          ST7735_DrawString(0, 6, "read error", ST7735_Color565(255, 0, 0));
         while(1){};
      }
    }
    // close the file
    Fresult = f_close(&Handle);
  } else{
    // print the error code
    ST7735_DrawString(0, 6, "open for read error", ST7735_Color565(255, 0, 0));
  }
  ST7735_DrawString(0, 6, "Success", ST7735_Color565(0, 255, 0));
  while(1){};
}
void TIMA1_IRQHandler(void){
  if((TIMA1->CPU_INT.IIDX) == 1){ // this will acknowledge
    GPIOB->DOUTTGL31_0 = GREEN; // toggle PB27
    DAC_Out(front[Count]);
    Count++;
    if(Count == BUFSIZE){
      Count = 0;
      uint16_t *pt = front;
      front = back;
      back = pt; // swap buffers
      flag = 1;  // need more data
    }
  }
}
// YOU MUST RUN main3 before running main4 to create the sound.bin file
int main4(void){ // run main4 to stream a sound file to DAC
  UINT successfulreads, successfulwrites;
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
//  TExaS_Init(0,0,&TExaS_LaunchPadLogic);
  TExaS_Init(ADC0,6,0); // PB20 connected to PA15
  front = SoundBuf2; // buffer being output to DAC
  back = SoundBuf;   // buffer being written to from SDC
  Count = 0;
  flag = 1; // 1 means need data into back
  BufCount = 0;
  DAC_Init(); // 12bit DAC on PA15
  TimerA1_IntArm(80000000/11025,1,0); // 11.025kHz
  __enable_irq();
  InitStatus = disk_initialize(0);  // initialize disk
  ST7735_InitR(INITR_REDTAB);   // SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("Streaming sound\n");
  if(InitStatus) diskError("disk_initialize", InitStatus, 0);
  ST7735_SetCursor(0, 1);
  ST7735_OutString("Mounting\n");
  MountFresult = f_mount(&g_sFatFs, "", 0);
  if(MountFresult){
    ST7735_DrawString(0, 0, "f_mount error", ST7735_Color565(0, 0, 255));
    while(1){};
  }
  // open the file to be read
  Fresult = f_open(&Handle, "sound.bin", FA_READ);
  if(Fresult){
    ST7735_DrawString(0, 6, "open error", ST7735_Color565(255, 0, 0));
    while(1){};
  }
  while(1){
    if(flag){ // 1 means need data
      flag = 0;
      GPIOB->DOUTSET31_0 = RED; // set PB26
      // 1.5ms to 1.6ms to read 512 bytes (256 16-bit integers)
      Fresult = f_read(&Handle, back, BUFSIZE*2, &successfulreads);
      GPIOB->DOUTCLR31_0 = RED; // clear PB26
      if(Fresult){
        ST7735_DrawString(0, 6, "read error", ST7735_Color565(255, 0, 0));
        while(1){};
      }
      BufCount++;
      if(BufCount == NUMBUF){ // could have seeked
        Fresult = f_close(&Handle);
        Fresult = f_open(&Handle, "sound.bin", FA_READ);
        BufCount = 0;
      }
    }
 // other tasks
    IdleTime++;
  }
}
//*******streams 8 bit sound from SDC to 12-bit DAC************
#define BUFSIZE8 512
uint8_t Buf[BUFSIZE8];
uint8_t Buf2[BUFSIZE8];
uint32_t Count8;
uint8_t *front8; // buffer being output to DAC
uint8_t *back8;  // buffer being loaded from SDC
int flag8; // 1 means need data into back
#define NUMBUF8 (1004563/BUFSIZE8)
uint32_t BufCount8; // 0 to NUMBUF8-1
void SysTick_IntArm(uint32_t period, uint32_t priority){
  SysTick->CTRL  = 0x00;      // disable during initialization
  SysTick->LOAD  = period-1;  // set reload register
  //The ARM Cortex-M0+ only implements the most significant 2 bits of each 8-bit priority field (giving the 4 priority levels).
  SCB->SHP[1]    = (SCB->SHP[1]&(~0xC0000000))|priority<<30;    // set priority (bits 31,30)
  SysTick->VAL   = 0;         // clear count, cause reload
  SysTick->CTRL  = 0x07;      // Enable SysTick IRQ and SysTick Timer
}
void SysTick_Handler(void){
  GPIOB->DOUTTGL31_0 = GREEN; // toggle PB27
  uint32_t data = front8[Count8]<<4;
//  if(data>4095) data = 4095;
  DAC_Out(data); // 8 to 12 bit
  Count8++;
  if(Count8 == BUFSIZE8){
    Count8 = 0;
    uint8_t *pt = front8;
    front8 = back8;
    back8 = pt; // swap buffers
    flag8 = 1;  // need more data
  }
}
int main5(void){
  UINT successfulreads, successfulwrites;
  __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  //  TExaS_Init(0,0,&TExaS_LaunchPadLogic);
  // TExaS_Init(ADC0,6,0); // PB20 connected to PA15
  front8 = Buf2; // buffer being output to DAC
  back8 = Buf;   // buffer being written to from SDC
  Count8 = 0;
  flag8 = 1; // 1 means need data into back
  BufCount8 = 0;
  DAC_Init(); // 12bit DAC on PA15
  SysTick_IntArm(80000000/11025,0); // 11.025kHz
  __enable_irq();
  InitStatus = disk_initialize(0);  // initialize disk
  ST7735_InitR(INITR_REDTAB);   // SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("Streaming sound\n");
  if(InitStatus) diskError("disk_initialize", InitStatus, 0);
  ST7735_SetCursor(0, 1);
  ST7735_OutString("Mounting\n");
  MountFresult = f_mount(&g_sFatFs, "", 0);
  if(MountFresult){
    ST7735_DrawString(0, 0, "f_mount error", ST7735_Color565(0, 0, 255));
    while(1){};
  }
    // open the file to be read
  Fresult = f_open(&Handle, "Swift.bin", FA_READ);
  if(Fresult){
    ST7735_DrawString(0, 6, "open error", ST7735_Color565(255, 0, 0));
    while(1){};
  }
  while(1){
    if(flag8){ // 1 means need data
      flag8 = 0;
      GPIOB->DOUTSET31_0 = RED; // set PB26
        // 1.5ms to 1.6ms to read 512 bytes (256 16-bit integers)
      Fresult = f_read(&Handle, back8, BUFSIZE8, &successfulreads);
      GPIOB->DOUTCLR31_0 = RED; // clear PB26
      if(Fresult){
        ST7735_DrawString(0, 6, "read error", ST7735_Color565(255, 0, 0));
        while(1){};
      }
      BufCount8++;
      if(BufCount8 == NUMBUF8){ // could have seeked
        Fresult = f_close(&Handle);
        Fresult = f_open(&Handle, "Swift.bin", FA_READ);
        BufCount8 = 0;
      }
    }
   // other tasks
    IdleTime++;
  }
}





// The simple unformatted test will destroy the formatting and
// erase everything on the SD card.
void main0(void){
  uint16_t block; int i; uint32_t n; uint32_t errors = 0;
  // simple test of eDisk
    __disable_irq();
  Clock_Init80MHz(0);
  LaunchPad_Init();
  __enable_irq();
  InitStatus = disk_initialize(0);  // initialize disk

  ST7735_InitR(INITR_REDTAB); //SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("Simple unformatted test\n");
  if(InitStatus) diskError("disk_initialize", InitStatus, 0);
  n = 1;    // seed
  for(block = 0; block < MAXBLOCKS; block++){
    for(i=0; i<512; i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      buffer[i] = 0xFF&n;
    }
    GPIOB->DOUTSET31_0 = BLUE ;
    Result = disk_write (0,buffer, block, 1);
    GPIOB->DOUTCLR31_0 = BLUE ;
    if(Result) diskError("disk_write", Result, block); // save to disk
  }
  n = 1;  // reseed, start over to get the same sequence
  for(block = 0; block < MAXBLOCKS; block++){
    GPIOB->DOUTSET31_0 = RED ;
    Result = disk_read (0,buffer, block,1);
    GPIOB->DOUTCLR31_0 = RED ;
    if(Result) diskError("disk_read ", Result, block); // read from disk
    for(i=0; i<512; i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      if(buffer[i] != (0xFF&n)){
        errors = errors + 1;
      }
    }
  }
  ST7735_DrawString(0, 0, "Test done", ST7735_Color565(0, 255, 0));
  ST7735_DrawString(0, 1, "Mismatches:", ST7735_Color565(0, 255, 0));
  ST7735_SetCursor(12, 1);
  ST7735_SetTextColor(ST7735_Color565(0, 255, 0));
  ST7735_OutUDec(errors);
  while(1){};
}


