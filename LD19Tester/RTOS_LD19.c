/* RTOS_LD19.c
 * Jonathan Valvano
 * Nov 5, 2025
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)
*/

#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/LaunchPad.h"
#include "../inc/Clock.h"
#include "../RTOS_Labs_common/LD19.h"
#include "../RTOS_Labs_common/SPI.h"
#include "../RTOS_Labs_common/ST7735_SDC.h"
#include "../RTOS_Labs_common/fixed.h"
#include <stdio.h>
uint16_t Data[540];

int main1(void){
  __disable_irq();
  Clock_Init80MHz(0); // no clock out to pin
  LaunchPad_Init();
  
  LD19_Init(Data);
   __enable_irq();
  while(1){
  }
}
#define SCALE 1000 //(3 meters)
#define IMIN 270
#define IMAX 540
const uint32_t Colors[8]={
  ST7735_CYAN, ST7735_MAGENTA, ST7735_YELLOW, ST7735_WHITE,   
  ((228 & 0xF8) << 8) | ((228 & 0xFC) << 3) | (228 >> 3), // light grey
  ST7735_GREEN, 
  ((255 & 0xF8) << 8) | ((102 & 0xFC) << 3) | (0 >> 3), // orange
  ((106 & 0xF8) << 8) | ((13 & 0xFC) << 3) | (173 >> 3) // purple
};
int main2(void){
  __disable_irq();
  Clock_Init80MHz(0); // no clock out to pin
  LaunchPad_Init();
  SPI1_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("LD19 test\n");
  
  LD19_Init(Data);

   __enable_irq();
  while(1){
    for(int j=0; j<8; j++){
      for(int i=IMIN; i<IMAX; i++){
        uint32_t d = Data[i]; // mm
        if(d > SCALE) d = SCALE;
        uint32_t y = 191-(128*d)/SCALE;
        uint32_t x = (128*(i-IMIN))/(IMAX-IMIN);
        ST7735_DrawPixel(x, y,Colors[j]);
      }
    }
    if(LaunchPad_InS1()||LaunchPad_InS2()){
      ST7735_FillScreen(ST7735_BLACK);
      ST7735_SetCursor(0, 0);
      ST7735_OutString("LD19 test\n");
      while(LaunchPad_InS1()||LaunchPad_InS2()){
        Clock_Delay1ms(10);
      }
    }
  }
}

// plots 0 to 128
// plots 32 to 159
int main(void){
  __disable_irq();
  Clock_Init80MHz(0); // no clock out to pin
  LaunchPad_Init();
  SPI1_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("LD19 test\n");
  
  LD19_Init(Data);

   __enable_irq();
  while(1){
    for(int j=0; j<8; j++){
      for(int i=IMIN; i<IMAX; i++){
        int32_t d = Data[i]; // mm
        // 270 < i <539
        // -SCALE < d < +SCALE
        // sine cosine return -65536 to +65536
        int32_t y=(sin540(i)*d)>>16; // -SCALE < y < 0 because 270 < i <539
        int32_t x=(cos540(i)*d)>>16; // -SCALE < x < +SCALE
        int32_t ix = 64+(64*x)/SCALE; // 0 to 127
        if(ix<0)ix=0; if(ix>127) ix=127;
        int32_t iy = 32-(128*y)/SCALE;
        if(iy<32)iy=32; if(iy>159) iy=159;
        ST7735_DrawPixel(ix, iy,Colors[j]);
              }
    }
    if(LaunchPad_InS1()||LaunchPad_InS2()){
      ST7735_FillScreen(ST7735_BLACK);
      ST7735_SetCursor(0, 0);
      ST7735_OutString("LD19 test\n");
      while(LaunchPad_InS1()||LaunchPad_InS2()){
        Clock_Delay1ms(10);
      }
    }
  }
}
int main4(void){ // sin cos test
  __disable_irq();
  Clock_Init80MHz(0); // no clock out to pin
  LaunchPad_Init();
  SPI1_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("LD19 test\n");
  int32_t d = 900; // mm
  for(int i=IMIN; i<IMAX; i++){
 //       if(d > SCALE) d = SCALE;
        // 270 < i <539
        // -SCALE < d < +SCALE
        // sine cosine return -65536 to +65536
        int32_t y=(sin540(i)*d)>>16; // -SCALE < y < 0 because 270 < i <539
        int32_t x=(cos540(i)*d)>>16; // -SCALE < x < +SCALE
        int32_t ix = 64+(64*x)/SCALE; // 0 to 127
        if(ix<0)ix=0; if(ix>127) ix=127;
        int32_t iy = 32-(128*y)/SCALE;
        if(iy<32)iy=32; if(iy>159) iy=159;
        ST7735_DrawPixel(ix, iy,Colors[0]);
  }
  while(1){};
}

