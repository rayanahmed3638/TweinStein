/* ST7735Rtestmain
 * Jonathan Valvano
 * December 28, 2025
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)
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
#include "../inc/Arabic.h"
#include "Logo.h"
#include <stdlib.h>
#include <stdio.h>

// ST7735

int main1(void){
  Clock_Init40MHz();
  LaunchPad_Init();
  SPI1_Init();
  GPIOB->DOUT31_0 = BLUE ;
  while(1){
    SPI1_Reset();
    GPIOB->DOUTTGL31_0 = BLUE;
  }
}
int main2(void){
uint32_t command=0;
  Clock_Init40MHz();
  LaunchPad_Init();
  SPI1_Init();
  GPIOB->DOUTSET31_0 = BLUE ;
  while(1){
    TFT_OutCommand(command);
    for(int i=0; i<8; i++){
      TFT_OutData(i);
    }
    command = (command+1)&0x0F;
    GPIOB->DOUTTGL31_0 = BLUE;
  }
}

int main3(void){ uint32_t data=0;
  Clock_Init40MHz();
  LaunchPad_Init();
  GPIOB->DOUTSET31_0 = BLUE ;
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("LCD test main3\n");
  while(1){
    ST7735_SetCursor(0, 1);
    ST7735_OutUDec4(data);
    data = (data+1)&0xFF;
    GPIOB->DOUTTGL31_0 = BLUE;
    Clock_Delay1ms(100); // 100ms
  }
}

const char Hello_English[] ="Hello";
const char Hello_Spanish[] ="\xADHola!";
const char Hello_Portuguese[] = "Ol\xA0";
const char Hello_French[] ="All\x83";

int main(void){ // main4
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ST7735_InitR(INITR_BLACKTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_OutString("LCD test main4\n");
  ST7735_DrawString(0, 1, (char *)Hello_English, ST7735_RED);
  ST7735_DrawString(0, 2, (char *)Hello_Spanish, ST7735_WHITE);
  ST7735_DrawString(0, 3, (char *)Hello_Portuguese, ST7735_YELLOW);
  ST7735_DrawString(0, 4, (char *)Hello_French, ST7735_CYAN);
  GPIOB->DOUTSET31_0 = BLUE ;
  while(1){
    GPIOB->DOUTTGL31_0 = BLUE;
    Clock_Delay1ms(100); // 100ms
  }
}

int main5(void){uint32_t sound; uint32_t time=0;
  Clock_Init40MHz();
  LaunchPad_Init();
  srand(1);
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_Drawaxes(ST7735_YELLOW, ST7735_BLACK, "time","rand", ST7735_WHITE, "", ST7735_BLACK, 3075, 1024);
  while(1){
    time++;
    GPIOB->DOUTTGL31_0 = BLUE;
    sound = 2048+400*rand()/RAND_MAX;
    ST7735_PlotPoint2(sound,ST7735_WHITE);
    Clock_Delay1ms(1); // 1ms
    if(time>10){
      time = 0;
      ST7735PlotIncrement();
    }
  }
}

int main6(void){uint32_t j; // main 6
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_OutString("Graphics test\n");
  ST7735_OutString("cubic function\n");
  ST7735_PlotClear(0,4095);  // range from 0 to 4095
  for(j=0;j<128;j++){
    ST7735_PlotPoints(j*j/2+900-(j*j/256)*j,32*j); // cubic,linear
    ST7735_PlotNext();
  }   // called 128 times
  while(1){
  }
}
int main7(void){ // main 7
int x, y, dx, dy;

  Clock_Init80MHz(0);
  LaunchPad_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_WHITE);

  x = 8;
  y = 64;
  dx = 1;
  dy = 1;
  ST7735_DrawBitmap(x, y, TEXAS, 104, 24);
  while((LaunchPad_InS1()==0)&&(LaunchPad_InS2()==0)){};

  while(1){
    ST7735_DrawBitmap(x, y, TEXAS, 104, 24);
    x = x + dx;
    y = y + dy;
    if((x >= 40) || (x <= -8)){
      dx = -1*dx;
    }
    if((y >= 170) || (y <= 10)){
      dy = -1*dy;
    }
  }
}

#define B ST7735_BLACK
#define Y ST7735_YELLOW
const uint16_t Enemy2[140] = { // 14 wide 10 high
  B,B,B,B,B,B,B,B,B,B,B,B,B,B,
  B,B,Y,B,B,B,B,B,B,B,B,Y,B,B,
  B,B,B,Y,B,B,B,B,B,B,Y,B,B,B,
  B,B,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,B,B,
  B,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,B,
  B,Y,Y,Y,B,Y,Y,Y,Y,B,Y,Y,Y,B,
  B,Y,B,Y,Y,Y,Y,Y,Y,Y,Y,B,Y,B,
  B,Y,B,B,Y,B,B,B,B,Y,B,B,Y,B,
  B,B,B,Y,B,B,B,B,B,B,Y,B,B,B,
  B,B,B,B,B,B,B,B,B,B,B,B,B,B};
void DrawEnemy2(int32_t x, int32_t y){
  ST7735_DrawBitmap(x, y, Enemy2, 14, 10);
}
#define T33ms 2666666
int main9(void){ int32_t i,x=64,y=80;
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  while(1){
    for(i=0;  i<20; i++){DrawEnemy2(x+i,y);   Clock_Delay(T33ms);}
    for(i=19; i>=0; i--){DrawEnemy2(x+i,y);   Clock_Delay(T33ms);}
    for(i=0;  i<20; i++){DrawEnemy2(x,  y+i); Clock_Delay(T33ms);}
    for(i=19; i>=0; i--){DrawEnemy2(x,  y+i); Clock_Delay(T33ms);}
    for(i=0;  i<20; i++){DrawEnemy2(x+i,y+i); Clock_Delay(T33ms);}
    for(i=19; i>=0; i--){DrawEnemy2(x+i,y+i); Clock_Delay(T33ms);}
  }
}

const uint16_t Enemy[96] = { // 12 wide, 8 high
  B,Y,B,B,B,B,B,B,B,B,Y,B,
  B,B,Y,B,B,B,B,B,B,Y,B,B,
  B,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,B,
  Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,Y,
  Y,Y,Y,B,Y,Y,Y,Y,B,Y,Y,Y,
  Y,B,Y,Y,Y,Y,Y,Y,Y,Y,B,Y,
  Y,B,B,Y,B,B,B,B,Y,B,B,Y,
  B,B,Y,B,B,B,B,B,B,Y,B,B};
const uint16_t EnemyBlank[96] = { // 12 by 8 black rectangle
  B,B,B,B,B,B,B,B,B,B,B,B,  B,B,B,B,B,B,B,B,B,B,B,B,
  B,B,B,B,B,B,B,B,B,B,B,B,  B,B,B,B,B,B,B,B,B,B,B,B,
  B,B,B,B,B,B,B,B,B,B,B,B,  B,B,B,B,B,B,B,B,B,B,B,B,
  B,B,B,B,B,B,B,B,B,B,B,B,  B,B,B,B,B,B,B,B,B,B,B,B};
int32_t Oldx, Oldy;
void DrawEnemy(int32_t x, int32_t y){
  if((x == Oldx)&&(y == Oldy)) return;
  ST7735_DrawBitmap(Oldx, Oldy, EnemyBlank, 12, 8);
  ST7735_DrawBitmap(x, y, Enemy, 12, 8);
  Oldx = x; Oldy = y;
}
int main10(void){ int32_t i,x=64,y=80;
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_DrawBitmap(x, y, Enemy, 12, 8); // initial object
  Oldx = x; Oldy = y;                    // initial position
  while(1){
    for(i=-50; i<+50; i+=5){DrawEnemy(x+i,y);   Clock_Delay(T33ms);}
    for(i=+50; i>-50; i-=5){DrawEnemy(x+i,y);   Clock_Delay(T33ms);}
    for(i=-50; i<+50; i+=5){DrawEnemy(x,  y+i); Clock_Delay(T33ms);}
    for(i=+50; i>-50; i-=5){DrawEnemy(x,  y+i); Clock_Delay(T33ms);}
    for(i=-50; i<+50; i+=5){DrawEnemy(x+i,y+i); Clock_Delay(T33ms);}
    for(i=+50; i>-50; i-=5){DrawEnemy(x+i,y+i); Clock_Delay(T33ms);}
  }
}
Arabic_t ArabicAlphabet[]={
alif,ayh,baa,daad,daal,dhaa,dhaal,faa,ghayh,haa,ha,jeem,kaaf,khaa,laam,meem,noon,qaaf,raa,saad,seen,sheen,ta,thaa,twe,waaw,yaa,zaa,space,dot,null
};
//Arabic_t Hello[]={meem,raa,ha,baa,alif,null}; // hello
Arabic_t Hello[]={alif,baa,ha,raa,meem,null}; // hello
//Arabic_t WeAreHonoredByYourPresence[]={sheen,raa,faa,ta,waaw,noon,alif,null}; // we are honored by your presence
Arabic_t WeAreHonoredByYourPresence[]={alif,noon,waaw,ta,faa,raa,sheen,null}; // we are honored by your presence
int main11(void){ // main 11
  Clock_Init80MHz(0);
  LaunchPad_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_WHITE);
  Arabic_SetCursor(0,15);
  Arabic_OutString(Hello);
  Arabic_SetCursor(0,31);
  Arabic_OutString(WeAreHonoredByYourPresence);
  Arabic_SetCursor(0,63);
  Arabic_OutString(ArabicAlphabet);
  while(1){

  }
}


