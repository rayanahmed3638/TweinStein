/* SSD1306testmain.c
 * Jonathan Valvano
 * June 7, 2024
 * Derived from i2c_controller_rw_multibyte_fifo_poll_LP_MSPM0G3507_nortos_ticlang
  VCC to +3.3V
  I2C SCL: PB2 is the SSD1306 SCL, with 1.5k pullup to 3.3V
  I2C SDA: PB3 is the SSD1306 SDA
  GND to GND

*/


//#define CONFIG_MSPM0G350X
//#define SYSCONFIG_WEAK __attribute__((weak))
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/I2C.h"
#include "../inc/SSD1306.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "Image.h"
//  PA0 is red LED1,   index 0 in IOMUX PINCM table
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S2 positive logic switch,  index 39 in IOMUX PINCM table
// PB21 is S3 negative logic switch,  index 48 in IOMUX PINCM table
#define CR 13
int main(void){ uint32_t data=0; int32_t sdat=-200;
  uint8_t letter=0x20;
  int i, j, count;
  Clock_Init80MHz(0);
  LaunchPad_Init();                  // set system clock to 80 MHz
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetCursor(0,0);
  SSD1306_ClearBuffer();
  SSD1306_OutBuffer();
  SSD1306_DrawBMP(0,60,logo,14,SSD1306_WHITE);
  SSD1306_OutBuffer();
  Clock_Delay1ms(1000);

  SSD1306_ClearBuffer();
  SSD1306_DrawBMP(0,63,cal,14,SSD1306_WHITE);
  SSD1306_OutBuffer();
  Clock_Delay1ms(1000);

  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0,"----- ECE445M--------",SSD1306_WHITE);
  SSD1306_DrawString(0,16,"123456789012345678901",SSD1306_WHITE);
  SSD1306_DrawString(0,24,"abcdefghijklmnopqrstu",SSD1306_WHITE);
  SSD1306_DrawString(0,32,"ABCDEFGHIJKLMNOPQRSTU",SSD1306_WHITE);
  SSD1306_DrawString(0,40,"!@#$%^&*()_+-=",SSD1306_WHITE);
  SSD1306_DrawString(0,48,"123456789012345678901",SSD1306_WHITE);
  SSD1306_DrawString(0,56,"vwxyz,./<>?;'\"[]\\{}|",SSD1306_WHITE);
  SSD1306_OutBuffer();
  Clock_Delay1ms(1000);

  SSD1306_ClearBuffer();
  SSD1306_DrawFastHLine(0, 5,50,SSD1306_WHITE);
  SSD1306_DrawFastVLine(5,10,25,SSD1306_WHITE);
  SSD1306_OutBuffer();
  Clock_Delay1ms(1000);

  SSD1306_DrawFullImage(checker);
  Clock_Delay1ms(1000);

  for(count=0; count<21; count=count+1){
    SSD1306_ClearBuffer();
    for(i=0; i<3; i=i+1){
      for(j=0; j<5; j=j+1){
        // 16x10 enemy; 5 empty columns to left, 1 empty row below
        SSD1306_DrawBMP(5+21*j+count, 10+11*i, SmallEnemy, 0, SSD1306_WHITE);
      }
    }
    SSD1306_OutBuffer();
    Clock_Delay1ms(100);               // delay ~0.25 sec at 48 MHz
  }
  count = 0;
  do{
  // three step process: clear, build, display
    SSD1306_ClearBuffer();
    SSD1306_DrawBMP(count, 63, ti, 0, SSD1306_WHITE);
    SSD1306_OutBuffer();
    Clock_Delay1ms(10);    // delay ~0.01 sec
    count = count + 1;
  }while(count < 80);
  do{
  // three step process: clear, build, display
    SSD1306_ClearBuffer();
    SSD1306_DrawBMP(count, 63, ti, 0, SSD1306_WHITE);
    SSD1306_OutBuffer();
    Clock_Delay1ms(10);    // delay ~0.01 sec
    count = count - 1;
  }while(count > 0);
  do{
  // three step process: clear, build, display
    SSD1306_ClearBuffer();
    SSD1306_DrawBMP(count, 63, ti, 0, SSD1306_WHITE);
    SSD1306_OutBuffer();
    Clock_Delay1ms(10);    // delay ~0.01 sec
    count = count + 1;
  }while(count < 18);
  SSD1306_OutClear();

  while(1){
    SSD1306_SetCursor(0,0);
    SSD1306_OutString("data =");
    SSD1306_OutUDec(data);
    SSD1306_OutChar(CR);
    SSD1306_OutString("sdat =");
    SSD1306_OutSDec(sdat);
    SSD1306_OutChar(CR);
    SSD1306_OutString("fix1 =");
    SSD1306_OutUFix1(data);
    SSD1306_OutChar(CR);
    SSD1306_OutString("char =");
    SSD1306_OutUHex7(letter); SSD1306_OutString(" = ");SSD1306_OutChar(letter);
    Clock_Delay1ms(500);
    data = data+27;
    sdat = -1*(sdat+27);
    letter++; if(letter<0x20) letter=0x20;
  }
}

//*************TEST OF TIMING**************

uint32_t ClearTime;
uint32_t DisplayTime;
uint32_t PrintBMPTime;
uint32_t OutCharTime;
uint32_t OutDecTime;
uint32_t ClearBufferTime;
int main1(void){
  uint32_t start,offset,delay;
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SysTick->LOAD = 0xFFFFFFF;   // 2) stops
  SysTick->VAL = 0;            // 3) any write to current clears it
  SysTick->CTRL = 0x00000005;  // 4) enable SysTick with core clock
  SysTick->VAL;
  offset = (start-SysTick->VAL)&0x00FFFFFF;

  start = SysTick->VAL;
  SSD1306_OutClear(); // erase screen
  delay = (start-SysTick->VAL)&0x00FFFFFF;
  ClearTime = (delay-offset)/80; // us
  SSD1306_DrawString(0, 0,"----- RSLK 2 --------",SSD1306_WHITE);
  SSD1306_DrawString(0,16,"123456789012345678901",SSD1306_WHITE);
  SSD1306_DrawString(0,24,"abcdefghijklmnopqrstu",SSD1306_WHITE);
  SSD1306_DrawString(0,32,"ABCDEFGHIJKLMNOPQRSTU",SSD1306_WHITE);
  SSD1306_DrawString(0,40,"!@#$%^&*()_+-=",SSD1306_WHITE);
  SSD1306_DrawString(0,48,"123456789012345678901",SSD1306_WHITE);
  SSD1306_DrawString(0,56,"vwxyz,./<>?;'\"[]\\{}|",SSD1306_WHITE);
  start = SysTick->VAL;
  SSD1306_OutBuffer();
  delay = (start-SysTick->VAL)&0x00FFFFFF;
  DisplayTime = (delay-offset)/80; // us
  SSD1306_OutClear();

  start = SysTick->VAL;
  SSD1306_DrawBMP(26, 21, SmallEnemy, 0, SSD1306_WHITE);
  delay = (start-SysTick->VAL)&0x00FFFFFF;
  PrintBMPTime = (delay-offset)/80; // us

  start = SysTick->VAL;
  SSD1306_OutChar('V');
  delay = (start-SysTick->VAL)&0x00FFFFFF;
  OutCharTime = (delay-offset)/80; // us

  start = SysTick->VAL;
  SSD1306_OutUDec(12345);
  delay = (start-SysTick->VAL)&0x00FFFFFF;
  OutDecTime = (delay-offset)/80; // us


  start = SysTick->VAL;
  SSD1306_ClearBuffer();
  delay = (start-SysTick->VAL)&0x00FFFFFF;
  ClearBufferTime = (delay-offset)/80; // us
  while(1){
    SSD1306_ClearBuffer();
    for(int j=11; j<60 ; j = j+10){
      SSD1306_DrawBMP(2, j, SmallEnemy, 0, SSD1306_WHITE);
      SSD1306_DrawBMP(22, j, SmallEnemy, 0, SSD1306_WHITE);
      SSD1306_DrawBMP(42, j, SmallEnemy, 0, SSD1306_WHITE);
      SSD1306_DrawBMP(62, j, SmallEnemy, 0, SSD1306_WHITE);
      SSD1306_DrawBMP(82, j, SmallEnemy, 0, SSD1306_WHITE);
      SSD1306_DrawBMP(102, j, SmallEnemy, 0, SSD1306_WHITE);
      }
    SSD1306_OutBuffer();
  }
}

//*************TEST OF LANGUAGE**************
typedef enum {English, Spanish, Portuguese, French} Language_t;
Language_t myLanguage;
typedef enum {HELLO, GOODBYE, LANGUAGE} phrase_t;
const char Hello_English[] ="Hello";
const char Hello_Spanish[] ="\xADHola!";
const char Hello_Portuguese[] = "Ol\xA0";
const char Hello_French[] ="All\x83";
const char Goodbye_English[]="Goodbye";
const char Goodbye_Spanish[]="Adi\xA2s";
const char Goodbye_Portuguese[] = "Tchau";
const char Goodbye_French[] = "Au revoir";
const char Language_English[]="English";
const char Language_Spanish[]="Espa\xA4ol";
const char Language_Portuguese[]="Portugu\x88s";
const char Language_French[]="Fran\x87" "ais";

const char *Phrases[3][4]={
  {Hello_English,Hello_Spanish,Hello_Portuguese,Hello_French},
  {Goodbye_English,Goodbye_Spanish,Goodbye_Portuguese,Goodbye_French},
  {Language_English,Language_Spanish,Language_Portuguese,Language_French}
};
//********SSD1306_OutPhrase*****************
// Print a string of characters to the SSD1306 LCD.
// Position determined by SSD1306_SetCursor command
// The string will not automatically wrap.
// inputs: Phrase type
// outputs: none
void SSD1306_OutPhrase(phrase_t message){
  const char *ptr = Phrases[message][myLanguage];
  while(*ptr){
    SSD1306_OutChar(*ptr);
    ptr = ptr + 1;
  }
}
void SSD1306_LanguageTest(Language_t l){
  Language_t old = myLanguage;
  myLanguage = l;
  SSD1306_OutPhrase(LANGUAGE); SSD1306_OutChar(' ');
  SSD1306_OutPhrase(HELLO); SSD1306_OutChar(' ');
  SSD1306_OutPhrase(GOODBYE); SSD1306_OutChar(13);
  myLanguage = old;
}
void SSD1306_PhraseTest(phrase_t message){
  Language_t old = myLanguage;
  myLanguage = English;
  SSD1306_OutPhrase(LANGUAGE); SSD1306_OutChar(' ');
  SSD1306_OutPhrase(message); SSD1306_OutChar(13);
  myLanguage = Spanish;
  SSD1306_OutPhrase(LANGUAGE); SSD1306_OutChar(' ');
  SSD1306_OutPhrase(message); SSD1306_OutChar(13);
  myLanguage = Portuguese;
  SSD1306_OutPhrase(LANGUAGE); SSD1306_OutChar(' ');
  SSD1306_OutPhrase(message); SSD1306_OutChar(13);
  myLanguage = French;
  SSD1306_OutPhrase(LANGUAGE); SSD1306_OutChar(' ');
  SSD1306_OutPhrase(message); SSD1306_OutChar(13);
  myLanguage = old;
}
void Pause(void){
  while(LaunchPad_InS2()==0){ Clock_Delay1ms(10);} // wait for touch
  while(LaunchPad_InS2()){ Clock_Delay1ms(10);} // wait for release
}
// use this to see extended ASCII
int main2(void){uint8_t l; // main 2
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_OutString("LanguageTest\n");
  for(phrase_t myPhrase=HELLO; myPhrase<= GOODBYE; myPhrase++){
    for(Language_t myL=English; myL<= French; myL++){
      SSD1306_OutString((char *)Phrases[LANGUAGE][myL]);SSD1306_OutChar(' ');
      SSD1306_OutString((char *)Phrases[myPhrase][myL]);SSD1306_OutChar(13);
    }
  }
  Pause();
  l = 128; // extended ASCII 128 to +255
  SSD1306_OutString("Extended ASCII\n");
  SSD1306_OutPhrase(HELLO);
  while(1){
    SSD1306_OutClear();
    for(int i=0;i<8;i++){
      SSD1306_SetCursor(0,i);
      SSD1306_OutUDec(l);
      for(int j=0; j<4; j++){
       SSD1306_OutChar(' ');
       SSD1306_OutChar(l+j);
      }
      if(l == 252){
        l = 32;
      }else{
        l += 4;
      }
    }
    Pause();
  }
}



void BookExamples(void){ // examples from the book
  int8_t cc = 0x56; // (�V�)
  int32_t xx = 100;
  int16_t yy = -100;
  float zz = 3.14159265;
  printf("Hello world\n");      //Hello world
  printf("cc = %c %d\n",cc,cc);  //cc = V 86
  printf("cc = %#x\n",cc);      //cc = 0x56
  printf("xx = %c %d\n",xx,xx);  //xx = d 100
  printf("xx = %#x\n",xx);      //xx = 0x64
  printf("yy = %d\n",yy);        //yy = -100
  printf("%#x   \n",yy);        //yy = 0xffffff9c
  printf("%e \n",zz);            //zz = 3.14159e+00
  printf("%E \n",zz);            //zz = 3.14159E+00
  printf("%f     \n",zz);        //zz = 3.14159
  printf("%g     \n",zz);        //zz = 3.14159 (shorter of two, either f or e)
  printf("%3.2f     \n",zz);    //zz =  3.14
}

int main3(void){ int32_t i,n; // main 3
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_InitPrintf();
  BookExamples();
  Pause();
  n = 0;
  while(1){
    printf("\ni=");
    for(i=0; i<1; i++){
      printf("%d ",i+n);
    }
    n = n+10000000; // notice what happens when this goes above 2,147,483,647
  }
}
// test of ploting
#define T0 -125
#define Tm 125
int main4(void){ int32_t t,y; // main 4
  Clock_Init80MHz(0);
  LaunchPad_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_SetPlot(T0,Tm,-100,100,SSD1306_WHITE);
  while(1){
    SSD1306_ClearBuffer();
    for(t=T0; t<=Tm; t= t+2){
      y = 90+(-12*t*t)/1000; // quadratic example
      SSD1306_DrawPoint(t,y);
      SSD1306_OutBuffer();   // 30ms, update entire screen
    }
  }
}
