/* RTOS_TFLuna.c
 * Jonathan Valvano
 * Oct 22, 2025
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)
 */

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../RTOS_Labs_common/TFLuna1.h"
#include "../RTOS_Labs_common/TFLuna2.h"
#include "../RTOS_Labs_common/TFLuna3.h"
#include "../RTOS_Labs_common/SPI.h"
#include "../RTOS_Labs_common/ST7735_SDC.h"
#include <stdio.h>
/* RTOS sensor board supported three TF-Luna sensors
    Serial TxD: PA17 is UART1 Tx (MSPM0 to TFLuna1)
    Serial RxD: PA18 is UART1 Rx (TFLuna1 to MSPM0)
    Serial TxD: PB17 is UART2 Tx (MSPM0 to TFLuna2)
    Serial RxD: PB18 is UART2 Rx (TFLuna2 to MSPM0)
    Serial TxD: PB12 is UART3 Tx (MSPM0 to TFLuna3), shared with LD19 Lidar 
    Serial RxD: PB13 is UART3 Rx (TFLuna3 to MSPM0) 
    UART3 is shared between LD19 and TFLuna3 (can have either but not both)
    overhead of TFLuna will be 17us/measurement at 100 measurements/sec
   */
uint32_t Dist1,Dist2,Dist3;
uint32_t Count1,Count2,Count3;
void Background1(uint32_t d){
  Dist1 = d;
  Count1++;
}
void Background2(uint32_t d){
  Dist2 = d;
  Count2++;
}
void Background3(uint32_t d){
  Dist3 = d;
  Count3++;
}
int main(void){
  __disable_irq();
  Clock_Init80MHz(0); // no clock out to pin
  LaunchPad_Init();
  SPI1_Init();
  ST7735_InitR(INITR_REDTAB); //INITR_REDTAB for AdaFruit, INITR_BLACKTAB for SPI HiLetgo ST7735R
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_SetCursor(0, 0);
  ST7735_OutString("TF Luna test\n");
  ST7735_OutString("Dist1= \n");
  ST7735_OutString("Dist2= \n");
  ST7735_OutString("Dist3= \n");
  TFLuna1_Init(&Background1);
  TFLuna2_Init(&Background2);
  TFLuna3_Init(&Background3);
  Count1=Count2=Count3=0;
   __enable_irq();
  TFLuna1_Format_Standard_mm(); // format in mm
  TFLuna1_Frame_Rate();         // 100 samples/sec
  TFLuna1_SaveSettings();  // save format and rate
  TFLuna1_System_Reset();  // start measurements

  TFLuna2_Format_Standard_mm(); // format in mm
  TFLuna2_Frame_Rate();         // 100 samples/sec
  TFLuna2_SaveSettings();  // save format and rate
  TFLuna2_System_Reset();  // start measurements

  TFLuna3_Format_Standard_mm(); // format in mm
  TFLuna3_Frame_Rate();         // 100 samples/sec
  TFLuna3_SaveSettings();  // save format and rate
  TFLuna3_System_Reset();  // start measurements
   while(1){
    if(Count1){
      Count1 = 0;
      ST7735_SetCursor(7, 1);
      ST7735_OutUDec4(Dist1);
    }
    if(Count2){
      Count2 = 0;
      ST7735_SetCursor(7, 2);
      ST7735_OutUDec4(Dist2);
    }
    if(Count3){
      Count3 = 0;
      ST7735_SetCursor(7, 3);
      ST7735_OutUDec4(Dist3);
    }
  }
}




