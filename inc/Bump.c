// Bump.c
// RSLK Bumper switches
// negative logic internal pullup
// June 28, 2024
// TM4C MSPM0
// PA5  PA27    Left, Bump 0,
// PF0  PB15    Center Left, Bump 1,
// PB3  PA28    Center Right, Bump 2
// PC4  PA31    Right, Bump 3
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
void Bump_Init(void){
  IOMUX->SECCFG.PINCM[PA27INDEX] = (uint32_t) 0x00060081; // input, pull up
  IOMUX->SECCFG.PINCM[PB15INDEX] = (uint32_t) 0x00060081; // input, pull up
  IOMUX->SECCFG.PINCM[PA28INDEX] = (uint32_t) 0x00060081; // input, pull up
  IOMUX->SECCFG.PINCM[PA31INDEX] = (uint32_t) 0x00060081; // input, pull up
}
// bit 0 is left
// bit 1 is left center
// bit 2 is right center
// bit 3 is right
uint32_t Bump_In(void){
    // bit 27 is left
    // bit 15 is left center
    // bit 28 is right center
    // bit 31 is right
  return
    ((GPIOA->DIN31_0&(1<<27)>>(27-0))|  // PA27 into bit 0
     (GPIOB->DIN31_0&(1<<15)>>(15-1))|  // PB15 into bit 1
     (GPIOA->DIN31_0&(1<<28)>>(28-2))|  // PA28 into bit 2
     (GPIOA->DIN31_0&(1<<31)>>(31-3)))  // PA31 into bit 3
     ^0x0F; // convert to positive logic
}
