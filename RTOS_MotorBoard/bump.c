
#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/SSD1306.h"
#include "../RTOS_Labs_common/PWMA0.h"
#include "../RTOS_Labs_common/PWMA1.h"
#include "../RTOS_Labs_common/PWMG6.h"
#include "bump.h"
#include <stddef.h>

#define bump_right_pin (1<<31)
#define bump_left_pin  (1<<27)
#define bump_center_pin (1<<28)
#define bump_pins (bump_right_pin | bump_center_pin | bump_left_pin)

extern void (*BumpTask)(bump_status);

void bump_init(){ 
    
    IOMUX->SECCFG.PINCM[PA28INDEX] = (uint32_t) 0x00060081; // input, pull up
    IOMUX->SECCFG.PINCM[PA31INDEX] = (uint32_t) 0x00060081;
    IOMUX->SECCFG.PINCM[PA27INDEX] = (uint32_t) 0x00060081;
  // From EdgeTriggered.c
    GPIOA->POLARITY31_16 |= (2 << 22) | (2 << 24) | (2 << 30);
   GPIOA->CPU_INT.ICLR = bump_pins;
    GPIOA->CPU_INT.IMASK |= bump_pins;

  // These were probably already done by S2 task but we'll just do it again
  NVIC->IP[0] = (NVIC->IP[0]&(~0x0000FF00))|1<<14;    // set priority (bits 15,14) IRQ 1
  NVIC->ISER[0] = 1 << 1; // Group1 interrupt

    
}
bump_status bump_read(void){
    uint32_t pins = GPIOA->DIN31_0;
    bump_status status = 0x00; // no collisiosn

    if((pins & bump_right_pin) == 0 ){
        status |= bump_right;
    }
    if((pins & bump_left_pin)==0){
        status|= bump_left;
    }
    if(( pins & bump_center_pin)==0){
        status |= bump_center;
    }
    return status;
}
void bump_enable_interuppts(){
    GPIOA->CPU_INT.ICLR =  bump_pins;
    GPIOA->CPU_INT.IMASK = bump_pins;
    NVIC->IP[0] = (NVIC->IP[0] & (~0x000000FF)) | (2 << 6);  // priority 2
    
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}
void bump_disable_interuppts(){
     GPIOA->CPU_INT.IMASK &= ~bump_pins;
}
void Bump_RegisterCallback(void( *callback)(bump_status status)){
    BumpTask= callback;
}
void bump_collision(){
    PWMA0_Break();
    PWMA1_Break();
}

void Set_Servo(int16_t angle){
   uint32_t center = 3100;

    if(angle > 57)  angle = 57;
    if(angle < -57) angle = -57;

    
    int32_t offset = (angle * 1000) / 57;
    uint32_t ServoDuty = center + offset;
 
    PWMG6_SetDuty(ServoDuty);
}