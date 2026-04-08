/* PWM1.c
 * Jonathan Valvano
 * April 3, 2024
 * Derived from timx_timer_mode_pwm_edge_sleep_LP_MSPM0G3507_nortos_ticlang
 
 */

/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define CONFIG_MSPM0G350X
#define SYSCONFIG_WEAK __attribute__((weak))
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/PWM1.h"
#include "../inc/Clock.h"
//  Timer A1  is on Power domain PD1
// for 32MHz bus clock, Timer clock is 32MHz
// for 40MHz bus clock, Timer clock is MCLK 40MHz
// for 80MHz bus clock, Timer clock is MCLK 80MHz
// CCP0 PWM output on PB4
// CCP1 PWM output on PB1
// PB4/TIMA1_C0;
// PB1/TIMA1_C1;
#define PB1INDEX  12 // UART0_RX  SPI1_CS3  TIMA1_C1  TIMA0_C2N
#define PB4INDEX  16 // UART1_TX  UART3_CTS TIMA1_C0  TIMA0_C2  TIMA1_C0N

// Initialize PWM outputs on PA12 PA13
// Rising edge synchronized
// timerClkSrc = 2 for 32768 Hz LFCLK
//             = 4 for 4MHz MFCLK
//             = 8 for 80/40/32/4 BUSCLK
// divide clock by timerClkPrescale+1, 0 to 255
// period sets the PWM period
// timerClkSrc could be 40MHz, 32MHz, 4MHz, or 32767Hz
// timerClkDivRatio = 1
// PWMFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1) * period))
// For example, source=LFCLK, prescale=0, period = 1000, PWM frequency = 32.768 Hz
// For example, source=BUSCLK, 80MHz bus, prescale=39, period = 10000, PWM frequency = 100Hz
void PWM1_Init(uint32_t timerClkSrc, uint32_t timerClkPrescale, uint32_t period, uint32_t duty0, uint32_t duty1){
  TIMA1->GPRCM.RSTCTL = (uint32_t)0xB1000003;
  TIMA1->GPRCM.PWREN = (uint32_t)0x26000001;
  Clock_Delay(2); // time for TimerA1 to power up
  IOMUX->SECCFG.PINCM[PB4INDEX]  = 0x00000084; // TIMA1 output CCP0
  IOMUX->SECCFG.PINCM[PB1INDEX]  = 0x00000084; // TIMA1 output CCP1

  TIMA1->CLKSEL = timerClkSrc; // 8=BUSCLK, 4= MFCLK, 2= LFCLK clock
  TIMA1->CLKDIV = 0x00; // divide by 1
  TIMA1->COMMONREGS.CPS = timerClkPrescale;     // divide by prescale+1,
  // 32768Hz/256 = 256Hz, 7.8125
  TIMA1->COUNTERREGS.LOAD  = period-1;  // reload register sets period
  // interrupts at 200kHz/period, 5us*period
  TIMA1->COUNTERREGS.CTRCTL = 0x02;
  // bits 5-4 CM =0, down
  // bits 3-1 REPEAT =001, continue
  // bit 0 EN enable (0 for disable, 1 for enable)
  TIMA1->COUNTERREGS.CCCTL_01[0] = 0; // no capture
  TIMA1->COUNTERREGS.CCCTL_01[1] = 0; // no capture
  // COC compare mode
  TIMA1->GEN_EVENT0.IMASK = 0x00; // no interrupts
  TIMA1->COMMONREGS.CCPD = 0x03;  // output CCP1 CCP0
  TIMA1->COMMONREGS.CCLKCTL = 1;
  TIMA1->COUNTERREGS.CC_01[0] = duty0;
  TIMA1->COUNTERREGS.CC_01[1] = duty1;
  TIMA1->COUNTERREGS.OCTL_01[0] = 0x0000; // connected to PWM
  TIMA1->COUNTERREGS.OCTL_01[1] = 0x0000; // connected to PWM
  TIMA1->COUNTERREGS.CCACT_01[0] = 0x0088;
  TIMA1->COUNTERREGS.CCACT_01[1] = 0x0088;
  // bits 10-9 CUACT 0 for no action on up compare
  // bits 7-6 CDACT 10 for make low on compare event down
  // bits 4-3 LACT 01 for make high on load event
  // bits 1-0 ZACT 00 for no action on zero event
  TIMA1->COUNTERREGS.CTRCTL |= 0x01;
}
void PWM1_SetDuty(uint32_t duty0, uint32_t duty1){
  TIMA1->COUNTERREGS.CC_01[0] = duty0;
  TIMA1->COUNTERREGS.CC_01[1] = duty1;
}


