/* InputCapture.c
 * Jonathan Valvano
 * June 11, 2024
 * Derived from timx_timer_mode_capture_edge_capture_LP_MSPM0G3507_nortos_ticlang
 
 */

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/InputCapture.h"
#include "../inc/Clock.h"

// input wave connected to PA12, TG0_C0
#define PA12INDEX 33
// dual signal waves connected to
//   PB8  ELA  TA0_C0
//   PB12 ERA  TA0_C1

// G0/G8 in power domain PD0, so
// for 32MHz bus clock, SYSCLK clock is 32MHz
// for 40MHz bus clock, SYSCLK clock is ULPCLK 20MHz
// for 80MHz bus clock, SYSCLK clock is ULPCLK 40MHz
// Arm interrupts on rising edge of PA12
// interrupts will be enabled in main after all initialization
// timerClkSrc = 2 for 32768 Hz LFCLK
//             = 4 for 4MHz MFCLK
//             = 8 for 80/32/4 BUSCLK
// prescale divide clock by prescale+1, 0 to 255
// priority is 0(highest),1,2 or 3(lowest)
  /*
   * timerClkSrc could be 40 MHz, 20MHz, 32MHz, 4MHz, or 32767Hz
   * timerClkDivRatio = 1
   * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
   * resolution = (timerClkDivRatio * (timerClkPrescale + 1)))/timerClkSrc
   * For example, source=LFCLK, prescale=255, resolution = 7.8125ms
   * For example, source=BUSCLK, prescale=19, resolution = 1us
   */
void InputCapture_Init(uint32_t timerClkSrc, uint32_t timerClkPrescale, uint32_t priority){
//  LaunchPad_Init();
  TIMG0->GPRCM.RSTCTL = 0xB1000003;
  TIMG0->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for TimerG0 to power up
  IOMUX->SECCFG.PINCM[PA12INDEX]  = 0x00040084; // TIMG0 CCP0

  TIMG0->CLKSEL = timerClkSrc; // 8=BUSCLK, 4= MFCLK, 2= LFCLK clock
  TIMG0->CLKDIV = 0x00; // divide by 1
  TIMG0->COMMONREGS.CPS = timerClkPrescale;     // divide by prescale+1,
  // 32768Hz/256 = 256Hz, 7.8125
  TIMG0->COUNTERREGS.LOAD  = 0xFFFF;  // set reload register
  // interrupts at rising edge of PA12
  TIMG0->COUNTERREGS.CTRCTL = 0x02;
  // bits 5-4 CM =0, down
  // bits 3-1 REPEAT =001, continue
  // bit 0 EN enable (0 for disable, 1 for enable)
  TIMG0->COUNTERREGS.CCCTL_01[0] = 0x00020001;
  // CCCTL_01
  // bit 17 1 capture
  // bits 14-12 ZCOND zero condition
  // bits 10-8 LCOND load condition
  // bits 6-4 ACOND advance condition
  // bits 2-0 CCOND capture condition, 1 is rising edge of CCP
  TIMG0->CPU_INT.IMASK |= 0x10; // CCD0 mask
  TIMG0->COMMONREGS.CCLKCTL = 1;
  TIMG0->COUNTERREGS.IFCTL_01[0] = 0x0002;
  // bit 12 FE =0 bypass
  // bit 11 CPV=0 voting
  // bit 9-8 FP=00 filter period 3
  // bit 7 INV=0 noninverted
  // bits 3-0 =0010 input is CCP0
  NVIC->ISER[0] = 1 << 16; // TIMG0 interrupt
  NVIC->IP[4] = (NVIC->IP[4]&(~0x000000FF))|(priority<<6);    // set priority (bits 7,6) IRQ 16
  TIMG0->COUNTERREGS.CTRCTL |= 0x01;
}

// Power Domain PD1
// dual signal waves connected to
//   PB8  ELA  TA0_C0
//   PB12 ERA  TA0_C1
// initialize A0 for rising edge input capture interrupt
// for 32MHz bus clock, Timer clock is 32MHz
// for 40MHz bus clock, Timer clock is MCLK 40MHz
// for 80MHz bus clock, Timer clock is MCLK 80MHz
// Arm interrupts on rising edge of PB8, PB12
// interrupts will be enabled in main after all initialization
// timerClkSrc = 2 for 32768 Hz LFCLK
//             = 4 for 4MHz MFCLK
//             = 8 for 80/32/4 BUSCLK
// prescale divide clock by prescale+1, 0 to 255
// priority is 0(highest),1,2 or 3(lowest)
  /*
   * timerClkSrc could be 80 MHz, 40MHz, 32MHz, 4MHz, or 32767Hz
   * timerClkDivRatio = 1
   * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
   * resolution = (timerClkDivRatio * (timerClkPrescale + 1)))/timerClkSrc
   * For example, source=LFCLK, prescale=255, resolution = 7.8125ms
   * For example, source=BUSCLK, prescale=79, resolution = 1us
   */
void InputCapture_InitDual(uint32_t timerClkSrc, uint32_t timerClkPrescale, uint32_t priority){
 // assumes LaunchPad_Init() has been called
  TIMA0->GPRCM.RSTCTL = 0xB1000003;
  TIMA0->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for TimerA0 to power up
  IOMUX->SECCFG.PINCM[PB8INDEX]  = 0x00040084; // TIMA0 CCP0
  IOMUX->SECCFG.PINCM[PB12INDEX] = 0x00040085; // TIMA0 CCP1

  TIMA0->CLKSEL = timerClkSrc; // 8=BUSCLK, 4= MFCLK, 2= LFCLK clock
  TIMA0->CLKDIV = 0x00; // divide by 1
  TIMA0->COMMONREGS.CPS = timerClkPrescale;     // divide by prescale+1,
  // 32768Hz/256 = 256Hz, 7.8125
  TIMA0->COMMONREGS.CCPD = 0;     // CCP are inputs
  TIMA0->COUNTERREGS.LOAD  = 0xFFFF;  // set reload register
  // interrupts on rising edge of PB8, PB12
  TIMA0->COUNTERREGS.CTRCTL = 0x02;
  // bits 5-4 CM =0, down
  // bits 3-1 REPEAT =001, continue
  // bit 0 EN enable (0 for disable, 1 for enable)
  TIMA0->COUNTERREGS.CCCTL_01[0] = 0x00020001;
  TIMA0->COUNTERREGS.CCCTL_01[1] = 0x00020001;
  // CCCTL_01
  // bit 17 1 capture
  // bits 14-12 ZCOND zero condition
  // bits 10-8 LCOND load condition
  // bits 6-4 ACOND advance condition
  // bits 2-0 CCOND capture condition, 1 is rising edge of CCP
  TIMA0->CPU_INT.IMASK |= 0x30; // CCD1 and CCD0 mask
  TIMA0->COMMONREGS.CCLKCTL = 1;
  TIMA0->COUNTERREGS.IFCTL_01[0] = 0x0002;
  // bit 12 FE =0 bypass
  // bit 11 CPV=0 voting
  // bit 9-8 FP=00 filter period 3
  // bit 7 INV=0 noninverted
  // bits 3-0 =0010 input is CCP0
  NVIC->ISER[0] = 1 << 18; // TIMA0 interrupt
  NVIC->IP[4] = (NVIC->IP[4]&(~0x00FF0000))|(priority<<22);    // set priority (bits 7,6) IRQ 18
  TIMA0->COUNTERREGS.CTRCTL |= 0x01;
}
