/* ADCTimer.c
 * Jonathan Valvano
 * December 8, 2025
 * Derived from adc12_single_conversion_vref_internal_LP_MSPM0G3507_nortos_ticlang
 *              adc12_single_conversion_LP_MSPM0G3507_nortos_ticlang
 *              adc12_triggered_by_timer_event_LP_MSPM0G3507_nortos_ticlang
 */


// ****note to students****
// the data sheet says the ADC does not work when clock is 80 MHz
// however, the ADC seems to work on my boards at 80 MHz
// I suggest you try 80MHz, but if it doesn't work, switch to 40MHz

#include <ti/devices/msp/msp.h>
#include "../inc/ADCTimer.h"
#include "../inc/Clock.h"


// Assumes 80 MHz bus
// power Domain PD0
// for 80MHz bus clock, Timer G0 clock is ULPCLK 40MHz
// frequency = TimerClock/prescale/period
// e.g., period=5000,prescale=8 is 40MHz/5000/8 = 1kHz
void ADC0_TimerG0_Init(uint32_t channel, uint32_t reference,uint16_t period, uint32_t prescale, uint32_t priority){
    // Reset ADC Timer G0 and VREF
    // RSTCLR
    //   bits 31-24 unlock key 0xB1
    //   bit 1 is Clear reset sticky bit
    //   bit 0 is reset ADC
  ADC0->ULLMEM.GPRCM.RSTCTL = 0xB1000003;
  if(reference == ADCVREF_INT){
     VREF->GPRCM.RSTCTL = 0xB1000003;
  }
  TIMG0->GPRCM.RSTCTL = 0xB1000003;

    // Enable power ADC G0 and VREF
    // PWREN
    //   bits 31-24 unlock key 0x26
    //   bit 0 is Enable Power
  ADC0->ULLMEM.GPRCM.PWREN = 0x26000001;
  if(reference == ADCVREF_INT){
    VREF->GPRCM.PWREN = 0x26000001;
  }
  TIMG0->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for ADC Vref G0 to power up

  TIMG0->CLKSEL = 0x08; // bus clock
  TIMG0->CLKDIV = 0x00; // divide by 1
  TIMG0->COMMONREGS.CPS = prescale-1;     // divide by prescale,
  TIMG0->COUNTERREGS.LOAD  = period-1;     // set reload register
  TIMG0->FPUB_0 = 1; // publish on channel 1  
  // bits 29-28 CVAE 00 (set to LOAD value)
  TIMG0->COUNTERREGS.CTRCTL = 0x02;
  // bits 5-4 CM =0, down
  // bits 3-1 REPEAT =001, continue
  // bit 0 EN enable (0 for disable, 1 for enable)
  TIMG0->GEN_EVENT0.IMASK = 1; // hardware event published on Chan1 
  TIMG0->COMMONREGS.CCLKCTL = 1;

  ADC0->ULLMEM.GPRCM.CLKCFG = 0xA9000000; // ULPCLK
  // bits 31-24 key=0xA9
  // bit 5 CCONSTOP= 0 not continuous clock in stop mode
  // bit 4 CCORUN= 0 not continuous clock in run mode
  // bit 1-0 0=ULPCLK,1=SYSOSC,2=HFCLK
  ADC0->ULLMEM.CLKFREQ = 7; // 40 to 48 MHz
  ADC0->ULLMEM.CTL0 = 0x03010000;
  // bits 26-24 = 011 divide by 8
  // bit 16 PWRDN=1 for manual, =0 power down on completion, if no pending trigger
  // bit 0 ENC=1 enable (1 to 0 will end conversion)
  ADC0->ULLMEM.CTL1 = 0x00000001;
  // bits 30-28 =0  no shift
  // bits 26-24 =0  no averaging
  // bit 20 SAMPMODE=0 timer triggers
  // bits 17-16 CONSEQ=01 ADC at start will be sampled once, 10 for repeated sampling
  // bit 8 SC=0 for stop, =1 to software start
  // bit 0 TRIGSRC=1 timer trigger
  ADC0->ULLMEM.CTL2 = 0x00000000;
  // bits 28-24 ENDADD (which  MEMCTL to end)
  // bits 20-16 STARTADD (which  MEMCTL to start)
  // bits 15-11 SAMPCNT (for DMA)
  // bit 10 FIFOEN=0 disable FIFO
  // bit 8  DMAEN=0 disable DMA
  // bits 2-1 RES=0 for 12 bit (=1 for 10bit,=2for 8-bit)
  // bit 0 DF=0 unsigned formant (1 for signed, left aligned)
  ADC0->ULLMEM.MEMCTL[0] = reference+channel;
  // bit 28 WINCOMP=0 disable window comparator
  // bit 24 TRIG trigger policy, =0 for auto next, =1 for next requires trigger
  // bit 20 BCSEN=0 disable burn out current
  // bit 16 = AVGEN =0 for no averaging
  // bit 12 = STIME=0 for SCOMP0
  // bits 9-8 VRSEL = 10 for internal VREF,(00 for VDDA)
  // bits 4-0 channel = 0 to 7 available
  ADC0->ULLMEM.SCOMP0 = 0x64; // sample clocks
  ADC0->ULLMEM.SCOMP1 = 0x32; // sample clocks
  if(reference == ADCVREF_INT){
    VREF->CLKSEL = 0x00000008; // bus clock
    VREF->CLKDIV = 0; // divide by 1
    VREF->CTL0 = 0x0001;
  // bit 8 SHMODE = off
  // bit 7 BUFCONFIG=0 for 2.4 (=1 for 1.4)
  // bit 0 is enable
    VREF->CTL2 = 0;
  // bits 31-16 HCYCLE=0
    // bits 15-0 SHCYCLE=0
    while((VREF->CTL1&0x01)==0){}; // wait for VREF to be ready
  }
  ADC0->ULLMEM.FSUB_0 = 0x00000001; // subscribe to chan 1
  ADC0->ULLMEM.CPU_INT.IMASK = 0x00000100; //MEMRESIFG0
  ADC0->ULLMEM.CTL0 |= 1; // start
 
  NVIC->ISER[0] = 1 << 4; // ADC0 interrupt
  NVIC->IP[1] = (NVIC->IP[1]&(~0x000000FF))|(priority<<6);    // set priority (bits 7,6) IRQ 4

  TIMG0->COUNTERREGS.CTRCTL |= 0x01;
}

// Assumes 80 MHz bus
// power Domain PD0
// for 80MHz bus clock, Timer G8 clock is ULPCLK 40MHz
// frequency = TimerClock/prescale/period
// e.g., period=5000,prescale=8 is 40MHz/5000/8 = 1kHz
void ADC1_TimerG8_Init(uint32_t channel, uint32_t reference,uint16_t period, uint32_t prescale, uint32_t priority){
    // Reset ADC Timer G8 and VREF
    // RSTCLR
    //   bits 31-24 unlock key 0xB1
    //   bit 1 is Clear reset sticky bit
    //   bit 0 is reset ADC
  ADC1->ULLMEM.GPRCM.RSTCTL = 0xB1000003;
  if(reference == ADCVREF_INT){
     VREF->GPRCM.RSTCTL = 0xB1000003;
  }
  TIMG8->GPRCM.RSTCTL = 0xB1000003;

    // Enable power ADC G0 and VREF
    // PWREN
    //   bits 31-24 unlock key 0x26
    //   bit 0 is Enable Power
  ADC1->ULLMEM.GPRCM.PWREN = 0x26000001;
  if(reference == ADCVREF_INT){
    VREF->GPRCM.PWREN = 0x26000001;
  }
  TIMG8->GPRCM.PWREN = 0x26000001;
  Clock_Delay(24); // time for ADC Vref G8 to power up

  TIMG8->CLKSEL = 0x08; // bus clock
  TIMG8->CLKDIV = 0x00; // divide by 1
  TIMG8->COMMONREGS.CPS = prescale-1;     // divide by prescale,
  TIMG8->COUNTERREGS.LOAD  = period-1;     // set reload register
  TIMG8->FPUB_0 = 1; // publish on channel 1  
  // bits 29-28 CVAE 00 (set to LOAD value)
  TIMG8->COUNTERREGS.CTRCTL = 0x02;
  // bits 5-4 CM =0, down
  // bits 3-1 REPEAT =001, continue
  // bit 0 EN enable (0 for disable, 1 for enable)
  TIMG8->GEN_EVENT0.IMASK = 1; // hardware event published on Chan1 
  TIMG8->COMMONREGS.CCLKCTL = 1;

  ADC1->ULLMEM.GPRCM.CLKCFG = 0xA9000000; // ULPCLK
  // bits 31-24 key=0xA9
  // bit 5 CCONSTOP= 0 not continuous clock in stop mode
  // bit 4 CCORUN= 0 not continuous clock in run mode
  // bit 1-0 0=ULPCLK,1=SYSOSC,2=HFCLK
  ADC1->ULLMEM.CLKFREQ = 7; // 40 to 48 MHz
  ADC1->ULLMEM.CTL0 = 0x03010000;
  // bits 26-24 = 011 divide by 8
  // bit 16 PWRDN=1 for manual, =0 power down on completion, if no pending trigger
  // bit 0 ENC=1 enable (1 to 0 will end conversion)
  ADC1->ULLMEM.CTL1 = 0x00000001;
  // bits 30-28 =0  no shift
  // bits 26-24 =0  no averaging
  // bit 20 SAMPMODE=0 timer triggers
  // bits 17-16 CONSEQ=01 ADC at start will be sampled once, 10 for repeated sampling
  // bit 8 SC=0 for stop, =1 to software start
  // bit 0 TRIGSRC=1 timer trigger
  ADC1->ULLMEM.CTL2 = 0x00000000;
  // bits 28-24 ENDADD (which  MEMCTL to end)
  // bits 20-16 STARTADD (which  MEMCTL to start)
  // bits 15-11 SAMPCNT (for DMA)
  // bit 10 FIFOEN=0 disable FIFO
  // bit 8  DMAEN=0 disable DMA
  // bits 2-1 RES=0 for 12 bit (=1 for 10bit,=2for 8-bit)
  // bit 0 DF=0 unsigned formant (1 for signed, left aligned)
  ADC1->ULLMEM.MEMCTL[0] = reference+channel;
  // bit 28 WINCOMP=0 disable window comparator
  // bit 24 TRIG trigger policy, =0 for auto next, =1 for next requires trigger
  // bit 20 BCSEN=0 disable burn out current
  // bit 16 = AVGEN =0 for no averaging
  // bit 12 = STIME=0 for SCOMP0
  // bits 9-8 VRSEL = 10 for internal VREF,(00 for VDDA)
  // bits 4-0 channel = 0 to 7 available
  ADC1->ULLMEM.SCOMP0 = 0x64; // sample clocks
  ADC1->ULLMEM.SCOMP1 = 0x32; // sample clocks
  if(reference == ADCVREF_INT){
    VREF->CLKSEL = 0x00000008; // bus clock
    VREF->CLKDIV = 0; // divide by 1
    VREF->CTL0 = 0x0001;
  // bit 8 SHMODE = off
  // bit 7 BUFCONFIG=0 for 2.4 (=1 for 1.4)
  // bit 0 is enable
    VREF->CTL2 = 0;
  // bits 31-16 HCYCLE=0
    // bits 15-0 SHCYCLE=0
    while((VREF->CTL1&0x01)==0){}; // wait for VREF to be ready
  }
  ADC1->ULLMEM.FSUB_0 = 0x00000001; // subscribe to chan 1
  ADC1->ULLMEM.CPU_INT.IMASK = 0x00000100; //MEMRESIFG0
  ADC1->ULLMEM.CTL0 |= 1; // start
 
  NVIC->ISER[0] = 1 << 5; // ADC1 interrupt
  NVIC->IP[1] = (NVIC->IP[1]&(~0x0000FF00))|(priority<<14);    // set priority (bits 15,14) IRQ 5

  TIMG8->COUNTERREGS.CTRCTL |= 0x01;
}
