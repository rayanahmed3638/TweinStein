/* Motor.c
 * Jonathan Valvano
 * v2.0.2
 * June 28, 2024
// Motor
//   TM4C MSPM0
//   PF2  PB4  Motor_PWML, ML+, IN3, PWM M1PWM6
//   PF3  PB1  Motor_PWMR, MR+, IN1, PWM M1PWM7
//   PA3  PB0  Motor_DIR_L,ML-, IN4, GPIO 0 means forward, 1 means backward
//   PA2  PB16 Motor_DIR_R,MR-, IN2, GPIO 0 means forward, 1 means backward
 *
 */
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../inc/Clock.h"
#include "../inc/PWM1.h"
#define DRV8847 1
#define DIR_L 1
#define DIR_R (1<<16)
#define MOTOR_PERIOD 10000
void Motor_Init(void){
  // LaunchPad_Init was called
  IOMUX->SECCFG.PINCM[PB0INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PB16INDEX] = (uint32_t) 0x00000081;
  // DOE31_0 Data output enable
  GPIOB->DOE31_0 |= (1 | (1<<16)); // enable outputs
  GPIOB->DOUTCLR31_0 = DIR_L | DIR_R;  // forward

  PWM1_Init(PWMUSEBUSCLK,39,MOTOR_PERIOD,MOTOR_PERIOD-4,MOTOR_PERIOD-4); // 100Hz, stopped

}
#if DRV8847
void Motor_Forward(uint32_t dutyLeft, uint32_t dutyRight){
  GPIOB->DOUTCLR31_0 = DIR_L | DIR_R;  // forward
  PWM1_SetDuty(MOTOR_PERIOD-dutyLeft,MOTOR_PERIOD-dutyRight);
}

void Motor_Backward(uint32_t dutyLeft, uint32_t dutyRight){
  GPIOB->DOUTSET31_0 = DIR_L | DIR_R;  // backward
  PWM1_SetDuty(dutyLeft,dutyRight);
}
void Motor_Right(uint32_t dutyLeft, uint32_t dutyRight){
  GPIOB->DOUTCLR31_0 = DIR_L;  // left forward
  GPIOB->DOUTSET31_0 = DIR_R;  // right backward
  PWM1_SetDuty(MOTOR_PERIOD-dutyLeft,dutyRight);
}
void Motor_Left(uint32_t dutyLeft, uint32_t dutyRight){
  GPIOB->DOUTCLR31_0 = DIR_R;  // right forward
  GPIOB->DOUTSET31_0 = DIR_L;  // left backward
  PWM1_SetDuty(dutyLeft,MOTOR_PERIOD-dutyRight);
}
#else
// L293 code
void Motor_Forward(uint32_t dutyLeft, uint32_t dutyRight){
  GPIOB->DOUTCLR31_0 = DIR_L | DIR_R;  // forward
  PWM1_SetDuty(MOTOR_PERIOD-dutyLeft,MOTOR_PERIOD-dutyRight);
}

void Motor_Backward(uint32_t dutyLeft, uint32_t dutyRight){
  GPIOB->DOUTSET31_0 = DIR_L | DIR_R;  // backward
  PWM1_SetDuty(dutyLeft,dutyRight);
}
void Motor_Right(uint32_t dutyLeft, uint32_t dutyRight){
  GPIOB->DOUTCLR31_0 = DIR_L;  // left forward
  GPIOB->DOUTSET31_0 = DIR_R;  // right backward
  PWM1_SetDuty(MOTOR_PERIOD-dutyLeft,dutyRight);
}
void Motor_Left(uint32_t dutyLeft, uint32_t dutyRight){
  GPIOB->DOUTCLR31_0 = DIR_R;  // right forward
  GPIOB->DOUTSET31_0 = DIR_L;  // left backward
  PWM1_SetDuty(dutyLeft,MOTOR_PERIOD-dutyRight);
}
#endif
