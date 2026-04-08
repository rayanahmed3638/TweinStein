/*!
 * @defgroup RTOS_PWM
 * @brief Pulse width modulation
 <table>
<caption id="RTOSPWMpins2">PWM pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>Description
<tr><td>PA8  <td>TA0_C0 PWM output
<tr><td>PA9  <td>TA0_C1 PWM output
<tr><td>PB4  <td>TA1_C0 PWM output
<tr><td>PB1  <td>TA1_C1 PWM output
</table>
 * @{*/
/**
 * @file      RTOS_PWM.h
 * @brief     Pulse width modulation
 * @details   Hardware creates fixed period, variable duty cycle waves
 * \image html PWM_100Hz.png  width=500px
 * @version   ECE319K v1.0
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 19, 2025
 <table>
<caption id="RTOSPWMpins3">PWM pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>Description
<tr><td>PA8  <td>TA0_C0 PWM output
<tr><td>PA9  <td>TA0_C1 PWM output
<tr><td>PB4  <td>TA1_C0 PWM output
<tr><td>PB1  <td>TA1_C1 PWM output
</table>
  ******************************************************************************/

/* 
 * Derived from timx_timer_mode_pwm_edge_sleep_LP_MSPM0G3507_nortos_ticlang
 
 */

#ifndef __PWM1_H__
#define __PWM1_H__

/**
 * \brief use PWMUSELFCLK to select LFCLK
 */
#define PWMUSELFCLK 2
/**
 * \brief use PWMUSEMFCLK to select MFCLK
 */
#define PWMUSEMFCLK 4
/**
 * \brief use PWMUSEBUSCLK to select bus CLK
 */
#define PWMUSEBUSCLK 8
/**
 * Initialize PWM outputs on PA8 PA9.
 * Rising edge synchronized. timerClkDivRatio = 1.
 * Once started, hardware will continuously output the waves.
 * - timerClkSrc = 
 *    - 2 for 32768 Hz LFCLK
 *    - 4 for 4MHz MFCLK (not tested)
 *    - 8 for 80/32/4 BUSCLK
 * - TimerA0  is on Power domain PD1
 *    - for 32MHz bus clock, Timer clock is 32MHz
 *    - for 40MHz bus clock, Timer clock is MCLK 40MHz
 *    - for 80MHz bus clock, Timer clock is MCLK 80MHz
 * - PWMFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1) * period))
 *    - For example, source=LFCLK, prescale = 0, period = 1000, PWM frequency = 32.768 Hz
 *    - For example, source=BUSCLK, 80MHz bus, prescale=79, period = 10000, PWM frequency = 100Hz 
 *
 * @param timerClkSrc is 2 4 or 8
 * @param timerClkPrescale divide clock by timerClkPrescale+1, 0 to 255
 * @param period sets the PWM period
 * @param duty0 sets the duty cycle on PA8 
 * @param duty1 sets the duty cycle on PA9 
 * @return none 
 * @note  Assumes LaunchPad_Init has been called to reset and activate power
 * @see PWM0_SetDuty
 * @brief Initialize PWM
 */
void PWM0_Init(uint32_t timerClkSrc, uint32_t timerClkPrescale,
              uint32_t period, uint32_t duty0, uint32_t duty1);
/**
 * Set duty cycles on PA12 PA13.
 * @param duty0 sets the duty cycle on PA8 
 * @param duty1 sets the duty cycle on PA9 
 * @return none 
 * @note  assumes PWM_Init was called
 * @see PWM_Init
 * @brief Set duty cycles
 */			  
void PWM0_SetDuty(uint32_t duty0, uint32_t duty1);

/**
 * Initialize PWM outputs on PB4 B0.
 * Rising edge synchronized. timerClkDivRatio = 1.
 * Once started, hardware will continuously output the waves.
 * - timerClkSrc = 
 *    - 2 for 32768 Hz LFCLK
 *    - 4 for 4MHz MFCLK (not tested)
 *    - 8 for 80/32/4 BUSCLK
 * - G0/G8  is on Power domain PD0
 *    - 32MHz bus clock, BUSCLK clock is 32MHz
 *    - 40MHz bus clock, BUSCLK clock is ULPCLK 20MHz
 *    - 80MHz bus clock, BUSCLK clock is ULPCLK 40MHz
 * - PWMFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1) * period))
 *    - For example, source=LFCLK, prescale = 0, period = 1000, PWM frequency = 32.768 Hz
 *    - For example, source=BUSCLK, 80MHz bus, prescale=79, period = 10000, PWM frequency = 100Hz 
 *
 * @param timerClkSrc is 2 4 or 8
 * @param timerClkPrescale divide clock by timerClkPrescale+1, 0 to 255
 * @param period sets the PWM period
 * @param duty0 sets the duty cycle on PB4 
 * @param duty1 sets the duty cycle on PB1 
 * @return none 
 * @note  Assumes LaunchPad_Init has been called to reset and activate power
 * @see PWM1_SetDuty
 * @brief Initialize PWM
 */
void PWM1_Init(uint32_t timerClkSrc, uint32_t timerClkPrescale,
              uint32_t period, uint32_t duty0, uint32_t duty1);
/**
 * Set duty cycles on PA12 PA13.
 * @param duty0 sets the duty cycle on PB4 
 * @param duty1 sets the duty cycle on PB1 
 * @return none 
 * @note  assumes PWM_Init was called
 * @see PWM1_Init
 * @brief Set duty cycles
 */			  
void PWM1_SetDuty(uint32_t duty0, uint32_t duty1);

#endif // __PWM_H__
/** @}*/

