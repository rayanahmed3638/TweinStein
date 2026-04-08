/*!
 * @defgroup PWMG6
 * @brief Pulse width modulation
 <table>
<caption id="PWMG6pins2">PWM pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>Description
<tr><td>PB6  <td>CCP0 PWM output
</table>
 * @{*/
/**
 * @file      PWMG6.h
 * @brief     Pulse width modulation
 * @details   Hardware creates fixed period, variable duty cycle waves
 * \image html PWM_100Hz.png  width=500px
 * @version   RTOS v1.0
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2026 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      November 24, 2025
 <table>
<caption id="PWMG6pins3">PWM pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>Description
<tr><td>PB6  <td>CCP0 PWM output
</table>
  ******************************************************************************/

/* 
 * Derived from timx_timer_mode_pwm_edge_sleep_LP_MSPM0G3507_nortos_ticlang
 
 */

#ifndef __PWMG6_H__
#define __PWMG6_H__

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
 * Initialize PWMG6 output on PB6.
 * Rising edge synchronized. timerClkDivRatio = 1.
 * Once started, hardware will continuously output the waves.
 * - timerClkSrc = 
 *    - 2 for 32768 Hz LFCLK
 *    - 4 for 4MHz MFCLK (not tested)
 *    - 8 for 80/32/4 BUSCLK
 * - A0/A1  is on Power domain PD1
 *    - 32MHz bus clock, BUSCLK clock is 32MHz
 *    - 40MHz bus clock, BUSCLK clock is ULPCLK 20MHz
 *    - 80MHz bus clock, BUSCLK clock is ULPCLK 40MHz
 * - PWMFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1) * period))
 *    - For example, source=LFCLK, prescale = 0, period = 1000, PWM frequency = 32.768 Hz
 *    - For example, source=BUSCLK, 80MHz bus, prescale=39, period = 40000, PWM frequency = 50Hz  (20ms)
 *
 * @param timerClkSrc is 2 4 or 8
 * @param timerClkPrescale divide clock by timerClkPrescale+1, 0 to 255
 * @param period sets the PWM period
 * @param duty sets the duty cycle on PB6 
 * @return none 
 * @note  Will call LaunchPad_Init to reset and activate power
 * @see PWMG6_SetDuty
 * @brief Initialize PWMG6
 */
void PWMG6_Init(uint32_t timerClkSrc, uint32_t timerClkPrescale,
              uint32_t period, uint32_t duty);
/**
 * Set duty cycles on  PB6.
 * @param duty sets the duty cycle on PB6 (high time)
 * @return none 
 * @note  assumes PWMG6_Init was called
 * @see PWMG6_Init
 * @brief Set duty cycles
 */			  
void PWMG6_SetDuty(uint32_t duty);


#endif // __PWMG6_H__
/** @}*/

