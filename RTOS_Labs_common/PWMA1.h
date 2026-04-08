/*!
 * @defgroup PWMA1
 * @brief Pulse width modulation
 <table>
<caption id="PWMA1pins2">PWM pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>Description
<tr><td>PB4  <td>CCP0 PWM output
<tr><td>PB1  <td>CCP1 PWM output
</table>
 * @{*/
/**
 * @file      PWMA1.h
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
<caption id="PWMA1pins3">PWM pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>Description
<tr><td>PB4  <td>CCP0 PWM output
<tr><td>PB1  <td>CCP1 PWM output
</table>
  ******************************************************************************/

/* 
 * Derived from timx_timer_mode_pwm_edge_sleep_LP_MSPM0G3507_nortos_ticlang
 
 */

#ifndef __PWMA1_H__
#define __PWMA1_H__

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
 * Initialize PWMA1 outputs on PB4, PB1.
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
 *    - For example, source=BUSCLK, 80MHz bus, prescale=39, period = 10000, PWM frequency = 200Hz 
 *
 * @param timerClkSrc is 2 4 or 8
 * @param timerClkPrescale divide clock by timerClkPrescale+1, 0 to 255
 * @param period sets the PWM period
 * @param duty0 sets the duty cycle on PB4 
 * @param duty1 sets the duty cycle on PB1 
 * @return none 
 * @note  Will call LaunchPad_Init to reset and activate power
 * @see PWMA1_SetDuty
 * @brief Initialize PWMA1
 */
void PWMA1_Init(uint32_t timerClkSrc, uint32_t timerClkPrescale,
              uint32_t period, uint32_t duty0, uint32_t duty1);
/**
 * Set duty cycles on  PB4, PB1.
 * @param duty0 sets the duty cycle on PB4 
 * @param duty1 sets the duty cycle on PB1 
 * @return none 
 * @note  assumes PWMA1_Init was called
 * @see PWMA1_Init
 * @brief Set duty cycles
 */			  
void PWMA1_SetDuty(uint32_t duty0, uint32_t duty1);

/**
 * Set duty cycle on PB4 (time low), and set PB1 high.
 * @param duty0 sets the duty cycle on PB4 
 * @return none 
 * @note  assumes PWMA1_Init was called
 * @see PWMA1_Init
 * @brief Motor forward
 */	
void PWMA1_Forward(uint32_t duty0);

/**
 * Set duty cycle on PB1 (time low), and set PB4 high.
 * @param duty1 sets the duty cycle on PB1 
 * @return none 
 * @note  assumes PWMA1_Init was called
 * @see PWMA1_Init
 * @brief Motor backward
 */	
void PWMA1_Backward(uint32_t duty1);

/**
 * Set PB1 high, and set PB4 high.
 * @param none 
 * @return none 
 * @note  assumes PWMA1_Init was called
 * @see PWMA1_Init
 * @brief Motor break
 */	
void PWMA1_Break(void);

/**
 * Set PB1 low, and set PB4 low. Goes into sleep mode
 * @param none 
 * @return none 
 * @note  assumes PWMA1_Init was called
 * @see PWMA1_Init
 * @brief Motor coast 
 */	
void PWMA1_Coast(void); 

#endif // __PWMA1_H__
/** @}*/

