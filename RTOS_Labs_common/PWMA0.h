/*!
 * @defgroup PWMA0
 * @brief Pulse width modulation
 <table>
<caption id="PWMA0pins2">PWM pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>Description
<tr><td>PB8  <td>CCP0 PWM output
<tr><td>PB9  <td>CCP1 PWM output
</table>
 * @{*/
/**
 * @file      PWMA0.h
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
<caption id="PWMA0pins3">PWM pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>Description
<tr><td>PB8  <td>CCP0 PWM output
<tr><td>PB9  <td>CCP1 PWM output
</table>
  ******************************************************************************/

/* 
 * Derived from timx_timer_mode_pwm_edge_sleep_LP_MSPM0G3507_nortos_ticlang
 
 */

#ifndef __PWMA0_H__
#define __PWMA0_H__

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
 * Initialize PWMA0 outputs on PB8, PB9.
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
 *    - For example, source=BUSCLK, bus=40MHz, prescale=19, period = 10000, PWM frequency = 10kHz 
 *
 * @param timerClkSrc is 2 4 or 8
 * @param timerClkPrescale divide clock by timerClkPrescale+1, 0 to 255
 * @param period sets the PWMA0 period
 * @param duty0 sets the duty cycle on PB8 
 * @param duty1 sets the duty cycle on PB9 
 * @return none 
 * @note  Will call LaunchPad_Init to reset and activate power
 * @see PWMA0_SetDuty
 * @brief Initialize PWMA0
 */
void PWMA0_Init(uint32_t timerClkSrc, uint32_t timerClkPrescale,
              uint32_t period, uint32_t duty0, uint32_t duty1);
/**
 * Set duty cycles on  PB8, PB9.
 * @param duty0 sets the duty cycle on PB8
 * @param duty1 sets the duty cycle on PB9 
 * @return none 
 * @note  assumes PWMA0_Init was called
 * @see PWM_Init
 * @brief Set duty cycles
 */			  
void PWMA0_SetDuty(uint32_t duty0, uint32_t duty1);

/**
 * Set duty cycle on PB8, and set PB9 high.
 * @param duty0 sets the duty cycle on PB8 
 * @return none 
 * @note  assumes PWMA0_Init was called
 * @see PWMA0_Init
 * @brief Motor forward
 */	
void PWMA0_Forward(uint32_t duty0);

/**
 * Set duty cycle on PB9, and set PB8 high.
 * @param duty1 sets the duty cycle on PB9 
 * @return none 
 * @note  assumes PWMA0_Init was called
 * @see PWMA0_Init
 * @brief Motor backward
 */	
void PWMA0_Backward(uint32_t duty1);

/**
 * Set PB8 high, and set PB9 high.
 * @param none 
 * @return none 
 * @note  assumes PWMA0_Init was called
 * @see PWMA0_Init
 * @brief Motor break
 */	
void PWMA0_Break(void);

/**
 * Set PB8 low, and set PB9 low. Goes into sleep mode
 * @param none 
 * @return none 
 * @note  assumes PWMA0_Init was called
 * @see PWMA0_Init
 * @brief Motor coast 
 */	
void PWMA0_Coast(void); 

#endif // __PWMA0_H__
/** @}*/

