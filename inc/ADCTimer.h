/*!
 * @defgroup ADC
 * @brief Analog to digital conversion
  <table>
<caption id="ADCpinsT">ADC pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>ADC channel<th>Sensor
<tr><td>PA27 <td>ADC0 channel 0 <td>J1.8 also MKII light interrupt
<tr><td>PA26 <td>ADC0 channel 1 <td>J1.6 MKII microphone in
<tr><td>PA25 <td>ADC0 channel 2 <td>J1.2 MKII Joystick X
<tr><td>PA24 <td>ADC0 channel 3 <td>J3.27 ***free***
<tr><td>PB25 <td>ADC0 channel 4 <td>J19.7 (insert 0ohm R74, no U3 OPA2365)
<tr><td>PB24 <td>ADC0 channel 5 <td>J1.5 also MKII joystick select button
<tr><td>PB20 <td>ADC0 channel 6 <td>J4.36 ***free***
<tr><td>PA22 <td>ADC0 channel 7 <td>J24 MKII Accelerometer Y
</table>
 * @{*/
/**
 * @file      ADCTimer.h
 * @brief     Initialize 12-bit ADC0 
 * @details   ADC input, timer trigger, 12-bit conversion<br>
 * The ADC allows two possible references: 2.5V or 3.3V.<br>
 * The internal 2.5V reference is lower noise, but limits the range to 0 to 2.5V.<br>
 * The other possibility is to use the analog voltage supply at 3.3V,
 * making the ADC range 0 to 3.3V. In this driver, the 3.3V range is selected.
 * There are several configurations (each with initialization and a
 * read ADC with software trigger, busy-wait function)<br>

 * @version   ECE319K v1.3
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      May 23, 2025
 <table>
<caption id="ADCpins2T">ADC pins on the MSPM0G3507</caption>
<tr><th>Pin  <th>ADC channel<th>Sensor
<tr><td>PA27 <td>ADC0 channel 0 <td>J1.8 also MKII light interrupt
<tr><td>PA26 <td>ADC0 channel 1 <td>J1.6 MKII microphone in
<tr><td>PA25 <td>ADC0 channel 2 <td>J1.2 MKII Joystick X
<tr><td>PA24 <td>ADC0 channel 3 <td>J3.27 ***free***
<tr><td>PB25 <td>ADC0 channel 4 <td>J19.7 (insert 0ohm R74, no U3 OPA2365)
<tr><td>PB24 <td>ADC0 channel 5 <td>J1.5 also MKII joystick select button
<tr><td>PB20 <td>ADC0 channel 6 <td>J4.36 ***free***
<tr><td>PA22 <td>ADC0 channel 7 <td>J24 MKII Accelerometer Y
<tr><td>PA15 <td>ADC1 channel 0 <td>J3.30 (also DACout)
<tr><td>PA16 <td>ADC1 channel 1 <td>J3.29 ***free***
<tr><td>PA17 <td>ADC1 channel 2 <td>J3.28 ***free***
<tr><td>PA18 <td>ADC1 channel 3 <td>J3.26 MKII Joystick Y
<tr><td>PB17 <td>ADC1 channel 4 <td>J2.18 ***free***
<tr><td>PB18 <td>ADC1 channel 5 <td>J3.25 MKII Accelerometer Z
<tr><td>PB19 <td>ADC1 channel 6 <td>J3.23 MKII Accelerometer X
<tr><td>PA21 <td>ADC1 channel 7 <td>J17.8 (insert R20, remove R3)
</table>
  ****note to students****<br>
  the data sheet says the ADC does not work when clock is 80 MHz
  however, the ADC seems to work on my boards at 80 MHz
  I suggest you try 80MHz, but if it doesn't work, switch to 40MHz
  ******************************************************************************/

#ifndef __ADCTimer_H__
#define __ADCTimer_H__
#include <ti/devices/msp/msp.h>

/**
 * \brief using ADCVREF_INT means choose internal 2.5V reference for accuracy
 */
#define ADCVREF_INT  0x200
/**
 * \brief using ADCVREF_EXT means choose external reference not tested
 */ 
#define ADCVREF_EXT  0x100
/**
 * \brief using ADCVREF_VDDA means choose power line 3.3V reference for 0 to 3.3V range. This is the mode we use in ECE319K
 */
#define ADCVREF_VDDA 0x000
/**
 * Initialize ADC0 for Timer G0 triggered sampling<br>
 * Assuming 80 MHz bus, the sampling is 40MHz/period/prescale<br>
 *  Pin  channel<br>
 *  PA27 0 <br>
 *  PA26 1 <br>
 *  PA25 2 <br>
 *  PA24 3 <br>
 *  PB25 4 <br>
 *  PB24 5 <br>
 *  PB20 6 <br>
 *  PA22 7 
 * @param channel is the 0 to 7
 * @param reference is ADCVREF_INT, ADCVREF_EXT, ADCVREF_VDDA
 * @param period is 16-bit period in bus cycles on Timer G0
 * @param prescale is 8-bit prescale in bus cycles on Timer G0
 * @param priority is 0 to 3 for ADC interrupts
 * @return none
 * @note uses internal pub/sub channel 1
 * @brief  Initialize 12-bit ADC0
 */
void ADC0_TimerG0_Init(uint32_t channel, uint32_t reference,uint16_t period, uint32_t prescale, uint32_t priority);

/**
 * Initialize ADC1 for Timer G8 triggered sampling<br>
 * Assuming 80 MHz bus, the sampling is 40MHz/period/prescale<br>
 *  Pin  channel<br>
 *  PA15 0 <br>
 *  PA16 1 <br>
 *  PA17 2 <br>
 *  PA18 3 <br>
 *  PB17 4 <br>
 *  PB18 5 <br>
 *  PB19 6 <br>
 *  PA21 7 
 * @param channel is the 0 to 7
 * @param reference is ADCVREF_INT, ADCVREF_EXT, ADCVREF_VDDA
 * @param period is 16-bit period in bus cycles on Timer G8
 * @param prescale is 8-bit prescale in bus cycles on Timer G8
 * @param priority is 0 to 3 for ADC interrupts
 * @return none
 * @note uses internal pub/sub channel 2
 * @brief  Initialize 12-bit ADC1
 */
void ADC1_TimerG8_Init(uint32_t channel, uint32_t reference,uint16_t period, uint32_t prescale, uint32_t priority);


#endif //  __ADCTimer_H__
/** @}*/
