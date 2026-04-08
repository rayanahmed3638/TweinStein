/*!
 * @defgroup LD19
 * @brief Asynchronous serial communication to LiDAR Sensor LD19 distance sensor
<table>
<caption id="LD19pins0">LiDAR Sensor LD19 distance sensor interface</caption>
<tr><th>Pin <th>LD19 <th>MSPM0
<tr><td>1   <td>Tx   <td>PB13 RxD:  is UART3 Rx (LD19 to MSPM0)
<tr><td>2   <td>PWM  <td>PB12 GPIO write a 0 to run fastest (MSPM0 to LD19)
<tr><td>3   <td>GND  <td>ground
<tr><td>4   <td>P5V  <td>5V
</table>
 * @{*/
/**
 * @file      LD19.h
 * @brief     Initialize LD19, interrupt synchronization
 * @details   LD19 initialization. UART3 230400 bps baud,
 * 1 start, 8 data bits, 1 stop, no parity.<br>

 * @version   RTOS
 * @author    Jonathan Valvano 
 * @copyright Valvano 2025
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      Nov 5, 2025 
<table>
<caption id="LD19pins1">LiDAR Sensor LD19 distance sensor interface</caption>
<tr><th>Pin <th>LD19 <th>MSPM0
<tr><td>1   <td>Tx   <td>PB13 RxD:  is UART3 Rx (LD19 to MSPM0)
<tr><td>2   <td>PWM  <td>PB12 GPIO write a 0 to run fastest (MSPM0 to LD19)
<tr><td>3   <td>GND  <td>ground
<tr><td>4   <td>P5V  <td>5V
</table>
UART3 is shared between LD19 and TFLuna3 (can have either but not both)

  ******************************************************************************/
#ifndef __LD19_H__
#define __LD19_H__


/**
 * initializeLiDAR Sensor LD19 distance sensor 230400 bps baud rate.<br>
 * interrupt synchronization
 * @param d 0 for debug, pointer to array of 540 elements for read time
 * @return none
 * @brief  Initialize LD19
*/
void LD19_Init(uint16_t *d);


// the following functions only needed for low-level debugging


/**
 * Wait for new serial port input<br>
 * Uses interrupt synchronization<br>
 * This function waits if the receive software FIFO is empty
 * @param Input: none
 * @return 8-bit code from TF Luna
 */
uint8_t LD19_InChar(void);

/**
 * Returns how much data available for reading
 * @param none
 * @return number of elements in receive software FIFO
 */
uint32_t LD19_InStatus(void);


#endif // __LD19_H__

/** @}*/
