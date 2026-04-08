/*!
 * @defgroup TFLuna
 * @brief Asynchronous serial communication to TFLuna2
 <table>
<caption id="TFLuna2pins0">SJ-PM-TF-Luna+A01 interface</caption>
<tr><th>TFLuna <th>Pin  <th>Description
<tr><td>1    <td>Red  5V
<tr><td>2    <td>Serial TxD: PA21/PA23/PB15/PB17 is UART2 Tx (MSPM0 to TFLuna2)
<tr><td>3    <td>Serial RxD: PA22/PA24/PB16/PB18 is UART2 Rx (TFLuna2 to MSPM0) 
<tr><td>4    <td>black ground
</table>
 * @{*/
/**
 * @file      TFLuna2.h
 * @brief     Initialize TFLuna2, interrupt synchronization
 * @details   TFLuna2 initialization. UART2 115200 bps baud,
 * 1 start, 8 data bits, 1 stop, no parity.<br>

 * @version   RTOS
 * @author    Jonathan Valvano 
 * @copyright Valvano 2025
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      Oct 23, 2025 
 <table>
<caption id="TFLuna2pins1">SJ-PM-TF-Luna+A01 interface</caption>
<tr><th>TFLuna <th>Pin  <th>Description
<tr><td>1    <td>Red  5V
<tr><td>2    <td>Serial TxD: PA21/PA23/PB15/PB17 is UART2 Tx (MSPM0 to TFLuna2)
<tr><td>3    <td>Serial RxD: PA22/PA24/PB16/PB18 is UART2 Rx (TFLuna2 to MSPM0) 
<tr><td>4    <td>black ground
</table>
  ******************************************************************************/
#ifndef __TFLuna2_H__
#define __TFLuna2_H__

/**
 * initialize UART2 for 115200 bps baud rate.<br>
 * to use PA9, set jumper J14<br>
 * interrupt synchronization
 * @param function pointer to a callback function
 * @return none
 * @brief  Initialize TFLuna2
*/
void TFLuna2_Init(void (*function)(uint32_t));



/**
 * Configure TFLuna2 for measuring distance in mm<br>
 * @param none
 * @return none
 * @brief mm units
*/
void TFLuna2_Format_Standard_mm(void);  

/**
 * Configure TFLuna2 for measuring distance in cm<br>
 * @param none
 * @return none
 * @brief cm units
*/
void TFLuna2_Format_Standard_cm(void);  

/**
 * Configure TFLuna2 for Pixhawk<br>
 * @param none
 * @return none
 * @brief Pixhawk
 * @warning see datasheet, never tested
*/
void TFLuna2_Format_Pixhawk(void);   
   
/**
 * Configure TFLuna2 sampling rate<br>
 * rate defined in #define TFLunaRate
 * @param none
 * @return none
 * @brief sampling rate
 * @warning only 100 Hz was tested
*/
void TFLuna2_Frame_Rate(void);

/**
 * execute TFLuna2_SaveSettings to activate changes to Format and Frame_Rate
 * @param none
 * @return none
 * @brief save format and rate
*/
void TFLuna2_SaveSettings(void);

/**
 * execute TFLuna2_System_Reset to start measurements
 * @param none
 * @return none
 * @brief start measurements
*/
void TFLuna2_System_Reset(void);


/**
 * Enable TFLuna output<br>
 * @param none
 * @return none
 * @brief enable
 * @warning  I didn't use these because output enabled was default
*/
void TFLuna2_Output_Enable(void);

/**
 * Disable TFLuna output<br>
 * @param none
 * @return none
 * @brief disable
 * @warning  I didn't use these because output enabled was default
*/
void TFLuna2_Output_Disable(void);


// the following functions only needed for low-level debugging

/**
 * Output message to serial port TFLuna2<br>
 * Uses interrupt synchronization<br>
 * msg[0] is 0x5A for command type<br>
 * msg[1] is length=n<br>
 * msg[2] is command<br>
 * msg[3]-msg[n-2] is optional payload<br>
 * msg[n-1] is 8-bit checksum<br> 
 * E.g., 0x5A,0x05,0x05,0x06,0x6A sets format to mm<br>
 * This function waits if the transmit software FIFO is full
 * @param msg pointer to a message to be transferred
 * @return none
 * @brief output message to TFLuna2
 */
void TFLuna2_SendMessage(const uint8_t msg[]);

/**
 * Wait for new serial port input<br>
 * Uses interrupt synchronization<br>
 * This function waits if the receive software FIFO is empty
 * @param Input: none
 * @return 8-bit code from TFLuna2
 */
uint8_t TFLuna2_InChar(void);

/**
 * Returns how much data available for reading
 * @param none
 * @return number of elements in receive software FIFO
 */
uint32_t TFLuna2_InStatus(void);

/**
 * Returns how many bytes are in the transmission software FIFO
 * @param none
 * @return number of elements in transmission software FIFO
 */
 uint32_t TFLuna2_OutStatus(void);

/**
 * Output string to serial port TFLuna2<br>
 * Uses interrupt synchronization<br>
 * This function waits if the transmit software FIFO is full
 * @param pt pointer to a NULL-terminated string to be transferred
 * @return none
 * @brief output string to TFLuna2
 */
void TFLuna2_OutString(uint8_t *pt);


/**
 * Output 8-bit to serial port TFLuna2<br>
 * Uses interrupt synchronization<br>
 * This function waits if the transmit software FIFO is full
 * @param data is an 8-bit ASCII character to be transferred
 * @return none
 * @brief output character to TFLuna2
 */
void TFLuna2_OutChar(uint8_t data);
#endif // __TFLuna2_H__

/** @}*/
