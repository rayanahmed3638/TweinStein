/*!
 * @defgroup SPI
 * @brief Synchronous serial communication
 <table>
<caption id="SPIpins5">SPI-LCD pins </caption>
<tr><th>Pin <th>Function  <th>Description
<tr><td>PB9 <td>SPI1 SCLK <tdLCD SPI clock (SPI)   
<tr><td>PB6 <td>GPIO CS   <td>LCD SPI CS     
<tr><td>PB0 <td>GPIO CS   <td>SDC SPI CS     
<tr><td>PB8 <td>SPI1 PICO <td>LCD SPI data (SPI)    
<tr><td>PB7 <td>SPI1 POCI <td>SCD SPI data (SPI)    
<tr><td>PB15<td>GPIO      <td>J2.17 LCD !RST =1 for run, =0 for reset  
<tr><td>PB16<td>GPIO      <td>J4.31 LCD D/C RS =1 for data, =0 for command  
</table>
 * @{*/
/**
 * @file      SPI1.h
 * @brief     Synchronous serial communication
 * @details   SPI uses a chip select, clock, data out and data in. This interface is used for TFT and SDC
 * This interface can be used for MKII LCD and the ST7735R LCD
 * \image html SPIinterface.png  width=500px
 * @version   RTOS v7.0
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      Dec 27, 2025
 <table>
<caption id="SPIpins6">SPI-LCD pins </caption>
<tr><th>Pin <th>Function  <th>Description
<tr><td>PB9 <td>SPI1 SCLK <tdLCD SPI clock (SPI)   
<tr><td>PB6 <td>GPIO CS   <td>LCD SPI CS     
<tr><td>PB0 <td>GPIO CS   <td>SDC SPI CS     
<tr><td>PB8 <td>SPI1 PICO <td>LCD SPI data (SPI)    
<tr><td>PB7 <td>SPI1 POCI <td>SCD SPI data (SPI)    
<tr><td>PB15<td>GPIO      <td>J2.17 LCD !RST =1 for run, =0 for reset  
<tr><td>PB16<td>GPIO      <td>J4.31 LCD D/C RS =1 for data, =0 for command  
</table>
  ******************************************************************************/
#ifndef __SPI_H__
#define __SPI_H__


// PB0 output used for SDC CS
#define SDC_CS           GPIOB
#define SDC_CS_PIN       (1<<0)         // CS controlled by software
#define SDC_CS_INDEX     (PB0INDEX)     // PB0 GPIO
#define SDC_CS_LOW()     (SDC_CS->DOUTCLR31_0 = SDC_CS_PIN)   // PB0 low
#define SDC_CS_HIGH()    (SDC_CS->DOUTSET31_0 = SDC_CS_PIN)   // PB0 high

// PB6 output used for SDC CS
//#define TFT_CS_LOW()     (GPIOB->DOUTCLR31_0 = (1<<6))   // PB6 low
//#define TFT_CS_HIGH()    (GPIOB->DOUTSET31_0 = (1<<6))   // PB6 high
#define TFT_CS           GPIOB
#define TFT_CS_PIN       (1<<6)         // TFT CS controlled by software
#define TFT_CS_INDEX     (PB6INDEX)     // PB6 GPIO
#define TFT_CS_LOW()     (TFT_CS->DOUTCLR31_0 = TFT_CS_PIN)   // PB6 low
#define TFT_CS_HIGH()    (TFT_CS->DOUTSET31_0 = TFT_CS_PIN)   // PB6 high

#define TFT_DC           GPIOB
#define TFT_DC_PIN       (1<<16)         // D/C controlled by software
#define TFT_DC_INDEX     (PB16INDEX)     // PB16 GPIO
#define TFT_DC_LOW()     (TFT_DC->DOUTCLR31_0 = TFT_DC_PIN)   // PB16 low
#define TFT_DC_HIGH()    (TFT_DC->DOUTSET31_0 = TFT_DC_PIN)   // PB16 high

#define TFT_RST          GPIOB
#define TFT_RST_PIN      (1<<15)         // !RST controlled by software
#define TFT_RST_INDEX    (PB15INDEX)     // PB15 GPIO
#define TFT_RST_LOW()    (TFT_RST->DOUTCLR31_0 = TFT_RST_PIN)   // PB15 low
#define TFT_RST_HIGH()   (TFT_RST->DOUTSET31_0 = TFT_RST_PIN)   // PB15 high


/**
 * Output 8-bit data to SPI port.
 * RS=PA13=1 for data.
 * @param data is an 8-bit data to be transferred
 * @return none 
 * @brief Output data
 */
void SPI_OutData(char data);

/**
 * Output 8-bit command to SPI port.
 * RS=PA13=0 for command
 * @param  command is an 8-bit command to be transferred
 * @return none 
 * @brief Output command
 */
void SPI_OutCommand(char command);

/**
 *  Reset LCD 
 * -# drive RST high for 500ms
 * -# drive RST low for 500ms
 * -# drive RST high for 500ms
 * 
 * @param none
 * @return none 
 * @brief Reset LCD
 */
void SPI1_Reset(void);

// SDC CS initialization
void CS_Init(void);

/**
 * Initialize SPI1 for 8 MHz baud clock
 * for both SDC and TFT
 * using busy-wait synchronization.
 * Calls Clock_Freq to get bus clock
 * - PB9 SPI1 SCLK   
 * - PB6 SPI1 CS0       
 * - PB8 SPI1 PICO  
 * - PB15 GPIO !RST =1 for run, =0 for reset    
 * - PA13 GPIO RS =1 for data, =0 for command
 *
 * @note SPI0,SPI1 in power domain PD1 SysClk equals bus CPU clock
 * @param none
 * @return none 
 * @brief initialize SPI1
 */
void SPI1_Init(void);

 //---------TFT_OutCommand------------
 // Output 8-bit command to SPI port
 // Input: data is an 8-bit data to be transferred
 // Output: none
 void TFT_OutCommand(char command);

/**
 * Output 8-bit data to SPI port.
 * RS=PB16=1 for data.
 * @param data is an 8-bit data to be transferred
 * @return none 
 * @brief Output data
 */
void TFT_OutData(char data);

#endif // __SPI_H__
/** @}*/