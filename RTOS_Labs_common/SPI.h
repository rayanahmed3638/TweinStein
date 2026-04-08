/*!
 * @defgroup SPI
 * @brief Synchronous serial communication
 <table>
<caption id="AdafruitLCDpins8">Adafruit ST7735R pins </caption>
<tr><th>Pin <th>Connection <th>Description
<tr><td>10<td>+3.3<td>Backlight   
<tr><td>9 <td>PB7 <td>SPI1 MISO (used for SDC)     
<tr><td>8 <td>PB9 <td>SPI1 SCLK clock out
<tr><td>7 <td>PB8 <td>SPI1 PICO data out    
<tr><td>6 <td>PB6 <td>GPIO CS0=TFT_CS   
<tr><td>5 <td>PB0 <td>CARD_CS (used for SDC)   
<tr><td>4 <td>PB16<td>Data/Command(GPIO), high for data, low for command   
<tr><td>3 <td>PB15<td>RESET, low to reset,  (GPIO)   
<tr><td>2 <td>+3.3<td>VCC   
<tr><td>1 <td>Gnd <td>ground   
</table>

<table>
<caption id="HiLetgopins8">HiLetgo ST7735 TFT and SDC pins </caption>
<tr><th>signal<th>Pin<th>Connection
<tr><td>LED-   <td>16<td>TFT, to ground
<tr><td>LED+   <td>15<td>TFT, to +3.3 V
<tr><td>SD_CS  <td>14<td>SDC, to PB0 chip select
<tr><td>MOSI   <td>13<td>SDC, to PB8 MOSI
<tr><td>MISO   <td>12<td>SDC, to PB7 MISO
<tr><td>SCK    <td>11<td>SDC, to PB9 serial clock
<tr><td>CS     <td>10<td>TFT, to PB6  SPI1 CS0
<tr><td>SCL    <td> 9<td>TFT, to PB9  SPI1 SCLK
<tr><td>SDA    <td> 8<td>TFT, to PB8  MOSI SPI1 PICO
<tr><td>A0     <td> 7<td>TFT, to PB16 Data/Command, high for data, low for command
<tr><td>RESET  <td> 6<td>TFT, to PB15 reset (GPIO), low to reset
<tr><td>NC     <td>3,4,5<td>not connected
<tr><td>VCC    <td> 2<td>to +3.3 V
<tr><td>GND    <td> 1<td>to ground
</table>

 * @{*/
/**
 * @file      SPI.h
 * @brief     160 by 128 pixel LCD
 * @details   Software driver functions for ST7735R display<br>
 * This is a library for the Adafruit 1.8" SPI display.<br>
 *  This library works with the Adafruit 1.8" TFT Breakout w/SD card<br>
 *  ----> http://www.adafruit.com/products/358<br>
 *  as well as Adafruit raw 1.8" TFT display<br>
 *  ----> http://www.adafruit.com/products/618<br>
 *   Check out the links above for our tutorials and wiring diagrams<br>
 *  These displays use SPI to communicate, 4 or 5 pins are required to
 *  interface (RST is optional)
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *  MIT license, all text above must be included in any redistribution
 * @version  ECE445M RTOS
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      December 27, 2025
 * interface <br>
 * \image html ST7735interface.png width=500px 
 * <br><br>ST7735 160 by 128 pixel LCD<br>
 * \image html ST7735.png width=500px 
  <table>
<caption id="AdafruitLCDpins2">Adafruit ST7735R pins </caption>
<tr><th>Pin <th>Connection <th>Description
<tr><td>10<td>+3.3<td>Backlight   
<tr><td>9 <td>PB7 <td>SPI1 MISO (used for SDC)     
<tr><td>8 <td>PB9 <td>SPI1 SCLK clock out
<tr><td>7 <td>PB8 <td>SPI1 PICO data out    
<tr><td>6 <td>PB6 <td>GPIO CS0=TFT_CS   
<tr><td>5 <td>PB0 <td>CARD_CS (used for SDC)   
<tr><td>4 <td>PB16<td>Data/Command(GPIO), high for data, low for command   
<tr><td>3 <td>PB15<td>RESET, low to reset,  (GPIO)   
<tr><td>2 <td>+3.3<td>VCC   
<tr><td>1 <td>Gnd <td>ground   
</table>


<table>
<caption id="HiLetgopins2">HiLetgo ST7735 TFT and SDC pins </caption>
<tr><th>signal<th>Pin<th>Connection
<tr><td>LED-   <td>16<td>TFT, to ground
<tr><td>LED+   <td>15<td>TFT, to +3.3 V
<tr><td>SD_CS  <td>14<td>SDC, to PB0 chip select
<tr><td>MOSI   <td>13<td>SDC, to PB8 MOSI
<tr><td>MISO   <td>12<td>SDC, to PB7 MISO
<tr><td>SCK    <td>11<td>SDC, to PB9 serial clock
<tr><td>CS     <td>10<td>TFT, to PB6  SPI1 CS0
<tr><td>SCL    <td> 9<td>TFT, to PB9  SPI1 SCLK
<tr><td>SDA    <td> 8<td>TFT, to PB8  MOSI SPI1 PICO
<tr><td>A0     <td> 7<td>TFT, to PB16 Data/Command, high for data, low for command
<tr><td>RESET  <td> 6<td>TFT, to PB15 reset (GPIO), low to reset
<tr><td>NC     <td>3,4,5<td>not connected
<tr><td>VCC    <td> 2<td>to +3.3 V
<tr><td>GND    <td> 1<td>to ground
</table>

<br>

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

/**
 *  SDC CS initialization 
 * -PB0 GPIO output
 * -drive low to enable SDC
 * -drive high to disable SDC
 * 
 * @param none
 * @return none 
 * @brief SDC CS initialization
 */
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
 * - PB16 GPIO RS =1 for data, =0 for command
 *
 * @note SPI0,SPI1 in power domain PD1 SysClk equals bus CPU clock
 * @param none
 * @return none 
 * @brief initialize SPI1
 */
void SPI1_Init(void);


 /**
 * Output 8-bit command to SPI port.
 * RS=PB16=0 for command.
 * @param data is an 8-bit command to be transferred
 * @return none 
 * @brief Output command
 */
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