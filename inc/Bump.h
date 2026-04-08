/*!
 * @defgroup RSLK
 * @brief RSLK Bumper switches
 <table>
<caption id="Bump pins">RSLK Bumper switches</caption>
<tr><th> TM4C <th> MSPM0  <th> Description
<tr><td> PA5  <td> PA27   <td> Left, Bump 0,
<tr><td> PF0  <td> PB15   <td> Center Left, Bump 1,
<tr><td> PB3  <td> PA28   <td> Center Right, Bump 2
<tr><td> PC4  <td> PA31   <td> Right, Bump 3
</table>
 * @{*/
/**
 * @file      Bump.h
 * @brief     RSLK Bumper switches
 * @details   Four negative logic internal pullup<br>

 * @version   RSLK v2.02
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2024 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 28, 2024
 <table>
<caption id="Bump pins2">RSLK Bumper switches</caption>
<tr><th> TM4C <th> MSPM0  <th> Description
<tr><td> PA5  <td> PA27   <td> Left, Bump 0,
<tr><td> PF0  <td> PB15   <td> Center Left, Bump 1,
<tr><td> PB3  <td> PA28   <td> Center Right, Bump 2
<tr><td> PC4  <td> PA31   <td> Right, Bump 3
</table>
  ******************************************************************************/


#ifndef __BUMP_H__
#define __BUMP_H__


/**
 * Initialize RSLK bump switches
 * @param none
 * @return none
 * @brief  Initialize bump switches
 */
void Bump_Init(void);

/**
 * Input RSLK bump switches<br>
 * bit 0 is left<br>
 * bit 1 is left center<br>
 * bit 2 is right center<br>
 * bit 3 is right
 * @param none
 * @return 0 to 15 in positive logic
 * @brief  Read bump switches
 */
uint32_t Bump_In(void);

#endif
