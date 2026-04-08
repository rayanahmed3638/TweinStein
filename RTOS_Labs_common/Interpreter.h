/**
 * @file      Interpreter.h
 * @brief     Real Time Operating System for Labs 1, 2, 3 and 4
 * @details   ECE445M
 * @version   V1.1
 * @author    Valvano
 * @copyright Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 10, 2025

 ******************************************************************************/

#if LAB6
#include <stdint.h>

// IO redirection for interpreter
struct InterpreterIORedir {
  uint32_t tid;
  void (*InString)(char *, uint16_t);
  void (*OutString)(char *);
  void (*OutChar)(char);
  void (*OutUDec)(uint32_t);
  void (*OutSDec)(long);
  void (*OutUHex)(uint32_t);
  void (*Fix2)(long);
}; 
extern struct InterpreterIORedir *InterpreterIO;
#endif


/**
 * @details  Run the user interface.
 * @param  none
 * @return none
 * @brief  Interpreter
 */
void Interpreter(void);

