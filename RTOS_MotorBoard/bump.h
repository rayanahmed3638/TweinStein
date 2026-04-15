/*
    Key points:
        -Switches (PA28, PA31, PA27) are all negative logic
        -interuppts triggered on falling edge

*/
#ifndef BUMP_H
#define BUMP_H
 
#include <stdint.h>
typedef enum{
    bump_none = 0x00, // no collisions 
    bump_right= 0x01, // pa31 collision 
    bump_left = 0x02, // pa27 
    bump_center = 0x04, /// pa28
    
} bump_status;
/**
 * @brief Initialize bump switch GPIO pins and interrupts
 * 
 * configures the bump switch pins as inputs with pull-up resistors
 * and enables falling-edge interrupts for collision detection
 */
void bump_init(void);

 /**
* @brief this can be used in polling just to read the switches
* @return the state of the swtiches
 */
bump_status bump_read(void);

/**
*@brief this is mostly for lab 7 where when the
* the switch is still against the wall 
so we disable interuppts for that split second
 */
void bump_disable_interuppts(void);

/**
 * @brief vice verse, enable once we are away from walls
  */
 void bump_enable_interuppts(void);
 
 
/**
* @brief this will cause both motors to turn off for now, we can change in lab 7
 */
void bump_collision(void);

void Set_Servo(int16_t angle);

/**
 * @brief register a callback function for collision events
 * 
 * the callback will be called from the ISR when a collision
 * is detected
 if there is no call back bump_collision is the default
 * 
 * @param callback function pointer to handler 
 * 
 * 
 */
void Bump_RegisterCallback(void (*callback)(bump_status status));
#endif