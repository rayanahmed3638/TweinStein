// CAN.h
// Runs on MSPM0G3507
// Use FDCAN0 to communicate on CAN bus PA12 and PA13
// 

// Jonathan Valvano
// December 11, 2025
// derived from can_to_uart_bridge_LP_MSPM0G3507_nortos_ticlang
/* 
 Copyright 2025 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// Use TCAN1057AVDRQ1 (not TCAN1057A-Q1, the version with pin 5 nc)
//  Pin1 TXD  ---- CAN_Tx PA12 FD-CAN module 0 transmit
//  Pin2 Vss  ---- ground
//  Pin3 VCC  ---- +5V with 0.1uF cap to ground
//  Pin4 RXD  ---- CAN_Rx PA13 FD-CAN module 0 receive (0 to 3.3V)
//  Pin5 VIO  ---- +3.3V (digital interface supply)
//  Pin6 CANL ---- to other CANL on network 
//  Pin7 CANH ---- to other CANH on network 
//  Pin8 RS   ---- ground, Slope-Control Input (maximum slew rate)
// 120 ohm across CANH, CANL on both ends of network

#ifndef __CAN_H__
#define __CAN_H__
#include <stdint.h>

typedef uint16_t MessageType_t;
#define CMD_MOTOR 0
#define DATA_STATS 1

typedef struct{
   MessageType_t MessageType;
   uint16_t Field1;
   uint16_t Field2;
   uint16_t Field3;
} CanMessage_t;

void CAN_Init(void);

void CAN_EnableInterrupts(uint32_t priority);

// 0 if failure
// 1 if ok
int CAN_Send(uint32_t id, uint32_t dlc, uint8_t *data);

// Returns 1 if receive data is available
//         0 if no receive data ready
int CAN_CheckMail(void);

// if receive data is ready, gets the data and returns true
// if no receive data is ready, returns false
int CAN_GetMailNonBlock(uint32_t *id, uint32_t *dlc, uint8_t *data);

// if receive data is ready, gets the data 
// if no receive data is ready, it waits until it is ready
void CAN_GetMail(uint32_t *id, uint32_t *dlc, uint8_t *data);

int CAN_SendMessage(uint32_t id, CanMessage_t*);

void CAN_ReadMessage(CanMessage_t* message);

int CAN_SetMotors(uint16_t Duty_L, uint16_t Duty_R, int16_t SteeringAngle);

int CAN_SendOSData(uint16_t jitter);

#endif //  __CAN_H__

