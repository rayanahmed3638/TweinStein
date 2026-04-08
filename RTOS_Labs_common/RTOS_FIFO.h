/*!
 * @defgroup FIFO
 * @brief First in first out queue
 * @{*/
/**
 * @file      RTOS_FIFO.h
 * @brief     Provide functions for a first in first out queue
 * @details   Runs on any Microcontroller.
 * Provide functions that initialize a FIFO, put data in, get data out,
 * and return the current size.  The file includes two FIFOs
 * using index implementation.  
 * @version   ECE445M v1.0
 * @author    Daniel Valvano and Jonathan Valvano
 * @copyright Copyright 2023 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      June 20, 2025

 */



#ifndef __FIFO_H__
#define __FIFO_H__


/**
 * \brief TXFIFOSIZE the size of the transmit FIFO, which can hold 0 to TXFIFOSIZE-1 elements.
 * The size must be a power of 2.
 */
 #define TXFIFOSIZE 64 // must be a power of 2


/**
 * Initialize the transmit FIFO 
 * @param none
 * @return none
 * @see TxFifo_Put() TxFifo_Get() TxFifo_Size()
 * @brief  Initialize FIFO
 * @note TXFIFOSIZE the size of the transmit FIFO
 */
void TxFifo_Init(void);


/**
 * Put character into the transmit FIFO 
 * @param data is a new character to save
 * @return 0 for fail because full, 1 for success
 * @see TxFifo_Init() TxFifo_Get() TxFifo_Size()
 * @brief Put FIFO
 * @note TXFIFOSIZE the size of the transmit FIFO
 */
int TxFifo_Put(char data);


/**
 * Get character from the transmit FIFO 
 * @param none
 * @return 0 for fail because empty, nonzero is data
 * @see TxFifo_Init() TxFifo_Put() TxFifo_Size()
 * @brief Get FIFO
 * @note TXFIFOSIZE the size of the transmit FIFO
 */
char TxFifo_Get(void);


/**
 * Determine how many elements are currently stored in the transmit FIFO 
 * @param none
 * @return number of elements in FIFO
 * @see TxFifo_Init() TxFifo_Put() TxFifo_Get()
 * @brief number of elements in FIFO
 * @note Does not change the FIFO
 */
uint32_t TxFifo_Size(void);


/**
 * \brief RXFIFOSIZE the size of the receive FIFO, which can hold 0 to RXFIFOSIZE-1 elements
 * The size must be a power of 2.
 */
#define RXFIFOSIZE 16 // must be a power of 2

/**
 * Initialize the receive FIFO 
 * @param none
 * @return none
 * @see RxFifo_Put() RxFifo_Get() RxFifo_Size()
 * @brief  Initialize FIFO
 * @note RXFIFOSIZE the size of the receive FIFO
 */
void RxFifo_Init(void);

/**
 * Put character into the receive FIFO 
 * @param data is a new character to save
 * @return 0 for fail because full, 1 for success
 * @see RxFifo_Init() RxFifo_Get() RxFifo_Size()
 * @brief Put FIFO
 * @note RXFIFOSIZE the size of the receive FIFO
 */
int RxFifo_Put(char data);

/**
 * Get character from the receive FIFO 
 * @param none
 * @return 0 for fail because empty, nonzero is data
 * @see RxFifo_Init() RxFifo_Put() RxFifo_Size()
 * @brief Get FIFO
 * @note RXFIFOSIZE the size of the receive FIFO
 */
char RxFifo_Get(void);

/**
 * Determine how many elements are currently stored in the receive FIFO 
 * @param none
 * @return number of elements in FIFO
 * @see RxFifo_Init() RxFifo_Put() RxFifo_Get()
 * @brief number of elements in FIFO
 * @note Does not change the FIFO
 */
uint32_t RxFifo_Size(void);



// macro to create an index FIFO
#define AddIndexFifo(NAME,SIZE,TYPE,SUCCESS,FAIL) \
uint32_t volatile NAME ## PutI;         \
uint32_t volatile NAME ## GetI;         \
TYPE static NAME ## Fifo [SIZE];        \
void NAME ## Fifo_Init(void){           \
  NAME ## PutI = NAME ## GetI = 0;      \
}                                       \
int NAME ## Fifo_Put (TYPE data){       \
  if(( NAME ## PutI - NAME ## GetI ) & ~(SIZE-1)){  \
    return(FAIL);                       \
  }                                     \
  NAME ## Fifo[ NAME ## PutI &(SIZE-1)] = data; \
  NAME ## PutI++;                       \
  return(SUCCESS);                      \
}                                       \
int NAME ## Fifo_Get (TYPE *datapt){    \
  if( NAME ## PutI == NAME ## GetI ){   \
    return(FAIL);                       \
  }                                     \
  *datapt = NAME ## Fifo[ NAME ## GetI &(SIZE-1)];  \
  NAME ## GetI++;                       \
  return(SUCCESS);                      \
}                                       \
unsigned short NAME ## Fifo_Size (void){\
 return ((unsigned short)( NAME ## PutI - NAME ## GetI ));  \
}
// e.g.,
// AddIndexFifo(Tx,32,unsigned char, 1,0)
// SIZE must be a power of two
// creates TxFifo_Init() TxFifo_Get() and TxFifo_Put()

#endif //  __FIFO_H__
/** @}*/
