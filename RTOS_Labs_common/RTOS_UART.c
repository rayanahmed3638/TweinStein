/* RTOS_UART.c
 * Jonathan Valvano
 * June 11, 2025
 * Derived from uart_rw_multibyte_fifo_poll_LP_MSPM0G3507_nortos_ticlang
 * PA.10 UART0 Tx to XDS Rx
 * PA.11 UART0 Rx from XDS Tx
 * Insert jumper J25: Connects PA10 to XDS_UART
 * Insert jumper J26: Connects PA11 to XDS_UART
 */


#include <ti/devices/msp/msp.h>
#include "../RTOS_Labs_common/RTOS_UART.h"
#include "file.h"
#include <stdio.h>
#include <string.h>

//------------UART_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART_OutString(char *pt){
  while(*pt){
    UART_OutChar(*pt);
    pt++;
  }
}


//------------UART_InUDec------------
// InUDec accepts ASCII input in unsigned decimal format
//     and converts to a 32-bit unsigned number
//     valid range is 0 to 4294967295 (2^32-1)
// Input: none
// Output: 32-bit unsigned number
// If you enter a number above 4294967295, it will return an incorrect value
// Backspace will remove last digit typed
uint32_t UART_InUDec(void){
uint32_t number=0, length=0;
char character;
  character = UART_InChar();
  while(character != CR){ // accepts until <enter> is typed
// The next line checks that the input is a digit, 0-9.
// If the character is not 0-9, it is ignored and not echoed
    if((character>='0') && (character<='9')) {
      number = 10*number+(character-'0');   // this line overflows if above 4294967295
      length++;
      UART_OutChar(character);
    }
// If the input is a backspace, then the return number is
// changed and a backspace is outputted to the screen
    else if((character==BS) && length){
      number /= 10;
      length--;
      UART_OutChar(character);
    }
    character = UART_InChar();
  }
  return number;
}

//-----------------------UART_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART_OutUDec(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART_OutUDec(n/10);
    n = n%10;
  }
  UART_OutChar(n+'0'); /* n is between 0 and 9 */
}

//-----------------------UART_OutUDec3-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Fixed format 3 digits with space after
void UART_OutUDec3(uint32_t n){
  if(n>999){
    UART_OutString("***");
  }else if(n >= 100){
    UART_OutChar(n/100+'0'); 
    n = n%100;
    UART_OutChar(n/10+'0'); 
    n = n%10;
    UART_OutChar(n+'0'); 
  }else if(n >= 10){
    UART_OutChar(' '); 
    UART_OutChar(n/10+'0'); 
    n = n%10;
    UART_OutChar(n+'0'); 
  }else{
    UART_OutChar(' '); 
    UART_OutChar(' '); 
    UART_OutChar(n+'0'); 
  }
  UART_OutChar(' ');
}

//-----------------------UART_OutUDec5-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Fixed format 5 digits with space after
void UART_OutUDec5(uint32_t n){
  if(n>99999){
    UART_OutString("*****");
  }else if(n >= 10000){
    UART_OutChar(n/10000+'0'); 
    n = n%10000;
    UART_OutChar(n/1000+'0'); 
    n = n%1000;
    UART_OutChar(n/100+'0'); 
    n = n%100;
    UART_OutChar(n/10+'0'); 
    n = n%10;
    UART_OutChar(n+'0'); 
  }else if(n >= 1000){
    UART_OutChar(' '); 
    UART_OutChar(n/1000+'0'); 
    n = n%1000;
    UART_OutChar(n/100+'0'); 
    n = n%100;
    UART_OutChar(n/10+'0'); 
    n = n%10;
    UART_OutChar(n+'0'); 
  }else if(n >= 100){
    UART_OutChar(' '); 
    UART_OutChar(' '); 
    UART_OutChar(n/100+'0'); 
    n = n%100;
    UART_OutChar(n/10+'0'); 
    n = n%10;
    UART_OutChar(n+'0'); 
  }else if(n >= 10){
    UART_OutChar(' '); 
    UART_OutChar(' '); 
    UART_OutChar(' '); 
    UART_OutChar(n/10+'0'); 
    n = n%10;
    UART_OutChar(n+'0'); 
  }else{
    UART_OutChar(' '); 
    UART_OutChar(' '); 
    UART_OutChar(' '); 
    UART_OutChar(' '); 
    UART_OutChar(n+'0'); 
  }
  UART_OutChar(' ');
}

//-----------------------UART_OutSDec-----------------------
// Output a 32-bit number in signed decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART_OutSDec(int32_t n){
  if(n<0){
    UART_OutChar('-'); n = -n;
  }
  UART_OutUDec((uint32_t)n);
}
//---------------------UART_InUHex----------------------------------------
// Accepts ASCII input in unsigned hexadecimal (base 16) format
// Input: none
// Output: 32-bit unsigned number
// No '$' or '0x' need be entered, just the 1 to 8 hex digits
// It will convert lower case a-f to uppercase A-F
//     and converts to a 16 bit unsigned number
//     value range is 0 to FFFFFFFF
// If you enter a number above FFFFFFFF, it will return an incorrect value
// Backspace will remove last digit typed
uint32_t UART_InUHex(void){
uint32_t number=0, digit, length=0;
char character;
  character = UART_InChar();
  while(character != CR){
    digit = 0x10; // assume bad
    if((character>='0') && (character<='9')){
      digit = character-'0';
    }
    else if((character>='A') && (character<='F')){
      digit = (character-'A')+0xA;
    }
    else if((character>='a') && (character<='f')){
      digit = (character-'a')+0xA;
    }
// If the character is not 0-9 or A-F, it is ignored and not echoed
    if(digit <= 0xF){
      number = number*0x10+digit;
      length++;
      UART_OutChar(character);
    }
// Backspace outputted and return value changed if a backspace is inputted
    else if((character==BS) && length){
      number /= 0x10;
      length--;
      UART_OutChar(character);
    }
    character = UART_InChar();
  }
  return number;
}

//--------------------------UART_OutUHex----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1 to 8 digits with no space before or after
void UART_OutUHex(uint32_t number){
// This function uses recursion to convert the number of
//   unspecified length as an ASCII string
  if(number >= 0x10){
    UART_OutUHex(number/0x10);
    UART_OutUHex(number%0x10);
  }
  else{
    if(number < 0xA){
      UART_OutChar(number+'0');
     }
    else{
      UART_OutChar((number-0x0A)+'A');
    }
  }
}
void OutHex(uint32_t number){
  if(number < 0xA){
    UART_OutChar(number+'0');
  }
  else{
   UART_OutChar((number-0x0A)+'A');
  }
}
void UART_OutUHex2(uint32_t number){  
  UART_OutString(" 0x");
  OutHex(number/0x10);
  OutHex(number%0x10);
}
//------------UART_InString------------
// Accepts ASCII characters from the serial port
//    and adds them to a string until <enter> is typed
//    or until max length of the string is reached.
// It echoes each character as it is inputted.
// If a backspace is inputted, the string is modified
//    and the backspace is echoed
// terminates the string with a null character
// uses interrupt synchronization on
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
// -- Modified by Agustinus Darmawan + Mingjie Qiu --
void UART_InString(char *bufPt, uint16_t max) {
int length=0;
char character;
  character = UART_InChar();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART_OutChar(BS);
      }
    }
    else if(length < max){
      *bufPt = character;
      bufPt++;
      length++;
      UART_OutChar(character);
    }
    character = UART_InChar();
  }
  *bufPt = 0;
}


/****************Fixed_Fix2Str***************
 converts fixed point number to ASCII string
 format signed 16-bit with resolution 0.01
 range -327.67 to +327.67
 Input: signed 16-bit integer part of fixed point number
         -32768 means invalid fixed-point number
 Output: null-terminated string exactly 8 characters plus null
 Examples
  12345 to " 123.45"  
 -22100 to "-221.00"
   -102 to "  -1.02" 
     31 to "   0.31" 
 -32768 to " ***.**"    
 */ 
void Fixed_Fix2Str(long const num,char *string){
  short n;
  if((num>99999)||(num<-99990)){
    strcpy((char *)string," ***.**");
    return;
  }
  if(num<0){
    n = -num;
    string[0] = '-';
  } else{
    n = num;
    string[0] = ' ';
  }
  if(n>9999){
    string[1] = '0'+n/10000;
    n = n%10000;
    string[2] = '0'+n/1000;
  } else{
    if(n>999){
      if(num<0){
        string[0] = ' ';
        string[1] = '-';
      } else {
        string[1] = ' ';
      }
      string[2] = '0'+n/1000;
    } else{
      if(num<0){
        string[0] = ' ';
        string[1] = ' ';
        string[2] = '-';
      } else {
        string[1] = ' ';
        string[2] = ' ';
      }
    }
  }
  n = n%1000;
  string[3] = '0'+n/100;
  n = n%100;
  string[4] = '.';
  string[5] = '0'+n/10;
  n = n%10;
  string[6] = '0'+n;
  string[7] = 0;
}
//--------------------------UART_Fix2----------------------------
// Output a 32-bit number in 0.01 fixed-point format
// Input: 32-bit number to be transferred -99999 to +99999
// Output: none
// Fixed format 
//  12345 to " 123.45"  
// -22100 to "-221.00"
//   -102 to "  -1.02" 
//     31 to "   0.31" 
// error     " ***.**"   
void UART_Fix2(long number){
  char message[10];
  Fixed_Fix2Str(number,message);
  UART_OutString(message);
}


int uart_open(const char *path, unsigned flags, int llv_fd){
  UART_Init(1);
  return 0;
}
int uart_close( int dev_fd){
    return 0;
}
int uart_read(int dev_fd, char *buf, unsigned count){char ch;
  ch = UART_InChar();    // receive from keyboard
  ch = *buf;         // return by reference
  UART_OutChar(ch);  // echo
  return 1;
}
int uart_write(int dev_fd, const char *buf, unsigned count){ unsigned int num=count;
    while(num){
        UART_OutChar(*buf);
        buf++;
        num--;
    }
    return count;
}
off_t uart_lseek(int dev_fd, off_t ioffset, int origin){
    return 0;
}
int uart_unlink(const char * path){
    return 0;
}
int uart_rename(const char *old_name, const char *new_name){
    return 0;
}

//------------UART_InitPrintf------------
// Initialize the UART for 115,200 baud rate (assuming 32 40 or 80 MHz bus clock),
// 8 bit word length, no parity bits, one stop bit
// Input: none
// Output: none
void UART_InitPrintf(void){int ret_val; FILE *fptr;
  UART_Init(1);
  ret_val = add_device("uart", _SSA, uart_open, uart_close, uart_read, uart_write, uart_lseek, uart_unlink, uart_rename);
  if(ret_val) return; // error
  fptr = fopen("uart","w");
  if(fptr == 0) return; // error
  freopen("uart:", "w", stdout); // redirect stdout to uart
  setvbuf(stdout, NULL, _IONBF, 0); // turn off buffering for stdout

}

