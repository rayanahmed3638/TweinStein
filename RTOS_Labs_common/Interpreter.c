// *************Interpreter.c**************
// Students implement this as part of EE445M Lab 1,2,3,4,5,6
// high level OS user interface
// Solution to labs 1,2,3,4,5,6
// Runs on MSPM0
// Jonathan W. Valvano 12/29/2025, valvano@mail.utexas.edu
#include <stdint.h>
#include <string.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735_SDC.h"
#include "../inc/ADC.h"
#include "../RTOS_Labs_common/RTOS_UART.h"
#include "../RTOS_Labs_common/LPF.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Labs_common/heap.h"
#include "../RTOS_Labs_common/Interpreter.h"

void Lab4(void); // Lab 4 performance data
void DFT(void); // Discrete Fourier Transform
void Robot(void); // Robot thread

extern Sema4_t LCDFree;
unsigned int currentDirectory = 0;

#define MAX_CMD_LEN 16

// readLine reads a line from UART into buf (up to maxLen-1 chars + null terminator)
// Echoes printable characters, handles backspace, terminates on carriage return
static void readLine(char *buf, int maxLen){
  int pos = 0;
  while(1){
    char c = UART_InChar();
    if(c == '\r') break;
    if(c == '\b' || c == 127){       // backspace or DEL
      if(pos > 0){
        pos--;
        buf[pos] = 0;
        UART_OutString("\b \b");
      }
    }else if(c >= ' ' && c <= '~'){  // printable ASCII only
      if(pos < maxLen - 1){
        buf[pos] = c;
        pos++;
        UART_OutChar(c);
      }
    }
  }
  buf[pos] = 0;
  UART_OutString("\n");
}

// findArg returns pointer to first character after the first space in line,
// or 0 if no space found. Null-terminates the command portion.
static char* findArg(char *line){
  for(int i = 0; line[i]; i++){
    if(line[i] == ' '){
      line[i] = 0;
      return &line[i+1];
    }
  }
  return 0;
}

// displayHelp prints all commands to the serial console
// Input: None
// Output: None
static void displayHelp(void){
  UART_OutString("Basic RTOS interpreter commands:\n\n"
                 "format                             Formats the SDC (will lose EVERYTHING)\n\n"
                 "cd [directory ID]                  selects working directory, options are 0 or 1\n\n"
                 "ls                                 prints name and contents of working directory\n\n"
                 "show [filename]                    prints contents of file in working directory\n\n"
                 "delete [filename]                  deletes the file named\n\n"
                 "Lab4                               executes Lab4()\n\n"
                 "robot                              launches the Robot() thread\n\n"
                 "dft                                displays dft data\n\n"
                 "clear                              clears the terminal output and LCD display\n\n"
                 "?                                  shows this help menu\n\n");
}

// Takes user input through serial console
// Loops infinitely, processing commands
void Interpreter(void){
  char line[MAX_CMD_LEN];
  char *arg;
  while(1){
    UART_OutString("\n> ");
    readLine(line, MAX_CMD_LEN);

    if(line[0] == 0) continue;

    arg = findArg(line);

    switch(line[0]){
      case '?':
        displayHelp();
        break;

      case 'f': // format
        UART_OutString("Formatting disk...\n");
        OS_bWait(&LCDFree);
        if(eFile_Format()){
          OS_bSignal(&LCDFree);
          UART_OutString("Format failed\n");
        }else if(eFile_Mount()){
          OS_bSignal(&LCDFree);
          UART_OutString("Format complete, mount failed\n");
        }else{
          OS_bSignal(&LCDFree);
          UART_OutString("Format complete\n");
        }
        break;

      case 'c': // cd or clear
        if(line[1] == 'd'){ // cd
          if(!arg || (arg[0] != '0' && arg[0] != '1') || arg[1] != 0){
            UART_OutString("Usage: cd 0 or cd 1\n");
          }else{
            unsigned char dirID = arg[0] - '0';
            OS_bWait(&LCDFree);
            if(eFile_SelectDirectory(dirID)){
              OS_bSignal(&LCDFree);
              UART_OutString("cd failed\n");
            }else{
              OS_bSignal(&LCDFree);
              currentDirectory = dirID;
            }
          }
        }else if(line[1] == 'l'){ // clear
          UART_OutString("\033[2J\033[H");
          OS_bWait(&LCDFree);
          ST7735_FillScreen(0);
          OS_bSignal(&LCDFree);
        }else{
          UART_OutString("Unknown command. Enter ? for help.\n");
        }
        break;

      case 'l': // ls or Lab4
        if(line[1] == 's'){ // ls
          OS_bWait(&LCDFree);
          if(eFile_DOpen("")){
            OS_bSignal(&LCDFree);
            UART_OutString("ls failed to open directory\n");
          }else{
            char *name;
            unsigned long size;
            unsigned int num = 0;
            unsigned long total = 0;
            UART_OutString("\nContents of Directory ");
            UART_OutUDec(currentDirectory);
            UART_OutString("\n\n");
            while(eFile_DirNext(&name, &size) == 0){
              UART_OutString("Filename = "); UART_OutString(name); UART_OutString("  ");
              UART_OutString("Size (bytes)= "); UART_OutUDec(size); UART_OutString("\n");
              total += size;
              num++;
            }
            UART_OutString("\nNumber of Files = "); UART_OutUDec(num); UART_OutString("\n");
            UART_OutString("Number of Bytes = "); UART_OutUDec(total); UART_OutString("\n");
            eFile_DClose();
            OS_bSignal(&LCDFree);
          }
        }else{
          UART_OutString("Unknown command. Enter ? for help.\n");
        }
        break;

      case 'L': // Lab4
        UART_OutString("Lab4 data\n");
        Lab4();
        break;

      case 's': // show
        if(!arg || arg[0] == 0){
          UART_OutString("Usage: show [filename]\n");
        }else{
          OS_bWait(&LCDFree);
          if(eFile_ROpen(arg)){
            OS_bSignal(&LCDFree);
            UART_OutString("File not found\n");
          }else{
            char data; int status;
            UART_OutString("\n--- Contents of "); UART_OutString(arg); UART_OutString(" ---\n");
            do{
              status = eFile_ReadNext(&data);
              if(status == 0) UART_OutChar(data);
            }while(status == 0);
            eFile_RClose();
            OS_bSignal(&LCDFree);
            UART_OutString("--- End of file ---\n");
          }
        }
        break;

      case 'd': // delete or dft
        if(line[1] == 'e'){ // delete
          if(!arg || arg[0] == 0){
            UART_OutString("Usage: delete [filename]\n");
          }else{
            OS_bWait(&LCDFree);
            if(eFile_Delete(arg)){
              OS_bSignal(&LCDFree);
              UART_OutString("Delete failed\n");
            }else{
              OS_bSignal(&LCDFree);
              UART_OutString("Deleted\n");
            }
          }
        }else if(line[1] == 'f'){ // dft
          UART_OutString("DFT data\n");
          DFT();
        }else{
          UART_OutString("Unknown command. Enter ? for help.\n");
        }
        break;

      case 'r': // robot
        UART_OutString("Launching Robot\n");
        OS_AddThread(&Robot, 128, 1);
        break;

      default:
        UART_OutString("Unknown command. Enter ? for help.\n");
        break;
    }
  }
}
