// Second Program to be saved to disk
// Position independent data
// Makes OS calls
// Last Modified: 1/10/2026
// Data segment pointer is R7

// Linkage to OS
   .equ OS_Id,0        // returns R0 as thread ID
   .equ OS_Kill,1      // kill this thread
   .equ OS_Sleep,2     // R0=sleep time in ms
   .equ OS_Time,3      // returns R0 as time in ms
   .equ OS_AddThread,4 // R0=>code, R1=stacksize, R2=priority
   .equ ST7735_Message,5   // R0=device, R1=line, R2=>string, R3=number
   
// Data segment, referenced from R7
  .equ ID,0         // 32-bit variable
  .equ Count,ID+4   // 32-bit variable
  .equ String,Count+4
  .equ DataSize2,String+4

   .text
   .thumb
   .align 2
   .global ProgramBlock2
ProgramBlock2:
   .long Start2-ProgramBlock2      // offset to start
   .long EndProcess2-ProgramBlock2 // size of code segment
   .long 128                     // size of stack segment
   .long DataSize2                // size of data segment
   .string "Prog2"              // program name
   .align 2     
Start2:
// Assumes R7 points to data segment in RAM
   SVC  #OS_Id
   STR  R0,[R7,#ID]
   MOVS R0,#100
   STR  R0,[R7,#Count]
   
   MOVS R0,#0   // device (top)
   MOVS R1,#1   // line
   MOV  R2,PC
h: LDR  R3,str2off
   ADDS R2,R2,R3
   LDR  R3,[R7,#ID]
   SVC  #ST7735_Message
   B    loop2
loop2:
   MOVS R0,#100
   SVC  #OS_Sleep
   LDR  R0,[R7,#Count]
   SUBS R0,#1
   STR  R0,[R7,#Count] // Count--
   BEQ  stop2
   MOVS R0,#0   // device (top)
   MOVS R1,#2   // line
   MOV  R2,PC
j: LDR  R3,str2offb
   ADDS R2,R2,R3
   LDR  R3,[R7,#Count]
   SVC  #ST7735_Message
   B    loop2
stop2:
   SVC  #OS_Kill
   B    stop2 // shouldn't get here
   .align 2
str2off: .long str2-h-2
str2offb: .long str2b-j-2
str2: .string "ID="
str2b: .string "Count="
   .align 2

EndProcess2:


/* unused constants
GPIOB_RSTCTL:   .long 0x400A2804
GPIOB_PWREN:    .long 0x400A2800
GPIOB_DOE31_0:  .long 0x400A32C0
GPIOB_DOUT31_0: .long 0x400A3280
GPIOB_DOUTSET31_0: .long 0x400A3290
GPIOB_DOUTCLR31_0: .long 0x400A32A0
GPIOB_DOUTTGL31_0: .long 0x400A32B0
GPIOB_DIN31_0:  .long 0x400A3380
IOMUXPA18:      .long 0x40428004+4*39
IOMUXPB21:      .long 0x40428004+4*48
IOMUXPB22:      .long 0x40428004+4*49
IOMUXPB26:      .long 0x40428004+4*56
IOMUXPB27:      .long 0x40428004+4*57
n0x00060081:    .long 0x00060081
n0x00050081:    .long 0x00050081
*/
    .end          // end of file
