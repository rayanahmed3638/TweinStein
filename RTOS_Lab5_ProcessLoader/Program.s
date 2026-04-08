// Program to be saved to disk
// Position independent data
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
  .equ x,0     // 32-bit variable
  .equ y,x+4   // 32-bit variable
  .equ DataSize,y+4

 .macro LEDToggle  // flip PA0
    LDR  R1,GPIOA_DOUTTGL31_0
    MOVS R0,#0x01
    STR  R0,[R1]
 .endm

   .text
   .thumb
   .align 2
   .global ProgramBlock
ProgramBlock:
   .long Start-ProgramBlock      // offset to start
   .long EndProcess-ProgramBlock // size of code segment
   .long 128                     // size of stack segment
   .long DataSize                // size of data segment
   .string "Blinky"              // program name
   .align 2     
Start:
// Assumes R7 points to data segment in RAM
   MOVS R0,#1
   STR  R0,[R7,#x]
   STR  R0,[R7,#y]
   BL   Init
loop:
   BL   fun1
   LDR  R0,[R7,#y]
   ADDS R0,#1
   STR  R0,[R7,#y]
   LEDToggle
   LDR  R0,n16000000 // bus cycles
   BL   Delay
   B    loop
   .align 2
n16000000:    .long 16000000
   .equ z,0  // local variable
fun1:
   LDR  R0,[R7,#y] // z=y
   PUSH {R0,LR}    // allocate z
l: LDR  R0,[R7,#x]
   ADDS R0,#1
   STR  R0,[R7,#x] // x++
   LDR  R0,[SP,#z]
   SUBS R0,#1
   STR  R0,[SP,#z] // z--
   BNE  l
   POP  {R0,PC}

// Make PA0 an output
Init:
   PUSH {LR}
   LDR  R0,IOMUXPA0
   MOVS R1, #0x81
   STR  R1,[R0] //regular GPIO
   LDR  R0,GPIOA_DOE31_0
   MOVS R4,#1  // PA0
   LDR  R2,[R0]
   ORRS R2,R2,R4
   STR  R2,[R0] //enable output
   POP  {PC}
   .align 2
GPIOA_DOE31_0:  .long 0x400A12C0
GPIOA_DOUT31_0: .long 0x400A1280
GPIOA_DIN31_0:  .long 0x400A1380
GPIOA_DOUTSET31_0: .long 0x400A1290
GPIOA_DOUTCLR31_0: .long 0x400A12A0
GPIOA_DOUTTGL31_0: .long 0x400A12B0
IOMUXPA0:     .long 0x40428004+4*0

// input: R0 bus cycles
// output: none
Delay:
   SUBS R0,R0,#2
dloop:
   SUBS R0,R0,#4 
   NOP  // C=0 on pass through 0
   BHS  dloop
   BX   LR
EndProcess:


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
