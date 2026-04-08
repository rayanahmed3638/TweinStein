// Position independent data
// Last Modified: 1/5/2026
// Data segment pointer is R7
// This program will build/debug/run without OS

// Globals referenced from R7
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
   .string "Fuzzy"               // program name
   .align 2     
Start:
// Assumes R7 points to data segment in RAM
    MOVS R0,#0
   STR  R0,[R7,#x]
   STR  R0,[R7,#y]
   BL   Init
loop:
   BL   fun1
   LDR  R0,[R7,#y]
   ADDS R0,#1
   STR  R0,[R7,#y]
   LEDToggle
   LDR  R0,n16000000 // 500ms
   BL   Delay
   B    loop
   .align 2
n16000000:    .long 16000000
   .type fun1, %function
fun1:
   LDR  R0,[R7,#x]
   ADDS R0,#1
   STR  R0,[R7,#x]
   BX   LR

// PA0 is output
   .type Init, %function
Init:
   PUSH {LR}
   LDR  R0,ResetValue
   LDR  R1,GPIOA_RSTCTL 
   STR  R0,[R1] // 0xB1000003
   LDR  R0,PowerValue
   LDR  R1,GPIOA_PWREN 
   STR  R0,[R1]  // =0x26000001
   MOVS R0,#24
   BL   Delay // time  to power up
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
GPIOA_RSTCTL:   .long 0x400A0804
GPIOA_PWREN:    .long 0x400A0800
GPIOA_DOE31_0:  .long 0x400A12C0
GPIOA_DOUT31_0: .long 0x400A1280
GPIOA_DIN31_0:  .long 0x400A1380
GPIOA_DOUTSET31_0: .long 0x400A1290
GPIOA_DOUTCLR31_0: .long 0x400A12A0
GPIOA_DOUTTGL31_0: .long 0x400A12B0
PowerValue:   .long 0x26000001
ResetValue:   .long 0xB1000003
IOMUXPA0:     .long 0x40428004+4*0

// input: R0 bus cycles
// output: none
   .type Delay, %function
Delay:
   SUBS R0,R0,#2
dloop:
   SUBS R0,R0,#4 
   NOP  // C=0 on pass through 0
   BHS  dloop
   BX   LR
EndProcess:

   .global main
main:
   LDR R7,n0x20200000 // sets data segment pointer
   LDR R1,=ProgramBlock
   LDR R2,[R1] // offset to code
   MOVS R3,#1
   ORRS R1,R3
   ADDS R1,R2
   BLX  R1
here:
   B here
   .align 2

n0x20200000:    .long 0x20200000
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
