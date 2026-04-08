// msp.s
// This file contains I/O port register addresses used in ECE319K
// Jonathan Valvano
// December 18, 2024
// mspm0 I/O GPIO registers
// usage
// .include "../inc/msp.s"
// I/O register is a place in the address map
//    - 32 bit value
//    - software reads/writes to perform, init, input, output

// I/O pin is physical wire/connection/thing we use for input/output
//    - MSPM0 has 60 I/O pins
//    - LaunchPad makes some of the pins available

// I/O port is a collection of pins
//    - MSPM0 has two ports
//    - Port A, PortB

// .equ makes a symbol in the symbol table name,value

// ---------------------------------------------------------------------------------------------
// The first batch of I/O registers are typically used once during initialization

   .equ GPIOA_DOE31_0,  0x400A12C0
   .equ GPIOB_DOE31_0,  0x400A32C0
   // data output enable, set bits to enable output pins
   // bit 31 is PA31/PB31
   // ...
   // bit 1 is PA1/PB1
   // bit 0 is PA0/PB0

   // every pin (60 of them as PINCM (IOMUX) register)
   // bit 18 enables input
   // bit 17 internal passive resistor pullup
   // bit 16 internal passive resistor pulldown
   // bit 7 =1 to enable pin
   // bits 5-0 = 000001 to GPIO general purpose input output
//  I/O register     Address            Mode2     Mode3    Mode4     Mode5      Mode6     Mode7     Mode8     Mode9
   .equ IOMUXPA0,    0x40428004+4*0  // UART0_TX  I2C0_SDA  TIMA0_C0  TIMA_FAL1 TIMG8_C1  FCC_IN
   .equ IOMUXPA1,    0x40428004+4*1  // UART0_RX  I2C0_SCL  TIMA0_C1  TIMA_FAL2 TIMG8_IDX TIMG8_C0
   .equ IOMUXPA2,    0x40428004+4*6  // TIMG8_C1  SPI0_CS0  TIMG7_C1  SPI1_CS0
   .equ IOMUXPA3,    0x40428004+4*7  // TIMG8_C0  SPI0_CS1  UART2_CTS TIMA0_C2  COMP1_OUT TIMG7_C0  TIMA0_C1  I2C1_SDA
   .equ IOMUXPA4,    0x40428004+4*8  // TIMG8_C1  SPI0_POCI UART2_RTS TIMA0_C3  LFCLK_IN  TIMG7_C1  TIMA0_C1N I2C1_SCL
   .equ IOMUXPA5,    0x40428004+4*9  // TIMG8_C0  SPI0_PICO TIMA_FAL1 TIMG0_C0  TIMG6_C0  FCC_IN
   .equ IOMUXPA6,    0x40428004+4*10 // TTIMG8_C1 SPI0_SCK  TIMA_FAL0 TIMG0_C1  HFCLK_IN  TIMG6_C1  TIMA0_C2N
   .equ IOMUXPA7,    0x40428004+4*13 // COMP0_OUT CLK_OUT   TIMG8_C0  TIMA0_C2  TIMG8_IDX TIMG7_C1  TIMA0_C1
   .equ IOMUXPA8,    0x40428004+4*18 // UART1_TX  SPI0_CS0  UART0_RTS TIMA0_C0  TIMA1_C0N
   .equ IOMUXPA9,    0x40428004+4*19 // UART1_RX  SPI0_PICO UART0_CTS TIMA0_C1  RTC_OUT   TIMA0_C0N TIMA1_C1N CLK_OUT
   .equ IOMUXPA10,   0x40428004+4*20 // UART0_TX  SPI0_POCI I2C0_SDA  TIMA1_C0  TIMG12_C0 TIMA0_C2  I2C1_SDA  CLK_OUT
   .equ IOMUXPA11,   0x40428004+4*21 // UART0_RX  SPI0_SCK  I2C0_SCL  TIMA1_C1  COMP0_OUT TIMA0_C2N I2C1_SCL
   .equ IOMUXPA12,   0x40428004+4*33 // UART3_CTS SPI0_SCK  TIMG0_C0  CAN_TX    TIMA0_C3  FCC_IN
   .equ IOMUXPA13,   0x40428004+4*34 // UART3_RTS SPI0_POCI UART3_RX  TIMG0_C1  CAN_RX    TIMA0_C3N
   .equ IOMUXPA14,   0x40428004+4*35 // UART0_CTS SPI0_PICO UART3_TX  TIMG12_C0 CLK_OUT
   .equ IOMUXPA15,   0x40428004+4*36 // UART0_RTS SPI1_CS2  I2C1_SCL  TIMA1_C0  TIMG8_IDX TIMA1_C0N TIMA0_C2
   .equ IOMUXPA16,   0x40428004+4*37 // COMP2_OUT SPI1_POCI I2C1_SDA  TIMA1_C1  TIMA1_C1N TIMA0_C2N FCC_IN
   .equ IOMUXPA17,   0x40428004+4*38 // UART1_TX  SPI1_SCK  I2C1_SCL  TIMA0_C3  TIMG7_C0  TIMA1_C0
   .equ IOMUXPA18,   0x40428004+4*39 // UART1_RX  SPI1_PICO I2C1_SDA  TIMA0_C3N TIMG7_C1  TIMA1_C1
   .equ IOMUXPA19,   0x40428004+4*40 // SWDIO
   .equ IOMUXPA20,   0x40428004+4*41 // SWCLK
   .equ IOMUXPA21,   0x40428004+4*45 // UART2_TX  TIMG8_C0  UART1_CTS TIMA0_C0  TIMG6_C0
   .equ IOMUXPA22,   0x40428004+4*46 // UART2_RX  TIMG8_C1  UART1_RTS TIMA0_C1  CLK_OUT   TIMA0_C0N TIMG6_C1
   .equ IOMUXPA23,   0x40428004+4*52 // UART2_TX  SPI0_CS3  TIMA0_C3  TIMG0_C0  UART3_CTS TIMG7_C0  TIMG8_C0
   .equ IOMUXPA24,   0x40428004+4*53 // UART2_RX  SPI0_CS2  TIMA0_C3N TIMG0_C1  UART3_RTS TIMG7_C1  TIMA1_C1
   .equ IOMUXPA25,   0x40428004+4*54 // UART3_RX  SPI1_CS3  TIMG12_C1 TIMA0_C3  TIMA0_C1N
   .equ IOMUXPA26,   0x40428004+4*58 // UART3_TX  SPI1_CS0  TIMG8_C0  TIMA_FAL0 CAN_TX    TIMG7_C0
   .equ IOMUXPA27,   0x40428004+4*59 // RTC_OUT   SPI1_CS1  TIMG8_C1  TIMA_FAL2 CAN_RX    TIMG7_C1
   .equ IOMUXPA28,   0x40428004+4*2  // UART0_TX  I2C0_SDA  TIMA0_C3  TIMA_FAL0 TIMG7_C0  TIMA1_C0
   .equ IOMUXPA29,   0x40428004+4*3  // I2C1_SCL  UART2_RTS TIMG8_C0  TIMG6_C0
   .equ IOMUXPA30,   0x40428004+4*4  // I2C1_SDA  UART2_CTS TIMG8_C1  TIMG6_C1
   .equ IOMUXPA31,   0x40428004+4*5  // UART0_RX  I2C0_SCL  TIMA0_C3N TIMG12_C1 CLK_OUT   TIMG7_C1  TIMA1_C1
   .equ IOMUXPB0,    0x40428004+4*11 // UART0_TX  SPI1_CS2  TIMA1_C0  TIMA0_C2
   .equ IOMUXPB1,    0x40428004+4*12 // UART0_RX  SPI1_CS3  TIMA1_C1  TIMA0_C2N
   .equ IOMUXPB2,    0x40428004+4*14 // UART3_TX  UART2_CTS I2C1_SCL  TIMA0_C3  UART1_CTS TIMG6_C0  TIMA1_C0
   .equ IOMUXPB3,    0x40428004+4*15 // UART3_RX  UART2_RTS I2C1_SDA  TIMA0_C3N UART1_RTS TIMG6_C1  TIMA1_C1
   .equ IOMUXPB4,    0x40428004+4*16 // UART1_TX  UART3_CTS TIMA1_C0  TIMA0_C2  TIMA1_C0N
   .equ IOMUXPB5,    0x40428004+4*17 // UART1_RX  UART3_RTS TIMA1_C1  TIMA0_C2N TIMA1_C1N
   .equ IOMUXPB6,    0x40428004+4*22 // UART1_TX  SPI1_CS0  SPI0_CS1  TIMG8_C0  UART2_CTS TIMG6_C0  TIMA1_C0N
   .equ IOMUXPB7,    0x40428004+4*23 // UART1_RX  SPI1_POCI SPI0_CS2  TIMG8_C1  UART2_RTS TIMG6_C1  TIMA1_C1N
   .equ IOMUXPB8,    0x40428004+4*24 // UART1_CTS SPI1_PICO TIMA0_C0  COMP1_OUT
   .equ IOMUXPB9,    0x40428004+4*25 // UART1_RTS SPI1_SCK  TIMA0_C1  TIMA0_C0N
   .equ IOMUXPB10,   0x40428004+4*26 // TIMG0_C0  TIMG8_C0  COMP1_OUT TIMG6_C0
   .equ IOMUXPB11,   0x40428004+4*27 // TIMG0_C1  TIMG8_C1  CLK_OUT   TIMG6_C1
   .equ IOMUXPB12,   0x40428004+4*28 // UART3_TX  TIMA0_C2  TIMA_FAL1 TIMA0_C1
   .equ IOMUXPB13,   0x40428004+4*29 // UART3_RX  TIMA0_C3  TIMG12_C0 TIMA0_C1N
   .equ IOMUXPB14,   0x40428004+4*30 // SPI1_CS3  SPI1_POCI SPI0_CS3  TIMG12_C1 TIMG8_IDX TIMA0_C0
   .equ IOMUXPB15,   0x40428004+4*31 // UART2_TX  SPI1_PICO UART3_CTS TIMG8_C0  TIMG7_C0
   .equ IOMUXPB16,   0x40428004+4*32 // UART2_RX  SPI1_SCK  UART3_RTS TIMG8_C1  TIMG7_C1
   .equ IOMUXPB17,   0x40428004+4*42 // UART2_TX  SPI0_PICO SPI1_CS1  TIMA1_C0  TIMA0_C2
   .equ IOMUXPB18,   0x40428004+4*43 // UART2_RX  SPI0_SCK  SPI1_CS2  TIMA1_C1  TIMA0_C2N
   .equ IOMUXPB19,   0x40428004+4*44 // COMP2_OUT SPI0_POCI TIMG8_C1  UART0_CTS TIMG7_C1
   .equ IOMUXPB20,   0x40428004+4*47 // SPI0_CS2  SPI1_CS0  TIMA0_C2  TIMG12_C0 TIMA_FAL1 TIMA0_C1  TIMA1_C1N
   .equ IOMUXPB21,   0x40428004+4*48 // SPI1_POCI TIMG8_C0
   .equ IOMUXPB22,   0x40428004+4*49 // SPI1_PICO TIMG8_C1
   .equ IOMUXPB23,   0x40428004+4*50 // SPI1_SCK  COMP0_OUT TIMA_FAL0
   .equ IOMUXPB24,   0x40428004+4*51 // SPI0_CS3  SPI0_CS1  TIMA0_C3  TIMG12_C1 TIMA0_C1N TIMA1_C0N
   .equ IOMUXPB25,   0x40428004+4*55 // UART0_CTS SPI0_CS0  TIMA_FAL2
   .equ IOMUXPB26,   0x40428004+4*56 // UART0_RTS SPI0_CS1  TIMA0_C3  TIMG6_C0  TIMA1_C0
   .equ IOMUXPB27,   0x40428004+4*57 // COMP2_OUT SPI1_CS1  TIMA0_C3N TIMG6_C1  TIMA1_C1


// ---------------------------------------------------------------------------------------------
// The second batch of I/O registers are typically used over and over to perform actual input/output
   .equ GPIOA_DOUT31_0, 0x400A1280
   .equ GPIOB_DOUT31_0, 0x400A3280
   // write 0/1 to a bit it changes output to 0/1
   // affects all I/O pins in register
   // read returns previous written value
   // logic boolean voltage
   //  0    false   VOL=0V
   //  1    true    VOH=3.3V
   // actual VOL VOH depends on external circuits to which pin is connected

   .equ GPIOA_DIN31_0,  0x400A1380
   .equ GPIOB_DIN31_0,  0x400A3380
   // read only (no writing )
   // returns input values from all input pins
   // logic boolean voltage
   //  0    false   0    to 0.99V
   //  1    true    2.31 to 3.6V


   .equ GPIOA_DOUTSET31_0, 0x400A1290
   .equ GPIOB_DOUTSET31_0, 0x400A3290
   // write only register, reading is not allowed
   // write 1's to set corresponding bits (friendly)
   // writing 0's have no effect

   .equ GPIOA_DOUTCLR31_0, 0x400A12A0
   .equ GPIOB_DOUTCLR31_0, 0x400A32A0
   // write only register, reading is not allowed
   // write 1's to clear corresponding bits (friendly)
   // writing 0's have no effect

   .equ GPIOA_DOUTTGL31_0, 0x400A12B0
   .equ GPIOB_DOUTTGL31_0, 0x400A32B0
   // write only register, reading is not allowed
   // write 1's to toggle corresponding bits (friendly)
   // writing 0's have no effect

// ---------------------------------------------------------------------------------------------
// The third batch of I/O registers allow use of SysTick timer
   .equ SysTick_CTRL,   0xE000E010
   // bit 16 COUNT, set and cleared by hardware
   //    set when VAL counts from 1 to 0,
   //    cleared by write to VAL or cleared when ISR runs
   // bit 2  CLKSRC = 1 to select bus clock
   // bit 1  INTEN  = 1 to arm interrupts, 0 for no interrupts
   // bit 0  ENABLE = 1 to turn on SysTick

   .equ SysTick_LOAD,   0xE000E014
   // 24 bit constant, interrupt period = (LOAD+1)*bus period

   .equ SysTick_VAL,    0xE000E018
   // 24-bit counter: LOAD,LOAD-1,...,2,1,0,LOAD,LOAD-1,...,

   .equ SCB_SHP1,       0xE000ED20
   // bits 31,30 are the two-bit priority for SysTick (0 highest, 3 lowest)



// ---------------------------------------------------------------------------------------------
// The fourth batch of I/O registers should not be used by 319K students
// All 319K starter projects call LaunchPad_Init, which configures these four registers
// 319K students will skip reset and activate because they steps will be done for you
   .equ GPIOA_RSTCTL,   0x400A0804
   .equ GPIOB_RSTCTL,   0x400A2804
// Writing 0xB1000003 to RSTCTL it will reset the port

   .equ GPIOA_PWREN,    0x400A0800
   .equ GPIOB_PWREN,    0x400A2800
// Writing 0x26000001 to PWREN will power it up or activate
// We could power down if we wish to save power (battery)
