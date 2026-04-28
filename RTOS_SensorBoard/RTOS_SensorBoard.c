/* RTOS_Lab4.c
 * Jonathan Valvano
 * December 30, 2025
 * Remove 3.3V J101 jumper to run RTOS sensor board or motor board
 * A two-pin female header is required on the LaunchPad TP10(XDS_VCC) and TP9(!RSTN)
 */


// Overtake Parameters
#define FRONT_DETECT    1200
#define D2_MIN_PASS     250
#define LD2_MIN_PASS    250
#define V_MAX_MMPS      1140 // TBD via calibration
#define V_ALPHA_Q8      77
#define R_ACC_ALPHA_Q8  205
#define R_THRESH_BASE   150
#define R_YAW_GAIN_Q8   77
#define R_CAP           2000
#define OFFSET_CMD      180
#define OFFSET_RATE     20
#define GZ_RAMP_GATE    500
#define PASS_CLEAR      900
#define T_CLEAR_MS      500
#define T_PASS_MIN      400
#define T_SETTLE        300
#define ABORT_SIDE      150
#define ABORT_GZ        1200
#define ABORT_DF        300 // guessed value

#define CALIB_THROTTLE 9500

// PD weights
#define kp_d 1
#define kd_d 2
#define kp_a 5
#define kd_a 2

#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../inc/Clock.h"
#include "../RTOS_Labs_common/ST7735_SDC.h"
#include "../RTOS_Labs_common/RTOS_UART.h"
#include "../RTOS_Labs_common/Interpreter.h"
#include "../RTOS_Labs_common/IRDistance.h"
#include "../RTOS_Labs_common/LPF.h"
#include "../RTOS_Labs_common/DFT16.h"
#include "../RTOS_Labs_common/TFLuna1.h"
#include "../RTOS_Labs_common/TFLuna2.h"
#include "../RTOS_Labs_common/TFLuna3.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Labs_common/CAN.h"
#include "Model.h"
#include "IMU.h"
#include <stdio.h>
#include <stdlib.h>


#define USE_MEDIAN_FILTER 0  // 1 = median (Median5 per sample, 5-sample pace), 0 = mean (8-sample average)

#define angle_ref 5 // degrees "0"

int32_t dist_ref_cur = 0; // mm

// PA10 is UART0 Tx    index 20 in IOMUX PINCM table
// PA11 is UART0 Rx    index 21 in IOMUX PINCM table
// Insert jumper J25: Connects PA10 to XDS_UART
// Insert jumper J26: Connects PA11 to XDS_UART
//  PA0 is red LED1,   index 0 in IOMUX PINCM table, negative logic
// PB22 is BLUE LED2,  index 49 in IOMUX PINCM table
// PB26 is RED LED2,   index 56 in IOMUX PINCM table
// PB27 is GREEN LED2, index 57 in IOMUX PINCM table
// PA18 is S1 positive logic switch,  conflict with TFLuna1, so S1 will not be used
// PB21 is S2 negative logic switch,  used for aperiodic task
// IR analog distance sensors
//   30 cm GP2Y0A41SK0F or 80 cm long range GP2Y0A21YK0F 
//   PA26 Right  ADC0_1
//   PA24 Center ADC0_3, used in Labs 1,2,3,4
//   PA22 Left   ADC0_7
//   PA27 Extra  ADC0_0

// RTOS sensor board supported three TF-Luna sensors
//    Serial TxD: PA17 is UART1 Tx (MSPM0 to TFLuna1)
//    Serial RxD: PA18 is UART1 Rx (TFLuna1 to MSPM0), conflict with LaunchPad S1
//    Serial TxD: PB17 is UART2 Tx (MSPM0 to TFLuna2), used in Labs 1,2,3,4
//    Serial RxD: PB18 is UART2 Rx (TFLuna2 to MSPM0), used in Labs 1,2,3,4
//    Serial TxD: PB12 is UART3 Tx (MSPM0 to TFLuna3), 
//    Serial RxD: PB13 is UART3 Rx (TFLuna3 to MSPM0), shared with LD19 Lidar 
//UART3 is shared between LD19 and TFLuna3 (can have either but not both)

// **** OS must run disk_timerproc();  at 1000Hz, every 1ms *****
long StartCritical(void);
void EndCritical(long sr);

uint32_t Running;           // true while robot is running
uint32_t NumCreated;   // number of foreground threads created

//---------------------User debugging-----------------------

// Unused sensor board pins, made outputs for debugging
// Jumper J14 select PB23
// Jumper J15 select PA16
void Logic_Init(void){
  IOMUX->SECCFG.PINCM[PA8INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PB23INDEX] = (uint32_t) 0x00000081; //****CHANGE from PA9****
  IOMUX->SECCFG.PINCM[PA16INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PB4INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PB1INDEX] = (uint32_t) 0x00000081;
  IOMUX->SECCFG.PINCM[PB20INDEX] = (uint32_t) 0x00000081;
  GPIOA->DOE31_0 |= (1<<8)|(1<<16);  //****CHANGE removing PA9****
  GPIOB->DOE31_0 |= (1<<4)|(1<<1)|(1<<20)|(1<<23);//****CHANGE adding PB23****
}
#define TogglePA8() (GPIOA->DOUTTGL31_0 = (1<<8))
#define SetPA8() (GPIOA->DOUTSET31_0 = (1<<8))
#define ClrPA8() (GPIOA->DOUTCLR31_0 = (1<<8))
#define TogglePB23() (GPIOB->DOUTTGL31_0 = (1<<23))
#define SetPB23() (GPIOB->DOUTSET31_0 = (1<<23))
#define ClrPB23() (GPIOB->DOUTCLR31_0 = (1<<23))
#define TogglePA16() (GPIOA->DOUTTGL31_0 = (1<<16))
#define TogglePB4() (GPIOB->DOUTTGL31_0 = (1<<4))
#define SetPB4() (GPIOB->DOUTSET31_0 = (1<<4))
#define ClrPB4() (GPIOB->DOUTCLR31_0 = (1<<4))
#define TogglePB1() (GPIOB->DOUTTGL31_0 = (1<<1))
#define TogglePB20() (GPIOB->DOUTTGL31_0 = (1<<20))

//--------------Arctan Lookup Table-----------------------
// TanTable[i] = tan(i degrees) * 1000, for i = 0 to 89
const uint32_t TanTable[90] = {
       0,   17,   35,   52,   70,   87,  105,  123,  141,  158,  // 0-9
     176,  194,  213,  231,  249,  268,  287,  306,  325,  344,  // 10-19
     364,  384,  404,  424,  445,  466,  488,  510,  532,  554,  // 20-29
     577,  601,  625,  649,  675,  700,  727,  754,  781,  810,  // 30-39
     839,  869,  900,  933,  966, 1000, 1036, 1072, 1111, 1150,  // 40-49
    1192, 1235, 1280, 1327, 1376, 1428, 1483, 1540, 1600, 1664,  // 50-59
    1732, 1804, 1881, 1963, 2050, 2145, 2246, 2356, 2475, 2605,  // 60-69
    2747, 2904, 3078, 3271, 3487, 3732, 4011, 4331, 4705, 5145,  // 70-79
    5671, 6314, 7115, 8144, 9514,11430,14300,19081,28636,57290   // 80-89
};
// arctan: inverse tangent using table lookup and binary search
// Input:  ratio * 1000 (e.g., 1000 for ratio=1.0, -500 for ratio=-0.5)
// Output: angle in degrees (-89 to 89)
// Usage:  angle = arctan((opposite * 1000) / adjacent);
int32_t arctan(int32_t ratio_x1000) {
    uint32_t abs_ratio = (ratio_x1000 < 0) ? (uint32_t)(-ratio_x1000) : (uint32_t)ratio_x1000;
    uint32_t lo = 0, hi = 88;
    if (abs_ratio >= TanTable[89]) {
        return (ratio_x1000 < 0) ? -89 : 89;
    }
    while (lo + 1 < hi) {
        uint32_t mid = (lo + hi) / 2;
        if (TanTable[mid] <= abs_ratio) lo = mid;
        else hi = mid;
    }
    return (ratio_x1000 < 0) ? -(int32_t)lo : (int32_t)lo;
}
// tangent: tangent of angle using TanTable (same table as arctan)
// Input:  angle in degrees
// Output: tan(angle) * 1000; returns 57290 (max) for angles near 90/270 (undefined)
// tan has period 180: sign positive in [0,89] and [180,269], negative in [91,179] and [271,359]
int32_t tangent(int32_t angle_deg) {
    while (angle_deg < 0)    angle_deg += 360;
    while (angle_deg >= 360) angle_deg -= 360;
    uint32_t mod = (uint32_t)angle_deg % 180;
    if (mod <= 89)  return  (int32_t)TanTable[mod];
    if (mod == 90)  return  57290; // clamp — approaches +∞ before, -∞ after
    return -(int32_t)TanTable[180 - mod]; // mod 91–179: negative, use symmetry
}
//-----------end of Arctan Lookup Table--------------------

//--------------Cosine Lookup Table-----------------------
// CosTable[i] = cos(i degrees) * 1000, for i = 0 to 90
const uint32_t CosTable[91] = {
    1000,  999,  999,  998,  997,  996,  994,  992,  990,  987,  // 0-9
     984,  981,  978,  974,  970,  965,  961,  956,  951,  945,  // 10-19
     939,  933,  927,  920,  913,  906,  898,  891,  882,  874,  // 20-29
     866,  857,  848,  838,  829,  819,  809,  798,  788,  777,  // 30-39
     766,  754,  743,  731,  719,  707,  694,  681,  669,  656,  // 40-49
     642,  629,  615,  601,  587,  573,  559,  544,  529,  515,  // 50-59
     500,  484,  469,  453,  438,  422,  406,  390,  374,  358,  // 60-69
     342,  325,  309,  292,  276,  258,  242,  224,  208,  190,  // 70-79
     173,  156,  139,  121,  104,   87,   69,   52,   35,   17,  // 80-89
       0                                                          // 90
};
// cosine: cosine of angle using table lookup
// Input:  angle in degrees (-180 to 180)
// Output: cos(angle) * 1000 (e.g., 1000 for 0 deg, 0 for 90 deg, -1000 for 180 deg)
int32_t cosine(int32_t angle_deg) {
    // Normalize to [0, 360)
    while (angle_deg < 0)   angle_deg += 360;
    while (angle_deg >= 360) angle_deg -= 360;
    // Use symmetry: cos is even and periodic
    if (angle_deg <= 90)  return  (int32_t)CosTable[angle_deg];
    if (angle_deg <= 180) return -(int32_t)CosTable[180 - angle_deg];
    if (angle_deg <= 270) return -(int32_t)CosTable[angle_deg - 180];
    return                        (int32_t)CosTable[360 - angle_deg];
}
//-----------end of Cosine Lookup Table--------------------

uint32_t Checks; // number of times virus checking has run
uint32_t ChecksWork; // number of checks in 10 second
//------------------Task 1--------------------------------
// real-time sampling ADC0 channel 3, using software start trigger
// 60-Hz notch high-Q, IIR filter, assuming fs=1000 Hz
// y(n) = (256x(n) -476x(n-1) + 256x(n-2) + 471y(n-1)-251y(n-2))/256 (1k sampling)
#define PERIOD TIME_1MS      // DAS 1kHz sampling period in system time units
#define FS 1000              // DAS sampling
#define RUNLENGTH (10000)     // display results and quit when FilterWork==RUNLENGTH

uint32_t FilterOutput, Distance, DistanceRaw;
uint32_t L_FilterOutput, L_Distance, L_DistanceRaw;
uint32_t L_Distance2;
uint32_t FrontDist = 1000; // mm, from TFLuna1 (front-facing); default far
Sema4_t LCDFree;  // SDC and LCD sharing

uint32_t FilterWork;
uint32_t MaxJitter3;  
#define JITTERSIZE3 512
uint32_t const JitterSize3=JITTERSIZE3;
uint32_t JitterHistogram3[JITTERSIZE3]={0,};
void Jitter3_Init(void){
  for(int i=0;i<JitterSize3;i++){
    JitterHistogram3[i] = 0;
  }
  MaxJitter3 = 0;
}
//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 1000 times/sec
// samples PA24 Center ADC0_3, calculates Distance
// inputs:  none
// outputs: none
void DAS(void){
  uint32_t input, L_input;
  static uint32_t LastTime;      // time at previous ADC sample, 12.5 ns
  uint32_t thisTime;             // time at current ADC sample, 12.5 ns
  uint32_t jitter;               // time between measured and expected, 12.5 ns
  TogglePA8();                   // toggle PA8
  ADC_InDual(ADC0, &input, &L_input); // ch1=right IR (PA26), ch7=left IR (PA22)
  TogglePA8();                   // toggle PA8
  thisTime = OS_Time();          // current time, 12.5 ns
  FilterOutput = Filter(input);
  DistanceRaw = FilterOutput;
  Distance = IRDistance_Right(FilterOutput);   // in mm
  L_FilterOutput = Filter2(L_input);
  L_DistanceRaw = L_FilterOutput;
  L_Distance = IRDistance_Left(L_FilterOutput); // in mm
  if(Running){    // finite time run
    FilterWork++;        // calculation finished
    if(FilterWork>2){    // ignore timing of first interrupt
      uint32_t diff = OS_TimeDifference(LastTime,thisTime);
      if(diff>PERIOD){
        jitter = (diff-PERIOD);  // in 12.5 ns
      }else{
        jitter = (PERIOD-diff);  // in 12.5 ns
      }
      if(jitter > MaxJitter3){
        MaxJitter3 = jitter; // in 12.5 ns
      }       // jitter should be 0 
      if (jitter > JITTERSIZE3-1) jitter = JITTERSIZE3-1;   // clamp
      JitterHistogram3[jitter]++; 
    }
    ChecksWork = Checks;
    LastTime = thisTime;
  }
  TogglePA8();    // toggle PA8
}
//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with PA28 button
// PA28 negative logic switch 
// one foreground task created with each button push
// foreground tread outputs a message and dies
uint32_t DataLost;     // data sent by Producer, but not received by Consumer

// ***********PA28Push*************
int ArmCrash=1;
void HandleCrash(void){
  TogglePB23();
  TogglePB23();
  uint32_t myId = OS_Id(); 
  ST7735_Message(1,0,"myID        =",myId); 
  ST7735_Message(1,1,"*Crash*,  t= ",OS_MsTime());
  ArmCrash=1;
  TogglePB23();
  OS_Kill();
} 
void PA28Push(void){ // real time task
  if(ArmCrash){
    ArmCrash = 0; // debounce
    NumCreated += OS_AddThread(&HandleCrash,128,1);  // test robot crash
  }
} 

//------------------Task 3--------------------------------
// hardware-triggered TFLuna distance sampling at 100Hz
// Producer runs as part of UART2 ISR
// Producer uses fifo to transmit 100 distance samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement
uint32_t DataLost;        // data sent by Producer, but not received by Consumer
uint32_t Distance2;       // mm
int32_t x[16],ReX[16],ImX[16];           // input and output arrays for FFT

Sema4_t TFLuna3Ready;  // signaled by Producer  (right, TFLuna3) after global update
Sema4_t TFLuna2Ready;  // signaled by Producer2 (left,  TFLuna2) after global update

//******** Producer ***************
// The Producer in this lab will be called from the UART2 ISR
// The TFLuna2 samples distance at about 100 Hz
// sends data to the consumer, runs periodically at 100Hz
void Producer(uint32_t data){
  if(Running){           // finite time run
    TogglePA16();        // toggle PA16
#if USE_MEDIAN_FILTER
    Distance2 = (uint32_t)Median5((int32_t)data);
#else
    Distance2 = data;
#endif
    OS_bSignal(&TFLuna3Ready);
    TogglePA16();        // toggle PA16
  }
}

void Producer2(uint32_t data){
  if(Running){
#if USE_MEDIAN_FILTER
    L_Distance2 = (uint32_t)Median5_2((int32_t)data);
#else
    L_Distance2 = data;
#endif
    OS_bSignal(&TFLuna2Ready);
  }
}

uint32_t FrontCount = 0; // debug: increments each time TFLuna1 ISR delivers data
void Producer3(uint32_t data){
#if USE_MEDIAN_FILTER
  FrontDist = (uint32_t)Median((int32_t)data);
#else
  FrontDist = data;
#endif
  FrontCount++;
}

void Display(void); 

// Describe the error with text on the LCD and then stall. 
// If you are getting disk errors, rerun Testmain1 Testmain2 Testmain3
void diskError(char *errtype, int32_t code){
  OS_bSignal(&LCDFree);
  ST7735_DrawString(0, 1, "Err: ", ST7735_RED);
  ST7735_DrawString(5, 1, errtype, ST7735_RED);
  ST7735_DrawString(0, 2, "Code:     ", ST7735_RED);
  ST7735_SetCursor(6, 2);
  ST7735_SetTextColor(ST7735_RED);
  ST7735_OutUDec(code);
  Running = 0;
  OS_Kill();
}
void StartFileDump(char *pt){
  OS_bWait(&LCDFree);
  eFile_Create(pt); // ignore error if file already exists
  if(eFile_WOpen(pt))  diskError("eFile_WOpen",0);
  if(eFile_WriteString("time_ms,elapsed,tf_r,tf_l,tf_front,sp,ot_state\n"))  diskError("eFile_WriteString",0);
  OS_bSignal(&LCDFree);
}
void EndFileDump(){
  OS_bWait(&LCDFree);
  if(eFile_WClose())           // diskError("eFile_WClose",0);
  OS_bSignal(&LCDFree);
}

void FileDump(uint32_t data, uint32_t data2){
  SetPB4();
  OS_bWait(&LCDFree);
  eFile_WriteUFix2(OS_MsTime()/10); eFile_Write('\t');
  eFile_WriteUDec(data); eFile_Write('\t');
  eFile_WriteUDec(data2); eFile_WriteString("\n\r");
  OS_bSignal(&LCDFree);
  ClrPB4();
}

#define NUMCOLS 7
// Dump row into csv
int FileDumpRow(int32_t* row){
  OS_bWait(&LCDFree);

  for (int i = 0; i < NUMCOLS; i++){
    if (eFile_WriteSDec(row[i])){
      OS_bSignal(&LCDFree);
      return 1;
    } 
    if (i < NUMCOLS - 1){
      if(eFile_Write(',')){
        OS_bSignal(&LCDFree);
        return 1;
      }
    }
  }
  
  if (eFile_Write('\n')){
    OS_bSignal(&LCDFree);
    return 1;
  }

  OS_bSignal(&LCDFree);
  return 0;
}

typedef enum { S_FOLLOW, S_DETECT, S_COMMIT, S_PASS, S_REJOIN } overtake_state_t;
static overtake_state_t ot_state = S_FOLLOW; // Follow by default
static int32_t ot_side = 0;
static uint32_t ot_detect_cnt = 0;
static uint32_t ot_state_entry_ms = 0;
static uint32_t ot_clear_since_ms = 0;
static int32_t front_prev_filt = 0;
static int32_t dist_ref_target = 0;
static int32_t v_hat = 0;
static int32_t r_acc = 0;
static int32_t prev_throttle_avg = 0;
static int32_t front_m1 = 0;
static int32_t front_m2 = 0;

int32_t median3_i32(int32_t a, int32_t b, int32_t c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

int32_t ramp_toward(int32_t cur, int32_t tgt, int32_t step) {
    if      (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
    else if (cur > tgt) { cur -= step; if (cur < tgt) cur = tgt; }
    return cur;
}

int32_t throttle_to_v_mmps(int32_t throttle) {
    if (throttle < 0) throttle = 0;
    return (int32_t)((long long)throttle * V_MAX_MMPS / 9999); // 9999 = max throttle cmd
}

// Overtaking logic follows, detects a car in front, commits to a side, passes, rejoins to the center, and then goes back to following (normal control)
// ot_side = -1 means right pass, +1 means left
static overtake_state_t overtake_step(int32_t front_filt, int32_t r_acc_in, int32_t r_thresh, int32_t d2, int32_t ld2, int32_t gz_ddps, uint32_t now_ms) {
    overtake_state_t next_state = ot_state; // Stay in this state by default

    // bail out if something wrong.
    if (ot_state != S_FOLLOW && ot_state != S_DETECT) {
        int bail = 0;
        if (front_filt < 200) bail = 1; // wall coming, < 20cm
        if (abs(gz_ddps) > ABORT_GZ) bail = 1; // spinning out, gyro too much
        if (ot_state == S_COMMIT || ot_state == S_PASS) {
            if (ot_side == -1 && d2 < ABORT_SIDE) bail = 1;
            if (ot_side == 1 && ld2 < ABORT_SIDE) bail = 1;
        }
        int32_t df_dt = front_filt - front_prev_filt;
        if (ot_state == S_COMMIT && df_dt > ABORT_DF) bail = 1; // object moved out of the way before we finished the pass
        if (bail) {
            dist_ref_target = 0;
            return S_FOLLOW;
        }
    }

    switch(ot_state) {
        case S_FOLLOW: // Normal algorithm, but could switch into overtaking if car is detected in front
            dist_ref_target = 0;

            // Car detected when obstacle in front is close and is closing in slower than a stationary wall would
            // Will move to detect state if there is also enough room on one side to pass
            if (front_filt < FRONT_DETECT && r_acc_in > r_thresh && (d2 > D2_MIN_PASS || ld2 > LD2_MIN_PASS)) {
                next_state = S_DETECT;
                ot_detect_cnt = 0;
            }
            break;

        case S_DETECT:
            // pick the side with more room to commit and pass
            dist_ref_target = 0;
            if (front_filt < FRONT_DETECT && r_acc_in > r_thresh) {
                if (d2 >= D2_MIN_PASS && d2 >= ld2) { // go right
                    ot_side = -1;
                    dist_ref_target = -OFFSET_CMD;
                    next_state = S_COMMIT;
                    ot_state_entry_ms = now_ms;
                    // ST7735_Message(1, 4, "COMMIT R", d2);
                } else if (ld2 >= LD2_MIN_PASS) { // go left
                    ot_side = 1;
                    dist_ref_target = OFFSET_CMD;
                    next_state = S_COMMIT;
                    ot_state_entry_ms = now_ms;
                } else {
                    if (front_filt < 400) next_state = S_FOLLOW; // too close, can't pass?
                }
            } else {
                next_state = S_FOLLOW; // fallback to following
            }
            break;

        case S_COMMIT:
            // Wait until we're close enough to a wall to pass around the car in front
            if (abs(dist_ref_cur - dist_ref_target) < 10) {
                next_state = S_PASS;
                ot_state_entry_ms = now_ms;
                ot_clear_since_ms = 0; // front is not clear
            }
            break;

        case S_PASS:
            // Stay passing until the front is clear for T_CLEAR_MS milliseconds and we've been passing for at least T_PASS_MIN
            if (front_filt > PASS_CLEAR) {
                if (ot_clear_since_ms == 0) ot_clear_since_ms = now_ms; // start clear-timer on first clear sample
                if ((now_ms - ot_clear_since_ms) > T_CLEAR_MS && (now_ms - ot_state_entry_ms) > T_PASS_MIN) {
                    next_state = S_REJOIN;
                    dist_ref_target = 0;
                    ot_state_entry_ms = now_ms;
                }
            } else {
                ot_clear_since_ms = 0; // reset if something still in front
            }
            break;

        case S_REJOIN:
            // Stay close to the wall if there's still something next to us
            if (ot_side == -1 && d2 < ABORT_SIDE) {
              dist_ref_target = -OFFSET_CMD;
              next_state = S_PASS;
            }
            if (ot_side == 1 && ld2 < ABORT_SIDE) {
              dist_ref_target = OFFSET_CMD;
              next_state = S_PASS;
            }
            // Otherwise, settle back into the center of the track and then settle
            if (abs(dist_ref_cur) < 10 && (now_ms - ot_state_entry_ms) > T_SETTLE) next_state = S_FOLLOW;
            break;
    }

    return next_state;
}

//******** Robot *************** 
// foreground Consumer thread, accepts data from producer
// inputs:  none
// outputs: none
char FileName[8]="robot0";

uint32_t elapsed = 0;
int32_t prevError = 0;
int32_t prevE_A = 0;
uint32_t prevTime = 0;
uint32_t startTime = 0;
int16_t gyroZ_bias = 0;
void Robot(void){
  DataLost = 0;       // new run with no lost data 
  FilterWork = 0;
  Running = 1;
  Jitter3_Init();
  Model_Inputs[steering_prev] = 32768; // Initialize to 0 degrees
  OS_ClearMsTime();    
  OS_Fifo_Init(256);
  NumCreated += OS_AddThread(&Display,128,0); 
  UART_OutString("Robot running...");
  StartFileDump(FileName);
  OS_ClearMsTime();
  while (LaunchPad_InS2() == 0); // REMOVE AFTER DATA COLLECTION

  // Gyro-Z bias estimation. Robot must remain stationary.
  // 1 s settle buffer so the user's hand leaves S2 and any vibration decays;
  // motors stay idle until the control loop below.

  OS_Sleep(1000);
  int32_t gzSum = 0;
  const uint32_t BIAS_SAMPLES = 64; // ~1.6 s at 25 ms spacing
  for (uint32_t i = 0; i < BIAS_SAMPLES; i++) {
    long csr = StartCritical();
    int16_t g = IMU_GyroZ;
    EndCritical(csr);
    gzSum += g;
    OS_Sleep(25); // wait for IMU update
  }
  gyroZ_bias = (int16_t)(gzSum / (int32_t)BIAS_SAMPLES);
  
  startTime = OS_MsTime();
  while(1) {
    elapsed = OS_MsTime() - prevTime;
    prevTime = OS_MsTime();
#if USE_MEDIAN_FILTER
    for (uint8_t i = 0; i < 5; i++) {
      OS_bWait(&TFLuna3Ready);
      OS_bWait(&TFLuna2Ready);
    }
    __disable_irq();
    uint32_t d2  = Distance2;
    uint32_t ld2 = L_Distance2;
    __enable_irq();
#else
    uint32_t d2 = 0;
    uint32_t ld2 = 0;
    for (uint8_t i = 0; i < 8; i++) {
      OS_bWait(&TFLuna3Ready);  // block until right-side TFLuna has fresh data
      OS_bWait(&TFLuna2Ready);  // block until left-side TFLuna has fresh data
      __disable_irq();
      d2 += Distance2;
      ld2 += L_Distance2;
      __enable_irq();
    }
    d2 >>= 3;
    ld2 >>= 3;
#endif

    __disable_irq();
    uint32_t d_ir  = Distance;
    uint32_t ld_ir = L_Distance;
    __enable_irq();

    // FOR CALIBRATION
    // ST7735_Message(1, 0, "L_DistanceRaw: ", L_DistanceRaw);
    // ST7735_Message(1, 1, "DistanceRaw: ", DistanceRaw);
    // IR correction: if TFLuna sees open space (>600mm) and reads significantly
    // more than the IR, the IR is past its calibrated range — clamp to 305mm.
    if (d2  > 600 && d2  > d_ir  + 150) d_ir  = 305;
    if (ld2 > 600 && ld2 > ld_ir + 150) ld_ir = 305;

    // Read IMU to use for controller refinement
    long sr = StartCritical();
    int16_t gz_raw = IMU_GyroZ;
    int16_t ax = IMU_AccelX;
    int16_t ay = IMU_AccelY;
    EndCritical(sr);

    // Read IMU inputs
    int16_t gz = (int16_t)(gz_raw - gyroZ_bias); // bias-corrected for heuristic path
    int32_t ax_mg = ((int32_t)ax * 1000) / ACCEL_SCALE;
    int32_t gz_ddps = ((int32_t)gz * 10) / GYRO_SCALE;

    __disable_irq();
    int32_t front = (int32_t)FrontDist;
    __enable_irq();

    // Filter front TFLuna
    int32_t front_filt = median3_i32(front, front_m1, front_m2);

    // Estimate velocity so we can drive overtake FSM
    // r > 0 means obstacle moving away (slower car ahead), r < 0 means closing faster than us
    int32_t dt_ms = elapsed ? elapsed : 20;
    int32_t v_meas = throttle_to_v_mmps(prev_throttle_avg);
    v_hat = v_hat + (((v_meas - v_hat) * V_ALPHA_Q8) >> 8);
    int32_t front_pred = front_prev_filt - (v_hat * dt_ms) / 1000;
    int32_t r = front_filt - front_pred;
    r_acc = ((r_acc * R_ACC_ALPHA_Q8) >> 8) + r; // accumulate residual
    if (r_acc >  R_CAP) r_acc =  R_CAP;
    if (r_acc < -R_CAP) r_acc = -R_CAP;
    int32_t r_thresh = R_THRESH_BASE + ((R_YAW_GAIN_Q8 * abs(gz_ddps)) >> 8); // harder to trip overtake logic while turning
    ot_state = overtake_step(front_filt, r_acc, r_thresh, d2, ld2, gz_ddps, OS_MsTime());
    
    // Slow ramp for overtaking adjustments
    int32_t ramp = (abs(gz_ddps) > GZ_RAMP_GATE) ? OFFSET_RATE/2 : OFFSET_RATE; 
    dist_ref_cur = ramp_toward(dist_ref_cur, dist_ref_target, ramp);
    
    front_m2 = front_m1; front_m1 = front_filt; front_prev_filt = front_filt; // shift history buffers

    // calculate angle to each wall, then project to perpendicular distance
    int32_t angle   = arctan(((int32_t)(d_ir*1414)  - (int32_t)(d2*1000)) /(int32_t)(224+d2))  - angle_ref;
    int32_t L_angle = arctan(((int32_t)(ld_ir*1414) - (int32_t)(ld2*1000))/(int32_t)(224+ld2));

    int32_t realDist   = (d_ir  * cosine(angle))   / 1000;
    int32_t L_realDist = (ld_ir * cosine(L_angle)) / 1000;
    int32_t e_d = realDist - L_realDist - dist_ref_cur; // lateral error: positive = too far right

    // outer PD: lateral error produces intended heading angle
    int32_t intend_angle = ((kp_d * e_d) / 10) + ((kd_d * (e_d - prevError)) / 10);
    prevError = e_d;

    // inner PD: angle error produces steering command
    int32_t e_a = intend_angle - angle;
    int32_t steeringAngle = ((e_a * kp_a) / 10) + (((e_a - prevE_A) * kd_a) / 10);
    prevE_A = e_a;

    uint16_t throttle = 9990; // assume max throttle
    
    // Follow the gap kicks in when turn comes up
    if(ot_state != S_COMMIT && ot_state != S_PASS && front < 800){
      if (front < 600) throttle -= 1000;
      if (front < 400) throttle -= 1000;
      if (front < 200) throttle -= 1000;
      int32_t urgency = (800-front) >> 2; // steer even harder for race day track
      steeringAngle = (ld2 > d2) ? -urgency : urgency; // turn left if more room on left
    }

    // Clamp steering angle
    if (steeringAngle < -35) steeringAngle = -35;
    else if (steeringAngle > 35) steeringAngle = 35;

    if (front < 120){ // too close, tell motors we crashed
      // ST7735_Message(1, 1, "AHHH", 0);
      CAN_TellCrashed(steeringAngle);
      continue;
    }

    // Differential steering and IMU-based Traction Control / Braking
    int32_t t_l = throttle;
    int32_t t_r = throttle;

    // If we're pulling too many lateral G's, we need to slow down
    if (ax_mg > 500 || ax_mg < -500) {
        t_l -= 2500;
        t_r -= 2500;
    }

    // If we're supposed to be turning but we're not, differential steering to force us to rotate
    if (steeringAngle > 15 && gz_ddps > -500) { 
        t_l -= 3500; 
    }
    else if (steeringAngle < -15 && gz_ddps < 500) {
        t_r -= 3500;
    } else {
        // Fallback to basic differential steering (older algorithm before IMU)
        if (steeringAngle <= -15) {
            t_r -= 2000;
        }
        else if (steeringAngle >= 15) {
            t_l -= 2000;
        }
    }
    
    // Prevent underflow
    if (t_l < 0) t_l = 0;
    if (t_r < 0) t_r = 0;

    uint16_t throttle_l = (uint16_t)t_l;
    uint16_t throttle_r = (uint16_t)t_r;

    // Normalize inputs to model
    // Want to place inputs in range [0, 65536], or [0,1] in fixed point
    // Model_Inputs[ir_right] = Model_Normalize(d_ir, CAP_IR);
    // Model_Inputs[ir_left] = Model_Normalize(ld_ir, CAP_IR);
    // Model_Inputs[tf_left] = Model_Normalize(ld2, CAP_TFLUNA);
    // Model_Inputs[tf_middle] = Model_Normalize(front, CAP_TFLUNA);
    // Model_Inputs[tf_right] = Model_Normalize(d2, CAP_TFLUNA);
    // // Clamp angles
    // if (L_angle < -CAP_ANGLE) L_angle = -CAP_ANGLE;
    // if (L_angle > CAP_ANGLE) L_angle = CAP_ANGLE;
    // if (angle < -CAP_ANGLE) angle = -CAP_ANGLE;
    // if (angle > CAP_ANGLE) angle = CAP_ANGLE;
    // // Normalize angles
    // Model_Inputs[angle_left] = (L_angle + CAP_ANGLE) << (16 - CAP_ANGLE_POW - 1);
    // Model_Inputs[angle_right] = (angle + CAP_ANGLE) << (16 - CAP_ANGLE_POW - 1);

    // Model_Inputs[yaw_rate]   = Model_NormalizeSigned((int32_t)(-gz_raw), CAP_YAW);
    // Model_Inputs[accel_lat]  = Model_NormalizeSigned((int32_t)ax, CAP_ACCEL);
    // Model_Inputs[accel_long] = Model_NormalizeSigned((int32_t)ay, CAP_ACCEL);

    // // Call model inference
    // Model_Inference();

    // // Apply model output to PD controller
    // if (ot_state == S_FOLLOW) {
    //   Model_ApplyResidual(&throttle_l, &throttle_r, &steeringAngle);
    // }

    // // Wait till after residuals are applied, so input to next is an accurate reflection
    // Model_Inputs[throttle_left_prev] = Model_Normalize(throttle_l, CAP_THROTTLE);
    // Model_Inputs[throttle_right_prev] = Modesoftwarel_Normalize(throttle_r, CAP_THROTTLE);
    // Model_Inputs[steering_prev] = Model_NormalizeSigned(steeringAngle, CAP_STEERING);

    ST7735_Message(1, 2, "throt r", throttle_r);
    ST7735_Message(1, 3, "throt L", throttle_l);
    CAN_SetMotors(throttle_l, throttle_r, steeringAngle);
    prev_throttle_avg = ((int32_t)throttle_l + throttle_r) / 2;

    // uint32_t sp;
    // __asm volatile ("mov %0, sp" : "=r" (sp));

    // Data collection
    // int32_t row[NUMCOLS] = {OS_MsTime(), elapsed, d2, ld2, front, (int32_t)sp, ot_state};
    // if (FileDumpRow(row)){
    //   EndFileDump();
    //   char* name;
    //   unsigned long size;
    //   eFile_DirNext(&name, &size);
    //   ST7735_Message(0, 2, "File dump complete ", 0);
    //   ST7735_Message(0, 3, "File size: ", size);
    //   while (1){ // Stop robot when we can no longer log
    //     CAN_SetMotors(0, 0, 0);
    //   }
    // }
  }
  EndFileDump();
  UART_OutString("done.\n\r>");
  FileName[5] = (FileName[5]+1)&0xF7; // 0 to 7
  Running = 0;             // robot no longer running
  OS_Kill();
}

 //************S2Push*************
// Called when S2 Button pushed, fall of PB21
// Adds another Robot foreground task
// background threads execute once and return
void S2Push(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&Robot,128,1);  // test eDisk
  }
}
//--------------end of Task 2-----------------------------
 
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
  uint32_t distance;
  uint32_t myId = OS_Id();
  ST7735_Message(0,1,"myId = ",myId);   // top half used for Display
  ST7735_Message(0,2,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
  while(Running) { 
    TogglePB1();        // toggle PB1
    distance = OS_MailBox_Recv();
// you will calibrate this in Lab 6
    TogglePB1();        // toggle PB1
    ST7735_Message(0,3,"Time(ms) =",OS_MsTime());  
    ST7735_Message(0,4,"work  =",FilterWork);  
    ST7735_Message(0,5,"d(mm) =",distance);  
    ST7735_Message(0,6,"d IR Raw=", DistanceRaw);
    ST7735_Message(0,7,"d IR =", Distance);
    if (Distance2 > 2000) Distance2 = 2000;
    ST7735_Message(1,0,"wall_angle =", arctan(((int32_t)(Distance*1414) - (int32_t)(Distance2*1000))/(int32_t)(224+Distance2)));
    TogglePB1();        // toggle PB1
 } 
  OS_Kill();  // done
} 

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
// foreground thread that runs without waiting or sleeping
// it executes a virus detector 
uint32_t Check(uint32_t start, uint32_t end){
  uint32_t sum=0;
  uint32_t *pt; pt = (uint32_t *)start;
  while((uint32_t)pt < end){
    sum += *pt++;
  }
  return sum;
}
//******** Virus Detector *************** 
// foreground thread, performs a checksum of all ROM
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
uint32_t Checksum;             // sum of data stored in ROM
uint32_t ChecksumOriginal;     // sum of data stored in ROM
uint32_t ChecksumErrors;
void VirusDetector(void){ 
  Checks = ChecksumErrors = 0;
  ChecksumOriginal = Check(0,0x20000);
  while(1) { 
    TogglePB20();        // toggle PB20
    Checksum = Check(0,0x20000);
    Checks++;
    if(Checksum !=  ChecksumOriginal){
      ChecksumErrors++; 
    }    
  }
}

//--------------end of Task 4-----------------------------

//------------------Task 5--------------------------------
// UART0 background ISR performs serial input/output
// Two software fifos are used to pass I/O data to foreground
// The interpreter runs as a foreground thread
// The UART0 driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// The UART0 ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// Similarly, the transmit channel waits on a semaphore in the foreground
// and the UART0 ISR signals this semaphore (TxRoomLeft) when getting data from fifo

//******** Interpreter *************** 
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
void Interpreter(void);    // just a prototype, link to your interpreter
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumCreated, MaxJitter, DataLost, FilterWork, Checks
      
// 2) print debugging parameters 
//    i.e., Checks, ChecksumErrors

// Call these from your interpreter
void Lab4(void){int i;
  UART_OutString("\r\nLab 4 performance data");
  UART_OutString("\r\nFilterWork     = "); UART_OutUDec(FilterWork);
  UART_OutString("\r\nNumCreated     = "); UART_OutUDec(NumCreated);
  UART_OutString("\r\nChecksWork     = "); UART_OutUDec(ChecksWork);
  UART_OutString("\r\nDataLost       = "); UART_OutUDec(DataLost); 
  UART_OutString("\r\nReal-time sampling jitter (cyc)");
  UART_OutString("\r\nTime,  Frequencies");
  for(i=0; i<JitterSize3; i++){
    if(JitterHistogram3[i]){ // skip blanks
      UART_OutString("\r\n "); 
      UART_OutUDec5(i);
      UART_OutUDec5(JitterHistogram3[i]);
    }
  }
  UART_OutString("\r\nMaxJitter3(cyc) = "); UART_OutUDec(MaxJitter3); 
}

void DFT(void){ int i;  int32_t real,imag,mag;
  UART_OutString("\r\nLab 2/3 DFT data");
  UART_OutString("\r\nInput,  Output Real, Output Imaginary, Magnitude");
  for(i=0; i<8; i++){
    real = ReX[i];
    imag = ImX[i];    
    mag = sqrt2(real*real+imag*imag);
    UART_OutString("\r\n"); UART_OutUDec(x[i]); UART_OutChar(' '); UART_OutSDec(real); UART_OutChar(' '); UART_OutSDec(imag);
    UART_OutChar(' '); UART_OutSDec(mag);
  }
}

//--------------end of Task 5-----------------------------

// IMU Task
void IMU_Task(void){
  while (1){
    IMU_Read(); // Update globals
    // Debugging
    // ST7735_Message(1, 1, "AccelX: ", IMU_AccelX);
    // ST7735_Message(1, 2, "AccelY: ", IMU_AccelY);
    // ST7735_Message(1, 3, "GyroZ: ", IMU_GyroZ);

    OS_Sleep(20); // ~50 Hz
  }
}


//*******************final user main DEMONTRATE THIS TO TA**********
int realmain(void){     // realmain
  OS_Init();        // initialize, disable interrupts

  Logic_Init();
  DataLost = 0;     // lost data between producer and consumer
  FilterWork = 0;
  Jitter3_Init();
  // initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init(256);    // ***note*** 4 is not big enough*****

  // hardware init
  ADC_InitDual(ADC0, 1, 7, ADCVREF_VDDA);  // ch1=right IR (PA26), ch7=left IR (PA22)
	OS_InitSemaphore(&LCDFree, 1);
  OS_InitSemaphore(&TFLuna3Ready, 0);  // Robot blocks until ISR produces first sample
  OS_InitSemaphore(&TFLuna2Ready, 0);

  // CAN init for sending motor commands to motor board
  CAN_Init();
  CAN_EnableInterrupts(1);

  Clock_Delay1ms(20); // Delay for IMU to work
  if (IMU_Init() != 0){
    ST7735_Message(1, 7, "IMU error", 0);
  }

  //Initialize LCD
  ST7735_InitR(INITR_REDTAB); // Motor board uses SSD1306, not ST7735

  // attach background tasks
  OS_AddS2Task(&S2Push,1);      // fall of PB21
  OS_AddPA28Task(&PA28Push,1);  // fall of PA28
  OS_AddPeriodicThread(&DAS,PERIOD/80000,0); // 1 kHz real time sampling of ADC0_3
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk

	// create initial foreground threads
  NumCreated = 0;
  NumCreated += OS_AddThread(&Interpreter,128,3);
  NumCreated += OS_AddThread(&Robot,128,1);  // CAN motor commands (waits for S2)
  NumCreated += OS_AddThread(&IMU_Task, 128, 1); // Gets IMU data
  NumCreated += OS_AddThread(&VirusDetector,128,7);
  
 
  LPF_Init7(500,7);
  TFLuna1_Init(&Producer3);
  TFLuna1_Format_Standard_mm();
  TFLuna1_Frame_Rate();
  TFLuna1_SaveSettings();
  TFLuna1_System_Reset();

  TFLuna2_Init(&Producer2);
  TFLuna2_Format_Standard_mm();
  TFLuna2_Frame_Rate();
  TFLuna2_SaveSettings();
  TFLuna2_System_Reset();

  TFLuna3_Init(&Producer);
  TFLuna3_Format_Standard_mm(); // format in mm
  TFLuna3_Frame_Rate();         // 100 samples/sec
  TFLuna3_SaveSettings();  // save format and rate
  TFLuna3_System_Reset();  // start measurements

  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Format())            diskError("eFile_Format",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 

uint32_t M=1;
uint32_t Random32(void){
  M = 1664525*M+1013904223;
  return M;
}
// 0 to 31
uint32_t Random5(void){
  return (Random32()>>27);
}
// 0 to 127
uint32_t Random7(void){
  return (Random32()>>25);
}
// 0 to 255
uint8_t Random8(void){
  return (Random32()>>24);
}

//*****************Test project 1*************************
// This test should run. If ST7735R works, but this fails:
// -  there may be bad solder joints on Sensor board
// -  the MSPM0 might have bad pins (PB0 or PB8)
// -  SDC not seated correctly or damaged
// Write and read test of random access disk blocks
// Warning: this overwrites whatever is on the disk
unsigned char buffer[512];  // don't put on stack
#define MAXBLOCKS 100
void TestDisk(void){  DSTATUS result;  uint32_t block;  int i; uint8_t n;
  // simple test of eDisk
  ST7735_DrawString(0, 1, "eDisk test      ", ST7735_WHITE);
  UART_OutString("\n\rECE445M, Lab 4 eDisk test\n\rTestmain1\n\r");
  result = eDisk_Init(0);  // initialize disk
  if(result) diskError("eDisk_Init",result);
  UART_OutString("Writing blocks\n\r");
  M = 1;    // seed
  for(block = 0; block < MAXBLOCKS; block++){
    for(i=0;i<512;i++){
      buffer[i] = Random8();        
    }
    SetPA8();     // PA8 high for 100 block writes
    if(eDisk_WriteBlock(buffer,block))diskError("eDisk_WriteBlock",block); // save to disk
    ClrPA8();     
  }  
  UART_OutString("Reading blocks\n\r");
  M = 1;  // reseed, start over to get the same sequence
  for(block = 0; block < MAXBLOCKS; block++){
    SetPB23();     // PB23 high for one block read
    if(eDisk_ReadBlock(buffer,block))diskError("eDisk_ReadBlock",block); // read from disk
    ClrPB23();
    for(i=0;i<512;i++){
      n = Random8(); // same pseudo random sequence
      if(buffer[i] != (0xFF&n)){
        UART_OutString("Read data not correct, block="); UART_OutUDec(block); 
        UART_OutString(", i="); UART_OutUDec(i); 
        UART_OutString(", expected "); UART_OutUDec(0xFF&n); 
        UART_OutString(", read "); UART_OutUDec(buffer[i]);       
        UART_OutString("\n\r");
        OS_Kill();
      }      
    }
  }  
  UART_OutString("Successful test of 100 blocks\n\r");
  ST7735_DrawString(0, 1, "eDisk successful", ST7735_YELLOW);
  Running = 0; // allow launch again
  OS_Kill();
}

void StartTestDisk(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&TestDisk,128,1);  // test eDisk
  }
}

int Testmain1(void){   // Testmain1
  OS_Init();           // initialize, disable interrupts
 // OS_AddDevices(1,1,0);  // attach printf to UART0, allow ST7735, not eFile
  Logic_Init();
  Running = 1; 

  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
  OS_AddS2Task(&StartTestDisk,1);
  OS_AddPA28Task(&StartTestDisk,1);
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&TestDisk,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


//*******************Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed 
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// S1 not needed, 
// S2 not needed
// logic analyzer on PB22 for systick interrupt (in your OS)
//                on PA8 to measure context switch time
void ThreadCS(void){    // only thread running
  while(1){
    TogglePA8();        // toggle PA8
  }
}
int TestmainCS(void){       // TestmainCS
  Logic_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&ThreadCS,128,0); 
  OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here
  return 0;               // this never executes
}


//*****************Test project 2*************************
// Filesystem test. 
// Warning: this reformats the disk, all existing data will be lost
void PrintDirectory(void){ char *name; unsigned long size; 
  unsigned int num;
  unsigned long total;
  num = 0;
  total = 0;
  UART_OutString("\n\r");
  if(eFile_DOpen(""))           diskError("eFile_DOpen",0);
  while(!eFile_DirNext(&name, &size)){
    UART_OutString("Filename = "); UART_OutString(name); UART_OutString("  ");
    UART_OutString("Size (bytes)= "); UART_OutUDec(size); UART_OutString("\n\r");
    total = total+size;
    num++;    
  }
  UART_OutString("Number of Files = "); UART_OutUDec(num); UART_OutString("\n\r");
  UART_OutString("Number of Bytes = "); UART_OutUDec(total); UART_OutString("\n\r");
  if(eFile_DClose())            diskError("eFile_DClose",0);
}
void TestFile(void){   int i; char data; int status;
  UART_OutString("\n\rECE445M Lab 4 eFile test 2\n\r");
  ST7735_DrawString(0, 1, "eFile test 2    ", ST7735_WHITE);
  // simple test of eFile
  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Format())            diskError("eFile_Format",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);
  PrintDirectory();
  if(eFile_Create("file1"))     diskError("eFile_Create",0);
  if(eFile_WOpen("file1"))      diskError("eFile_WOpen",0);
  for(i=5; i<=15; i++){
    eFile_WriteString("Testmain2\tabcdefghijklmnopqrstuvwxyz\t");
    eFile_WriteUFix2(OS_MsTime()/10); eFile_Write('\t');
    eFile_WriteUDec(i); eFile_Write('\t');
    eFile_WriteSFix2(i-10); eFile_Write('\t');
    eFile_WriteSDec(i-10); eFile_WriteString("\n\r");
    OS_Sleep(10);
  }
  if(eFile_WClose())            diskError("eFile_WClose",0);
  PrintDirectory();

  if(eFile_ROpen("file1"))      diskError("eFile_ROpen",0);
  do{
    status = eFile_ReadNext(&data);
    if(status == 0) UART_OutChar(data);
  }while(status==0);
  if(eFile_RClose())  diskError("eFile_RClose",0);
  if(eFile_Delete("file1"))     diskError("eFile_Delete",0);
  PrintDirectory();
  if(eFile_Unmount())           diskError("eFile_Unmount",0);
  UART_OutString("Successful test\n\r");
  ST7735_DrawString(0, 1, "eFile successful", ST7735_YELLOW);
  Running=0; // launch again
  OS_Kill();
}

void StartFileTest(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&TestFile,128,1);  // test eFile
  }
}

int Testmain2(void){   // Testmain2 
  OS_Init();           // initialize, disable interrupts
  Logic_Init();
  Running = 1; 

  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
  OS_AddS2Task(&StartFileTest,1);
  OS_AddPA28Task(&StartFileTest,1);
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&TestFile,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

void PrintFile3(char *pt){int status; char data; 
  OS_bWait(&LCDFree);  
  eFile_ROpen(pt);
  do{
    status = eFile_ReadNext(&data);
    if(status == 0) UART_OutChar(data);
  }while(status==0);
  eFile_RClose();
  OS_bSignal(&LCDFree);
}
void Dump3(uint32_t run,int32_t data){
  SetPA8();
  OS_bWait(&LCDFree);
  eFile_WriteString("Testmain3\tabcdefghijklmnopqrstuvwxyz\t");
  eFile_WriteUFix2(OS_MsTime()/10); eFile_Write('\t');
  eFile_WriteUDec(run); eFile_Write('\t');
  eFile_WriteSFix2(data); eFile_Write('\t');
  eFile_WriteSDec(data); eFile_WriteString("\n\r");
  OS_bSignal(&LCDFree);
  ClrPA8();
}
//*****************Test project 3*************************
// Filesystem stream test. 
// Warning: this reformats the disk, all existing data will be lost
uint32_t Run3=0;
void TestFile3(void){   int i; char data; 
  UART_OutString("\n\rECE445M Lab 4 eFile test 3\n\r");
  ST7735_Message(0,1,"eFile test 3", Run3);
  // test of eFile

  PrintDirectory();
  OS_bWait(&LCDFree);
  eFile_Create(FileName);
  eFile_WOpen(FileName);
  OS_bSignal(&LCDFree);
  for(i=-5; i<=5; i++){
    Dump3(Run3,i);
    Run3++;
    OS_Sleep(10);
  }
  OS_bWait(&LCDFree);
  eFile_WClose();
  OS_bSignal(&LCDFree);
  PrintDirectory();
  PrintFile3(FileName);
  UART_OutString("Successful test 3, Run3="); UART_OutUDec(Run3);
  UART_OutString("\n\r");
  ST7735_Message(0,2,"Run3 =",Run3);  

  Running = 0; // allowed to launch again
  FileName[5] = (FileName[5]+1)&0xF7; // 0 to 7
 
  OS_Kill();
}
void Chaos3(void){
  ST7735_Message(1,0,"Chaos",3); 
  while(1){
    for(int l=1; l<5; l++){
      ST7735_Message(1,l,"n =",Random8());  
    }
    OS_Sleep(100);
  }
}
void StartFileTest3(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&TestFile3,128,1);  // test eFile
  }
}

int Testmain3(void){   // Testmain3 
  OS_Init();           // initialize, disable interrupts
  Logic_Init();
  Running = 1; 
	OS_InitSemaphore(&LCDFree, 1);

  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,1,0);   // time out routines for disk
  OS_AddS2Task(&StartFileTest3,1);
  OS_AddPA28Task(&StartFileTest3,1);
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&TestFile3,128,1);  
  NumCreated += OS_AddThread(&Chaos3,128,1);  
  NumCreated += OS_AddThread(&VirusDetector,128,3); 

  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Format())            diskError("eFile_Format",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


//*****************Dump robot0 over UART*************************
// Press S2 to stream the contents of "robot0" out UART0.
// Capture on host with log_serial.sh (at repo root).
// Does NOT format the disk — preserves logged data.
void DumpRobotFile(void){
  PrintFile3("robot0");
  Running = 0; // allow re-trigger via S2
  OS_Kill();
}

void StartDumpRobotFile(void){
  if(Running == 0){
    Running = 1;
    NumCreated += OS_AddThread(&DumpRobotFile, 128, 1);
  }
}

int DumpRobotFileMain(void){
  OS_Init();
  Logic_Init();
  Running = 0;
  OS_InitSemaphore(&LCDFree, 1);

  OS_AddPeriodicThread(&disk_timerproc, 1, 0);
  OS_AddS2Task(&StartDumpRobotFile, 1);
  OS_AddPA28Task(&StartDumpRobotFile, 1);

  NumCreated = 0;

  NumCreated += OS_AddThread(&VirusDetector, 128, 7);
  NumCreated += OS_AddThread(&Interpreter, 128, 1);

  if(eFile_Init())  diskError("eFile_Init", 0);
  if(eFile_Mount()) diskError("eFile_Mount", 0);

  UART_OutString("\n\rReady. Press S2 (PB21) to dump robot0.\n\r");
  OS_Launch(TIME_2MS);
  return 0;
}


//*****************Test project IMU*************************
// Read IMU, scale to physical units, display on LCD.
// Accel in milli-g (1000 mg = 1 g).
// Gyro  in deci-dps (10 ddps = 1 deg/s).
// Temp  in centi-C  (100 cC = 1 C).
void TestIMU(void){
  while(1){
    IMU_Read();

    int32_t ax_mg   = ((int32_t)IMU_AccelX * 1000) / 16384;
    int32_t ay_mg   = ((int32_t)IMU_AccelY * 1000) / 16384;
    int32_t az_mg   = ((int32_t)IMU_AccelZ * 1000) / 16384;
    int32_t gx_ddps = ((int32_t)IMU_GyroX  * 10)   / 131;
    int32_t gy_ddps = ((int32_t)IMU_GyroY  * 10)   / 131;
    int32_t gz_ddps = ((int32_t)IMU_GyroZ  * 10)   / 131;
    int32_t t_cC    = ((int32_t)IMU_Temp   * 100)  / 340 + 3653;

    ST7735_Message(0, 0, "ax (mg)  =", ax_mg);
    ST7735_Message(0, 1, "ay (mg)  =", ay_mg);
    ST7735_Message(0, 2, "az (mg)  =", az_mg);
    ST7735_Message(0, 3, "gx(ddps) =", gx_ddps);
    ST7735_Message(0, 4, "gy(ddps) =", gy_ddps);
    ST7735_Message(0, 5, "gz(ddps) =", gz_ddps);
    ST7735_Message(0, 6, "T  (cC)  =", t_cC);

    OS_Sleep(100); // 10 Hz display update
  }
}

int TestmainIMU(void){
  OS_Init();
  Logic_Init();

  ST7735_InitR(INITR_REDTAB);

  int err = IMU_Init();
  if(err != 0){
    ST7735_Message(0, 0, "IMU_Init err =", err);
    while(1){} // halt
  }

  NumCreated = 0;
  NumCreated += OS_AddThread(&TestIMU, 128, 1);
  NumCreated += OS_AddThread(&VirusDetector, 128, 2);

  OS_Launch(TIME_2MS);
  return 0;
}


//*******************Trampoline for selecting which main to execute**********
int main(void) { 			// main
  __disable_irq();
  Clock_Init80MHz(0); // no clock out to pin
  LaunchPad_Init();   // LaunchPad_Init must be called once and before other I/O initializations
  realmain();
}


