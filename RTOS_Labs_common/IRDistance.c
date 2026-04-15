// IRDistance.c
// Runs on any microcontroller
// Provide mid-level functions that convert raw ADC
// values from the GP2Y0A21YK0F infrared distance sensors to
// distances in mm. STUDENTS WILL NEED TO CALIBRATE THEIR OWN SENSORS
// Jonathan Valvano
// Jan 15, 2020

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2020
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2020, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/



#include <stdint.h>

/* Calibration data
distance measured from front of the sensor to the wall                
d(cm) 1/d    bL     al     aR   bR  adcSample d (0.01cm)  error
10    0.100  2813  2830  2820  2830  2823.25  1006        0.06
15    0.067  1935  1976  1986  1978  1968.75  1482       -0.18
20    0.050  1520  1500  1520  1550  1522.5   1966       -0.34
30    0.033  1040  1096  1028   933  1024.25  3099        0.99
  
      adcSample = 26813/d+159      2681300    
      d = 26813/(adcSample-159)      -159    
*/
const int32_t A[4] = { 268130,268130,268130,268130};
const int32_t B[4] = { -159,-159,-159,-159};
const int32_t C[4] = { 0,0,0,0};
const int32_t IRmax[4] = { 494,494,494,494};
// returns distance in mm
int32_t IRDistance_Convert(int32_t adcSample, uint32_t sensor){
  if(adcSample < IRmax[sensor]){
    return 300;
  }
  return A[sensor]/(adcSample + B[sensor]) + C[sensor];
}

/* Calibrated constants from IR_Calib.xlsx (GP2Y0A21, 2-12 inch range)
   d(mm) = A / (adc + B) + C
   Right (PA26, ADC0 ch1): A=76921, B=-495, C=20, RMSE~4.6mm
   Left  (PA22, ADC0 ch7): A=83326, B=-498, C=18, RMSE~7.5mm
   Note: left sensor 5-inch calibration point appears ~0.5in short;
         that measurement produces ~18mm error; all other points are <11mm.
   Below the minimum calibrated ADC the sensor is beyond ~12in/305mm,
   so 305 is returned as the out-of-range sentinel. */
#define IR_RIGHT_A      76921
#define IR_RIGHT_B       -495
#define IR_RIGHT_C         20
#define IR_RIGHT_MIN_ADC  763   // ADC at ~12 inches

#define IR_LEFT_A       83326
#define IR_LEFT_B        -498
#define IR_LEFT_C          18
#define IR_LEFT_MIN_ADC   790   // ADC at ~12 inches

// returns distance in mm for the right IR sensor (PA26, ADC0 ch1)
int32_t IRDistance_Right(int32_t adcSample){
  if(adcSample < IR_RIGHT_MIN_ADC){
    return 305;
  }
  return IR_RIGHT_A / (adcSample + IR_RIGHT_B) + IR_RIGHT_C;
}

// returns distance in mm for the left IR sensor (PA22, ADC0 ch7)
int32_t IRDistance_Left(int32_t adcSample){
  if(adcSample < IR_LEFT_MIN_ADC){
    return 305;
  }
  return IR_LEFT_A / (adcSample + IR_LEFT_B) + IR_LEFT_C;
}



