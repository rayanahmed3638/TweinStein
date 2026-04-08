// DFT16.c
// 16-point DFT
// 12-bit binary fixed point
#include <stdint.h>
const int32_t ReW[16]={4096, 3784, 2896, 1567, 0, -1567, -2896, -3784, -4096, -3784, -2896, -1567, 0, 1567, 2896, 3784};
const int32_t ImW[16]={0, -1567, -2896, -3784, -4096, -3784, -2896, -1567, 0, 1567, 2896, 3784, 4096, 3784, 2896, 1567};
// 12-bit data*12-bit fixed*16 element, 12+12+4 = 28bit intermediate values, no overflow possible
// units of ReX and ImX match units of x
void DFT16(int32_t x[16],int32_t ReX[16],int32_t ImX[16]){
  ReX[0] = 0;
  for(int i=0; i<16; i++)  ReX[0] += x[i];
  ReX[0] >>= 4; // matches units of x
  ImX[0] = 0;   // real inputs
  for(int k=1; k<16; k++){
    ReX[k] = ImX[k] = 0;
    for(int n=0; n<16; n++){
      ReX[k] += x[n]*ReW[(k*n)&0x0F];
      ImX[k] += x[n]*ImW[(k*n)&0x0F];
    }
    ReX[k] >>= 15; // matches units of x
    ImX[k] >>= 15; // matches units of x
  } 
}
