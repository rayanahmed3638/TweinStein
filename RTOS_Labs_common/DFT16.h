// DFT16.h
// 16-point DFT
// 12-bit binary fixed point

// fill x with input data
// point ReX and ImX to empty arrays
// call DFT16
// results in ReX and ImX
// units of ReX and ImX match units of x
void DFT16(int32_t x[16],int32_t ReX[16],int32_t ImX[16]);

