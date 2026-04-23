#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#define I2C_ADDRESS 0x68

// MPU-6050 register addresses
#define MPU_WHO_AM_I     0x75
#define MPU_PWR_MGMT_1   0x6B
#define MPU_CONFIG       0x1A
#define MPU_ACCEL_XOUT_H 0x3B

// Digital low-pass filter bandwidth (CONFIG register bits [2:0]).
//   0 = 256 Hz gyro BW (default, noisiest)
//   1 = 188 Hz, 2 =  98 Hz, 3 = 44 Hz, 4 = 21 Hz, 5 = 10 Hz, 6 = 5 Hz
// Lower BW = more smoothing, more lag. 3 is a good default for 50 Hz control.
#define IMU_DLPF_CFG     0x03

// Number of back-to-back samples averaged per IMU_Read call.
// Must be power of 2 so compiler uses shift for divide.
#define IMU_AVG_SAMPLES  4

// Conversion scales
#define ACCEL_SCALE 16384 // for 1 G
#define GYRO_SCALE 131 // degrees/sec

// Latest sensor readings, updated on each IMU_Read call.
// Raw signed 16-bit ADC counts (big-endian from chip, assembled).
// Default scale factors
//   accel: 16384 LSB/g    (+/-2 g range)
//   gyro:  131 LSB/(deg/s) (+/-250 dps range)
//   temp:  T_C = raw/340 + 36.53
extern volatile int16_t IMU_AccelX;
extern volatile int16_t IMU_AccelY;
extern volatile int16_t IMU_AccelZ;
extern volatile int16_t IMU_Temp;
extern volatile int16_t IMU_GyroX;
extern volatile int16_t IMU_GyroY;
extern volatile int16_t IMU_GyroZ;

// Initialize MPU-6050.
// Calls I2C_Init, verifies WHO_AM_I, clears SLEEP bit to wake chip.
// Returns 0 on success, nonzero on error (see IMU.c for codes).
int IMU_Init(void);

// Burst read all 14 sensor bytes and update IMU_* globals.
// Returns 0 on success, nonzero on error.
int IMU_Read(void);

#endif
