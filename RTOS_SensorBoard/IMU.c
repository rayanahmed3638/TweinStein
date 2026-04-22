// IMU.c — GY-521 (MPU-6050) driver over I2C1
// Assumes I2C.c driver on PB2 (SCL) / PB3 (SDA) at 400 kHz.
// AD0 tied low for slave address 0x68.

#include <stdint.h>
#include "IMU.h"
#include "../inc/I2C.h"

volatile int16_t IMU_AccelX;
volatile int16_t IMU_AccelY;
volatile int16_t IMU_AccelZ;
volatile int16_t IMU_Temp;
volatile int16_t IMU_GyroX;
volatile int16_t IMU_GyroY;
volatile int16_t IMU_GyroZ;

// Error codes: 0 = OK, 1 = WHO_AM_I send failed, 2 = WHO_AM_I recv failed,
//              3 = WHO_AM_I mismatch, 4 = wake write failed,
//              5 = DLPF config write failed
int IMU_Init(void){
    uint8_t who;

    I2C_Init();

    // WHO_AM_I sanity check (readable even while chip is asleep)
    if(I2C_Send1(I2C_ADDRESS, MPU_WHO_AM_I) == 0) return 1;
    if(I2C_RecvN(I2C_ADDRESS, &who, 1) == 0)      return 2;
    if(who != I2C_ADDRESS)                        return 3;

    // Wake chip: clear SLEEP bit in PWR_MGMT_1
    if(I2C_Send2(I2C_ADDRESS, MPU_PWR_MGMT_1, 0x00) == 0) return 4;

    // Enable digital low-pass filter (reduces gyro/accel noise)
    if(I2C_Send2(I2C_ADDRESS, MPU_CONFIG, IMU_DLPF_CFG) == 0) return 5;

    return 0;
}

int IMU_Read(void){
    uint8_t buf[14];
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_tp = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for(int i = 0; i < IMU_AVG_SAMPLES; i++){
        if(I2C_Send1(I2C_ADDRESS, MPU_ACCEL_XOUT_H) == 0) return 1;
        if(I2C_RecvN(I2C_ADDRESS, buf, 14) == 0)          return 2;

        sum_ax += (int16_t)((buf[0]  << 8) | buf[1]);
        sum_ay += (int16_t)((buf[2]  << 8) | buf[3]);
        sum_az += (int16_t)((buf[4]  << 8) | buf[5]);
        sum_tp += (int16_t)((buf[6]  << 8) | buf[7]);
        sum_gx += (int16_t)((buf[8]  << 8) | buf[9]);
        sum_gy += (int16_t)((buf[10] << 8) | buf[11]);
        sum_gz += (int16_t)((buf[12] << 8) | buf[13]);
    }

    IMU_AccelX = (int16_t)(sum_ax / IMU_AVG_SAMPLES);
    IMU_AccelY = (int16_t)(sum_ay / IMU_AVG_SAMPLES);
    IMU_AccelZ = (int16_t)(sum_az / IMU_AVG_SAMPLES);
    IMU_Temp   = (int16_t)(sum_tp / IMU_AVG_SAMPLES);
    IMU_GyroX  = (int16_t)(sum_gx / IMU_AVG_SAMPLES);
    IMU_GyroY  = (int16_t)(sum_gy / IMU_AVG_SAMPLES);
    IMU_GyroZ  = (int16_t)(sum_gz / IMU_AVG_SAMPLES);

    return 0;
}
