#pragma once
#include <Arduino.h>
// #include <Adafruit_MPU6050.h> // REMOVE

// MPU6050 Register Definitions (ensure all necessary ones are here)
#define MPU6050_I2C_ADDRESS     0x68
#define MPU6050_RA_WHO_AM_I     0x75 // WHO_AM_I register
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_PWR_MGMT_2   0x6C
#define MPU6050_RA_USER_CTRL    0x6A
#define MPU6050_RA_FIFO_EN      0x23
#define MPU6050_RA_INT_PIN_CFG  0x37
#define MPU6050_RA_INT_ENABLE   0x38
#define MPU6050_RA_INT_STATUS   0x3A
#define MPU6050_RA_FIFO_COUNTH  0x72
#define MPU6050_RA_FIFO_COUNTL  0x73
#define MPU6050_RA_FIFO_R_W     0x74

// Add any other registers you might need, e.g., for specific interrupts or configurations

// Accelerometer Full Scale Range options for ACCEL_CONFIG register (AFS_SEL bits)
#define MPU6050_ACCEL_FS_SEL_2G  0x00 // +/- 2g
#define MPU6050_ACCEL_FS_SEL_4G  0x08 // +/- 4g
#define MPU6050_ACCEL_FS_SEL_8G  0x10 // +/- 8g
#define MPU6050_ACCEL_FS_SEL_16G 0x18 // +/- 16g

// Low Power Wake-up Control options for PWR_MGMT_2 register (LP_WAKE_CTRL bits)
#define MPU6050_LP_WAKE_CTRL_1_25HZ 0x00 // 1.25 Hz
#define MPU6050_LP_WAKE_CTRL_5HZ    0x01 // 5 Hz
#define MPU6050_LP_WAKE_CTRL_20HZ   0x02 // 20 Hz
#define MPU6050_LP_WAKE_CTRL_40HZ   0x03 // 40 Hz


struct MPUData{
    float *mean_ax_arr;
    float *mean_ay_arr;
    float *mean_az_arr;
    float *mean_mv_arr;
    float *std_mv_arr;
    uint8_t windowSize;
};

// bool getMovAvail(Adafruit_MPU6050 *mpu); // REMOVE
bool getMovAvail(); // ADD

void writeMPURegister(uint8_t regAddress, uint8_t data);
uint8_t readMPURegister(uint8_t regAddress);
void readMPURegisters(uint8_t regAddress, uint8_t count, uint8_t *dest);

// void mpuSetup(Adafruit_MPU6050 *mpu); // REMOVE
bool mpuSetup(); // ADD - Changed to return bool for success/failure
MPUData *readMPUData();

// Helper to get current sensitivity based on range set in mpuSetup
float getMPUSensitivity();