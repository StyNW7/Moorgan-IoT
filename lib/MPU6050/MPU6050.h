#include <Arduino.h>
#include <Adafruit_MPU6050.h>

// MPU6050 Register Definitions
#define MPU6050_I2C_ADDRESS     0x68

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

typedef struct{
    float *mean_ax_arr;
    float *mean_ay_arr;
    float *mean_az_arr;
    float *mean_mv_arr;
    float *std_mv_arr;
    uint8_t windowSize;
}MPUData;

bool getMovAvail(Adafruit_MPU6050 *mpu);
void writeMPURegister(uint8_t regAddress, uint8_t data);
uint8_t readMPURegister(uint8_t regAddress);
void readMPURegisters(uint8_t regAddress, uint8_t count, uint8_t *dest);
void mpuSetup(Adafruit_MPU6050 *mpu);
MPUData readMPUData();