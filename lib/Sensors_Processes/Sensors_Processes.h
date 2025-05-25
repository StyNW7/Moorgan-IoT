#pragma once

#include <Sensors_Processes.h>
#include <Adafruit_MPU6050.h>
#include <DS18B20.h>

typedef struct{
    float *mean_ax_arr;
    float *mean_ay_arr;
    float *mean_az_arr;
    float *mean_mv_arr;
    float *std_mv_arr;
    uint8_t windowSize;
}MPUData;

typedef struct{
    float heart_rate;
    float oxygen;
}MAXData;

typedef struct{
    float temp_in_c;
    MPUData mpu_data;
    MAXData max_data;
}MainData;

class Sensors_Processes{
    private:
        DS18B20 *dsb;
        Adafruit_MPU6050 *mpu;
        
        Sensors_Processes();
        static Sensors_Processes *instance;
        Sensors_Processes(const Sensors_Processes&) = delete;
        Sensors_Processes& operator=(const Sensors_Processes&) = delete;

        void mpuSetup();
    public:
        ~Sensors_Processes();
        static Sensors_Processes* getInstance();
        
        MPUData readMPUData();
        void readTempData();
        void setup();
        bool getTempAvail();
        bool getMovAvail();
};