#pragma once

#include <Sensors_Processes.h>
#include <Adafruit_MPU6050.h>
#include <DS18B20.h>
#include <MPU6050.h>

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
        MAX30102 *max;
        
        Sensors_Processes();
        static Sensors_Processes *instance;
        Sensors_Processes(const Sensors_Processes&) = delete;
        Sensors_Processes& operator=(const Sensors_Processes&) = delete;

    public:
        ~Sensors_Processes();
        static Sensors_Processes* getInstance();
        
        float readTempData();
        void setup();
        bool getTempAvail();
        void pullData();
};