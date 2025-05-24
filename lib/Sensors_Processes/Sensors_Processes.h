#pragma once

#include <Sensors_Processes.h>
#include <Adafruit_MPU6050.h>
#include <DS18B20.h>

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
        
        void readMPUData();
        void readTempData();
        void setup();
        bool getTempAvail();
        bool getMovAvail();
};