#pragma once

#include <Sensors_Processes.h>
#include <Adafruit_MPU6050.h>
#include <DS18B20.h>

class Sensors_Processes{
    private:
        static Sensors_Processes *instance;
        DS18B20 *dsb;
        Adafruit_MPU6050 *mpu;

        Sensors_Processes();
        Sensors_Processes(const Sensors_Processes&) = delete;
        Sensors_Processes& operator=(const Sensors_Processes&) = delete;

        void mpuSetup();
    public:
        ~Sensors_Processes();

        bool getTempAvail();
        bool getMovAvail();
        void startReading();
        Sensors_Processes* getInstance();
        void setup();
};