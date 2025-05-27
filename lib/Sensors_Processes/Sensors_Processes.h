#pragma once

#include <Sensors_Processes.h>
#include <Adafruit_MPU6050.h>
#include <DS18B20.h>
#include <DataStream.h>
#include <MPU6050.h>
#include <MAX30102.h>

class Sensors_Processes{
    private:
        DS18B20 *dsb;
        Adafruit_MPU6050 *mpu;
        MAX30102 *max;
        DataStream *datastream;
        unsigned long long uploadinterval;
        uint8_t readingitter;
        uint16_t readingcount;
        
        Sensors_Processes();
        static Sensors_Processes *instance;
        Sensors_Processes(const Sensors_Processes&) = delete;
        Sensors_Processes& operator=(const Sensors_Processes&) = delete;

    public:
        ~Sensors_Processes();
        static Sensors_Processes* getInstance();
        
        unsigned long long getUploadInterval();
        void setUploadInterval(unsigned long long interv);
        float readTempData();
        void setup();
        bool getTempAvail();
        void pullData();
};