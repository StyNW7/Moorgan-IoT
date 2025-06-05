#pragma once

// #include <Adafruit_MPU6050.h> // REMOVE
#include <DS18B20.h>
#include <DataStream.h>
#include <MPU6050.h> // Your MPU6050 library
#include <MAX30102.h>

class Sensors_Processes{
    private:
        DS18B20 *dsb;
        // Adafruit_MPU6050 *mpu; // REMOVE
        MAX30102 *max;
        DataStream *datastream;
        unsigned long long sleepinterval;
        unsigned long long lasttime;
        OneWire * _oneWireInstanceForDS18B20; 
        uint8_t readingitter;
        uint16_t readingcount;
        
        Sensors_Processes();
        static Sensors_Processes *instance;
        Sensors_Processes(const Sensors_Processes&) = delete;
        Sensors_Processes& operator=(const Sensors_Processes&) = delete;

    public:
        ~Sensors_Processes();
        static Sensors_Processes* getInstance();
        
        unsigned long long getsleepinterval();
        void sendTelemetryRequest();
        char *parsingDatastreamToJson(char *message);
        char *generateRandomJson(const char* message);
        void setsleepinterval(unsigned long long interv);
        bool performDataCycle(int* rtc_count_ptr);
        float readTempData();
        void setup(bool isWakeUp); // << MODIFIED HERE
        bool getTempAvail();
        // void pullData();
};