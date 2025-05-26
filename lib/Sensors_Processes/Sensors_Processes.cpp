#include <Sensors_Processes.h>
#include <config.h>
#include <WiFi.h>
#include <Tools.h>
#include <MPU6050.h>
#include <MAX30102.h>
#include "DS18B20.h"

// function declarations
Sensors_Processes* Sensors_Processes::instance = nullptr;
// SENSORSSSSSSSSSSSS
// all the sensors method and attributes

// Add the constructor definition here
Sensors_Processes::Sensors_Processes() {
    dsb = nullptr;
    mpu = nullptr;
    max = nullptr;
    datastream = nullptr;
}

Sensors_Processes* Sensors_Processes::getInstance() {
    // initialize sensor processes instance
    if (instance == nullptr) {
        instance = new Sensors_Processes();
    }

    return instance;
}

Sensors_Processes::~Sensors_Processes(){
    if(dsb){
        delete dsb;
        dsb = nullptr;
    }

    if(mpu){
        delete mpu;
        mpu = nullptr;
    }

    if(max){
        delete max;
        max = nullptr;
    }
    
    if(datastream){
        datastream->clearData();
        delete datastream;
        datastream = nullptr;
    }
}

void Sensors_Processes::setup(){

    Wire.begin(); // Initialize I2C (SDA, SCL default for ESP32-C3)
    Wire.setClock(400000); // 400kHz I2C clock

    if(dsb == nullptr){
        dsb = new DS18B20(new OneWire(DS18B20_PIN), DS18B20_RESOLUTION);
    }

    if(mpu == nullptr){
        mpu = new Adafruit_MPU6050();
    }

    if(max == nullptr){
        max = new MAX30102();
    }

    datastream = new DataStream();
    dsb->begin();
    mpuSetup(mpu);
    max->setup();
}

// all the temperature sensor methods
// and attributes
bool Sensors_Processes::getTempAvail(){
    if(dsb){
        return dsb->isConnected();
    }
    return false;
}

float Sensors_Processes::readTempData(){

    dsb->requestTemperatures();
    
    int waitedMs = 0;
    while(!dsb->isConversionComplete()){
        vTaskDelay(100 / portTICK_PERIOD_MS);
        waitedMs += 100;
        if (waitedMs >= 2000) {
            // Handle timeout (return error value or throw)
            return NAN; // or another error indicator
        }
    }

    return dsb->getTempC();
}

void Sensors_Processes::pullData(){
    // if(connected to wifi and mqtt connection established)

    MainData *data = new MainData();
    
    if(getTempAvail()){
        data->temp_in_c = readTempData();
    } else {
        xprintln("Temperature sensor not available.");
    }

    if(getMovAvail(mpu)){
        data->mpu_data = readMPUData();
    } else {
        xprintln("Movement sensor not available.");
    }

    if(max->isConnected()){
        max->runReading(&(data->max_data));
    } else {
        xprintln("MAX30102 sensor not available.");
    }

    data->distance_from_wifi_m = pow(10,((WIFIMEASUREDPOWER - WiFi.RSSI()) / (10.0f * PATHLOSSEXPONENT)));

    // Add the data to the data stream
    if(datastream){
        datastream->addData(data);
    } else {
        xprintln("Data stream not initialized.");
        delete data;
    }

    // make the esp32 sleep for a minute
    
    
}