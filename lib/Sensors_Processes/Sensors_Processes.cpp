#include <Sensors_Processes.h>
#include <config.h>
#include <Tools.h>
#include <MPU6050.h>
#include <OneWire.h>
#include "DS18B20.h"


// function declarations

Sensors_Processes* Sensors_Processes::instance = nullptr;
// SENSORSSSSSSSSSSSS
// all the sensors method and attributes

// Add the constructor definition here
Sensors_Processes::Sensors_Processes() {
    dsb = nullptr;
    mpu = nullptr;
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
}

void Sensors_Processes::setup(){
    if(dsb == nullptr){
        dsb = new DS18B20(new OneWire(DS18B20_PIN), DS18B20_RESOLUTION);
    }

    if(mpu == nullptr){
        mpu = new Adafruit_MPU6050();
    }

    
    dsb->begin();
    mpuSetup(mpu);
    // setup MAX30102
}

// all the temperature sensor methods
// and attributes
bool Sensors_Processes::getTempAvail(){
    if(dsb){
        return dsb->isConnected();
    }
    return false;
}

void Sensors_Processes::readTempData(){

    dsb->requestTemperatures();
    dsb->getTempC();
    
    while(!dsb->isConversionComplete()){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}


// all the heart rate and oxygen sensor methods
// and attributes

void Sensors_Processes::maxSetup(){

}
