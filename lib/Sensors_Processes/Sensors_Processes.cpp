#include <Sensors_Processes.h>
#include <config.h>
#include <Tools.h>
#include <MPU6050.h>
#include <OneWire.h>
#include <MAX30102.h>
#include "DS18B20.h"

MainData minuteDatas[60];


// function declarations
Sensors_Processes* Sensors_Processes::instance = nullptr;
// SENSORSSSSSSSSSSSS
// all the sensors method and attributes

// Add the constructor definition here
Sensors_Processes::Sensors_Processes() {
    dsb = nullptr;
    mpu = nullptr;
    max = nullptr;
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
    
    while(!dsb->isConversionComplete()){
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    return dsb->getTempC();
}

void Sensors_Processes::pullData(){

}