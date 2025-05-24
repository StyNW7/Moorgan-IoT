#include <Sensors_Processes.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <config.h>
#include <OneWire.h>
#include "DS18B20.h"

Sensors_Processes* Sensors_Processes::instance = nullptr;

typedef struct {
    float temperature;
    // float 
} mainData;

typedef struct {
    DS18B20 *dsb;
} tempData;

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

    // delete instance
    if (instance) {
        delete instance;
        instance = nullptr;
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
    mpuSetup();
    // setup MAX30102
}

bool Sensors_Processes::getTempAvail(){
    if(dsb){
        return dsb->isConnected();
    }
    return false;
}

bool Sensors_Processes::getMovAvail(){
    if(mpu){
        return mpu->getMotionInterruptStatus();
    }
    return false;
}

void Sensors_Processes::mpuSetup(){
    mpu->begin();
    mpu->setCycleRate(MPU6050_CYCLE_5_HZ);
    mpu->setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu->setGyroRange(MPU6050_RANGE_500_DEG);
    mpu->setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void Sensors_Processes::startReading(){
    xTaskCreate(
        writeTemp,
        "writeTemp",
        2048,
        (void *)dsb,
        1,
        NULL
    );
}

void writeTemp(void *arg){
    DS18B20 *dsb = (DS18B20 *)arg;
    dsb->requestTemperatures();
    
    while(!dsb->isConversionComplete()){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
