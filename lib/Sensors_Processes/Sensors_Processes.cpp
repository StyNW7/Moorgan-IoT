#include <Sensors_Processes.h>
#include <config.h>
#include <WiFi.h>
#include <Tools.h>
#include <MPU6050.h>
#include <MAX30102.h>
#include <esp_wifi.h>
#include <BatteryMonitor.h>
#include "DS18B20.h"
#include <AzureIoTHub.h>
#include <IOTHubInstance.h>

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
    uploadinterval= 60ULL;
    readingitter = 60; // 60 times reading itteration for an hour
    readingcount = 0;
}

unsigned long long Sensors_Processes::getUploadInterval(){
    return uploadinterval;
}

void Sensors_Processes::setUploadInterval(unsigned long long interv){
    uploadinterval = interv;
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
    setupBatteryMon();
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


// Holy Grail function of the entire project where all the logic is implemented
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

    // add battery info to the main data frame
    data->battery_percentage = getBatteryCapacity();

    // Add the data to the data stream
    if(datastream){
        datastream->addData(data);
    } else {
        xprintln("Data stream not initialized.");
        delete data;
    }

    ++readingcount;
    
    if(WiFi.status() == WL_CONNECTED ){
        // check if mqtt connection is established or not
        if (IOTHubInstance::getInstance()->isAzureIoTConnected() != NULL) {
            // check if enough iterations have been done before sending telemetry data
            if(readingcount >= readingitter){
                // parse into json format
                sendTelemetryRequest();
                // save the json format
                // send telemetry data to mqtt service Azure IOT
                // check if package sent successfuly
                // then if finished, clear the data stream
            }

            // mqtt keep alive job
            IoTHubClient_LL_DoWork(IOTHubInstance::getInstance()->getIotHubClientHandle());
        } else {
            IOTHubInstance::getInstance()->setupAzureIoTClient(); // Try to set up again
        }
    }
    
    
    xflush(); // Ensure all serial data is sent before sleeping
    esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP); //set to how many microsecond for one sleep
    esp_light_sleep_start();
}

void Sensors_Processes::sendTelemetryRequest(){

    char *jsonbuffer = parsingDatastreamToJson();

    if(IOTHubInstance::getInstance()->sendJsonToAzure(jsonbuffer)){
        delete[] jsonbuffer; // Free the allocated memory for JSON buffer
        datastream->clearData(); // Clear the data stream after sending
        xprintln("Telemetry data sent successfully.");
    }
}


char *Sensors_Processes::parsingDatastreamToJson(){
    // use malloc to allocate memory for the JSON buffer string
}