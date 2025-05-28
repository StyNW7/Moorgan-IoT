#include <Sensors_Processes.h>
#include <config.h>
#include <stdlib.h>
#include <string.h>
#include <WiFi.h>
#include <Tools.h>
#include <MPU6050.h>
#include <MAX30102.h>
#include <esp_wifi.h>
#include <BatteryMonitor.h>
#include <ArduinoJson.h>
#include "DS18B20.h"
#include <AzureIoTHub.h>
#include <IOTHubInstance.h>
#include <ESP32C3Monitor.h>

// function declarations
Sensors_Processes* Sensors_Processes::instance = nullptr;
#ifdef DEVMODE
ESP32C3_Monitor monitor;
#endif
// SENSORSSSSSSSSSSSS
// all the sensors method and attributes

// Add the constructor definition here
Sensors_Processes::Sensors_Processes() {
    dsb = nullptr;
    mpu = nullptr;
    max = nullptr;
    datastream = nullptr;
    _oneWireInstanceForDS18B20 = nullptr; 
    sleepinterval= 60000;
    readingitter = DEFAULTUPLOADINTERVAL; // 60 times reading itteration for an hour
    readingcount = 0;
    lasttime = 0;
}

unsigned long long Sensors_Processes::getsleepinterval(){
    return sleepinterval;
}

void Sensors_Processes::setsleepinterval(unsigned long long interv){
    sleepinterval = interv;
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

    if(_oneWireInstanceForDS18B20){
        delete _oneWireInstanceForDS18B20;
        _oneWireInstanceForDS18B20 = nullptr;
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
        if (_oneWireInstanceForDS18B20 == nullptr) { // Check if already created
            _oneWireInstanceForDS18B20 = new OneWire(DS18B20_PIN);
        }
        dsb = new DS18B20(_oneWireInstanceForDS18B20, DS18B20_RESOLUTION);
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
    max->runReading(); // make sure there's already data in the buffer
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

    // compute how much time to sleep after reading the sensors
    if(lasttime == 0){
        lasttime = millis();
    } else {
        unsigned long long elapsed = millis() - lasttime;
        if(elapsed < sleepinterval){
            unsigned long long sleepTime_us = (sleepinterval - elapsed) * 1000ULL;
            xflush(); // Ensure all serial data is sent before sleeping
            esp_sleep_enable_timer_wakeup(sleepTime_us); // set to microseconds
            esp_light_sleep_start();
        }
        lasttime = millis();
    }


    // Create a new MainData object to hold the sensor data
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
    
    #ifdef DEVMODE
    xprintln("Memory left after reading Movement.");
    monitor.printStatus();
    #endif
    
    if(max->isConnected()){
        data->max_data = max->runReading();
    } else {
        xprintln("MAX30102 sensor not available.");
    }
    
    #ifdef DEVMODE
    xprintln("Memory left after reading hr and spo.");
    monitor.printStatus();
    #endif
    
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
    
    if(WiFi.status() == WL_CONNECTED){
        // check if mqtt connection is established or not
        if (IOTHubInstance::getInstance()->isAzureIoTConnected()) {
            // check if enough iterations have been done before sending telemetry data
            if(readingcount >= readingitter){
                // parse into json format
                sendTelemetryRequest();
            }

            // mqtt keep alive job
            IoTHubClient_LL_DoWork(IOTHubInstance::getInstance()->getIotHubClientHandle());
        } else {
            IOTHubInstance::getInstance()->setupAzureIoTClient(); // Try to set up again
        }
    }

    #ifdef DEVMODE
    xprintln("Data pulled successfully.");
    monitor.printStatus();
    #endif

    vTaskDelay(50 / portTICK_PERIOD_MS); // Small delay to allow other tasks to run
}

void Sensors_Processes::sendTelemetryRequest(){

    char *jsonbuffer = parsingDatastreamToJson();

    if(IOTHubInstance::getInstance()->sendJsonToAzure(jsonbuffer)){
        free(jsonbuffer); // Free the allocated memory for JSON buffer
        datastream->clearData(); // Clear the data stream after sending
        xprintln("Telemetry data sent successfully.");
    }
}


char* Sensors_Processes::parsingDatastreamToJson() {
    if (datastream == nullptr || datastream->getSize() == 0) {
        // ... (existing code for empty datastream) ...
    }

    // ArduinoJson v7: DynamicJsonDocument is now just JsonDocument.
    // Capacity calculation is less critical as it can grow, but good for initial estimate.
    const size_t capacityPerNode = 800;
    size_t streamSize = datastream->getSize();
    // For v7, you typically don't need to pre-calculate JSON_ARRAY_SIZE.
    // Just provide a reasonable starting capacity.
    JsonDocument doc; // Default constructor, will allocate as needed.
                      // Or, if you want to give a hint: JsonDocument doc(streamSize * capacityPerNode);

    JsonArray rootArray = doc.to<JsonArray>();

    MainData* current = datastream->getHead();
    while (current != nullptr) {
        // v7: Use add<JsonObject>() for arrays or obj[key].to<JsonObject>() for objects
        JsonObject nodeObj = rootArray.add<JsonObject>();

        // Timestamp (ISO 8601 format in UTC)
        char timeStr[30];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", gmtime(&current->timestamp));
        nodeObj["timestamp"] = timeStr;

        nodeObj["temp_in_c"] = serialized(String(current->temp_in_c, 2));
        nodeObj["battery_percentage"] = serialized(String(current->battery_percentage, 1));
        nodeObj["wifi_distance_m"] = serialized(String(current->distance_from_wifi_m, 2));

        // MAX30102 Data
        // v7: Use nodeObj[key].to<JsonObject>()
        JsonObject maxObj = nodeObj["max30102"].to<JsonObject>();
        maxObj["heart_rate"] = serialized(String(current->max_data->heart_rate, 1));
        maxObj["oxygen"] = serialized(String(current->max_data->oxygen, 1));

        // MPU6050 Data
        if (current->mpu_data != nullptr) {
            // v7: Use nodeObj[key].to<JsonObject>()
            JsonObject mpuObj = nodeObj["mpu6050"].to<JsonObject>();
            const MPUData* mpu = current->mpu_data;
            mpuObj["window_size"] = mpu->windowSize;

            auto serializeFloatArray = [&](JsonArray& arr, float* data, uint8_t size) {
                if (data == nullptr) return;
                for (uint8_t i = 0; i < size; ++i) {
                    arr.add(serialized(String(data[i], 3)));
                }
            };

            if (mpu->windowSize > 0) {
                // v7: Use mpuObj[key].to<JsonArray>()
                JsonArray axArr = mpuObj["mean_ax"].to<JsonArray>();
                serializeFloatArray(axArr, mpu->mean_ax_arr, mpu->windowSize);

                JsonArray ayArr = mpuObj["mean_ay"].to<JsonArray>();
                serializeFloatArray(ayArr, mpu->mean_ay_arr, mpu->windowSize);

                JsonArray azArr = mpuObj["mean_az"].to<JsonArray>();
                serializeFloatArray(azArr, mpu->mean_az_arr, mpu->windowSize);

                JsonArray mvArr = mpuObj["mean_mv"].to<JsonArray>();
                serializeFloatArray(mvArr, mpu->mean_mv_arr, mpu->windowSize);

                JsonArray stdArr = mpuObj["std_mv"].to<JsonArray>();
                serializeFloatArray(stdArr, mpu->std_mv_arr, mpu->windowSize);
            }
        } else {
            nodeObj["mpu6050"] = nullptr;
        }

        current = current->next;
    }

    String outputJsonString;
    serializeJson(doc, outputJsonString);

    if (doc.overflowed()) {
        // ... (existing overflow handling) ...
    }

    // ... (existing code to allocate and return char*) ...
    char* json_c_str = (char*)malloc(outputJsonString.length() + 1);
    if (json_c_str == nullptr) {
        xprintln("Failed to allocate memory for JSON C-string!");
        return nullptr;
    }
    strcpy(json_c_str, outputJsonString.c_str());
    return json_c_str;
}