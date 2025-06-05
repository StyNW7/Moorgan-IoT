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

Sensors_Processes::Sensors_Processes() {
    dsb = nullptr;
    max = nullptr;
    datastream = nullptr;
    _oneWireInstanceForDS18B20 = nullptr; 
    sleepinterval= 60000;
    readingitter = DEFAULTUPLOADINTERVAL; 
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

// MODIFIED setup method
void Sensors_Processes::setup(bool isWakeUp){ // << MODIFIED HERE

    Wire.begin(); 
    Wire.setClock(400000); 

    if(dsb == nullptr){
        if (_oneWireInstanceForDS18B20 == nullptr) { 
            _oneWireInstanceForDS18B20 = new OneWire(DS18B20_PIN);
        }
        dsb = new DS18B20(_oneWireInstanceForDS18B20, DS18B20_RESOLUTION);
    }

    if(max == nullptr){
        max = new MAX30102();
    }

    datastream = new DataStream();

    #ifndef TESTINGMODE
    dsb->begin();
    if (!mpuSetup(isWakeUp)) { // << PASS isWakeUp flag
        xprintln("FATAL: MPU6050 setup failed!");
    }
    max->setup();
    setupBatteryMon();
    max->runReading(); 
    #endif 
}

// ... (rest of Sensors_Processes.cpp remains the same) ...

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
            return NAN; 
        }
    }
    return dsb->getTempC();
}

void Sensors_Processes::sendTelemetryRequest() {
    #ifndef TESTINGMODE
    char *jsonbuffer = parsingDatastreamToJson("Data read successfully");
    #else
    char *jsonbuffer = generateRandomJson("Testing data transmission");
    #endif

    if (jsonbuffer == nullptr) {
        xprintln("ERROR: Failed to generate JSON buffer for telemetry.");
        return;
    }

    if (IOTHubInstance::getInstance()->sendJsonToAzure(jsonbuffer)) {
        xprintln("Telemetry message enqueued by SDK. Waiting for confirmation via DoWork().");
        if (datastream) { 
             datastream->clearData(); 
             xprintln("Datastream cleared (optimistic).");
        }
    } else {
        xprintln("ERROR: Failed to enqueue telemetry data via IOTHubInstance.");
    }

    if (jsonbuffer != nullptr) {
        free(jsonbuffer); 
        jsonbuffer = nullptr;
    }
}

bool Sensors_Processes::performDataCycle(int* rtc_count_ptr) {
    xprintln("Sensors_Processes: Performing data cycle...");
    bool telemetrySentAttempted = false; 

    #ifndef TESTINGMODE
    MainData *data = new MainData();
    if (!data) {
        xprintln("Error: Failed to allocate memory for MainData!");
        return false; 
    }

    data->mpu_data = nullptr; 
    data->max_data = nullptr;

    if(getTempAvail()){ 
        data->temp_in_c = readTempData(); 
    } else {
        xprintln("Temperature sensor not available.");
        data->temp_in_c = NAN; 
    }

    // Check MPU availability before reading. getMovAvail() checks FIFO count.
    // If mpuSetup on wake-up preserved FIFO, getMovAvail() should find data.
    if(getMovAvail()){ 
        xprintln("MPU6050 data available in FIFO, proceeding to read.");
        data->mpu_data = readMPUData(); 
    } else {
        xprintln("Movement sensor not available or no new movement data in FIFO.");
    }
    
    if(max && max->isConnected()){ 
        data->max_data = max->runReading(); 
    } else {
        xprintln("MAX30102 sensor not available or not connected.");
    }
    
    data->distance_from_wifi_m = pow(10,((WIFIMEASUREDPOWER - WiFi.RSSI()) / (10.0f * PATHLOSSEXPONENT)));
    data->battery_percentage = getBatteryCapacity();

    if(datastream){
        datastream->addData(data); 
    } else {
        xprintln("Data stream not initialized. Cleaning up allocated sensor data.");
        if (data) {
            if(data->mpu_data) { 
                if(data->mpu_data->mean_ax_arr) free(data->mpu_data->mean_ax_arr);
                if(data->mpu_data->mean_ay_arr) free(data->mpu_data->mean_ay_arr);
                if(data->mpu_data->mean_az_arr) free(data->mpu_data->mean_az_arr);
                if(data->mpu_data->mean_mv_arr) free(data->mpu_data->mean_mv_arr);
                if(data->mpu_data->std_mv_arr) free(data->mpu_data->std_mv_arr);
                delete data->mpu_data; 
            }
            if(data->max_data) { 
                 delete data->max_data; 
            }
            delete data;
        }
    }
    #else 
    xprintln("TESTINGMODE: Skipping real sensor data collection.");
    #endif

    (*rtc_count_ptr)++; 
    xprintf("RTC Telemetry Reading Count: %d (Threshold: %d)\n", *rtc_count_ptr, readingitter);
    
    if (*rtc_count_ptr >= readingitter) { 
        xprintf("Reading count reached/exceeded threshold. Sending telemetry.\n");
        sendTelemetryRequest(); 
        telemetrySentAttempted = true; 
        *rtc_count_ptr = 0; 
        xprintln("RTC Telemetry Reading Count reset to 0.");
    } else {
        xprintf("Reading count below threshold. Telemetry not sent this cycle.\n");
    }

    #ifdef DEVMODE
    xprintln("Sensors_Processes: Data cycle finished.");
    ESP32C3_Monitor::getInstance().printStatus(); 
    #endif
    return telemetrySentAttempted; 
}


char* Sensors_Processes::parsingDatastreamToJson(char *message) {
    if (datastream == nullptr || datastream->getSize() == 0) {
        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        root["message"] = message ? message : "No message provided";
        root["data"] = nullptr;

        String outputJsonString;
        serializeJson(doc, outputJsonString);

        char* json_c_str = (char*)malloc(outputJsonString.length() + 1);
        if (json_c_str == nullptr) {
            xprintln("Failed to allocate memory for JSON C-string!");
            return nullptr;
        }
        strcpy(json_c_str, outputJsonString.c_str());
        return json_c_str;
    }

    JsonDocument doc; 
    JsonArray rootArray = doc.to<JsonArray>();
    MainData* current = datastream->getHead();

    while (current != nullptr) {
        JsonObject nodeObj = rootArray.add<JsonObject>();
        char timeStr[30];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%SZ", gmtime(&current->timestamp));
        nodeObj["timestamp"] = timeStr;
        nodeObj["message"] = message ? message : "No message provided";
        nodeObj["temp_in_c"] = serialized(String(current->temp_in_c, 2));
        nodeObj["battery_percentage"] = serialized(String(current->battery_percentage, 1));
        nodeObj["wifi_distance_m"] = serialized(String(current->distance_from_wifi_m, 2));

        JsonObject maxObj = nodeObj["max30102"].to<JsonObject>();
        maxObj["heart_rate"] = serialized(String(current->max_data->heart_rate, 1));
        maxObj["oxygen"] = serialized(String(current->max_data->oxygen, 1));
        
        if (current->mpu_data != nullptr) {
            JsonObject mpuObj = nodeObj["mpu6050"].to<JsonObject>();
            const MPUData* mpu = current->mpu_data;
            mpuObj["window_size"] = mpu->windowSize;

            auto serializeFloatArray = [&](JsonArray& arr, float* data_arr, uint8_t size) {
                if (data_arr == nullptr) return;
                for (uint8_t i = 0; i < size; ++i) {
                    arr.add(serialized(String(data_arr[i], 3)));
                }
            };

            if (mpu->windowSize > 0) {
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
         xprintln("JSON document overflowed during serialization!");
    }

    char* json_c_str = (char*)malloc(outputJsonString.length() + 1);
    if (json_c_str == nullptr) {
        xprintln("Failed to allocate memory for JSON C-string!");
        return nullptr;
    }
    strcpy(json_c_str, outputJsonString.c_str());
    return json_c_str;
}

char* Sensors_Processes::generateRandomJson(const char* message) {
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();
    root["message"] = message ? message : "No message provided";
    root["random_field_1"] = serialized(String(random(0, 10000) / 100.0f, 2));
    root["random_field_2"] = serialized(String(random(0, 10000) / 100.0f, 2));
    root["random_field_3"] = serialized(String(random(0, 10000) / 100.0f, 2));
    JsonArray arr = root["random_array"].to<JsonArray>();
    for (int i = 0; i < 5; ++i) {
        arr.add(serialized(String(random(0, 10000) / 100.0f, 2)));
    }

    String outputJsonString;
    serializeJson(doc, outputJsonString);

    char* json_c_str = (char*)malloc(outputJsonString.length() + 1);
    if (json_c_str == nullptr) {
        xprintln("Failed to allocate memory for random JSON C-string!");
        return nullptr;
    }
    strcpy(json_c_str, outputJsonString.c_str());
    return json_c_str;
}