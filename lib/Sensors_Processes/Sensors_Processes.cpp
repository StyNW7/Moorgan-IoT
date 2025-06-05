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
// DS18B20 temperature sensor pin and resolution
// SENSORSSSSSSSSSSSS
// all the sensors method and attributes

// Add the constructor definition here
Sensors_Processes::Sensors_Processes() {
    dsb = nullptr;
    // mpu = nullptr;
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

    // if(mpu == nullptr){
    //     mpu = new Adafruit_MPU6050();
    // }

    if(max == nullptr){
        max = new MAX30102();
    }

    datastream = new DataStream();

    #ifndef TESTINGMODE
    dsb->begin();
    if (!mpuSetup()) { // Call your new mpuSetup, check return
        xprintln("FATAL: MPU6050 setup failed!");
        // Handle failure, maybe by looping indefinitely or setting an error flag
    }
    max->setup();
    setupBatteryMon();
    max->runReading(); // make sure there's already data in the buffer
    #endif 

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
void Sensors_Processes::pullData() {
    IOTHubInstance* iotHub = IOTHubInstance::getInstance(); // Get instance once

    // --- Sleep Logic ---
    if (lasttime == 0) {
        lasttime = millis();
    } else {
        unsigned long long elapsed = millis() - lasttime;
        if (elapsed < sleepinterval) {
            unsigned long long sleepTime_us = (sleepinterval - elapsed) * 1000ULL;
            xprintf("Preparing to sleep for: %llu microseconds.\n", sleepTime_us);


            // Also, ensure the connection status flag reflects the disconnection.
            // A direct method like iotHub->setConnectionStatus(false); could be useful.
            // The connectionStatusCallback should ideally set iotHubConnected to false
            // when the client is destroyed or connection is lost.

            // Note: platform_deinit() should NOT be called here if it's managed globally
            // by the IOTHubInstance constructor/destructor.

            // Optional: Explicitly disconnect WiFi. Light sleep often does this, but being explicit can be safer.
            // if (WiFi.status() == WL_CONNECTED) {
            //     xprintln("Disconnecting WiFi before light sleep...");
            //     WiFi.disconnect(true); // true to turn off WiFi radio
            //     unsigned long disconnect_start = millis();
            //     while(WiFi.status() == WL_CONNECTED && (millis() - disconnect_start < 1000)) {
            //         delay(100); // Wait up to 1 second for disconnect
            //     }
            //     if (WiFi.status() == WL_CONNECTED) xprintln("WiFi did not disconnect quickly.");
            //     else xprintln("WiFi disconnected.");
            // }

            iotHub->prepareForSleep();

            xflush(); // Ensure log messages are sent
            xprintln("Entering light sleep...");
            esp_sleep_enable_timer_wakeup(sleepTime_us);
            esp_light_sleep_start();
            xprintln("Woke up from light sleep.");
            // lasttime will be updated after this block.
        }
        lasttime = millis(); // Update lasttime after potential sleep or if sleep was skipped
    }

    // --- Post-Wake / Normal Operation ---

    // **Step 2: Ensure WiFi is reconnected**
    // The Provisioning system (SysProvEvent) should handle WiFi disconnections and reconnections.
    // We just check the status here.
    xprintln("Checking WiFi connection status...");
    if (WiFi.status() != WL_CONNECTED) {
        xprintln("WiFi is not connected. Waiting for Provisioning system or automatic reconnection...");
        int wifi_check_retries = 0;
        while(WiFi.status() != WL_CONNECTED && wifi_check_retries < 10) { // Check for up to 5 seconds
            delay(500);
            xprint(".");
            wifi_check_retries++;
        }
        xprintln("");
        if (WiFi.status() != WL_CONNECTED) {
            xprintln("WiFi still not connected. Azure IoT Hub operations will likely fail or be skipped.");
        } else {
            xprintln("WiFi reconnected or was already connected.");
        }
    } else {
        xprintln("WiFi is connected.");
    }

    // **Step 3: Re-initialize Azure IoT Hub client if necessary**
    // The previous call to IoTHubClient_LL_Destroy (if it happened) means the client needs to be set up again.
    // The check !iotHub->isAzureIoTConnected() along with the fixes to setupAzureIoTClient (to destroy old handle)
    // should manage this.
    if (WiFi.status() == WL_CONNECTED) {
        // ADDED: Disable WiFi modem power save mode
        xprintln("Disabling WiFi power save mode (setting to WIFI_PS_NONE)...");
        esp_wifi_set_ps(WIFI_PS_NONE);

        xprintln("Re-synchronizing NTP time after wake-up...");
        iotHub->syncTimeNTP();

        xprintln("Short delay for network stack stabilization (1000ms)...");
        delay(1000);

        if (!iotHub->isAzureIoTConnected()) {
            xprintln("Azure IoT Hub is NOT connected post-wake. Setting up Azure IoT client...");
            if (!iotHub->setupAzureIoTClient()) {
                xprintln("Failed to set up Azure IoT client post-wake. Will retry in the next cycle.");
            }
        }

        // Optional: If you want to re-enable power saving for WiFi during the active period
        // (before the next light sleep cycle), you could call esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        // here or just before calling esp_light_sleep_start(). However, for testing,
        // it's simpler to leave it disabled first to see if it impacts the connection.
        // If it works, you can then decide on a strategy for re-enabling it.

    } else {
        xprintln("WiFi not connected, skipping Azure IoT Hub client setup.");
    }


    // --- Collect Sensor Data ---
    #ifndef TESTINGMODE
    MainData *data = new MainData();
    if (!data) {
        xprintln("Error: Failed to allocate memory for MainData!");
        return; // Early exit
    }
    data->mpu_data = nullptr; // Initialize pointers
    data->max_data = nullptr;

    if(getTempAvail()){ //
        data->temp_in_c = readTempData(); //
    } else {
        xprintln("Temperature sensor not available.");
        data->temp_in_c = NAN; 
    }

    if(getMovAvail()){ //
        data->mpu_data = readMPUData(); //
    } else {
        xprintln("Movement sensor not available or no new movement data.");
    }
    
    #ifdef DEVMODE
    xprintln("Memory left after reading Movement."); //
    ESP32C3_Monitor::getInstance().printStatus(); //
    #endif
    
    if(max->isConnected()){ //
        data->max_data = max->runReading(); //
    } else {
        xprintln("MAX30102 sensor not available.");
    }
    
    #ifdef DEVMODE
    xprintln("Memory left after reading hr and spo."); //
    ESP32C3_Monitor::getInstance().printStatus(); //
    #endif
    
    data->distance_from_wifi_m = pow(10,((WIFIMEASUREDPOWER - WiFi.RSSI()) / (10.0f * PATHLOSSEXPONENT))); //
    data->battery_percentage = getBatteryCapacity(); //

    if(datastream){
        datastream->addData(data); //
    } else {
        xprintln("Data stream not initialized.");
        // Clean up allocated data if not added to stream
        if (data) {
            if(data->mpu_data) { 
                free(data->mpu_data->mean_ax_arr); free(data->mpu_data->mean_ay_arr);
                free(data->mpu_data->mean_az_arr); free(data->mpu_data->mean_mv_arr);
                free(data->mpu_data->std_mv_arr);
                delete data->mpu_data;
            }
            if(data->max_data) { 
                 delete data->max_data; 
            }
            delete data;
        }
    }
    #else // TESTINGMODE
    // If in testing mode, and you have specific logic, it goes here.
    // Potentially generate random data if not reading sensors.
    #endif

    ++readingcount; //
    
    // --- Process Telemetry ---
    // Ensure client handle is valid and connected before calling DoWork or sending.
    if (iotHub->getIotHubClientHandle() != nullptr) {
        if (iotHub->isAzureIoTConnected()) {
            if (readingcount >= readingitter) {
                xprintf("Reading count (%d) reached threshold (%d). Sending telemetry request...\n", readingcount, readingitter); //
                sendTelemetryRequest(); //
                readingcount = 0; //
                xprintln("Reading count reset to 0 after sending telemetry."); //
            } else {
                xprintf("Reading count (%d) has not reached threshold (%d). Not sending telemetry yet.\n", readingcount, readingitter); //
            }
        } else {
            xprintln("Azure IoT Hub not connected. Skipping telemetry send for this cycle.");
            // DoWork (called next) should attempt to process reconnections if setup was successful.
        }
        xprintln("Calling IoTHubClient_LL_DoWork..."); //
        IoTHubClient_LL_DoWork(iotHub->getIotHubClientHandle()); //
    } else {
        xprintln("Azure IoT Hub client handle is NULL post-wake. Cannot call DoWork or send telemetry this cycle.");
    }

    #ifdef DEVMODE
    xprintln("Data pulled successfully."); //
    ESP32C3_Monitor::getInstance().printStatus(); //
    #endif

    vTaskDelay(50 / portTICK_PERIOD_MS); 
}

void Sensors_Processes::sendTelemetryRequest(){
    #ifndef TESTINGMODE
    char *jsonbuffer = parsingDatastreamToJson("Data read successfully");
    #else
    char *jsonbuffer = generateRandomJson("Testing data transmission");
    #endif

    if(IOTHubInstance::getInstance()->sendJsonToAzure(jsonbuffer)){
        datastream->clearData(); // Clear the data stream after sending
        xprintln("Telemetry data sent successfully.");
    }

    free(jsonbuffer); // Free the allocated memory for JSON buffer
}


char* Sensors_Processes::parsingDatastreamToJson(char *message) {
    if (datastream == nullptr || datastream->getSize() == 0) {
        // If the datastream is empty, return a minimal JSON with just the message
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
        nodeObj["message"] = message ? message : "No message provided";

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

char* Sensors_Processes::generateRandomJson(const char* message) {
    // Create a JSON document
    JsonDocument doc;
    JsonObject root = doc.to<JsonObject>();

    // Add the message
    root["message"] = message ? message : "No message provided";

    // Add 3 random float fields
    root["random_field_1"] = serialized(String(random(0, 10000) / 100.0f, 2));
    root["random_field_2"] = serialized(String(random(0, 10000) / 100.0f, 2));
    root["random_field_3"] = serialized(String(random(0, 10000) / 100.0f, 2));

    // Add an array with 5 random floats
    JsonArray arr = root["random_array"].to<JsonArray>();
    for (int i = 0; i < 5; ++i) {
        arr.add(serialized(String(random(0, 10000) / 100.0f, 2)));
    }

    // Serialize to string
    String outputJsonString;
    serializeJson(doc, outputJsonString);

    // Allocate and copy to char*
    char* json_c_str = (char*)malloc(outputJsonString.length() + 1);
    if (json_c_str == nullptr) {
        xprintln("Failed to allocate memory for random JSON C-string!");
        return nullptr;
    }
    strcpy(json_c_str, outputJsonString.c_str());
    return json_c_str;
}