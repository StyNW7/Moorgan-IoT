#include <Arduino.h>
#include <Tools.h>
#include <WiFi.h>
#include <config.h> // Ensures NO_PROVISIONING and MICROSECOND_SLEEP are included
#include <freertos/task.h>
#include <Sensors_Processes.h>
#include <IOTHubInstance.h>
#include <esp_sleep.h> // Required for deep sleep

// Conditional includes based on NO_PROVISIONING
#ifndef NO_PROVISIONING
#include <Provision.h> 
#else
#include <esp_wifi.h> 
#include <secreets.h> 
#endif

// RTC Variable Definitions
RTC_DATA_ATTR bool rtc_is_first_boot = true; 
RTC_DATA_ATTR int rtc_telemetry_reading_count = 0;

#define MICROSECOND_SLEEP_SHORT_FAILURE (30ULL * 1000000ULL) 

void setup() {
    Serial.begin(115200);
    xprintln("\n\n===================================");
    xprintln("Moorgan-IoT: Booting up / Woke from Deep Sleep...");
    
    bool is_wake_up = false;
    if (rtc_is_first_boot) {
        xprintln("Device is performing its first boot sequence after power-on or flash.");
        rtc_is_first_boot = false; // Clear the flag for subsequent boots (which will be wake-ups)
        is_wake_up = false;
    } else {
        xprintln("Device woke up from deep sleep.");
        is_wake_up = true;
    }
    xprint("Device UUID (from config): "); xprintln(PRE_REGISTERED_DEVICE_UUID);
    xprintln("===================================");

#ifndef NO_PROVISIONING
    xprintln("Provisioning enabled path (should be skipped).");
#else 
    xprintln("NO_PROVISIONING defined: Attempting WiFi connection with predefined credentials from secreets.h...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif

    xprint("Attempting to connect to WiFi");
    int wifi_connect_retries = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_connect_retries < 20) { 
        delay(500);
        xprint(".");
        wifi_connect_retries++;
    }
    xprintln("");

    if (WiFi.status() == WL_CONNECTED) {
        xprintln("WiFi connected!");
        xprint("IP Address: ");
        xprintln(WiFi.localIP());
        
        esp_wifi_set_ps(WIFI_PS_NONE);
        xprintln("WiFi power save mode disabled (WIFI_PS_NONE).");

        IOTHubInstance* iotHub = IOTHubInstance::getInstance(); 
        iotHub->syncTimeNTP();

        if (!iotHub->setupAzureIoTClient()) {
            xprintln("FATAL: Failed to initiate Azure IoT Client setup. Going to deep sleep.");
            esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP_SHORT_FAILURE);
            esp_deep_sleep_start();
        } else {
            xprintln("Azure IoT Client setup successfully initiated.");
            xprintln("Attempting to establish Azure IoT Hub connection (will try for up to 15 seconds)...");
            
            unsigned long azureConnectStartTime = millis();
            bool connectedToAzureInSetup = false;

            while (millis() - azureConnectStartTime < 15000) { 
                if (iotHub->getIotHubClientHandle() != nullptr) {
                    IoTHubClient_LL_DoWork(iotHub->getIotHubClientHandle());
                } else {
                    xprintln("Error: IoT Hub client handle became null during connection attempt.");
                    break; 
                }

                if (iotHub->isAzureIoTConnected()) {
                    xprintln("Successfully connected to Azure IoT Hub in setup!");
                    connectedToAzureInSetup = true;
                    break;
                }
                delay(500); 
                xprint(">"); 
            }
            xprintln(""); 

            if (!connectedToAzureInSetup) {
                xprintln("Warning: Failed to connect to Azure IoT Hub within the timeout in setup.");
            }
        }
    } else {
        xprintln("FATAL: Failed to connect to WiFi. Going to deep sleep.");
        esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP_SHORT_FAILURE);
        esp_deep_sleep_start();
    }

    // Pass 'is_wake_up' to the sensor setup
    Sensors_Processes::getInstance()->setup(is_wake_up);
    xprintln("Sensors setup complete.");
    xprintln("-----------------------------------");
    xprintln("Setup phase complete. Proceeding to main logic cycle.");
    xprintln("-----------------------------------");
}

void loop() {
    xprintln("\n>>> Starting main logic cycle <<<");
    IOTHubInstance* iotHub = IOTHubInstance::getInstance(); 
    bool messageWasSentThisCycle = false; 

    if (WiFi.status() == WL_CONNECTED && iotHub->isAzureIoTConnected()) {
        messageWasSentThisCycle = Sensors_Processes::getInstance()->performDataCycle(&rtc_telemetry_reading_count);

        if (iotHub->getIotHubClientHandle() != nullptr) {
            xprintln("Calling IoTHubClient_LL_DoWork() after data cycle...");
            IoTHubClient_LL_DoWork(iotHub->getIotHubClientHandle()); 

            if (messageWasSentThisCycle) {
                xprintln("Telemetry send was attempted this cycle. Running DoWork for up to 5 seconds for confirmation...");
                unsigned long postSendDoWorkStart = millis();
                int doWorkLoops = 0;
                while (millis() - postSendDoWorkStart < 5000) { 
                    IoTHubClient_LL_DoWork(iotHub->getIotHubClientHandle());
                    delay(200); 
                    doWorkLoops++;
                }
                xprintf("Finished extended DoWork after sending (ran %d times).\n", doWorkLoops);
                xprintln("Check logs for 'Telemetry send confirmation: OK'");
            }
        } else {
            xprintln("Warning: Azure IoT Hub client handle is NULL in loop. Cannot call DoWork.");
        }
        xprintln("Data cycle and DoWork processing completed.");

    } else {
        xprint("Condition check in loop: WiFi Connected = ");
        xprint((WiFi.status() == WL_CONNECTED) ? "true" : "false");
        xprint(", Azure IoT Hub Connected = ");
        xprintln(iotHub->isAzureIoTConnected() ? "true" : "false");
        xprintln("WiFi or Azure IoT Hub not connected at start of loop.");
        xprintln("Skipping data cycle. Will enter deep sleep and retry setup on wake.");
    }

    xprint("Preparing for deep sleep. Sleep duration: ");
    xprint(MICROSECOND_SLEEP / 1000000ULL);
    xprintln(" seconds.");
    
    xprintln(">>> End of main logic cycle. Entering deep sleep. <<<");
    xflush(); 

    esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP);
    esp_deep_sleep_start();
}