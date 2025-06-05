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
#include <Provision.h> // This will not be compiled if NO_PROVISIONING is defined in config.h by preprocessor
#else
#include <esp_wifi.h> // For esp_wifi_set_ps
#include <secreets.h> // For WIFI_SSID, WIFI_PASSWORD
#endif

// RTC Variable Definitions
RTC_DATA_ATTR bool rtc_is_first_boot = true; // Can be used for other one-time initializations if needed
RTC_DATA_ATTR int rtc_telemetry_reading_count = 0;
// RTC_DATA_ATTR bool rtc_is_provisioned = false; // This flag is not strictly needed when NO_PROVISIONING is defined

// Define a shorter sleep interval for failures, if desired
#define MICROSECOND_SLEEP_SHORT_FAILURE (30ULL * 1000000ULL) // 30 seconds, example

void setup() {
    Serial.begin(115200);
    xprintln("\n\n===================================");
    xprintln("Moorgan-IoT: Booting up / Woke from Deep Sleep...");
    if (rtc_is_first_boot) {
        xprintln("Device is performing its first boot sequence after power-on or flash.");
        // Perform any other true one-time initializations here
        rtc_is_first_boot = false;
    }
    xprint("Device UUID (from config): "); xprintln(PRE_REGISTERED_DEVICE_UUID);
    xprintln("===================================");

#ifndef NO_PROVISIONING
    // This block is skipped because NO_PROVISIONING is defined in your config.h
    xprintln("Provisioning enabled path (should be skipped).");
    Provision::getInstance()->setupProvision(); 
    // ... (other provisioning logic)
#else 
    // This block is EXECUTED because NO_PROVISIONING is defined
    xprintln("NO_PROVISIONING defined: Attempting WiFi connection with predefined credentials from secreets.h...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif

    xprint("Attempting to connect to WiFi");
    int wifi_connect_retries = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_connect_retries < 20) { // Retry for ~10 seconds
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

        IOTHubInstance::getInstance()->syncTimeNTP();
        if (!IOTHubInstance::getInstance()->setupAzureIoTClient()) {
            xprintln("FATAL: Failed to setup Azure IoT Client. Going to deep sleep.");
            esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP_SHORT_FAILURE);
            esp_deep_sleep_start();
        } else {
            xprintln("Azure IoT Client setup successfully.");
        }
    } else {
        xprintln("FATAL: Failed to connect to WiFi. Going to deep sleep.");
        esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP_SHORT_FAILURE);
        esp_deep_sleep_start();
    }

    Sensors_Processes::getInstance()->setup();
    xprintln("Sensors setup complete.");
    xprintln("-----------------------------------");
    xprintln("Setup phase complete. Proceeding to main logic cycle.");
    xprintln("-----------------------------------");
}

void loop() {
    xprintln("\n>>> Starting main logic cycle <<<");

    if (WiFi.status() == WL_CONNECTED && IOTHubInstance::getInstance()->isAzureIoTConnected()) {
        Sensors_Processes::getInstance()->performDataCycle(&rtc_telemetry_reading_count);

        if (IOTHubInstance::getInstance()->getIotHubClientHandle() != nullptr) {
            xprintln("Calling IoTHubClient_LL_DoWork()...");
            IoTHubClient_LL_DoWork(IOTHubInstance::getInstance()->getIotHubClientHandle());
        } else {
            xprintln("Warning: Azure IoT Hub client handle is NULL in loop. Cannot call DoWork.");
        }
        xprintln("Data cycle and DoWork completed.");

    } else {
        xprintln("WiFi or Azure IoT Hub not connected at start of loop.");
        xprintln("Skipping data cycle. Will enter deep sleep and retry setup on wake.");
    }

    xprint("Preparing for deep sleep. Sleep duration: ");
    xprint(MICROSECOND_SLEEP / 1000000ULL);
    xprintln(" seconds.");
    
    // Optional: Cleanly destroy the Azure client handle before sleeping.
    // If "host not found" was an issue, uncommenting this might help,
    // though deep sleep's full reset should generally handle this.
    // IOTHubInstance* iotHub = IOTHubInstance::getInstance();
    // if (iotHub && iotHub->getIotHubClientHandle()) {
    //    iotHub->prepareForSleep();
    //    xprintln("Azure IoT Hub client prepared for sleep (handle potentially destroyed).");
    // }


    xprintln(">>> End of main logic cycle. Entering deep sleep. <<<");
    xflush(); 

    esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP);
    esp_deep_sleep_start();
}