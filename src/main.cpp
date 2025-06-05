#include <Arduino.h>
#include <Tools.h>
#include <WiFi.h>
#include <config.h>
#include <freertos/task.h>
#include <Sensors_Processes.h>
#include <IOTHubInstance.h> // For Azure IoT Hub communication
#ifndef NO_PROVISIONING
#include <Provision.h> // Singleton class for provisioning & include UUID data
#else
#include <esp_wifi.h>
#include <secreets.h>
#include <ESP32C3Monitor.h> // For ESP32C3_Monitor class
#include <AzureIoTHub.h> // For Azure IoT Hub communication
#endif


void setup() {
  // #ifdef DEVMODE
    Serial.begin(115200);
  // #endif

  // let provision do it's thing and also setup for other connections (mqtt to Azure IoT Hub)
  #ifndef NO_PROVISIONING
  Provision::getInstance()->setupProvision();
  #else
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
  IOTHubInstance::getInstance()->syncTimeNTP();
  IOTHubInstance::getInstance()->setupAzureIoTClient();
  #endif
  // initiate setup for all the sensors first
  Sensors_Processes::getInstance()->setup();

  // sleep to let the sensors read datas first
  #ifndef TESTINGMODE
  xprintln("Sleeping for 1 minute to allow sensors to gather data a little...");
  xflush(); // Ensure all serial data is sent before sleeping
  esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP); //set to how many microsecond for one sleep
  esp_light_sleep_start();
  #endif
  Sensors_Processes::getInstance()->sendTelemetryRequest();
  IoTHubClient_LL_DoWork(IOTHubInstance::getInstance()->getIotHubClientHandle());
}

void loop(){
  Sensors_Processes::getInstance()->pullData();
}