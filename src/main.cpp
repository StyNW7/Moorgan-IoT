#include <Arduino.h>
#include <Tools.h>
#include <WiFi.h>
#include <config.h>
#include <freertos/task.h>
#include <Sensors_Processes.h>
#ifndef NO_PROVISIONING
#include <Provision.h> // Singleton class for provisioning & include UUID data
#else
#include <secreets.h>
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
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
  #endif
  // initiate setup for all the sensors first
  Sensors_Processes::getInstance()->setup();

  // sleep to let the sensors read datas first
  xprintln("Sleeping for 1 minute to allow sensors to gather data a little...");
  xflush(); // Ensure all serial data is sent before sleeping
  esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP); //set to how many microsecond for one sleep
  esp_light_sleep_start();
}

void loop(){
  Sensors_Processes::getInstance()->pullData();
}