#include <Arduino.h>
#include <Tools.h>
#include <WiFi.h>
#include <config.h>
#include <freertos/task.h>
#include <Sensors_Processes.h>
#include <Provision.h> // Singleton class for provisioning & include UUID data

void dataPullingTask(void *pvParameter);

void loop(){}

void setup() {
  #ifdef DEVMODE
    Serial.begin(115200);
  #endif

  // let provision do it's thing and also setup for other connections (mqtt to Azure IoT Hub)
  Provision::getInstance()->setupProvision();
  // initiate setup for all the sensors first
  Sensors_Processes::getInstance()->setup();

  // sleep to let the sensors read datas first
  xflush(); // Ensure all serial data is sent before sleeping
  esp_sleep_enable_timer_wakeup(MICROSECOND_SLEEP); //set to how many microsecond for one sleep
  esp_light_sleep_start();

  xTaskCreate(
    dataPullingTask,
    "Data Pulling Task",
    1024*5,
    Sensors_Processes::getInstance(),
    5,
    NULL
  );
}

void dataPullingTask(void *pvParameter){
  Sensors_Processes *self = static_cast<Sensors_Processes*>(pvParameter);
  self->pullData();
}