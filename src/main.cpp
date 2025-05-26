#include <Arduino.h>
#include <Tools.h>
#include <WiFi.h>
#include <config.h>
#include <freertos/task.h>
#include <Sensors_Processes.h>
#include <Provision.h> // Singleton class for provisioning & include UUID data

void loop(){}

void setup() {
  #ifdef DEVMODE
    Serial.begin(115200);
  #endif

  Provision::getInstance()->setupProvision();
  Sensors_Processes::getInstance()->setup();


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