#include <Arduino.h>
#include <Tools.h>
#include <WiFi.h>
#include <config.h>
#include <FreeRTOS/task.h>
#include <Provision.h> // Singleton class for provisioning & include UUID data

void loop(){}

void setup() {
  #ifdef DEVMODE
    Serial.begin(115200);
  #endif

  Provision::getInstance()->setupProvision();
}