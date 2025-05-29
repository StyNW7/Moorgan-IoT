#include <Arduino.h>
#include <Tools.h>          // For xprintln, xprint, xprintf etc. (if DEVMODE is enabled)
#include <WiFi.h>           // For WiFi.status()
#include <Provision.h>      // For WiFi provisioning and initial Azure IoT Hub setup
#include <IOTHubInstance.h> // For Azure IoT Hub communication
#include <Sensors_Processes.h> // For the generateRandomJson method
#include <config.h>         // For DEVMODE, telemetry interval, etc.

// Define a telemetry send interval (e.g., every 30 seconds).
// This can also be defined in config.h if preferred.
#ifndef TELEMETRY_INTERVAL_MS
#define TELEMETRY_INTERVAL_MS (30 * 1000UL) // 30 seconds, UL for unsigned long
#endif

unsigned long lastTelemetrySendTime = 0;

void setup() {
  // Initialize Serial communication if DEVMODE is enabled in config.h
  #ifdef DEVMODE
    Serial.begin(115200);
    // Optional: Wait for Serial to be ready to ensure initial messages are captured.
    // unsigned long serial_timeout_start = millis();
    // while (!Serial && (millis() - serial_timeout_start < 2000)) { // Wait up to 2 seconds
    //   delay(10);
    // }
    xprintln(F("----------------------------------------------------"));
    xprintln(F("Moorgan IoT - Provisioning & Azure IoT Hub Test Mode"));
    xprintln(F("----------------------------------------------------"));
    xprintln(F("Serial initialized (DEVMODE enabled)."));
  #endif

  xprintln(F("System Initializing..."));
  Provision::getInstance()->setupProvision();

  #ifdef DEVMODE
    xprintln(F("--- Post-Setup Status ---"));
    if (WiFi.status() == WL_CONNECTED) {
      xprint(F("WiFi Status: Connected. IP Address: "));
      xprintln(WiFi.localIP());
    } else {
      xprintln(F("WiFi Status: Not Connected. Provisioning might be active or encountered an issue."));
    }
    xprint(F("Azure IoT Hub Connection Status (initial check): "));
    xprintln(IOTHubInstance::getInstance()->isAzureIoTConnected() ? F("Connected") : F("Not Connected"));
    xprintln(F("-------------------------"));
  #endif

  xprintln(F("Setup phase complete. Entering main loop."));
}

void loop() {
  // --- Handle Azure IoT Hub Communication ---
  // `IoTHubClient_LL_DoWork()` is critical for the Azure IoT SDK. It processes the MQTT
  // message queue, handles keep-alives, connection retries, and message acknowledgments.
  // It must be called frequently when WiFi is connected.
  if (WiFi.status() == WL_CONNECTED) {
    if (IOTHubInstance::getInstance()->getIotHubClientHandle() != nullptr) {
      IoTHubClient_LL_DoWork(IOTHubInstance::getInstance()->getIotHubClientHandle());
    } else {
      // This state (WiFi connected, but IoT Hub client handle is null) suggests an issue
      // during `IOTHubInstance::setupAzureIoTClient()`. The Provision class's WiFi event
      // handlers might attempt to re-initialize it if connection events are re-triggered.
      #ifdef DEVMODE
        // xprintln(F("Warning: WiFi connected, but Azure IoT Hub client handle is NULL."));
      #endif
    }
  } else {
    // WiFi is not connected. The `Provision` class's event system (`SysProvEvent`)
    // is designed to handle reconnection attempts or re-initiate provisioning based on WiFi events.
    // A delay here prevents a busy loop if WiFi is persistently down, allowing background tasks.
    #ifdef DEVMODE
      // xprintln(F("Loop: WiFi is not connected. Waiting for Provisioning system..."));
    #endif
    delay(1000); // Pause for 1 second, allowing event handlers and other tasks to work.
    return;      // Skip telemetry sending if WiFi is down.
  }

  // --- Send Telemetry Data ---
  // Check if connected to Azure IoT Hub and if it's time to send new telemetry.
  if (IOTHubInstance::getInstance()->isAzureIoTConnected()) {
    if (millis() - lastTelemetrySendTime >= TELEMETRY_INTERVAL_MS) {
      #ifdef DEVMODE
        xprintln(F("INFO: Time to send dummy telemetry."));
      #endif

      // Generate random JSON data using the method from the Sensors_Processes class.
      // The message string included in the JSON is customizable here.
      char *jsonData = Sensors_Processes::getInstance()->generateRandomJson("Moorgan-IoT Test Telemetry");

      if (jsonData != nullptr) {
        #ifdef DEVMODE
          xprint(F("INFO: Generated JSON: "));
          xprintln(jsonData);
        #endif
        // Attempt to send the JSON data to Azure IoT Hub.
        if (IOTHubInstance::getInstance()->sendJsonToAzure(jsonData)) {
          #ifdef DEVMODE
            xprintln(F("INFO: Dummy telemetry data successfully enqueued for Azure IoT Hub."));
          #endif
        } else {
          #ifdef DEVMODE
            xprintln(F("ERROR: Failed to enqueue dummy telemetry data for Azure IoT Hub."));
          #endif
        }
        // Free the memory allocated by `generateRandomJson`.
        free(jsonData);
        jsonData = nullptr; // Best practice to avoid using a dangling pointer.
      } else {
        #ifdef DEVMODE
          xprintln(F("ERROR: Failed to generate random JSON data."));
        #endif
      }
      lastTelemetrySendTime = millis(); // Update the timestamp for the last send attempt.
    }
  } else {
    // WiFi might be connected, but Azure IoT Hub is not.
    // `IoTHubClient_LL_DoWork()` (called above) is responsible for handling reconnection attempts.
    #ifdef DEVMODE
      // if (WiFi.status() == WL_CONNECTED) {
      //   xprintln(F("INFO: WiFi connected, but Azure IoT Hub is not. Waiting for DoWork to reconnect..."));
      // }
    #endif
  }

  // A short delay in the main loop is generally good practice. It allows other
  // lower-priority FreeRTOS tasks (if any) to run and prevents the loop from
  // consuming 100% CPU if main tasks are infrequent.
  // The frequency of `DoWork` calls is important for MQTT responsiveness.
  delay(100); // e.g., call DoWork and check telemetry status roughly 10 times per second.
}