#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <config.h> // Make sure PPG_BUFFER_SIZE, MAX30102_INT_PIN etc. are here
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// MAXData struct is used to return the results of the calculation
typedef struct {
    float heart_rate;
    float oxygen;
} MAXData;

// ppgSample struct holds the raw data from the sensor
struct ppgSample {
    uint32_t ir;
    uint32_t red;
};

class MAX30102 {
public:
    // Member Variables
    ppgSample ppgDataBuffer[PPG_BUFFER_SIZE];
    uint16_t validSamplesCount;
    SemaphoreHandle_t ppgDataNotificationSemaphore;
    uint8_t readingscount;
    float last_heart_rate;
    float last_oxygen_saturation;

    static MAX30102* instance;

    // Constructor & Destructor
    MAX30102();
    ~MAX30102();

    // Core Methods
    void setup();
    MAXData* runReading();
    void ppgSensorTask(MAXData* dataframe);
    bool isSensorProcessing();

    // Helper & Register Methods
    bool writeRegister(uint8_t reg_addr, uint8_t value);
    bool readRegister(uint8_t reg_addr, uint8_t *value);
    bool readBurst(uint8_t start_reg_addr, uint8_t *buffer, uint8_t count);
    bool isConnected();
    bool initializeForSpotCheck();
    void shutdownSensor();
    uint16_t collectPPGData(TickType_t total_duration_ticks, TickType_t discard_duration_ticks);
    
    // --- UPDATED CALCULATION METHOD ---
    // This new function will calculate both HR and SpO2 and handle signal quality.
    void calculateHRAndSpO2(const ppgSample samples[], uint16_t num_samples, float& out_hr, float& out_spo2);

    // Static methods for ISR and task wrapper
    static void IRAM_ATTR isrWrapper();
};