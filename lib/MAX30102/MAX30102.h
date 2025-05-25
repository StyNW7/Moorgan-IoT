#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <config.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

typedef struct{
    float heart_rate;
    float oxygen;
}MAXData;

struct ppgSample {
    uint32_t ir;
    uint32_t red;
};

class MAX30102 {
public:
    ppgSample ppgDataBuffer[PPG_BUFFER_SIZE];
    uint16_t validSamplesCount;
    SemaphoreHandle_t ppgDataNotificationSemaphore;

    static MAX30102* instance;

    MAX30102();

    void setup();
    void runReading();
    void ppgSensorTask();
    void interruptServiceRoutine();

    bool writeRegister(uint8_t reg_addr, uint8_t value);
    bool readRegister(uint8_t reg_addr, uint8_t *value);
    bool readBurst(uint8_t start_reg_addr, uint8_t *buffer, uint8_t count);

    bool initializeForSpotCheck();
    void shutdownSensor();
    uint16_t collectPPGData(TickType_t total_duration_ticks, TickType_t discard_duration_ticks);

    float calculateHeartRate(const ppgSample samples[], uint16_t num_samples);
    float calculateSpO2(const ppgSample samples[], uint16_t num_samples);

    static void IRAM_ATTR isrWrapper();
    static void taskWrapper(void* pvParameters);
};