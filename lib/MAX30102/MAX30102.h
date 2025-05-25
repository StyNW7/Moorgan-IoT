#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <config.h> // Make sure PPG_BUFFER_SIZE, MAX30102_INT_PIN etc. are here or defined appropriately
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Define MAXData struct if it's primarily used with MAX30102 and not a general data type
// If it's general, it might belong in its own header or a more general one.
typedef struct {
    float heart_rate;
    float oxygen;
} MAXData;

// Define ppgSample struct here, as it's part of the MAX30102 class's public interface (ppgDataBuffer)
// and used in method parameters (calculateHeartRate, calculateSpO2)
struct ppgSample {
    uint32_t ir;
    uint32_t red;
};

class MAX30102 {
public:
    // Member Variables
    MAXData *dataframetofill; // Add this member
    ppgSample ppgDataBuffer[PPG_BUFFER_SIZE];
    uint16_t validSamplesCount; // Initialize in constructor or rely on default (0 for numeric types if static/global)
    SemaphoreHandle_t ppgDataNotificationSemaphore; // Initialize in constructor

    static MAX30102* instance;

    // Constructor
    MAX30102();
    // It's good practice to have a destructor if you allocate resources (like semaphores)
    ~MAX30102(); // Consider adding if semaphore needs explicit deletion

    // Methods
    void setup();
    void runReading(MAXData *dataframe); // Ensure signature matches .cpp
    void ppgSensorTask();
    void interruptServiceRoutine();

    bool writeRegister(uint8_t reg_addr, uint8_t value);
    bool readRegister(uint8_t reg_addr, uint8_t *value);
    bool readBurst(uint8_t start_reg_addr, uint8_t *buffer, uint8_t count);

    bool isConnected();
    bool initializeForSpotCheck();
    void shutdownSensor();
    uint16_t collectPPGData(TickType_t total_duration_ticks, TickType_t discard_duration_ticks);

    float calculateHeartRate(const ppgSample samples[], uint16_t num_samples);
    float calculateSpO2(const ppgSample samples[], uint16_t num_samples);

    // Static methods for ISR and task wrapper
    static void IRAM_ATTR isrWrapper();
    static void taskWrapper(void* pvParameters);
};