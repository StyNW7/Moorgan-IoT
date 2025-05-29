#pragma once

#ifndef ESP32C3_MONITOR_H
#define ESP32C3_MONITOR_H

// Standard Arduino include
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h" // For older Arduino versions
#endif

// ESP-IDF includes for ESP32 specific functionalities
#include "esp_system.h"    // For esp_get_free_heap_size()
#include "esp_heap_caps.h" // For heap_caps_get_total_size(), heap_caps_get_free_size()
#include "esp32/clk.h"    // For esp_clk_cpu_freq()

/**
 * @class ESP32C3_Monitor
 * @brief A simple class to monitor and print memory usage and CPU frequency on ESP32-C3.
 */
class ESP32C3_Monitor {
public:
    /**
     * @brief Gets the single instance of the ESP32C3_Monitor.
     * @return Reference to the singleton instance.
     */
    static ESP32C3_Monitor& getInstance();

    /**
     * @brief Prints the current memory status (total, free, used heap) and CPU frequency
     * to the Serial monitor.
     */
    void printStatus();

    /**
     * @brief Gets the current free heap size.
     * @return Free heap size in bytes.
     */
    uint32_t getFreeHeap();

    /**
     * @brief Gets the total heap size available for default allocations.
     * @return Total heap size in bytes.
     */
    uint32_t getTotalHeap();

    /**
     * @brief Gets the current CPU frequency.
     * @return CPU frequency in MHz.
     */
    uint32_t getCpuFrequencyMHz();

    // Delete copy constructor and assignment operator
    ESP32C3_Monitor(const ESP32C3_Monitor&) = delete;
    ESP32C3_Monitor& operator=(const ESP32C3_Monitor&) = delete;

private:
    /**
     * @brief Private constructor for ESP32C3_Monitor.
     */
    ESP32C3_Monitor();
    
    static ESP32C3_Monitor instance; // Static instance
};

#endif // ESP32C3_MONITOR_H