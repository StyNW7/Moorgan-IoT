// ESP32C3_Monitor.cpp
#include "ESP32C3Monitor.h"
#include <Tools.h>

/**
 * @brief Constructor for ESP32C3_Monitor.
 * Currently does not perform any specific initialization.
 */
ESP32C3_Monitor::ESP32C3_Monitor() {
    // Initialization code can go here if needed in the future
}

/**
 * @brief Prints the current memory status and CPU frequency to the Serial monitor.
 * It displays total heap, free heap, used heap (with percentage), and CPU frequency in MHz.
 */
void ESP32C3_Monitor::printStatus() {
    // --- Memory Information ---
    // Get total heap memory available for general purpose allocations (MALLOC_CAP_DEFAULT)
    uint32_t totalHeap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    
    // Get current free heap memory
    // esp_get_free_heap_size() is a convenient way to get the free heap for MALLOC_CAP_DEFAULT.
    // Alternatively, heap_caps_get_free_size(MALLOC_CAP_DEFAULT) can be used.
    uint32_t freeHeap = esp_get_free_heap_size(); 

    uint32_t usedHeap = totalHeap - freeHeap;
    float memoryUsagePercentage = 0.0f;
    if (totalHeap > 0) {
        memoryUsagePercentage = ((float)usedHeap / totalHeap) * 100.0f;
    }

    // Print memory information to Serial
    xprintln(F("--- Memory Status ---")); // F() macro saves RAM by storing string in Flash

    xprint(F("Total Heap: "));
    xprint(totalHeap);
    xprintln(F(" bytes"));

    xprint(F("Free Heap:  "));
    xprint(freeHeap);
    xprintln(F(" bytes"));
    
    xprint(F("Used Heap:  "));
    xprint(usedHeap);
    xprint(F(" bytes ("));
    xprintf("%.2f", memoryUsagePercentage); // Print percentage with 2 decimal places
    xprintln(F("%)"));

    // --- CPU Frequency ---
    // esp_clk_cpu_freq() returns the CPU frequency in Hz.
    // Divide by 1,000,000 to convert to MHz.
    uint32_t cpuFreqHz = esp_clk_cpu_freq();
    uint32_t cpuFreqMHz = cpuFreqHz / 1000000; 

    // Print CPU frequency information to Serial
    xprintln(F("--- CPU Status ---"));
    xprint(F("CPU Frequency: "));
    xprint(cpuFreqMHz);
    xprintln(F(" MHz"));
    xprintln(F("---------------------"));
}

uint32_t ESP32C3_Monitor::getFreeHeap() {
    return esp_get_free_heap_size();
}

uint32_t ESP32C3_Monitor::getTotalHeap() {
    return heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
}

uint32_t ESP32C3_Monitor::getCpuFrequencyMHz() {
    return esp_clk_cpu_freq() / 1000000;
}
