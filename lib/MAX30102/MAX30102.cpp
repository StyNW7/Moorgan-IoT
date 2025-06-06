#include <Wire.h>
#include <Tools.h>
#include <config.h>
#include <MAX30102.h> // Include the header file
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Register definitions (these are fine here or could be in the .h if generally useful)
#define MAX30102_ADDRESS            0x57
#define MAX30102_REG_INT_STATUS_1   0x00
#define MAX30102_REG_INT_STATUS_2   0x01
#define MAX30102_REG_INT_ENABLE_1   0x02
#define MAX30102_REG_INT_ENABLE_2   0x03
#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_OVF_COUNTER    0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07
#define MAX30102_REG_FIFO_CONFIG    0x08
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A
#define MAX30102_REG_LED1_PA        0x0C // Red LED
#define MAX30102_REG_LED2_PA        0x0D // IR LED
#define MAX30102_REG_MULTI_LED_CTRL1 0x11
#define MAX30102_REG_MULTI_LED_CTRL2 0x12
#define MAX30102_REG_PART_ID        0xFF

// DO NOT redefine ppgSample here
// DO NOT redefine class MAX30102 here

// Static member definition
MAX30102* MAX30102::instance = nullptr;

// Constructor
MAX30102::MAX30102() {
    instance = this;
    validSamplesCount = 0;
    ppgDataNotificationSemaphore = NULL; // Will be created in setup
    readingscount = 255; // Initialize reading count
    last_heart_rate = 0.0f; // Initialize last heart rate
    last_oxygen_saturation = 0.0f; // Initialize last oxygen saturation
}


// Setup method - Definition
void MAX30102::setup() {
    ppgDataNotificationSemaphore = xSemaphoreCreateBinary();
    if (ppgDataNotificationSemaphore == NULL) {
        xprintln("Fatal Error: Failed to create PPG data semaphore!");
        // Consider how to handle this - maybe a status flag in the class
        return;
    }
    // Ensure MAX30102_INT_PIN is defined (e.g., in config.h)
    pinMode(MAX30102_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MAX30102_INT_PIN), isrWrapper, FALLING);
    xprintln("MAX30102: Main setup complete");
}

bool MAX30102::isSensorProcessing() {
    // If the semaphore is not NULL, assume processing is ongoing
    return ppgDataNotificationSemaphore != NULL;
}

// runReading method - Definition
MAXData *MAX30102::runReading() {
    MAXData *data = (MAXData *)malloc(sizeof(MAXData));
    
    if (data == nullptr) {
        xprintln("Error: Failed to allocate memory for MAXData in runReading!");
        static MAXData errorData = { -1.0f, -1.0f }; // Or some other error indicator values
        return &errorData;
    }

    if(readingscount >= PPG_READ_EVERY){
        ppgSensorTask(data);
        return data;
    } else {
        ++readingscount;
        data->heart_rate = last_heart_rate;
        data->oxygen = last_oxygen_saturation;
        return data;
    }
}

// ppgSensorTask method - Definition
void MAX30102::ppgSensorTask(MAXData * dataframe) {
    xprintln("Task: PPG Sensor Task Started.");
    if (!initializeForSpotCheck()) {
        xprintln("Task: MAX30102 setup failed.");
        if (dataframe != nullptr) { // Check if dataframetofill is valid
            dataframe->heart_rate = last_heart_rate;
            dataframe->oxygen = last_oxygen_saturation;
        }
        return;
    }
    // Ensure TOTAL_OPERATION_DURATION_MS and DISCARD_DURATION_MS are defined
    validSamplesCount = collectPPGData(
        pdMS_TO_TICKS(TOTAL_OPERATION_DURATION_MS),
        pdMS_TO_TICKS(DISCARD_DURATION_MS)
    );
    shutdownSensor();

    if (validSamplesCount > 0) {
        xprintln("\n--- Data Processing ---");
        // Optional: Print a few samples for debugging
        float heart_rate_bpm = 0;
        float oxygen_saturation_percent = 0;
        calculateHRAndSpO2(ppgDataBuffer, validSamplesCount, heart_rate_bpm, oxygen_saturation_percent);
        
        if (dataframe != nullptr) { // Check if dataframetofill is valid
            dataframe->heart_rate = heart_rate_bpm;
            dataframe->oxygen = oxygen_saturation_percent;
        } else {
            xprintln("Task: dataframetofill is null, only store to last_HR and last_oxy reading variable.");
        }

        last_heart_rate = heart_rate_bpm;
        last_oxygen_saturation = oxygen_saturation_percent;

        xprintf("\nEstimated Heart Rate: %.1f BPM\n", heart_rate_bpm);
        xprintf("Estimated SpO2: %.1f %%\n", oxygen_saturation_percent);
    } else {
        xprintln("Task: No valid PPG samples were collected.");
    }
    xprintln("\nTask: PPG Sensor Task finished its cycle.");
}

// writeRegister method - Definition
bool MAX30102::writeRegister(uint8_t reg_addr, uint8_t value) {
    Wire.beginTransmission(MAX30102_ADDRESS);
    Wire.write(reg_addr);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

// readRegister method - Definition
bool MAX30102::readRegister(uint8_t reg_addr, uint8_t *value) {
    Wire.beginTransmission(MAX30102_ADDRESS);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) {
        xprintln("I2C: Failed to send reg addr for read");
        return false;
    }
    if (Wire.requestFrom((uint8_t)MAX30102_ADDRESS, (uint8_t)1) == 1) {
        *value = Wire.read();
        return true;
    }
    xprintln("I2C: Failed to read byte");
    return false;
}

// readBurst method - Definition
bool MAX30102::readBurst(uint8_t start_reg_addr, uint8_t *buffer, uint8_t count) {
    Wire.beginTransmission(MAX30102_ADDRESS);
    Wire.write(start_reg_addr);
    if (Wire.endTransmission(false) != 0) {
        xprintln("I2C: Failed to send reg addr for burst read");
        return false;
    }
    if (Wire.requestFrom((uint8_t)MAX30102_ADDRESS, count) == count) {
        for (uint8_t i = 0; i < count; i++) {
            buffer[i] = Wire.read();
        }
        return true;
    }
    xprintln("I2C: Failed to read burst bytes");
    return false;
}

// initializeForSpotCheck method - Definition
bool MAX30102::initializeForSpotCheck() {
    uint8_t temp_val;
    xprintln("Initializing MAX30102 for spot check (50SPS, 4x Avg)..."); // Ensure SAMPLES_PER_SECOND matches this if used elsewhere
    if (!readRegister(MAX30102_REG_PART_ID, &temp_val) || temp_val != 0x15) {
        xprintln("MAX30102: Not found or wrong Part ID.");
        return false;
    }
    xprintln("MAX30102: Part ID OK.");
    if (!writeRegister(MAX30102_REG_MODE_CONFIG, 0x40)) { // Reset
        xprintln("MAX30102: Reset failed."); return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset
    uint32_t startTime = millis();
    do { // Wait for reset bit to clear
        if (!readRegister(MAX30102_REG_MODE_CONFIG, &temp_val)) return false;
        if (millis() - startTime > 500) { xprintln("MAX30102: Reset timeout."); return false; }
    } while (temp_val & 0x40);
    xprintln("MAX30102: Soft reset complete.");

    // Clear any pending interrupts
    readRegister(MAX30102_REG_INT_STATUS_1, &temp_val);
    readRegister(MAX30102_REG_INT_STATUS_2, &temp_val);

    // FIFO Config: sample averaging = 4, FIFO rolls on full, FIFO almost full = 17 (means 15 samples free)
    if (!writeRegister(MAX30102_REG_FIFO_CONFIG, 0x5F)) { // 0b01011111: SMP_AVE=4, FIFO_ROLLOVER_EN=1, FIFO_A_FULL=15
        xprintln("MAX30102: FIFO config failed."); return false;
    }
    // Interrupt Enable 1: A_FULL_EN = 1 (FIFO Almost Full Flag)
    if (!writeRegister(MAX30102_REG_INT_ENABLE_1, 0x80)) {
        xprintln("MAX30102: Interrupt enable failed."); return false;
    }
    if (!writeRegister(MAX30102_REG_INT_ENABLE_2, 0x00)) { // No other interrupts
        xprintln("MAX30102: INT_ENABLE_2 config failed."); return false;
    }

    // Clear FIFO pointers
    if (!writeRegister(MAX30102_REG_FIFO_WR_PTR, 0x00)) return false;
    if (!writeRegister(MAX30102_REG_OVF_COUNTER, 0x00)) return false;
    if (!writeRegister(MAX30102_REG_FIFO_RD_PTR, 0x00)) return false;

    // LED Pulse Amplitude - Example: 50% of 50mA = 25mA. Adjust as needed.
    uint8_t led_pa = 0x3F; // ~12.6mA. Max 0x7F for ~25.5mA, 0xFF for 51mA. Start lower.
    if (!writeRegister(MAX30102_REG_LED1_PA, led_pa)) return false; // Red LED
    if (!writeRegister(MAX30102_REG_LED2_PA, led_pa)) return false; // IR LED

    // SpO2 Config: ADC Range=4096nA, Sample Rate=50SPS, LED PW=411us
    if (!writeRegister(MAX30102_REG_SPO2_CONFIG, 0x27)) { // 0b00100111: SPO2_ADC_RGE=4096nA, SPO2_SR=50Hz, LED_PW=411us
        xprintln("MAX30102: SpO2 config failed."); return false;
    }
    // Multi-LED Mode Control Registers (for SpO2 mode)
    // SLOT1 = RED, SLOT2 = IR
    if (!writeRegister(MAX30102_REG_MULTI_LED_CTRL1, 0x12)) return false; // SLOT2=IR, SLOT1=RED
    if (!writeRegister(MAX30102_REG_MULTI_LED_CTRL2, 0x00)) return false; // No other slots

    // Mode Config: SpO2 mode enabled
    if (!writeRegister(MAX30102_REG_MODE_CONFIG, 0x03)) { // 0b00000011 for SpO2 mode
        xprintln("MAX30102: Mode set to SpO2 failed."); return false;
    }
    xprintln("MAX30102: Initialized successfully.");
    return true;
}

// shutdownSensor method - Definition
void MAX30102::shutdownSensor() {
    xprintln("MAX30102: Shutting down sensor...");
    // Set mode to SHDN
    writeRegister(MAX30102_REG_MODE_CONFIG, 0x80); // Bit 7 SHDN = 1
}

// collectPPGData method - Definition
uint16_t MAX30102::collectPPGData(TickType_t total_duration_ticks, TickType_t discard_duration_ticks) {
    uint8_t wr_ptr, rd_ptr, ovf_counter;
    uint8_t samples_in_fifo;
    uint8_t fifo_sample_buffer[32 * 6]; // Max FIFO size * 6 bytes per sample (3 for IR, 3 for Red)
    uint16_t current_stored_sample_index = 0;
    uint8_t interrupt_status_reg_val;

    TickType_t operation_start_time = xTaskGetTickCount();
    TickType_t discard_end_time = operation_start_time + discard_duration_ticks;
    TickType_t collection_end_time = operation_start_time + total_duration_ticks;

    xprintln("PPG: Starting data collection loop...");
    xprint("PPG: Total duration: "); xprint(total_duration_ticks * portTICK_PERIOD_MS); xprintln(" ms");
    xprint("PPG: Discarding data for first: "); xprint(discard_duration_ticks * portTICK_PERIOD_MS); xprintln(" ms");

    // Clear any initial interrupt flag and semaphore
    readRegister(MAX30102_REG_INT_STATUS_1, &interrupt_status_reg_val);
    xSemaphoreTake(ppgDataNotificationSemaphore, 0); // Clear semaphore if already set

    while (xTaskGetTickCount() < collection_end_time) {
        // Wait for FIFO almost full interrupt or timeout
        if (xSemaphoreTake(ppgDataNotificationSemaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
            // Interrupt occurred, read status to clear it (datasheet might specify which bit to check)
            readRegister(MAX30102_REG_INT_STATUS_1, &interrupt_status_reg_val);
            // if (!(interrupt_status_reg_val & 0x80)) { /* A_FULL was not set, maybe another interrupt? */ }
        } else {
            xprintln("PPG: Semaphore timeout. Checking FIFO manually.");
            // If timeout, still check FIFO status
        }

        if (!readRegister(MAX30102_REG_FIFO_WR_PTR, &wr_ptr)) continue;
        if (!readRegister(MAX30102_REG_FIFO_RD_PTR, &rd_ptr)) continue;
        if (!readRegister(MAX30102_REG_OVF_COUNTER, &ovf_counter)) continue;

        if (ovf_counter > 0) {
            xprint("PPG: FIFO Overflow detected! Count: "); xprintln(ovf_counter);
            // Consider resetting FIFO or other recovery
        }

        samples_in_fifo = (wr_ptr - rd_ptr);
        if (samples_in_fifo < 0) samples_in_fifo += 32; // FIFO size is 32

        if (samples_in_fifo > 0) {
            uint8_t bytes_to_read = samples_in_fifo * 6; // 3 bytes for IR, 3 for Red
            if (readBurst(MAX30102_REG_FIFO_DATA, fifo_sample_buffer, bytes_to_read)) {
                TickType_t current_tick = xTaskGetTickCount();
                for (int i = 0; i < samples_in_fifo; i++) {
                    if (current_tick >= discard_end_time) { // Start storing only after discard period
                        if (current_stored_sample_index < PPG_BUFFER_SIZE) {
                            // IR data: 3 bytes (MSB, Mid, LSB). MAX30102 uses 18-bit ADC.
                            // Data is [ D17 D16 | D15 D14 D13 D12 D11 D10 | D9 D8 D7 D6 D5 D4 D3 D2 ]
                            // First byte read: [0 0 0 0 0 0 D17 D16]
                            // Second byte:     [D15 D14 D13 D12 D11 D10 D9 D8]
                            // Third byte:      [D7 D6 D5 D4 D3 D2 D1 D0] (D1, D0 are 0 for 18-bit)
                            // We are interested in the top 18 bits.
                            ppgDataBuffer[current_stored_sample_index].ir =
                                (((uint32_t)fifo_sample_buffer[i * 6 + 0] & 0x03) << 16) | // Top 2 bits of 18-bit data
                                ((uint32_t)fifo_sample_buffer[i * 6 + 1] << 8)  | // Middle 8 bits
                                (uint32_t)fifo_sample_buffer[i * 6 + 2];          // Lowest 8 bits
                            ppgDataBuffer[current_stored_sample_index].red =
                                (((uint32_t)fifo_sample_buffer[i * 6 + 3] & 0x03) << 16) |
                                ((uint32_t)fifo_sample_buffer[i * 6 + 4] << 8)  |
                                (uint32_t)fifo_sample_buffer[i * 6 + 5];
                            current_stored_sample_index++;
                        } else {
                            xprintln("PPG: Buffer full. Stopping collection early.");
                            goto collection_done; // Exit outer loop
                        }
                    }
                }
            } else {
                xprintln("PPG: FIFO read burst failed.");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to allow other tasks and prevent busy-looping if no data
    }

collection_done:
    xprint("PPG: Collection finished. Stored valid samples: "); xprintln(current_stored_sample_index);
    return current_stored_sample_index;
}

void MAX30102::calculateHRAndSpO2(const ppgSample samples[], uint16_t num_samples, float& out_hr, float& out_spo2) {
    // Default to 0, which we'll treat as invalid data
    out_hr = 0;
    out_spo2 = 0;

    if (num_samples < SAMPLES_PER_SECOND * 2) {
        xprintln("Calc: Not enough samples for a reliable calculation.");
        return;
    }

    // --- Heart Rate Calculation ---
    float ir_filtered[PPG_BUFFER_SIZE];
    uint32_t ir_max = 0, ir_min = UINT32_MAX;

    // Apply a simple moving average filter to the IR signal to reduce noise
    for (int i = 0; i < num_samples; i++) {
        float sum = 0;
        int count = 0;
        for (int j = max(0, i - 2); j <= min((int)num_samples - 1, i + 2); j++) {
            sum += samples[j].ir;
            count++;
        }
        ir_filtered[i] = sum / count;
        if (ir_filtered[i] > ir_max) ir_max = ir_filtered[i];
        if (ir_filtered[i] < ir_min) ir_min = ir_filtered[i];
    }

    // Check if signal has enough dynamic range
    if (ir_max - ir_min < 1000) {
        xprintln("Calc: IR signal is too flat. Check sensor placement.");
        return;
    }

    // Adaptive threshold for peak detection
    float threshold = ir_min + (ir_max - ir_min) * 0.6f;
    int peak_count = 0;
    int peak_positions[100]; // Assume no more than 100 peaks in the buffer
    int peak_index = 0;

    for (int i = 2; i < num_samples - 2; i++) {
        if (peak_index >= 100) break; // Prevent buffer overflow on peak_positions

        // Condition for a point to be a peak
        if (ir_filtered[i] > threshold &&
            ir_filtered[i] > ir_filtered[i-1] &&
            ir_filtered[i] > ir_filtered[i+1] &&
            ir_filtered[i] > ir_filtered[i-2] &&
            ir_filtered[i] > ir_filtered[i+2]) {

            // Validate distance between peaks (at 50Hz, 300bpm is a peak every 10 samples)
            if (peak_index == 0 || (i - peak_positions[peak_index-1] > 10)) {
                peak_positions[peak_index++] = i;
            }
        }
    }
    peak_count = peak_index;

    // Calculate heart rate from the average distance between peaks
    if (peak_count > 1) {
        float avg_interval = 0;
        for (int i = 1; i < peak_count; i++) {
            avg_interval += (peak_positions[i] - peak_positions[i-1]);
        }
        avg_interval /= (peak_count - 1);
        out_hr = (60.0f * SAMPLES_PER_SECOND) / avg_interval;
    }

    // --- SpO2 Calculation ---
    uint64_t ir_sum = 0, red_sum = 0;
    for(int i=0; i<num_samples; ++i) {
        ir_sum += samples[i].ir;
        red_sum += samples[i].red;
    }

    float dc_ir = (float)ir_sum / num_samples;
    float dc_red = (float)red_sum / num_samples;

    // Calculate AC component using RMS (more stable than peak-to-peak)
    float ac_ir_sum_sq = 0, ac_red_sum_sq = 0;
    for(int i=0; i<num_samples; ++i) {
        ac_ir_sum_sq += pow(samples[i].ir - dc_ir, 2);
        ac_red_sum_sq += pow(samples[i].red - dc_red, 2);
    }
    float ac_ir_rms = sqrt(ac_ir_sum_sq / num_samples);
    float ac_red_rms = sqrt(ac_red_sum_sq / num_samples);

    // Calculate R and SpO2
    float R = (ac_red_rms / dc_red) / (ac_ir_rms / dc_ir);
    out_spo2 = 104.0f - 17.0f * R;

    // --- Final Data Validation ---
    if (out_hr < 40 || out_hr > 180) out_hr = 0;
    if (out_spo2 < 70 || out_spo2 > 100) out_spo2 = 0;

    if(out_hr == 0 && out_spo2 == 0) {
        xprintln("Calc: Failed to derive valid HR and SpO2. Data is likely noisy.");
    }
}

MAX30102::~MAX30102() {
    if (ppgDataNotificationSemaphore != NULL) {
        vSemaphoreDelete(ppgDataNotificationSemaphore);
        ppgDataNotificationSemaphore = NULL;
    }

    // Ensure MAX30102_INT_PIN is valid before detaching
    if (MAX30102_INT_PIN >= 0 && MAX30102_INT_PIN < NUM_DIGITAL_PINS) { // Basic check for valid pin
        detachInterrupt(digitalPinToInterrupt(MAX30102_INT_PIN));
    }
}

// New function definition to check sensor connectivity
bool MAX30102::isConnected() {
    uint8_t part_id;
    if (!readRegister(MAX30102_REG_PART_ID, &part_id)) {
        xprintln("MAX30102: Failed to read Part ID (I2C communication error).");
        return false; // I2C communication failed
    }
    if (part_id == 0x15) { // Expected Part ID for MAX30102
        return true;
    } else {
        xprintf("MAX30102: Unexpected Part ID: 0x%02X\n", part_id);
        return false; // Wrong Part ID or sensor not responding correctly
    }
}
