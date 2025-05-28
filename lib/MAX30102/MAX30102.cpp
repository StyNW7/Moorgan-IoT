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
    dataframetofill = nullptr; // Initialize members
    validSamplesCount = 0;
    ppgDataNotificationSemaphore = NULL; // Will be created in setup
    readingscount = 255; // Initialize reading count
    last_heart_rate = 0.0f; // Initialize last heart rate
    last_oxygen_saturation = 0.0f; // Initialize last oxygen saturation
}

// ISR Wrapper - Definition
void IRAM_ATTR MAX30102::isrWrapper() {
    if (instance) instance->interruptServiceRoutine();
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
        for(int i=0; i < 10 && i < validSamplesCount; ++i) {
            xprint("Sample "); xprint(i);
            xprint(" - IR: "); xprint(ppgDataBuffer[i].ir);
            xprint(", Red: "); xprintln(ppgDataBuffer[i].red);
        }

        float heart_rate_bpm = calculateHeartRate(ppgDataBuffer, validSamplesCount);
        float oxygen_saturation_percent = calculateSpO2(ppgDataBuffer, validSamplesCount);
        
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

// interruptServiceRoutine method - Definition
void MAX30102::interruptServiceRoutine() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (ppgDataNotificationSemaphore != NULL) {
        xSemaphoreGiveFromISR(ppgDataNotificationSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
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

// calculateHeartRate method - Definition
float MAX30102::calculateHeartRate(const ppgSample samples[], uint16_t num_samples) {
    // Ensure SAMPLES_PER_SECOND is defined (e.g., in config.h, matching sensor config)
    if (num_samples < SAMPLES_PER_SECOND * 5) { // Need at least 5 seconds of data
        xprintln("HR Calc: Not enough samples for HR calculation.");
        return 0.0;
    }

    int peak_count = 0;
    uint64_t ir_sum_long = 0; // Use 64-bit to avoid overflow for sum
    for(int i=0; i<num_samples; ++i) ir_sum_long += samples[i].ir;
    uint32_t ir_avg = ir_sum_long / num_samples;

    bool above_threshold = false;
    // Simple peak detection based on crossing the average
    for (uint16_t i = 0; i < num_samples; i++) {
        if (samples[i].ir > ir_avg && !above_threshold) {
            peak_count++;
            above_threshold = true;
        } else if (samples[i].ir < ir_avg) {
            above_threshold = false;
        }
    }

    float duration_seconds = (float)num_samples / SAMPLES_PER_SECOND;
    if (duration_seconds <= 0 || peak_count <= 1) { // Need more than one peak to calculate rate
        xprintln("HR Calc: Not enough peaks or invalid duration.");
        return 0.0;
    }

    float bpm = ((float)peak_count / duration_seconds) * 60.0f;

    // Basic plausibility check for cattle (adjust ranges as needed)
    if (bpm < 30.0 || bpm > 160.0) {
        xprint("HR Calc: Calculated BPM ("); xprint(bpm); xprintln(") out of typical range for cattle (30-160).");
        // Could return 0 or the calculated value depending on requirements
    }
    return bpm;
}

// calculateSpO2 method - Definition
float MAX30102::calculateSpO2(const ppgSample samples[], uint16_t num_samples) {
    if (num_samples < SAMPLES_PER_SECOND * 5) { // Need at least 5 seconds of data
        xprintln("SpO2 Calc: Not enough samples for SpO2 calculation.");
        return 0.0;
    }

    uint64_t sum_ir_dc_long = 0, sum_red_dc_long = 0;
    uint32_t min_ir_val = samples[0].ir, max_ir_val = samples[0].ir;
    uint32_t min_red_val = samples[0].red, max_red_val = samples[0].red;

    for(uint16_t i=0; i<num_samples; ++i) {
        sum_ir_dc_long += samples[i].ir;
        sum_red_dc_long += samples[i].red;

        if(samples[i].ir < min_ir_val) min_ir_val = samples[i].ir;
        if(samples[i].ir > max_ir_val) max_ir_val = samples[i].ir;
        if(samples[i].red < min_red_val) min_red_val = samples[i].red;
        if(samples[i].red > max_red_val) max_red_val = samples[i].red;
    }

    float dc_ir = (float)sum_ir_dc_long / num_samples;
    float dc_red = (float)sum_red_dc_long / num_samples;

    // AC component is peak-to-peak amplitude
    float ac_ir = (float)(max_ir_val - min_ir_val);
    float ac_red = (float)(max_red_val - min_red_val);

    // Check for valid signal strength
    if (dc_ir <= 1000 || dc_red <= 1000 || ac_ir <= 100 || ac_red <= 100 ) { // Thresholds may need tuning
        xprintln("SpO2 Calc: Signal too weak or flat for reliable SpO2 calculation.");
        xprint("DC_IR: "); xprint(dc_ir); xprint(", DC_RED: "); xprint(dc_red);
        xprint(", AC_IR: "); xprint(ac_ir); xprint(", AC_RED: "); xprintln(ac_red);
        return 0.0;
    }

    // Ratio of Ratios (R)
    float R = (ac_red / dc_red) / (ac_ir / dc_ir);

    // Empiric SpO2 formula (common approximation, may need calibration for specific sensor/setup)
    // SpO2 = A - B * R.  Common values are A=110, B=25. Or A=104, B=17 for some.
    // This formula is highly dependent on the sensor, wavelength, and subject.
    // For real accuracy, calibration is essential.
    float spo2_estimate = 110.0f - 25.0f * R;

    if (spo2_estimate > 100.0f) spo2_estimate = 100.0f; // Cap at 100%
    if (spo2_estimate < 70.0f) { // Unlikely low for healthy, conscious animal
        xprint("SpO2 Calc: Calculated SpO2 ("); xprint(spo2_estimate); xprint("%) out of typical range or R value problematic. R = "); xprintln(R);
        // Could return 0 or the calculated value
    }
    return spo2_estimate;
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
