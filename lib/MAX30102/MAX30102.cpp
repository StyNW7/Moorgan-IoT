#include <Wire.h>
#include <Tools.h>
#include <config.h>
#include <MAX30102.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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

struct ppgSample {
    uint32_t ir;
    uint32_t red;
};


class MAX30102 {
    public:
    MAXData *dataframetofill;
    ppgSample ppgDataBuffer[PPG_BUFFER_SIZE];
    uint16_t validSamplesCount = 0;
    SemaphoreHandle_t ppgDataNotificationSemaphore = NULL;

    static MAX30102* instance;

    static void IRAM_ATTR isrWrapper() {
        if (instance) instance->interruptServiceRoutine();
    }

    MAX30102() {
        instance = this;
    }

    void setup() {
        ppgDataNotificationSemaphore = xSemaphoreCreateBinary();
        if (ppgDataNotificationSemaphore == NULL) {
            xprintln("Fatal Error: Failed to create PPG data semaphore!");
            return;
        }
        pinMode(MAX30102_INT_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(MAX30102_INT_PIN), isrWrapper, FALLING);
        xprintln("Main setup complete");
    }

    void runReading(MAXData *dataframe) {
        this->dataframetofill = dataframe;

        BaseType_t taskCreated = xTaskCreate(
            taskWrapper,
            "PPGSensorTask",
            1024*9,
            this,
            1,
            NULL
        );
        if (taskCreated != pdPASS) {
            xprintln("Fatal Error: Failed to create PPG Sensor Task!");
            return;
        }
    }

    static void taskWrapper(void* pvParameters) {
        MAX30102* self = static_cast<MAX30102*>(pvParameters);
        self->ppgSensorTask();
    }

    void ppgSensorTask() {
        xprintln("Task: PPG Sensor Task Started.");
        if (!initializeForSpotCheck()) {
            xprintln("Task: MAX30102 setup failed. Deleting task.");
            vTaskDelete(NULL);
            return;
        }
        validSamplesCount = collectPPGData(
            pdMS_TO_TICKS(TOTAL_OPERATION_DURATION_MS),
            pdMS_TO_TICKS(DISCARD_DURATION_MS)
        );
        shutdownSensor();

        if (validSamplesCount > 0) {
            xprintln("\n--- Data Processing ---");
            for(int i=0; i < 10 && i < validSamplesCount; ++i) {
                xprint("Sample "); xprint(i);
                xprint(" - IR: "); xprint(ppgDataBuffer[i].ir);
                xprint(", Red: "); xprintln(ppgDataBuffer[i].red);
            }
            float heart_rate_bpm = calculateHeartRate(ppgDataBuffer, validSamplesCount);
            float oxygen_saturation_percent = calculateSpO2(ppgDataBuffer, validSamplesCount);
            dataframetofill->heart_rate = heart_rate_bpm;
            dataframetofill->oxygen = oxygen_saturation_percent;

            xprintf("\nEstimated Heart Rate: %.1f BPM\n", heart_rate_bpm);
            xprintf("Estimated SpO2: %.1f %%\n", oxygen_saturation_percent);
        } else {
            xprintln("Task: No valid PPG samples were collected.");
        }
        xprintln("\nTask: PPG Sensor Task finished its cycle.");
        vTaskDelete(NULL);
    }

    void interruptServiceRoutine() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (ppgDataNotificationSemaphore != NULL) {
            xSemaphoreGiveFromISR(ppgDataNotificationSemaphore, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }
    }

    bool writeRegister(uint8_t reg_addr, uint8_t value) {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(reg_addr);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }

    bool readRegister(uint8_t reg_addr, uint8_t *value) {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(reg_addr);
        if (Wire.endTransmission(false) != 0) { 
            xprintln("I2C: Failed to send reg addr for read");
            return false; 
        }
        if (Wire.requestFrom(MAX30102_ADDRESS, (uint8_t)1) == 1) {
            *value = Wire.read();
            return true;
        }
        xprintln("I2C: Failed to read byte");
        return false;
    }

    bool readBurst(uint8_t start_reg_addr, uint8_t *buffer, uint8_t count) {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(start_reg_addr);
        if (Wire.endTransmission(false) != 0) { 
            xprintln("I2C: Failed to send reg addr for burst read");
            return false; 
        }
        if (Wire.requestFrom(MAX30102_ADDRESS, count) == count) {
            for (uint8_t i = 0; i < count; i++) {
                buffer[i] = Wire.read();
            }
            return true;
        }
        xprintln("I2C: Failed to read burst bytes");
        return false;
    }

    bool initializeForSpotCheck() {
        uint8_t temp_val;
        xprintln("Initializing MAX30102 for spot check (50SPS, 4x Avg)...");
        if (!readRegister(MAX30102_REG_PART_ID, &temp_val) || temp_val != 0x15) {
            xprintln("MAX30102: Not found or wrong Part ID.");
            return false;
        }
        xprintln("MAX30102: Part ID OK.");
        if (!writeRegister(MAX30102_REG_MODE_CONFIG, 0x40)) {
            xprintln("MAX30102: Reset failed."); return false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        uint32_t startTime = millis();
        do {
            if (!readRegister(MAX30102_REG_MODE_CONFIG, &temp_val)) return false;
            if (millis() - startTime > 500) { xprintln("MAX30102: Reset timeout."); return false; }
        } while (temp_val & 0x40);
        xprintln("MAX30102: Soft reset complete.");
        readRegister(MAX30102_REG_INT_STATUS_1, &temp_val); 
        readRegister(MAX30102_REG_INT_STATUS_2, &temp_val); 
        if (!writeRegister(MAX30102_REG_FIFO_CONFIG, 0x5F)) {
            xprintln("MAX30102: FIFO config failed."); return false;
        }
        if (!writeRegister(MAX30102_REG_INT_ENABLE_1, 0x80)) {
            xprintln("MAX30102: Interrupt enable failed."); return false;
        }
        if (!writeRegister(MAX30102_REG_INT_ENABLE_2, 0x00)) {
            xprintln("MAX30102: INT_ENABLE_2 config failed."); return false;
        }
        if (!writeRegister(MAX30102_REG_FIFO_WR_PTR, 0x00)) return false;
        if (!writeRegister(MAX30102_REG_OVF_COUNTER, 0x00)) return false;
        if (!writeRegister(MAX30102_REG_FIFO_RD_PTR, 0x00)) return false;
        uint8_t led_pa = 0x3F;
        if (!writeRegister(MAX30102_REG_LED1_PA, led_pa)) return false;
        if (!writeRegister(MAX30102_REG_LED2_PA, led_pa)) return false;
        if (!writeRegister(MAX30102_REG_SPO2_CONFIG, 0x23)) {
            xprintln("MAX30102: SpO2 config failed."); return false;
        }
        if (!writeRegister(MAX30102_REG_MULTI_LED_CTRL1, 0x21)) return false;
        if (!writeRegister(MAX30102_REG_MULTI_LED_CTRL2, 0x00)) return false;
        if (!writeRegister(MAX30102_REG_MODE_CONFIG, 0x03)) {
            xprintln("MAX30102: Mode set to SpO2 failed."); return false;
        }
        xprintln("MAX30102: Initialized successfully.");
        return true;
    }

    void shutdownSensor() {
        xprintln("MAX30102: Shutting down sensor...");
        writeRegister(MAX30102_REG_MODE_CONFIG, 0x80);
    }

    uint16_t collectPPGData(TickType_t total_duration_ticks, TickType_t discard_duration_ticks) {
        uint8_t wr_ptr, rd_ptr, ovf_counter;
        uint8_t samples_in_fifo;
        uint8_t fifo_sample_buffer[32 * 6];
        uint16_t current_stored_sample_index = 0;
        uint8_t interrupt_status_reg_val;

        TickType_t operation_start_time = xTaskGetTickCount();
        TickType_t discard_end_time = operation_start_time + discard_duration_ticks;
        TickType_t collection_end_time = operation_start_time + total_duration_ticks;

        xprintln("PPG: Starting data collection loop...");
        xprint("PPG: Total duration: "); xprint(total_duration_ticks * portTICK_PERIOD_MS); xprintln(" ms");
        xprint("PPG: Discarding data for first: "); xprint(discard_duration_ticks * portTICK_PERIOD_MS); xprintln(" ms");
        readRegister(MAX30102_REG_INT_STATUS_1, &interrupt_status_reg_val); 
        xSemaphoreTake(ppgDataNotificationSemaphore, 0);

        while (xTaskGetTickCount() < collection_end_time) {
            if (xSemaphoreTake(ppgDataNotificationSemaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
                readRegister(MAX30102_REG_INT_STATUS_1, &interrupt_status_reg_val);
            } else {
                xprintln("PPG: Semaphore timeout. Checking FIFO manually.");
            }
            if (!readRegister(MAX30102_REG_FIFO_WR_PTR, &wr_ptr)) continue;
            if (!readRegister(MAX30102_REG_FIFO_RD_PTR, &rd_ptr)) continue;
            if (!readRegister(MAX30102_REG_OVF_COUNTER, &ovf_counter)) continue;
            if (ovf_counter > 0) {
                xprint("PPG: FIFO Overflow detected! Count: "); xprintln(ovf_counter);
            }
            samples_in_fifo = (wr_ptr - rd_ptr);
            if (samples_in_fifo < 0) samples_in_fifo += 32;
            if (samples_in_fifo > 0) {
                uint8_t bytes_to_read = samples_in_fifo * 6;
                if (readBurst(MAX30102_REG_FIFO_DATA, fifo_sample_buffer, bytes_to_read)) {
                    TickType_t current_tick = xTaskGetTickCount();
                    for (int i = 0; i < samples_in_fifo; i++) {
                        if (current_tick >= discard_end_time) {
                            if (current_stored_sample_index < PPG_BUFFER_SIZE) {
                                ppgDataBuffer[current_stored_sample_index].ir = 
                                    (((uint32_t)fifo_sample_buffer[i * 6 + 0] & 0x03) << 16) |
                                    ((uint32_t)fifo_sample_buffer[i * 6 + 1] << 8) |
                                    (uint32_t)fifo_sample_buffer[i * 6 + 2];
                                ppgDataBuffer[current_stored_sample_index].red = 
                                    (((uint32_t)fifo_sample_buffer[i * 6 + 3] & 0x03) << 16) |
                                    ((uint32_t)fifo_sample_buffer[i * 6 + 4] << 8) |
                                    (uint32_t)fifo_sample_buffer[i * 6 + 5];
                                current_stored_sample_index++;
                            } else {
                                xprintln("PPG: Buffer full. Stopping collection early.");
                                goto collection_done;
                            }
                        }
                    }
                } else {
                    xprintln("PPG: FIFO read burst failed.");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
    collection_done:
        xprint("PPG: Collection finished. Stored valid samples: "); xprintln(current_stored_sample_index);
        return current_stored_sample_index;
    }

    float calculateHeartRate(const ppgSample samples[], uint16_t num_samples) {
        if (num_samples < SAMPLES_PER_SECOND * 5) {
            xprintln("HR Calc: Not enough samples for HR calculation.");
            return 0.0; 
        }
        int peak_count = 0;
        uint32_t ir_avg = 0;
        for(int i=0; i<num_samples; ++i) ir_avg += samples[i].ir;
        ir_avg /= num_samples;
        bool above_threshold = false;
        for (uint16_t i = 0; i < num_samples; i++) {
            if (samples[i].ir > ir_avg && !above_threshold) {
                peak_count++;
                above_threshold = true;
            } else if (samples[i].ir < ir_avg) {
                above_threshold = false;
            }
        }
        float duration_seconds = (float)num_samples / SAMPLES_PER_SECOND;
        if (duration_seconds <= 0 || peak_count <= 1) {
            xprintln("HR Calc: Not enough peaks or invalid duration.");
            return 0.0;
        }
        float bpm = ((float)peak_count / duration_seconds) * 60.0f;
        if (bpm < 30.0 || bpm > 160.0) {
            xprint("HR Calc: Calculated BPM out of typical range for cattle: "); xprintln(bpm);
        }
        return bpm;
    }

    float calculateSpO2(const ppgSample samples[], uint16_t num_samples) {
        if (num_samples < SAMPLES_PER_SECOND * 5) {
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
        float ac_ir = (float)(max_ir_val - min_ir_val);
        float ac_red = (float)(max_red_val - min_red_val);
        if (dc_ir <= 1000 || dc_red <= 1000 || ac_ir <= 100 || ac_red <= 100 ) {
            xprintln("SpO2 Calc: Signal too weak or flat for reliable SpO2 calculation.");
            return 0.0; 
        }
        float R = (ac_red / dc_red) / (ac_ir / dc_ir);
        float spo2_estimate = 110.0f - 25.0f * R; 
        if (spo2_estimate > 100.0f) spo2_estimate = 100.0f;
        if (spo2_estimate < 70.0f) {
            xprint("SpO2 Calc: Calculated SpO2 out of typical range or R value problematic. R = "); xprintln(R);
        }
        return spo2_estimate;
    }
};

MAX30102* MAX30102::instance = nullptr;
