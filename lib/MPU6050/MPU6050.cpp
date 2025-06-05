#include <MPU6050.h>
#include <config.h>
#include <Wire.h>
#include <Tools.h>

#ifdef MPU_INTERRUPT_PIN
volatile bool mpuFifoOverflowInterrupt = false; 
#endif

static float currentMPUSensitivity = 4096.0f; // Default for +/- 8G
static uint8_t currentAccelRangeBits = MPU6050_ACCEL_FS_SEL_8G;


float getMPUSensitivity() {
    return currentMPUSensitivity;
}

bool getMovAvail() { 
    uint8_t fifoCountBytes[2];
    readMPURegisters(MPU6050_RA_FIFO_COUNTH, 2, fifoCountBytes);
    uint16_t fifoCount = (fifoCountBytes[0] << 8) | fifoCountBytes[1];
    if (fifoCount > 0) return true; 
    return false; 
}


#ifdef MPU_INTERRUPT_PIN
void IRAM_ATTR onMpuHardwareInterrupt() {
    mpuFifoOverflowInterrupt = true;
}
#endif

void writeMPURegister(uint8_t regAddress, uint8_t data) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(regAddress);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t readMPURegister(uint8_t regAddress) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(regAddress);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDRESS, (uint8_t)1);
    return Wire.read();
}

void readMPURegisters(uint8_t regAddress, uint8_t count, uint8_t *dest) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(regAddress);
    Wire.endTransmission(false);
    uint8_t i = 0;
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDRESS, count);
    while (Wire.available() && i < count) {
        dest[i++] = Wire.read();
    }
}

// MODIFIED mpuSetup
bool mpuSetup(bool isWakeUp){ 
    xprintln(isWakeUp ? "MPU6050 Re-initializing (Wake-Up)..." : "MPU6050 Initializing (First Boot)...");

    uint8_t who_am_i = readMPURegister(MPU6050_RA_WHO_AM_I);
    if (who_am_i != 0x68 && who_am_i != 0x72 && who_am_i != 0x70) { 
        xprint("Failed to find MPU6050 chip. WHO_AM_I: "); 
        xprintf("0x%02X\n", who_am_i);
        return false;
    }
    xprintln("MPU6050 Found!");

    uint8_t pwr_mgmt_1_val;
    uint8_t pwr_mgmt_2_val;
    uint8_t user_ctrl_val;

    if (!isWakeUp) {
        // --- Full Reset and Initialization Sequence for First Boot ---
        xprintln("Performing full MPU6050 device reset and FIFO reset.");
        writeMPURegister(MPU6050_RA_PWR_MGMT_1, 0x80); // Set PWR_MGMT_1 to 0x80 to reset device
        delay(100); // Wait for reset to complete

        // Wake up MPU6050 from sleep (after reset)
        writeMPURegister(MPU6050_RA_PWR_MGMT_1, 0x00); // Clear SLEEP bit
        delay(50);

        // Set Accelerometer Range (important for sensitivity calculation later)
        currentAccelRangeBits = MPU6050_ACCEL_FS_SEL_8G; // Corresponds to +/- 8g
        writeMPURegister(MPU6050_RA_ACCEL_CONFIG, currentAccelRangeBits);
        // Update sensitivity (simplified here, your existing logic for this was more detailed)
        currentMPUSensitivity = 4096.0f; 
        xprintln("Accelerometer range set to +/-8G (first boot).");


        // Configure MPU6050 for Low-Power Cycle Mode & Accel-Only FIFO
        pwr_mgmt_1_val = readMPURegister(MPU6050_RA_PWR_MGMT_1); // Read current (should be 0x00 after wake)
        pwr_mgmt_1_val |= (1 << 5); // Set CYCLE bit
        pwr_mgmt_1_val &= ~(1 << 6); // Ensure SLEEP bit is 0
        writeMPURegister(MPU6050_RA_PWR_MGMT_1, pwr_mgmt_1_val);
    
        pwr_mgmt_2_val = MPU6050_LP_WAKE_CTRL_1_25HZ << 6; 
        writeMPURegister(MPU6050_RA_PWR_MGMT_2, pwr_mgmt_2_val);
        float cycle_rate_hz = 1.25f;
        xprint("MPU6050 set to "); xprint(cycle_rate_hz); xprintln(" Hz cycle rate (accelerometer only).");

        pwr_mgmt_1_val = readMPURegister(MPU6050_RA_PWR_MGMT_1);
        pwr_mgmt_1_val |= (1 << 3); // Set TEMP_DIS bit (bit 3)
        writeMPURegister(MPU6050_RA_PWR_MGMT_1, pwr_mgmt_1_val);
        xprintln("Temperature sensor explicitly disabled.");

        // Configure User Control (USER_CTRL - 0x6A): Reset FIFO, then Enable FIFO.
        writeMPURegister(MPU6050_RA_USER_CTRL, (1 << 2)); // Assert FIFO_RESET (bit 2)
        delay(50); 
        writeMPURegister(MPU6050_RA_USER_CTRL, (1 << 6)); // Assert FIFO_EN (bit 6), FIFO_RESET is now 0
        xprintln("FIFO Reset and Enabled (first boot).");

    } else {
        // --- Re-initialization Sequence for Wake-up (Preserve FIFO) ---
        xprintln("Attempting to preserve MPU6050 FIFO content on wake-up.");

        // Ensure MPU6050 is awake and in the correct power mode
        pwr_mgmt_1_val = readMPURegister(MPU6050_RA_PWR_MGMT_1);
        bool mpu_was_sleeping = (pwr_mgmt_1_val & (1 << 6)); // Check SLEEP bit
        
        pwr_mgmt_1_val &= ~(1 << 6); // Clear SLEEP bit (ensure MPU is awake)
        pwr_mgmt_1_val |= (1 << 5);  // Re-assert CYCLE bit
        pwr_mgmt_1_val |= (1 << 3);  // Re-assert TEMP_DIS bit
        writeMPURegister(MPU6050_RA_PWR_MGMT_1, pwr_mgmt_1_val);
        if (mpu_was_sleeping) {
            delay(50); // Give MPU time to wake up if it was sleeping
            xprintln("MPU6050 was in sleep mode, now woken.");
        }
        xprintln("MPU6050 PWR_MGMT_1 reconfigured for cycle mode, temp disabled.");

        // Re-apply Accelerometer Range to ensure correct sensitivity for readings
        currentAccelRangeBits = MPU6050_ACCEL_FS_SEL_8G;
        writeMPURegister(MPU6050_RA_ACCEL_CONFIG, currentAccelRangeBits);
        currentMPUSensitivity = 4096.0f; // For +/- 8G
        xprintln("Accelerometer range re-applied (+/-8G).");


        // Re-apply LP_WAKE_CTRL in PWR_MGMT_2
        pwr_mgmt_2_val = MPU6050_LP_WAKE_CTRL_1_25HZ << 6;
        writeMPURegister(MPU6050_RA_PWR_MGMT_2, pwr_mgmt_2_val);
        xprintln("MPU6050 PWR_MGMT_2 reconfigured for 1.25Hz cycle rate.");

        // Ensure FIFO is enabled in USER_CTRL without resetting its content
        user_ctrl_val = readMPURegister(MPU6050_RA_USER_CTRL);
        user_ctrl_val |= (1 << 6);  // Set FIFO_EN (bit 6)
        user_ctrl_val &= ~(1 << 2); // Ensure FIFO_RESET (bit 2) is clear
        writeMPURegister(MPU6050_RA_USER_CTRL, user_ctrl_val);
        xprintln("FIFO Ensured Enabled (wake-up), content should be preserved.");
    }

    // Common configuration for both scenarios:
    // Select ONLY Accelerometer Data for FIFO (FIFO_EN register - 0x23)
    writeMPURegister(MPU6050_RA_FIFO_EN, 0b00001000); // ACCEL_FIFO_EN (bit 3)
    xprintln("FIFO source configured for Accelerometer data only.");

    #ifdef MPU_INTERRUPT_PIN
    // Re-configure MPU INT Pin for FIFO Overflow Interrupt (ESP32 side might need re-arming too)
    uint8_t intPinConfig = 0;
    intPinConfig |= (1 << 7); // ACTL: Active LOW
    intPinConfig |= (1 << 5); // LATCH_INT_EN: Latched interrupt
    writeMPURegister(MPU6050_RA_INT_PIN_CFG, intPinConfig);
    writeMPURegister(MPU6050_RA_INT_ENABLE, 0b00010000); // FIFO_OFLOW_EN (bit 4)
    xprintln("MPU INT pin configured for FIFO Overflow (Failsafe).");
    // Note: ESP32's attachInterrupt might need to be called again in main setup if using MPU interrupt for wake-up.
    // For timer-based wake-up as in main.cpp, this MPU-side config is for its interrupt behavior.
    #endif

    xprintf("  DEBUG: PWR_MGMT_1 = 0x%02X\n", readMPURegister(MPU6050_RA_PWR_MGMT_1));
    xprintf("  DEBUG: PWR_MGMT_2 = 0x%02X\n", readMPURegister(MPU6050_RA_PWR_MGMT_2));
    xprintf("  DEBUG: USER_CTRL = 0x%02X\n", readMPURegister(MPU6050_RA_USER_CTRL));
    xprintf("  DEBUG: FIFO_EN_REG = 0x%02X\n", readMPURegister(MPU6050_RA_FIFO_EN));

    xprintln(isWakeUp ? "MPU6050 Re-initialization (Wake-Up) complete." : "MPU6050 Initialization (First Boot) complete.");
    return true;
}


MPUData *readMPUData() {
    MPUData *data = new MPUData;
    data->mean_ax_arr = nullptr;
    data->mean_ay_arr = nullptr;
    data->mean_az_arr = nullptr;
    data->mean_mv_arr = nullptr;
    data->std_mv_arr = nullptr;
    data->windowSize = 0;


    #ifdef MPU_INTERRUPT_PIN
    if (mpuFifoOverflowInterrupt) {
        mpuFifoOverflowInterrupt = false; 
        xprintln("\n!! MPU HARDWARE INTERRUPT: FIFO Overflowed !!");
        uint8_t intStatus = readMPURegister(MPU6050_RA_INT_STATUS); 
        xprintf("!! MPU INT_STATUS: 0x%02X\n", intStatus);
    }
    #endif

    xprintln("\n--- Collecting data from MPU6050 FIFO ---");

    uint8_t fifoCountBytes[2];
    readMPURegisters(MPU6050_RA_FIFO_COUNTH, 2, fifoCountBytes);
    uint16_t fifoCount = (fifoCountBytes[0] << 8) | fifoCountBytes[1];
    xprint("Current FIFO Count: "); xprint(fifoCount); xprintln(" bytes.");

    if (fifoCount > 1024) { 
        xprintln("Warning: FIFO count reported > 1024. Correcting to 1024 bytes (max FIFO size).");
        fifoCount = 1024; // Max FIFO size
    }
    // Ensure fifoCount is a multiple of 6 (one sample = 6 bytes for accel X, Y, Z)
    fifoCount = (fifoCount / 6) * 6;


    if (fifoCount == 0) {
        xprintln("FIFO is empty. No new data to read for this interval.");
        data->windowSize = 0; 
    } else {
        // Dynamically allocate fifoBuffer if fifoCount can be large, or ensure it's sized for max (1024)
        // For simplicity, using a stack buffer assuming fifoCount won't exceed typical stack limits here.
        // If concerned, use heap: uint8_t* fifoBuffer = new uint8_t[fifoCount]; then delete[] later.
        uint8_t fifoBuffer[1024]; // Max size
        uint16_t bytesToRead = fifoCount;

        readMPURegisters(MPU6050_RA_FIFO_R_W, bytesToRead, fifoBuffer);
        xprint("Read "); xprint(bytesToRead); xprintln(" bytes from FIFO.");

        int numSamples = bytesToRead / 6;
        xprint("Number of accelerometer samples processed: "); xprintln(numSamples);

        uint16_t num_total_windows = 0;
        uint8_t windowSize = WINDOWSIZE; 

        if (numSamples > 0 && windowSize > 0) {
            num_total_windows = (numSamples + windowSize - 1) / windowSize; 
        } else {
            num_total_windows = 0;
        }
        
        data->windowSize = num_total_windows; 

        float *mean_ax_arr = nullptr;
        float *mean_ay_arr = nullptr;
        float *mean_az_arr = nullptr;
        float *mean_mv_arr = nullptr;
        float *std_mv_arr = nullptr;

        if (num_total_windows > 0) {
            mean_ax_arr = (float *)calloc(num_total_windows, sizeof(float));
            mean_ay_arr = (float *)calloc(num_total_windows, sizeof(float));
            mean_az_arr = (float *)calloc(num_total_windows, sizeof(float));
            mean_mv_arr = (float *)calloc(num_total_windows, sizeof(float));
            std_mv_arr = (float *)calloc(num_total_windows, sizeof(float));
            if (!mean_ax_arr || !mean_ay_arr || !mean_az_arr || !mean_mv_arr || !std_mv_arr) {
                xprintln("Error: calloc failed for MPU data arrays. No data will be processed.");
                numSamples = 0; 
                num_total_windows = 0;
                data->windowSize = 0;
                if (mean_ax_arr) free(mean_ax_arr); if (mean_ay_arr) free(mean_ay_arr);
                if (mean_az_arr) free(mean_az_arr); if (mean_mv_arr) free(mean_mv_arr);
                if (std_mv_arr) free(std_mv_arr);
                // Ensure the pointers in 'data' are also nullified if allocation failed
                mean_ax_arr = mean_ay_arr = mean_az_arr = mean_mv_arr = std_mv_arr = nullptr; 
            }
        }

        float temp_mv_g[windowSize > 0 ? windowSize : 1] = {0}; 
        float sensitivity = getMPUSensitivity(); 

        for (uint16_t i = 0; i < numSamples; i++) { 
            uint16_t offset = i * 6; 
            int16_t ax_raw = (int16_t)((fifoBuffer[offset + 0] << 8) | fifoBuffer[offset + 1]);
            int16_t ay_raw = (int16_t)((fifoBuffer[offset + 2] << 8) | fifoBuffer[offset + 3]);
            int16_t az_raw = (int16_t)((fifoBuffer[offset + 4] << 8) | fifoBuffer[offset + 5]);

            float ax_g = (float)ax_raw / sensitivity;
            float ay_g = (float)ay_raw / sensitivity;
            float az_g = (float)az_raw / sensitivity;
            
            if (windowSize > 0 && num_total_windows > 0 && mean_mv_arr) { // Check mean_mv_arr for safety
                temp_mv_g[i % windowSize] = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
                uint16_t current_window_idx = i / windowSize; 
                if (current_window_idx < num_total_windows) { 
                    mean_ax_arr[current_window_idx] += ax_g;
                    mean_ay_arr[current_window_idx] += ay_g;
                    mean_az_arr[current_window_idx] += az_g;
                    mean_mv_arr[current_window_idx] += temp_mv_g[i % windowSize];
                }
            }

            bool is_last_sample_of_all = (i + 1 == numSamples);

            if (windowSize > 0 && num_total_windows > 0 && std_mv_arr && ((i + 1) % windowSize == 0 || is_last_sample_of_all)) { // Check std_mv_arr
                uint16_t current_window_idx = i / windowSize; 
                if (current_window_idx >= num_total_windows) continue; 

                uint8_t samples_in_current_window_being_finalized;
                bool is_this_a_partial_window_at_end = false;

                if (((i + 1) % windowSize == 0)) { 
                    samples_in_current_window_being_finalized = windowSize;
                } else { 
                    samples_in_current_window_being_finalized = (i % windowSize) + 1;
                    is_this_a_partial_window_at_end = true;
                }

                bool process_this_window = true;
                if (is_this_a_partial_window_at_end) {
                    if (samples_in_current_window_being_finalized < (uint8_t)(windowSize * 0.25f)) {
                        process_this_window = false;
                        xprintf("Partial window %d (size %d) discarded as < 25%% of windowSize %d.\n",
                                current_window_idx + 1, samples_in_current_window_being_finalized, windowSize);
                        
                        if (current_window_idx == data->windowSize -1 && data->windowSize > 0) { 
                           data->windowSize--; 
                        }
                        mean_ax_arr[current_window_idx] = 0.0f; // Nullify discarded window data
                        mean_ay_arr[current_window_idx] = 0.0f;
                        mean_az_arr[current_window_idx] = 0.0f;
                        mean_mv_arr[current_window_idx] = 0.0f;
                    }
                }

                if (process_this_window && samples_in_current_window_being_finalized > 0) {
                    mean_ax_arr[current_window_idx] /= samples_in_current_window_being_finalized;
                    mean_ay_arr[current_window_idx] /= samples_in_current_window_being_finalized;
                    mean_az_arr[current_window_idx] /= samples_in_current_window_being_finalized;
                    mean_mv_arr[current_window_idx] /= samples_in_current_window_being_finalized;

                    float sum_sq = 0.0f;
                    for (uint8_t j = 0; j < samples_in_current_window_being_finalized; j++) {
                        sum_sq += (temp_mv_g[j] - mean_mv_arr[current_window_idx]) * (temp_mv_g[j] - mean_mv_arr[current_window_idx]);
                    }

                    if (samples_in_current_window_being_finalized > 1) {
                        std_mv_arr[current_window_idx] = sqrtf(sum_sq / (samples_in_current_window_being_finalized - 1));
                    } else {
                        std_mv_arr[current_window_idx] = 0.0f; 
                    }
                    xprintf("mean for %s window %d (size %d): x %.2f, y %.2f, z %.2f\nMagnitude info: mean %.2f, std %.3f\n",
                            (samples_in_current_window_being_finalized == windowSize ? "full" : "partial"),
                            current_window_idx + 1, samples_in_current_window_being_finalized,
                            mean_ax_arr[current_window_idx], mean_ay_arr[current_window_idx], mean_az_arr[current_window_idx],
                            mean_mv_arr[current_window_idx], std_mv_arr[current_window_idx]);
                }
            }
        } 

        data->mean_ax_arr = mean_ax_arr;
        data->mean_ay_arr = mean_ay_arr;
        data->mean_az_arr = mean_az_arr;
        data->mean_mv_arr = mean_mv_arr;
        data->std_mv_arr = std_mv_arr;
        
        if (numSamples > 0 && data->windowSize == 0 && windowSize > 0) {
             xprintln("All samples were in a partial window too small to process, or no windows were formed.");
        } else if (numSamples == 0 && bytesToRead > 0 && bytesToRead % 6 != 0) {
             xprint("Warning: Partial accelerometer sample data in FIFO (");
             xprint(bytesToRead % 6); xprintln(" bytes). Not enough for a full sample.");
        } else if (numSamples == 0 && bytesToRead == 0){
            // Handled by "FIFO is empty"
        } else if (numSamples == 0 && data->windowSize == 0){ 
             xprintln("No complete accelerometer samples to compute statistics, or all processed windows were invalid.");
        }


        if (bytesToRead > 0 && bytesToRead % 6 != 0 && numSamples > 0) { 
            xprint("Warning: Partial accelerometer sample data at end of FIFO (");
            xprint(bytesToRead % 6); xprintln(" bytes remaining).");
        }
    }

    // Reset the FIFO after reading its content to prepare for the next cycle
    // This is important to clear out old data and prevent re-reading it.
    writeMPURegister(MPU6050_RA_USER_CTRL, (1 << 6) | (1 << 2)); // FIFO_EN=1, FIFO_RESET=1
    delayMicroseconds(50); // Small delay for reset to take effect
    writeMPURegister(MPU6050_RA_USER_CTRL, (1 << 6) ); // FIFO_EN=1, FIFO_RESET=0
    xprintln("MPU FIFO Reset after reading. Ready for next collection cycle.");

    return data;
}