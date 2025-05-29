#include <MPU6050.h>
#include <config.h>
#include <Wire.h>
#include <Tools.h>
// #include <Adafruit_MPU6050.h> // REMOVE
// #include <Adafruit_Sensor.h> // REMOVE

// all the movement sensor methods
// and attributes

#ifdef MPU_INTERRUPT_PIN
volatile bool mpuFifoOverflowInterrupt = false; // Flag for MPU's FIFO overflow interrupt
#endif

// Store the currently set accelerometer sensitivity
static float currentMPUSensitivity = 16384.0f; // Default to +/- 2G, will be updated in mpuSetup
static uint8_t currentAccelRangeBits = MPU6050_ACCEL_FS_SEL_2G;


float getMPUSensitivity() {
    return currentMPUSensitivity;
}

// bool getMovAvail(Adafruit_MPU6050 *mpu){ // REMOVE
//     if(mpu){
//         return mpu->getMotionInterruptStatus();
//     }
//     return false;
// }

bool getMovAvail() { // ADD
    // Read the INT_STATUS register (0x3A)
    // Bit 6 is MOT_INT (Motion detection interrupt)
    // Bit 3 is WOM_INT (Wake-on-Motion interrupt) - check datasheet for which one you use.
    // Adafruit library's getMotionInterruptStatus() checks MOT_INT.
    // For cycle mode, you might be interested in other interrupt sources or simply data availability in FIFO.
    // If this function is meant to check if FIFO has data or if a specific motion interrupt triggered,
    // the logic needs to reflect that.
    // For now, let's assume it checks the general interrupt status register for any relevant bit.
    // This might need adjustment based on your specific interrupt setup for motion.
    // A simple check might be if there's data in the FIFO.
    uint8_t fifoCountBytes[2];
    readMPURegisters(MPU6050_RA_FIFO_COUNTH, 2, fifoCountBytes);
    uint16_t fifoCount = (fifoCountBytes[0] << 8) | fifoCountBytes[1];
    if (fifoCount > 0) return true; // Data is available

    // Or, if you have a specific motion interrupt configured on INT pin:
    // uint8_t intStatus = readMPURegister(MPU6050_RA_INT_STATUS);
    // return (intStatus & (1 << 6)); // Example: checks MOT_INT bit
    return false; // Placeholder - adjust based on your motion detection strategy
}


#ifdef MPU_INTERRUPT_PIN
// Optional ISR for MPU's FIFO Overflow Interrupt (Failsafe)
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

// void mpuSetup(Adafruit_MPU6050 *mpu){ // REMOVE
bool mpuSetup(){ // ADD
    xprintln("MPU6050 Low-Power Cycle Mode FIFO Test (Direct Wire.h) - ESP32-C3");
    xprintln("Collecting Accelerometer data then reading FIFO.");

    // Wire.begin(); // ESP32-C3 default I2C pins. This should be called once, typically in main setup.
                 // If Sensors_Processes::setup calls Wire.begin(), that's fine.

    // Check WHO_AM_I register
    uint8_t who_am_i = readMPURegister(MPU6050_RA_WHO_AM_I);
    if (who_am_i != 0x68 && who_am_i != 0x72 && who_am_i != 0x70) { // MPU6050 can show 0x68, some variants 0x72, ICM20602 shows 0x12 or 0xAF. Common MPU6050 should be 0x68.
        xprint("Failed to find MPU6050 chip. WHO_AM_I: 0x"); 
        xprintf("0x%02X\n", who_am_i);
        return false;
    }
    xprintln("MPU6050 Found!");

    // Reset device
    writeMPURegister(MPU6050_RA_PWR_MGMT_1, 0x80); // Set PWR_MGMT_1 to 0x80 to reset
    delay(100); // Wait for reset to complete


    // Set Accelerometer Range (important for sensitivity calculation later)
    // Example: MPU6050_RANGE_8_G from Adafruit library corresponds to AFS_SEL = 2
    currentAccelRangeBits = MPU6050_ACCEL_FS_SEL_8G; // Corresponds to +/- 8g
    writeMPURegister(MPU6050_RA_ACCEL_CONFIG, currentAccelRangeBits);
    
    uint8_t accelConfigVal = readMPURegister(MPU6050_RA_ACCEL_CONFIG);
    accelConfigVal = (accelConfigVal >> 3) & 0x03; // Extract AFS_SEL bits

    xprint("Accelerometer range set to: ");
    switch (accelConfigVal) {
        case 0: xprintln("+-2G"); currentMPUSensitivity = 16384.0f; break; // LSB/g for 2G
        case 1: xprintln("+-4G"); currentMPUSensitivity = 8192.0f; break;  // LSB/g for 4G
        case 2: xprintln("+-8G"); currentMPUSensitivity = 4096.0f; break;  // LSB/g for 8G
        case 3: xprintln("+-16G"); currentMPUSensitivity = 2048.0f; break; // LSB/g for 16G
        default: xprintln("Unknown"); currentMPUSensitivity = 4096.0f; break; // Default to 8G
    }


    // --- Configure MPU6050 for Low-Power Cycle Mode & Accel-Only FIFO ---
    // Wake up MPU6050 from sleep
    writeMPURegister(MPU6050_RA_PWR_MGMT_1, 0x00); // Clear SLEEP bit
    delay(50);


    // 1. Set Cycle Rate. This puts MPU in cycle mode, activates accelerometer
    //    at this rate, and typically powers down gyros & temp sensor.
    //    In PWR_MGMT_1 (0x6B), set CYCLE bit (bit 5) to 1.
    //    In PWR_MGMT_2 (0x6C), set LP_WAKE_CTRL (bits 7-6) for wake-up frequency.
    //    e.g., for 1.25Hz: LP_WAKE_CTRL = 0 (0b00xxxxxx)
    uint8_t pwr_mgmt_1_val = readMPURegister(MPU6050_RA_PWR_MGMT_1);
    pwr_mgmt_1_val |= (1 << 5); // Set CYCLE bit
    pwr_mgmt_1_val &= ~(1 << 6); // Ensure SLEEP bit is 0
    writeMPURegister(MPU6050_RA_PWR_MGMT_1, pwr_mgmt_1_val);
    
    // Set LP_WAKE_CTRL in PWR_MGMT_2. For 1.25 Hz, LP_WAKE_CTRL is 0.
    // Also, ensure gyro and accel are not in standby (STBY_XA, etc. bits should be 0)
    // For cycle mode, the datasheet implies STBY bits for Gyro will be effectively set by cycle mode.
    uint8_t pwr_mgmt_2_val = MPU6050_LP_WAKE_CTRL_1_25HZ << 6; // Shift to bits 7-6
    writeMPURegister(MPU6050_RA_PWR_MGMT_2, pwr_mgmt_2_val);

    float cycle_rate_hz = 1.25f;
    xprint("MPU6050 set to "); xprint(cycle_rate_hz); xprintln(" Hz cycle rate (accelerometer only).");

    // 2. Explicitly disable Temperature Sensor (TEMP_DIS bit in PWR_MGMT_1)
    pwr_mgmt_1_val = readMPURegister(MPU6050_RA_PWR_MGMT_1);
    pwr_mgmt_1_val |= (1 << 3); // Set TEMP_DIS bit (bit 3)
    writeMPURegister(MPU6050_RA_PWR_MGMT_1, pwr_mgmt_1_val);
    xprintln("Temperature sensor explicitly disabled in PWR_MGMT_1.");

    // For debugging: Check PWR_MGMT_1 and PWR_MGMT_2
    xprintf("  DEBUG: PWR_MGMT_1 = 0x%02X\n", readMPURegister(MPU6050_RA_PWR_MGMT_1));
    xprintf("  DEBUG: PWR_MGMT_2 = 0x%02X\n", readMPURegister(MPU6050_RA_PWR_MGMT_2));


    // 3. Configure User Control (USER_CTRL - 0x6A): Reset FIFO, then Enable FIFO.
    writeMPURegister(MPU6050_RA_USER_CTRL, 0b00000100); // Assert FIFO_RESET (clears old data)
    delay(50); // Short delay for reset
    writeMPURegister(MPU6050_RA_USER_CTRL, 0b01000000); // Assert FIFO_EN (FIFO_RESET is now 0)
    xprintln("FIFO Reset and Enabled.");

    // 4. Select ONLY Accelerometer Data for FIFO (FIFO_EN register - 0x23)
    writeMPURegister(MPU6050_RA_FIFO_EN, 0b00001000); // ACCEL_FIFO_EN (bit 3)
    xprintln("FIFO configured for Accelerometer data only.");

    #ifdef MPU_INTERRUPT_PIN
    // 5. (Optional Failsafe) Configure MPU INT Pin for FIFO Overflow Interrupt
    uint8_t intPinConfig = 0;
    intPinConfig |= (1 << 7); // ACTL: Active LOW
    intPinConfig |= (1 << 5); // LATCH_INT_EN: Latched interrupt
    writeMPURegister(MPU6050_RA_INT_PIN_CFG, intPinConfig);
    writeMPURegister(MPU6050_RA_INT_ENABLE, 0b00010000); // FIFO_OFLOW_EN (bit 4)
    xprintln("MPU INT pin configured for FIFO Overflow (Failsafe).");

    pinMode(MPU_INTERRUPT_PIN, INPUT); // External pull-up resistor is assumed if not on module
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), onMpuHardwareInterrupt, FALLING);
    xprintln("ESP32 interrupt armed for MPU FIFO Overflow.");
    float timeToOverflow = (1024.0f / 6.0f) / cycle_rate_hz; // 6 bytes for accel
    xprintf("  Approx. time for FIFO to overflow (accel only): %.1f seconds.\n", timeToOverflow);
    #else
    xprintln("MPU Hardware Interrupt for FIFO overflow is NOT configured in this sketch.");
    #endif

    xprintln("\nSetup complete. ESP32 will read MPU FIFO data periodically.");
    int expected_samples = cycle_rate_hz * (60000 / 1000.0); // Example for 1 minute
    int expected_bytes = expected_samples * 6; // 6 bytes per accel sample
    xprint("Expected samples per example collection cycle (1 min): ~"); xprintln(expected_samples);
    xprint("Expected bytes per example collection cycle (1 min): ~"); xprintln(expected_bytes);
    return true;
}


MPUData *readMPUData() {
    MPUData *data = new MPUData;
    // Initialize MPUData fields to nullptr and 0
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
        xprintln("Warning: FIFO count reported > 1024. Reading max 1024 bytes.");
        fifoCount = 1024;
    }
    // Ensure fifoCount is a multiple of 6 (one sample = 6 bytes for accel X, Y, Z)
    fifoCount = (fifoCount / 6) * 6;


    if (fifoCount == 0) {
        xprintln("FIFO is empty. No new data to read for this interval.");
        data->windowSize = 0; // No data, so no windows
    } else {
        uint8_t fifoBuffer[1024]; 
        uint16_t bytesToRead = fifoCount;

        readMPURegisters(MPU6050_RA_FIFO_R_W, bytesToRead, fifoBuffer);
        xprint("Read "); xprint(bytesToRead); xprintln(" bytes from FIFO.");

        int numSamples = bytesToRead / 6;
        xprint("Number of accelerometer samples processed: "); xprintln(numSamples);

        uint16_t num_total_windows = 0;
        uint8_t windowSize = WINDOWSIZE; // From config.h

        if (numSamples > 0 && windowSize > 0) {
            num_total_windows = (numSamples + windowSize - 1) / windowSize; 
        } else {
            num_total_windows = 0;
        }
        
        data->windowSize = num_total_windows; // Store actual number of windows processed

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
                // Free any partially allocated arrays
                if (mean_ax_arr) free(mean_ax_arr); if (mean_ay_arr) free(mean_ay_arr);
                if (mean_az_arr) free(mean_az_arr); if (mean_mv_arr) free(mean_mv_arr);
                if (std_mv_arr) free(std_mv_arr);
            }
        }


        float temp_mv_g[windowSize > 0 ? windowSize : 1] = {0}; 
        float sensitivity = getMPUSensitivity(); // Use the sensitivity set during setup

        for (uint16_t i = 0; i < numSamples; i++) { // Changed uint8_t to uint16_t for i
            uint16_t offset = i * 6; // Changed uint8_t to uint16_t for offset
            int16_t ax_raw = (int16_t)((fifoBuffer[offset + 0] << 8) | fifoBuffer[offset + 1]);
            int16_t ay_raw = (int16_t)((fifoBuffer[offset + 2] << 8) | fifoBuffer[offset + 3]);
            int16_t az_raw = (int16_t)((fifoBuffer[offset + 4] << 8) | fifoBuffer[offset + 5]);

            float ax_g = (float)ax_raw / sensitivity;
            float ay_g = (float)ay_raw / sensitivity;
            float az_g = (float)az_raw / sensitivity;
            
            if (windowSize > 0 && num_total_windows > 0) { 
                temp_mv_g[i % windowSize] = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

                uint16_t current_window_idx = i / windowSize; // Changed uint8_t to uint16_t
                if (current_window_idx < num_total_windows) { 
                    mean_ax_arr[current_window_idx] += ax_g;
                    mean_ay_arr[current_window_idx] += ay_g;
                    mean_az_arr[current_window_idx] += az_g;
                    mean_mv_arr[current_window_idx] += temp_mv_g[i % windowSize];
                }
            }

            bool is_last_sample_of_all = (i + 1 == numSamples);

            if (windowSize > 0 && num_total_windows > 0 && ((i + 1) % windowSize == 0 || is_last_sample_of_all)) {
                uint16_t current_window_idx = i / windowSize; // Changed uint8_t to uint16_t
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
                        
                        if (current_window_idx == data->windowSize -1 && data->windowSize > 0) { // Use data->windowSize which is num_total_windows
                           data->windowSize--; // Decrement the count of valid windows
                        }
                        mean_ax_arr[current_window_idx] = 0.0f;
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
        // data->windowSize is already set to num_total_windows, potentially decremented if last partial window was too small

        if (numSamples > 0 && data->windowSize == 0 && windowSize > 0) {
             xprintln("All samples were in a partial window too small to process, or no windows were formed.");
        } else if (numSamples == 0 && bytesToRead > 0 && bytesToRead % 6 != 0) {
             xprint("Warning: Partial accelerometer sample data in FIFO (");
             xprint(bytesToRead % 6); xprintln(" bytes). Not enough for a full sample.");
        } else if (numSamples == 0 && bytesToRead == 0){
            // Handled by "FIFO is empty"
        } else if (numSamples == 0 && data->windowSize == 0){ // Updated condition
             xprintln("No complete accelerometer samples to compute statistics, or all processed windows were invalid.");
        }


        if (bytesToRead > 0 && bytesToRead % 6 != 0 && numSamples > 0) { 
            xprint("Warning: Partial accelerometer sample data at end of FIFO (");
            xprint(bytesToRead % 6); xprintln(" bytes remaining).");
        }
    }


    writeMPURegister(MPU6050_RA_USER_CTRL, 0b01000100); // FIFO_EN=1, FIFO_RESET=1
    xprintln("MPU FIFO Reset. Ready for next collection cycle.");

    return data;
}