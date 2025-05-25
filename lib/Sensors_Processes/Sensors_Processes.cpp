#include <Sensors_Processes.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <config.h>
#include <Tools.h>
#include <Wire.h>
#include <cfloat>
#include <OneWire.h>
#include "DS18B20.h"


// MPU6050 Register Definitions
#define MPU6050_I2C_ADDRESS     0x68

#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_PWR_MGMT_2   0x6C
#define MPU6050_RA_USER_CTRL    0x6A
#define MPU6050_RA_FIFO_EN      0x23
#define MPU6050_RA_INT_PIN_CFG  0x37
#define MPU6050_RA_INT_ENABLE   0x38
#define MPU6050_RA_INT_STATUS   0x3A
#define MPU6050_RA_FIFO_COUNTH  0x72
#define MPU6050_RA_FIFO_COUNTL  0x73
#define MPU6050_RA_FIFO_R_W     0x74

// function declarations


#ifdef MPU_INTERRUPT_PIN
volatile bool mpuFifoOverflowInterrupt = false; // Flag for MPU's FIFO overflow interrupt
#endif

typedef struct {
    float temperature;
    // float 
} mainData;

typedef struct {
    DS18B20 *dsb;
} tempData;

Sensors_Processes* Sensors_Processes::instance = nullptr;
// SENSORSSSSSSSSSSSS
// all the sensors method and attributes

// Add the constructor definition here
Sensors_Processes::Sensors_Processes() {
    dsb = nullptr;
    mpu = nullptr;
}

Sensors_Processes* Sensors_Processes::getInstance() {
    // initialize sensor processes instance
    if (instance == nullptr) {
        instance = new Sensors_Processes();
    }

    return instance;
}

Sensors_Processes::~Sensors_Processes(){
    if(dsb){
        delete dsb;
        dsb = nullptr;
    }

    if(mpu){
        delete mpu;
        mpu = nullptr;
    }
}

void Sensors_Processes::setup(){
    if(dsb == nullptr){
        dsb = new DS18B20(new OneWire(DS18B20_PIN), DS18B20_RESOLUTION);
    }

    if(mpu == nullptr){
        mpu = new Adafruit_MPU6050();
    }

    
    dsb->begin();
    mpuSetup();
    // setup MAX30102
}

// all the temperature sensor methods
// and attributes
bool Sensors_Processes::getTempAvail(){
    if(dsb){
        return dsb->isConnected();
    }
    return false;
}

void Sensors_Processes::readTempData(){

    dsb->requestTemperatures();
    dsb->getTempC();
    
    while(!dsb->isConversionComplete()){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

// all the movement sensor methods
// and attributes

bool Sensors_Processes::getMovAvail(){
    if(mpu){
        return mpu->getMotionInterruptStatus();
    }
    return false;
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

void Sensors_Processes::mpuSetup(){
    xprintln("MPU6050 Low-Power Cycle Mode FIFO Test - ESP32-C3");
    xprintln("Collecting Accelerometer data for ~1 minute then reading FIFO.");

    Wire.begin(); // ESP32-C3 default I2C pins, or Wire.begin(SDA_PIN, SCL_PIN);

    if (!mpu->begin(MPU6050_I2C_ADDRESS, &Wire)) {
        xprintln("Failed to find MPU6050 chip. Halting.");
        while (1) delay(10);
    }
    xprintln("MPU6050 Found!");

    // Set Accelerometer Range (important for sensitivity calculation later)
    mpu->setAccelerometerRange(MPU6050_RANGE_2_G); // Or MPU6050_RANGE_4_G, _8_G, _16_G
    xprint("Accelerometer range set to: ");
    switch (mpu->getAccelerometerRange()) {
        case MPU6050_RANGE_2_G:  xprintln("+-2G");  break;
        case MPU6050_RANGE_4_G:  xprintln("+-4G");  break;
        case MPU6050_RANGE_8_G:  xprintln("+-8G");  break;
        case MPU6050_RANGE_16_G: xprintln("+-16G"); break;
    }


    // --- Configure MPU6050 for Low-Power Cycle Mode & Accel-Only FIFO ---

    // 1. Set Cycle Rate. This puts MPU in cycle mode, activates accelerometer
    //    at this rate, and typically powers down gyros & temp sensor.
    mpu->setCycleRate(MPU6050_CYCLE_1_25_HZ); // Accelerometer samples at 1.25 Hz
    float cycle_rate_hz = 1.25f;
    xprint("MPU6050 set to "); xprint(cycle_rate_hz); xprintln(" Hz cycle rate (accelerometer only).");

    // 2. Explicitly disable Temperature Sensor (TEMP_DIS bit in PWR_MGMT_1)
    //    While setCycleRate might imply temp sensor off, this ensures it.
    uint8_t pwr_mgmt_1 = readMPURegister(MPU6050_RA_PWR_MGMT_1);
    pwr_mgmt_1 |= (1 << 3); // Set TEMP_DIS bit (bit 3)
    writeMPURegister(MPU6050_RA_PWR_MGMT_1, pwr_mgmt_1);
    xprintln("Temperature sensor explicitly disabled in PWR_MGMT_1.");

    // For debugging: Check PWR_MGMT_1 and PWR_MGMT_2
    // PWR_MGMT_1 (0x6B): Expect CYCLE=1 (bit 5), SLEEP=0 (bit 6), TEMP_DIS=1 (bit 3)
    // PWR_MGMT_2 (0x6C): Expect Gyros STBY_XG,YG,ZG=1 (bits 2,1,0), LP_WAKE_CTRL set for rate
    xprintf("  DEBUG: PWR_MGMT_1 = 0x%02X\n", readMPURegister(MPU6050_RA_PWR_MGMT_1));
    xprintf("  DEBUG: PWR_MGMT_2 = 0x%02X\n", readMPURegister(MPU6050_RA_PWR_MGMT_2));


    // 3. Configure User Control (USER_CTRL - 0x6A): Reset FIFO, then Enable FIFO.
    //    Bit 6: FIFO_EN, Bit 2: FIFO_RESET (self-clearing)
    writeMPURegister(MPU6050_RA_USER_CTRL, 0b00000100); // Assert FIFO_RESET (clears old data)
    delay(50); // Short delay for reset
    writeMPURegister(MPU6050_RA_USER_CTRL, 0b01000000); // Assert FIFO_EN (FIFO_RESET is now 0)
    xprintln("FIFO Reset and Enabled.");

    // 4. Select ONLY Accelerometer Data for FIFO (FIFO_EN register - 0x23)
    //    Bit 3: ACCEL_FIFO_EN. All others (temp, gyro, slaves) = 0.
    writeMPURegister(MPU6050_RA_FIFO_EN, 0b00001000);
    xprintln("FIFO configured for Accelerometer data only.");

    #ifdef MPU_INTERRUPT_PIN
    // 5. (Optional Failsafe) Configure MPU INT Pin for FIFO Overflow Interrupt
    uint8_t intPinConfig = 0;
    intPinConfig |= (1 << 7); // ACTL: Active LOW
    intPinConfig |= (1 << 5); // LATCH_INT_EN: Latched interrupt
    // Consider if open-drain is needed based on your MPU module: intPinConfig |= (1 << 6);
    writeMPURegister(MPU6050_RA_INT_PIN_CFG, intPinConfig);
    writeMPURegister(MPU6050_RA_INT_ENABLE, 0b00010000); // FIFO_OFLOW_EN (bit 4)
    xprintln("MPU INT pin configured for FIFO Overflow (Failsafe).");

    pinMode(MPU_INTERRUPT_PIN, INPUT); // External pull-up resistor is assumed
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), onMpuHardwareInterrupt, FALLING);
    xprintln("ESP32 interrupt armed for MPU FIFO Overflow.");
    float timeToOverflow = (1024.0f / 6.0f) / cycle_rate_hz; // 6 bytes for accel
    xprintf("  Approx. time for FIFO to overflow (accel only): %.1f seconds.\n", timeToOverflow);
    #else
    xprintln("MPU Hardware Interrupt for FIFO overflow is NOT configured in this sketch.");
    #endif

    xprintln("\nSetup complete. ESP32 will read MPU FIFO data every ~60 seconds.");
    int expected_samples = cycle_rate_hz * (60000 / 1000.0);
    int expected_bytes = expected_samples * 6; // 6 bytes per accel sample
    xprint("Expected samples per collection cycle: ~"); xprintln(expected_samples);
    xprint("Expected bytes per collection cycle: ~"); xprintln(expected_bytes);
}


// Read MPU6050 data from FIFO, process accelerometer data. this method is 
// a one time read method, and it reads data from multiple reading intervals 
// stored in the FIFO. It is designed to be called from a main loop with delay 
// (presumably from esp light sleep timer). Recommend to be called periodically
MPUData Sensors_Processes::readMPUData() {
    // esp_timer_get_time() can also be used for ESP sleeping applications,
    MPUData data;

    #ifdef MPU_INTERRUPT_PIN
    // Check for MPU's own FIFO overflow interrupt (optional failsafe)
    if (mpuFifoOverflowInterrupt) {
        mpuFifoOverflowInterrupt = false; // Reset flag
        xprintln("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        xprintln("!! MPU HARDWARE INTERRUPT: FIFO Overflowed before 1-min timer!  !!");
        xprintln("!! Data might have been lost or ESP32 didn't read in time.      !!");
        uint8_t intStatus = readMPURegister(MPU6050_RA_INT_STATUS); // Clear interrupt on MPU
        xprintf("!! MPU INT_STATUS: 0x%02X                                       !!\n", intStatus);
        xprintln("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        // Consider triggering an immediate read or specific error handling here
    }
    #endif

    // ESP32 timed data collection
    xprintln("\n--- Collecting data from MPU6050 FIFO ---");

    // 1. Get FIFO count
    uint8_t fifoCountBytes[2];
    readMPURegisters(MPU6050_RA_FIFO_COUNTH, 2, fifoCountBytes);
    uint16_t fifoCount = (fifoCountBytes[0] << 8) | fifoCountBytes[1];
    xprint("Current FIFO Count: "); xprint(fifoCount); xprintln(" bytes.");

    if (fifoCount > 1024) { // Should ideally not happen
        xprintln("Warning: FIFO count reported > 1024. Reading max 1024 bytes.");
        fifoCount = 1024;
    }

    if (fifoCount == 0) {
        xprintln("FIFO is empty. No new data to read for this interval.");
    } else {
        uint8_t fifoBuffer[1024]; // Max possible FIFO size
        uint16_t bytesToRead = fifoCount;

        readMPURegisters(MPU6050_RA_FIFO_R_W, bytesToRead, fifoBuffer);
        xprint("Read "); xprint(bytesToRead); xprintln(" bytes from FIFO.");

        // Accelerometer data is 6 bytes/sample (ax_h, ax_l, ay_h, ay_l, az_h, az_l)
        int numSamples = bytesToRead / 6;
        xprint("Number of accelerometer samples processed: "); xprintln(numSamples);

        uint8_t windowSize = 12; //sample
        uint16_t num_total_windows = 0;

        if (numSamples > 0 && windowSize > 0) { // Check windowSize > 0 to prevent division by zero
            num_total_windows = (numSamples + windowSize - 1) / windowSize;
        }

        // window feature array 
        float *mean_ax_arr = (float *)calloc(num_total_windows, sizeof(float));
        float *mean_ay_arr = (float *)calloc(num_total_windows, sizeof(float));
        float *mean_az_arr = (float *)calloc(num_total_windows, sizeof(float));
        float *mean_mv_arr = (float *)calloc(num_total_windows, sizeof(float));
        float *std_mv_arr = (float *)calloc(num_total_windows, sizeof(float));

        // buffer to store feature temporarily
        float temp_mv_g[windowSize] = {0};
        
        // each window consist of that amount of sample, then it will be summed together to find
        // vm stdev, vm mean, x mean, y mean, z mean 

        for (uint8_t i = 0; i < numSamples; i++) {
            uint8_t offset = i * 6;
            int16_t ax_raw = (int16_t)((fifoBuffer[offset + 0] << 8) | fifoBuffer[offset + 1]);
            int16_t ay_raw = (int16_t)((fifoBuffer[offset + 2] << 8) | fifoBuffer[offset + 3]);
            int16_t az_raw = (int16_t)((fifoBuffer[offset + 4] << 8) | fifoBuffer[offset + 5]);

            // Sensitivity for MPU6050_RANGE_8_G: 4096 LSB/g
            float ax_g = (float)ax_raw / 4096.0f;
            float ay_g = (float)ay_raw / 4096.0f;
            float az_g = (float)az_raw / 4096.0f;
            temp_mv_g[i % windowSize] = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

            // Use integer division for floor division in C++ (since both i and windowSize are integers)
            mean_ax_arr[i / windowSize] += ax_g;
            mean_ay_arr[i / windowSize] += ay_g;
            mean_az_arr[i / windowSize] += az_g;
            mean_mv_arr[i / windowSize] += temp_mv_g[i % windowSize];

            if((i+1) % windowSize == 0){
                mean_ax_arr[i / windowSize] /= windowSize;
                mean_ay_arr[i / windowSize] /= windowSize;
                mean_az_arr[i / windowSize] /= windowSize;
                mean_mv_arr[i / windowSize] /= windowSize;
                // Calculate standard deviation of mv_g for this window
                float sum_sq = 0.0f;
                for (uint8_t j = 0; j < windowSize; j++) {
                    sum_sq += (temp_mv_g[j] - mean_mv_arr[i / windowSize]) * (temp_mv_g[j] - mean_mv_arr[i / windowSize]);
                }
                std_mv_arr[i / windowSize] = sqrtf(sum_sq / (windowSize-1));
                // std_mv_arr = sqrtf(sumsq_ax / numSamples - mean_ax * mean_ax);
                xprintf("mean for window %d : x %.2f, y %.2f, z %.2f\nMagnitude info: mean %.2f, std %f", (i / windowSize)+1, mean_ax_arr[i/windowSize], mean_ay_arr[i/windowSize], mean_az_arr[i/windowSize], mean_mv_arr[i/windowSize], std_mv_arr[i/windowSize]);
            } else if(i+1 >= numSamples){
                uint8_t partialWindowSize = (i+1) % windowSize;
                mean_ax_arr[i/windowSize] /= partialWindowSize;
                mean_ay_arr[i/windowSize] /= partialWindowSize;
                mean_az_arr[i/windowSize] /= partialWindowSize;
                mean_mv_arr[i/windowSize] /= partialWindowSize;
                float sum_sq = 0.0f;
                for (uint8_t j = 0; j < partialWindowSize; j++) {
                    sum_sq += (temp_mv_g[j] - mean_mv_arr[i / windowSize]) * (temp_mv_g[j] - mean_mv_arr[i / windowSize]);
                }
                std_mv_arr[i / windowSize] = sqrtf(sum_sq / (partialWindowSize-1));
                xprintf("mean for window %d : x %.2f, y %.2f, z %.2f\nMagnitude info: mean %.2f, std %f", (i / windowSize)+1, mean_ax_arr[i/windowSize], mean_ay_arr[i/windowSize], mean_az_arr[i/windowSize], mean_mv_arr[i/windowSize], std_mv_arr[i/windowSize]);
            }
        }
            // save the data to the data struct/object and return this object
            data.mean_ax_arr = mean_ax_arr;
            data.mean_ay_arr = mean_ay_arr;
            data.mean_az_arr = mean_az_arr;
            data.mean_mv_arr = mean_mv_arr;
            data.std_mv_arr = std_mv_arr;
            data.windowSize = num_total_windows;
        
        if (numSamples == 0) {
            xprintln("No accelerometer samples to compute statistics.");
        }

        if (bytesToRead % 6 != 0 && bytesToRead > 0) { // Check for partial packet if not empty
            xprint("Warning: Partial accelerometer sample data at end of FIFO (");
            xprint(bytesToRead % 6); xprintln(" bytes remaining).");
        }
    }

    // 2. Reset the MPU FIFO to prepare for the next collection cycle.
    //    This ensures FIFO_EN remains 1, and FIFO_RESET (self-clearing) is triggered.
    writeMPURegister(MPU6050_RA_USER_CTRL, 0b01000100); // FIFO_EN=1, FIFO_RESET=1
    xprintln("MPU FIFO Reset. Ready for next collection cycle.");

    // Fill data fields here as needed, for example:
    // (Make sure MPUData struct/class has these fields and proper ownership/cleanup.)

    return data;
}

// all the heart rate and oxygen sensor methods
// and attributes


