#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>
#include <iothubtransportmqtt.h>
#include <AzureIoTUtility.h>
#include <ArduinoJson.h>
#include <secreets.h>
#include "time.h"
#include "esp_sntp.h"
#include "Wire.h"
#include "esp_sleep.h"
#include <math.h>

// --- Azure IoT Hub Root CA Certificate ---
extern const char* AZURE_IOT_ROOT_CA_CERTIFICATE;

// --- Configuration from config.h ---
#define MAX30102_INT_PIN 3
#define SAMPLES_PER_SECOND 50
#define ACTIVE_READ_DURATION_S 20
#define PPG_BUFFER_SIZE (SAMPLES_PER_SECOND * ACTIVE_READ_DURATION_S)
#define NTPSERVER "pool.ntp.org"
#define GMT_OFFSET_SEC (7 * 3600)
#define DAYLIGHT_OFFSET_SEC 0
#define MQTT_KEEP_ALIVE_MINUTES 29
#define MICROSECOND_SLEEP (60ULL*1000000ULL)
#define MICROSECOND_SLEEP_SHORT_FAILURE (30ULL * 1000000ULL)

// --- RTC Data ---
RTC_DATA_ATTR bool rtc_is_first_boot = true;

// --- MAX30102 Class and Defines ---
#define MAX30102_ADDRESS 0x57
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_REG_MODE_CONFIG 0x09
#define MAX30102_REG_SPO2_CONFIG 0x0A
#define MAX30102_REG_LED1_PA 0x0C
#define MAX30102_REG_LED2_PA 0x0D
#define MAX30102_REG_PART_ID 0xFF
#define MAX30102_REG_FIFO_WR_PTR 0x04
#define MAX30102_REG_FIFO_RD_PTR 0x06

struct ppgSample {
    uint32_t ir;
    uint32_t red;
};

class MAX30102 {
public:
    ppgSample ppgDataBuffer[PPG_BUFFER_SIZE];

    bool writeRegister(uint8_t reg_addr, uint8_t value) {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(reg_addr);
        Wire.write(value);
        return Wire.endTransmission() == 0;
    }

    bool readRegister(uint8_t reg_addr, uint8_t *value) {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(reg_addr);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom((uint8_t)MAX30102_ADDRESS, (uint8_t)1) != 1) return false;
        *value = Wire.read();
        return true;
    }

    bool readBurst(uint8_t start_reg_addr, uint8_t *buffer, uint8_t count) {
        Wire.beginTransmission(MAX30102_ADDRESS);
        Wire.write(start_reg_addr);
        if (Wire.endTransmission(false) != 0) return false;
        if ((uint8_t)Wire.requestFrom((uint8_t)MAX30102_ADDRESS, count) != count) return false;
        for (int i = 0; i < count; i++) {
            buffer[i] = Wire.read();
        }
        return true;
    }
    
    void setup() {
        Wire.begin();
        uint8_t part_id;
        if (!readRegister(MAX30102_REG_PART_ID, &part_id) || part_id != 0x15) {
            Serial.println("MAX30102 not found!");
            return;
        }
        writeRegister(0x09, 0x40); delay(100);
        writeRegister(0x08, 0x0F);
        writeRegister(0x09, 0x03);
        writeRegister(0x0A, 0x27);
        writeRegister(0x0C, 0x24);
        writeRegister(0x0D, 0x24);
        writeRegister(0x11, 0x21);
    }
    
    void readFifo(ppgSample* buffer, uint16_t &sampleCount) {
        uint8_t readPointer, writePointer;
        readRegister(MAX30102_REG_FIFO_RD_PTR, &readPointer);
        readRegister(MAX30102_REG_FIFO_WR_PTR, &writePointer);
        int numSamplesInFifo = writePointer - readPointer;
        if (numSamplesInFifo < 0) numSamplesInFifo += 32;

        for (int i = 0; i < numSamplesInFifo; i++) {
            if (sampleCount < PPG_BUFFER_SIZE) {
                uint8_t temp_buffer[6];
                readBurst(MAX30102_REG_FIFO_DATA, temp_buffer, 6);
                buffer[sampleCount].red = (((uint32_t)temp_buffer[0] & 0x03) << 16) | ((uint32_t)temp_buffer[1] << 8) | temp_buffer[2];
                buffer[sampleCount].ir = (((uint32_t)temp_buffer[3] & 0x03) << 16) | ((uint32_t)temp_buffer[4] << 8) | temp_buffer[5];
                sampleCount++;
            }
        }
    }

    void purgeFifo() {
        uint8_t readPointer, writePointer;
        readRegister(MAX30102_REG_FIFO_RD_PTR, &readPointer);
        readRegister(MAX30102_REG_FIFO_WR_PTR, &writePointer);
        while (readPointer != writePointer) {
             uint8_t temp_buffer[6];
             readBurst(MAX30102_REG_FIFO_DATA, temp_buffer, 6);
             readRegister(MAX30102_REG_FIFO_RD_PTR, &readPointer);
        }
    }
};

// --- IOTHubInstance ---
static IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle;
static bool iotHubConnected = false;

static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback) {
    iotHubConnected = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    Serial.println(iotHubConnected ? "Connected to Azure IoT Hub" : "Disconnected from Azure IoT Hub");
}
static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback) {
    Serial.println(result == IOTHUB_CLIENT_CONFIRMATION_OK ? "Send confirmation OK" : "Send confirmation FAILED");
}
void setupAzureIoT() {
    if (platform_init() != 0) { Serial.println("Failed to initialize the platform."); return; }
    iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(IOTHUB_CONNECTION_STRING, MQTT_Protocol);
    if (iotHubClientHandle == NULL) { Serial.println("Failed to create IoT Hub client"); return; }
    if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_TRUSTED_CERT, AZURE_IOT_ROOT_CA_CERTIFICATE) != IOTHUB_CLIENT_OK) {
        Serial.println("Failed to set trusted CA certificate option.");
    }
    int keepAlive = MQTT_KEEP_ALIVE_MINUTES * 60;
    IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_KEEP_ALIVE, &keepAlive);
    IoTHubClient_LL_SetConnectionStatusCallback(iotHubClientHandle, connectionStatusCallback, NULL);
}
void sendToAzure(const char* telemetryData) {
    if (!iotHubConnected) { Serial.println("Not connected to Azure, cannot send message."); return; }
    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(telemetryData);
    if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, messageHandle, sendConfirmationCallback, NULL) != IOTHUB_CLIENT_OK) {
        Serial.println("Failed to send message");
    }
    IoTHubMessage_Destroy(messageHandle);
}

MAX30102 max30102;

void goToDeepSleep(uint64_t sleep_time_us) {
    Serial.printf("Entering deep sleep for %llu seconds.\n", sleep_time_us / 1000000ULL);
    esp_sleep_enable_timer_wakeup(sleep_time_us);
    esp_deep_sleep_start();
}
void setup() {
    Serial.begin(115200);
    Serial.println(rtc_is_first_boot ? "\n--- First Boot ---" : "\n--- Woke from Deep Sleep ---");
    if(rtc_is_first_boot) rtc_is_first_boot = false;

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) { delay(500); Serial.print("."); retries++; }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) { Serial.println("Failed to connect to WiFi. Entering deep sleep."); goToDeepSleep(MICROSECOND_SLEEP_SHORT_FAILURE); }
    Serial.println("WiFi connected!");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTPSERVER);
    setupAzureIoT();
    Serial.println("Attempting to connect to Azure IoT Hub...");
    unsigned long connectStart = millis();
    while(!iotHubConnected && (millis() - connectStart < 15000)) { IoTHubClient_LL_DoWork(iotHubClientHandle); delay(100); }
    if (!iotHubConnected) { Serial.println("Failed to connect to Azure IoT Hub. Entering deep sleep."); goToDeepSleep(MICROSECOND_SLEEP_SHORT_FAILURE); }
    
    max30102.setup();
    Serial.println("Setup complete.");
}

void loop() {
    Serial.println("\n>>> Starting Main Logic Cycle <<<");
    
    Serial.println("Purging old data from sensor buffer...");
    max30102.purgeFifo();

    Serial.printf("Starting %d-second data gathering...\n", ACTIVE_READ_DURATION_S);
    uint16_t sampleCount = 0;
    unsigned long startTime = millis();
    while (millis() - startTime < (ACTIVE_READ_DURATION_S * 1000)) {
        max30102.readFifo(max30102.ppgDataBuffer, sampleCount);
        delay(20);
    }
    Serial.printf("Gathered %d new samples.\n", sampleCount);

    if (sampleCount > SAMPLES_PER_SECOND * 2) { 
        // --- Improved Heart Rate and SpO2 Calculation ---
        // Moving average filter for IR signal (window size 5)
        float ir_filtered[PPG_BUFFER_SIZE];
        for (int i = 0; i < sampleCount; i++) {
            float sum = 0;
            int count = 0;
            for (int j = max(0, i - 2); j <= min(sampleCount - 1, i + 2); j++) {
                sum += max30102.ppgDataBuffer[j].ir;
                count++;
            }
            ir_filtered[i] = sum / count;
        }

        // --- Heart Rate Calculation ---
        uint32_t ir_max = 0, ir_min = UINT32_MAX;
        for (int i = 0; i < sampleCount; i++) {
            if (ir_filtered[i] > ir_max) ir_max = ir_filtered[i];
            if (ir_filtered[i] < ir_min) ir_min = ir_filtered[i];
        }
        
        // Adaptive threshold for peak detection
        float threshold = ir_min + (ir_max - ir_min) * 0.6f;
        int peak_count = 0;
        int peak_positions[100]; // Store peak indices
        int peak_index = 0;

        // Detect peaks with validation
        for (int i = 2; i < sampleCount - 2; i++) {
            if (ir_filtered[i] > threshold &&
                ir_filtered[i] > ir_filtered[i-1] &&
                ir_filtered[i] > ir_filtered[i+1] &&
                ir_filtered[i] > ir_filtered[i-2] &&
                ir_filtered[i] > ir_filtered[i+2]) {
                // Validate peak interval (min 200ms at 50Hz = 10 samples)
                if (peak_index == 0 || (i - peak_positions[peak_index-1] > 10)) {
                    peak_positions[peak_index++] = i;
                    peak_count++;
                }
            }
        }

        // Calculate average peak-to-peak interval
        float avg_interval = 0;
        if (peak_count > 1) {
            for (int i = 1; i < peak_count; i++) {
                avg_interval += (peak_positions[i] - peak_positions[i-1]);
            }
            avg_interval /= (peak_count - 1);
        }

        float heartRate = 0;
        if (avg_interval > 0) {
            heartRate = (60.0f * SAMPLES_PER_SECOND) / avg_interval;
        }
        if (heartRate > 180 || heartRate < 40 || peak_count < 2) {
            heartRate = 0; // Invalid heart rate
        }

        // --- SpO2 Calculation ---
        // Calculate RMS for AC components
        float ac_ir_sum = 0, ac_red_sum = 0;
        float dc_ir = 0, dc_red = 0;
        uint32_t red_max = 0, red_min = UINT32_MAX;

        for (int i = 0; i < sampleCount; i++) {
            dc_ir += max30102.ppgDataBuffer[i].ir;
            dc_red += max30102.ppgDataBuffer[i].red;
            if (max30102.ppgDataBuffer[i].red > red_max) red_max = max30102.ppgDataBuffer[i].red;
            if (max30102.ppgDataBuffer[i].red < red_min) red_min = max30102.ppgDataBuffer[i].red;
        }
        dc_ir /= sampleCount;
        dc_red /= sampleCount;

        for (int i = 0; i < sampleCount; i++) {
            float ir_diff = max30102.ppgDataBuffer[i].ir - dc_ir;
            float red_diff = max30102.ppgDataBuffer[i].red - dc_red;
            ac_ir_sum += ir_diff * ir_diff;
            ac_red_sum += red_diff * red_diff;
        }
        float ac_ir_rms = sqrt(ac_ir_sum / sampleCount);
        float ac_red_rms = sqrt(ac_red_sum / sampleCount);

        float R = (ac_red_rms / dc_red) / (ac_ir_rms / dc_ir);
        float spo2 = 110.0f - 25.0f * R; // Improved empirical formula
        if (spo2 > 100 || spo2 < 70 || ac_ir_rms < 100 || ac_red_rms < 100) {
            spo2 = 0; // Invalid SpO2 due to low signal quality
        }

        // --- Signal Quality Check ---
        bool valid_data = (heartRate > 0 || spo2 > 0) && (ir_max - ir_min > 1000) && (red_max - red_min > 1000);

        // --- JSON and Sending ---
        if (valid_data) {
            JsonDocument jsonDoc;
            jsonDoc["heartRate"] = round(heartRate * 10) / 10.0;
            jsonDoc["spo2"] = round(spo2 * 10) / 10.0;
            char telemetryData[256];
            serializeJson(jsonDoc, telemetryData);

            Serial.print("Sending Telemetry: "); Serial.println(telemetryData);
            sendToAzure(telemetryData);

            Serial.println("Running DoWork for 5 seconds to ensure message delivery...");
            unsigned long doWorkStart = millis();
            while(millis() - doWorkStart < 5000) { IoTHubClient_LL_DoWork(iotHubClientHandle); delay(100); }
        } else {
            Serial.println("Invalid data: insufficient signal quality.");
        }
    } else {
        Serial.println("Not enough samples collected to send telemetry.");
    }
    
    goToDeepSleep(MICROSECOND_SLEEP);
}


const char* AZURE_IOT_ROOT_CA_CERTIFICATE = \
// DigiCert Global Root G2
// Source: User-uploaded file DigiCertGlobalRootG2.txt
"-----BEGIN CERTIFICATE-----\n" \
"MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n" \
"2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n" \
"1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n" \
"q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n" \
"tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n" \
"vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n" \
"BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n" \
"5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n" \
"1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n" \
"NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n" \
"Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n" \
"8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n" \
"pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n" \
"MrY=\n" \
"-----END CERTIFICATE-----\n" \
// Microsoft RSA Root Certificate Authority 2017
// Source: User-uploaded file Microsoft RSA Root Certificate Authority 2017.txt
"-----BEGIN CERTIFICATE-----\n" \
"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\n" \
"RA+GsCyRxj3qrg+E\n" \
"-----END CERTIFICATE-----\n";