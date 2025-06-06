#pragma once
// #define DEVMODE
// #define TESTINGMODE
#define NO_PROVISIONING
#define PROVISIONING_SERVICE_NAME "PROV_moorgan_colar" // service name have to start with PROV_
#define PRE_REGISTERED_DEVICE_UUID "70e3adae-7d64-409b-8e6c-213416f2ea72" // UUID of the device, can be found in Azure IoT Hub -> Devices -> Your Device -> Device ID
// UUID can be changed, for now no azure auto provisioning is implemented
// uploading interval
#define DEFAULTUPLOADINTERVAL 2 // don't forget to set this back to 60

// temperature sensor
#define DS18B20_PIN 10
#define DS18B20_RESOLUTION 12
#define CALIBRATE_DS18B20 1.5f // set to 1 to calibrate DS18B20 sensor, 0 to skip calibration

// mpu gyro accel sensor
// #define MPU_INTERRUPT_PIN 5
#define WINDOWSIZE 6

// max hr spo sensor configuration constants
#define MAX30102_INT_PIN 3
#define DISCARD_DURATION_MS 5000  // 5 seconds for sensor/signal to stabilize
#define ACTIVE_READ_DURATION_MS 30000 // 30 seconds for actual data collection
#define TOTAL_OPERATION_DURATION_MS (DISCARD_DURATION_MS + ACTIVE_READ_DURATION_MS) // 35 seconds
#define SAMPLES_PER_SECOND 50
#define PPG_BUFFER_SIZE (SAMPLES_PER_SECOND*(ACTIVE_READ_DURATION_MS/1000))
#define PPG_READ_EVERY 1

// WiFi and Provisioning settings
#define TIMEOUTPROVISION (5 * 60 * 1000UL) // 5 minutes in milliseconds
#define PROVISIONSLEEPT (60ULL * 1000000ULL) // 1 minute in microseconds

// wifi connection measurements
#define WIFIMEASUREDPOWER -30
#define PATHLOSSEXPONENT 2.3f
#define CONNECTIONRETRYCOUNT 20
#define NTPSERVER "pool.ntp.org"
#define NTPSERVER2 "time.nist.gov"
#define GMTOFFSET_INDO (7*3600)

// Battery percentage info line
#define BATTERYADCPIN 4

// Sleep settings
#define MICROSECOND_SLEEP (60ULL*1000000ULL) // 1 minute in microseconds

// MQTT connection settings
#define MQTT_KEEP_ALIVE_MINUTES 29 // MQTT Keep-Alive setting for the SDK
// #define IOTHUB_CONNECTION_STRING "YOUR_DEVICE_CONNECTION_STRING" // Get this from Azure portal: IoT Hub -> Devices -> Your Device -> Primary Connection String
// connection string is set in the secreets.h file
#define GMT_OFFSET_SEC (7 * 3600) // WIB (Western Indonesian Time) is UTC+7
#define DAYLIGHT_OFFSET_SEC 0 
        