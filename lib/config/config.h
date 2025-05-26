#define DEVMODE
#define PROVISIONING_SERVICE_NAME "PROV_moorgan_colar" // service name have to start with PROV_

// temperature sensor
#define DS18B20_PIN 4
#define DS18B20_RESOLUTION 10

// mpu gyro accel sensor
// #define MPU_INTERRUPT_PIN 5
#define WINDOWSIZE 6

// max hr spo sensor configuration constants
#define MAX30102_INT_PIN 4
#define DISCARD_DURATION_MS 5000  // 5 seconds for sensor/signal to stabilize
#define ACTIVE_READ_DURATION_MS 30000 // 30 seconds for actual data collection
#define TOTAL_OPERATION_DURATION_MS (DISCARD_DURATION_MS + ACTIVE_READ_DURATION_MS) // 35 seconds
#define SAMPLES_PER_SECOND 50
#define PPG_BUFFER_SIZE (SAMPLES_PER_SECOND*(ACTIVE_READ_DURATION_MS/1000))

// wifi connection measurements
#define WIFIMEASUREDPOWER -30
#define PATHLOSSEXPONENT 2.3f
#define CONNECTIONRETRYCOUNT 20
#define NTPSERVER "pool.ntp.org"
#define GMTOFFSET_INDO (7*3600)

#define MICROSECOND_SLEEP (60ULL * 1000000ULL) // 1 minute in microseconds
