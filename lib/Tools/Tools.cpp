#include <Arduino.h>
#include <config.h>
#include <esp_system.h>


void playTone(uint8_t pin){
    tone(pin, 440, 150);
    vTaskDelay(pdMS_TO_TICKS(150));
    tone(pin, 523, 150);
    vTaskDelay(pdMS_TO_TICKS(150));
    tone(pin, 659, 150);
    vTaskDelay(pdMS_TO_TICKS(200));
    noTone(pin);
}

void playTone2(uint8_t pin) {
  tone(pin, 500, 200); // First tone, 500 Hz for 200 ms
  vTaskDelay(pdMS_TO_TICKS(250));                 // Pause between tones
  tone(pin, 400, 200); // Second tone, 400 Hz for 200 ms
  vTaskDelay(pdMS_TO_TICKS(250));                 // Pause between tones
  tone(pin, 300, 200); // Third tone, 300 Hz for 200 ms
  vTaskDelay(pdMS_TO_TICKS(250));
  noTone(pin);                 // Pause after the sequence
}

void delayMicrosecondsNonBlocking(uint32_t microseconds) {
    uint64_t start = esp_timer_get_time(); // Get the current time in µs
    while ((esp_timer_get_time() - start) < microseconds) {
        taskYIELD(); // Yield to allow other tasks to run
    }
}


void generateUUID(uint8_t uuid[16]) {
    esp_fill_random(uuid, sizeof(uuid));
    // Set version to 4 -- truly random
    uuid[6] = (uuid[6] & 0x0F) | 0x40;
    // Set variant to 10xxxxxx
    uuid[8] = (uuid[8] & 0x3F) | 0x80;
}

/*----------------------------------
|                                  |
|   Development Mode Only Codes    |
|                                  |
----------------------------------*/

#ifdef DEVMODE
template<typename T>
void xprintln(const T& value) {
    Serial.println(value);
}

template<typename T>
void xprint(const T& value) {
    Serial.print(value);
}

inline void xprintf(const char* format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    Serial.print(buf);
}

void checkFlashPSRAM(){
        if (psramFound()) {
        Serial.println("PSRAM is enabled and available!");
        Serial.print("PSRAM size is ");
        Serial.println(ESP.getPsramSize());
    } else {
        Serial.println("PSRAM is not available.");
    }

    Serial.print("The flash size is ");
    Serial.println(ESP.getFlashChipSize());
}

#else
template<typename T>
void xprintln(const T& value);

template<typename T>
void xprint(const T& value);

void xprintf(const char* format, ...);
#endif
