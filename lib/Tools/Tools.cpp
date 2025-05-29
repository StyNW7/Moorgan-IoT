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

// only use this to delay a very short time like microseconds, else use vTaskDelay
// it can hog the CPU resource if used for a long delay time
void delayMicrosecondsNonBlocking(uint32_t microseconds) {
    uint64_t start = esp_timer_get_time(); // Get the current time in µs
    while ((esp_timer_get_time() - start) < microseconds) {
        taskYIELD(); // Yield to allow other tasks to run
    }
}

/*----------------------------------
|                                  |
|   Development Mode Only Codes    |
|                                  |
----------------------------------*/
#ifdef DEVMODE
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

void xflush(){
    Serial.flush();
}
#else
// Add the definition for xflush when DEVMODE is not defined
void xflush() {
    // This version does nothing, as was the intent in the original header's #else block
}
#endif
