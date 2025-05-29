#pragma once
#include <Arduino.h>
#include <config.h>

#ifdef DEVMODE
void checkFlashPSRAM();
#endif

void playTone(uint8_t);
void playTone2(uint8_t pin);
void delayMicrosecondsNonBlocking(uint32_t microseconds);

// Template debug print functions
#ifdef DEVMODE
template<typename T>
void xprint(const T& value) {
    Serial.print(value);
}

template<typename T>
void xprintln(const T& value) {
    Serial.println(value);
}

inline void xprintf(const char* format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    Serial.print(buf);
}

void xflush(); // Declaration
#else
template<typename T>
void xprint(const T&) {}
template<typename T>
void xprintln(const T&) {}
inline void xprintf(const char*, ...) {}
void xflush(); // Declaration
#endif