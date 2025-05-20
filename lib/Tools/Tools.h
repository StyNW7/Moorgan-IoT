#include <Arduino.h>
#include <config.h>

#ifdef DEVMODE
void checkFlashPSRAM();
#endif

void playTone(uint8_t);
void playTone2(uint8_t pin);
void delayMicrosecondsNonBlocking(uint32_t microseconds);

template<typename T>
void xprint(const T& value);

template<typename T>
void xprintln(const T& value);

void xprintf(const char* format, ...);