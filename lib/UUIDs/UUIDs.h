#pragma once
#include <Arduino.h>

class UUIDs {
private:
    uint8_t uuid[16];
public:
    UUIDs();
    uint8_t* getUUID();
    size_t getSize();
    void setUUID(const uint8_t uuid[16]);
    void generate();
    char* getStringc();
    void print();
    bool isUnset() const;
};