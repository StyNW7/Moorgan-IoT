#include <Arduino.h>
#include <Tools.h>
#include <config.h>
#include "UUIDs.h"
#include <esp_system.h>
#include <cstring>

UUIDs::UUIDs() {
    memset(this->uuid, 0, sizeof(this->uuid));
}

uint8_t* UUIDs::getUUID() {
    return this->uuid;
}

size_t UUIDs::getSize() {
    return sizeof(this->uuid);
}

void UUIDs::setUUID(const uint8_t uuid[16]) {
    memcpy(this->uuid, uuid, sizeof(this->uuid));
}

void UUIDs::generate() {
    esp_fill_random(this->uuid, sizeof(this->uuid));
    // Set version to 4 -- truly random
    this->uuid[6] = (this->uuid[6] & 0x0F) | 0x40;
    // Set variant to 10xxxxxx
    this->uuid[8] = (this->uuid[8] & 0x3F) | 0x80;
}

char* UUIDs::getStringc() {
    char* uuid_str = (char*)malloc(37); // 36 characters + null terminator
    if (uuid_str) {
        snprintf(uuid_str, 37,
                 "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                 this->uuid[0], this->uuid[1], this->uuid[2], this->uuid[3],
                 this->uuid[4], this->uuid[5], this->uuid[6], this->uuid[7],
                 this->uuid[8], this->uuid[9], this->uuid[10], this->uuid[11],
                 this->uuid[12], this->uuid[13], this->uuid[14], this->uuid[15]);
    }
    return uuid_str;
}

void UUIDs::print() {
#ifdef DEVMODE
    char* uuid_str = this->getStringc();
    if (uuid_str) {
        xprint(uuid_str);
        free(uuid_str);
    }
#endif
}

bool UUIDs::isUnset() const {
    for (int i = 0; i < sizeof(this->uuid); ++i) {
        if (uuid[i] != 0) return false;
    }
    return true;
}