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
        uuid[0], uuid[1], uuid[2], uuid[3],
        uuid[4], uuid[5],
        uuid[6], uuid[7],
        uuid[8], uuid[9],
        uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
    }
    return uuid_str;
}

bool UUIDs::fromString(const char* uuid_str) {
    if (!uuid_str || strlen(uuid_str) != 36)
        return false;

    unsigned int vals[16];
    int parsed = sscanf(uuid_str,
        "%8x-%4x-%4x-%4x-%12x",
        &vals[0], &vals[1], &vals[2], &vals[3], &vals[4]);
    if (parsed != 5)
        return false;

    // Now split the parsed ints into bytes
    this->uuid[0] = (vals[0] >> 24) & 0xFF;
    this->uuid[1] = (vals[0] >> 16) & 0xFF;
    this->uuid[2] = (vals[0] >> 8) & 0xFF;
    this->uuid[3] = vals[0] & 0xFF;

    this->uuid[4] = (vals[1] >> 8) & 0xFF;
    this->uuid[5] = vals[1] & 0xFF;

    this->uuid[6] = (vals[2] >> 8) & 0xFF;
    this->uuid[7] = vals[2] & 0xFF;

    this->uuid[8] = (vals[3] >> 8) & 0xFF;
    this->uuid[9] = vals[3] & 0xFF;

    for (int i = 0; i < 6; ++i)
        this->uuid[10 + i] = (vals[4] >> (8 * (5 - i))) & 0xFF;

    return true;
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