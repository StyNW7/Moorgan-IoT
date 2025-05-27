#pragma once
#include <MPU6050.h>
#include <MAX30102.h>
#include <time.h>

typedef struct {
    float temp_in_c;
    MPUData *mpu_data;
    MAXData max_data;
    float distance_from_wifi_m;
    time_t timestamp;
    float battery_percentage;
    MainData *next;
}MainData;


class DataStream {
    private:
        MainData *head;
        MainData *tail;
        size_t size;
        MainData *current;

    public:
        MainData *getHead();
        MainData *getTail();
        size_t getSize();
        void addData(MainData *data);
        void clearData();
};
