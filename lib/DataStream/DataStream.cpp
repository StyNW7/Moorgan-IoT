#include "DataStream.h"
#include <MAX30102.h>
#include <MPU6050.h>

MainData *DataStream::getHead(){
    return head;
}

MainData *DataStream::getTail(){
    return tail;
}

size_t DataStream::getSize(){
    return size;
}

void DataStream::addData(MainData *data){
    if (data == nullptr) {
        return; // Do not add null data
    }

    time(&(data->timestamp)); // Set the current timestamp

    if (head == nullptr) {
        head = data; // If the list is empty, set head to the new data
        tail = data; // Set tail to the new data as well
    } else {
        tail->next = data; // Link the current tail to the new data
        tail = data; // Update tail to the new data
    }
    size++; // Increment the size of the list
}

void DataStream::clearData(){
    MainData *current = head;
    while (current != nullptr) {
        MainData *next = current->next;
        // clean up the MPUData and MAXData they are dynamically allocated
        free(current->mpu_data->mean_ax_arr);
        free(current->mpu_data->mean_ay_arr);
        free(current->mpu_data->mean_az_arr);
        free(current->mpu_data->mean_mv_arr);
        free(current->mpu_data->std_mv_arr);
        delete current->mpu_data;
        free(current->max_data); // Clean up MAXData

        delete current; // Free the memory of the current node
        current = next; // Move to the next node
    }
    head = nullptr; // Reset head to null
    tail = nullptr; // Reset tail to null
    current = nullptr; // Reset current to null
    size = 0; // Reset size to 0
}