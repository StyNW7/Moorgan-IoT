#pragma once
#include <UUID.h>

// This is a Provisioning extension library for ESP32
// It provides a singleton class to manage the provisioning process

class Provision {
    private:
        UUID* uuid;
        bool redo_provision;
        bool connected;
        bool is_provisioned;
        uint8_t retry_connection_counter;
        char* pop;
        
        // Private constructor to prevent external instantiation
        Provision();
        
        // Static instance of the singleton
        static Provision* instance;
        
        // Deleted copy constructor and assignment operator to prevent copying
        Provision(const Provision&) = delete;
        Provision& operator=(const Provision&) = delete;
        
    public:
        // Destructor remains public to allow proper cleanup when needed
        ~Provision();
        
        // Static method to get the singleton instance
        static Provision* getInstance();
        
        bool getIsProvisioned();
        uint8_t getRetryConnectionCounter();
        void setIsProvisioned(bool tof);
        void addToRetryConnectionCounter();
        void resetRetryConnectionCounter();
        void setupProvision();
        void setPop(const char * pop);
        char* getPop() { return this->pop; }
        bool getRedoProvision() { return this->redo_provision; }
        void setRedoProvision(bool redo_provision) { this->redo_provision = redo_provision; }
        bool getConnected() { return this->connected; }
        void setConnected(bool connected) { this->connected = connected; }
        UUID* getUUID() { return this->uuid; }
};