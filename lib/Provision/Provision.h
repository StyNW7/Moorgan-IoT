#pragma once
#include <UUIDs.h>

// This is a Provisioning extension library for ESP32
// It provides a singleton class to manage the provisioning process

class Provision {
    private:
        UUIDs* uuid;
        bool redo_provision;
        bool connected;
        bool is_provisioned;
        uint8_t retry_connection_counter;
        char* pop;
        bool isAttemptingConnectionPhase; // True if in a connection/re-provisioning cycle
        unsigned long connectionAttemptStartTimeMs; // Timestamp when the current attempt phase started
        bool isBleProvCurrentlyActive; // To track if BLE provisioning is the active part
        
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

        bool getIsAttemptingConnectionPhase() const;

        void setIsAttemptingConnectionPhase(bool status);

        unsigned long getConnectionAttemptStartTimeMs() const ;

        void setConnectionAttemptStartTimeMs(unsigned long time);

        bool getIsBleProvCurrentlyActive() const ;

        void setIsBleProvCurrentlyActive(bool status) ;
        
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
        void setConnected(bool connected);
        UUIDs* getUUID() { return this->uuid; }
};