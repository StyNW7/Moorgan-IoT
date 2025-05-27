#pragma once

#include <Arduino.h>
#include <AzureIoTHub.h> // For IOTHUB_CLIENT_LL_HANDLE and other SDK types

class IOTHubInstance {
public:
    // Static method to get the singleton instance
    static IOTHubInstance* getInstance();

    // Deleted copy constructor and assignment operator to prevent copies
    IOTHubInstance(const IOTHubInstance&) = delete;
    IOTHubInstance& operator=(const IOTHubInstance&) = delete;

    // Public interface methods (formerly global functions)
    void syncTimeNTP();
    IOTHUB_CLIENT_LL_HANDLE getIotHubClientHandle();
    bool setupAzureIoTClient();
    bool sendJsonToAzure(const char* jsonPayload);
    bool isAzureIoTConnected();

    // Destructor for cleanup
    ~IOTHubInstance();

private:
    // Private constructor to prevent external instantiation
    IOTHubInstance();

    // Singleton instance
    static IOTHubInstance* instance;

    // Member variables (formerly global static variables)
    IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle;
    bool iotHubConnected;
    unsigned long lastTelemetrySend;

    // Azure IoT Hub Root CA Certificate (can be defined in .cpp)
    // If you prefer to define it here, it would be:
    // static const char* AZURE_IOT_ROOT_CA;

    // SDK Callback functions (must be static)
    static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback);
    static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback);
    static IOTHUBMESSAGE_DISPOSITION_RESULT receiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback);
};