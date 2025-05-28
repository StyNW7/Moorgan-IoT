#pragma once

#include <Arduino.h>
#include <AzureIoTHub.h>             // For IOTHUB_CLIENT_LL_HANDLE
#include <azure_prov_client/prov_device_ll_client.h> // For DPS client
#include <azure_prov_client/prov_security_factory.h> // For DPS security
#include <prov_transport_mqtt_client.h> // For DPS transport (or _ws_client)

// Forward declaration
class Provision; // Assuming Provision class provides UUID

class IOTHubInstance {
public:
    static IOTHubInstance* getInstance();
    IOTHubInstance(const IOTHubInstance&) = delete;
    IOTHubInstance& operator=(const IOTHubInstance&) = delete;
    ~IOTHubInstance();

    void syncTimeNTP();
    bool initializeAndConnect(); // New orchestrating method
    bool sendJsonToAzure(const char* jsonPayload);
    bool isAzureIoTConnected();
    IOTHUB_CLIENT_LL_HANDLE getIotHubClientHandle(); // To call DoWork externally

private:
    IOTHubInstance(); // Private constructor for singleton

    static IOTHubInstance* instance;

    // IoT Hub Client specific
    IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle;
    bool iotHubConnected;
    unsigned long lastTelemetrySend;
    char* iothubConnectionString; // Will be constructed after DPS or loaded from NVS

    // DPS Client specific
    PROV_DEVICE_LL_HANDLE provDeviceHandle;
    bool dpsProvisioningComplete;
    bool dpsProvisioningSucceeded;
    char* dpsAssignedHubUri;
    char* dpsAssignedDeviceId;

    // Configuration (these would ideally come from your config.h or NVS)
    // These are illustrative. You'll need to get these from your config.h or other sources.
    // const char* ID_SCOPE = YOUR_ID_SCOPE_FROM_DPS;
    // const char* REGISTRATION_ID = YOUR_DEVICE_UUID; // This will be your device's UUID
    // const char* DEVICE_SYMMETRIC_KEY = YOUR_DEVICE_SYMMETRIC_KEY; // Device-specific key for DPS attestation

    // Methods
    bool loadProvisioningInfoFromNVS(); // Conceptual
    void saveProvisioningInfoToNVS();   // Conceptual
    bool provisionDevice();
    bool connectToIoTHub();             // Connects using either DPS-obtained or NVS-loaded info

    // Static Callbacks for IoT Hub Client
    static void hubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback);
    static void hubSendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback);
    static IOTHUBMESSAGE_DISPOSITION_RESULT hubReceiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback);

    // Static Callbacks for DPS Client
    static void dpsRegisterCallback(PROV_DEVICE_RESULT register_result, const char* iothub_uri, const char* device_id, void* user_context);
    static void dpsStatusCallback(PROV_DEVICE_REG_STATUS reg_status, void* user_context);
};