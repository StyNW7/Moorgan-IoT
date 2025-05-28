#include <IOTHubInstance.h>
#include <WiFi.h>
#include <WiFiClientSecure.h> // For the underlying secure client if not handled by SDK transport
#include <time.h>
#include <UUIDs.h>     // Assuming this provides your UUID
#include <Provision.h>   // For Provision::getInstance()->getUUID()->getStringc()
#include <config.h>    // For ID_SCOPE, DEVICE_SYMMETRIC_KEY, NTPSERVER etc.
#include <Tools.h>     // For xprint, xprintln

// For MQTT_Protocol needed by IoTHubClient_LL_CreateFromConnectionString
// and Prov_Device_MQTT_Protocol for DPS
#include "iothubtransportmqtt.h" 
#include <AzureIoTProtocol_MQTT.h> // Ensure this provides MQTT_Protocol() correctly
                                   // For DPS, Prov_Device_MQTT_Protocol() is from <azure_prov_client/prov_transport_mqtt_client.h>

#include <AzureIoTUtility.h> // For platform_init/deinit

// Define the Azure IoT Hub Root CA Certificate (ensure this is defined globally or accessible)
// This same CA might be used for both DPS global endpoint and IoT Hub endpoint.
extern const char* AZURE_IOT_ROOT_CA_CERTIFICATE;

// Initialize static instance pointer
IOTHubInstance* IOTHubInstance::instance = nullptr;

// Static method to get the singleton instance
IOTHubInstance* IOTHubInstance::getInstance() {
    if (instance == nullptr) {
        instance = new IOTHubInstance();
    }
    return instance;
}

// Private constructor
IOTHubInstance::IOTHubInstance() :
    iotHubClientHandle(nullptr),
    iotHubConnected(false),
    lastTelemetrySend(0),
    iothubConnectionString(nullptr), // Initialize
    provDeviceHandle(nullptr),
    dpsProvisioningComplete(false),
    dpsProvisioningSucceeded(false),
    dpsAssignedHubUri(nullptr),
    dpsAssignedDeviceId(nullptr)
{
    // Load provisioning info from NVS here if it exists
    // if (loadProvisioningInfoFromNVS()) {
    //     xprintln("Loaded provisioning info from NVS.");
    // } else {
    //     xprintln("No provisioning info in NVS or NVS not used.");
    // }
}

// Destructor
IOTHubInstance::~IOTHubInstance() {
    if (iotHubClientHandle != nullptr) {
        IoTHubClient_LL_Destroy(iotHubClientHandle);
        iotHubClientHandle = nullptr;
    }
    if (provDeviceHandle != nullptr) { // Clean up DPS handle
        Prov_Device_LL_Destroy(provDeviceHandle);
        provDeviceHandle = nullptr;
    }
    if (iothubConnectionString) free(iothubConnectionString);
    if (dpsAssignedHubUri) free(dpsAssignedHubUri);
    if (dpsAssignedDeviceId) free(dpsAssignedDeviceId);

    platform_deinit();
    xprintln("IOTHub instance destroyed and resources cleaned up.");
}

void IOTHubInstance::syncTimeNTP() {
    xprintln("Synchronizing time with NTP server...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTPSERVER, NTPSERVER2); // From config.h

    time_t now = time(nullptr);
    int retries = 0;
    while (now < 8 * 3600 * 2) {
        delay(300); xprint("."); now = time(nullptr); retries++;
        if (retries > 20) { // Increased retries
            xprintln("\nFailed to synchronize time with NTP.");
            return;
        }
    }
    struct tm timeinfo; gmtime_r(&now, &timeinfo);
    xprint("\nCurrent time: "); xprintln(asctime(&timeinfo));
}

// New orchestrating method
bool IOTHubInstance::initializeAndConnect() {
    xprintln("Initializing Azure IoT connection sequence...");

    if (WiFi.status() != WL_CONNECTED) {
        xprintln("Wi-Fi not connected. Cannot proceed.");
        // Consider adding Wi-Fi setup logic here or ensuring it's done before calling this.
        return false;
    }
    
    syncTimeNTP(); // Ensure time is synced for TLS

    // Try to load existing provisioning info (conceptual)
    if (loadProvisioningInfoFromNVS()) {
        xprintln("Device already provisioned. Using stored credentials.");
        // Construct connection string from dpsAssignedHubUri, dpsAssignedDeviceId, and DEVICE_SYMMETRIC_KEY
        // This assumes DEVICE_SYMMETRIC_KEY is the one to use for IoT Hub directly
        size_t connStringLen = strlen("HostName=") + strlen(dpsAssignedHubUri) +
                               strlen(";DeviceId=") + strlen(dpsAssignedDeviceId) +
                               strlen(";SharedAccessKey=") + strlen(DEVICE_SYMMETRIC_KEY) + 1;
        iothubConnectionString = (char*)malloc(connStringLen);
        if (iothubConnectionString) {
            snprintf(iothubConnectionString, connStringLen, "HostName=%s;DeviceId=%s;SharedAccessKey=%s",
                     dpsAssignedHubUri, dpsAssignedDeviceId, DEVICE_SYMMETRIC_KEY); // From config.h
        } else {
            xprintln("Failed to allocate memory for connection string from NVS info.");
            return false;
        }
    } else {
        xprintln("Device not provisioned or NVS info not found. Attempting DPS provisioning...");
        if (!provisionDevice()) {
            xprintln("DPS Provisioning failed.");
            return false;
        }
        // After successful provisioning, dpsAssignedHubUri and dpsAssignedDeviceId are set
        // And iothubConnectionString is constructed by provisionDevice or here
         size_t connStringLen = strlen("HostName=") + strlen(dpsAssignedHubUri) +
                               strlen(";DeviceId=") + strlen(dpsAssignedDeviceId) +
                               strlen(";SharedAccessKey=") + strlen(DEVICE_SYMMETRIC_KEY) + 1;
        iothubConnectionString = (char*)malloc(connStringLen);
        if (iothubConnectionString) {
            snprintf(iothubConnectionString, connStringLen, "HostName=%s;DeviceId=%s;SharedAccessKey=%s",
                     dpsAssignedHubUri, dpsAssignedDeviceId, DEVICE_SYMMETRIC_KEY);
        } else {
            xprintln("Failed to allocate memory for connection string post-DPS.");
            return false;
        }
        saveProvisioningInfoToNVS(); // Save for next time
    }
    
    // Now connect to IoT Hub using the obtained/constructed connection string
    return connectToIoTHub();
}


bool IOTHubInstance::connectToIoTHub() {
    xprintln("Setting up Azure IoT Hub client connection...");

    if (WiFi.status() != WL_CONNECTED || iothubConnectionString == nullptr) {
        xprintln("Wi-Fi not connected or connection string missing. Cannot setup IoT Hub Client.");
        return false;
    }

    if (platform_init() != 0) { // Ensure platform is initialized (can be called multiple times)
        xprintln("Failed to re-initialize the platform for IoT Hub client.");
        return false;
    }

    // Create IoT Hub client from the (possibly DPS-derived) connection string
    iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(iothubConnectionString, MQTT_Protocol());
    if (iotHubClientHandle == NULL) {
        xprintln("Failed to create IoT Hub client handle from connection string.");
        platform_deinit(); // Or manage deinit more carefully if DPS also used it
        return false;
    }

    if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_TRUSTED_CERT, AZURE_IOT_ROOT_CA_CERTIFICATE) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set trusted CA certificate for IoT Hub client.");
    } else {
        xprintln("Trusted CA certificate option set for IoT Hub client.");
    }

    int keepAliveIntervalSeconds = MQTT_KEEP_ALIVE_MINUTES * 60; // From config.h
    if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_KEEP_ALIVE, &keepAliveIntervalSeconds) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set MQTT keep alive for IoT Hub client.");
    }

    if (IoTHubClient_LL_SetConnectionStatusCallback(iotHubClientHandle, IOTHubInstance::hubConnectionStatusCallback, this) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set IoT Hub connection status callback.");
    }
    if (IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, IOTHubInstance::hubReceiveMessageCallback, this) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set IoT Hub message callback.");
    }

    xprintln("Azure IoT Hub client setup for connection complete. DoWork will connect.");
    // Note: Actual connection happens when IoTHubClient_LL_DoWork() is called in the main loop.
    // The hubConnectionStatusCallback will update iotHubConnected.
    return true;
}


bool IOTHubInstance::provisionDevice() {
    xprintln("Starting Device Provisioning Process...");
    dpsProvisioningComplete = false;
    dpsProvisioningSucceeded = false;

    if (platform_init() != 0) { // Ensure platform is initialized
        xprintln("Failed to initialize platform for DPS.");
        return false;
    }
    
    // Use Prov_Device_MQTT_Protocol() or Prov_Device_MQTT_WS_Protocol() from <azure_prov_client/prov_transport_mqtt_client.h>
    provDeviceHandle = Prov_Device_LL_Create(PROV_GLOBAL_DEVICE_ENDPOINT, ID_SCOPE, Prov_Device_MQTT_Protocol); // ID_SCOPE from config.h
    if (provDeviceHandle == NULL) {
        xprintln("Failed to create DPS Prov_Device_LL_Create handle.");
        platform_deinit();
        return false;
    }

    if (IoTHubClient_LL_SetOption(reinterpret_cast<IOTHUB_CLIENT_LL_HANDLE>(provDeviceHandle), OPTION_TRUSTED_CERT, AZURE_IOT_ROOT_CA_CERTIFICATE) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set trusted CA for DPS.");
        // Not necessarily fatal, but log it. Some platforms might have certs pre-loaded.
    }

    // For Symmetric Key:
    // The REGISTRATION_ID here is your device's unique UUID.
    // The DEVICE_SYMMETRIC_KEY is the key associated with this UUID in DPS (either individual enrollment key or group-key-derived device key).
    // Ensure REGISTRATION_ID and DEVICE_SYMMETRIC_KEY are correctly defined in config.h
    // or retrieved, for example, via Provision::getInstance()->getUUID()->getStringc() for REGISTRATION_ID
    const char* registrationId = Provision::getInstance()->getUUID()->getStringc(); // Get UUID
    if (DEVICE_SYMMETRIC_KEY == nullptr || registrationId == nullptr) {
        xprintln("DPS Symmetric Key or Registration ID is null.");
        Prov_Device_LL_Destroy(provDeviceHandle);
        platform_deinit();
        return false;
    }
    
    // If using group enrollment, the DEVICE_SYMMETRIC_KEY here should be the *device-specific key*
    // derived from the group key and the registrationId.
    // If the SDK doesn't derive it, you might need to:
    // char* derivedDeviceKey = compute_device_key_from_group_key(registrationId, GROUP_SYMMETRIC_KEY);
    // And use derivedDeviceKey below. For this example, we assume DEVICE_SYMMETRIC_KEY is the correct one to use.
    if (Prov_Device_LL_SetSymmetricKeyInfo(provDeviceHandle, registrationId, DEVICE_SYMMETRIC_KEY) != 0) {
         xprintln("Failed to set symmetric key info for DPS.");
         Prov_Device_LL_Destroy(provDeviceHandle);
         platform_deinit();
         return false;
    }


    if (Prov_Device_LL_Register_Device(provDeviceHandle, IOTHubInstance::dpsRegisterCallback, this, IOTHubInstance::dpsStatusCallback, this) != PROV_DEVICE_RESULT_OK) {
        xprintln("Failed to start DPS registration process.");
        Prov_Device_LL_Destroy(provDeviceHandle);
        platform_deinit();
        return false;
    }

    xprintln("DPS registration process started. Waiting for completion (driven by DoWork)...");
    unsigned long dpsStartTime = millis();
    while (!dpsProvisioningComplete) {
        Prov_Device_LL_DoWork(provDeviceHandle);
        delay(100); // Keep this short to be responsive
        if (millis() - dpsStartTime > DPS_TIMEOUT_MS) { // DPS_TIMEOUT_MS from config.h e.g., 60000
            xprintln("DPS provisioning timed out.");
            break;
        }
    }

    Prov_Device_LL_Destroy(provDeviceHandle); // Always destroy handle after use
    provDeviceHandle = nullptr; 
    // platform_deinit(); // Defer platform_deinit until application truly ends, or manage carefully if IoT Hub client also uses it.

    return dpsProvisioningSucceeded;
}


// --- Static Callback Method Implementations for DPS ---
void IOTHubInstance::dpsRegisterCallback(PROV_DEVICE_RESULT register_result, const char* iothub_uri, const char* device_id, void* user_context) {
    IOTHubInstance* self = static_cast<IOTHubInstance*>(user_context);
    if (self == nullptr) return;

    if (register_result == PROV_DEVICE_RESULT_OK) {
        xprintln("DPS Registration Successful!");
        xprint("Assigned IoT Hub URI: "); xprintln(iothub_uri);
        xprint("Assigned Device ID: "); xprintln(device_id);

        if (self->dpsAssignedHubUri) free(self->dpsAssignedHubUri);
        self->dpsAssignedHubUri = strdup(iothub_uri);

        if (self->dpsAssignedDeviceId) free(self->dpsAssignedDeviceId);
        self->dpsAssignedDeviceId = strdup(device_id);
        
        self->dpsProvisioningSucceeded = true;
    } else {
        xprint("DPS Registration Failed! Result: "); xprintln(register_result);
        self->dpsProvisioningSucceeded = false;
    }
    self->dpsProvisioningComplete = true;
}

void IOTHubInstance::dpsStatusCallback(PROV_DEVICE_REG_STATUS reg_status, void* user_context) {
    // IOTHubInstance* self = static_cast<IOTHubInstance*>(user_context);
    xprint("DPS Provisioning Status: ");
    switch (reg_status) {
        case PROV_DEVICE_REG_STATUS_CONNECTED: xprintln("Client connected to DPS."); break;
        case PROV_DEVICE_REG_STATUS_REGISTERING: xprintln("Registering with DPS."); break;
        case PROV_DEVICE_REG_STATUS_ASSIGNING: xprintln("Waiting for DPS to assign IoT Hub."); break;
        case PROV_DEVICE_REG_STATUS_ASSIGNED: xprintln("Device assigned to IoT Hub by DPS."); break; // This is good
        case PROV_DEVICE_REG_STATUS_ERROR: xprintln("Error during DPS registration."); break;
        default: xprintln("Unknown DPS status."); break;
    }
}


// --- Conceptual NVS Functions (you'll need to implement these using ESP32 NVS API) ---
bool IOTHubInstance::loadProvisioningInfoFromNVS() {
    xprintln("Attempting to load provisioning info from NVS...");
    // TODO: Implement NVS read for dpsAssignedHubUri and dpsAssignedDeviceId
    // Example:
    // SharedPreferences preferences; // Using a hypothetical NVS wrapper
    // if (preferences.begin("iot_prov", true)) { // read-only
    //     String uri = preferences.getString("hub_uri", "");
    //     String id = preferences.getString("dev_id", "");
    //     preferences.end();
    //     if (uri.length() > 0 && id.length() > 0) {
    //         if (dpsAssignedHubUri) free(dpsAssignedHubUri);
    //         dpsAssignedHubUri = strdup(uri.c_str());
    //         if (dpsAssignedDeviceId) free(dpsAssignedDeviceId);
    //         dpsAssignedDeviceId = strdup(id.c_str());
    //         return (dpsAssignedHubUri != nullptr && dpsAssignedDeviceId != nullptr);
    //     }
    // }
    return false; // Placeholder
}

void IOTHubInstance::saveProvisioningInfoToNVS() {
    xprintln("Saving provisioning info to NVS...");
    if (dpsProvisioningSucceeded && dpsAssignedHubUri && dpsAssignedDeviceId) {
        // TODO: Implement NVS write for dpsAssignedHubUri and dpsAssignedDeviceId
        // Example:
        // SharedPreferences preferences;
        // if (preferences.begin("iot_prov", false)) { // read-write
        //     preferences.putString("hub_uri", dpsAssignedHubUri);
        //     preferences.putString("dev_id", dpsAssignedDeviceId);
        //     preferences.end();
        //     xprintln("Provisioning info saved.");
        // } else {
        //     xprintln("Failed to open NVS for writing.");
        // }
    } else {
        xprintln("No valid provisioning info to save.");
    }
}


// Implement existing methods (sendJsonToAzure, isAzureIoTConnected, callbacks)
// Ensure they use the member variables like iotHubClientHandle, iotHubConnected.
// The static callbacks need to cast userContextCallback to IOTHubInstance* as shown.

bool IOTHubInstance::sendJsonToAzure(const char* jsonPayload) {
    if (iotHubClientHandle == NULL || !iotHubConnected) {
        xprintln("Azure IoT Hub client not initialized or not connected. Cannot send data.");
        return false;
    }
    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(jsonPayload);
    if (messageHandle == NULL) { xprintln("Failed to create IoT Hub message."); return false; }
    IoTHubMessage_SetContentTypeSystemProperty(messageHandle, "application%2Fjson");
    IoTHubMessage_SetContentEncodingSystemProperty(messageHandle, "utf-8");
    xprint("Sending JSON to Azure: "); xprintln(jsonPayload);
    if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, messageHandle, IOTHubInstance::hubSendConfirmationCallback, (void*)messageHandle) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to send message to Azure IoT Hub.");
        IoTHubMessage_Destroy(messageHandle);
        return false;
    }
    return true;
}

bool IOTHubInstance::isAzureIoTConnected() {
    return iotHubConnected;
}

IOTHUB_CLIENT_LL_HANDLE IOTHubInstance::getIotHubClientHandle(){
    return iotHubClientHandle;
}

// Static IoT Hub Callbacks - ensure they use userContext to get 'self'
void IOTHubInstance::hubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback) {
    IOTHubInstance* self = static_cast<IOTHubInstance*>(userContextCallback);
    if (self) { self->iotHubConnected = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED); }
    xprint("\nHub Connection Status: "); // Differentiate from DPS status
    switch (reason) { /* ... same as before ... */ 
        case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:      xprintln("Expired SAS token"); break;
        case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:        xprintln("Device disabled"); break;
        case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:         xprintln("Bad credential"); break;
        case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:          xprintln("Retry expired"); break;
        case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:             xprintln("No network"); break;
        case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:    xprintln("Communication error"); break;
        case IOTHUB_CLIENT_CONNECTION_OK:                     xprintln("OK"); break;
        case IOTHUB_CLIENT_CONNECTION_NO_PING_RESPONSE:       xprintln("No Ping Response"); break;
        default:                                              xprintln("Unknown reason"); break;
    }
    if (self && self->iotHubConnected) { xprintln("Successfully connected to Azure IoT Hub."); }
    else { xprintln("Disconnected from Azure IoT Hub."); }
}

void IOTHubInstance::hubSendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback) {
    IOTHUB_MESSAGE_HANDLE messageHandle = (IOTHUB_MESSAGE_HANDLE)userContextCallback;
    if (result == IOTHUB_CLIENT_CONFIRMATION_OK) { xprintln("Hub Telemetry send confirmation: OK"); }
    else { xprint("Hub Telemetry send confirmation: FAILED, result: "); xprintln(result); }
    if (messageHandle != NULL) { IoTHubMessage_Destroy(messageHandle); }
}

IOTHUBMESSAGE_DISPOSITION_RESULT IOTHubInstance::hubReceiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback) {
    //IOTHubInstance* self = static_cast<IOTHubInstance*>(userContextCallback);
    const unsigned char* buffer; size_t size;
    if (IoTHubMessage_GetByteArray(message, &buffer, &size) != IOTHUB_MESSAGE_OK) {
        xprintln("Failed to get C2D message data from Hub."); return IOTHUBMESSAGE_REJECTED;
    }
    char* temp = (char*)malloc(size + 1);
    if (temp == NULL) { xprintln("Failed to allocate memory for Hub C2D message."); return IOTHUBMESSAGE_ABANDONED; }
    memcpy(temp, buffer, size); temp[size] = '\0';
    xprint("Received C2D Message from Hub: "); xprintln(temp);
    // Process message
    free(temp);
    return IOTHUBMESSAGE_ACCEPTED;
}

// The AZURE_IOT_ROOT_CA_CERTIFICATE definition remains at the bottom of this .cpp file as before.
// const char* AZURE_IOT_ROOT_CA_CERTIFICATE = \
// ... your cert string ...