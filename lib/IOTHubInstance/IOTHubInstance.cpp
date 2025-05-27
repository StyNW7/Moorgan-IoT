#include "IOTHubInstance.h" // Include the class header
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <UUIDs.h>
#include <Provision.h>
#include <config.h> // For IOTHUB_CONNECTION_STRING, MQTT_KEEP_ALIVE_MINUTES, etc.
#include <Tools.h>

// Azure IoT SDK specific headers
#include <AzureIoTProtocol_MQTT.h>
#include <AzureIoTUtility.h>
#include "iothubtransportmqtt.h" // For MQTT_Protocol

// Define the Azure IoT Hub Root CA Certificate here
// IMPORTANT: YOU MUST PROVIDE THIS CERTIFICATE STRING.
// This could also be a static const member of the class, initialized here.
extern const char* AZURE_IOT_ROOT_CA_CERTIFICATE; // PASTE YOUR CERTIFICATE HERE

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
    lastTelemetrySend(0)
{
    // Constructor logic, if any, beyond member initialization
}

// Destructor
IOTHubInstance::~IOTHubInstance() {
    if (iotHubClientHandle != nullptr) {
        IoTHubClient_LL_Destroy(iotHubClientHandle);
        iotHubClientHandle = nullptr;
    }
    platform_deinit(); // Deinitialize the platform
    xprintln("IOTHub instance destroyed and resources cleaned up.");
}

// --- Member Method Implementations ---

void IOTHubInstance::syncTimeNTP() {
    xprintln("Synchronizing time with NTP server...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTPSERVER, NTPSERVER2);

    time_t now = time(nullptr);
    int retries = 0;
    while (now < 8 * 3600 * 2) { // Check if time is reasonably set
        delay(300);
        xprint(".");
        now = time(nullptr);
        retries++;
        if (retries > 10) { // Increased retries
            xprintln("\nFailed to synchronize time with NTP after multiple retries.");
            return;
        }
    }
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    xprint("\nCurrent time: ");
    xprintln(asctime(&timeinfo));
}

bool IOTHubInstance::setupAzureIoTClient() {
    xprintln("Setting up Azure IoT Hub client (Singleton)...");

    if (WiFi.status() != WL_CONNECTED) {
        xprintln("Wi-Fi not connected. Cannot setup Azure IoT Client.");
        return false;
    }

    time_t now = time(nullptr);
    if (now < 8 * 3600 * 2) {
         xprintln("Time not synchronized. TLS connection might fail. Ensure syncTimeNTP() was called and succeeded.");
    }

    if (platform_init() != 0) {
        xprintln("Failed to initialize the platform.");
        return false;
    }

    iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(IOTHUB_CONNECTION_STRING, MQTT_Protocol);
    if (iotHubClientHandle == NULL) {
        xprintln("Failed to create IoT Hub client handle.");
        platform_deinit();
        return false;
    }

    if (strcmp(AZURE_IOT_ROOT_CA_CERTIFICATE, "PASTE_YOUR_AZURE_IOT_ROOT_CA_CERTIFICATE_PEM_STRING_HERE") == 0 || strlen(AZURE_IOT_ROOT_CA_CERTIFICATE) < 100) {
        xprintln("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        xprintln("ERROR: AZURE_IOT_ROOT_CA_CERTIFICATE is not set or is a placeholder!");
        xprintln("You MUST provide the Azure IoT Hub Root CA certificate string.");
        xprintln("TLS connection will likely fail if this is not correctly set.");
        xprintln("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        // Not setting the CA, which will likely cause connection failure.
    } else {
        if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_TRUSTED_CERT, AZURE_IOT_ROOT_CA_CERTIFICATE) != IOTHUB_CLIENT_OK) {
            xprintln("Failed to set trusted CA certificate option. This may lead to connection issues.");
            // Not returning false here, to allow attempt to connect without it if user insists or for testing.
        } else {
            xprintln("Trusted CA certificate option set.");
        }
    }

    int keepAliveIntervalSeconds = MQTT_KEEP_ALIVE_MINUTES * 60;
    if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_KEEP_ALIVE, &keepAliveIntervalSeconds) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set MQTT keep alive option.");
    } else {
        xprint("MQTT Keep Alive option set to: ");
        xprint(MQTT_KEEP_ALIVE_MINUTES);
        xprintln(" minutes.");
    }

    if (IoTHubClient_LL_SetConnectionStatusCallback(iotHubClientHandle, IOTHubInstance::connectionStatusCallback, this) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set connection status callback.");
    }

    // Optional: Set message callback for C2D messages
    if (IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, IOTHubInstance::receiveMessageCallback, this) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set message callback.");
    }

    xprintln("Azure IoT Hub client setup complete. The SDK will attempt to connect in the loop via DoWork().");
    return true;
}

bool IOTHubInstance::sendJsonToAzure(const char* jsonPayload) {
    if (iotHubClientHandle == NULL || !iotHubConnected) {
        xprintln("Azure IoT Hub client not initialized or not connected. Cannot send data.");
        return false;
    }

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(jsonPayload);
    if (messageHandle == NULL) {
        xprintln("Failed to create IoT Hub message.");
        return false;
    }

    IoTHubMessage_SetContentTypeSystemProperty(messageHandle, "application%2Fjson");
    IoTHubMessage_SetContentEncodingSystemProperty(messageHandle, "utf-8");

    xprint("Sending JSON to Azure: ");
    xprintln(jsonPayload);

    // Pass messageHandle as context so it can be destroyed in the callback
    if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, messageHandle, IOTHubInstance::sendConfirmationCallback, (void*)messageHandle) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to send message to Azure IoT Hub.");
        IoTHubMessage_Destroy(messageHandle);
        return false;
    }
    return true;
}

bool IOTHubInstance::isAzureIoTConnected() {
    return iotHubConnected;
}

// --- Static Callback Method Implementations ---

void IOTHubInstance::connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback) {
    IOTHubInstance* self = static_cast<IOTHubInstance*>(userContextCallback);
    if (self) {
        self->iotHubConnected = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    }

    xprint("\nConnection Status: ");
    switch (reason) {
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
    if (self && self->iotHubConnected) {
        xprintln("Successfully connected to Azure IoT Hub.");
    } else {
        xprintln("Disconnected from Azure IoT Hub.");
    }
}

void IOTHubInstance::sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback) {
    IOTHUB_MESSAGE_HANDLE messageHandle = (IOTHUB_MESSAGE_HANDLE)userContextCallback; // Context is the message handle
    if (result == IOTHUB_CLIENT_CONFIRMATION_OK) {
        xprintln("Telemetry send confirmation: OK");
    } else {
        xprint("Telemetry send confirmation: FAILED, result: ");
        xprintln(result);
    }
    if (messageHandle != NULL) {
      IoTHubMessage_Destroy(messageHandle);
    }
}

IOTHUBMESSAGE_DISPOSITION_RESULT IOTHubInstance::receiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback) {
    IOTHubInstance* self = static_cast<IOTHubInstance*>(userContextCallback);
    // self can be used if needed to interact with the instance, e.g. self->someMemberVariable

    const unsigned char* buffer;
    size_t size;
    if (IoTHubMessage_GetByteArray(message, &buffer, &size) != IOTHUB_MESSAGE_OK) {
        xprintln("Failed to get C2D message data.");
        return IOTHUBMESSAGE_REJECTED;
    } else {
        char* temp = (char*)malloc(size + 1);
        if (temp == NULL) {
            xprintln("Failed to allocate memory for C2D message.");
            return IOTHUBMESSAGE_ABANDONED;
        }
        memcpy(temp, buffer, size);
        temp[size] = '\0';
        xprint("Received C2D Message: ");
        xprintln(temp);

        ///////////////////////////////////
        //                               //
        // Process the message as needed //
        //                               //
        ///////////////////////////////////

        free(temp);
        return IOTHUBMESSAGE_ACCEPTED;
    }
}

IOTHUB_CLIENT_LL_HANDLE IOTHubInstance::getIotHubClientHandle(){
    return iotHubClientHandle;
}


const char* AZURE_IOT_ROOT_CA_CERTIFICATE = "";