#include "IOTHubInstance.h" // Include the class header
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <UUIDs.h>
#include <config.h> 
#include <secreets.h> // For IOTHUB_CONNECTION_STRING, MQTT_KEEP_ALIVE_MINUTES, etc.
#include <Tools.h>

// Azure IoT SDK specific headers
#include <AzureIoTProtocol_MQTT.h>
#include <AzureIoTUtility.h>
#include "iothubtransportmqtt.h" // For MQTT_Protocol

// Define the Azure IoT Hub Root CA Certificate here
// IMPORTANT: YOU MUST PROVIDE THIS CERTIFICATE STRING.
// This could also be a static const member of the class, initialized here.
extern const char* AZURE_IOT_ROOT_CA_CERTIFICATE; // PASTE YOUR CERTIFICATE HERE
bool IOTHubInstance::platformInitialized = false;

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
    if (!platformInitialized) {
        if (platform_init() == 0) {
            platformInitialized = true;
            xprintln("Platform initialized successfully by IOTHubInstance constructor.");
        } else {
            xprintln("FATAL: Failed to initialize platform in IOTHubInstance constructor.");
            // You might want to set an error flag here to prevent further operations
        }
    }
}

// Destructor
IOTHubInstance::~IOTHubInstance() {
    if (iotHubClientHandle != nullptr) {
        IoTHubClient_LL_Destroy(iotHubClientHandle);
        iotHubClientHandle = nullptr;
    }
    if (platformInitialized) {
        platform_deinit(); // Deinitialize the platform
        platformInitialized = false;
        xprintln("IOTHubInstance destroyed and platform de-initialized.");
    } else {
        xprintln("IOTHubInstance destroyed (platform was not initialized by it or already de-initialized).");
    }
}

// --- Member Method Implementations ---

void IOTHubInstance::prepareForSleep() {
    if (this->iotHubClientHandle != nullptr) {
        xprintln("IOTHubInstance: Preparing for sleep.");

        // Step 1: Disable further operations and unregister general callbacks
        // to prevent them from interfering during the destruction process.
        // Set a flag or change state if your app logic uses isAzureIoTConnected()
        // to gate operations like sendJsonToAzure().
        this->iotHubConnected = false; // Mark as disconnected to stop new operations

        xprintln("IOTHubInstance: Setting ConnectionStatus and Message callbacks to NULL before DoWork/Destroy.");
        // Attempt to unregister global callbacks. If these fail, log it but proceed,
        // as the main goal is to destroy the client.
        if (IoTHubClient_LL_SetConnectionStatusCallback(this->iotHubClientHandle, NULL, NULL) != IOTHUB_CLIENT_OK) {
            xprintln("Warning: Failed to set ConnectionStatusCallback to NULL in prepareForSleep.");
        }
        if (IoTHubClient_LL_SetMessageCallback(this->iotHubClientHandle, NULL, NULL) != IOTHUB_CLIENT_OK) {
            xprintln("Warning: Failed to set MessageCallback to NULL in prepareForSleep.");
        }

        // Step 2: Give the SDK a brief chance to process any immediate pending items.
        // Call DoWork a few times. This is a best-effort to flush outgoing messages
        // or process acknowledgments that might be very close to completion.
        // Avoid extensive looping here if the client might already be unstable.
        xprintln("IOTHubInstance: Performing minimal DoWork cycles before destroy.");
        for (int i = 0; i < 2; ++i) { // Very few cycles
            if (this->iotHubClientHandle != nullptr) { // Important to re-check
                IoTHubClient_LL_DoWork(this->iotHubClientHandle);
                delay(10); // Shortest practical delay
            } else {
                break; // Handle became null unexpectedly
            }
        }

        // Step 3: Destroy the client and nullify the handle.
        xprintln("IOTHubInstance: Destroying client handle for sleep.");
        IoTHubClient_LL_Destroy(this->iotHubClientHandle);
        this->iotHubClientHandle = nullptr; // CRITICAL: Nullify the handle immediately
                                        // iotHubConnected is already set to false.
    } else {
        xprintln("IOTHubInstance: Preparing for sleep. Client handle was already null.");
        this->iotHubConnected = false; // Ensure consistency
    }
}

void IOTHubInstance::syncTimeNTP() {
    xprintln("Synchronizing time with NTP server...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTPSERVER, NTPSERVER2);

    time_t now = time(nullptr);
    int retries = 0;
    // Check if time is reasonably set (e.g., after Jan 1, 2023)
    // (1672531200 is Jan 1, 2023 00:00:00 UTC)
    while (now < 1672531200UL && retries < 30) { // Increased retries
        delay(1000); // Increased delay
        xprint(".");
        now = time(nullptr);
        retries++;
    }

    if (now < 1672531200UL) {
        xprintln("\nFailed to synchronize time with NTP after multiple retries.");
        // Consider setting a flag indicating time sync failure
    } else {
        struct tm timeinfo;
        gmtime_r(&now, &timeinfo);
        xprint("\nCurrent time: ");
        xprintln(asctime(&timeinfo));
    }
}

const char* extractHostName(const char* connectionString) {
    if (connectionString == nullptr) return nullptr;
    const char* hostNameKey = "HostName=";
    const char* hostNameStart = strstr(connectionString, hostNameKey);
    if (hostNameStart == nullptr) return nullptr;

    hostNameStart += strlen(hostNameKey);
    const char* hostNameEnd = strchr(hostNameStart, ';');
    if (hostNameEnd == nullptr) return nullptr; // Should not happen in a valid conn string

    // This is a bit tricky as we need to return a string that persists.
    // For simplicity in this context, we'll assume a static buffer.
    // A more robust solution might involve dynamic allocation or passing a buffer.
    static char extractedHost[128]; // Adjust size as needed
    size_t len = hostNameEnd - hostNameStart;
    if (len < sizeof(extractedHost)) {
        strncpy(extractedHost, hostNameStart, len);
        extractedHost[len] = '\0';
        return extractedHost;
    }
    return nullptr; // Hostname too long for buffer
}

const char* IOTHubInstance::getAzureHostName() {
    // Extract hostname from IOTHUB_CONNECTION_STRING
    // This is a simplified way; a more robust parser might be needed if conn string format varies.
    return extractHostName(IOTHUB_CONNECTION_STRING);
}

bool IOTHubInstance::setupAzureIoTClient() {
    xprintln("Setting up Azure IoT Hub client (Singleton)...");

    // Step 1: Clean up any pre-existing client instance thoroughly.
    if (this->iotHubClientHandle != nullptr) {
        xprintln("Destroying existing IoT Hub client handle before re-setup (within setupAzureIoTClient).");
        this->iotHubConnected = false; // Mark as disconnected

        xprintln("IOTHubInstance: Setting ConnectionStatus and Message callbacks to NULL for existing client.");
        if (IoTHubClient_LL_SetConnectionStatusCallback(this->iotHubClientHandle, NULL, NULL) != IOTHUB_CLIENT_OK) {
            xprintln("Warning: Failed to set ConnectionStatusCallback to NULL for existing client.");
        }
        if (IoTHubClient_LL_SetMessageCallback(this->iotHubClientHandle, NULL, NULL) != IOTHUB_CLIENT_OK) {
            xprintln("Warning: Failed to set MessageCallback to NULL for existing client.");
        }
        
        // Minimal DoWork before destroying this old handle
        for (int i = 0; i < 2; ++i) {
            if (this->iotHubClientHandle != nullptr) {
                IoTHubClient_LL_DoWork(this->iotHubClientHandle);
                delay(10);
            } else break;
        }

        IoTHubClient_LL_Destroy(this->iotHubClientHandle);
        this->iotHubClientHandle = nullptr; // CRITICAL
    }

    // Step 2: Pre-requisite checks (WiFi, Time, Platform)
    if (WiFi.status() != WL_CONNECTED) {
        xprintln("Wi-Fi not connected. Cannot setup Azure IoT Client.");
        return false;
    }

    time_t now = time(nullptr);
    if (now < 1672531200UL) { // Example: Jan 1, 2023 UTC
         xprintln("Time not synchronized. TLS connection might fail. Ensure syncTimeNTP() was called and succeeded.");
         // return false; // Strongly consider making this a failure condition
    }

    if (!platformInitialized) { // Assuming platformInitialized is managed as per previous discussions
        xprintln("Platform not initialized! Cannot setup Azure IoT Client.");
        return false;
    }

    // Step 3: Create the new client handle
    this->iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(IOTHUB_CONNECTION_STRING, MQTT_Protocol);
    if (this->iotHubClientHandle == NULL) {
        xprintln("Failed to create IoT Hub client handle.");
        this->iotHubConnected = false; // Ensure status
        return false;
    }
    this->iotHubConnected = false; // Initial state for new handle, will be updated by callback

    // Step 4: Set options for the new client
    if (IoTHubClient_LL_SetOption(this->iotHubClientHandle, OPTION_TRUSTED_CERT, AZURE_IOT_ROOT_CA_CERTIFICATE) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set trusted CA certificate option.");
        IoTHubClient_LL_Destroy(this->iotHubClientHandle); // Clean up partially created client
        this->iotHubClientHandle = nullptr;
        return false;
    }
    xprintln("Trusted CA certificate option set.");

    int keepAliveIntervalSeconds = MQTT_KEEP_ALIVE_MINUTES * 60;
    if (IoTHubClient_LL_SetOption(this->iotHubClientHandle, OPTION_KEEP_ALIVE, &keepAliveIntervalSeconds) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set MQTT keep alive option."); // Log but might not be fatal
    } else {
        xprint("MQTT Keep Alive option set to: "); xprint(MQTT_KEEP_ALIVE_MINUTES); xprintln(" minutes.");
    }

    // Step 5: Register actual callbacks for the new client
    xprintln("IOTHubInstance: Registering actual callbacks for the new client.");
    if (IoTHubClient_LL_SetConnectionStatusCallback(this->iotHubClientHandle, IOTHubInstance::connectionStatusCallback, this) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set connection status callback for new client.");
        IoTHubClient_LL_Destroy(this->iotHubClientHandle);
        this->iotHubClientHandle = nullptr;
        return false;
    }

    if (IoTHubClient_LL_SetMessageCallback(this->iotHubClientHandle, IOTHubInstance::receiveMessageCallback, this) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set message callback for new client.");
        // Decide if this is fatal, if so, destroy and return false.
    }

    xprintln("Azure IoT Hub client setup initiated. The SDK will attempt to connect via DoWork().");
    return true;
}

bool IOTHubInstance::sendJsonToAzure(const char* jsonPayload) {
    
    if (iotHubClientHandle == NULL) {
        xprintln("Azure IoT Hub client not initialized because iotHubClientHandle is null. Cannot send data.");
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
    if (self == nullptr) {
        xprintln("ConnectionStatusCallback: userContextCallback is NULL!");
        return;
    }

    self->iotHubConnected = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);

    switch (reason) {
        case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:
            xprintln("Connection Status: SAS Token Expired");
            break;
        case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:
            xprintln("Connection Status: Device Disabled");
            break;
        case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:
            xprintln("Connection Status: Bad Credential");
            break;
        case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:
            xprintln("Connection Status: Retry Expired");
            if (self->iotHubClientHandle != nullptr) {
                // IoTHubClient_LL_Destroy(self->iotHubClientHandle); // Destroy might be too aggressive here,
                                                                  // as DoWork might attempt retries.
                                                                  // Let DoWork manage the handle lifecycle during retries.
                                                                  // Only nullify if we are sure it's unrecoverable by DoWork.
            }
            // self->iotHubClientHandle = nullptr; // Consider if this is the right place
            break;
        case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:
            xprintln("Connection Status: No network");
            // If no network, the handle is likely invalid for new operations.
            // However, IoTHubClient_LL_DoWork is responsible for reconnection attempts.
            // Aggressively destroying/nullifying here might interfere with SDK's internal retry logic.
            // The SDK should eventually transition to RETRY_EXPIRED if it can't recover.
            break;
        case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:
            xprintln("Connection Status: Communication Error");
            break;
        case IOTHUB_CLIENT_CONNECTION_OK:
            xprintln("Connection Status: OK");
            break;
        case IOTHUB_CLIENT_CONNECTION_NO_PING_RESPONSE:
            xprintln("Connection Status: No Ping Response");
            break;
        default:
            xprint("Connection Status: Unknown reason code: ");
            xprintln(reason);
            break;
    }

    if (!self->iotHubConnected) {
        xprintln("Disconnected from Azure IoT Hub.");
        // If truly disconnected and unrecoverable by DoWork, then cleanup might be needed.
        // For now, let setupAzureIoTClient handle re-creation if isAzureIoTConnected is false.
    } else {
        xprintln("Successfully connected to Azure IoT Hub.");
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


const char* AZURE_IOT_ROOT_CA_CERTIFICATE = \
// DigiCert Global Root G2
// Source: User-uploaded file DigiCertGlobalRootG2.txt
"-----BEGIN CERTIFICATE-----\n" \
"MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n" \
"2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n" \
"1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n" \
"q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n" \
"tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n" \
"vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n" \
"BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n" \
"5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n" \
"1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n" \
"NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n" \
"Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n" \
"8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n" \
"pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n" \
"MrY=\n" \
"-----END CERTIFICATE-----\n" \
// Microsoft RSA Root Certificate Authority 2017
// Source: User-uploaded file Microsoft RSA Root Certificate Authority 2017.txt
"-----BEGIN CERTIFICATE-----\n" \
"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\n" \
"RA+GsCyRxj3qrg+E\n" \
"-----END CERTIFICATE-----\n";