// Include necessary libraries for Azure IoT Hub SDK
// The exact includes might vary slightly based on the specific Arduino wrapper
// library version for the Azure C SDK you have installed.
#include <WiFi.h>
#include <WiFiClientSecure.h> // Still used by the SDK for the underlying secure connection
#include <time.h>             // For time synchronization
#include <Provision.h>        // to include UUID for device identification
#include <config.h>           // Configuration header for constants and settings
#include <Tools.h>            // Utility functions, e.g., xprintln, xflush

// Azure IoT SDK specific headers
// These are typical, ensure they match your installed library
#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>
#include <AzureIoTUtility.h> // For platform_init, etc.
#include "iothubtransportmqtt.h"

// -----------------------------------------------------------------------------
// CONFIGURATION - REPLACE WITH YOUR DETAILS
// -----------------------------------------------------------------------------

// Azure IoT Hub Root CA Certificate
// IMPORTANT: YOU NEED TO PROVIDE THIS CERTIFICATE STRING.
// Define AZURE_IOT_ROOT_CA with the correct PEM formatted certificate.
// Ensure this variable is defined before it's used to set the trusted certificate option.
extern const char* AZURE_IOT_ROOT_CA;

// -----------------------------------------------------------------------------
// GLOBAL VARIABLES
// -----------------------------------------------------------------------------
static IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle; // Handle for the IoT Hub client
static bool iotHubConnected = false;              // Flag to track connection status

// -----------------------------------------------------------------------------
// FUNCTION DECLARATIONS
// -----------------------------------------------------------------------------
void syncTimeNTP();
bool setupAzureIoTClient();
bool sendJsonToAzure(const char* jsonPayload);
bool isAzureIoTConnected();

// Callbacks for the Azure IoT SDK
static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback);
static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback);
static IOTHUBMESSAGE_DISPOSITION_RESULT receiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback); // For C2D messages

// -----------------------------------------------------------------------------
// TIME SYNCHRONIZATION FUNCTION (NTP)
// -----------------------------------------------------------------------------
void syncTimeNTP() {
    xprintln("Synchronizing time with NTP server...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTPSERVER, NTPSERVER2);

    time_t now = time(nullptr);
    int retries = 0;
    while (now < 8 * 3600 * 2) { // Check if time is reasonably set
        delay(300);
        xprint(".");
        now = time(nullptr);
        retries++;
        if (retries > 10) {
            xprintln("\nFailed to synchronize time with NTP.");
            return;
        }
    }
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    xprint("\nCurrent time: ");
    xprintln(asctime(&timeinfo));
}

// -----------------------------------------------------------------------------
// AZURE IOT HUB SDK CLIENT SETUP FUNCTION
// -----------------------------------------------------------------------------
bool setupAzureIoTClient() {
    xprintln("Setting up Azure IoT Hub client...");

    if (WiFi.status() != WL_CONNECTED) {
        xprintln("Wi-Fi not connected. Attempting Wi-Fi setup...");
        return false;
    }

    time_t now = time(nullptr);
    if (now < 8 * 3600 * 2) {
         xprintln("Time not synchronized. TLS connection might fail.");
    }

    // Initialize the Azure IoT platform (specific to some SDK wrappers for Arduino)
    if (platform_init() != 0) {
        xprintln("Failed to initialize the platform.");
        return false;
    }

    // Create the IoT Hub client handle from the connection string
    iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(IOTHUB_CONNECTION_STRING, MQTT_Protocol);
    if (iotHubClientHandle == NULL) {
        xprintln("Failed to create IoT Hub client handle.");
        platform_deinit();
        return false;
    }

    // Set options

    // Set trusted certificate (Root CA)
    // IMPORTANT: You MUST define the AZURE_IOT_ROOT_CA string yourself.
    if (strcmp(AZURE_IOT_ROOT_CA, "PASTE_YOUR_AZURE_IOT_ROOT_CA_CERTIFICATE_PEM_STRING_HERE") == 0 || strlen(AZURE_IOT_ROOT_CA) < 100) {
        xprintln("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        xprintln("ERROR: AZURE_IOT_ROOT_CA is not set or is a placeholder!");
        xprintln("You MUST provide the Azure IoT Hub Root CA certificate string.");
        xprintln("TLS connection will likely fail if this is not correctly set.");
        xprintln("Attempting to proceed without setting CA, this may fail severely.");
        xprintln("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        // Optionally, you could prevent further execution:
        // IoTHubClient_LL_Destroy(iotHubClientHandle);
        // platform_deinit();
        // return false;
    } else {
        if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_TRUSTED_CERT, AZURE_IOT_ROOT_CA) != IOTHUB_CLIENT_OK) {
            xprintln("Failed to set trusted CA certificate option.");
            // IoTHubClient_LL_Destroy(iotHubClientHandle); // Clean up
            // platform_deinit();
            // return false; // Decide if this is a fatal error for your setup
        } else {
            xprintln("Trusted CA certificate option set.");
        }
    }


    // Set MQTT Keep Alive interval (in seconds)
    int keepAliveIntervalSeconds = MQTT_KEEP_ALIVE_MINUTES * 60;
    if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_KEEP_ALIVE, &keepAliveIntervalSeconds) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set MQTT keep alive option.");
    } else {
        xprint("MQTT Keep Alive option set to: ");
        xprint(MQTT_KEEP_ALIVE_MINUTES);
        xprintln(" minutes.");
    }

    // Set the connection status callback
    if (IoTHubClient_LL_SetConnectionStatusCallback(iotHubClientHandle, connectionStatusCallback, NULL) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to set connection status callback.");
    }

    // Set the message callback (for Cloud-to-Device messages) - Optional
    // if (IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, receiveMessageCallback, NULL) != IOTHUB_CLIENT_OK) {
    //     xprintln("Failed to set message callback.");
    // }

    // Enable Wi-Fi power save mode if needed for your application
    // It's generally better to do this after the initial connection complexities are handled.
    // esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    // xprintln("Wi-Fi Max Modem Sleep enabled (if ESP32 core supports it well with SDK).");

    xprintln("Azure IoT Hub client setup complete. The SDK will attempt to connect in the loop via DoWork().");
    return true; // Setup is complete, connection attempt will happen in IoTHubClient_LL_DoWork
}

// -----------------------------------------------------------------------------
// SEND JSON DATA TO AZURE IOT HUB FUNCTION
// -----------------------------------------------------------------------------
bool sendJsonToAzure(const char* jsonPayload) {
    if (iotHubClientHandle == NULL || !iotHubConnected) {
        xprintln("Azure IoT Hub client not initialized or not connected. Cannot send data.");
        return false;
    }

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(jsonPayload);
    if (messageHandle == NULL) {
        xprintln("Failed to create IoT Hub message.");
        return false;
    }

    // Optional: Add message properties (e.g., content type)
    IoTHubMessage_SetContentTypeSystemProperty(messageHandle, "application%2Fjson");
    IoTHubMessage_SetContentEncodingSystemProperty(messageHandle, "utf-8");

    xprint("Sending JSON to Azure: ");
    xprintln(jsonPayload);

    if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, messageHandle, sendConfirmationCallback, (void*)messageHandle) != IOTHUB_CLIENT_OK) {
        xprintln("Failed to send message to Azure IoT Hub.");
        IoTHubMessage_Destroy(messageHandle); // Clean up message
        return false;
    }
    // The message handle is typically destroyed in the sendConfirmationCallback *after* successful send.
    // If SendEventAsync fails immediately, we destroy it here.
    // If it's accepted for sending, the callback is responsible for destroying it.
    // For simplicity in this example, if it fails here, we destroy.
    // A more robust implementation tracks handles passed to async calls.
    return true;
}

// -----------------------------------------------------------------------------
// CHECK AZURE IOT HUB CONNECTION STATUS FUNCTION
// -----------------------------------------------------------------------------
bool isAzureIoTConnected() {
    return iotHubConnected; // This flag is updated by the connectionStatusCallback
}

// -----------------------------------------------------------------------------
// SDK CALLBACKS
// -----------------------------------------------------------------------------

// Connection Status Callback
static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback) {
    iotHubConnected = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    xprint("\nConnection Status: ");
    switch (reason) {
        case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:      xprintln("Expired SAS token"); break;
        case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:        xprintln("Device disabled"); break;
        case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:         xprintln("Bad credential"); break;
        case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:          xprintln("Retry expired"); break;
        case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:             xprintln("No network"); break;
        case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:    xprintln("Communication error"); break;
        case IOTHUB_CLIENT_CONNECTION_OK:                     xprintln("OK"); break;
        case IOTHUB_CLIENT_CONNECTION_NO_PING_RESPONSE:       xprintln("No Ping Response"); break; // Keep-alive failure
        default:                                              xprintln("Unknown reason"); break;
    }
    if (iotHubConnected) {
        xprintln("Successfully connected to Azure IoT Hub.");
    } else {
        xprintln("Disconnected from Azure IoT Hub.");
    }
}

// Send Confirmation Callback
static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback) {
    IOTHUB_MESSAGE_HANDLE messageHandle = (IOTHUB_MESSAGE_HANDLE)userContextCallback;
    if (result == IOTHUB_CLIENT_CONFIRMATION_OK) {
        xprintln("Telemetry send confirmation: OK");
    } else {
        xprint("Telemetry send confirmation: FAILED, result: ");
        // You can map IOTHUB_CLIENT_CONFIRMATION_RESULT to a string here for more details
        xprintln(result);
    }
    if (messageHandle != NULL) {
      IoTHubMessage_Destroy(messageHandle); // IMPORTANT: Destroy the message handle after it's processed
    }
}

// Receive Message Callback (for Cloud-to-Device messages) - Optional
static IOTHUBMESSAGE_DISPOSITION_RESULT receiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback) {
    const unsigned char* buffer;
    size_t size;
    if (IoTHubMessage_GetByteArray(message, &buffer, &size) != IOTHUB_MESSAGE_OK) {
        xprintln("Failed to get message data.");
        return IOTHUBMESSAGE_REJECTED;
    } else {
        char* temp = (char*)malloc(size + 1);
        if (temp == NULL) {
            xprintln("Failed to allocate memory for message.");
            return IOTHUBMESSAGE_ABANDONED; // Or REJECTED
        }
        memcpy(temp, buffer, size);
        temp[size] = '\0';
        xprint("Received C2D Message: ");
        xprintln(temp);
        free(temp);
        return IOTHUBMESSAGE_ACCEPTED;
    }
}

// -----------------------------------------------------------------------------
// ARDUINO LOOP FUNCTION
// -----------------------------------------------------------------------------
unsigned long lastTelemetrySend = 0;
const long telemetryInterval = 60000; // Send telemetry every 60 seconds (example)

void loop() {
    if (iotHubClientHandle != NULL) {
        IoTHubClient_LL_DoWork(iotHubClientHandle); // CRITICAL: This processes network activity for the SDK
    } else {
        xprintln("IoT Hub client handle is NULL. Attempting to re-initialize...");
        delay(5000); // Wait before retrying setup
        setupAzureIoTClient(); // Try to set up again
        return; // Skip the rest of the loop if setup failed
    }

    // Example: Send telemetry periodically if connected
    unsigned long now = millis();
    if (isAzureIoTConnected() && (now - lastTelemetrySend > telemetryInterval)) {
        lastTelemetrySend = now;

        float temperature = 20.0 + (esp_random() % 100) / 10.0; // More ESP32-idiomatic random
        float humidity = 50.0 + (esp_random() % 200) / 10.0;
        char jsonBuffer[256];
        snprintf(jsonBuffer, sizeof(jsonBuffer),
                 "{\"temperature\":%.2f, \"humidity\":%.2f, \"deviceId\":\"%s\", \"messageId\":%lu}",
                 temperature, humidity, Provision::getInstance()->getUUID()->getStringc(), millis());

        sendJsonToAzure(jsonBuffer);
    }

    // Your other application logic can go here
    delay(100); // Small delay to prevent hammering the DoWork too fast if nothing else is happening
                // but ensure DoWork is called frequently enough.
}


const char* AZURE_IOT_ROOT_CA = "";