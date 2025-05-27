// Include necessary libraries for Azure IoT Hub SDK
// The exact includes might vary slightly based on the specific Arduino wrapper
// library version for the Azure C SDK you have installed.
#include <WiFi.h>
#include <WiFiClientSecure.h> // Still used by the SDK for the underlying secure connection
#include <time.h>             // For time synchronization

// Azure IoT SDK specific headers
// These are typical, ensure they match your installed library
#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>
#include <AzureIoTUtility.h> // For platform_init, etc.

// -----------------------------------------------------------------------------
// CONFIGURATION - REPLACE WITH YOUR DETAILS
// -----------------------------------------------------------------------------

// Wi-Fi Credentials
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Azure IoT Hub Device Connection String
// Get this from Azure portal: IoT Hub -> Devices -> Your Device -> Primary Connection String
const char* IOTHUB_CONNECTION_STRING = "YOUR_DEVICE_CONNECTION_STRING";

// MQTT Keep-Alive setting for the SDK
const int MQTT_KEEP_ALIVE_MINUTES = 20;

// NTP server for time synchronization (important for TLS certificate validation)
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.nist.gov";
const long  GMT_OFFSET_SEC = 0;
const int   DAYLIGHT_OFFSET_SEC = 0;

// Azure IoT Hub Root CA Certificate
// IMPORTANT: YOU NEED TO PROVIDE THIS CERTIFICATE STRING.
// Define AZURE_IOT_ROOT_CA with the correct PEM formatted certificate.
// Example:
// const char* AZURE_IOT_ROOT_CA = \
// "-----BEGIN CERTIFICATE-----\n" \
// "MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n" \
// "... (rest of your certificate string) ...\n" \
// "-----END CERTIFICATE-----\n";
// Ensure this variable is defined before it's used to set the trusted certificate option.
const char* AZURE_IOT_ROOT_CA = "PASTE_YOUR_AZURE_IOT_ROOT_CA_CERTIFICATE_PEM_STRING_HERE";

// -----------------------------------------------------------------------------
// GLOBAL VARIABLES
// -----------------------------------------------------------------------------
static IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle; // Handle for the IoT Hub client
static bool iotHubConnected = false;              // Flag to track connection status

// -----------------------------------------------------------------------------
// FUNCTION DECLARATIONS
// -----------------------------------------------------------------------------
void setupWiFi();
void syncTimeNTP();
bool setupAzureIoTClient();
bool sendJsonToAzure(const char* jsonPayload);
bool isAzureIoTConnected();

// Callbacks for the Azure IoT SDK
static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* userContextCallback);
static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback);
// static IOTHUBMESSAGE_DISPOSITION_RESULT receiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback); // For C2D messages

// -----------------------------------------------------------------------------
// WIFI SETUP FUNCTION
// -----------------------------------------------------------------------------
void setupWiFi() {
    Serial.println();
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        retries++;
        if (retries > 40) {
            Serial.println("\nFailed to connect to Wi-Fi.");
            return;
        }
    }
    Serial.println("\nWi-Fi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

// -----------------------------------------------------------------------------
// TIME SYNCHRONIZATION FUNCTION (NTP)
// -----------------------------------------------------------------------------
void syncTimeNTP() {
    Serial.println("Synchronizing time with NTP server...");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);

    time_t now = time(nullptr);
    int retries = 0;
    while (now < 8 * 3600 * 2) { // Check if time is reasonably set
        delay(500);
        Serial.print(".");
        now = time(nullptr);
        retries++;
        if (retries > 30) {
            Serial.println("\nFailed to synchronize time with NTP.");
            return;
        }
    }
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.print("\nCurrent time: ");
    Serial.println(asctime(&timeinfo));
}

// -----------------------------------------------------------------------------
// AZURE IOT HUB SDK CLIENT SETUP FUNCTION
// -----------------------------------------------------------------------------
bool setupAzureIoTClient() {
    Serial.println("Setting up Azure IoT Hub client...");

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi not connected. Attempting Wi-Fi setup...");
        setupWiFi();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Wi-Fi connection failed. Cannot proceed.");
            return false;
        }
    }

    syncTimeNTP();
    time_t now = time(nullptr);
    if (now < 8 * 3600 * 2) {
         Serial.println("Time not synchronized. TLS connection might fail.");
    }

    // Initialize the Azure IoT platform (specific to some SDK wrappers for Arduino)
    if (platform_init() != 0) {
        Serial.println("Failed to initialize the platform.");
        return false;
    }

    // Create the IoT Hub client handle from the connection string
    iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(IOTHUB_CONNECTION_STRING, MQTT_Protocol);
    if (iotHubClientHandle == NULL) {
        Serial.println("Failed to create IoT Hub client handle.");
        platform_deinit();
        return false;
    }

    // Set options

    // Set trusted certificate (Root CA)
    // IMPORTANT: You MUST define the AZURE_IOT_ROOT_CA string yourself.
    if (strcmp(AZURE_IOT_ROOT_CA, "PASTE_YOUR_AZURE_IOT_ROOT_CA_CERTIFICATE_PEM_STRING_HERE") == 0 || strlen(AZURE_IOT_ROOT_CA) < 100) {
        Serial.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("ERROR: AZURE_IOT_ROOT_CA is not set or is a placeholder!");
        Serial.println("You MUST provide the Azure IoT Hub Root CA certificate string.");
        Serial.println("TLS connection will likely fail if this is not correctly set.");
        Serial.println("Attempting to proceed without setting CA, this may fail severely.");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        // Optionally, you could prevent further execution:
        // IoTHubClient_LL_Destroy(iotHubClientHandle);
        // platform_deinit();
        // return false;
    } else {
        if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_TRUSTED_CERT, AZURE_IOT_ROOT_CA) != IOTHUB_CLIENT_OK) {
            Serial.println("Failed to set trusted CA certificate option.");
            // IoTHubClient_LL_Destroy(iotHubClientHandle); // Clean up
            // platform_deinit();
            // return false; // Decide if this is a fatal error for your setup
        } else {
            Serial.println("Trusted CA certificate option set.");
        }
    }


    // Set MQTT Keep Alive interval (in seconds)
    int keepAliveIntervalSeconds = MQTT_KEEP_ALIVE_MINUTES * 60;
    if (IoTHubClient_LL_SetOption(iotHubClientHandle, OPTION_KEEP_ALIVE, &keepAliveIntervalSeconds) != IOTHUB_CLIENT_OK) {
        Serial.println("Failed to set MQTT keep alive option.");
    } else {
        Serial.print("MQTT Keep Alive option set to: ");
        Serial.print(MQTT_KEEP_ALIVE_MINUTES);
        Serial.println(" minutes.");
    }

    // Set the connection status callback
    if (IoTHubClient_LL_SetConnectionStatusCallback(iotHubClientHandle, connectionStatusCallback, NULL) != IOTHUB_CLIENT_OK) {
        Serial.println("Failed to set connection status callback.");
    }

    // Set the message callback (for Cloud-to-Device messages) - Optional
    // if (IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, receiveMessageCallback, NULL) != IOTHUB_CLIENT_OK) {
    //     Serial.println("Failed to set message callback.");
    // }

    // Enable Wi-Fi power save mode if needed for your application
    // It's generally better to do this after the initial connection complexities are handled.
    // esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    // Serial.println("Wi-Fi Max Modem Sleep enabled (if ESP32 core supports it well with SDK).");

    Serial.println("Azure IoT Hub client setup complete. The SDK will attempt to connect in the loop via DoWork().");
    return true; // Setup is complete, connection attempt will happen in IoTHubClient_LL_DoWork
}

// -----------------------------------------------------------------------------
// SEND JSON DATA TO AZURE IOT HUB FUNCTION
// -----------------------------------------------------------------------------
bool sendJsonToAzure(const char* jsonPayload) {
    if (iotHubClientHandle == NULL || !iotHubConnected) {
        Serial.println("Azure IoT Hub client not initialized or not connected. Cannot send data.");
        return false;
    }

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(jsonPayload);
    if (messageHandle == NULL) {
        Serial.println("Failed to create IoT Hub message.");
        return false;
    }

    // Optional: Add message properties (e.g., content type)
    IoTHubMessage_SetContentTypeSystemProperty(messageHandle, "application%2Fjson");
    IoTHubMessage_SetContentEncodingSystemProperty(messageHandle, "utf-8");

    Serial.print("Sending JSON to Azure: ");
    Serial.println(jsonPayload);

    if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle, messageHandle, sendConfirmationCallback, (void*)messageHandle) != IOTHUB_CLIENT_OK) {
        Serial.println("Failed to send message to Azure IoT Hub.");
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
    Serial.print("\nConnection Status: ");
    switch (reason) {
        case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:      Serial.println("Expired SAS token"); break;
        case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:        Serial.println("Device disabled"); break;
        case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:         Serial.println("Bad credential"); break;
        case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:          Serial.println("Retry expired"); break;
        case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:             Serial.println("No network"); break;
        case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:    Serial.println("Communication error"); break;
        case IOTHUB_CLIENT_CONNECTION_OK:                     Serial.println("OK"); break;
        case IOTHUB_CLIENT_CONNECTION_NO_PING_RESPONSE:       Serial.println("No Ping Response"); break; // Keep-alive failure
        default:                                              Serial.println("Unknown reason"); break;
    }
    if (iotHubConnected) {
        Serial.println("Successfully connected to Azure IoT Hub.");
    } else {
        Serial.println("Disconnected from Azure IoT Hub.");
    }
}

// Send Confirmation Callback
static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback) {
    IOTHUB_MESSAGE_HANDLE messageHandle = (IOTHUB_MESSAGE_HANDLE)userContextCallback;
    if (result == IOTHUB_CLIENT_CONFIRMATION_OK) {
        Serial.println("Telemetry send confirmation: OK");
    } else {
        Serial.print("Telemetry send confirmation: FAILED, result: ");
        // You can map IOTHUB_CLIENT_CONFIRMATION_RESULT to a string here for more details
        Serial.println(result);
    }
    if (messageHandle != NULL) {
      IoTHubMessage_Destroy(messageHandle); // IMPORTANT: Destroy the message handle after it's processed
    }
}

// Receive Message Callback (for Cloud-to-Device messages) - Optional
/*
static IOTHUBMESSAGE_DISPOSITION_RESULT receiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback) {
    const unsigned char* buffer;
    size_t size;
    if (IoTHubMessage_GetByteArray(message, &buffer, &size) != IOTHUB_MESSAGE_OK) {
        Serial.println("Failed to get message data.");
        return IOTHUBMESSAGE_REJECTED;
    } else {
        char* temp = (char*)malloc(size + 1);
        if (temp == NULL) {
            Serial.println("Failed to allocate memory for message.");
            return IOTHUBMESSAGE_ABANDONED; // Or REJECTED
        }
        memcpy(temp, buffer, size);
        temp[size] = '\0';
        Serial.print("Received C2D Message: ");
        Serial.println(temp);
        free(temp);
        return IOTHUBMESSAGE_ACCEPTED;
    }
}
*/

// -----------------------------------------------------------------------------
// ARDUINO SETUP FUNCTION
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);

    Serial.println("Azure IoT Hub SDK Client for ESP32");

    if (!setupAzureIoTClient()) {
        Serial.println("Failed to initialize Azure IoT Hub client. Check configurations.");
        // You might want to loop here or restart ESP
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
        Serial.println("IoT Hub client handle is NULL. Attempting to re-initialize...");
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
                 temperature, humidity, DEVICE_ID, millis()); // Using DEVICE_ID from config

        sendJsonToAzure(jsonBuffer);
    }

    // Your other application logic can go here
    delay(100); // Small delay to prevent hammering the DoWork too fast if nothing else is happening
                // but ensure DoWork is called frequently enough.
}
