#include <Arduino.h>
#include "Provision.h"
#include "wifi_provisioning/manager.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <IOTHubInstance.h>
#include <WiFiProv.h>
#include <config.h>
#include <Tools.h>
#include <UUIDs.h>
#include <Preferences.h>
#include <freertos/task.h>

void SysProvEvent(arduino_event_t *sys_event, Provision *provision);


// Initialize the static instance pointer to nullptr
Provision* Provision::instance = nullptr;

// Implement the getInstance method
Provision* Provision::getInstance() {
    if (instance == nullptr) {
        instance = new Provision();
    }
    return instance;
}

Provision::Provision() {
    this->uuid = new UUIDs();
    this->retry_connection_counter=0;
    this->redo_provision = false;
    this->is_provisioned = false;
    this->connected = false;
    this->pop = NULL;
    this->isAttemptingConnectionPhase = false;
    this->connectionAttemptStartTimeMs = 0;
    this->isBleProvCurrentlyActive = false;
}

Provision::~Provision() {
    delete this->uuid;
    this->uuid = NULL;
    
    if (this->pop) {
        free(this->pop);
        this->pop = NULL;
    }
    
    // Reset the instance pointer if the singleton is destroyed
    if (instance == this) {
        instance = nullptr;
    }
}

bool Provision::getIsProvisioned(){
    return this->is_provisioned;
}

uint8_t Provision::getRetryConnectionCounter(){
    return this->retry_connection_counter;
}

void Provision::setIsProvisioned(bool tof){
    this->is_provisioned = tof;
}

void Provision::addToRetryConnectionCounter(){
    ++(this->retry_connection_counter);
}

void Provision::resetRetryConnectionCounter(){
    this->retry_connection_counter = 0;
}

bool Provision::getIsAttemptingConnectionPhase() const {
    return this->isAttemptingConnectionPhase;
}

void Provision::setIsAttemptingConnectionPhase(bool status) {
    this->isAttemptingConnectionPhase = status;
}

unsigned long Provision::getConnectionAttemptStartTimeMs() const {
    return this->connectionAttemptStartTimeMs;
}

void Provision::setConnectionAttemptStartTimeMs(unsigned long time) {
    this->connectionAttemptStartTimeMs = time;
}

bool Provision::getIsBleProvCurrentlyActive() const {
    return this->isBleProvCurrentlyActive;
}

void Provision::setIsBleProvCurrentlyActive(bool status) {
    this->isBleProvCurrentlyActive = status;
}

void Provision::setConnected(bool status) {
    this->connected = status;
    if (status) { // If successfully connected
        this->isAttemptingConnectionPhase = false; // End the attempt phase
        this->isBleProvCurrentlyActive = false;    // BLE provisioning is not active if connected
    }
}

void Provision::setPop(const char * pop) {
    if(this->pop){
        free(this->pop);
        this->pop = NULL;
    }

    this->pop = (char *)malloc(strlen(pop) + 1); // +1 for null-terminator
    strcpy(this->pop, pop);
}


void Provision::setupProvision() {
    #ifndef PRE_REGISTERED_DEVICE_UUID
    Preferences pref;
    // uuid creation system (for unique id every device)
    pref.begin("uuid",true);
    // check if there's a uuid available or generated already
    pref.getBytes("uuid", this->uuid->getUUID(), this->uuid->getSize());
    if (!this->uuid->isUnset()) {
        xprintln("UUID already generated !");
        this->uuid->print();
        // if uuid is available then use it
    } else {
        xprintln("UUID not available !\nGenerating new UUID");
        // if not then end the pref.begin and open the pref.begin with readonly false
        pref.end();
        pref.begin("uuid", false);
        // generate the uuid array via class
        this->uuid->generate();
        this->uuid->print();
        // save the uuid in the nvs
        // this is the uuid that will be used for the device permanently
        pref.putBytes("uuid", this->uuid->getUUID(), this->uuid->getSize());
        xprintln("UUID generated and saved in NVS");
    }
    // then pref.end()
    pref.end();
    #else
    xprintln("Using pre-registered UUID:");
    xprint(this->uuid->fromString(PRE_REGISTERED_DEVICE_UUID));
    #endif
    // this can be implemented in the tools, no need to do it here
    // DEVICE INITIALIZATION end

    // WiFi initialization
    xprintln("WiFi Provisioning initialization");
    
    // Use the singleton instance instead of creating a new one
    WiFi.onEvent([](arduino_event_t *sys_event) {
        SysProvEvent(sys_event, Provision::getInstance());
    });

    xprintln("Begin Provisioning using BLE");
    // WiFiProv.beginProvision(
    WiFiProv.beginProvision(
        WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BLE, WIFI_PROV_SECURITY_1, this->pop, PROVISIONING_SERVICE_NAME, NULL, this->uuid->getUUID(), false
    );

    xprintln("Connecting to WiFi...");
    WiFiProv.printQR(PROVISIONING_SERVICE_NAME, (const char*)this->pop, "BLE");
    while (true) {
        if (WiFi.status() == WL_CONNECTED) {
            xprintln("WiFi connected !!!");
            this->connected = true;
            // setup time
            esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
            
            xprintln("Time configured via NTP.");
            break;
        }
        delay(1000);// blocking delay
    }
    is_provisioned = true;
}


/******************************************************
|                                                     |
|    These are callback function for the events       |
|                                                     |   
******************************************************/
void SysProvEvent(arduino_event_t *sys_event, Provision *provision) {
    switch (sys_event->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        provision->resetRetryConnectionCounter();
        provision->setConnected(true); // Assuming this method exists or you add it
        provision->setIsAttemptingConnectionPhase(false); // Successfully connected, end attempt phase
        provision->setIsBleProvCurrentlyActive(false);
        xprint("\nConnected IP address : ");
        xprintln(IPAddress(sys_event->event_info.got_ip.ip_info.ip.addr));
        
        // De-initialize provisioning manager only after successful connection and IP.
        // This was in your original ARDUINO_EVENT_WIFI_STA_GOT_IP
        wifi_prov_mgr_deinit(); 
        xprintln("Provisioning manager de-initialized.");
        provision->setIsProvisioned(true); // Mark as provisioned

        // The rest of your GOT_IP logic (NTP, Azure client setup)
        IOTHubInstance::getInstance()->syncTimeNTP();
        if (!IOTHubInstance::getInstance()->setupAzureIoTClient()) {
            xprintln("Failed to initialize Azure IoT Hub client. Check configurations.");
        }
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        xprintln("Time configured via NTP.");
        break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
        xprintln("WiFi Disconnected.");
        provision->setConnected(false); // Mark as not connected

        if (!provision->getIsAttemptingConnectionPhase()) {
            // Start of a new disconnection/reconnection/provisioning attempt cycle
            provision->setIsAttemptingConnectionPhase(true);
            provision->setConnectionAttemptStartTimeMs(millis());
            provision->resetRetryConnectionCounter(); // Reset retries for this new cycle
            xprintln("Starting new connection/provisioning attempt cycle.");
        }

        if (provision->getRetryConnectionCounter() >= CONNECTIONRETRYCOUNT) {
            xprintln("Max WiFi reconnection retries reached. Attempting BLE provisioning.");
            // No need to call provision->setConnected(false) again, already done.
            WiFiProv.beginProvision(
                WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BLE, WIFI_PROV_SECURITY_1, provision->getPop(), PROVISIONING_SERVICE_NAME, NULL, provision->getUUID()->getUUID(), false
            );
            // Note: ARDUINO_EVENT_PROV_START will set isBleProvCurrentlyActive = true
        } else {
            provision->addToRetryConnectionCounter();
            xprintf("WiFi reconnection attempt #%d\n", provision->getRetryConnectionCounter());
            WiFi.begin(); // Explicitly try to reconnect with stored credentials
        }

        // Check for overall timeout of the current attempt phase
        if (provision->getIsAttemptingConnectionPhase() && (millis() - provision->getConnectionAttemptStartTimeMs() >= TIMEOUTPROVISION)) {
            xprintln("Connection/Provisioning attempt timed out.");
            
            if (provision->getIsBleProvCurrentlyActive()) {
                xprintln("Stopping active BLE provisioning due to timeout.");
                wifi_prov_mgr_stop_provisioning(); // Stop BLE if it was active
                provision->setIsBleProvCurrentlyActive(false);
            }
            
            xflush(); // Ensure all serial data is sent before sleeping
            esp_sleep_enable_timer_wakeup(PROVISIONSLEEPT); // set to microseconds
            xprintln("Entering light sleep due to timeout...");
            esp_light_sleep_start();
            
            // After waking up, if still disconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED will be triggered again.
            // To ensure the logic attempts provisioning next if it was a timeout:
            provision->resetRetryConnectionCounter(); // Reset counter
            provision->addToRetryConnectionCounter(); // Increment once
            while(provision->getRetryConnectionCounter() < CONNECTIONRETRYCOUNT) { // Ensure it reaches count for provisioning
                 provision->addToRetryConnectionCounter();
            }
            // Or simpler:
            // provision->retry_connection_counter = CONNECTIONRETRYCOUNT; // Directly set to trigger provisioning next time
            
            provision->setIsAttemptingConnectionPhase(false); // Reset for the next cycle after wake-up
            // The WiFiProv.beginProvision call here is removed because the event system will re-trigger
            // this handler if still disconnected, and the logic above will then decide to start provisioning
            // if retry_connection_counter is >= CONNECTIONRETRYCOUNT.
        }
        break;
    }
    case ARDUINO_EVENT_PROV_START:
        xprintln("\nProvisioning started via BLE. Give Credentials of your access point using smartphone app");
        provision->setIsBleProvCurrentlyActive(true);
        // If provisioning starts, ensure we are in an "attempting" phase.
        if (!provision->getIsAttemptingConnectionPhase()) {
            provision->setIsAttemptingConnectionPhase(true);
            provision->setConnectionAttemptStartTimeMs(millis());
        }
        WiFiProv.printQR(PROVISIONING_SERVICE_NAME, (const char*)provision->getPop(), "BLE"); // Print QR on PROV_START
        break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
    {
        xprintln("\nReceived Wi-Fi credentials via BLE");
        xprint("\tSSID : ");
        xprintln((const char *)sys_event->event_info.prov_cred_recv.ssid);
        xprint("\tPassword : ");
        xprintln((char const *)sys_event->event_info.prov_cred_recv.password);
        provision->setIsBleProvCurrentlyActive(false); // BLE part done for this session
        break;
    }
    case ARDUINO_EVENT_PROV_CRED_FAIL:
    {
        xprintln("\nProvisioning failed via BLE!");
        // ... (your existing logging) ...
        provision->setIsBleProvCurrentlyActive(false);
        // Keep isAttemptingConnectionPhase = true, let timeout or further disconnects handle it.
        break;
    }
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
        xprintln("\nProvisioning Credentials Accepted via BLE. Attempting to connect to AP...");
        provision->setIsBleProvCurrentlyActive(false); // Credentials accepted, BLE interaction over.
        break;
    case ARDUINO_EVENT_PROV_END:
        xprintln("\nProvisioning Process Ended (BLE session ended).");
        provision->setIsBleProvCurrentlyActive(false);
        // If not yet connected, isAttemptingConnectionPhase remains true.
        // The system will try to connect with new credentials. If that fails, STA_DISCONNECTED will occur.
        break;
    default:
        break;
    }
}