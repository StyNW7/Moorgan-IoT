#include <Arduino.h>
#include "Provision.h"
#include "wifi_provisioning/manager.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <IOTHubInstance.h>
#include <WiFiProv.h>
#include <config.h>
#include <Tools.h>
#include <UUID.h>
#include <Preferences.h>
#include <freertos/task.h>

void provision_wifi_check (void *in) ;
void SysProvEvent(arduino_event_t *sys_event, Provision *provision);

#ifdef DEVMODE
typedef struct {
    char *pop;
    char *service_name;
    bool *connected;
} ProvisioningData;
#endif

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
    this->uuid = new UUID();
    this->retry_connection_counter=0;
    this->redo_provision = false;
    this->is_provisioned = false;
    this->connected = false;
    this->pop = NULL;
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
    return this->is_provisioned;
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

void Provision::setPop(const char * pop) {
    if(this->pop){
        free(this->pop);
        this->pop = NULL;
    }

    this->pop = (char *)malloc(strlen(pop) + 1); // +1 for null-terminator
    strcpy(this->pop, pop);
}


void Provision::setupProvision() {
    // Suspend all tasks except the current one
    vTaskSuspendAll();

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
            IOTHubInstance::syncTimeNTP();
            if (!IOTHubInstance::setupAzureIoTClient()) {
                xprintln("Failed to initialize Azure IoT Hub client. Check configurations.");
                // You might want to loop here or restart ESP
            }
            esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
            
            xprintln("Time configured via NTP.");
            break;
        }
        delay(1000);// blocking delay
    }
    
    is_provisioned = true;
    xTaskResumeAll();
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
        xprint("\nConnected IP address : ");
        xprintln(IPAddress(sys_event->event_info.got_ip.ip_info.ip.addr));
        wifi_prov_mgr_deinit(); // De-initialize the provisioning manager to stop the provisioning process
        xprintln("Provisioning stopped");

        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
        if(provision->getRetryConnectionCounter() >= CONNECTIONRETRYCOUNT){
            provision->setConnected(false);
            vTaskSuspendAll();
            WiFiProv.beginProvision(
                WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BLE, WIFI_PROV_SECURITY_1, provision->getPop(), PROVISIONING_SERVICE_NAME, NULL, provision->getUUID()->getUUID(), false
            );
            xprintln("Provisioning started");
        } else {
            provision->addToRetryConnectionCounter();
        }

        break;
    }
    case ARDUINO_EVENT_PROV_START:            xprintln("\nProvisioning started\nGive Credentials of your access point using smartphone app"); break;
    case ARDUINO_EVENT_PROV_CRED_RECV:
    {
        xprintln("\nReceived Wi-Fi credentials");
        xprint("\tSSID : ");
        xprintln((const char *)sys_event->event_info.prov_cred_recv.ssid);
        xprint("\tPassword : ");
        xprintln((char const *)sys_event->event_info.prov_cred_recv.password);
        break;
    }
    case ARDUINO_EVENT_PROV_CRED_FAIL:
    {
        xprintln("\nProvisioning failed!\nPlease reset to factory and retry provisioning\n");
        if (sys_event->event_info.prov_fail_reason == WIFI_PROV_STA_AUTH_ERROR) {
            xprintln("\nWi-Fi AP password incorrect");
        } else {
            xprintln("\nWi-Fi AP not found....Add API \"");
        }

        xprintln("Resetting device to factory settings");
        // reset the device to factory settings

        break;
    }
    case ARDUINO_EVENT_PROV_CRED_SUCCESS: xprintln("\nProvisioning Successful"); break;
    case ARDUINO_EVENT_PROV_END:          xprintln("\nProvisioning Ends"); break;
    default:                              break;
    }
}