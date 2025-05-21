#include <Arduino.h>
#include <Tools.h>
#include "wifi_provisioning/manager.h"
#include <WiFi.h>
#include <config.h>
#include <UUID.h>
#include <WiFiProv.h>
#include <FreeRTOS/task.h>
#include <Preferences.h>

const char * service_name = "Prov_Moorgan_Colar"; // Name of your device (the Espressif apps expects by default device name starting with "Prov_")

// Global variables
bool wifi_connected = false; // Flag to check if WiFi is connected

Preferences pref;
UUID uuid;

void SysProvEvent(arduino_event_t *sys_event);

void loop(){}

void setup() {
  // variables that are discarded after setup finishes
  const char * pop = NULL; // Proof of possession - otherwise called a PIN - string provided by the device, entered by the user in the phone app
  const char * service_key = NULL; // Password used for SofAP method (NULL = no password needed)
  
  // starting serial communication if in DEVMODE
  #ifdef DEVMODE
    Serial.begin(115200);
  #endif


  // DEVICE INITIALIZATION
  xprintln("Device initialization");
  
  // uuid creation system (for unique id every device)
  pref.begin("uuid",true);
  // check if there's a uuid available or generated already
  pref.getBytes("uuid", uuid.getUUID(), uuid.getSize());
  if (!uuid.isUnset()) {
    xprintln("UUID already generated !");
    uuid.print();
    // if uuid is available then use it
  } else {
    xprintln("UUID not available !\nGenerating new UUID");
    // if not then end the pref.begin and open the pref.begin with readonly false
    pref.end();
    pref.begin("uuid", false);
    // generate the uuid array via class
    uuid.generate();
    uuid.print();
    // save the uuid in the nvs
    // this is the uuid that will be used for the device permanently
    pref.putBytes("uuid", uuid.getUUID(), uuid.getSize());
    xprintln("UUID generated and saved in NVS");
  }

  // then pref.end()
  pref.end();
  // this can be implemented in the tools, no need to do it here
  // DEVICE INITIALIZATION end

  // WiFi initialization
  xprintln("WiFi Provisioning initialization");
  WiFi.onEvent(SysProvEvent);

  xprintln("Begin Provisioning using BLE");
  // WiFiProv.beginProvision(
  WiFiProv.beginProvision(
    WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BLE, WIFI_PROV_SECURITY_1, pop, service_name, service_key, uuid.getUUID(), false
  );


  xTaskCreate(
    [] (void * param) {
      // this is the task that will be used to check if the device is connected to the wifi
      while (true) {
        if (WiFi.status() == WL_CONNECTED) {
          wifi_connected = true;
          xprintln("WiFi connected");
          break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    },
    "wifi_check",
    4096,
    NULL,
    1,
    NULL
  );

}

void SysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      xprint("\nConnected IP address : ");
      xprintln(IPAddress(sys_event->event_info.got_ip.ip_info.ip.addr));
      wifi_prov_mgr_deinit(); // De-initialize the provisioning manager to stop the provisioning process
      xprintln("Provisioning stopped");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: xprintln("\nDisconnected. Connecting to the AP again... "); break;
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