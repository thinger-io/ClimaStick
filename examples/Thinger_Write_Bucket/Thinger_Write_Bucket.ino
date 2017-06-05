#define _DEBUG_
#include <ClimaStick.h>

#define USERNAME "jt"
#define DEVICE_ID "climaMove"
#define DEVICE_CREDENTIAL "climaMove"

#define SSID "Trincado04"
#define SSID_PASSWORD "trincadoK7"

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
 long rssi;
void setup() {
  Serial.begin(115200);
  // configure board wifi
  thing.add_wifi(SSID, SSID_PASSWORD);
  // initialize board sensors
  thing.init_sensors();
  // define the "environment" resource
  thing.init_environment_resource();
  thing["RSSI"]>>outputValue(rssi);
}

void loop() { 
  thing.handle();
    rssi = WiFi.RSSI();
  
}
