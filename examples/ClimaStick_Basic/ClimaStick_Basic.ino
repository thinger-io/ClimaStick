#include <ClimaStick.h>
 
#define USERNAME "your_user_name"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
 
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  // configure board wifi
  thing.add_wifi(SSID, SSID_PASSWORD);
  // initialize board sensors
   // digital pin control example (i.e. turning on/off a light, a relay, configuring a parameter, etc)
  thing["led"] << digitalPin(BUILTIN_LED);

  // resource output example (i.e. reading a sensor value)
  thing["millis"] >> outputValue(millis());

  // more details at http://docs.thinger.io/arduino/
}

void loop() { 
  thing.handle();
}
