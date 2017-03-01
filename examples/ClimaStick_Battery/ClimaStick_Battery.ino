#include <ClimaStick.h>
 
#define USERNAME "your_user_name"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
 
void setup() {
  thing.add_wifi(SSID, SSID_PASSWORD);
  
  thing["battery"] >> [](pson& out){
    out["voltage"] = thing.get_battery_voltage();
    out["load"] = thing.get_battery_load();
  };
}

void loop() { 
  thing.handle();
}