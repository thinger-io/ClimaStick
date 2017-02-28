#define _batterySwProtection_ //This parameter enables the auto-sleep mode when battery voltage flows down 3.65V
#include <ClimaStick.h>
 
#define USERNAME "your_user_name"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
 
void setup() {
  thing.add_wifi(SSID, SSID_PASSWORD);
  
  thing["battery"]  >> [](pson& out){
    out["voltaje"]=(float)batteryVoltaje();
    out["load"]=(float)batteryLoad();
  };
  
   thing["button"] >> outputValue(digitalRead(BUTTON));
}

void loop() { 
  thing.handle(); 
    
}