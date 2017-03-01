#include <ClimaStick.h>
 
#define USERNAME "your_user_name"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
  
void setup() {
    thing.add_wifi(SSID, SSID_PASSWORD);
    thing.init_sensors();

    thing["environment"] >> [] (pson& out){
        Environmental clima = thing.get_clima();
        out["temperature"]=clima.temperature;
        out["humidity"]=clima.humidity;
        out["altitude"]=clima.altitude;
        out["pressure"]=clima.pressure;
        out["lux"]=clima.lux;
    };
}
 
void loop() {
   thing.handle();
}