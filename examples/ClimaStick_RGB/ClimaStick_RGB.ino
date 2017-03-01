#include <ClimaStick.h>
 
#define USERNAME "your_user_name"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
  
void setup() {
    thing.add_wifi(SSID, SSID_PASSWORD);
      
    thing["RGB"] << [](pson& in){
        if(in.is_empty()){
            RGB rgb =  thing.get_rgb();
            in["r"] = rgb.r;
            in["g"] = rgb.g;
            in["b"] = rgb.b;
        }else{
            thing.set_rgb(in["r"], in["g"], in["b"]);
        }
    };

    thing["setColorByName"] << [](pson& in){
        String colorName = in["ColorName"];
        thing.set_rgb(colorName);
    };
}
 
void loop() {
   thing.handle();
}