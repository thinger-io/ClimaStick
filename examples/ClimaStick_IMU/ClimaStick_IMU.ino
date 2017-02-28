#include <ClimaStick.h>
 
#define USERNAME "your_user_name"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
  
void setup() {
    thing.add_wifi(SSID, SSID_PASSWORD);
      
    builtinSensorsBegin();  //Inicializing sensors
     
    thing["accelerometer"] >> [](pson& out){
        getMotion();
        out["ax"]=accel.x;
        out["ay"]=accel.y;
        out["az"]=accel.z;
        };
      
    thing["gyroscope"] >> [](pson& out){
        getMotion();
        out["gx"]=gyro.x;
        out["gy"]=gyro.y;
        out["gz"]=gyro.z;
        };
    
    thing["compass"] >> [](pson& out){
        getCompass();
        out["heading"]=compass.heading;
        out["degHeading"]=compass.headingDegrees;
        };
     
    thing["magnetometer"] >> [] (pson& out){
        getMagnet();
        out["rawX"]=magnet.x;
        out["rawY"]=magnet.y;
        out["rawZ"]=magnet.z;
        out["normX"]=magnet.nx;
        out["normY"]=magnet.ny;
        out["normZ"]=magnet.nz;
        };
   
}
 
void loop() {
     
    thing.handle();
 
}
