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

    thing["accelerometer"] >> [](pson& out){
        Accelerometer accel = thing.get_acceleration();
        out["ax"]=accel.ax;
        out["ay"]=accel.ay;
        out["az"]=accel.az;
    };
      
    thing["gyroscope"] >> [](pson& out){
        Gyroscope gyro = thing.get_gyroscope();
        out["gx"]=gyro.gx;
        out["gy"]=gyro.gy;
        out["gz"]=gyro.gz;
    };
    
    thing["compass"] >> [](pson& out){
        Compass compass = thing.get_compass();
        out["heading"]=compass.heading;
        out["degHeading"]=compass.headingDegrees;
    };
     
    thing["magnetometer"] >> [] (pson& out){
        Magnetometer magnet = thing.get_magnetometer();
        out["normX"]=magnet.x;
        out["normY"]=magnet.y;
        out["normZ"]=magnet.z;
    };

    thing["magnetometer_raw"] >> [] (pson& out){
        Magnetometer magnet = thing.get_raw_magnetometer();
        out["rawX"]=magnet.x;
        out["rawY"]=magnet.y;
        out["rawZ"]=magnet.z;
    };
}
 
void loop() {
    thing.handle();
}
