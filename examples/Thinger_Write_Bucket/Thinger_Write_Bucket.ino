#include <ClimaStick.h>

#define USERNAME "your_user_name"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);
int timer = 0;
void setup() {
   // configure board wifi
  thing.add_wifi(SSID, SSID_PASSWORD);
  // initialize board sensors
  thing.init_sensors();
  // define the "environment" resource
  thing.init_environment_resource();
}

void loop() { 
  thing.handle();
     
   if(millis()>timer+(60000*10)){  //call each 10 minutes
     //call thinger.io endpoint function and attacht the pson
     thing.write_bucket("bucket_id", "environment");
     //actualize time counter
     timer = millis();
   }
  
}
