/*This example is ready to receive a PSON with all environmental data, sended by an other climaStick though a 
 * Thinger.io call_device() instruction. After receive this data we will be able to use it for different purposes
 */
 
#include <ClimaStick.h>

#define USERNAME "your_user_name"
#define DEVICE_ID "receiverDevice"  //DEVICE_ID should be the same as we used in the call_device instruction
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

void setup() {
  // configure board wifi
  thing.add_wifi(SSID, SSID_PASSWORD);
 
  thing[“externalEnvironment”] << [](pson& in){   //resource name should be the same as we used in the call_device instruction
      float received_temperature = in["temperature"];
      float received_temperature = in["humidity"];
      float received_temperature = in["altitude"];
      float received_temperature = in["pressure"];
      float received_temperature = in["lux"];
    };
}

 
}

void loop() { 
  thing.handle();
 
}
