/*This example will read some variables from enviromental sensors of ClimaStick and send it to 
 * another device using Thinger.io call_device feature. you will need to create two different devices:
 * An emitter (this) and a receiver one (the other device) with the code that we have included into 
 * "Thinger_CallDevice_receiver.ino" example.
 */
 
#include <ClimaStick.h>

#define USERNAME "your_user_name"
#define DEVICE_ID "your_device_id"
#define DEVICE_CREDENTIAL "your_device_credential"

#define SSID "your_wifi_ssid"
#define SSID_PASSWORD "your_wifi_ssid_password"

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

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
  // Call to "receiverDevice" and send "environment" resource data to "externalEnvironment" resource of destinyDevice
  thing.call_device("receiverDevice", "externalEnvironment", thing["environment"]);
  // sleep the device 60 seconds
  thing.sleep(60);
}
