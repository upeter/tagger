#include <Arduino.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"


 
bool initBluetooth__()
{
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
 
}
 
String getDeviceAddress__() {
    const uint8_t* point = esp_bt_dev_get_address();
    String addressString = "";

    for (int i = 0; i < 6; i++) {
        char str[3];
        sprintf(str, "%02X", (int)point[i]);
        addressString += str;

        if (i < 5) {
            addressString += ":";
        }
    }
    return addressString;
}

void setup_20() {
  Serial.begin(115200);
  
  delay(2000); // Pause for 2 seconds

  initBluetooth__();
  Serial.println("BlueTooth Address is: ");
   String mac_adr = getDeviceAddress__();
   Serial.println(mac_adr);


  
}
 
void loop_20() {
  
}