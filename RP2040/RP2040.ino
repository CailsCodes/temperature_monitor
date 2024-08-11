#include <Wire.h>
#include "DFRobot_WiFi_IoT_Module.h"
#include "WiFi_Credentials.h"

DFRobot_WiFi_IoT_Module_I2C wifiModule(&Wire, 0x18);

void setup() {

    //Init communication port 
    while(wifiModule.begin() != 0){  
    //Serial.println("init ERROR!!!!");
    delay(100);
    }
    Serial.println("init Success");
    //Connect to WiFi
    while(wifiModule.connectWifi(WIFI_SSID, WIFI_PASSWORD) != 0){  
    //Serial.print(".");
    delay(100);
    }
    Serial.println("Wifi Connect Success");

    // MQTT configuration
    wifiModule.MQTTBegin(mqtt_server, mqtt_port, mqtt_client_id, mqtt_pwd, mqtt_device_id);

    // Subscribe to topic
    wifiModule.subscribe(mqtt_topic);

    //Connect to SHT41
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }
}

void loop() {

    if (Serial.available() > 0) {
        // read the incoming byte:
        String tempData = Serial.readString();
        tempData.trim();

        // Publish messages (example)
        wifiModule.publish(mqtt_topic, tempData);

        // Handle incoming messages (optional)
        wifiModule.loop();

      }
    int freq = 10 * 1000; // how frequently to read data (seconds)
    delay(freq);

}