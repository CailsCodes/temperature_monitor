#include <Wire.h>
#include "DFRobot_WiFi_IoT_Module.h"
#include "WiFi_Credentials.h"
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
DFRobot_WiFi_IoT_Module_I2C wifiModule(&Wire, 0x16);

void setup() {

    // Start NeoPixel lights
    pinMode(20, OUTPUT);
    digitalWrite(20, HIGH);
    pixel.begin();
    pixel.setBrightness(10);
    pixel.show();

    //Connect to SHT41
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    //Init communication port 
    while(wifiModule.begin() != 0){  
      delay(100);
    }

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
    rainbow(freq);

}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void rainbow(int del) {
  uint16_t i, j, num_pixels;

  num_pixels = pixel.numPixels();
  // cycles of all colors on wheel
  for (j=0; j<256; j++) { 
    for(i=0; i< num_pixels; i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / num_pixels) + j) & 255));
      }
    pixel.show();
    delay(del/255);
  }
}
