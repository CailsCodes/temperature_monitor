// SPDX-FileCopyrightText: 2024 Limor Fried for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include <Adafruit_SHT4x.h>
#include <Adafruit_FreeTouch.h>
#include <Adafruit_NeoPixel.h>

// Declare NeoTrinkey IO
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_FreeTouch touch = Adafruit_FreeTouch(1, OVERSAMPLE_4, RESISTOR_50K, FREQ_MODE_NONE);
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setup() {

  // Start serial interface
  Serial.begin(115200);

  while (!Serial) {
    delay(10);     // will pause until serial console opens
  }
  
  // Start data interface
  if (! sht4.begin()) {
    Serial.println("# Couldn't find SHT4x");
    while (1) delay(1);
  }

  // Declare mode
  sht4.setPrecision(SHT4X_HIGH_PRECISION);  
  sht4.setHeater(SHT4X_NO_HEATER);

  // Can touch input be detected?
  if (! touch.begin())  {
      Serial.println("Failed to begin QTouch on pin 1");
    }


  // Start NeoPixel lights
  pixel.begin();
  pixel.setBrightness(10);
  pixel.show();

  // Print headers:
  Serial.println("Temperature in *C, Relative Humidity %, Vapour-Pressure Deficit in kPa, Touched");
}

void loop() {
  sensors_event_t humidity, temp;
  bool touched = false;
  int freq = 10 * 1000; // how frequently to read data (seconds)

  if (touch.measure() > 250){
    touched = true;
  }

  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  float svp = 0.6108 * exp(17.27 * temp.temperature / (temp.temperature + 237.3));
  // actual vapor pressure
  float avp = humidity.relative_humidity / 100 * svp;
  // VPD = saturation vapor pressure - actual vapor pressure
  float vpd = svp - avp;

  // Output data to serial
  Serial.print(temp.temperature);
  Serial.print(", ");
  Serial.print(humidity.relative_humidity);
  Serial.print(", ");
  Serial.print(vpd);
  Serial.print(", ");
  Serial.println(touched);

  rainbow(freq);

  // 10 second between readings
  // delay(10000);  
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



