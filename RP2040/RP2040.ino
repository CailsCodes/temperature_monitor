#include <Wire.h>
// #include "FreeRTOS.h"
#include "cppQueue.h"

// WiFi module imports
#include "DFRobot_WiFi_IoT_Module.h"
#include "WiFi_Credentials.h"

#include <Adafruit_NeoPixel.h>

// USB interface imports
#include "Adafruit_TinyUSB.h"

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
DFRobot_WiFi_IoT_Module_I2C wifiModule(&Wire, 0x16);
cppQueue dataQueue(sizeof(uint8_t), 5, FIFO, true);

const int freq = 5 * 1000; // how frequently to read data (seconds)

#define LANGUAGE_ID 0x0409  // English

Adafruit_USBH_Host USBHost;
Adafruit_USBD_CDC USBSer1;

// Defining values for USB host config
typedef enum {
  PIO_USB_PINOUT_DPDM = PIN_USB_HOST_DP,  // DM = DP+1
  PIO_USB_PINOUT_DMDP,      // DM = DP-1
} PIO_USB_PINOUT;

typedef struct {
    uint8_t pin_dp;
    uint8_t pio_tx_num;
    uint8_t sm_tx;
    uint8_t tx_ch;
    uint8_t pio_rx_num;
    uint8_t sm_rx;
    uint8_t sm_eop;
    void* alarm_pool;
    int8_t debug_pin_rx;
    int8_t debug_pin_eop;
    bool skip_alarm_pool;
    PIO_USB_PINOUT pinout;
} pio_usb_configuration_t;

// #ifndef PIN_USB_HOST_DP
// #define PIN_USB_HOST_DP  16
// #endif

#ifndef PIO_USB_DP_PIN_DEFAULT
#define PIO_USB_DP_PIN_DEFAULT 16
#endif

#define PIO_USB_TX_DEFAULT 0
#define PIO_SM_USB_TX_DEFAULT 0
#define PIO_USB_DMA_TX_DEFAULT 0
#define PIO_USB_RX_DEFAULT 0
#define PIO_SM_USB_RX_DEFAULT 1
#define PIO_SM_USB_EOP_DEFAULT 2
#define PIO_USB_DEBUG_PIN_NONE (-1)
#define PIO_USB_EP_POOL_CNT 32
#define PIO_USB_DEV_EP_CNT 16
#define PIO_USB_DEVICE_CNT 4
#define PIO_USB_HUB_PORT_CNT 8
#define PIO_USB_ROOT_PORT_CNT 2
#define PIO_USB_EP_SIZE 64

#define PIO_USB_DEFAULT_CONFIG                                             \
  {                                                                        \
    PIO_USB_DP_PIN_DEFAULT, PIO_USB_TX_DEFAULT, PIO_SM_USB_TX_DEFAULT,     \
        PIO_USB_DMA_TX_DEFAULT, PIO_USB_RX_DEFAULT, PIO_SM_USB_RX_DEFAULT, \
        PIO_SM_USB_EOP_DEFAULT, NULL, PIO_USB_DEBUG_PIN_NONE,              \
        PIO_USB_DEBUG_PIN_NONE, false, PIO_USB_PINOUT_DPDM                 \
  }


void setup() {
  Serial.begin(115200);

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
    while ( !Serial ) delay(10);   // wait for native usb
    Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while(1) delay(1);
  }

  // Start NeoPixel lights
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  pixel.begin();
  pixel.setBrightness(10);
  pixel.show();

  //Init communication port 
  while(wifiModule.begin() != 0){  
    delay(100);
  }

  //Connect to WiFi
  while(wifiModule.connectWifi(WIFI_SSID, WIFI_PASSWORD) != 0){  
    delay(100);
  }
  Serial.println("Wifi Connect Success");
  
  // MQTT configuration
  wifiModule.MQTTBegin(mqtt_server, mqtt_port, mqtt_client_id, mqtt_pwd, mqtt_device_id);

  // Subscribe to topic
  wifiModule.subscribe(mqtt_topic);

}

void setup1(){
  //Connect to SHT41
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  USBHost.configure_pio_usb(1, &pio_cfg);
  // if (!TinyUSBDevice.isInitialized()) {
  //   TinyUSBDevice.begin(0);
  // }
  USBHost.begin(1);
  USBSer1.begin(115200);
}

void loop() {

  if (!dataQueue.isEmpty()) {
    String inputData;
    char snd;
    while (dataQueue.pop(&snd)){
      inputData += snd;
    }

    //String inputData = String((char *)tempData);
    inputData.trim();
    Serial.println(inputData);
    wifiModule.publish(mqtt_topic, inputData);
  }

  if (!USBSer1) { // turn the neopixel red if there's no connection to SHT41
    pixel.Color(255, 0, 0);
    delay(freq);
  }
  else {
    rainbow(freq);
  }
}

void loop1(){
  // read the incoming byte:
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  USBSer1.task();
  #endif

  char buf[80];
  uint32_t count = 0;
  while (USBSer1.available()) {
    buf[count++] = (char) USBSer1.read();
  }

  if (count){
    dataQueue.push(&buf);
  }
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
