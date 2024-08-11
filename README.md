# Temperature sensor on Adafruit Feather RP2040 with USB port

Components:
- Adafruit Feather RP2040 with USB port
- Gravity: WiFi IoT module
- Adafruit SHT41 Trinkey: USB Temperature and Humidity Sensor

### Dependencies
- [CircuitPython Feather RP2040 with USB Type A Host](https://circuitpython.org/board/adafruit_feather_rp2040_usb_host/) v9.11
- [Adafruit CircuitPython ConnectionManager](https://github.com/adafruit/Adafruit_CircuitPython_ConnectionManager/tree/67d649b363cc3464b7959652a32cb62662ee60c3)
- [Adafruit CircuitPython MiniMQTT](https://github.com/adafruit/Adafruit_CircuitPython_MiniMQTT/tree/5f222c2065b434f72587ea970b8893ade5537898)
- [circup](https://github.com/adafruit/circup)

### Install
```
pip3 install circup

# Install ConnectionManager and load onto Feather
pip3 install adafruit-circuitpython-connectionmanager
circup install adafruit_connection_manager

# Install MiniMQTT package and load onto Feather
pip3 install adafruit-circuitpython-minimqtt
circup install adafruit_minimqtt
```