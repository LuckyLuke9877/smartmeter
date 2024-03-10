# Overview

ESP32 SmartMeter that reads data from Kaifa MA309 and converts it to Sunspec data model on Modbus server.
Using Esphome framework providing web-ui and OTA-update. 

# Credits
This is an extension of [this project](https://github.com/DomiStyle/esphome-dlms-meter?tab=readme-ov-file).
Thanks a lot for the good work.

# Tested with
- Fronius Gen24 inverter: config => "Kommunikation/Modbus RTU" as "Master"; add "Komponente/Primärzähler"
- Kaifa MA309 on VKW ( Vorarlberg Netz ). Note: Needs security key from energy provider.

# HW
- [esp32 Lilygo RS485 / CAN](https://github.com/Xinyuan-LilyGO/T-CAN485/tree/main). CAN-bus is not needed.
- [M-Bus board](https://www.mikroe.com/m-bus-slave-click)
- wiring is similar to "credits" project, but see m-bus definitions in smart_meter.yaml for Lilygo board
- Fronius wiring: Modbus RTU shield, +, - to esp.RS485 A, B

# Description
- Kaifa broadcasts data in ~5sec interval
- receive data via M-Bus and convert them to Sunspec data model
- provide data on Modbus RTU - server
- Fronius inverter reads data in ~1sec interval
- if everything works correct, esp.led blinks green

# Build SW
- read the "SW installation" in the "credits" - project ( add your security key )
- for USB connection run (use your ttyUSB): "esphome run ./smart_meter.yaml --device /dev/ttyUSBx"
- for Wifi connection run (use your local address): "esphome run ./smart_meter.yaml --device 192.168.xxx.xxx"
- optional build / run the tests in "test" folder
  - smart_meter_client: used to test the Modbus - server communication with an other Lilygo board running a Modbus - client
  - GTest: for Sunspec model and Modbus - server
