# Overview

ESP32 SmartMeter that reads data from Kaifa MA309 and converts it to Sunspec data model on Modbus server.
Using Esphome framework providing web-ui and OTA-update. 

# Credits
This is an extension of [the project](https://github.com/DomiStyle/esphome-dlms-meter?tab=readme-ov-file).
Thanks a lot for the good work.

# HW
- [esp32 Lilygo RS485 / CAN](https://github.com/Xinyuan-LilyGO/T-CAN485/tree/main). CAN-bus is not needed.
- [M-Bus board](https://www.mikroe.com/m-bus-slave-click)
- wiring is similar to "credits" project, but see m-bus definitions in smart_meter.yaml
- Fronius wiring: Modbus RTU shield, +, - to esp.RS485

# Tested with
- Fronius Gen24 inverter: config => "Kommunikation/Modbus RTU" as "Master"; add "Komponente/Primärzähler"
- Kaifa MA309 on VKW ( Vorarlberg Netz ). Needs security key from energy provider.

# Description
- Kaifa broadcasts data in ~5sec interval
- receive data via M-Bus and convert them to Sunspec data model
- provide data on Modbus RTU - server
- Fronius inverter reads data in ~1sec interval
- if everything works correct, esp.led blinks green
