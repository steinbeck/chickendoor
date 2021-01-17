# Chicken Door
## Introduction
This is code for an ESP8266 or ESP32 as well as ESPHome-based automatic chicken door.
The ESP is hooked up to a BME280 climate sensor (not needed, just for fun) as well as a BH1750 light sensor, as well as a DC motordriver connected to a 30 cm DC accuator to open and close the door. 
The door is supposed to open in the morning after a certain lux value has been reached for a certain amount of time and close in the evening when it has been darker than xyz lux for some time. 

