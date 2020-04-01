# ADS122C05
Micropython Library For the Texas Instruments ADS122C04 24bit ADC.

Tested on the ESP32 but should work for esp8266 and other versions of mucropytion/ circuit python with
slight modifications.

## General Usage ##
** Initial setup **
```Python
from machine import Pin, I2C
from ADS122C04_ESP import ADC
```
** Next, you need to create and I2C object **
```Python
# 5 and 17 are just used as an example, most pin compinations could be used for an i2c bus
i2c = I2C(scl=Pin(5), sda=Pin(17))
```



to create an adc object, you need to pass it four things:
1: an I2C object
*2: an debvice address
