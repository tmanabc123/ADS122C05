# ADS122C05
Micropython Library For the Texas Instruments ADS122C04 24bit ADC.

Tested on the ESP32 but should work for esp8266 and other versions of mucropytion/ circuit python with
slight modifications.

## General Usage ##
**Initial setup**
```Python
from machine import Pin, I2C
from ADS122C04_ESP import ADC
```
**Next, you need to create and I2C object and find your device's address.**
```Python
# 5 and 17 are just used as an example, most pin compinations could be used for an i2c bus
i2c_bus = I2C(scl=Pin(5), sda=Pin(17))
```
After setting up the i2c bus, assuming you have everything connected correctly, you can 
now find it's address by doing:
```Python
# This will return a list containing the addresses of all currently connected devices.
i2c_bus.scan()
```

**Now to create the adc object**
to create an adc object, you need to pass four things to the ADC class that was previously imported:
1: an I2C object
2: the device address
3: the pin numer that you connected to the data ready pin on the ADC (can be any viable input pin w/ internal pullup)
4: the reset pin (any viable output pin)

```Python
# Assuming an address of 69, drdy of 16, and reset of 4:
adc = ADC(i2c_bus, 69, 16, 4)
```
