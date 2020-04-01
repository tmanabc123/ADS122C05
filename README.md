# ADS122C05
Micropython Library For the Texas Instruments ADS122C04 24bit ADC.\
(most main functions complete)\
Tested and working on the ESP32 but should work for esp8266 and other versions of mucropytion/ circuit python with
slight modifications.

## Initial Setup ##
**Import needed libraries**
```Python
from machine import Pin, I2C
from ADS122C04_ESP import ADC
```
**Next, you need to create and I2C object and find your device's address.**
```Python
# 5 and 17 are just used as an example, most pin compinations could be used for an i2c bus
i2c_bus = I2C(scl=Pin(5), sda=Pin(17))
```
After setting up the i2c bus, assuming you have everything connected correctly, you can now find it's address:
```Python
# This will return a list containing the addresses of all currently connected devices.
i2c_bus.scan()
```

**Now to create the adc object**\
To create an adc object, you need to pass four things to the ADC class that was previously imported:\
1: an I2C object\
2: the device address\
3: the pin numer that you connected to the data ready pin on the ADC (can be any viable input pin w/ internal pullup)\
4: the reset pin (any viable output pin)

```Python
# Assuming an address of 69, drdy of 16, and reset of 4:
adc = ADC(i2c_bus, 69, 16, 4)
```

## Currently Working Functions:
**SET_SINGLE(channel)**\
**SET_DIFFERENTIAL(AINp,AINn)**\
**SYSTEM_MONITOR(state)**\
**PGA(gain)**\
**PGA_ENABLED(state)**\
**DATA_RATE(rate)**\
**OPERATING_MODE(mode)**\
**CONVERSION_MODE(mode)**\
**VOLTAGE_REF(ref)**\
**TEMP_MODE(state)**\
**check_conversion(state)**\
**read()**\
**start_conversion()**\
**reset()**\
**powerdown()**

## Functions in progress:
**read_temp()**\
**enable_CRC16()**\
**EFFECTIVE_RES()**\
**NOISE_FREE_RES()**

## Examples
**Setting up adc to read positive voltage from channel 1 and negative from ground**
```Python
from machine import Pin, I2C
from ADS122C04_ESP import ADC

# make I2C object 
port  = I2C(scl=-Pin(5), sda=Pin(17))

# After determining the address and selecting the pins as described earlier, 
# Create ADC object
adc = ADC(port, 69, 16, 4)

# Set conversion mode to continuous
adc.CONVERSION_MODE('continuous')

# Set the voltage reference to the internal 2.048V reference
adc.VOLTAGE_REF('internal')

# Set the positive input to channel 0 and negative to ground
adc.SET_SINGLE(0)

# Set operating mode to turbo (increased accuract at cost of more power)
adc.OPERATING_MODE('turbo')

# Set data rate to 40 samples per second (slowest and most accurate)
adc.DATA_RATE(40)

# Send initial start conversion command (only needs to be sent once when in \
continuous conversion mode)
adc.start_conversion()

```
* to read the direct values from the adc:

<div class="text-red mb-2">
  .text-red on white
</div>
