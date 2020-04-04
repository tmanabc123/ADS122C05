# ADS122C05
Micropython Library For the Texas Instruments ADS122C04 24bit ADC.\
(most main functions complete)\
Tested and working on the ESP32 but should work for esp8266 and other versions of mucropytion/ circuit python with
slight modifications.
## Hardware Setup ##
ADS122C04 pinout\
![Image of pinout]
(https://github.com/tmanabc123/ADS122C05/adc_pinout.png)

## Software Setup ##
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
Used for setting a single channel as a positive input with AVSS(gnd) as the negative input\
```Python
# Channel can be 0 - 3
channel = 3
adc.SET_SINGLE(channel)
```
**SET_DIFFERENTIAL(AINp,AINn)**\
Used when you want to measure the voltage between two of the pins\
```Python
# Set the negative and positive inputs (can be anything 0 - 3)
neg = 0
pos = 1
adc.SET_DIFFERENTIAL(pos, neg)
```
**SYSTEM_MONITOR(state)**\
Used for testing purposes
```Python
# Set ADC input as [(AVDD-AVSS)/4]
adc.SYSTEM_MONITOR(0)
# Set ADC input as [(AVDD-AVSS)/4]
adc.SYSTEM_MONITOR(1)
# AINp and AINn shorted to (AVDD+AVSS)/2
adc.SYSTEM_MONITOR(2)
```
**PGA(gain)**\
Sets the gain. If nothing is entered, the current gain is returned
```Python
# Gain from 1-128 can be entered. When the input is in single channel mode with negative set to
 AVSS(agnd), or when the PGA(programmable gain amplifier) is disabled, the gain must be 1, 2, or 4.
gain = 4

# Set the gain to 4x
adc.PGA(4)

# To return the current gain:
adc.PGA()
```
**PGA_ENABLED(state)**\
Enable or Disable the PGA
```Python
# Can be set to True or False. If nothing is entered, the current state of the pga is returned
pga_status = True
adc.PGA_ENABLED(pga_status)

# To return the current status:
adc.PGA_ENABLED()
```
**OPERATING_MODE(mode)**\
Sets the operating mode to normal or turbo. In normal mode, the max smaple rate is 1000 Samples per second and the modulator clock is 256-kHz. In turbo mode, the max sample rate is 2000 SPS and the modulator clock is 512-kHz. Turbo mode provides more accurate readings. If nothing is entered, the current mode is returned
```Python
# ADC can be set to turbo or normal
mode = normal
adc.OPERATING_MODE(normal)

# To return the current operating mode:
adc.OPERATING_MODE()
```
**DATA_RATE(rate)**\
Sets the data rate. Must be 20,45,90,175,330,600, or 1000 for normal mode or 40,90,180,350,660,1200 or 2000 for turbo.
If nothing is entered, the current data rate is returned.
The slower the data rate, the more accurate the rading will be.
```Python
# Set the data rate to 40 sps
rate = 40
adc.DATA_RATE(40)

# To return the current data rate:
adc.DATA_RATE()
```
**CONVERSION_MODE(mode)**\
Used to set the conversion mode to single or continuous. If nothing is entered, the current mode is returned
```Python
adc.
```
**VOLTAGE_REF(ref)**\
```Python

```
**TEMP_MODE(state)**\
```Python

```
**check_conversion(state)**\
```Python

```
**read()**\
```Python

```
**start_conversion()**\
```Python

```
**reset()**\
```Python

```
**powerdown()**
```Python

```

## Functions in progress:
**read_temp()**\
**enable_CRC16()**\
**EFFECTIVE_RES()**\
**NOISE_FREE_RES()**

## Examples
**Setting up adc to read positive voltage from channel 1 and negative from ground**
**DO NOT PUT MORE THAN 2.048V on the input!!!**
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
**reading the direct register values from the latest adc conversion**
```Python
 # Will return a value between 0 and 16777216
 adc.read()
```

**Interpreting results:**
The adc is really only 23 bits even though it advertises as being 24 bit. This is because the
24th bit is a sign bit.
If the adc reading is greater than 2^23 (8388608), the voltage is negative.
For anything lower thatn 2^23, the voltage is positive.
In either case, calculating the actual voltage from the register value is very straightfoward

**Converting the ADC reading to a voltage**\
Reminder: If you want to measure a negative voltage, Make sure to use a bipolar power supply (AVSS=-2.5v, AVDD=2.5v)
```Python
# A function for reading the actual voltage 
def read_voltage(n=0):
    # Take n readings and average them
    if n != 0:
        sum = 0
        i = 0
        while i < n:
            i = i + 1
            a = adc.read()
            neg_check = a >> 23
            b = a & 0b011111111111111111111111
            voltage = b / (2**23) * 2.048
            sum += voltage
        if neg_check == 1:
            return (sum / n) * (-1)
        else:
            return (sum / n)

    else:
        a = adc.read()
        neg_check = a >> 23
        b = a & 0b011111111111111111111111
        voltage = b / (2**23) * 2.048
        if neg_check == 1:
            return voltage * (-1)
        else:
            return voltage
```
**For voltages higher than the 2.048V reference, you need to use a resistor divider to drop the voltage at the ADC input.**
