from machine import I2C, Pin
import math
class CONST:
    # Command Bytes

    # This command resets the device to the default states. No delay 
    # time is required after the RESET command is latched before starting 
    # to communicate with the device as long as the timing requirements 
    # (see the I2C Timing Requirements table) for the (repeated) START 
    # and STOP conditions are met.
    # can technically be 0b0000011x where x can be either
    _RESET = 0b00000110
    

    # In single-shot conversion mode, the START/SYNC command is used to 
    # start a single conversion, or (when sent during an ongoing conversion) 
    # to reset the digital filter and then restart a single new conversion. 
    # When the device is set to continuous conversion mode, the START/SYNC 
    # command must be issued one time to start converting continuously. 
    # Sending the START/SYNC command when converting in continuous conversion 
    # mode resets the digital filter and restarts continuous conversions.
    # can technically be 0b00001xxx where x can be either
    _START_SYNC = 0b00001000
    

    # The POWERDOWN command places the device into power-down mode. This 
    # command shuts down all internal analog components and turns off both I
    # DACs, but holds all register values. In case the POWERDOWN command is 
    # issued when a conversion is ongoing, the conversion completes before 
    # the ADS122C04 enters power-down mode. As soon as a START/SYNC command 
    # is issued, all analog components return to their previous states.
    # can technically be 0b0000001x where x can be either
    _PWRDWN = 0b00000010
    


    # The RDATA command loads the output shift register with the most recent 
    # conversion result. Reading conversion data must be performed as shown in 
    # Figure 57 by using two I2C communication frames. The first frame is an I2C 
    # write operation where the R/W bit at the end of the address byte is 0 to 
    # indicate a write. In this frame, the host sends the RDATA command to the 
    # ADS122C04. The second frame is an I2C read operation where the R/W bit at
    #  the end of the address byte is 1 to indicate a read. The ADS122C04 
    #  reports the latest ADC conversion data in this second I2C frame. If a 
    #  conversion finishes in the middle of the RDATA command byte, the state 
    #  of the DRDY pin at the end of the read operation signals whether the 
    #  old or the new result is loaded. If the old result is loaded, DRDY stays 
    #  low, indicating that the new result is not read out. The new conversion 
    #  result loads when DRDY is high.
    # can technically be 0b0001xxxx where x can be either
    _RDATA = 0b00010000 
    

    
    # RREG (0010 rrxx):
    # The RREG command reads the value of the register at address rr. Reading a register 
    # must be performed as shown in Figure 58 by using two I2C communication frames. The 
    # first frame is an I2C write operation where the R/W bit at the end of the address 
    # byte is 0 to dicate a write. In this frame, the host sends the RREG command including 
    # the register address to the ADS122C04. The second frame is an I2C read operation 
    # where the R/W bit at the end of the address byte is 1 to indicate a read. 
    # The ADS122C04 reports the contents of the requested
    # register in this second I2C frame.
    _RREG_OFFSET = 0b0010
    
    # WREG (0100 rrxx dddd dddd)
    # The WREG command writes dddd dddd to the register at address rr. Multiple 
    # registers can be written within the same I2C frame by simply issuing another 
    # WREG command without providing a STOP condition following the previous register 
    # write. Figure 59 shows the sequence for writing an arbitrary number of registers. 
    # The R/W bit at the end of the address byte is 0 to indicate a write. The WREG 
    # command forces the digital filter to reset and any ongoing ADC conversion to restart.
    _WREG_OFFSET = 0b0100

    # list of all commands
    _LIST = [_RESET, _START_SYNC, _PWRDWN, _RDATA]

    # List of valid gains
    _GAIN = [1,2,4,8,16,32,64,128]

    # List of valid data rates
    _NORMAL_RATES = [20,45,90,175,330,600,1000]
    _TURBO_RATES = [40,90,180,350,660,1200,2000]

    
    # list 1 = SPS 20,45,90,175,330,600,1000
    # list 2 = Gain 1,2,4,8,16,32,64,128
    # list 2 = uVrms or uVpp
    # Noise in μVRMS (μVPP) at AVDD = 3.3 V, AVSS = 0 V, Normal Mode, PGA Enabled, and Internal VREF = 2.048 V
    _NORMAL_NOISE = [[[5.10, 21.69], [2.49, 10.71], [1.25, 5.74], [0.64, 2.92], [0.41, 1.52], [0.24, 0.98], [0.14, 0.54], [0.11, 0.46]],
    [[6.53, 29.99], [3.02, 14.47], [1.67, 6.80], [0.93, 4.00], [0.52, 2.43], [0.28, 1.39], [0.17, 0.71], [0.13, 0.57]],
    [[9.01, 41.61], [4.67, 24.36], [2.41, 10.95], [1.24, 6.54], [0.73, 3.46], [0.41, 2.06], [0.25, 1.20], [0.19, 0.91]],
    [[12.78, 63.79], [6.75, 37.30], [3.26, 17.00], [1.92, 9.81], [1.02, 5.27], [0.60, 3.32], [0.35, 1.93], [0.25, 1.49]],
    [[17.75, 107.88], [8.75, 48.95], [4.72, 28.25], [2.62, 14.47], [1.42, 8.06], [0.85, 4.64], [0.50, 2.93], [0.37, 1.91]],
    [[24.73, 153.77], [12.89, 76.01], [6.81, 38.94], [3.84, 22.30], [2.02, 12.07], [1.18, 6.69], [0.70, 4.49], [0.51, 3.14]],
    [[36.90, 228.90], [18.07, 108.90], [9.48, 58.24], [5.49, 31.55], [2.86, 17.41], [1.65, 10.23], [1.04, 6.21], [0.73, 4.69]]]

    # Noise in μVRMS (μVPP) at AVDD = 3.3 V, AVSS = 0 V, Normal Mode, PGA Disabled, and Internal VREF = 2.048 V
    _NORMAL_NOISE_PGA_DISABLED = [[[5.04, 19.71], [2.53, 10.06], [1.57, 5.68]],
    [[6.57, 33.34], [3.43, 14.00], [1.60, 6.98]],
    [[8.75, 42.59], [4.35, 22.83], [2.13, 10.52]],
    [[12.64, 65.71], [6.27, 35.00], [3.40, 16.83]],
    [[18.58, 106.06], [9.33, 52.59], [4.54, 26.30]],
    [[25.74, 150.81], [12.57, 79.15], [6.47, 36.87]],
    [[36.98, 221.61], [18.67, 111.61], [9.27, 55.07]]]

    # Noise in μVRMS (μVPP) at AVDD = 3.3 V, AVSS = 0 V, Turbo Mode, PGA Enabled, and Internal VREF = 2.048 V
    _TURBO_NOISE = [[[4.41, 19.43], [2.25, 10.62], [1.12, 5.32], [0.63, 2.74], [0.36, 1.64], [0.22, 1.10], [0.13, 0.63], [0.10, 0.51]],
    [[5.76, 30.73], [2.98, 14.16], [1.62, 7.84], [0.92, 4.43], [0.52, 2.59], [0.31, 1.59], [0.18, 0.97], [0.15, 0.76]],
    [[8.49, 44.61], [4.48, 22.25], [2.29, 13.23], [1.34, 6.83], [0.71, 4.11], [0.43, 2.49], [0.28, 1.51], [0.22, 1.05]],
    [[12.77, 71.04], [6.33, 37.00], [3.33, 19.17], [1.89, 10.76], [1.04, 5.91], [0.61, 3.54], [0.41, 2.13], [0.29, 1.64]],
    [[17.10, 105.64], [9.04, 54.97], [4.51, 27.74], [2.84, 16.98], [1.42, 8.45], [0.86, 5.07], [0.57, 3.32], [0.41, 2.38]],
    [[25.26, 153.74], [12.51, 78.75], [6.58, 39.68], [3.90, 23.84], [2.11, 13.19], [1.23, 7.46], [0.81, 5.17], [0.58, 3.50]],
    [[35.35, 226.39], [17.82, 112.98], [9.40, 59.37], [5.37, 32.97], [3.02, 18.73], [1.76, 11.12], [1.12, 7.06], [0.83, 5.41]]]

    # Noise in μVRMS (μVPP) at AVDD = 3.3 V, AVSS = 0 V, Turbo Mode, PGA Disabled, and Internal VREF = 2.048 V
    _TURBO_NOISE_PGA_DISABLED = [[[4.30, 18.73], [2.18, 9.84], [1.10, 5.38]],
    [[6.19, 32.78], [3.14, 13.53], [1.42, 7.19]],
    [[9.08, 47.57], [4.49, 25.48], [2.18, 10.96]],
    [[12.40, 72.79], [5.89, 33.34], [3.07, 18.31]],
    [[17.59, 103.97], [9.05, 51.15], [4.39, 24.69]],
    [[24.67, 149.07], [12.56, 76.35], [6.31, 37.48]],
    [[34.54, 224.19], [17.76, 113.98], [8.85, 56.87]]]

class ADC():
    """Pass ADC an i2c object to init ADC"""
    def __init__(self, i2c, addr, drdy, reset):
        # device address
        # 64 if A0 and A1 pulled to GND/DVSS
        self.addr = addr

        # scl=5 sda=17 on esp32 w/sma 
        self.i2c = i2c

        # define data ready pin (active low)
        # pin 16 on esp32 w/sma
        self.DRDY = Pin(drdy, Pin.IN, Pin.PULL_UP) 

        # define reset pin (active low)
        # pin 4 on esp32 w/sma
        self.RESET = Pin(reset, Pin.OUT)
        # activate chip
        self.RESET.value(1)

        # define buffer for storing data to be writen
        self.write_buffer = bytearray(1)

        # define buffer for storing read register values
        self.reg_buffer = bytearray(1)

        # define buffer for reading the conversion result
        self.conversion_buffer = bytearray(3)

        # definr register update buffer
        self.reg_update_buffer = bytearray(1)

        # b/c PGA must be bypassed while AINn = AVSS:
        self.pga_bypass = False

        # stored register values to keep up with data
        self.reg0 = 0x00
        self.reg1 = 0x00
        self.reg2 = 0x00
        self.reg3 = 0x00

        # send reset command to make sure device is properly reset after power-up
        self.SEND_COMMAND(CONST._RESET)

        



    # function for sending single commands
    def SEND_COMMAND(self, command):
        if command not in CONST._LIST:
            raise ValueError("not a valid command")
        # send start condition
        self.i2c.start()
        # buffer data to send (address and write bit)
        self.write_buffer[0] = self.addr << 1 & 0xFE
        # send the data
        self.i2c.write(self.write_buffer)
        # buffer data to send (command)
        self.write_buffer[0] = command
        # send the data
        self.i2c.write(self.write_buffer)
        # send stop condition
        self.i2c.stop()

    # function for writing to control register
    def REG_WRITE(self, reg, dat):
        if reg > 3:
            raise ValueError("register must be in range (0,4)")
        # send start condition
        self.i2c.start()
        # buffer data to send (address and write bit)
        self.write_buffer[0] = self.addr << 1 & 0xFE
        # send the data
        self.i2c.write(self.write_buffer)
        # buffer data to send (WREG command plus the selected register)
        self.write_buffer[0] = CONST._WREG_OFFSET << 4
        temp = reg << 2
        self.write_buffer[0] = self.write_buffer[0] + temp
        # send the data
        self.i2c.write(self.write_buffer)
        # buffer data to send (selected dat)
        self.write_buffer[0] = dat
        # send the data
        self.i2c.write(self.write_buffer)
        # send stop condition
        self.i2c.stop()



    # function for reading from control register
    def REG_READ(self, reg):
        if reg > 3:
            raise ValueError("register must be in range (0,4)")
        # send start condition
        self.i2c.start()
        # buffer data to send (address plus write bit)
        self.write_buffer[0] = self.addr << 1 & 0xFE
        # send the data
        self.i2c.write(self.write_buffer)
        # buffer data to send (WREG command plus the selected register)
        self.write_buffer[0] = CONST._RREG_OFFSET << 4
        temp = reg << 2
        self.write_buffer[0] = self.write_buffer[0] + temp
        # send the data
        self.i2c.write(self.write_buffer)
        # send start again condition
        self.i2c.start()
        # buffer data to send (address plus read bit)
        self.write_buffer[0] = self.addr << 1 | 0x01
        # send the data
        self.i2c.write(self.write_buffer)
        # read one byte from selected register into reg_buffer 
        self.i2c.readinto(self.reg_buffer)
        # send stop condition
        self.i2c.stop()
        return self.reg_buffer


    def SET_SINGLE(self, channel):
        self.reg0 = self.REG_READ(0x00)[0]
        # The default if only a single channel number is in input
        # Ground is GND adn the measurement is single ended at the given channel.
        # Because AINN = AVSS, PGA will be bypassed and only gains 1,2, and 4 can be used
        if channel == 0:
            mux = 0b1000
        if channel == 1:
            mux = 0b1001
        if channel == 2:
            mux = 0b1010
        if channel == 3:
            mux = 0b1011
        # update gain and PGA
        self.PGA_ENABLED(False)
        self.PGA(1)

        # Update mux
        temp = self.reg0 & 0b00001111
        temp = temp | (mux << 4)
        self.REG_WRITE(0x00, temp)
        # update locally stored register values
        self.reg0 = temp
        

    def SET_DIFFERENTIAL(self, AINp, AINn):
        self.reg0 = self.REG_READ(0x00)[0]
            # Differential input settings:
        if AINp == 0 and AINn == 1:
            mux = 0b0000
        if AINp == 0 and AINn == 2:
            mux = 0b0001
        if AINp == 0 and AINn == 3:
            mux = 0b0010
        if AINp == 1 and AINn == 0:
            mux = 0b0011
        if AINp == 1 and AINn == 2:
            mux = 0b0100
        if AINp == 1 and AINn == 3:
            mux = 0b0101
        if AINp == 2 and AINn == 3:
            mux = 0b0110
        if AINp == 3 and AINn == 2:
            mux = 0b0111

        # Update mux
        temp = self.reg0 & 0b00001111
        temp = temp | (mux << 4)
        self.REG_WRITE(0x00, temp)
        # update locally stored register values
        self.reg0 = temp


    def SYSTEM_MONITOR(self, System_monitor):
        self.reg0 = self.REG_READ(0x00)[0]
        # Used primarily for system monioring and error checking
        # Sets the currently selected voltage reference [(Vrefp-Vrefn)/4] as ADC input
        if System_monitor == 0:
            mux = 0b1100
        # Sets analog supply [(AVDD-AVSS)/4] as ADC input 
        if System_monitor == 1:
            mux = 0b1101
        # AINp and AINn shorted to (AVDD+AVSS)/2
        if System_monitor == 2:
            mux = 0b1110


        # Update mux
        temp = self.reg0 & 0b00001111
        temp = temp | (mux << 4)
        self.REG_WRITE(0x00, temp)
        # update locally stored register values
        self.reg0 = temp


    # Checks to see if AINn = AVSS for PGA programming uses
    def AVSS_CHECK(self):
        self.reg0 = self.REG_READ(0x00)[0]
        mux = self.reg0 >> 4
        if mux in {0b1000, 0b1001, 0b1010, 0b1011}:
            return True

            
    # Sets the programable gain amplifier. if nothing is entered, the current gain
    # is returned

    # Disable PGA if option selected
    # Allows absolute input voltage range to span from AVSS-0.1v to AVDD+0.1V
    # The pga can obly be disabled for gains 1,2, and 4
    # The pga is enabled for gains of 8 to 128 regardless of the bypass setting
    def PGA(self, gain=999):
        # Return the current gain if nothing is entered
        self.reg0 = self.REG_READ(0x00)[0]
        if gain == 999:
            cg = self.reg0
            cg = cg >> 1 
            cg = cg & 0b0000111
            cg = 2 ** cg
            return cg
        # Set the gain to whatever is entered
        else:
            if gain not in CONST._GAIN:
                raise ValueError("Gain must be 1,2,4,8,16,32,64, or 128")
            else:
                # make sure gain isnt higher than 4 if AINn = VSS
                if self.AVSS_CHECK() == 1 or self.PGA_ENABLED() == False:
                    if gain > 4:
                        raise ValueError("Gain must be 1,2, or 4 when AINn = AVSS or when PGA is disabled")
                    else:
                        temp = self.reg0
                        temp = temp & 0xF1
                        g = int(math.log(gain,2)) << 1
                        temp = temp | g
                        self.REG_WRITE(0x00, temp)
                        # update locally stored register values
                        self.reg0 = temp
                        print("----Gain updated to %s----" % gain)
                else:
                    temp = self.reg0
                    temp = temp & 0xF1
                    g = int(math.log(gain,2)) << 1
                    temp = temp | g
                    self.REG_WRITE(0x00, temp)
                    # update locally stored register values
                    self.reg0 = temp
                    print("----Gain updated to %s----" % gain)

    
    # Checks to see if the pga is enabled or disabled
    # 1 = enabled, 0 = disabled
    # enables or disables based on input
    def PGA_ENABLED(self, value=999):
        self.reg0 = self.REG_READ(0x00)[0]
        if value == 999:
            pga_status = self.reg0 & 0b0000001
            if pga_status == 0:
                return 1
            else:
                return 0
        # Disable the PGA
        elif value == False:
            # if gain is higher than 4 when disabling pga, change it to 4
            if self.PGA() > 4:
                self.PGA(4)
                print("----Changing Gain to 4----")
            temp = self.reg0 & 0b11111110
            temp = temp | 0b00000001
            self.REG_WRITE(0x00, temp)
            # update locally stored register values
            self.reg0 = temp
            print("----PGA disabled----")
        # Enable the PGA
        elif value == True:
            # abort turning on if AINn = AVSS
            if self.AVSS_CHECK() == True:
                raise SystemError("Can't enable PGA when AINn = AVSS. Change MUX settings to enable PGA")
            else:
                temp = self.reg0 & 0b11111110
                self.REG_WRITE(0x00, temp)
                # update locally stored register values
                self.reg0 = temp
                print("----PGA enabled----")


        else:
            print("Unknown value, must be True or False")


    # Sets and checks the data rate
    # Changes depending on operating mode
    def DATA_RATE(self, rate=999):
        # Dictionaries for making things easier 
        reg_rates = {20:0b000, 45:0b001, 90:0b010, 175:0b011, 330:0b100, 600:0b101, 1000:110}
        reg_rates_read = {0:20, 1:45, 2:90, 3:175, 4:330, 5:600, 6:1000}
        turbo_rates = {40:0b000, 90:0b001, 180:0b010, 350:0b011, 660:0b100, 1200:0b101, 2000:110}
        turbo_rates_read = {0:40, 1:90, 2:180, 3:350, 4:660, 5:1200, 6:2000}

        # Read reg 1 and store it in reg1
        self.reg1 = self.REG_READ(0x01)[0]
        current_rate = self.reg1 >> 5
        op_mode = (self.reg1 >> 4) & 0b0001

        # return current rate
        if rate == 999:
            if op_mode == 0:
                return reg_rates_read[current_rate]
            else:
                return turbo_rates_read[current_rate]
        # set new rate
        else:
            # for normal mode:
            if op_mode == 0:
                if rate not in CONST._NORMAL_RATES:
                    raise ValueError("Data rate must be 20,45,90,175,330,600, or 1000")
                else:
                    temp = self.reg1 & 0b00011111
                    temp = temp | (reg_rates[rate] << 5)
                    self.REG_WRITE(0x01, temp)
                    # update locally stored register values
                    self.reg1 = temp
                    print("----Rate updated to %s SPS----" % rate)
            # for turbo mode:
            else:
                if rate not in CONST._TURBO_RATES:
                    raise ValueError("Data rate bust be 40,90,180,350,660,1200 or 2000")
                else:
                    temp = self.reg1 & 0b00011111
                    temp = temp | (turbo_rates[rate] << 5)
                    self.REG_WRITE(0x01, temp)
                    # update locally stored register values
                    self.reg1 = temp
                    print("----Rate updated to %s SPS----" % rate)


    # Read and set the operating mode (normal or turbo)
    def OPERATING_MODE(self, mode=999):
        self.reg1 = self.REG_READ(0x01)[0]
        # Read back current mode
        if mode == 999:
            current_mode = (self.reg1 >> 4) & 0b0001
            if current_mode == 0:
                return 'normal'
            if current_mode == 1:
                return 'turbo'
        # Set mode to normal
        elif mode == 'normal':
            temp = self.reg1 & 0b11101111
            self.REG_WRITE(0x01, temp)
            # update locally stored register values
            self.reg1 = temp
        # Set mode to turbo
        elif mode == 'turbo':
            temp = self.reg1 | 0b00010000
            self.REG_WRITE(0x01, temp)
            # update locally stored register values
            self.reg1 = temp
        else:
            raise ValueError("Invalid mode, must be 'normal' or 'turbo'")


    # Set conversion mode (single or continuous)
    # If no input is given, read back the current conversion mode
    def CONVERSION_MODE(self, mode=999):
        self.reg1 = self.REG_READ(0x01)[0]
        # Read back current mode
        if mode == 999:
            current_mode = (self.reg1 >> 3) & 0b00001
            if current_mode == 0:
                return 'single'
            if current_mode == 1:
                return 'continuous'
        # Set mode to single-shot
        elif mode == 'single':
            temp = self.reg1 & 0b11110111
            self.REG_WRITE(0x01, temp)
            # update locally stored register values
            self.reg1 = temp
        # Set mode to continuous
        elif mode == 'continuous':
            temp = self.reg1 | 0b00001000
            self.REG_WRITE(0x01, temp)
            # update locally stored register values
            self.reg1 = temp
        else:
            raise ValueError("Invalid mode, must be 'single' or 'continuous'")
        
    # Select voltage reference or read back currently selected one
    def VOLTAGE_REF(self, ref=999):
        self.reg1 = self.REG_READ(0x01)[0]
        # Dictionary for reference types to clean things up a bit
        ref_list = {0:'internal', 1:'external', 2:'analog_supply', 3:'analog_supply'}
        # Read back current selected v_ref
        if ref == 999:
            current_ref = (self.reg1 >> 1) & 0b0000011
            return ref_list[current_ref]
        # Internal 2.048v ref
        elif ref == 'internal':
            temp = self.reg1 & 0b11111001
            self.REG_WRITE(0x01, temp)
            # update locally stored register values
            self.reg1 = temp
        # External ref using the REFP and REFN inputs
        elif ref == 'external':
            temp = self.reg1 & 0b11111001
            temp = temp | 0b00000010
            self.REG_WRITE(0x01, temp)
            # update locally stored register values
            self.reg1 = temp
        # Analog supply (AVDD - AVSS) used as reference
        elif ref == 'analog':
            temp = self.reg1 & 0b11111001
            temp = temp | 0b00000100
            # update locally stored register values
            self.reg1 = temp
        else:
            raise ValueError("Invalid conversion mode, must be 'internal', 'continuous', or 'analog_supply'")

    # Enable / Disable temperature sensing mode and read back current mode
    # When enabled, register 0 has no effect, and the internal ref. is used
    def TEMP_MODE(self, mode=999):
        self.reg1 = self.REG_READ(0x01)[0]
        # Read back current state
        if mode == 999:
            current_state = self.reg1 & 0b00000001
            return current_state
        # Enable temp mode
        elif mode == True:
            temp = self.reg1 | 0b00000001
            self.REG_WRITE(0x01, temp)
            # update locally stored register values
            self.reg1 = temp
        elif mode == False:
            temp = self.reg1 & 0b11111110
            self.REG_WRITE(0x01, temp)
            # update locally stored register values
            self.reg1 = temp
        else:
            raise ValueError("Invalid state, must be True or False")


    # Check the DRDY register and return 1 if there is new data or
    # 0 if there isn't
    def check_conversion(self):
        dat = self.REG_READ(2)
        dat = dat & 0b10000000
        if dat == 128:
            return 1
        else:
            return 0


    def enable_CRC16(self):
        # get existing value for reg 3
        self.reg_update_buffer[0] = self.self.REG_READ(0x02)[0]
        # modifiy the existing reg value to keep other settings but enable CRC16
        self.reg_update_buffer[0] = self.reg_update_buffer[0] | 0b00100000
        self.reg_update_buffer[0] = self.reg_update_buffer[0] & 0b11101111
        # write the modified date to the register
        self.REG_WRITE(0x02, )



        

    # read the 24 bit result from last conversion
    # bleow is the same as reading three bytes from register 10
    # 
    def read(self):
        # Clear the conversion buffer from the previous data
        self.conversion_buffer = bytearray(3)
        # send start condition
        self.i2c.start()
        # buffer data to send (address plus write bit)
        self.write_buffer[0] = self.addr << 1 & 0xFE
        # send the data
        self.i2c.write(self.write_buffer)
        # buffer data to send (WREG command plus the selected register)
        self.write_buffer[0] = CONST._RDATA
        # send the data
        self.i2c.write(self.write_buffer)
        # send start again condition
        self.i2c.start()
        # buffer data to send (address plus read bit)
        self.write_buffer[0] = self.addr << 1 | 0x01
        # send the data
        self.i2c.write(self.write_buffer)
        # read one byte from selected register into reg_buffer 
        self.i2c.readinto(self.conversion_buffer)

        self.conversion_buffer = self.i2c.readfrom_mem(self.addr, CONST._RDATA, 3)
        return self.conversion_buffer[0] << 16 | self.conversion_buffer[1] << 8 | self.conversion_buffer[2]

    # Return the temperature
    def read_temp(self):
        # The temp data only inculudes the 14MSBs of the output registers
        # Each count is 0.03125°C
        # check for temp mode
        
        temp = (self.read() >> 10) * 0.03125
        return temp


    # in single-shot:
    #       starts a single conversion or if sent during a conversion,
    #       resets the digital filter and starts a new conversion
    # in continuous mode:
    #       must be sent at beginning to start conversions.
    def start_conversion(self):
        self.SEND_COMMAND(CONST._START_SYNC)

    # resets the device to default states (all regisdters to 0x00)
    def reset(self):
        self.SEND_COMMAND(CONST._RESET)

    # shuts down all the analog components and turns off both IDACs, but holds all 
    # the register values. If powerdown is issued during conversion, the conversion complets
    # first. components return upon Start/Sync command.
    def powerdown(self):
        self.SEND_COMMAND(CONST._PWRDWN)

    
    # Calculate effective resolution using the rms noise and eqn 1: ln [2 · VREF / (Gain · VRMS-Noise)] / ln(2)
    # asumes internal vref unless specified otherwise (for external vref)
    def EFFECTIVE_RES(self ,vref=-1):
        # check operating mode
        current_op_mode = self.OPERATING_MODE()

        # check sample rate and translate to list location using the dictionary
        normal_sps_translate = {20: 0, 45: 1, 90: 2, 175: 3, 330: 4, 600: 5, 1000: 6}
        turbo_sps_translate = {40: 0, 90: 1, 180: 2, 350: 3, 660: 4, 1200: 5, 2000: 6}
        current_sps = self.DATA_RATE()
        if current_op_mode == 'turbo':
            translated_sps = turbo_sps_translate[current_sps]
        if current_op_mode == 'normal':
            translated_sps = normal_sps_translate[current_sps]
        

        # check gain and translate it to list loation
        current_gn = self.PGA()
        translated_gain = int(math.log(current_gn) / math.log(2))

        # check pga status
        current_pga_status = self.PGA_ENABLED()
        
        # Set correct reference voltage for calculations
        # Check vref or use manual if provided
        if vref != -1:
            current_ref_voltage = vref
        else:
            current_ref = self.VOLTAGE_REF()
            if current_ref == 'internal':
                current_ref_voltage = 2.048
            elif current_ref == 'analog_supply':
                current_ref_voltage = 3.3

        # Select appropriate noise for current states
        # If the pga is disabled
        if current_pga_status == 0:
            # If turbo mode is enabled
            if current_op_mode == 'turbo':
                noise = CONST._TURBO_NOISE_PGA_DISABLED[translated_sps][translated_gain][0] * (10 ** (-6))
            # If normal mode is enabled
            else:
                noise = CONST._NORMAL_NOISE_PGA_DISABLED[translated_sps][translated_gain][0] * (10 ** (-6))
        # If the pga is enabled
        else:
            # If turbo mode is enabled
            if current_op_mode == 'turbo':
                noise = CONST._TURBO_NOISE[translated_sps][translated_gain][0]
            # If normal mode is enabled
            else:
                noise = CONST._NORMAL_NOISE[translated_sps][translated_gain][0]

        # Calculate and return the current effective resolution (uV RMS)
        res = math.log((2 * current_ref_voltage) / (current_gn * noise)) / math.log(2)
        return res


    # Calculate noise free resolution using the peak to peak noise and eqn 2: ln [2 · VREF / (Gain · VPP-Noise)] / ln(2)
    # asumes internal vref unless specified otherwise
    def NOISE_FREE_RES(SPS, vref=2.048):
        # check operating mode
        current_op_mode = self.OPERATING_MODE()

        # check sample rate and translate to list location using the dictionary
        normal_sps_translate = {20: 0, 45: 1, 90: 2, 175: 3, 330: 4, 600: 5, 1000: 6}
        turbo_sps_translate = {40: 0, 90: 1, 180: 2, 350: 3, 660: 4, 1200: 5, 2000: 6}
        current_sps = self.DATA_RATE()
        if current_op_mode == 'turbo':
            translated_sps = turbo_sps_translate[current_sps]
        if current_op_mode == 'normal':
            translated_sps = normal_sps_translate[current_sps]
        

        # check gain and translate it to list loation
        current_gn = self.PGA()
        translated_gain = int(math.log(current_gn) / math.log(2))

        # check pga status
        current_pga_status = self.PGA_ENABLED()
        
        # Set correct reference voltage for calculations
        # Check vref or use manual if provided
        if vref != -1:
            current_ref_voltage = vref
        else:
            current_ref = self.VOLTAGE_REF()
            if current_ref == 'internal':
                current_ref_voltage = 2.048
            elif current_ref == 'analog_supply':
                current_ref_voltage = 3.3

        # Select appropriate noise for current states
        # If the pga is disabled
        if current_pga_status == 0:
            # If turbo mode is enabled
            if current_op_mode == 'turbo':
                noise = CONST._TURBO_NOISE_PGA_DISABLED[translated_sps][translated_gain][1] * (10 ** (-6))
            # If normal mode is enabled
            else:
                noise = CONST._NORMAL_NOISE_PGA_DISABLED[translated_sps][translated_gain][1] * (10 ** (-6))
        # If the pga is enabled
        else:
            # If turbo mode is enabled
            if current_op_mode == 'turbo':
                noise = CONST._TURBO_NOISE[translated_sps][translated_gain][1]
            # If normal mode is enabled
            else:
                noise = CONST._NORMAL_NOISE[translated_sps][translated_gain][1]

        # Calculate and return the current effective resolution (uV RMS)
        res = math.log((2 * current_ref_voltage) / (current_gn * noise)) / math.log(2)
        return res


    # Calculate noise free resolution using the peak to peak noise and eqn 2: ln [2 · VREF / (Gain · VPP-Noise)] / ln(2)
    # asumes internal vref unless specified otherwise





