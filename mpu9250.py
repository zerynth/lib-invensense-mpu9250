# Zerynth - libs - invensense-mpu9250/mpu9250.py
#
# Zerynth library for MPU9250 motion sensor
#
# @Author: Stefano Torneo
#
# @Date: 2020-09-21
# @Last Modified by: 
# @Last Modified time:

"""
.. module:: MPU9250

**************
MPU9250 Module
**************

.. _datasheet: https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf
               https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf

MPU-9250 features three 16-bit analog-to-digital converters (ADCs) for digitizing the gyroscope outputs, three
16-bit ADCs for digitizing the accelerometer outputs, and three 16-bit ADCs for digitizing the magnetometer
outputs. For precision tracking of both fast and slow motions, the parts feature a user-programmable
gyroscope full-scale range of ±250, ±500, ±1000, and ±2000°/sec (dps), a user-programmable accelerometer
full-scale range of ±2g, ±4g, ±8g, and ±16g, and a magnetometer full-scale range of ±4800µT.
Communication with all registers of the device is performed using I2C at 400kHz.
Additional features include an embedded temperature sensor.

"""

import i2c
import threading

GRAVITIY_MS2 = 9.80665

# two's complement
#
# @param      v      integer value to be converted
# @param      n_bit  number of bits of v's representation
#
# @return     the two's complement of v
#
def _tc(v, n_bit=16):
    mask = 2**(n_bit - 1)
    return -(v & mask) + (v & ~mask)

# Define some constants from the datasheet

# Scale Modifiers
ACCEL_SENSITIVITY_2G = 16384.0
ACCEL_SENSITIVITY_4G = 8192.0
ACCEL_SENSITIVITY_8G = 4096.0
ACCEL_SENSITIVITY_16G = 2048.0

GYRO_SENSITIVITY_250DPS = 131.0
GYRO_SENSITIVITY_500DPS = 65.5
GYRO_SENSITIVITY_1000DPS = 32.8
GYRO_SENSITIVITY_2000DPS = 16.4

# MPU-9250 Registers
MPU9250_ADDRESS = 0x68 # Device address when ADO = 0
MPU9250_WHOAMI = 0x71 # default value
MPU9250_WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

ACCEL_XOUT0 = 0x3B
ACCEL_XOUT1 = 0x3C
ACCEL_YOUT0 = 0x3D
ACCEL_YOUT1 = 0x3E
ACCEL_ZOUT0 = 0x3F
ACCEL_ZOUT1 = 0x40

TEMP_OUT0 = 0x41
TEMP_OUT1 = 0x42

GYRO_XOUT0 = 0x43
GYRO_XOUT1 = 0x44
GYRO_YOUT0 = 0x45
GYRO_YOUT1 = 0x46
GYRO_ZOUT0 = 0x47
GYRO_ZOUT1 = 0x48

REG_CONFIG = 0x1A
REG_INT_STATUS = 0x3A
INT_PIN_CFG = 0x37

# Electrical values for temperature
ROOM_TEMP_OFFSET = 21
TEMP_SENSITIVITY = 333.87

# Magnetometer Registers
AK8963_ADDRESS = 0x0C
AK8963_WHO_AM_I  = 0x00 
AK8963_WHOAMI = 0x48 # default value
AK8963_INFO = 0x01
AK8963_ST1 = 0x02  # data ready status bit 0
AK8963_XOUT_L = 0x03  # data
AK8963_XOUT_H = 0x04
AK8963_YOUT_L = 0x05
AK8963_YOUT_H = 0x06
AK8963_ZOUT_L = 0x07
AK8963_ZOUT_H = 0x08
AK8963_ST2 = 0x09  # Data overflow bit 3 and data read error status bit 2
AK8963_CNTL1 = 0x0A  # Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
AK8963_ASTC = 0x0C  # Self test control
AK8963_I2CDIS = 0x0F  # I2C disable
AK8963_ASAX = 0x10  # Fuse ROM x-axis sensitivity adjustment value
AK8963_ASAY = 0x11  # Fuse ROM y-axis sensitivity adjustment value
A8963_ASAZ = 0x12  # Fuse ROM z-axis sensitivity adjustment value

# CNTL1 Mode select
## Power down mode
AK8963_MODE_DOWN   = 0x00
## One shot data output
AK8963_MODE_ONE    = 0x01

## Continous data output 8Hz
AK8963_MODE_C8HZ   = 0x02
## Continous data output 100Hz
AK8963_MODE_C100HZ = 0x06

# Magneto Scale Select
## 14bit output
AK8963_BIT_14 = 0x00
## 16bit output
AK8963_BIT_16 = 0x01

# Magnetometer Scale Modifiers
MAGNOMETER_SCALE_MODIFIER_BIT_14 = 4912.0/8190.0
MAGNOMETER_SCALE_MODIFIER_BIT_16 = 4912.0/32760.0
    
class MPU9250():
    """
    
===============
 MPU9250 class
===============

.. class:: MPU9250(drvname, addr, clk=400000)

    Creates an intance of the MPU9250 class.

    :param drvname: I2C Bus used '( I2C0, ... )'
    :param addr: Slave address, default 0x69
    :param clk: Clock speed, default 400kHz
    
    Temperature, accelerometer, gyroscope and magnetometer values can be easily obtained from the sensor: ::

        from invensense.mpu9250 import mpu9250

        ...

        mpu = mpu9250.MPU9250(I2C0)

        temp, acc, gyro, magneto = mpu.get_values()

    """

    # dictionary of accel full-scale ranges
    accel_fullscale = {
        '2': 0,
        '4': 1,
        '8': 2,
        '16': 3
    }

    # list of accel sensitivity
    accel_sensitivity = [
        ACCEL_SENSITIVITY_2G,
        ACCEL_SENSITIVITY_4G,
        ACCEL_SENSITIVITY_8G,
        ACCEL_SENSITIVITY_16G
    ]

    # dictionary of gyro full-scale ranges
    gyro_fullscale = {
        '250': 0,
        '500': 1,
        '1000': 2,
        '2000': 3
    }

    # list of gyro sensitivity
    gyro_sensitivity = [
        GYRO_SENSITIVITY_250DPS,
        GYRO_SENSITIVITY_500DPS,
        GYRO_SENSITIVITY_1000DPS,
        GYRO_SENSITIVITY_2000DPS
    ]

    # list of magnetometer scale
    magneto_scale = [
        MAGNOMETER_SCALE_MODIFIER_BIT_14,
        MAGNOMETER_SCALE_MODIFIER_BIT_16
    ]

    # list of magnetometer mode
    magneto_mode = [
        AK8963_MODE_C8HZ,
        AK8963_MODE_C100HZ
    ]

    def __init__(self, drvname, addr=0x69, clk=400000):
        
        # lock init
        self.lockI2C = threading.Lock() # lock for I2C0

        self._init_MPU9250(drvname, addr, clk)

        self._init_AK8963(drvname, AK8963_ADDRESS, clk)

    def _init_MPU9250(self, drvname, addr, clk):

        if (addr != 0x68 and addr != 0x69):
            raise ValueError
        
        self.lockI2C.acquire()

        self.mpu9250 = i2c.I2C(drvname, addr, clk)
        try:
            self.mpu9250.start()
        except PeripheralError as e:
            print(e)

        # Check MPU9250 Who Am I Register
        if (self.mpu9250.write_read(MPU9250_WHO_AM_I, n=1)[0] != MPU9250_WHOAMI):
            raise ValueError
        self.lockI2C.release()

        # Set Clock source
        self.set_clock_source(1)
        # Set scales
        self.set_accel_fullscale(2)
        self.set_gyro_fullscale(2000)
        # Set dlpf mode
        self.set_dlpf_mode(0)
        # Disable Sleep Mode
        self.set_sleep_mode(False)
        # Enable config magnetometer
        self.lockI2C.acquire()
        self.mpu9250.write_bytes(INT_PIN_CFG, 0x02)
        self.lockI2C.release()

    def _init_AK8963(self, drvname, addr, clk):
        if (addr != 0x0C):
            raise ValueError

        self.lockI2C.acquire()

        self.ak8963 = i2c.I2C(drvname, addr, clk)
        try:
            self.ak8963.start()
        except PeripheralError as e:
            print(e)

        # Check AK8963 Who Am I Register
        if (self.ak8963.write_read(AK8963_WHO_AM_I, n=1)[0] != AK8963_WHOAMI):
            raise ValueError
        
        # set power down mode
        self.ak8963.write_bytes(AK8963_CNTL1, 0x00)

        # set read FuseROM mode
        self.ak8963.write_bytes(AK8963_CNTL1, 0x0F)

        # read coef data
        data = self.ak8963.write_read(AK8963_ASAX, n=3)

        self.magXcoef = (data[0] - 128) / 256.0 + 1.0
        self.magYcoef = (data[1] - 128) / 256.0 + 1.0
        self.magZcoef = (data[2] - 128) / 256.0 + 1.0
       
        self.lockI2C.release()

        self.set_magneto_config()

    # MPU-9250 Methods

    ##
    ## @brief      Get the value of bit SLEEP from PWR_MGMT_1 register.
    ##
    ## @param      self
    ## @return     value of bit SLEEP from PWR_MGMT_1 register.
    ##
    def is_sleep_mode(self):
        self.lockI2C.acquire()
        value = self.mpu9250.write_read(PWR_MGMT_1, n=1)[0]
        return ((value >> 6) & 1)
        self.lockI2C.release()

    ##
    ## @brief      Set the bit SLEEP of PWR_MGMT_1 register, according to the value of param state. 
    ##
    ## @param      self
    ## @param      state    boolean value that is the value of bit SLEEP of PWR_MGMT_1 register to set.
    ## @return     nothing
    ##
    def set_sleep_mode(self, state):
        if (state != True and state != False):
            raise ValueError
        
        self.lockI2C.acquire()

        value = self.mpu9250.write_read(PWR_MGMT_1, n=1)[0]

        if (state):
            value |= (1 << 6)
        else: 
            value &= ~(1 << 6)

        self.mpu9250.write_bytes(PWR_MGMT_1, value)
        self.lockI2C.release()
    
    def set_dlpf_mode(self, dlpf):
        """
    .. method:: set_dlpf_mode(dlpf)

        **Parameters**:

        **dlpf**: is the DLPF mode to set. Values range accepted are 0-7 (see datasheet).

        Set the DLPF mode.

        """
        if (dlpf not in [0, 1, 2, 3, 4, 5, 6, 7]):
            raise ValueError
        
        self.lockI2C.acquire()
        value = self.mpu9250.write_read(REG_CONFIG, n=1)[0]
        value &= 0b11111000
        value |= dlpf
        self.mpu9250.write_bytes(REG_CONFIG, value)
        self.lockI2C.release()

    def get_clock_source(self):
        """
    .. method:: get_clock_source()

        Return the clock source the sensor is set to.

        """
        self.lockI2C.acquire()
        clock_source = self.mpu9250.write_read(PWR_MGMT_1, n=1)[0]
        clock_source &= 0b00000111
        self.lockI2C.release()
        return clock_source

    def set_clock_source(self, clksel):
        """
    .. method:: set_clock_source(clksel)

        **Parameters**:

        **clksel**: is the clock source to set. Values accepted are 0, 1, 2, 3, 4, 5 or 7.

        ======== =====================
         clksel    Clock source
        ======== =====================
         0         Internal 8MHz oscillator
         1         PLL with X axis gyroscope reference
         2         PLL with Y axis gyroscope reference
         3         PLL with Z axis gyroscope reference
         4         PLL with external 32.768kHz reference
         5         PLL with external 19.2MHz reference
         6         Reserved
         7         Stops the clock and keeps the timing generator
        ======== =====================

        Set the clock source. 

        """
        if (clksel not in [0, 1, 2, 3, 4, 5, 7]):
            raise ValueError
        
        self.lockI2C.acquire()
        value = self.mpu9250.write_read(PWR_MGMT_1, n=1)[0]
        value &= 0b11111000
        value |=  clksel
        self.mpu9250.write_bytes(PWR_MGMT_1, value)
        self.lockI2C.release()

    def get_temp(self):
        """
    .. method:: get_temp()

        Return the temperature in degrees Celsius.

        """
        self.lockI2C.acquire()
        # Read the raw data from the registers
        data = self.mpu9250.write_read(TEMP_OUT0, n=2)
        raw_temp = _tc(data[0] << 8 | data[1])
        # Get the actual temperature using the formule given in the MPU-9250 Register Map and Descriptions
        actual_temp = ((raw_temp - ROOM_TEMP_OFFSET) / TEMP_SENSITIVITY) + 21
        self.lockI2C.release()

        # Return the temperature
        return actual_temp
    
    def set_accel_fullscale(self, full_scale):
        """
    .. method:: set_accel_fullscale(full_scale)

        :param full_scale: is the full-scale range to set the accelerometer to. Possible values are 2, 4, 8 or 16.
        
        Set the full-scale range of the accelerometer.

        """
        if (full_scale not in [2, 4, 8, 16]):
            raise ValueError
        
        self.lockI2C.acquire()
        # First change it to 0x00 to make sure we write the correct value later
        self.mpu9250.write_bytes(ACCEL_CONFIG, 0x00)

        # get corrisponding full-scale value from dictionary
        full_scale = self.accel_fullscale[str(full_scale)]

        # Write the new full-scale to the ACCEL_CONFIG register
        value = self.mpu9250.write_read(ACCEL_CONFIG, n=1)[0]
        value &= 0b11100111
        value |= (full_scale << 3)
        self.mpu9250.write_bytes(ACCEL_CONFIG, value)
        self.lockI2C.release()

    def get_accel_fullscale(self):
        """
    .. method:: get_accel_fullscale()
        
        Return the full-scale value the accelerometer is set to.
        When something went wrong, it returns -1.
        
        """
        self.lockI2C.acquire()
        # Get the raw value
        raw_data = self.mpu9250.write_read(ACCEL_CONFIG, n=1)[0]
        raw_data &= 0b00011000
        raw_data >>= 3
        self.lockI2C.release()

        # get accel full-scale
        values = ['2', '4', '8', '16']
        for i in range(4):
            if (raw_data == self.accel_fullscale[values[i]]):
                return values[i]
        return -1

    def get_accel_values(self, g = False):
        """
    .. method:: get_accel_values(g = False)

        :param g: is the format of accelerometer values. 
                  If g = False is m/s^2, otherwise is g. 
                  Default value is False.
        
        Return the X, Y and Z accelerometer values in a dictionary.
        
        """
        if (g != True and g != False):
            raise ValueError
        
        self.lockI2C.acquire()
        # Read the raw data from the registers
        data = self.mpu9250.write_read(ACCEL_XOUT0, n=6)
        x = _tc(data[0] << 8 | data[1]) # X-axis value
        y = _tc(data[2] << 8 | data[3]) # Y-axis value
        z = _tc(data[4] << 8 | data[5]) # Z-axis value
        self.lockI2C.release()

        full_scale = self.get_accel_fullscale()

        # set default accel sensitivity
        accel_sensitivity = ACCEL_SENSITIVITY_2G

        # get accel full-scale
        values = ['2', '4', '8', '16']
        for i in range(len(values)):
            if (full_scale == self.accel_fullscale[values[i]]):
                accel_sensitivity = self.accel_sensitivity[i]

        x = x / accel_sensitivity
        y = y / accel_sensitivity
        z = z / accel_sensitivity

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * GRAVITIY_MS2
            y = y * GRAVITIY_MS2
            z = z * GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_fullscale(self, full_scale):
        """
    .. method:: set_gyro_fullscale(full_scale)

        :param full_scale: is the full-scale range to set the gyroscope to. Values accepted: 250, 500, 1000 or 2000.
        
        Set the full-scale range of the gyroscope.
        
        """
        if (full_scale not in [250, 500, 1000, 2000]):
            raise ValueError
        
        self.lockI2C.acquire()

        # First change it to 0x00 to make sure we write the correct value later
        self.mpu9250.write_bytes(GYRO_CONFIG, 0x00)

        # get gyro full-scale from dictionary
        full_scale = self.gyro_fullscale[str(full_scale)]

        # Write the new full-scale to the ACCEL_CONFIG register
        value = self.mpu9250.write_read(GYRO_CONFIG, n=1)[0]
        value &= 0b11100111
        value |= (full_scale << 3)
        self.mpu9250.write_bytes(GYRO_CONFIG, value)

        self.lockI2C.release()

    def get_gyro_fullscale(self):
        """
    .. method:: get_gyro_fullscale()
        
        Return the full-scale value the gyroscope is set to.
        When something went wrong, it returns -1.
        
        """
        self.lockI2C.acquire()
        # Get the raw value
        raw_data = self.mpu9250.write_read(GYRO_CONFIG, n=1)[0]
        raw_data &= 0b00011000
        raw_data >>= 3
        self.lockI2C.release()

        # get gyro full-scale
        values = ['250', '500', '1000', '2000']
        for i in range(len(values)):
            if (raw_data == self.gyro_fullscale[values[i]]):
                return values[i]
        return -1

    def get_gyro_values(self):
        """
    .. method:: get_gyro_values()
        
        Return the X, Y and Z gyroscope values in a dictionary.
        
        """
        self.lockI2C.acquire()
        # Read the raw data from the registers
        data = self.mpu9250.write_read(GYRO_XOUT0, n=6)
        x = _tc(data[0] << 8 | data[1]) # X-axis value
        y = _tc(data[2] << 8 | data[3]) # Y-axis value
        z = _tc(data[4] << 8 | data[5]) # Z-axis value
        self.lockI2C.release()

        full_scale = self.get_gyro_fullscale()

        # set default gyro sensitivity
        gyro_sensitivity = GYRO_SENSITIVITY_250DPS

        # get gyro sensitivity
        values = ['250', '500', '1000', '2000']
        for i in range(len(values)):
            if (full_scale == self.gyro_fullscale[values[i]]):
                gyro_sensitivity = self.gyro_sensitivity[i]

        x = x / gyro_sensitivity
        y = y / gyro_sensitivity
        z = z / gyro_sensitivity

        return {'x': x, 'y': y, 'z': z}

    def set_magneto_config(self, mfs=0, mode=0):
        """
    .. method:: set_magneto_config(mfs = 0, mode = 0)

        **Parameters**:

        **mfs**: is the magneto scale (default value is 0).

        ======== =====================
         mfs       Magneto Scale
        ======== =====================
         0        14 bit resolution
         1        16 bit resolution
        ======== =====================
        
        **mode**: is the magneto mode (default value is 0).

        ======== =====================
         mode       Magneto Mode
        ======== =====================
         0        Continous data output 8Hz
         1        Continous data output 100Hz
        ======== =====================

        Set the magnetometer scale and mode.

        """
        
        if (mfs not in [0, 1]):
            raise ValueError
            
        if (mode not in [0, 1]):
            raise ValueError
        
        self.mres = self.magneto_scale[mfs]
        mode = self.magneto_mode[mode]
        
        self.lockI2C.acquire()
        # set power down mode
        self.ak8963.write_bytes(AK8963_CNTL1, 0x00)
        # set scale and continous mode
        self.ak8963.write_bytes(AK8963_CNTL1, (mfs << 4 | mode))
        self.lockI2C.release()
    
    def get_magneto_values(self):
        """
    .. method:: get_magneto_values()
        
        Return the X, Y and Z magnetometer values in a dictionary.
        
        """
        x = y = z = 0

        self.lockI2C.acquire()
        # check data ready
        drdy = self.ak8963.write_read(AK8963_ST1, n=1)[0]
        if drdy & 0x01 :
            data = self.ak8963.write_read(AK8963_XOUT_L, n=7)

            # check overflow
            if (data[6] & 0x08) != 0x08:
                x = _tc(data[1] << 8 | data[0])
                y = _tc(data[3] << 8 | data[2])
                z = _tc(data[5] << 8 | data[4])

                x = x * self.mres * self.magXcoef
                y = y * self.mres * self.magYcoef
                z = z * self.mres * self.magZcoef
        self.lockI2C.release()

        return {'x': x, 'y': y, 'z': z}
        
    def get_values(self, g=False):
        """
    .. method:: get_values(g = False)
        
        :param g: is the format of accelerometer values. 
                  If g = False is m/s^2, otherwise is g. 
                  Default value is False.

        Return the values of temperature, gyroscope, accelerometer and magnetometer in a list [temp, accel, gyro, magneto].
        
        """
        if (g != True and g != False):
            raise ValueError

        temp = self.get_temp()
        accel = self.get_accel_values(g)
        gyro = self.get_gyro_values()
        magneto = self.get_magneto_values()

        return [temp, accel, gyro, magneto]

    def is_data_ready(self):
        """
    .. method:: is_data_ready()

        Return 1 if data is ready, otherwise 0.
        
        """
        self.lockI2C.acquire()
        data = self.mpu9250.write_read(REG_INT_STATUS, n=1)[0]
        is_ready = ((data >> 0) & 1)
        self.lockI2C.release()
        return is_ready