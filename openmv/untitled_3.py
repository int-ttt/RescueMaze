# Untitled - By: leekwangseok - Sat Jan 18 2025
import sensor, image, pyb, time, sys
import micropython
import math, struct
from pyb import UART, Pin, Timer
from time import sleep_ms, sleep_us
from pyb import I2C
from machine import SoftI2C, Pin
from VL53L0X import VL53L0X
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

I2C_J7 = SoftI2C(scl = 'P2', sda = 'P3', freq = 800000, timeout=50000) #Soft I2C
I2C_J8 = SoftI2C(scl = 'P4', sda = 'P5', freq = 800000, timeout=50000) #Soft I2C
I2C_J9 = SoftI2C(scl = 'P7', sda = 'P8', freq = 800000, timeout=50000) #Soft I2C
I2C_J15= SoftI2C(scl = 'P6', sda = 'P9', freq = 800000, timeout=50000) #Soft I2C


clock = time.clock()

TOF1 = VL53L0X(I2C_J7)

TOF1.start()

while(True):
    print(TOF1.read())
    pyb.delay(100)

