# Untitled - By: JEON - 2023년 5월 11일

# 이프로그램은 OpenMV 카메라에서 물체를 찾아 해당 데이터를 UART로 보내는 프로그램.
#
# 보내는 PACKET의 포맷은 아래와 같음.
#
# PACKET FORMAT (Unsigned Integer 2 Byte는 LSB, MSB순으로 되어 있으며 Little Endian 형태로 전송됨.)
# ---------------------------------------------------------------------------------------------------------
# PACKET HEADER, PACKET SeqNo, ULTRA, TOF, YAW, ROLL, PITCH, Object Count n, OBJECT[0]...OBJECT[n-1], CHECKSUM
# ---------------------------------------------------------------------------------------------------------
# 1. PACKET HEADER  : 2 Byte (0xAA , 0xCC)              -- 0XAA와 0xCC 가 연달아 들어오면 PCKET의 시작임.
# 2. PACKET SeqNo   : Unsigned Integer 1 Byte (0 ~ 127) -- PACKET 송신시마다 1씩 증가함. (수신측에서 이값이 바껴야 새로운 PACKET으로 인식)
# 3. TOF[4]         : 4개의 TOF 정보. 구조는 아래와 같음. (전체 4바이트)
#       -----------------------------------------
#       : TOF1, TOF2, TOF3, TOF4
#       -----------------------------------------
#       : TOF1          : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
#       : TOF2          : Unsigned Integer 2 Byte
#       : TOF3          : Unsigned Integer 2 Byte
#       : TOF4          : Unsigned Integer 2 Byte
#       -----------------------------------------
# ---------------------------------------------------------------------------------------------------------
# 9. CHECKSUM : PACKET HEADER를 포함한 모든 수신데이터의 Exclusive Or 값. (chksum ^= all recieve data)
# ---------------------------------------------------------------------------------------------------------
mode = 0
tof1 = 0
tof2 = 0
yaw = 0.0
roll = 0.0
pitch = 0.0

imuV = []

Blobs = [(20, 51, 33, 72, 19, 127),
         (39, 70, -57, -10, -15, 68),
         (0, 100, -128, 127, -128, 127) ] #Blobs[x] ; x=0: red, x=1: green, x=2: blue

Circle = []
Blob = []
Blue = []
Median = []
Mean = []

PACKET = b''

PacketSeqNO = 0

import sensor, image, pyb, time, sys
import micropython
import math, struct
from pyb import UART, Pin, Timer
from time import sleep_ms, sleep_us
from pyb import I2C
from machine import SoftI2C, Pin
from VL53L0X import VL53L0X
from MLX90614 import MLX90614



micropython.alloc_emergency_exception_buf(200)

length = {'Int8' : 1, 'uInt8' : 1, 'Int16' : 2, 'uInt16' : 2, 'Int32' : 4, 'uInt32' : 4, 'float' : 4}
format = {'Int8' : '<b', 'uInt8' : '<B', 'Int16' : '<h', 'uInt16' : '<H', 'Int32' : '<l', 'uInt32' : '<L', 'float' : '<f'}

def addData(type, array):
    global PACKET
    if isinstance(array,list):
        array = array[:5]     # max array size is 5 byte
        for element in array:
            PACKET += struct.pack(format[type], element)
    else:
        PACKET += struct.pack(format[type], array)

#---------------------------------------------------------------------
# 사용할 I2C 채널을 선택하세요.
#---------------------------------------------------------------------
#---------------------------------------------------------------------
# Software I2C 채널
#---------------------------------------------------------------------
I2C_J7 = SoftI2C(scl = 'P2', sda = 'P3', freq = 800000, timeout=50000) #Soft I2C
I2C_J8 = SoftI2C(scl = 'P4', sda = 'P5', freq = 800000, timeout=50000) #Soft I2C
I2C_J9 = SoftI2C(scl = 'P7', sda = 'P8', freq = 800000, timeout=50000) #Soft I2C
I2C_J15= SoftI2C(scl = 'P6', sda = 'P9', freq = 800000, timeout=50000) #Soft I2C

#---------------------------------------------------------------------
# Hardware I2C 채널
#---------------------------------------------------------------------
#I2C_J8 = I2C(2, I2C.MASTER, baudrate = 100000)    #Hardware I2C
#I2C_J9 = I2C(4, I2C.MASTER, baudrate = 400000)    #Hardware I2C
#---------------------------------------------------------------------
TOF1 = VL53L0X(I2C_J7)
TOF2 = VL53L0X(I2C_J8)
TOF3 = VL53L0X(I2C_J9)
TEMP1 = MLX90614(I2C_J7)
TEMP2 = MLX90614(I2C_J8)
#TOF4 = VL53L0X(I2C_J15)
#IMU = BNO055(I2C_J9)
TOF1.start()
TOF2.start()
TOF3.start()
#TOF4.start()

sensor.reset()
base_height = 47

td1 = 109 - base_height
td2 = 57 - base_height
td3 = 64 - base_height
#td4 = 101 - base_height

# -------------------------------------------------------------
# UART 1, and baudrate.
# -------------------------------------------------------------
uart = UART(1, 115200)

def getTime():
    return time.time() - timediff

rcvP = bytearray([])
timediff = 0
while True:
    rcvP = uart.read()
    if rcvP != 0xAA:
        timediff = time.time()
        break

t1 = 0
t2 = 0
t3 = 0
t4 = 0

while True:
    PACKET = b''
    t1 = TOF1.read()
    t2 = TOF2.read()
    t3 = TOF3.read()
#    t4 = TOF4.read()

    te1 = TEMP1.getObjCelsius()
    te2 = TEMP2.getObjCelsius()

#    print(t1, t2, t3, t4)
    tes1 = str(te1).split('.')
    tes2 = str(te2).split('.')

#    t4v = t4 - td4
#    t1v = 0
#    t2v = 0
    t1v = t1 - td1
    t2v = t2 - td2
    t3v = t3 - td3
    t4v = 0
    errorPacket = 0
    print(t1, t2, t3, t4, t1v, t2v, t3v, t4v)

    addData('uInt8', 0xAA)         # PACKET HEADER 0xAA, 0xCC
    addData('uInt8', 0xCC)
    addData('uInt8', PacketSeqNO)
    addData('uInt8', errorPacket)        # Error Packet

    addData('uInt16', t1v)              # TOF1 : 부호 없는 2 바이트 정수('<H').
    addData('uInt16', t2v)              # TOF2 : 부호 없는 2 바이트 정수('<H').
    addData('uInt16', t3v)              # TOF3 : 부호 없는 2 바이트 정수('<H').
    addData('uInt16', t4v)              # TOF4 : 부호 없는 2 바이트 정수('<H').
    addData('uint8', int(tes1[0]))           # TEMP1 : 부호 없는 1 바이트 온도 정수 부분
    addData('uint8', int(tes1[1][0:2])) # TEMP1 : 부호 없는 1 바이트 온도 소수 부분
    addData('uint8', int(tes2[0]))           # TEMP2 : 부호 없는 1 바이트 온도 정수 부분
    addData('uint8', int(tes2[1][0:2])) # TEMP2 : 부호 없는 1 바이트 온도 소수 부분

    chksum = 0
    for b in PACKET:
       chksum ^= b
    addData('uInt8', chksum)

    uart.write(PACKET)
    pyb.delay(10)

