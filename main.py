#!/usr/bin/env pybricks-micropython

# 받는 PACKET의 포맷은 아래와 같음.
#
# PACKET FORMAT (Unsigned Integer 2 Byte는 LSB, MSB순으로 되어 있으며 Little Endian 형태로 전송됨.)
# ---------------------------------------------------------------------------------------------------------
# PACKET HEADER, PACKET SeqNo, ULTRA, TOF, YAW, ROLL, PITCH, Object Count n, OBJECT[0]...OBJECT[n-1], CHECKSUM
# ---------------------------------------------------------------------------------------------------------
# 1. PACKET HEADER  : 2 Byte (0xAA , 0xCC)              -- 0XAA와 0xCC 가 연달아 들어오면 PCKET의 시작임.
# 2. PACKET SeqNo   : Unsigned Integer 1 Byte (0 ~ 127) -- PACKET 송신시마다 1씩 증가함. (수신측에서 이값이 바껴야 새로운 PACKET으로 인식)
# 3. TOF1   : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
# 4. TOF2   : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
# 5. TOF3   : Unsigned Integer 2 Byte   -- VL53L0X를 이용한 TOF 거리 측정 값.(Format:'<H')
# 6. YAW    : Float 4 Byte              -- BNO055를 이용한 YAW 값.(Format:'<f')
# 7. ROLL   : Float 4 Byte              -- BNO055를 이용한 ROLL 값.(Format:'<f')
# 8. PITCH  : Float 4 Byte              -- BNO055를 이용한 PITCH 값.(Format:'<f')
# 9. OBJECT COUNT n  : Unsigned Integer 1 Byte       -- 포함하고 있는 Object의 갯수
# 10. OBJECT[n]      : n개의 Object 정보. 구조는 아래와 같음. (OBJECT당 10 Byte)
#       -----------------------------------------
#       : Object Type, X, Y, Width, Height
#       -----------------------------------------
#       : Object Type   : Unsigned Integer 2 Byte  -- Object의 종류 (예: 1 = Silver Ball, 2 = Black Ball ...)
#       : X             : Unsigned Integer 2 Byte
#       : Y             : Unsigned Integer 2 Byte
#       : R             : Unsigned Integer 2 Byte
#       : MAGNITUDE     : Unsigned Integer 2 Byte
#       -----------------------------------------
# ---------------------------------------------------------------------------------------------------------
# 10. CHECKSUM : PACKET HEADER를 포함한 모든 수신데이터의 Exclusive Or 값. (chksum ^= all recieve data)
# ---------------------------------------------------------------------------------------------------------

import struct

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from ucollections import namedtuple
from pybricks.iodevices import UARTDevice

PacketIndex = 0
ObjectIndex = 0
g1 = GyroSensor(Port.S1)
# g2 = GyroSensor(Port.S4)
rm = Motor(Port.C)
lm = Motor(Port.B)
robot = DriveBase(lm, rm, wheel_diameter=75, axle_track=120)
gyro = GyroSensor(Port.S4)
TOFData = namedtuple("TOFData", ["condition", "t1", "t2", "t3", "t4"])
null_tof = TOFData(False, 0, 0, 0, 0)
# us = UltrasonicSensor(Port.S2)
azimuth = namedtuple('azimuth', ['n', 's', 'w', 'e'])
direction = namedtuple('direction', ['x', 'y'])

turn_cl = StopWatch()

def check_drift():
    global using_gyro
    
    if robot.state()[1] == 0:
        print("checking drift")
        g1a = g1.angle()
        g2a = g2.angle()
        wait(100)
        if g1a == g1.angle():
            using_gyro = g1
        elif g2a == g2.angle():
            using_gyro = g2
        else:
            g1.reset_angle(last_angle)
            g2.reset_angle(last_angle)
    else:
        print('motor is running')

# last_angle = using_gyro.angle()
# robot.drive(0, 80)
# while using_gyro.angle() - last_angle < 90:
#     print(g1.angle(), g2.angle(), using_gyro.angle())
# robot.stop()
# wait(100)
# print(g1.angle(), g2.angle())
# check_drift()
    

rcvPACKET = bytearray([])
Object = list([])
ObjectCount = 0

def _parsing(_data) -> tuple["uInt16", "uInt16", "uInt16", "uInt16", "uInt16"]:
    buf = struct.unpack("<hhhhh", _data)
    return tuple(buf)

def readPacket() -> bool:
    global PacketIndex
    global ObjectIndex
    global rcvPACKET
    global ObjectCount
    
    read = False

    rcvPACKET = bytearray([])
    rcvPACKET += ser.read()
    PacketIndex = 0
    if rcvPACKET[0] == 0xAA:
        PacketIndex = 1
        rcvPACKET += ser.read()
        if rcvPACKET[1] == 0xCC:
            PacketIndex = 2
            rcvPACKET += ser.read(10)

            rcvPACKET += ser.read() #CHECKSUM

            chksum = 0
            for r in rcvPACKET:     #CHECKSUM 계산
                chksum ^= r
                
            if chksum != 0:         #CHECKSUM ERROR
                print("CHECK SUM ERROR")
                return False

            return True
    return False
            
def getTOF():
    read = True
    if ser.waiting() >= 5:  # 수신 데이터가 있으면 (최소 사이즈는 5 바이트임)
        wait(1) 
        if readPacket():    # 데이터 수신에 성공했으면 Parsing 함
            tof1 = struct.unpack("<H",rcvPACKET[4:6])[0]
            tof2 = struct.unpack("<H",rcvPACKET[6:8])[0]
            tof3 = struct.unpack("<H",rcvPACKET[8:10])[0]
            tof4 = struct.unpack("<H",rcvPACKET[10:12])[0]

            # print("OK")
            # print("TOF1  : ",tof1)
            # print("TOF2  : ",tof2)
            # print("TOF3  : ",tof3)
            # print("TOF4  : ",tof4)
            # print(rcvPACKET, ser.read_all())
            # if tof1 < 50:
            #     print("TOF1  : ",tof1)
            # if tof2 < 50:
            #     print("TOF2  : ",tof2)
            return TOFData(True, tof1, tof2, tof3, tof4)
        else: return null_tof
    else:
        return null_tof

def gTOF():
    while True:
        tof = getTOF()
        if tof.condition:
            break
    return tof

lastYaw = 0
def pid_control(speed, gain, kp, kd):
    # 전역변수를 지역변수로
    global lastYaw; global max_error
    # 라인트레이싱 - PID control
    yaw = 0 - gyro.angle()
    differential = yaw - lastYaw
    lastYaw = yaw
    # 적분 상수를 0으로 입력할 경우
    steering = (kp * yaw + kd * differential) * gain
    
    robot.drive(-speed, steering)

lastTurnYaw = 0
def pid_turn_control(dest, gain, kp, kd):
    global lastTurnYaw
    yaw = dest-gyro.angle()
    diff = yaw - lastTurnYaw
    lastTurnYaw = yaw
    steering = (kp * yaw + kd * diff) * gain

    robot.drive(0, steering)

# Initialize the EV3
ev3 = EV3Brick()

def pid_turn(dest):
    i = 0
    turn_cl.reset()
    print(turn_cl.time())
    while turn_cl.time() < 1700:
        pid_turn_control(dest, 5, 1, 0)
        if gyro.angle() == 0 and i == 0:
            turn_cl.reset()
            i = 1
        
        
    


# Initialize sensor port 2 as a uart device
ser = UARTDevice(Port.S3, 115200)
ser.clear()
wait(100)
ser.write(b'\xaa')

gyro.reset_angle(0)
#
# print(us.distance())
#
# while us.distance() >= :
#     pid_control(100, 5, 1, 0)
#
# tof = gTOF()
#
# if tof.t1 > 100:
#     pid_turn(90)
# elif tof.t2 > 100:
#     pid_turn(-90)
#


nextDir = [
    [[0, -1], [-1, 0], [1, 0]],
    [[-1, 0], [0, 1], [0, -1]],
    [[0, 1], [1, 0], [-1, 0]],
    [[1, 0], [0, -1], [0, 1]]
]


openList = []
heading = 0 # heading
            # north = 0
            # west 1
            # south = 2
            # east 3

# dir
# forward 0
# left 1
# right 2

def node():
    global heading, openList
    gyro.reset_angle(0)
    robot.reset()
    while True:
        tof = getTOF()
        pid_control(150, 5, 1, 0)
        if tof.condition:
            if tof.t3 <= 50:
                break
        if robot.distance() < -300:
            break
    robot.stop()
    tof = gTOF()
    rotation = 0
    if tof.t1 > 150:
        openList.insert(0, nextDir[heading][2])
        rotation = -1
    if tof.t2 > 150:
        openList.insert(0, nextDir[heading][1])
        rotation = 1
    

    return tof

# while True:
#     while us.distance() >= 60:
#         getTOF()
#         pid_control(150, 5, 1, 0)
#     wait(50)
#     tof = gTOF()
#     print(tof.t1, tof.t2)
#     if tof.t1 > 100:
#         robot.turn(90)
#     elif tof.t2 > 100:
#         robot.turn(-90)
#     else:
#         robot.turn(180)
#     ev3.speaker.beep()
#     robot.stop(Stop.BRAKE)
#     wait(10)
#     gyro.reset_angle(0)


# while True:
#     while True:
#         ev3.screen.clear()
#         tof = getTOF()
#         ev3.screen.draw_text(25, 25, str(tof.t1) + " " + str(tof.t2) + " " + str(tof.t3) + " " +  str(tof.t4))
#         wait(5)
#         if Button.CENTER in ev3.buttons.pressed():
#             break
#     while us.distance() > 20:
#         tof = getTOF()
#         print(tof.t1, tof.t2)
#         pid_control(100, 5, 1, 0)
#     robot.stop()
#     break

print(1)
for i in range(5):
    tof = node()

    nextNode = openList[0]
    tof = gTOF()
    if tof.t3 < 200:
        if nextNode[0] == 1:
            pid_turn(90)
        else:
            pid_turn(-90)
        
        openList.remove(nextNode)
    gyro.reset_angle(0)

    print(openList, tof, nextNode)

# while True:
#     tof = node()

    
#     pid_turn(90)
#     pass