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


from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from cam import *
from ucollections import namedtuple
from pybricks.iodevices import UARTDevice

PacketIndex = 0
ObjectIndex = 0
# g1 = GyroSensor(Port.S1)
# g2 = GyroSensor(Port.S4)
rm = Motor(Port.C,gears=[12,20])
lm = Motor(Port.B, gears=[12,20])
robot = DriveBase(lm, rm, wheel_diameter=75, axle_track=120)
gyro = GyroSensor(Port.S4)
# us = UltrasonicSensor(Port.S2)
azimuth = namedtuple('azimuth', ['n', 's', 'w', 'e'])
direction = namedtuple('direction', ['x', 'y'])
# Initialize the EV3
ev3 = EV3Brick()
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
    


"""
    pid movement control
"""
lastYaw = 0
def pid_control(speed, gain, kp, kd):
    # 전역변수를 지역변수로
    global lastYaw; global max_error
    # 라인트레이싱 - PID control
    yaw = 0 + gyro.angle()
    differential = yaw - lastYaw
    lastYaw = yaw
    # 적분 상수를 0으로 입력할 경우
    steering = (kp * yaw + kd * differential) * gain
    
    robot.drive(-speed, steering)

lastTurnYaw = 0
def pid_turn_control(dest, gain, kp, kd):
    global lastTurnYaw
    yaw = dest+gyro.angle()
    diff = yaw - lastTurnYaw
    lastTurnYaw = yaw
    steering = (kp * yaw + kd * diff) * gain

    robot.drive(0, steering)


"""
    movement functions
"""

def pid_turn(dest, time = 1700):
    global heading
    i = 0
    turn_cl.reset()

    while turn_cl.time() < time:
        g = gyro.angle()
        print(g, gyro.angle() + dest + int(dest / 90))
        pid_turn_control(dest + int(dest / 90), 7.6, 1.3, 2)
        if gyro.angle() == 0 and i == 0:
            turn_cl.reset()
            i = 1
    print(int(dest / 90))
    gyro.reset_angle(0)
    heading += int(dest / 90)
        
def back():
    pid_turn(180, 2000)
    
    ser.clear()
    robot.stop(Stop.HOLD)
    robot.reset()
    wait(30)
    gyro.reset_angle(0)

    while True:
        
        tof = getTOF()
        pid_control(200, 6, 1, 1.7)
        if tof.condition:
            if tof.t3 != 0 and tof.t3 <= 68:
                break
        if robot.distance() < -285:
            break
    robot.stop()
    

def node():
    global heading, openList
    robot.reset()
    print(robot.distance())
    gyro.reset_angle(0)
    
    ser.clear()
    while True:
        tof = getTOF()
        pid_control(200, 6, 1, 1.7)
        if tof.condition:
            if tof.t3 != 0 and tof.t3 <= 68:
                break
        if robot.distance() < -285:
            break
    print(tof)
    robot.stop()
    lm.stop()
    rm.stop()
    tof = gTOF()
    n, w, e = 1, 1, 1
    if tof.t1 > 150:
        openList.insert(0, nextDir[heading][2])
        e = 0
    if tof.t2 > 150:
        w = 0
        openList.insert(0, nextDir[heading][1])
    if tof.t3 > 200:
        n = 0
    n, s, w, e = getAzimuth(n, w, e)
    print(int(str(int(e)) + str(int(w)) + str(int(s)) +  str(int(n)), 2), n, w, e)

    return tof

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

"""
    heading
"""

def checkHeading():
    global heading
    if heading < 0:
        heading += 4
    elif heading > 3:
        heading -= 4

def getAzimuth(n, w, e):
    s = 0
    tn, ts, tw, te = n, 0, w, e
    if heading == 3:
        tn = e
        tw = n
        ts = w
        te = s
    elif heading == 2:
        tn = s
        tw = e
        ts = n
        te = w
    elif heading == 1:
        tn = w
        tw = s
        ts = e
        te = n
    return tn, ts, tw, te



nextDir = [
    [direction(0, -1), direction(-1, 0), direction(1, 0)],
    [direction(-1, 0), direction(0, 1), direction(0, -1)],
    [direction(0, 1), direction(1, 0), direction(-1, 0)],
    [direction(1, 0), direction(0, -1), direction(0, 1)]
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

"""             ___             ___
    0=      1=      2=      3=
                        ___     ___
                ___             ___
    4= |    5= |    6= |    7= |   
       |       |       |___    |___
                ___             ___
    8=     |9=     |10=    |11=    |
           |       |    ___|    ___|
                ___             ___
    12=|   |13=|   |14=|   |15=|   |
       |   |   |   |   |___|   |___|
    0000 0001 0010 0011
    0100 0101 0110 0111
    1000 1001 1010 1011
    1100 1101 1110 1111
"""

back_list = []

"""
    list configuration
"""

def appendList(dir):
    global grid
    tGrid = [([0 for i in range(len(grid[0])+ abs(dir.x))] + []) for j in range(len(grid) + abs(dir.y))]
    print(tGrid)
    grid = tGrid

tof = gTOF()
n, w, e = tof.t3 < 150, tof.t1 < 150, tof.t2 < 150



grid = [[int(str(int(e)) + str(int(w)) + '1' + str(int(n)), 2)]]
print(grid)

ser.clear()

node()
pid_turn(-90)
node()
pid_turn(-90)
node()
back()
pid_turn(-90)
node()
pid_turn(-90)
node()
pid_turn(90)
node()
node()
pid_turn(90)

node()
pid_turn(90)
node()
pid_turn(-90)
node()
node()
pid_turn(-90)
node()
back()
node()
pid_turn(90)
node()
pid_turn(-90)
node()
node()
node()
pid_turn(90)
node()
back()
node()
pid_turn(-90)
node()
pid_turn(90)
node()


quit()

print(1)
for i in range(5):
    

    tof = node()
    checkHeading()
    nextNode = openList[0]
    ser.clear()
    tof = gTOF()
    if tof.t3 < 200:
        if nextNode[0] == 1:
            pid_turn(90)
        else:
            pid_turn(-90)
        
        openList.remove(nextNode)
    gyro.reset_angle(0)
    print(openList, tof, nextNode)
    robot.reset()

# while True:
#     tof = node()

    
#     pid_turn(90)
#     pass