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
from pybricks.parameters import Port, Stop, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from cam import *
from ucollections import namedtuple
from pybricks.iodevices import UARTDevice
from grid import *

PacketIndex = 0
ObjectIndex = 0
# g1 = GyroSensor(Port.S1)
# g2 = GyroSensor(Port.S4)
rm = Motor(Port.C,gears=[12,20])
lm = Motor(Port.B, gears=[12,20])
robot = DriveBase(lm, rm, wheel_diameter=75.1, axle_track=120)
gyro = GyroSensor(Port.S4)
# us = UltrasonicSensor(Port.S2)
azimuth = namedtuple('azimuth', ['n', 's', 'w', 'e'])
nodeData = namedtuple('nodeData', ['x', 'y', ''])
# Initialize the EV3
ev3 = EV3Brick()
turn_cl = StopWatch()
speed_cl = StopWatch()

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

def turn_correction():
    turn_cl.reset()
    i = 0
    while turn_cl.time() < 1500:
        pid_turn_control(0, 7.6, 1.3, 2)

def pid_turn(dest, time = 1700):
    global heading, lastYaw
    i = 0
    turn_cl.reset()

    while turn_cl.time() < time:
        pid_turn_control(dest + int(dest / 90), 7.6, 1.3, 2)
        if gyro.angle() == 0 and i == 0:
            turn_cl.reset()
            i = 1
    print(int(dest / 90))
    gyro.reset_angle(0)
    lastYaw = 0
    heading += int(dest / 90)
    checkHeading()
        
def back():
    pid_turn(180, 2250)

    ser.clear()

    wait(30)
    if gyro.angle() != 0:
        turn_correction()
    robot.reset()
    gyro.reset_angle(0)

    while True:
        
        tof = getTOF()
        pid_control(200, 6, 1, 1.7)
        if tof.condition:
            if tof.t3 != 0 and tof.t3 <= 90:
                break
        if robot.distance() < -295:
            break
    robot.stop()
    

def node(dir):
    global heading, openList
    robot.reset()
    print("heading : ",robot.distance(), heading)
    gyro.reset_angle(0)
    print(openList)

    ser.clear()
    while True:
        tof = getTOF()
        pid_control(200, 6, 1, 1.7)
        if tof.condition:
            if tof.t3 != 0 and tof.t3 <= 90:
                break
        if robot.distance() < -295:
            break
    print(tof)
    robot.stop(Stop.BRAKE)
    tof = gTOF()
    on, ow, oe = tof.t3 <= 200, tof.t2 <= 170, tof.t1 <= 170
    n, s, w, e = correctAzimuth(int(on), int(ow), int(oe))
    wall = int(str(e) + str(w) + str(s) +  str(n), 2)
    print(wall, n, w, e, dir)
    appendList(dir, wall)
    if not oe:
        openList.insert(0, nextDir[heading][2])
    if not on:
        openList.insert(0, nextDir[heading][0])
    if not ow:
        openList.insert(0, nextDir[heading][1])
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

def correctAzimuth(n, w, e):
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

def getAzimuth(wall):
    b = format(wall, 'b')
    e, w, s, n = b
    return int(n), int(s), int(w), int(e)

nextDir = [
    [direction(0, -1), direction(-1, 0), direction(1, 0)],
    [direction(1, 0), direction(0, -1), direction(0, 1)],
    [direction(0, 1), direction(1, 0), direction(-1, 0)],
    [direction(-1, 0), direction(0, 1), direction(0, -1)],
]


openList = []
heading = 0 # heading
            # north = 0
            # west 3
            # south = 2
            # east 1

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

lookDirList = [
    { (0, -1): 0, (1, 0): 2, (0, 1): 3, (-1, 0): 1 },
    { (0, -1): 1, (1, 0): 0, (0, 1): 2, (-1, 0): 3 },
    { (0, -1): 3, (1, 0): 1, (0, 1): 0, (-1, 0): 2 },
    { (0, -1): 2, (1, 0): 3, (0, 1): 1, (-1, 0): 0 }
]
    

def appendList(dir, wall):
    speed_cl.reset()
    global grid, nextNode, openList, closedList
    # dx, dy 초기화
    dx, dy = 0, 0
    tx, ty = -dir.x, -dir.y
    # 다음 노드 계산
    nextNode = Node(nextNode.x + dir.x, nextNode.y + dir.y, wall)
    # 그리드 크기 계산
    new_width = len(grid[0])
    new_height = len(grid)

    # 방향에 따른 그리드 확장 여부 및 nextNode 위치 조정
    if dir.x < 0 and nextNode.x < 0:  # 왼쪽으로 이동
        new_width += 1
        nextNode.x = 0
        dx = 1  # 왼쪽으로 이동 시 첫 번째 열에 위치
    elif dir.x > 0 and nextNode.x >= new_width:  # 오른쪽으로 이동
        new_width += 1
        nextNode.x = new_width - 1
        dx = new_width - 1  # 오른쪽 끝에 위치

    if dir.y < 0 and nextNode.y < 0:  # 위쪽으로 이동
        new_height += 1
        nextNode.y = 0
        dy = 1  # 첫 번째 행에 위치
    elif dir.y > 0 and nextNode.y >= new_height:  # 아래쪽으로 이동
        new_height += 1
        nextNode.y = new_height - 1
        dy = new_height - 1  # 마지막 행에 위치
    
    if dir.x == 0:
        tx = 0
    if dir.y == 0:
        ty = 0

    # 새로운 그리드 생성
    if new_width > len(grid[0]) or new_height > len(grid):
        # 필요할 때만 새로운 그리드를 생성
        tGrid = [[NNode() for _ in range(new_width)] for _ in range(new_height)]
        
        # 기존 그리드 복사
        for y in range(len(grid)):
            for x in range(len(grid[0])):
                tGrid[y][x] = grid[y][x]
    else:
        # 기존 그리드 유지
        tGrid = grid
    print(tGrid, tx, ty)

    for y in range(len(grid)):
        for x in range(len(grid[0])):
            node = grid[y][x]
            node.addXY(dx, dy)
            tGrid[y + dy][x + dx] = node

    # nextNode 삽입
    tGrid[nextNode.y][nextNode.x] = nextNode
    
    # 그리드 갱신
    grid = tGrid
    for e in openList:
        e.addXY(tx, ty)
    for e in closedList:
        e.addXY(dx, dy)
    try:
        closedList.append(closedDIrection(nextNode.x, nextNode.y, dir.ox, dir.oy))
    except:
        closedList.append(direction(nextNode.x, nextNode.y))
    # 확장된 그리드 출력
    for row in tGrid:
        print(row)

def checkDir(dir):
    global nextNode
    if  (dir.x != 0 and dir.y != 0) or not (-1 <= dir.x <= 1) and not (-1 <= dir.y <= 1):
        pid_turn(180, 2250)
        turn_correction()
        li = closedList + []
        li.pop(-1)
        print(li)
        l=1
        while True:
            if (dir.x == 0 or dir.y == 0) and -1 <= dir.x <= 1 and -1 <= dir.y <= 1:
                tDir = li.pop(-l)
                dir = closedDirection(dir.x, dir.y, tDir.x, tDir.y)
                print(111111, li, dir, l)
                break
            
            lastNode = li.pop(-1)
            
            print(lastNode, nextNode)
            dx, dy = lastNode.x - nextNode.x, lastNode.y - nextNode.y
            if (not 1 >= dx >= -1) or (not 1 >= dx >= -1) or (dx != 0 and dy != 0):
                continue
            print(dx, dy)
            i = lookDirList[heading][(dx, dy)]

            if i == 1:
                pid_turn(-90)
            if i == 2:
                pid_turn(90)
            robot.reset()
            ser.clear()
            while True:
                tof = getTOF()
                pid_control(200, 6, 1, 1.7)
                if tof.condition:
                    if tof.t3 != 0 and tof.t3 <= 90:
                        print(tof.t3)
                        break
                if robot.distance() < -295:
                    break
            nextNode.addXY(dx, dy)
            tDir = nextDir[heading][0]
            tx, ty = -tDir.x, -tDir.y
            dir.addXY(tx, ty)
            
    return dir

tof = gTOF()

n, w, e = tof.t3 < 150, tof.t1 < 150, tof.t2 < 150
wall = int(str(int(e)) + str(int(w)) + '1' + str(int(n)), 2)

parent = Node(0, 0, int(str(int(e)) + str(int(w)) + '1' + str(int(n)), 2))

if not e:
    openList.insert(0, nextDir[heading][2])
if not n:
    openList.insert(0, nextDir[heading][0])
if not w:
    openList.insert(0, nextDir[heading][1])
grid = [[parent]]
print(grid)

ser.clear()

gyro.reset_angle(0)
# --------------- variable configuration ---------------
closedList = [direction(0, 0)]
closedDir = [direction(0,0)]
startPos = [0,0]
nextNode = Node(0, 0, wall)
gyroAngle = 0


# if 0 < 0.111 < 1:
#     pid_turn(90)

# while True:
#     pid_turn(-90)
#     wait(799)

# back()
# while True:
for i in range(12):
    dir = openList.pop(0)
    dir = checkDir(dir)
    lookDir = lookDirList[heading][(dir.x, dir.y)]
    if lookDir == 1:
        pid_turn(-90)
    if lookDir == 2:
        pid_turn(90)
    wait(200)
    node(dir)
    print(openList, nextNode)
    print(closedList)
    
quit()

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
# 현재 그리드가 [[14]] 이고 nextNode(0,0,14)이고 dir(0, -1)일때 두 식을 사용해서 값을 출력해봐