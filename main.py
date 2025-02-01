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
g1 = GyroSensor(Port.S1)
g2 = GyroSensor(Port.S4)
rm = Motor(Port.C,gears=[12,20])
lm = Motor(Port.B, gears=[12,20])
robot = DriveBase(lm, rm, wheel_diameter=75.1, axle_track=120)
gyro = GyroSensor(Port.S4)
# us = UltrasonicSensor(Port.S2)
cs = ColorSensor(Port.S2)
azimuth = namedtuple('azimuth', ['n', 's', 'w', 'e'])
nodeData = namedtuple('nodeData', ['x', 'y', ''])
# Initialize the EV3
ev3 = EV3Brick()
turn_cl = StopWatch()
speed_cl = StopWatch()

def check_drift():
    global gyro
    
    if robot.state()[1] == 0:
        print("checking drift")
        g1a = g1.angle()
        g2a = g2.angle()
        wait(100)
        if g1a == g1.angle():
            gyro = g1
        elif g2a == g2.angle():
            gyro = g2
        else:
            g1.reset_angle(gyroAngle)
            g2.reset_angle(gyroAngle)
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
    yaw = 0 + gyro.angle() + gyroAngle
    differential = yaw - lastYaw
    lastYaw = yaw
    # 적분 상수를 0으로 입력할 경우
    steering = (kp * yaw + kd * differential) * gain
    
    robot.drive(-speed, steering)

lastTurnYaw = 0
def pid_turn_control(dest, gain, kp, kd):
    global lastTurnYaw, gyroAngle
    yaw = dest+gyro.angle() + gyroAngle
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
    global heading, lastYaw, gyroAngle
    i = 0
    turn_cl.reset()

    while turn_cl.time() < time:
        pid_turn_control(dest - int(dest / 90), 6.3, 1.3, 2)
        if gyro.angle() == gyroAngle and i == 0:
            turn_cl.reset()
            i = 1
    print(gyroAngle, gyro.angle())
    lastYaw = 0
    gyroAngle += dest
    heading += int(dest / 90)
    checkHeading()
    robot.stop(Stop.BRAKE)
    check_drift()
        
def back():
    pid_turn(180, 2250)

    ser.clear()

    wait(30)
    if gyro.angle() != 0:
        turn_correction()
    robot.reset()

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
    # 거리값 리셋
    robot.reset()
    # print("heading : ",robot.distance(), heading)
    # print(openList)

    ser.clear()
    while True:
        tof = getTOF()
        pid_control(250, 8, 1, 1.7)
        if tof.condition:
            if tof.t3 != 0 and tof.t3 <= 100:
                robot.stop(Stop.BRAKE)
                break
        if robot.distance() < -305:
            break
    # print(tof)
    robot.stop(Stop.BRAKE)
    tof = gTOF()
    on, ow, oe = tof.t3 <= 200, tof.t2 <= 170, tof.t1 <= 170
    if on == 1 and ow == 1 and oe == 1:
        wait(100)
        tof = gTOF()
        on, ow, oe = tof.t3 <= 200, tof.t2 <= 170, tof.t1 <= 170
    n, s, w, e = correctAzimuth(int(on), int(ow), int(oe))
    wall = int(str(e) + str(w) + str(s) +  str(n), 2)
    print(wall, n, w, e, dir, on, ow, oe)
    n, w, e = appendList(dir, wall)

    print(n, w, e)
    if not oe and e[0]:
        openList.insert(0, e[1])
    if not on and n[0]:
        openList.insert(0, n[1])
    if not ow and w[0]:
        openList.insert(0, w[1])
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
        dx = 0  # 오른쪽 끝에 위치

    if dir.y < 0 and nextNode.y < 0:  # 위쪽으로 이동
        new_height += 1
        nextNode.y = 0
        dy = 1  # 첫 번째 행에 위치
    elif dir.y > 0 and nextNode.y >= new_height:  # 아래쪽으로 이동
        new_height += 1
        nextNode.y = new_height - 1
        dy = 0  # 마지막 행에 위치

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
    print(tGrid, tx, ty, dx, dy)

    for y in range(len(grid)):
        for x in range(len(grid[0])):
            node = grid[y][x]
            node.addXY(dx, dy)
            tGrid[y + dy][x + dx] = node

    # nextNode 삽입
    tGrid[nextNode.y][nextNode.x] = nextNode
    n, w, e = nextDir[heading][0].copy(), nextDir[heading][1].copy(), nextDir[heading][2].copy()
    nc, wc, ec = 1, 1, 1
    nn = 'dir({},{})'.format(nextNode.x + n.x, nextNode.y + n.y)
    ww = 'dir({},{})'.format(nextNode.x + w.x, nextNode.y + w.y)
    ee = 'dir({},{})'.format(nextNode.x + e.x, nextNode.y + e.y)
    # 그리드 갱신
    grid = tGrid
    for ele in openList:
        ele.addXY(tx, ty)
    for ele in closedList:
        ele.addXY(dx, dy)
        if ele.getStr() == nn:
            nc = 0
        if ele.getStr() == ww:
            wc = 0
        if ele.getStr() == ee:
            ec = 0
    try:
        closedList.append(closedDIrection(nextNode.x, nextNode.y, dir.ox, dir.oy))
    except:
        closedList.append(direction(nextNode.x, nextNode.y))

    return (nc, n), (wc, w), (ec, e)

def checkDir(dir):
    global nextNode
    # 만약
    if  (dir.x != 0 and dir.y != 0) or (not -1 <= dir.x <= 1) or (not -1 <= dir.y <= 1):
        pid_turn(180, 2250)
        turn_correction()
        li = closedList + []
        li.pop(-1)
        print(dir, li)
        l = 0
        oDir = direction(dir.x, dir.y)
        while True:

            if (dir.x == 0 or dir.y == 0) and -1 <= dir.x <= 1 and -1 <= dir.y <= 1 :
                tDir = li.pop(-l)
                dir = closedDirection(dir.x, dir.y, tDir.x, tDir.y)
                
                print(111111, li, dir, l)
                break
            
            lastNode = li.pop(-1)
            
            print(lastNode, nextNode, dir)
            dx, dy = lastNode.x - nextNode.x, lastNode.y - nextNode.y
            if (not 1 >= dx >= -1) or (not 1 >= dy >= -1) or (dx != 0 and dy != 0):
                continue
            print(dx, dy)
            i = lookDirList[heading][(dx, dy)]
            if i != 0:
                robot.stop(Stop.BRAKE)
            if i == 1:
                pid_turn(-90)
            if i == 2:
                pid_turn(90)
            robot.reset()
            ser.clear()
            while True:
                tof = getTOF()
                pid_control(250, 6, 1, 1.7)
                if tof.condition:
                    if tof.t3 != 0 and tof.t3 <= 100:
                        print(tof.t3)
                        robot.stop(Stop.BRAKE)
                        break
                if robot.distance() < -300:
                    break
            robot.stop()
            nextNode.addXY(dx, dy)
            tDir = nextDir[heading][0]
            tx, ty = -tDir.x, -tDir.y
            dir.addXY(tx, ty)
            for e in openList:
                e.addXY(tx, ty)

    return dir

def checkList(dir):
    result = False
    for e in closedList:
        if (e.x, e.y) == (dir.x, dir.y):
            result = True
            break
    return result

colors=[Color.RED, Color.GREEN, Color.BLUE]
def colorCheck():
    color = cs.color()
    if color in colors:
        if color in colors[0:2]:
            ev3.speaker.beep()
            wait(3000)
            ev3.speaker.beep()
        if color == colors[2]:
            ev3.speaker.beep()
            wait(5000)
            ev3.speaker.beep()


tof = gTOF()

n, w, e = tof.t3 < 150, tof.t1 < 150, tof.t2 < 150
wall = int(str(int(e)) + str(int(w)) + '1' + str(int(n)), 2)

parent = Node(0, 0, int(str(int(e)) + str(int(w)) + '1' + str(int(n)), 2))

if not e:
    openList.insert(0, nextDir[heading][2].copy())
if not n:
    openList.insert(0, nextDir[heading][0].copy())
if not w:
    openList.insert(0, nextDir[heading][1].copy())
grid = [[parent]]
print(grid)

ser.clear()

g1.reset_angle(0)
g2.reset_angle(0)
# --------------- variable configuration ---------------
closedList = [direction(0, 0)]
closedDir = [direction(0, 0)]
startPos = [0,0]
nextNode = Node(0, 0, wall)
gyroAngle = 0


# if 0 < 0.111 < 1:
#     pid_turn(90)

# while True:
#     pid_turn(-90)
#     wait(799)

# back()
while True:
    if len(openList) == 0:
        break
    dir = openList.pop(0)
    dir = checkDir(dir)
    lookDir = lookDirList[heading][(dir.x, dir.y)]
    if lookDir == 1:
        pid_turn(-90)
    if lookDir == 2:
        pid_turn(90)
    node(dir)
    colorCheck()
    print(openList, nextNode, lookDir)
    print(closedList)

for e in grid:
    print(e)
quit()
