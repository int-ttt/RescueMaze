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

dirTuple = namedtuple('dirTuple', ['x','y'])

class Node:
    def __init__(self, x, y, wall):
        self.x = x
        self.y = y
        self.wall = wall

    def __repr__(self):
        return 'Node(x={}, y={}, wall={})'.format(self.x, self.y, self.wall)



    def setXY(self, x, y):
        self.x = x;
        self.y = y;

    def addXY(self, x, y):
        self.x += x
        self.y += y
    
class NNode(Node):
    def __init__(self):
        super().__init__(0, 0, -1)

class direction:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return 'direction(x={},y={})'.format(self.x, self.y)

    
    def addXY(self, x, y):
        self.x += x
        self.y += y
    
    def copy(self):
        return direction(self.x, self.y)
    
    def equals(self, dir):
        return self.x == dir.x and self.y == dir.y
    
    def getStr(self):
        return 'dir({},{})'.format(self.x, self.y)

class closedDirection:
    def __init__(self, x, y, ox, oy):
        self.x = x
        self.y = y
        self.ox = ox
        self.oy = oy

    def __repr__(self):
        return 'direction(x={},y={},ox={},oy={})'.format(self.x, self.y, self.ox, self.oy)
    
    def addXY(self, x, y):
        self.x += x
        self.y += y
        self.ox += x
        self.oy += y
    
    def equals(self, dir):
        return self.x == dir.x and self.y == dir.y
    
    def getStr(self):
        return 'dir({},{})'.format(self.x, self.y)