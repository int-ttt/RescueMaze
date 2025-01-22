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

class Node:
    def __init__(self, x, y, wall):
        self.x = x
        self.y = y
        self.wall = wall

    def __repr__(self):
        return 'Node({},{},{})'.format(self.x, self.y, self.wall)
    
    def setXY(self, x, y):
        self.x = x;
        self.y = y;

    def addXY(self, x, y):
        self.x += x
        self.y += y

class NNode(Node):
    def __init__(self):
        super().__init__(0, 0, -1)