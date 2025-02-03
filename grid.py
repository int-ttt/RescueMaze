from ucollections import namedtuple

dirTuple = namedtuple('dirTuple', ['x','y'])

class Node:
    def __init__(self, x, y, wall):
        self.x = x
        self.y = y
        self.wall = wall
        self.color = 0 # color : red; 1 green; 2 blue; 3

    def __repr__(self):
        return 'Node(x={}, y={}, wall={})'.format(self.x, self.y, self.wall)
    
    def setColor(self, color):
        self.color = color

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