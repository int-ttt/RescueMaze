from ucollections import namedtuple
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, DataLog
import struct


ser = UARTDevice(Port.S3, 115200)
ser.clear()
wait(100)

TOFData = namedtuple("TOFData", ["condition", "t1", "t2", "t3", "t4"])
null_tof = TOFData(False, 0, 0, 0, 0)

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
    ser.clear()
    while True:
        tof = getTOF()
        if tof.condition:
            break
    print("gTOF_debug : {}".format(tof))
    return tof