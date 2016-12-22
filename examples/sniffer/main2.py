import serial
import argopen
from binascii import unhexlify
from subprocess import Popen, PIPE
import sys
def toHex(s):
    lst = []
    for ch in s:
        hv = hex(ord(ch)).replace('0x', '')
        if len(hv) == 1:
            hv = '0'+hv
        lst.append(hv)
    
    return reduce(lambda x,y:x+y, lst)
ser1 = serial.Serial(
   port='/dev/ttyUSB0',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   xonxoff=False, \
   bytesize=serial.EIGHTBITS,\
   rtscts=False,\
   timeout=0.001)
ser2 = serial.Serial(
   port='/dev/ttyUSB1',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   xonxoff=False, \
   bytesize=serial.EIGHTBITS,\
   rtscts=False,\
   timeout=0.001)
#ser1.flushInput()
#ser1.flushOutput()
#ser2.flushInput()
#ser2.flushOutput()
#p1 = Popen(['python', './ReadCOM.py', "/dev/ttyUSB0", "115200", "0"], stdin=PIPE, stdout=PIPE, stderr=PIPE) # read COM1 permanently
p2 = Popen(['python', './ReadCOM.py', "/dev/ttyUSB1", "115200", "1"], stdin=PIPE, stdout=PIPE, stderr=PIPE) # read COM2 permanently
while True:
    out=p2.stdout.readline()
    if out!='':
	    out="".join(out.split("\n"))
	    ser1.write(out.decode('hex'))   
#    out=p1.stdout.readline()
#    if out!='':
#	    out="".join(out.split("\n"))
#	    ser2.write(out.decode('hex'))
 
