import sys
import serial
import argopen
import time
from binascii import unhexlify
#import myqueues
ser = serial.Serial(
   port=sys.argv[1],\
   baudrate=int(sys.argv[2]),\
#   parity=serial.PARITY_NONE,\
#   stopbits=serial.STOPBITS_ONE,\
#   bytesize=serial.EIGHTBITS,\
#   xonxoff=False,\
#   rtscts=False,\
   timeout=0.001)
#serout = serial.Serial(
#   port=sys.argv[2],\
#   baudrate=int(sys.argv[3]),\
#   parity=serial.PARITY_NONE,\
#   stopbits=serial.STOPBITS_ONE,\
#   bytesize=serial.EIGHTBITS,\
#   xonxoff=False,\
#   rtscts=False,\
#   timeout=0.001)
usleep = lambda x: time.sleep(x/1000000.0)
if ser.isOpen():
	try:
		ser.flushInput()
		ser.flushOutput()
	except Exception, e1:
        	print "error communicating...: " + str(e1)
inputstr=sys.argv[3]
ser.write(inputstr.decode('hex'))
