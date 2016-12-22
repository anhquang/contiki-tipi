#!/usr/bin/python
import serial
import sys
import argopen
myport=sys.argv[1]
#print sys.argv[1]
ser = serial.Serial(
   port=myport,\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
  timeout=0)
#ser1 = serial.Serial(
#   port='/dev/ttyUSB0',\
#   baudrate=115200,\
#   parity=serial.PARITY_NONE,\
#   stopbits=serial.STOPBITS_ONE,\
#   bytesize=serial.EIGHTBITS,\
#  timeout=0)
#ser.close()
#ser.open()
#ser1.close()
#ser1.open()
stdout=argopen.argopen('-','wb')
stdout.flush()
ser.flushInput()
ser.flushOutput()
#ser1.flushInput()
#ser1.flushOutput()
#ser.close()
#ser1.close()
