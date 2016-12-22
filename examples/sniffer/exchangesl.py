#!/usr/bin/python
import serial
ser1 = serial.Serial(
   port='/dev/ttyUSB1',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=0)
ser2 = serial.Serial(
   port='/dev/ttyUSB2',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=0)
print("connected to: " + ser1.portstr)
ser1.write("help\n");
while True:
	for c in ser1.read():
	   if c:
	     ser2.write(c)
ser1.close()
ser2.close()
