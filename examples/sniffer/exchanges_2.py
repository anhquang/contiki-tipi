#!/usr/bin/python
import serial
import sys
ser1 = serial.Serial(
   port='/dev/ttyUSB1',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=0)

while True:
	c = ser1.read()
	for char in c:
		print c
ser1.close()
