#!/usr/bin/python
import serial
import struct
import termios
from collections import deque 
MAX_MTU = 200
ser1 = serial.Serial(
   port='/dev/ttyUSB0',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=0)
ser2 = serial.Serial(
   port='/dev/ttyUSB1',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=0)
readBufferQueue = deque([])  
def writeToSerialPort(serialFD, byteArray):  
#    byteString = ''.join(chr(b) for b in byteArray) #convert byte list to a string  
    serialFD.write(str(byteArray).encode())  
    return  
def getSerialByte(serialFD):       
     if len(readBufferQueue) == 0:  
      #fetch a new data chunk from the serial port       
     	while len(readBufferQueue) < MAX_MTU: 
		c=serialFD.read()
		if c:
	       		newByte = ord(c)  
		        readBufferQueue.append(newByte)  
       	newByte = readBufferQueue.popleft()  
	return newByte  
     else:  
        newByte = readBufferQueue.popleft()  
        return newByte  
def disconnectFromSerialPort(serialFD):  
     serialFD.close()  
     return  

while 1:
	b1=None
	b2=None
	b1=getSerialByte(ser1)
	if b1 is not None:
	  writeToSerialPort(ser2,b1)
	b2=getSerialByte(ser2)
	if b2 is not None:
	  writeToSerialPort(ser1,b2)

disconnectFromSerialPort(ser1)
disconnectFromSerialPort(ser2)
#s_end= "\xC0"
#s_esc= "\xDB"
#s_esc_end= "\xDC"
#s_esc_esc= "\xDD"
#ser1.write(s_end)
#ser1.write("SNIF".encode())
#ser1.write(s_esc)
#ser1.write(s_esc_end)
#ser1.write(s_esc)
#ser1.write(s_esc_esc)
#ser1.write(s_end)
#ser1.close()
