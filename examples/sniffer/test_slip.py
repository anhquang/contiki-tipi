#!/usr/bin/python
import serial
ser1 = serial.Serial(
   port='/dev/ttyUSB0',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=0)
s_end= "\xC0"
s_esc= "\xDB"
s_esc_end= "\xDC"
s_esc_esc= "\xDD"
ser1.write("SNIF".encode())
ser1.write(s_end)
magic="\xC1\x1F\xFE\x72"
data="\x61\xCC\x20\xCD\xAB\xDB\xDD\xD8\x65\x14\x00\x74\x12\x00\x21\xF1\x6E\x14\x00\x74\x12\x00\x7E\xF6\x00\x00\x01\xF0\x22\x3D\x16\x2E\x8C\x63\x48\x65\x6C\x6C\x6F\x20\x32\x30\x20\x66\x72\x6F\x6D\x20\x74\x68\x65\x20\x63\x6C\x69\x65\x6E\x74"
ser1.write(magic)
ser1.write(data)
ser1.write(s_end)
ser1.close()