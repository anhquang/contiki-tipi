import SerialComm

sp=SerialComm.connectToSerialPort()
while True:
	byteArry=SerialComm.readFromSerialPort(sp)
	for char in byteArry:
		print chr(char)
