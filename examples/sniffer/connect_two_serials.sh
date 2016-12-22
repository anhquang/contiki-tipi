python flush.py /dev/ttyUSB0
python flush.py /dev/ttyUSB1
socat /dev/ttyUSB0,raw,echo=0,crnl /dev/ttyUSB1,raw,echo=0,crnl
