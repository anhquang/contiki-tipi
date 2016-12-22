from collections import deque 

readBufferQueue1 = deque([])  
readBufferQueue2 = deque([])  
def getByteFromQueue(queueid):
	if (queueid==0):
		if (len(readBufferQueue1)>0):
			return readBufferQueue1.popleft()
		else:
			return None
	if (queueid==1):
		if (len(readBufferQueue2)>0):
			return readBufferQueue2.popleft()
		else:
			return None
def appendBytetoQueue(queueid,newbyte):
	print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
	if (queueid==0):
		readBufferQueue1.append(newbyte)
	if (queueid==1):
		readBufferQueue2.append(newbyte)
	print(len(readBufferQueue[queueid]))
