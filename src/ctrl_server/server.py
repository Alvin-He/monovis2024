# import twisted.internet.endpoints
# import twisted.internet.protocol
# from twisted.internet.protocol import connectionDone
# import twisted.internet.reactor
# import twisted.internet.tcp
# import twisted.internet.udp
# import twisted.protocols
# from twisted.python.failure import Failure
# from twisted.internet import reactor

import collections
import numpy as np
import cv2
import time

import asyncio
import logging
logging.basicConfig(level=logging.DEBUG)

def debug_showImg(data_frame:bytes, width:int, height:int):
    npImage = np.frombuffer(data_frame, dtype=np.uint8).reshape((height, width, 3))
    cv2.imshow("servergot", npImage)
    cv2.waitKey(1)
    

class camera_data_frames():
	def __init__(self, camId, maxBufCount = 5) -> None:
		self.deque = collections.deque(maxlen=maxBufCount)
		self.id = camId

	def get(self, frameNum=0):
		return self.deque[frameNum]
	
	def add(self, data:bytes):
		return self.deque.append(data)
		

class camera_transport_proto(): 
	def __init__(self, transport) -> None:
		self.transport = transport
		self.dataframes: camera_data_frames
		self.lastMessageBytePosition = 0
		self.inFrameTransport = False
		self.frameBuf = bytes()
		
		self.camID = np.uint8(np.iinfo(np.uint8).max)
		self.frameSize = np.array([np.uint16(np.iinfo(np.uint16).max), np.uint16(np.iinfo(np.uint16).max)]).astype(int)
		self.expectedNumOfDataBytes = 0
		self.initialized = False

		self.prevTime = time.time()

	def dataReceived(self, data: bytes) -> None:
		startTime = time.time()
		if len(data) == 0: return
		# print(f"data recevied:{data!r}")
		# |-HEADER-									  -|-BODY-									-|    
		# 1byte			2bytes			2byte		   	xbit
		# camId		 	frame width	 	frame height	frame data(width*height amount of bits)
		# uint8			uint16			uint16		  	x of uint8
  
		# branch for handling chunked data inflow
		if self.inFrameTransport:
			numBytesWanted = self.expectedNumOfDataBytes - len(self.frameBuf)
			usuableFrameDataLength = len(data)
			if usuableFrameDataLength <= numBytesWanted:
				self.frameBuf += data
			else: 
				self.frameBuf += data[:numBytesWanted]
				self.dataframes.add(self.frameBuf)
				self.inFrameTransport = False	
				print(f"Got frame in {time.time() - self.prevTime}")
				self.prevTime = time.time()
				debug_showImg(self.dataframes.get(0), self.frameSize[0][0], self.frameSize[1][0])

				self.frameBuf = bytes()
				overflow = data[numBytesWanted:]
				self.dataReceived(overflow)
			timeUsed = time.time() -startTime
			print(timeUsed)
			return
   
		# new frame handling
		camID = np.frombuffer(data[0:1], dtype=np.uint8)
		frameSize = np.array([np.frombuffer(data[1:3], dtype=np.uint16), np.frombuffer(data[3:5], dtype=np.uint16)]).astype(int)
		if not self.initialized: # initialize the camera params to ensure data correctness for future requests
			self.camID = camID
			self.frameSize = frameSize
			self.expectedNumOfDataBytes = (frameSize[0] * frameSize[1])[0] * 3 #convert the single num array to int, the extra mutiply by three accounts for the 3 color channels
			self.initialized = True
			self.dataframes = camera_data_frames(camID)
		# check data correctness 
		if not self.camID == camID or not np.array_equal(self.frameSize, frameSize): 
			print("ERROR: 4669400, Bad message start, incorrect camera params when compared to previous frames")
			self.transport.abort()
			return 
		self.inFrameTransport = True

		numBytesWanted = self.expectedNumOfDataBytes - len(self.frameBuf)
		usuableFrameDataLength = len(data) - 5
		if usuableFrameDataLength <= numBytesWanted:
			self.frameBuf += data[5:]
		else: 
			self.frameBuf += data[5:numBytesWanted]
			self.dataframes.add(self.frameBuf)
			self.inFrameTransport = False
			
			print(f"Got frame in {time.time() - self.prevTime}")
			self.prevTime = time.time()
			debug_showImg(self.dataframes.get(0), self.frameSize[0][0], self.frameSize[1][0])
   
			self.frameBuf = bytes()
			overflow = data[numBytesWanted:]
			self.dataReceived(overflow)
		timeUsed = time.time() -startTime
		print(timeUsed)

exampleHeader = np.uint8(0).tobytes() + np.uint16(0).tobytes() + np.uint16(0).tobytes()
async def connection_handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    # header parsing
	header = await reader.readexactly(len(exampleHeader))
	
	camID = np.frombuffer(header[0:1], dtype=np.uint8)
	frameSize = np.array([np.frombuffer(header[1:3], dtype=np.uint16), np.frombuffer(header[3:5], dtype=np.uint16)]).astype(int)
	expectedNumOfDataBytes = 8 + (frameSize[0] * frameSize[1])[0] * 3 #convert the single num array to int, the extra mutiply by three accounts for the 3 color channels
	dataframes = camera_data_frames(camID)

	# writer.writelines(["info"])
	# await writer.drain()

	currentFrame = 0
	lastTime = 0
	# repeately recevive frames
	while True: 
		startTime = time.time()
  
		data = await reader.readexactly(expectedNumOfDataBytes)

		dataframes.add(data[:-8])
		# newTime = float(np.frombuffer(data[-8:], dtype=np.double))
		# print(newTime-lastTime)
		# lastTime = newTime
		# debug_showImg(dataframes.get(0), frameSize[0][0], frameSize[1][0])
  
		timeUsed = time.time() - startTime
		currentFrame += 1
		print(timeUsed)
	

async def main(): 
    print("Programing starting")
    server = await asyncio.start_server(connection_handler, "127.0.0.50", "8080", limit=1) #50mb buffer limit
    async with server:
        await server.serve_forever()
    
    
if __name__ == "__main__": 
    asyncio.run(main())