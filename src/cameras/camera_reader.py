### Pyhton search path set up
import sys
import os

# import twisted.internet
# import twisted.internet.endpoints
# import twisted.internet.interfaces
# import twisted.internet.protocol
# from twisted.internet.protocol import connectionDone
# import twisted.internet.task
# import twisted.protocols
# from twisted.python.failure import Failure
sys.path.append(os.path.abspath(os.path.dirname(os.path.realpath(sys.argv[0]))+"/.."))

import cv2

# import twisted
# from twisted.internet import reactor
import time
import const
import helpers
import numpy as np
import asyncio
import logging
logging.basicConfig(level=logging.DEBUG)

CAM_ID = 0
TRANSPORT_WIDTH = 640
TRANSPORT_HEIGHT = 480

cap = cv2.VideoCapture(CAM_ID)

def getImage():
    ret, image = cap.read()
    if not ret: return 

    #resize 
    cuda_buf = cv2.cuda.GpuMat()
    cuda_buf.upload(image)
    resized_buf = cv2.cuda.resize(cuda_buf, (TRANSPORT_WIDTH, TRANSPORT_HEIGHT), interpolation=cv2.INTER_LINEAR)

    image = resized_buf.download()
    
    cv2.imshow("cam proced", image)
    cv2.waitKey(1)
    return image


class UDPCameraServerProtocol(asyncio.DatagramProtocol):
    def __init__(self) -> None:
        super().__init__()
    
    def connection_made(self, transport: asyncio.DatagramTransport) -> None:
        frame = getImage()
        transport.write(frame.astype(np.uint8).tobytes())
    


# |-HEADER-									  -|-BODY-									-|    
# 1byte			2bytes			2byte		   	xbit
# camId		 	frame width	 	frame height	frame data(width*height amount of bits)
# uint8			uint16			uint16		  	x of uint8
headerBytes = np.uint8(CAM_ID).tobytes() + np.uint16(TRANSPORT_WIDTH).tobytes() + np.uint16(TRANSPORT_HEIGHT).tobytes()
async def connection_handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    writer.transport.set_write_buffer_limits(0,0)
    # current_frame = 0
    # while True: 
    #     command = (await reader.readline()).decode()
    #     if command == "sync":            
    #         frame = getImage()
    #         writer.write(frame.astype(np.uint8).tobytes())
    #         writer.drain()
    #         current_frame += 1
    #         print(current_frame)
    #     elif command == "info": 
    #         writer.writelines(
    #             [f"CamID:{CAM_ID}", f"FrameWidth:{TRANSPORT_WIDTH}", f"FrameHeight:{TRANSPORT_HEIGHT}"]
    #         )
    
    
    # send headers 
    writer.write(headerBytes)
    await writer.drain()

    time_start=time.time()
    current_frame = 0
    while True: 
        
        cur_time = time.time()
        frame = getImage()
        print(cur_time)
        writer.write(frame.astype(np.uint8).tobytes()+np.double(cur_time).tobytes())
        await writer.drain()
        
        
        # print(f"TOOK {cur_time - time_start}")
        # delta_left = (time_start + 1/const.APP_UPDATES_PER_SEC) - cur_time

        # if delta_left < 0:
            # delta_left = 0

        time_start = cur_time
        # print(delta_left)
        # await asyncio.sleep(delta_left)
        current_frame += 1
        

async def main(): 
    print("Programing starting")
    reader, writer = await asyncio.open_connection("127.0.0.50", "8080", limit=1) #50mb buffer limit
    await connection_handler(reader, writer)
    
    
if __name__ == "__main__": 
    asyncio.run(main())