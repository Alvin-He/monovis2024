### Pyhton search path set up
import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(os.path.realpath(sys.argv[0]))+"/.."))


import argparse
import socket
import numpy as np
import io

import const
import cv2

size = (640,480)
bufSize = size[0]*size[1]

stdin = io.BufferedIOBase(sys.stdin.buffer)
buf = bytearray(bufSize)
while True:   
    stdin.readinto(buf)
    if len(buf) != bufSize: continue
    image = np.frombuffer(buf, dtype=np.uint8)
    buf.clear()
    cv2.imshow("got", image)
    cv2.waitKey(1)