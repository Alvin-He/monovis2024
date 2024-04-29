### Pyhton search path set up
import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(os.path.realpath(sys.argv[0]))+"/.."))

## This file defines the interval that all the rest of the programs run on
## bascially process synchronization

import time
import const

def update():
    print("update")

time_start=time.time()
while True:
    cur_time = time.time()
    update()
    delta_left = (time_start + 1/const.APP_UPDATES_PER_SEC) - cur_time

    if delta_left < 0:
        delta_left = 0

    time_start =  cur_time
    time.sleep(delta_left)