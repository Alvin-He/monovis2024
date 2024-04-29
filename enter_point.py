### Pyhton search path set up
import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(os.path.realpath(sys.argv[0]))+"/src"))

import yaml
import os
import sys
import io
import socket

import subprocess

import helpers as helpers
from const import *

# note: default to udp cause most likely this func will be used to create sockets for camera data
def createSocket(name, type=socket.SOCK_STREAM):
    socket_path = APP_INTR_COMM_PREFIX + str(name)

    # remove the socket file if it already exists
    try:
        os.unlink(socket_path)
    except OSError:
        if os.path.exists(socket_path):
            raise
    
    server = socket.socket(socket.AF_UNIX, type)
    server.bind(socket_path)
    return server


if __name__ == "__main__": 
    print("Booting Mono Vis, @C 2024 Alvin-He and authors") 

    print("Parsing config file")
    config = helpers.parse_config(APP_CONFIG_PATH)

    #create pipes that will be used to pass processes 
    for id in CAMERAS: 
        rootPath = helpers.get_script_path()

        control_server = createSocket(COMM_CTRL_SOCK_PFX + str(0))
        control_server.listen(1)

        stream = subprocess.Popen(["python3", rootPath + "/src/comms/stream.py"])
        camera = subprocess.Popen(["python3", rootPath + "/src/cameras/camera_reader.py"])

        control_server.sendmsg([COMM_CTRL_CMD_BROADCAST_FDS], [(socket.SOL_SOCKET, socket.SCM_RIGHTS, [stream, camera])])
        
