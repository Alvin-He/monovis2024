# vision-server
 
AprilTag pose estimation solution meant to run on an NVIDIA Jetson Nano.

### CLI Arguments
 ```bash
    -s --server <ipaddress:port>  Network Tables server ip address
    -c --cameras <path> Path to the cameras file to load, default $PWD/cameras.py
    --nt3 <bool/True> communicating to a nt3 client or not
    -d --debug <bool/False> Weather to enable debug
     
 ```