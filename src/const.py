
import helpers
import os

APP_CONFIG_PATH= helpers.get_script_path() + '/config.yaml'
APP_INTR_COMM_PFX='/run/user/{}/monovis_intercomm_'.format(str(os.getuid()))

COMM_FRAME_TERMSEQ=b"\n"
COMM_CTRL_SOCK_PFX="ctrlsock"
COMM_CTRL_CMD_BROADCAST_FDS="update_broadcast_fds"


APP_UPDATES_PER_SEC=30

CAMERAS = [0]