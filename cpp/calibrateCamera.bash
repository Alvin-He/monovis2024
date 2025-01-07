#! /bin/bash 

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

project_xmlFile="$SCRIPT_DIR/calibrationAdvancedParams.xml"
output_xmlFile="$SCRIPT_DIR/calibrationResults.xml"

cam_id=2
camera_flip="false"

board_type="charuco"
#"chessboard"
board_dictionary="DICT_5X5_250"
block_length=1 # length of one block on pattern
block_width=7
board_height=8
#9
board_width=11
#6


capture_delay=1

    # --dst=$block_length \
#    -h=$board_height \
#    -w=$board_width
#    --sz=$block_width \

#opencv_interactive-calibration \
exec "$SCRIPT_DIR/build/artifacts/calib" \
    --ad=$board_dictionary \
    --ci=$cam_id \
    -d=$capture_delay \
    --flip=$camera_flip \
    --pf=$project_xmlFile \
    --of=$output_xmlFile \
    -t=$board_type \
    -w=$board_width \
    -h=$board_height \
    # --sz="20" \


