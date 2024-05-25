#! /bin/bash

OPENCV_VERSION=4.x

WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $WS_DIR
set -ex 

source ./.venv/bin/activate

echo "downloading opencv version $OPENCV_VERSION to ./opencv"
wget -O opencv-$OPENCV_VERSION.zip https://github.com/opencv/opencv/archive/refs/heads/$OPENCV_VERSION.zip
unzip opencv-$OPENCV_VERSION.zip
mv ./opencv-$OPENCV_VERSION/ ./opencv/

cd ./opencv
echo "downloading opencv contrib version $OPENCV_VERSION to ./opencv/opencv_contrib"
wget -O opencv_contrib-$OPENCV_VERSION.zip https://github.com/opencv/opencv_contrib/archive/refs/heads/$OPENCV_VERSION.zip
unzip opencv_contrib-$OPENCV_VERSION.zip
mv ./opencv_contrib-$OPENCV_VERSION/ ./opencv_contrib/

echo "cleaning up archives"
rm ../opencv-$OPENCV_VERSION.zip
rm ./opencv_contrib-$OPENCV_VERSION.zip

