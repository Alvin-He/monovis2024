#! /usr/bin/bash

if [[ $UID -ne 0 ]]; then 
    echo "This script must be ran with root"
    exit 1
fi

sudo apt install wget tar -y
sudo apt install build-essential cmake pkg-config unzip yasm git checkinstall -y
sudo apt install libjpeg-dev libpng-dev libtiff-dev -y 
sudo apt install libavcodec-dev libavformat-dev libswscale-dev -y 
sudo apt install libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev -y

sudo apt-get install libxine2-dev libv4l-dev v4l-utils -y 

sudo apt-get install libgtk-3-dev -y 
sudo apt-get install libtbb-dev -y
sudo apt-get install libatlas-base-dev gfortran -y 
sudo apt-get install libeigen3-dev -y 