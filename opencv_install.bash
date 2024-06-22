#! /bin/bash

PYTHON_VER="3.12"

WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $WS_DIR
set -ex 

source ./.venv/bin/activate
# pip install numpy
# ./opencv_get_dependencies.bash

cd opencv
mkdir -p build 
cd build

# ../../bin/opencv 

cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_C_COMPILER=gcc-10 \
-D CMAKE_CXX_COMPILER=g++-10 \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_TBB=ON \
-D ENABLE_FAST_MATH=1 \
-D CUDA_FAST_MATH=1 \
-D WITH_CUBLAS=1 \
-D WITH_CUDA=ON \
-D BUILD_opencv_cudacodec=OFF \
-D WITH_CUDNN=OFF \
-D OPENCV_DNN_CUDA=OFF \
-D WITH_V4L=ON \
-D WITH_QT=OFF \
-D WITH_OPENGL=ON \
-D WITH_GSTREAMER=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_PC_FILE_NAME=opencv.pc \
-D OPENCV_ENABLE_NONFREE=ON \
-D PYTHON_INCLUDE_DIR=$(python -c "import sysconfig; print(sysconfig.get_path('include'))")  \
-D PYTHON_LIBRARY=$(python -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))") \
-D OPENCV_PYTHON3_INSTALL_PATH=$(python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])") \
-D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D BUILD_EXAMPLES=OFF ..


# specify cuda version if there are any cuda error, use the newest version avaliabe on platform
# cuDNN also just breaks all the c++ code for some reason
# gcc and g++ must be version 11 or lower due to gnu bug on syntax 

# cmake -D CMAKE_BUILD_TYPE=RELEASE \
# -D CMAKE_INSTALL_PREFIX=../../bin/opencv \
# -D WITH_TBB=ON \
# -D ENABLE_FAST_MATH=1 \
# -D CUDA_FAST_MATH=1 \
# -D WITH_CUBLAS=1 \
# -D WITH_CUDA=ON \
# -D BUILD_opencv_cudacodec=ON \
# -D WITH_CUDNN=ON \
# -D OPENCV_DNN_CUDA=ON \
# # -D CUDA_ARCH_BIN=7.5 \
# -D WITH_V4L=ON \
# -D WITH_QT=OFF \
# -D WITH_OPENGL=ON \
# -D WITH_GSTREAMER=ON \
# -D OPENCV_GENERATE_PKGCONFIG=ON \
# -D OPENCV_PC_FILE_NAME=opencv.pc \
# -D OPENCV_ENABLE_NONFREE=ON \
# -D OPENCV_PYTHON3_INSTALL_PATH=../../.venv/lib/python3.10/site-packages \
# -D PYTHON_EXECUTABLE=../../.venv/bin/python3 \
# -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
# -D INSTALL_PYTHON_EXAMPLES=OFF \
# -D INSTALL_C_EXAMPLES=OFF \
# -D BUILD_EXAMPLES=OFF ..

if [[ $1 = "build" ]]; then
    make "-j$(nproc)"
    exit
fi

if [[ $1 = "install" ]]; then
    make "-j$(nproc)"
    make install "-j$(nproc)"
fi
