#! /bin/bash

set -x 
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $SCRIPT_DIR

echo "installing dependencies"
sudo apt install libgoogle-glog-dev -y 

echo "checking ceres-solver package dependencies"
if [[ -z $(pkg-config --list-all | grep eigen3) ]]; then 
    echo "Please ensure you have a Eigen3 installed that is findable by pkg-config/cmake, Installation canceled"
    exit 1 
fi;
if [[ -z $(pkg-config --list-all | grep blas) ]]; then 
    echo "Please ensure you have a BLAS provider installed that is findable by pkg-config/cmake, Installation canceled"
    exit 1 
fi;

if [[ -z $(pkg-config --list-all | grep lapack) ]]; then 
    echo "Please ensure you have a LAPACK provider installed that is findable by pkg-config/cmake, Installation canceled"
    exit 1 
fi;

USE_CUDA="ON"
if [[ -z $(pkg-config --list-all | grep cuda) ]]; then 
    echo "No CUDA detected, not building with Nvidia CUDA"
    USE_CUDA="OFF"
fi;

git clone https://ceres-solver.googlesource.com/ceres-solver -b 2.2.0


cd ceres-solver

echo "configuring cmake"
mkdir -p build && cd build
cmake .. -DUSE_CUDA=$USE_CUDA

echo "building"
cmake --build . --config Release -j $(nproc)

echo "installing"
sudo cmake --install .

echo "cleaning up"
cd ..
cd ..
rm -r ceres-solver 