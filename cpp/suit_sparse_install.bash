#! /bin/bash

set -x 
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $SCRIPT_DIR

echo "installing SuiteSparse package dependencies"
sudo apt install libmpfr6 libmpfrc++-dev libgmp-dev -y

if [[ -z $(pkg-config --list-all | grep blas) ]]; then 
    echo "Please ensure you have a BLAS provider installed that is findable by pkg-config/cmake, Installation canceled"
    exit 1 
fi;

if [[ -z $(pkg-config --list-all | grep lapack) ]]; then 
    echo "Please ensure you have a LAPACK provider installed that is findable by pkg-config/cmake, Installation canceled"
    exit 1 
fi;

echo "cloning SuiteSparse stable branch"
git clone https://github.com/DrTimothyAldenDavis/SuiteSparse.git -b stable

cd SuiteSparse

echo "configuring cmake"
mkdir -p build && cd build
cmake ..

echo "building"
cmake --build . --config Release -j $(nproc)

echo "installing"
sudo cmake --install .

echo "cleaning up"
cd ..
cd ..
rm -r SuiteSparse 



