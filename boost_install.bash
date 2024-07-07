#! /bin/bash

BOOST_HEADER_DIR="./boost_1_85_0"

WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $WS_DIR
set -ex 

cd ./cpp
mkdir -p include
mkdir -p libs

cd $BOOST_HEADER_DIR

echo "compiling boost with cobalt"
sh ./bootstrap.sh
./b2 cxxstd=20 -j $(nproc)

# creating symlinks
ln -s -f $(readlink -f "./stage/lib") "$(readlink -f "../libs")/boost"
ln -s -f $(readlink -f "./boost") "$(readlink -f "../include")/boost"
