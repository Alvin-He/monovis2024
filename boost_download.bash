#! /bin/bash

BOOST_ARCHIVE_LINK="https://archives.boost.io/release/1.85.0/source/boost_1_85_0.tar.gz"

WS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $WS_DIR
set -ex 

cd ./cpp
echo "Downloading boost from $BOOST_ARCHIVE_LINK"
wget -O boost.tar.gz $BOOST_ARCHIVE_LINK
tar -xf boost.tar.gz

echo "finished, cleaning up"
rm boost.tar.gz