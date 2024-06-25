#! /bin/bash

echo "build & install script for ntcore and WPILIB dev tools, installing to /usr/local/ntcore."
if [[ $1 == "release" ]]; then 
    echo "Building Release, Call this script with \`./install_ntcore.bash debug\` to build debug version."
elif [[ $1 == "debug" ]]; then
    echo "Building Debug. Call this script with \`./install_ntcore.bash release\` to build optimized version."
else
    echo "Usuage: \`./install_ntcore.bash <release || debug> <devtools ?> \` "
    exit
fi
if [[ $2 == "devtools" ]]; then 
    echo "Also building WPILIB dev tools."
else
    echo "WPILIB dev tools not built. Call this script with \`./install_ntcore.bash <release || debug> devtools\` to build outline viewer."
fi

echo "Made by @Alvin-He for FRC4669 - Galileo Robotics, target allwpilib version v2024.3.2"
echo " "
echo " "

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $SCRIPT_DIR
set -ex

# fixes git safety checks https://stackoverflow.com/a/71904131
git config --global --add safe.directory '*'

if [[ -z "$CMAKE_PATH" ]]; then
    CMAKE_PATH="cmake"
fi

if [ -d "./allwpilib" ]; then
    echo "allwpilib exist, skipping download"
else
    echo "downloading allwpilib"
    git clone https://github.com/wpilibsuite/allwpilib.git -b v2024.3.2
fi
cd allwpilib

if [[ -d "./build" ]]; then 
    echo "removing old cmake build directory"
    rm -r ./build
fi

echo "installing build-essential and protobuf dependency"
sudo apt install build-essential -y
sudo apt install protobuf-compiler -y


echo "configuring cmake"

$CMAKE_PATH \
-D WITH_JAVA=OFF \
-D WITH_JAVA_SOURCE=OFF \
-D WITH_CSCORE=OFF \
-D WITH_WPIMATH=OFF \
-D WITH_WPIUNITS=OFF \
-D WITH_WPILIB=OFF \
-D WITH_EXAMPLES=OFF \
-D WITH_TESTS=OFF \
-D WITH_GUI=OFF \
-D WITH_SIMULATION_MODULES=OFF \
-D WITH_NTCORE=ON \
-D CMAKE_INSTALL_PREFIX=/usr/local/ntcore \
-S . -B ./build

echo "building"

if [[ $1 == "release" ]]; then 
    $CMAKE_PATH --build ./build --config Release --target ntcore -j $(nproc)
else
    $CMAKE_PATH --build ./build --config Debug --target ntcore -j $(nproc)
fi

if [[ $2 == "devtools" ]]; then 
    echo "configuring cmake for devtools"

    $CMAKE_PATH \
    -D WITH_JAVA=OFF \
    -D WITH_JAVA_SOURCE=OFF \
    -D WITH_CSCORE=OFF \
    -D WITH_WPIMATH=ON \
    -D WITH_WPIUNITS=OFF \
    -D WITH_WPILIB=OFF \
    -D WITH_EXAMPLES=OFF \
    -D WITH_TESTS=OFF \
    -D WITH_GUI=ON \
    -D WITH_SIMULATION_MODULES=OFF \
    -D WITH_NTCORE=ON \
    -D CMAKE_INSTALL_PREFIX=/usr/local/ntcore \
    -S . -B ./build

    # building one of the dev tools just builds everything
    $CMAKE_PATH --build ./build --config Release --target outlineviewer -j $(nproc)
fi

cd ./build
make install -j $(nproc)
if [[ $2 == "devtools" ]]; then 
    if [[ -d "/usr/local/ntcore/bin/" ]]; then 
        rm -r /usr/local/ntcore/bin/
    fi
    mv ./bin/ /usr/local/ntcore/bin/
fi
set +x
printf "\n\n\nFinished, you may delete $SCRIPT_DIR/allwpilib if you want to save space.\n"
printf "ntcore is installed at /usr/local/ntcore\n"
printf "\nAdd this to your CMakeLists.txt to use ntcore:\n
set(NTCORE_INSTALL_PATH /usr/local/ntcore)
set(ntcore_DIR \${NTCORE_INSTALL_PATH}/share/ntcore)
set(wpiutil_DIR \${NTCORE_INSTALL_PATH}/share/wpiutil)
set(wpinet_DIR \${NTCORE_INSTALL_PATH}/share/wpinet)
include_directories(SYSTEM \${NTCORE_INSTALL_PATH}/include)
find_package(ntcore REQUIRED)\n\n"
printf "NOTE:DO NOT find_package(wpilib), the wpilib package file is broken due to this special install of only ntcore.\n"

if [[ $2 == "devtools" ]]; then 
printf "\n\n"
printf "WPILIB Tools installed at /usr/local/ntcore/bin, You may add this directory to PATH for ease of access. \n\n"
fi

printf "Read this if you encounter any problems: https://github.com/wpilibsuite/allwpilib/blob/v2024.3.2/README-CMAKE.md \n"