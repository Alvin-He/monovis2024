if __name__ != '__main__':
    raise Exception("Error: importing a cli toolchain script!")
import ctypes
import os

WINDOWS = ""
try:
    is_admin = os.geteuid() == 0
    WINDOWS = False
except AttributeError:
    is_admin = ctypes.windll.shell32.IsUserAnAdmin() != 0
    WINDOWS = True

if not is_admin: 
    raise Exception("Not admin, exiting")

import sys
import subprocess
import pathlib

dependencyInstall = subprocess.run("python -m pip install psutil", shell=True)
if dependencyInstall.returncode != 0:
    print("failed to acqiure dependencies, exiting")

import psutil

NTCORE_INSTALL_PATH="/usr/local/ntcore"
if WINDOWS:
    NTCORE_INSTALL_PATH= str(pathlib.Path.home().as_posix()) + "/local/ntcore"

arg_version = sys.argv[1] if 1 < len(sys.argv) else ""
arg_devtools = sys.argv[2] if 2 < len(sys.argv) else ""
CMAKE_PATH = os.environ.get("CMAKE_PATH", "")
if len(CMAKE_PATH) <= 0:
    CMAKE_PATH="cmake" 

print("build & install script for ntcore and WPILIB dev tools, installing to $NTCORE_INSTALL_PATH.")
if arg_version == "release": 
    print("Building Release, Call this script with `./install_ntcore.bash debug` to build debug version.")
elif arg_version == "debug":
    print("Building Debug. Call this script with `./install_ntcore.bash release` to build optimized version.")
else:
    print("Usuage: `./install_ntcore.bash <release || debug> <devtools ?> ` ")
    exit(1)

if arg_devtools == "devtools": 
    print("Also building WPILIB dev tools.")
else:
    print("WPILIB dev tools not built. Call this script with `./install_ntcore.bash <release || debug> devtools` to build outline viewer.")


print("Made by @Alvin-He for FRC4669 - Galileo Robotics, target allwpilib version v2024.3.2")
print(" ")
print(" ")

for process in psutil.process_iter():
    if "code" in process.name():
        print("VSCODE is running, please close any VSCODE processes before running this script.")
        print("VSCODE is known to crash pretty high spec computers when this script is running for some unknown reason.")
        print("Exiting, nothing done due to VSCODE active.")
        exit(1)


SCRIPT_DIR=str(pathlib.Path(__file__).parent.absolute().as_posix())
CWD=SCRIPT_DIR

SHELL="bash "
if WINDOWS:
    SHELL = "powershell "
def exec(cmd: str, cwd=CWD, safeExec = False):
    proc = subprocess.run(SHELL + cmd, shell=True, cwd=cwd)
    if safeExec: return
    proc.check_returncode()


print("installing script dependencies")
# also serves as a sudo check
# sudo apt install patchelf -y

# fixes git safety checks https://stackoverflow.com/a/71904131
exec("git config --global --add safe.directory '*'")


if pathlib.Path(f"{CWD}/allwpilib").exists():
    print("allwpilib exist, skipping download")
else:
    print("downloading allwpilib")
    exec("git clone https://github.com/wpilibsuite/allwpilib.git -b v2024.3.2", cwd=CWD)

CWD += "/allwpilib"

if pathlib.Path(f"{CWD}/build").exists(): 
    print("removing old cmake build directory")
    exec(f"rm -r -v {CWD}/build", CWD)


print("installing build-essential and protobuf dependency")

if WINDOWS:
    os.environ["Protobuf_DIR"]=str(pathlib.Path("~/scoop/apps/protobuf/current")) 
    # exec("winget install protobuf --disable-interactivity", CWD, True)
else:
    exec("""
    sudo apt install build-essential -y
    sudo apt install protobuf-compiler libprotobuf-dev -y
    """, CWD)


print("conguring cmake")

buildCMD = " ".join(f"""{CMAKE_PATH} 
-D WITH_JAVA=OFF 
-D WITH_JAVA_SOURCE=OFF 
-D WITH_CSCORE=OFF 
-D WITH_WPIMATH=OFF 
-D WITH_WPIUNITS=OFF 
-D WITH_WPILIB=OFF 
-D WITH_EXAMPLES=OFF 
-D WITH_TESTS=OFF 
-D WITH_GUI=OFF 
-D WITH_SIMULATION_MODULES=OFF 
-D WITH_NTCORE=ON 
-D CMAKE_INSTALL_PREX={NTCORE_INSTALL_PATH} 
-S {CWD} -B {CWD}/build""".splitlines())
exec(buildCMD, CWD)

print("building")

if arg_version == "release": 
    exec(f"{CMAKE_PATH} --build {CWD}/build --cong Release --target ntcore -j $(nproc)")
else:
    exec(f"{CMAKE_PATH} --build {CWD}/build --cong Debug --target ntcore -j $(nproc))")


if arg_devtools == "devtools": 
    print("conguring cmake for devtools")

    buildCMD = " ".join(f"""{CMAKE_PATH} 
    -D WITH_JAVA=OFF 
    -D WITH_JAVA_SOURCE=OFF 
    -D WITH_CSCORE=OFF 
    -D WITH_WPIMATH=ON 
    -D WITH_WPIUNITS=OFF 
    -D WITH_WPILIB=OFF 
    -D WITH_EXAMPLES=OFF 
    -D WITH_TESTS=OFF 
    -D WITH_GUI=ON 
    -D WITH_SIMULATION_MODULES=OFF 
    -D WITH_NTCORE=ON 
    -D CMAKE_INSTALL_PREX={NTCORE_INSTALL_PATH} 
    -S {CWD} -B {CWD}/build""".splitlines())
    exec(buildCMD, CWD)

    # building one of the dev tools just builds everything
    exec(f"{CMAKE_PATH} --build ./build --cong Release --target outlineviewer -j $(nproc)")


# CWD += "./build"
# make install -j $(nproc)
# if arg_devtools == "devtools": 
#     if -d "$NTCORE_INSTALL_PATH/bin/": 
#         print("removing old binaries")
#         rm -r -v "$NTCORE_INSTALL_PATH/bin/"
    
#     print("patching binaries")
#     for le in "./bin"/*
#     do
#         patchelf --debug --set-rpath $NTCORE_INSTALL_PATH/lib $le 
#     done
#     mv -v ./bin/ "$NTCORE_INSTALL_PATH/bin/"

#     print("moving additional .so les")
#     cp -v ./lib/libeldImages.so $NTCORE_INSTALL_PATH/lib/
#     cp -v ./lib/libimgui.so $NTCORE_INSTALL_PATH/lib/

# print("\n\n\nnished, you may delete $SCRIPT_DIR/allwpilib if you want to save space.\n", end="")
# print("ntcore is installed at $NTCORE_INSTALL_PATH\n", end="")
# print("""\nAdd this to your CMakeLists.txt to use ntcore:\n
# set(NTCORE_INSTALL_PATH $NTCORE_INSTALL_PATH)
# set(ntcore_DIR ${NTCORE_INSTALL_PATH}/share/ntcore)
# set(wpiutil_DIR ${NTCORE_INSTALL_PATH}/share/wpiutil)
# set(wpinet_DIR ${NTCORE_INSTALL_PATH}/share/wpinet)
# include_directories(SYSTEM ${NTCORE_INSTALL_PATH}/include)
# nd_package(ntcore REQUIRED)\n\n""", end="")
# print("NOTE:DO NOT nd_package(wpilib), the wpilib package le is broken due to this special install of only ntcore.\n", end="")

# if arg_devtools == "devtools": 
#     print("\n\n", end="")
#     print("WPILIB Tools installed at $NTCORE_INSTALL_PATH/bin, You may add this directory to PATH for ease of access. \n\n", end="")


# print("Read this if you encounter any problems: https://github.com/wpilibsuite/allwpilib/blob/v2024.3.2/README-CMAKE.md \n", end="")
