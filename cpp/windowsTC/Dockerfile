FROM ubuntu:22.04

RUN apt-get update -y && apt-get upgrade -y

# install build tools

# install compiler 
RUN apt install gcc-12-aarch64-linux-gnu g++-12-aarch64-linux-gnu -y
# # cross compile compilers & linker
# set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc-12")
# set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++-12")
# set(CMAKE_AR "aarch64-linux-gnu-ar")
# set(CMAKE_AS "aarch64-linux-gnu-as")
# set(CMAKE_NM "aarch64-linux-gnu-nm")
# set(CMAKE_LINKER "aarch64-linux-gnu-ld")

RUN apt install cmake -y

# sysroot mount point
VOLUME [ "/sysroot" ]
# project source code directory
VOLUME [ "/src" ]

CMD [ "bash" ]