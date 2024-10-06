# CROSS compile config for raspberry pi
cmake_minimum_required(VERSION 3.22.0)

# Specify the cross-compilation toolchain
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_HOST_SYSTEM_PROCESSOR x86_64)

set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)
set(CMAKE_CROSSCOMPILING TRUE)

# Define the path to the toolchain
set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc-12")
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++-12")
set(CMAKE_Fortran_COMPILER "aarch64-linux-gnu-gfortran-12")
set(CMAKE_AR "aarch64-linux-gnu-ar")
set(CMAKE_AS "aarch64-linux-gnu-as")
set(CMAKE_NM "aarch64-linux-gnu-nm")
set(CMAKE_LINKER "aarch64-linux-gnu-ld")
set(CMAKE_CXX_STANDARD 20)

# Tell CMake to use the sysroot for finding libraries and headers
set(CMAKE_SYSROOT "/sysroot" CACHE INTERNAL "" FORCE)
set(CMAKE_SYSROOT_COMPILE "${CMAKE_SYSROOT}" CACHE INTERNAL "" FORCE)
set(CMAKE_SYSROOT_LINK "${CMAKE_SYSROOT}" CACHE INTERNAL "" FORCE)
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/installed/arm64" CACHE INTERNAL "" FORCE)
include_directories(
    SYSTEM "${CMAKE_SYSROOT}/usr/local/include"
    SYSTEM "${CMAKE_SYSROOT}/usr/include"
)

set(PKG_CONFIG_LIBDIR "${CMAKE_SYSROOT}/usr/local/lib/aarch64-linux-gnu/pkgconfig:${CMAKE_SYSROOT}/usr/local/lib/pkgconfig:${CMAKE_SYSROOT}/usr/local/share/pkgconfig:${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${CMAKE_SYSROOT}/usr/lib/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig" CACHE INTERNAL "" FORCE)

set(LD_PATH ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu)
set(CMAKE_PATH ${LD_PATH}/cmake/)

# Additional flags (optional, may vary depending on the project)
set(CMAKE_C_FLAGS_INIT "-fPIC")
set(CMAKE_CXX_FLAGS_INIT "-fPIC")
set(CMAKE_Fortran_FLAGS_INIT "-fPIC")

# # Set linker flags (important for linking with sysroot libraries)
set(CMAKE_EXE_LINKER_FLAGS_INIT "--sysroot=\"${CMAKE_SYSROOT}\"")

set(CMAKE_EXPORT_COMPILE_COMMANDS=ON)
# Ensure CMake searches for programs and libraries inside the sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


