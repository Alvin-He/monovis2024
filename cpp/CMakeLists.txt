
cmake_minimum_required(VERSION 3.22.0)

project(
	MonoVision
	VERSION 0.0.1
	LANGUAGES CXX)
# set( CMAKE_VERBOSE_MAKEFILE on )
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#add_link_options("-fuse-ld=lld")
set(CUDAToolkit_ROOT /usr/local/cuda)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(NTCORE_INSTALL_PATH /usr/local/ntcore)
set(ntcore_DIR ${NTCORE_INSTALL_PATH}/share/ntcore)
set(wpiutil_DIR ${NTCORE_INSTALL_PATH}/share/wpiutil)
set(wpinet_DIR ${NTCORE_INSTALL_PATH}/share/wpinet)
include_directories(SYSTEM ${NTCORE_INSTALL_PATH}/include)


include_directories(
	"include/"
	"src/"
)
add_subdirectory(include/fmt EXCLUDE_FROM_ALL)
link_libraries(fmt)

add_library(boost_cobalt STATIC IMPORTED)
set_property(TARGET boost_cobalt PROPERTY IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libs/boost/libboost_cobalt.a)
add_library(boost_timer STATIC IMPORTED)
set_property(TARGET boost_timer PROPERTY IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libs/boost/libboost_timer.a)
add_library(boost_program_options STATIC IMPORTED)
set_property(TARGET boost_program_options PROPERTY IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libs/boost/libboost_program_options.a)

find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED 
	# PATHS ${CMAKE_SOURCE_DIR}/../bin/opencv/lib/cmake/opencv4/
	)
find_package(Threads REQUIRED)
find_package(ntcore REQUIRED)

add_executable(prog src/entry.cpp)
target_compile_options(prog 
	PUBLIC "-fcoroutines")
target_link_libraries(prog
	PRIVATE boost_cobalt
	PRIVATE boost_program_options
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
	PRIVATE boost_timer
	PRIVATE Eigen3::Eigen
	PRIVATE ntcore
)
