
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

find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED 
	# PATHS ${CMAKE_SOURCE_DIR}/../bin/opencv/lib/cmake/opencv4/
	)
find_package(Threads REQUIRED)

add_executable(server src/Server.cpp)

add_executable(cameraReader src/CameraReader.cpp)
target_compile_options(cameraReader 
	PUBLIC "-fcoroutines")
target_link_libraries(cameraReader 
	PRIVATE ${OpenCV_LIBS}
	PRIVATE boost_cobalt
	PRIVATE Threads::Threads)

add_executable(serverAsync src/ServerAsync.cpp)
target_compile_options(serverAsync 
	PUBLIC "-fcoroutines")
target_link_libraries(serverAsync 
	PRIVATE boost_cobalt
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
)

add_library(camera STATIC src/camera/camera.cpp)
target_link_libraries(camera
	PRIVATE ${OpenCV_LIBS}
	PRIVATE boost_cobalt
)

add_executable(prog src/entry.cpp)
target_compile_options(prog 
	PUBLIC "-fcoroutines")
target_link_libraries(prog
	PRIVATE boost_cobalt
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
	PRIVATE boost_timer
	PRIVATE Eigen3::Eigen
)
