
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
find_package(Threads REQUIRED)

include_directories(
	"include/"
)

add_library(boost_cobalt STATIC IMPORTED)
set_property(TARGET boost_cobalt PROPERTY IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libs/boost/libboost_cobalt.a)

find_package(OpenCV REQUIRED 
	# PATHS ${CMAKE_SOURCE_DIR}/../bin/opencv/lib/cmake/opencv4/
	)

add_executable(server src/Server.cpp)

add_executable(cameraReader src/CameraReader.cpp)
target_compile_options(cameraReader 
	PUBLIC "-fcoroutines"
	PUBLIC "-pthread"
	)
target_link_libraries(cameraReader PRIVATE ${OpenCV_LIBS})
target_link_libraries(cameraReader PRIVATE boost_cobalt)

add_executable(serverAsync src/ServerAsync.cpp)
target_compile_options(serverAsync 
	PUBLIC "-fcoroutines"
	# PUBLIC "-pthread"
	)
target_link_libraries(serverAsync 
	PRIVATE boost_cobalt
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
)

add_executable(echo src/echo_server.cpp)
target_compile_options(echo 
	PUBLIC "-fcoroutines"
	PUBLIC "-pthread"
	)
target_link_libraries(echo 
	PRIVATE boost_cobalt)
