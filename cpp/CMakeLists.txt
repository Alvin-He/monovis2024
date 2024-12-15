
cmake_minimum_required(VERSION 3.22.0)

project(
	MonoVision
	VERSION 0.0.1
	LANGUAGES CXX)
# set( CMAKE_VERBOSE_MAKEFILE on )
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/artifacts")

set(THREADS_PREFER_PTHREAD_FLAG ON)

set(NTCORE_INSTALL_PATH "${CMAKE_SYSROOT}/usr/local/ntcore")
set(ntcore_DIR ${NTCORE_INSTALL_PATH}/share/ntcore)
set(wpiutil_DIR ${NTCORE_INSTALL_PATH}/share/wpiutil)
set(wpinet_DIR ${NTCORE_INSTALL_PATH}/share/wpinet)
include_directories(SYSTEM ${NTCORE_INSTALL_PATH}/include)

set(CUDAToolkit_ROOT /usr/local/cuda)

include_directories(
	"include/"
	"src/"
)
add_subdirectory(include/fmt EXCLUDE_FROM_ALL)
link_libraries(fmt)

if(NOT Boost_INCLUDE_DIRS)
	set(Boost_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include/boost")
endif()
include_directories(${Boost_INCLUDE_DIRS})
if(NOT Boost_LIBRARY_DIRS) 
	set(Boost_LIBRARY_DIRS "${CMAKE_SOURCE_DIR}/libs/boost")
endif()
add_library(boost_cobalt STATIC IMPORTED)
set_property(TARGET boost_cobalt PROPERTY IMPORTED_LOCATION ${Boost_LIBRARY_DIRS}/libboost_cobalt.a)
add_library(boost_timer STATIC IMPORTED)
set_property(TARGET boost_timer PROPERTY IMPORTED_LOCATION ${Boost_LIBRARY_DIRS}/libboost_timer.a)
add_library(boost_program_options STATIC IMPORTED)
set_property(TARGET boost_program_options PROPERTY IMPORTED_LOCATION ${Boost_LIBRARY_DIRS}/libboost_program_options.a)
add_library(boost_thread STATIC IMPORTED)
set_property(TARGET boost_thread PROPERTY IMPORTED_LOCATION ${Boost_LIBRARY_DIRS}/libboost_thread.a)
add_library(boost_chrono STATIC IMPORTED)
set_property(TARGET boost_chrono PROPERTY IMPORTED_LOCATION ${Boost_LIBRARY_DIRS}/libboost_chrono.a)
add_library(boost_filesystem STATIC IMPORTED)
set_property(TARGET boost_filesystem PROPERTY IMPORTED_LOCATION ${Boost_LIBRARY_DIRS}/libboost_filesystem.a)

add_library(cuAprilTags STATIC IMPORTED)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	set_property(TARGET cuAprilTags PROPERTY IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/cuAprilTags/libcuapriltags_aarch64.a")
else()
	set_property(TARGET cuAprilTags PROPERTY IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/libs/cuAprilTags/libcuapriltags_x86_64.a")
endif()

find_package(CUDAToolkit)
find_package(OpenSSL REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
# find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED 
	# PATHS ${CMAKE_SOURCE_DIR}/../bin/opencv/lib/cmake/opencv4/
	)
find_package(Threads REQUIRED)
find_package(ntcore REQUIRED)

# add_executable(prog src/entry.cpp)
# target_compile_options(prog 
# 	PUBLIC "-fcoroutines")
# target_link_libraries(prog
# 	PRIVATE boost_cobalt
# 	PRIVATE boost_program_options
# 	PRIVATE boost_thread
# 	PRIVATE boost_chrono
# 	PRIVATE ${OpenCV_LIBS}
# 	PRIVATE Threads::Threads
# 	PRIVATE boost_timer
# 	PRIVATE Eigen3::Eigen
# 	PRIVATE ntcore
# )

add_executable(apriltag_service src/services/EstimateApriltag.cpp)
target_compile_options(apriltag_service 
	PUBLIC "-fcoroutines")
target_link_libraries(apriltag_service
	PRIVATE boost_cobalt
	PRIVATE boost_program_options
	PRIVATE boost_thread
	PRIVATE boost_chrono
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
	PRIVATE boost_timer
	PRIVATE Eigen3::Eigen
	PRIVATE ntcore
	PRIVATE ${OPENSSL_LIBRARIES}
	# PRIVATE ceres
)

if(CUDAToolkit_FOUND) 
	target_link_libraries(apriltag_service 	
		PRIVATE cuAprilTags
		PRIVATE CUDA::cudart
		PRIVATE CUDA::cuda_driver)
endif()

add_executable(pose_service src/services/PoseTracker.cpp)
target_compile_options(pose_service 
	PUBLIC "-fcoroutines")
target_link_libraries(pose_service
	PRIVATE boost_cobalt
	PRIVATE boost_program_options
	PRIVATE boost_thread
	PRIVATE boost_chrono
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
	PRIVATE boost_timer
	PRIVATE Eigen3::Eigen
	PRIVATE ntcore
	PRIVATE ${OPENSSL_LIBRARIES}
)

add_executable(redis_test src/redis.test.cpp)
target_compile_options(redis_test 
	PUBLIC "-fcoroutines")
target_link_libraries(redis_test
	PRIVATE boost_cobalt
	PRIVATE boost_program_options
	PRIVATE boost_thread
	PRIVATE boost_chrono
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
	PRIVATE boost_timer
	PRIVATE Eigen3::Eigen
	PRIVATE ntcore
	PRIVATE ${OPENSSL_LIBRARIES}
)

add_executable(test_world_update tests/apriltag.world.test.cpp)
target_compile_options(test_world_update 
	PUBLIC "-fcoroutines")
target_link_libraries(test_world_update
	PRIVATE boost_cobalt
	PRIVATE boost_program_options
	PRIVATE boost_thread
	PRIVATE boost_chrono
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
	PRIVATE boost_timer
	PRIVATE Eigen3::Eigen
	PRIVATE ntcore
	PRIVATE ${OPENSSL_LIBRARIES}
)

file(GLOB calib_srcs interactive_calib/*.cpp)
add_executable(calib ${calib_srcs})
# add_executable(calib src/calibrate/calibrate_camera_charuco.cpp)
target_compile_options(calib 
	PUBLIC "-fcoroutines")
target_link_libraries(calib
	PRIVATE boost_cobalt
	PRIVATE boost_program_options
	PRIVATE boost_thread
	PRIVATE boost_chrono
	PRIVATE boost_filesystem
	PRIVATE ${OpenCV_LIBS}
	PRIVATE Threads::Threads
	PRIVATE boost_timer
	PRIVATE Eigen3::Eigen
	PRIVATE ntcore
	PRIVATE ${OPENSSL_LIBRARIES}
)
