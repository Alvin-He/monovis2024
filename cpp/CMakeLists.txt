
cmake_minimum_required(VERSION 3.22.0)

project(
	MonoVision
	VERSION 0.0.1
	LANGUAGES CXX)


include_directories(
	"include/"
)

add_executable(server src/Server.cpp)

add_executable(cameraReader src/CameraReader.cpp)