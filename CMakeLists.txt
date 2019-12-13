cmake_minimum_required(VERSION 2.8)
project(DynamixelWrapper)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

set(COMMON_SOURCES src/DynamixelWrapper.cpp)
set(DXL_INC_DIR /usr/local/include/dynamixel_sdk)

include_directories(${DXL_INC_DIR} ./src ./include)

add_executable(ccvtest ${COMMON_SOURCES} example/ccvtest.cpp)
add_library(dxl_wrapper ${COMMON_SOURCES} src/DynamixelWrapper.cpp)


target_link_libraries(ccvtest dxl_x64_cpp rt)	# Regular PC
#target_link_libraries(ccvtest dxl_sbc_cpp rt)	# Raspberry Pi


INSTALL (
	FILES include/DynamixelWrapper.hpp
	DESTINATION /usr/local/include
)

INSTALL (
	FILES build/libdxl_wrapper.a
	DESTINATION /usr/local/lib
)

