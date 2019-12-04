cmake_minimum_required(VERSION 2.8)
project(ccvtest)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")
set(COMMON_SOURCES src/DynamixelWrapper.cpp)
set(DXL_INC_DIR /home/parallels/DynamixelSDK/c++/include/dynamixel_sdk 
				/home/pi/dev/DynamixelSDK/c++/include/dynamixel_sdk)

include_directories(${DXL_INC_DIR} ./src)

add_executable(ccvtest ${COMMON_SOURCES} example/ccvtest.cpp)
target_link_libraries(ccvtest dxl_x64_cpp rt)	# Regular PC
#target_link_libraries(ccvtest dxl_sbc_cpp rt)	# Raspberry Pi

INSTALL(
    TARGETS ccvtest
    EXPORT ${PROJECT_NAME}-export
    RUNTIME DESTINATION bin/${PROJECT_NAME}
)

INSTALL(
    EXPORT ${PROJECT_NAME}-export
    FILE ${PROJECT_NAME}-config.cmake
    DESTINATION share/cmake/${PROJECT_NAME}
)
