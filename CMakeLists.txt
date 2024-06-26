# LDROBOT Sensor Team, generate: 2024-06-26,01:02
cmake_minimum_required(VERSION 2.8.3)
project(ldlidar_ssl)
 
set(VAL  1)
if(${VAL} EQUAL 1)
  set(CMAKE_BUILD_TYPE "Release") 
  set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -Wall")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  message(STATUS "Mode: Release")
  message(STATUS "optional:-std=c++11")
else()
  set(CMAKE_BUILD_TYPE "Debug") 
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  message(STATUS "Mode: Debug") 
  message(STATUS "optional:-std=c++11")  
endif() 
 
include_directories(
 ${CMAKE_CURRENT_SOURCE_DIR}/include/ldlidar_driver
)
 
add_executable(${PROJECT_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/demo_cascade.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cmd_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/rtrnet.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/ldlidar_driver/libldlidar_driver_ssl.a 
  pthread
)
