cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-fPIC")

project(PCL_OpenNI)

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(openni_viewer_simple openni_viewer_simple.cpp)

target_link_libraries(openni_viewer_simple ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})
