cmake_minimum_required(VERSION 3.0)
project(example)

set(CMAKE_CXX_STANDARD 17)

# use without ROS, and pure fmt
# first install fmt from https://github.com/fmtlib/fmt


include(ExternalProject)

set(EIGEN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen-install")
set(EIGEN_INCLUDE_DIR "${EIGEN_INSTALL_DIR}/include/eigen3")

file(MAKE_DIRECTORY ${EIGEN_INCLUDE_DIR})

ExternalProject_Add(
    eigen
    URL  https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen-src"
    BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen-build"
    INSTALL_DIR "${EIGEN_INSTALL_DIR}"
    CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    -DCMAKE_BUILD_TYPE=Release
)

file(MAKE_DIRECTORY ${EIGEN_INSTALL_DIR}/include)  # avoid race condition

add_library(eigenlib INTERFACE IMPORTED GLOBAL)
add_dependencies(eigenlib eigen)
target_compile_features(eigenlib INTERFACE cxx_std_14)

set_target_properties(eigenlib PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INSTALL_DIR}/include/eigen3"
)


find_package(fmt REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example
        fmt::fmt
        eigenlib
)

# use with rosfmt
# first install rosfmt with: sudo apt install ros-noetic-rosfmt

#set(rosfmt_DIR /opt/ros/noetic/share/rosfmt/cmake)
#find_package(rosfmt REQUIRED)
#include_directories(${rosfmt_INCLUDE_DIRS})
#add_executable(example example.cpp)
#target_link_libraries(example
#        ${rosfmt_LIBRARIES}
#)


# use with catkin and rosfmt
# first install rosfmt with: sudo apt install ros-noetic-rosfmt
# catkin_make
#find_package(catkin REQUIRED COMPONENTS
#        roscpp
#        rosfmt)
#include_directories(${catkin_INCLUDE_DIRS})
#add_executable(example example.cpp)
#target_link_libraries(example
#        ${catkin_LIBRARIES}
#)