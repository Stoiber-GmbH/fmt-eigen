cmake_minimum_required(VERSION 3.0)
project(fmt_eigen)

# use without ROS, and pure fmt
# first install fmt from https://github.com/fmtlib/fmt

option(FMT_EIGEN_BUILD_EXAMPLE "build example" ON)
option(FMT_EIGEN_FETCH_EIGEN "fetch eigen" ON)

if (FMT_EIGEN_FETCH_EIGEN)

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

  add_library(Eigen3 INTERFACE IMPORTED GLOBAL)
  add_dependencies(Eigen3 eigen)
  target_compile_features(Eigen3 INTERFACE cxx_std_14)

  set_target_properties(Eigen3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INSTALL_DIR}/include/eigen3"
  )
else()
  find_package(Eigen3 REQUIRED)
endif()


find_package(fmt REQUIRED)

add_library(fmt_eigen INTERFACE)
target_link_libraries(fmt_eigen INTERFACE fmt::fmt)
target_include_directories(fmt_eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(fmt_eigen INTERFACE cxx_std_17)

add_library(fmt_eigen::fmt_eigen ALIAS fmt_eigen)

if (FMT_EIGEN_BUILD_EXAMPLE)
  add_executable(example example.cpp)
  target_link_libraries(example
          fmt_eigen
          fmt::fmt
          Eigen3
  )
endif()


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