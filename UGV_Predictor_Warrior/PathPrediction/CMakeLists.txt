cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(PathPred)

# cmake build type
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# build shared libs flag
option(BUILD_SHARED_LIBS "Build shared libraries" ON)


# operating system conditional compile
if (WIN32)
    message(STATUS "Platform :  windows")
elseif(APPLE)
    message(STATUS "Platform :  apple")
elseif (UNIX)
    message(STATUS "Platform :  unix")
endif()


# cmake find module
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


link_directories(/usr/local/cuda/lib64)  # 替换成您的 CUDA 安装路径

# # 设置RPATH，以确保运行时可以找到正确的库
set(CMAKE_INSTALL_RPATH "/home/guanglei/ThirdParty/libtorch/lib;/usr/lib/x86_64-linux-gnu")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
set(CMAKE_CUDA_COMPILER /usr/local/cuda-11.3/bin/nvcc)
set(Torch_DIR /home/guanglei/ThirdParty/libtorch/share/cmake/Torch)

find_package(OpenMP QUIET)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)


find_package(OpenCV REQUIRED)
message(STATUS ${OpenCV_LIBRARIES})
include_directories(${OpenCV_INCLUDE_DIRS})

# openMP
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# debug postfix
set(CMAKE_DEBUG_POSTFIX _d)

# C++11
set(CMAKE_CXX_STANDARD 11)

#"-march=native" 会使Eigen报关于字节对齐的错误
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
else()
    # Debug 关O3优化
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -Wno-return-type -Wno-deprecated-declarations")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-return-type -Wno-deprecated-declarations")
endif()


find_package(Torch REQUIRED)

include_directories(
        /usr/include
        /usr/local/include
        /opt/ros/noetic/include
        ../../../devel/include
        ./
        message
        ${catkin_INCLUDE_DIRS}
        )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) # for .a

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)



add_subdirectory(misc)
add_subdirectory(CSatellitePatch)

# include_directories(${OpenCV_INCLUDE_DIRS})

## set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

enable_language(CUDA)  # 启用CUDA语言支持



# set bin output dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../bin/)


aux_source_directory(. SRC_LIST)

add_executable(PathPred ${SRC_LIST})
set_target_properties(PathPred  PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
target_compile_options(PathPred PRIVATE ${OpenMP_CXX_FLAGS})

target_link_libraries(PathPred "${TORCH_LIBRARIES}" ${catkin_LIBRARIES} ${OpenCV_LIBS}  CSatellitePatch Misc)

set_property(TARGET PathPred PROPERTY CXX_STANDARD 14)






