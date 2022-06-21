cmake_minimum_required(VERSION 3.2)
project(orbbec_camera_driver)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# set version infomation
set (orbbec_camera_driver_VERSION_MAJOR 1)
set (orbbec_camera_driver_VERSION_MINOR 0)

configure_file (
"${PROJECT_SOURCE_DIR}/orbbec_camera_driverConfig.h.in"
"${PROJECT_BINARY_DIR}/orbbec_camera_driverConfig.h"
)

#### find my packages in /install
get_filename_component(TOP_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)
set(CMAKE_PREFIX_PATH ${TOP_DIR}/install)

find_package(zmq_communicater 1.0 REQUIRED)
find_package(FlatbuffersMessage 1.0 REQUIRED)

include_directories (${zmq_communicater_INCLUDE_DIR})
include_directories (${FlatbuffersMessage_INCLUDE_DIR})

# message(STATUS "zmq_communicater_include_dir: " ${zmq_communicater_INCLUDE_DIR})
# message(STATUS "zmq_communicater_LIBS: " ${zmq_communicater_LIBRARIES})

#### find general packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories (${EIGEN3_INCLUDE_DIRS})

#### set dependence lisbs with src code
# uvc libs
set(UVC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libuvc)
set(UVC_LIBS "${UVC_LIBS_DIR}/libuvc.so")
# openni2 libs
set(OPENNI2_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/openni2)
set (OPENNI2_LIBS "${OPENNI2_LIBS_DIR}/libOpenNI2.so;${OPENNI2_LIBS_DIR}/OpenNI2/Drivers/liborbbec.so")# 这个路径是固定的，不是这个目录结构OpenNI2/Drivers/lib*.so就找不到
# this project
include_directories ("${PROJECT_SOURCE_DIR}/include;${PROJECT_SOURCE_DIR}/src")

# 构建驱动库
add_library(orbbec_camera_driver SHARED
            src/camera.cpp
            src/uvc_camera.cpp
            src/oni_camera.cpp)

target_link_libraries(orbbec_camera_driver PUBLIC
                    ${OpenCV_LIBRARIES}
                    ${UVC_LIBS}
                    ${OPENNI2_LIBS}
                    ${Eigen_LIBRARIES}
                    zmq_communicater
                    )


# 构建驱动节点
add_executable(orbbec_camera_node
                src/orbbec_camera_node.cpp)
                
target_link_libraries(orbbec_camera_node PUBLIC
                ${OpenCV_LIBRARIES}
                ${UVC_LIBS}
                ${OPENNI2_LIBS}
                ${Eigen_LIBRARIES}
                zmq_communicater
                orbbec_camera_driver
                )

# file(COPY ${CMAKE_CURRENT_LIST_DIR}/resource DESTINATION ${CMAKE_BINARY_DIR}/)
# add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")
                    
