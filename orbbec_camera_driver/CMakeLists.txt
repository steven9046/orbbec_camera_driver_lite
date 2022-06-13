cmake_minimum_required(VERSION 2.8)
project(orbbec_camera_driver)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# uvc libs
set(UVC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libuvc)
set(UVC_LIBS "${UVC_LIBS_DIR}/libuvc.so")

# openni2 libs
set(OPENNI2_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/openni2)
set (OPENNI2_LIBS "${OPENNI2_LIBS_DIR}/libOpenNI2.so;${OPENNI2_LIBS_DIR}/OpenNI2/Drivers/liborbbec.so")# 这个路径是固定的，不是这个目录结构OpenNI2/Drivers/lib*.so就找不到


include_directories ("${PROJECT_SOURCE_DIR}/include;${PROJECT_SOURCE_DIR}/src")
include_directories (${OpenCV_INCLUDE_DIRS})
include_directories (${EIGEN3_INCLUDE_DIRS})

# 构建驱动库
add_library(orbbec_camera_driver SHARED
            src/camera.cpp
            src/uvc_camera.cpp
            src/oni_camera.cpp)
target_link_libraries(orbbec_camera_driver 
                    ${OpenCV_LIBRARIES}
                    ${UVC_LIBS}
                    ${OPENNI2_LIBS}
                    ${Eigen_LIBRARIES}
                    )

# 构建驱动节点
add_executable(orbbec_camera_node
                src/orbbec_camera_node.cpp)
target_link_libraries(orbbec_camera_node
                orbbec_camera_driver)

# file(COPY ${CMAKE_CURRENT_LIST_DIR}/resource DESTINATION ${CMAKE_BINARY_DIR}/)
# add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")
                    
