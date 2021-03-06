cmake_minimum_required(VERSION 2.8)
project(orbbec_camera_driver)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)

configure_file (
"${PROJECT_SOURCE_DIR}/TutorialConfig.h.in"
"${PROJECT_BINARY_DIR}/TutorialConfig.h"
)

# 这里find的package之后的include和ibraries都是用这个宏扩展的
# 这里没有版本要求，用的都是最基本的功能
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    find_package(Pangolin 0.4 REQUIRED)
    add_definitions(-DLINUX_X86)
endif()

# uvc libs
set(UVC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_SYSTEM_PROCESSOR}/libuvc)
set(UVC_LIBS "${UVC_LIBS_DIR}/libuvc.so")

# openni2 libs
set(OPENNI2_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_SYSTEM_PROCESSOR}/openni2)
set (OPENNI2_LIBS "${OPENNI2_LIBS_DIR}/libOpenNI2.so;${OPENNI2_LIBS_DIR}/OpenNI2/Drivers/liborbbec.so")# 这个路径是固定的，不是这个目录结构OpenNI2/Drivers/lib*.so就找不到

# zmq libs
set(ZMQ_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_SYSTEM_PROCESSOR}/libzmq)
set(ZMQ_LIBS "${ZMQ_LIBS_DIR}/libzeromq.so")

# flatbuffers libs
set(FLATBUFFERS_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_SYSTEM_PROCESSOR}/libflatbuffers)
set(FLATBUFFERS_LIBS "${FLATBUFFERS_LIBS_DIR}/libflatbuffers.so")

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories (${PCL_INCLUDE_DIRS})
include_directories (${Eigen_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include;${PROJECT_SOURCE_DIR}/src")

include_directories (${Pangolin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
                    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
                    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)


# tensorflow lite
# set(TENSORFLOW_SOURCE_DIR "/home/ss/tensorflow_src")
# if(NOT TENSORFLOW_SOURCE_DIR)
#   get_filename_component(TENSORFLOW_SOURCE_DIR
#     "${CMAKE_CURRENT_LIST_DIR}/../../../../"
#     ABSOLUTE
#   )
# endif()

# add_subdirectory(
#   "${TENSORFLOW_SOURCE_DIR}/tensorflow/lite"
#   "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite"
#   EXCLUDE_FROM_ALL
# )      

# # tflite library
# set(TFLITE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_SYSTEM_PROCESSOR}/tflite)
# set(TFLITE_LIBS "${TFLITE_SOURCE_DIR}/libtensorflowlite.so")
# include_directories ("${PROJECT_SOURCE_DIR}/include")

# message(STATUS "TENSORFLOW_SOURCE_DIR: " ${TENSORFLOW_SOURCE_DIR})
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    add_executable(orbbec_camera_driver
                    src/camera.cpp
                    src/uvc_camera.cpp
                    src/oni_camera.cpp
                    src/frame.cpp
                    src/orb/ORBextractor.cpp
                    src/orb/ORBmatcher.cpp
                    src/orb/Optimizer.cpp
                    src/orb/OptimizableTypes.cpp
                    src/orb/MapPoint.cpp
                    src/orb/Frame.cpp
                    src/orb/Tracking.cpp
                    src/orb/Settings.cpp
                    src/orb/CameraModels/Converter.cpp
                    src/orb/CameraModels/GeometricTools.cpp
                    src/orb/CameraModels/TwoViewReconstruction.cpp
                    src/orb/CameraModels/Pinhole.cpp
                    src/orb/MapDrawer.cpp
                    src/viewer.cpp
                    main.cpp)
else()
    add_executable(orbbec_camera_driver
                    src/camera.cpp
                    src/uvc_camera.cpp
                    src/oni_camera.cpp
                    src/frame.cpp
                    src/orb/ORBextractor.cpp
                    src/orb/ORBmatcher.cpp
                    src/orb/Optimizer.cpp
                    src/orb/OptimizableTypes.cpp
                    src/orb/MapPoint.cpp
                    src/orb/Frame.cpp
                    src/orb/Tracking.cpp
                    src/orb/Settings.cpp
                    src/orb/CameraModels/Converter.cpp
                    src/orb/CameraModels/GeometricTools.cpp
                    src/orb/CameraModels/TwoViewReconstruction.cpp
                    src/orb/CameraModels/Pinhole.cpp
                    main.cpp
                    )
endif()

# # For InferenceHelper
# set(INFERENCE_HELPER_DIR ${CMAKE_CURRENT_LIST_DIR}/src/InferenceHelper/)
# set(INFERENCE_HELPER_ENABLE_TFLITE ON CACHE BOOL "TFLITE")
# set(INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK ON CACHE BOOL "TFLITE_XNNPACK")
# add_subdirectory(${INFERENCE_HELPER_DIR}/inference_helper inference_helper)
# target_include_directories(orbbec_camera_driver PUBLIC ${INFERENCE_HELPER_DIR}/inference_helper)
# target_link_libraries(orbbec_camera_driver InferenceHelper)
 
# # Link Common Helper module
# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/common_helper common_helper)
# target_include_directories(orbbec_camera_driver PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src/common_helper)
# target_link_libraries(orbbec_camera_driver CommonHelper)

# Link ImageProcessor module
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/image_processor image_processor)
target_include_directories(orbbec_camera_driver PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src/image_processor)
target_link_libraries(orbbec_camera_driver ImageProcessor)

target_link_libraries(orbbec_camera_driver 
                    ${OpenCV_LIBRARIES}
                    ${PCL_LIBRARIES}
                    ${Eigen_LIBRARIES}
                    ${UVC_LIBS}
                    ${ZMQ_LIBS}
                    ${OPENNI2_LIBS}
                    ${TFLITE_LIBS}
                    ${FLATBUFFERS_LIBS}
                    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
                    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
                    -lcrypto
                    )

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    target_link_libraries(orbbec_camera_driver
                            ${Pangolin_LIBRARIES})
endif()

add_executable(reciever reciever.cpp)
target_link_libraries(reciever 
                    ${ZMQ_LIBS}
                    ${FLATBUFFERS_LIBS}
                    )

file(COPY ${CMAKE_CURRENT_LIST_DIR}/resource DESTINATION ${CMAKE_BINARY_DIR}/)
add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")
                    
