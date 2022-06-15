cmake_minimum_required(VERSION 2.8)
project(zmq_communicater)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)

# 得到上级目录
string(REGEX REPLACE "(.+)\\zmq_communicater.*" "\\1" TOP_DIR ${PROJECT_SOURCE_DIR})
message(STATUS "TOP_DIR: " ${TOP_DIR})

# uvc libs
set(ZMQ_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libzmq)
set(ZMQ_LIBS "${ZMQ_LIBS_DIR}/libzeromq.so")

# flatbuffers_message libs
set(FLATBUFFERS_LIBS_DIR ${TOP_DIR}message/libs/libflatbuffers)
set(FLATBUFFERS_INCLUDE_DIR ${TOP_DIR}message/include)
set(FLATBUFFERS_LIBS "${FLATBUFFERS_LIBS_DIR}/libflatbuffers.so")



include_directories ("${PROJECT_SOURCE_DIR}/include;${PROJECT_SOURCE_DIR}/src")
include_directories (${FLATBUFFERS_INCLUDE_DIR})

# 构建驱动库
add_library(zmq_communicater SHARED
            include/zmq_publisher.hpp
            include/zmq_subscriber.hpp
            src/zmq_base.cpp
            )

target_link_libraries(zmq_communicater 
                    ${ZMQ_LIBS}
                    ${FLATBUFFERS_LIBS}
                    )

# 构建测试节点
add_executable(zmq_publisher_test_node
                src/zmq_publisher_test_node.cpp)
target_link_libraries(zmq_publisher_test_node
                    zmq_communicater)

add_executable(zmq_subscriber_test_node
                src/zmq_subscriber_test_node.cpp)
target_link_libraries(zmq_subscriber_test_node
                        zmq_communicater
                        pthread)

# file(COPY ${CMAKE_CURRENT_LIST_DIR}/resource DESTINATION ${CMAKE_BINARY_DIR}/)
# add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")
                    
