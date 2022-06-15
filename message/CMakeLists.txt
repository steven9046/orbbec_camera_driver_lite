cmake_minimum_required(VERSION 2.8)
project(FlatbuffersMessage)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)

# flatbuffers libs
set(FLATBUFFERS_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libflatbuffers)
set(FLATBUFFERS_LIBS "${FLATBUFFERS_LIBS_DIR}/libflatbuffers.so")

include_directories ("${PROJECT_SOURCE_DIR}/include")

# 构建驱动库
add_library(FlatbuffersMessage SHARED
            flat_buffers_message.cpp
            )
set(FlatbuffersMessage_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(FlatbuffersMessage 
                    ${FLATBUFFERS_LIBS}
                    )

# # 构建测试节点
# add_executable(zmq_publisher_test_node
#                 src/zmq_publisher_test_node.cpp)
# target_link_libraries(zmq_publisher_test_node
#                     zmq_communicater)

# add_executable(zmq_subscriber_test_node
#                 src/zmq_subscriber_test_node.cpp)
# target_link_libraries(zmq_subscriber_test_node
#                         zmq_communicater
#                         pthread)

# file(COPY ${CMAKE_CURRENT_LIST_DIR}/resource DESTINATION ${CMAKE_BINARY_DIR}/)
# add_definitions(-DRESOURCE_DIR="${CMAKE_BINARY_DIR}/resource/")
                    
