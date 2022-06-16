cmake_minimum_required(VERSION 2.8)
project(zmq_communicater)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)

# 得到上级目录,设置搜索路径(findpackage),设置安装路径(prefix)
get_filename_component(TOP_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)
set(flatbuffer_message_DIR ${TOP_DIR}/install/FlatbuffersMessage/lib/cmake/flatbuffer_message)
find_package(flatbuffer_message 1.0 REQUIRED)

# uvc libs
set(ZMQ_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libzmq)
set(ZMQ_LIBS "${ZMQ_LIBS_DIR}/libzeromq.so")

include_directories ("${PROJECT_SOURCE_DIR}/include;${PROJECT_SOURCE_DIR}/src")
include_directories (${flatbuffer_message_INCLUDE_DIR})

# 构建驱动库
add_library(zmq_communicater SHARED
            include/zmq_publisher.hpp
            include/zmq_subscriber.hpp
            src/zmq_base.cpp
            )

target_link_libraries(zmq_communicater 
                    ${ZMQ_LIBS}
                    ${flatbuffer_message_LIBS}
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

set(CMAKE_INSTALL_PREFIX ${TOP_DIR}install/zmq_communicater)    
install(TARGETS zmq_communicater zmq_publisher_test_node zmq_subscriber_test_node
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)
