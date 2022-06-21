cmake_minimum_required(VERSION 3.2)
project(zmq_communicater)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set (zmq_communicater_VERSION_MAJOR 1)
set (zmq_communicater_VERSION_MINOR 0)

configure_file (
"${PROJECT_SOURCE_DIR}/zmq_communicaterConfig.h.in"
"${PROJECT_BINARY_DIR}/zmq_communicaterConfig.h"
)

# 得到上级目录,设置搜索路径(findpackage),设置安装路径(prefix)
get_filename_component(TOP_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)
set(CMAKE_PREFIX_PATH ${TOP_DIR}/install)

find_package(FlatbuffersMessage 1.0 REQUIRED)

# zmq libs
set(ZMQ_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
set(ZMQ_LIBS "${ZMQ_LIBS_DIR}/libzmq.so")
message(STATUS "zmq lib dir: " ${ZMQ_LIBS})

# 构建驱动库
add_library(zmq_communicater SHARED
            include/zmq_publisher.hpp
            include/zmq_subscriber.hpp
            src/zmq_base.cpp
            )

target_link_libraries(zmq_communicater PUBLIC
                    ${ZMQ_LIBS}
                    )

target_include_directories (zmq_communicater PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)

target_include_directories (zmq_communicater PUBLIC
                            $<BUILD_INTERFACE:${FlatbuffersMessage_INCLUDE_DIR}>
                            $<INSTALL_INTERFACE:include>
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

set(CMAKE_INSTALL_PREFIX ${TOP_DIR}/install/zmq_communicater)    
install(TARGETS zmq_communicater zmq_publisher_test_node zmq_subscriber_test_node
        EXPORT zmq_communicater
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)


# 构建驱动库
# add_library(FlatbuffersMessage SHARED
#             flat_buffers_message.cpp
#             )

# target_link_libraries(FlatbuffersMessage 
#                     ${FLATBUFFERS_LIBS}
#                     )

# 下边这三句没有尝试
# set(public_headers include/zmq.h include/zmq_utils.h)install
# set(includedir ${prefix}/${CMAKE_INSTALL_INCLUDEDIR})
# PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT SDK)

# 将库文件，可执行文件，头文件安装到指定目录
# install(TARGETS zmq_communicater  
#         DESTINATION lib 
#         EXPORT zmq_communicater # 并导出库信息
#         )

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY libs/ DESTINATION lib)

#  里用库信息生成 .cmake 文件 并安装到指定位置
install(EXPORT zmq_communicater 
        FILE zmq_communicater.cmake
        DESTINATION lib/cmake/zmq_communicater
        )

## .cmake 信息
include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/zmq_communicaterConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/zmq_communicaterConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/zmq_communicater"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/zmq_communicaterConfigVersion.cmake"
  VERSION "${zmq_communicater_VERSION_MAJOR}.${zmq_communicater_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/zmq_communicaterConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/zmq_communicaterConfigVersion.cmake
  DESTINATION lib/cmake/zmq_communicater
)
