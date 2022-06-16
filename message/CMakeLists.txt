cmake_minimum_required(VERSION 2.8)
project(FlatbuffersMessage)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set (FlatbufferMessage_VERSION_MAJOR 1)
set (FlatbufferMessage_VERSION_MINOR 0)

configure_file (
"${PROJECT_SOURCE_DIR}/flatbuffer_messageConfig.h.in"
"${PROJECT_BINARY_DIR}/flatbuffer_messageConfig.h"
)

# 设置安装路径 /home/ss/orbbec_camera_driver/install
# string(REGEX REPLACE "(.+)\\message.*" "\\1" TOP_DIR ${PROJECT_SOURCE_DIR})
get_filename_component(TOP_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)
message(STATUS "TOP_DIR: " ${TOP_DIR})
set(CMAKE_INSTALL_PREFIX ${TOP_DIR}/install/FlatbuffersMessage)
message(STATUS "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})


# flatbuffers libs
set(FLATBUFFERS_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libflatbuffers)
set(FLATBUFFERS_LIBS "${FLATBUFFERS_LIBS_DIR}/libflatbuffers.so")

include_directories ("${PROJECT_SOURCE_DIR}/include")

# 构建驱动库
add_library(FlatbuffersMessage SHARED
            flat_buffers_message.cpp
            )

target_link_libraries(FlatbuffersMessage 
                    ${FLATBUFFERS_LIBS}
                    )

# 下边这三句没有尝试
# set(public_headers include/zmq.h include/zmq_utils.h)
# set(includedir ${prefix}/${CMAKE_INSTALL_INCLUDEDIR})
# PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT SDK)

# 将库文件，可执行文件，头文件安装到指定目录
install(TARGETS FlatbuffersMessage  
        DESTINATION lib 
        EXPORT flatbuffer_message # 并导出库信息
        )

install(DIRECTORY include/ DESTINATION include)

#  里用库信息生成 .cmake 文件 并安装到指定位置
install(EXPORT flatbuffer_message 
        FILE flatbuffer_message.cmake
        DESTINATION lib/cmake/flatbuffer_message
        )

## .cmake 信息
include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/flatbuffer_messageConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_messageConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/flatbuffer_message"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_messageConfigVersion.cmake"
  VERSION "${FlatbufferMessage_VERSION_MAJOR}.${FlatbufferMessage_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_messageConfig.cmake
  DESTINATION lib/cmake/flatbuffer_message
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/flatbuffer_messageConfigVersion.cmake
  DESTINATION lib/cmake/flatbuffer_message
)