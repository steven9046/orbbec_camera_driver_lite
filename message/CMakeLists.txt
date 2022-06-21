cmake_minimum_required(VERSION 2.8)
project(FlatbuffersMessage)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set (FlatbufferMessage_VERSION_MAJOR 1)
set (FlatbufferMessage_VERSION_MINOR 0)

configure_file (
"${PROJECT_SOURCE_DIR}/FlatbuffersMessageConfig.h.in"
"${PROJECT_BINARY_DIR}/FlatbuffersMessageConfig.h"
)

# 设置安装路径 /home/ss/orbbec_camera_driver/install
get_filename_component(TOP_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)
set(CMAKE_INSTALL_PREFIX ${TOP_DIR}/install/FlatbuffersMessage)

# flatbuffers libs
set(FLATBUFFERS_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libflatbuffers)
set(FLATBUFFERS_LIBS "${FLATBUFFERS_LIBS_DIR}/libflatbuffers.so")


# 构建驱动库
add_library(FlatbuffersMessage SHARED
            flat_buffers_message.cpp
            )

target_link_libraries(FlatbuffersMessage 
                    ${FLATBUFFERS_LIBS}
                    )

target_include_directories (FlatbuffersMessage PUBLIC 
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
                            )

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY libs/libflatbuffers/ DESTINATION lib)

# 将库文件，可执行文件，头文件安装到指定目录
install(TARGETS FlatbuffersMessage
        EXPORT FlatbuffersMessage # 并导出库信息
        DESTINATION lib 
        )

install(EXPORT FlatbuffersMessage 
        FILE FlatbuffersMessage.cmake
        DESTINATION lib/cmake/FlatbuffersMessage
        )

## .cmake 信息
include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/FlatbuffersMessageConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/FlatbuffersMessageConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/FlatbuffersMessage"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/FlatbuffersMessageConfigVersion.cmake"
  VERSION "${FlatbufferMessage_VERSION_MAJOR}.${FlatbufferMessage_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/FlatbuffersMessageConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/FlatbuffersMessageConfigVersion.cmake
  DESTINATION lib/cmake/FlatbuffersMessage
)