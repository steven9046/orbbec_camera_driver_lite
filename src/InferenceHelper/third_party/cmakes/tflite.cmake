set(TFLITE_INC
    ${CMAKE_CURRENT_LIST_DIR}/../tensorflow
    ${CMAKE_CURRENT_LIST_DIR}/../tensorflow/tensorflow/lite/tools/make/downloads/flatbuffers/include
    ${CMAKE_CURRENT_LIST_DIR}/../tensorflow/tensorflow/lite/tools/make/downloads/absl
)

# if(DEFINED  ANDROID_ABI)
#     # set(TFLITE_LIB ${CMAKE_CURRENT_LIST_DIR}/../tensorflow_prebuilt/android/${ANDROID_ABI}/libtensorflowlite.so)
#     add_library(TFLITE SHARED IMPORTED GLOBAL)
#     set_target_properties(
#         TFLITE
#         PROPERTIES IMPORTED_LOCATION
#         ${CMAKE_CURRENT_LIST_DIR}/../tensorflow_prebuilt/android/${ANDROID_ABI}/libtensorflowlite.so
#     )
#     set(TFLITE_LIB TFLITE)
# elseif(MSVC_VERSION)
#     set(TFLITE_LIB ${CMAKE_CURRENT_LIST_DIR}/../tensorflow_prebuilt/x64_windows/libtensorflowlite.so.if.lib)
#     file(COPY ${CMAKE_CURRENT_LIST_DIR}/../tensorflow_prebuilt/x64_windows/libtensorflowlite.so DESTINATION ${CMAKE_BINARY_DIR})
# else()
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(TFLITE_LIB ${CMAKE_CURRENT_LIST_DIR}/../tensorflow_prebuilt/x64_linux/libtensorflowlite.so)
    # elseif(${BUILD_SYSTEM} STREQUAL "armv7")
    #     set(TFLITE_LIB ${CMAKE_CURRENT_LIST_DIR}/../tensorflow_prebuilt/armv7/libtensorflowlite.so)
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(TFLITE_LIB ${CMAKE_CURRENT_LIST_DIR}/../tensorflow_prebuilt/aarch64/libtensorflowlite.so)
    else()	
        message(FATAL_ERROR "[tflite] unsupported platform")
    endif()
    file(COPY ${TFLITE_LIB} DESTINATION ${CMAKE_BINARY_DIR})
# endif()
