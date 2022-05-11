cmake_minimum_required(VERSION 3.5)
project(md80)

add_compile_options(-Wall -Wextra -Wpedantic)

if(DEFINED CANDLE_BUILD_PYTHON AND DEFINED CANDLE_BUILD_STATIC)
    message(FATAL_ERROR "CANDLE_BUILD_PYTHON only valid with SHARED build type!")
endif()    

add_subdirectory(Candle)

if(DEFINED CANDLE_BUILD_PYTHON)
    add_subdirectory(pyCandle)
endif()

message(STATUS "\nCANDLE_BUILD_STATIC: ${CANDLE_BUILD_STATIC}\nCANDLE_BUILD_PYTHON: ${CANDLE_BUILD_PYTHON}")