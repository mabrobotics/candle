cmake_minimum_required(VERSION 3.5)
project(md80)

add_compile_options(-Wall -Wextra -Wpedantic)

# Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

if(DEFINED CANDLE_BUILD_PYTHON)
    set(CANDLE_BUILD_STATIC "TRUE")
    add_subdirectory(Candle)
    add_subdirectory(pyCandle)
else()
    add_subdirectory(Candle)
endif()	


message(STATUS "\nCANDLE_BUILD_STATIC: ${CANDLE_BUILD_STATIC}\nCANDLE_BUILD_PYTHON: ${CANDLE_BUILD_PYTHON}")
