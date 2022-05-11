cmake_minimum_required(VERSION 3.5)

add_compile_options(-Wall -Wextra -Wpedantic)

project(pybindtest)

add_subdirectory(pybind)

if(NOT TARGET candle)
    add_library(candle SHARED IMPORTED)
    set_property(TARGET candle PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/Candle/libcandle.so) #TODO: verify if this 'if' is needed at all 
endif()

pybind11_add_module(pyCandle main.cpp)

target_link_libraries(pyCandle PUBLIC candle)
target_include_directories(pyCandle PUBLIC ${CMAKE_SOURCE_DIR}/include)
