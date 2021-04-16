cmake_minimum_required(VERSION 3.5)

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(candle SHARED uart.c candle.cpp)
target_include_directories (candle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(candle PROPERTIES PUBLIC_HEADER candle.hpp)
