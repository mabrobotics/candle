cmake_minimum_required(VERSION 3.5)

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(md80 SHARED md80.cpp)

target_include_directories (md80 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(md80 candle pthread)
set_target_properties(md80 PROPERTIES PUBLIC_HEADER md80.hpp)
