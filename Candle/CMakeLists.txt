cmake_minimum_required(VERSION 3.5)

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(candle SHARED usbDevice.cpp candle.cpp)
target_include_directories (candle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(candle pthread)
set_target_properties(candle PROPERTIES PUBLIC_HEADER candle.hpp)

add_executable(canTest test.cpp)
target_link_libraries(canTest candle)
