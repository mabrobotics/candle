cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic)

file(GLOB CANDLE_SOURCE "src/*cpp")

# candle can be compiled both as STATIC or SHARED. User may choose what they prefer
# add_library(candle STATIC ${CANDLE_SOURCE})
add_library(candle SHARED ${CANDLE_SOURCE})

target_include_directories (candle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(candle pthread)
set_target_properties(candle PROPERTIES PUBLIC_HEADER candle.hpp)

add_executable(example1 examples/example1.cpp)
target_link_libraries(example1 candle)

add_executable(example2 examples/example2.cpp)
target_link_libraries(example2 candle)

add_executable(example3 examples/example3.cpp)
target_link_libraries(example3 candle)

add_executable(example4 examples/example4.cpp)
target_link_libraries(example4 candle)

add_executable(example5 examples/example5.cpp)
target_link_libraries(example5 candle)
