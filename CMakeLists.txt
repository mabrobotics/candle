cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 20)

project(md80)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wall -Wextra -Wpedantic)

if(ARCH STREQUAL "armhf")
    message("[CANDLE] Compiling for armhf")
    set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
    set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
elseif(ARCH STREQUAL "aarm64")
    message("[CANDLE] Compiling for aarm64")
    set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
    set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
endif()

file(GLOB CANDLE_SOURCE "src/*cpp")

if(CANDLE_BUILD_STATIC)
    set(CANDLE_BUILD_STATIC "TRUE")
    set(CANDLE_BUILD_SHARED "FALSE")
    add_library(candle STATIC ${CANDLE_SOURCE})
else()
    set(CANDLE_BUILD_STATIC "FALSE")
    set(CANDLE_BUILD_SHARED "TRUE")
    add_library(candle SHARED ${CANDLE_SOURCE})
endif()	

message(STATUS "\nCANDLE_BUILD_SHARED: ${CANDLE_BUILD_SHARED}\nCANDLE_BUILD_STATIC: ${CANDLE_BUILD_STATIC}")

target_include_directories (candle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(candle pthread)
set_target_properties(candle PROPERTIES PUBLIC_HEADER candle.hpp)

if(NOT DEFINED CANDLE_OMIT_EXAMPLES)
    add_executable(example1 examples_cpp/example1.cpp)
    target_link_libraries(example1 candle)

    add_executable(example2 examples_cpp/example2.cpp)
    target_link_libraries(example2 candle)

    add_executable(example3 examples_cpp/example3.cpp)
    target_link_libraries(example3 candle)

    add_executable(example4 examples_cpp/example4.cpp)
    target_link_libraries(example4 candle)

    add_executable(example5 examples_cpp/example5.cpp)
    target_link_libraries(example5 candle)

    add_executable(example6 examples_cpp/example6.cpp)
    target_link_libraries(example6 candle)

    add_executable(example7 examples_cpp/example7.cpp)
    target_link_libraries(example7 candle)

    add_executable(example8 examples_cpp/example8.cpp)
    target_link_libraries(example8 candle)

    add_executable(example9 examples_cpp/example9.cpp)
    target_link_libraries(example9 candle)
    
    add_executable(example10 examples_cpp/example10.cpp)
    target_link_libraries(example10 candle)

    add_executable(example11 examples_cpp/example11.cpp)
    target_link_libraries(example11 candle)

endif()


