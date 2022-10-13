cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wall -Wextra -Wpedantic)

file(GLOB CANDLE_SOURCE "src/*cpp")

if(DEFINED CANDLE_BUILD_STATIC)
    set(LIB_TYPE STATIC)
else()
    set(LIB_TYPE SHARED)
endif()
message("CANDLE BUILT TYPE: ${LIB_TYPE}")

add_library(candle ${LIB_TYPE} ${CANDLE_SOURCE})

target_include_directories (candle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(candle pthread)
ament_target_dependencies(candle rclcpp sensor_msgs)
set_target_properties(candle PROPERTIES PUBLIC_HEADER candle.hpp)

# if(NOT DEFINED CANDLE_OMIT_EXAMPLES)
#     add_executable(example1 examples/example1.cpp)
#     target_link_libraries(example1 candle)

#     add_executable(example2 examples/example2.cpp)
#     target_link_libraries(example2 candle)

#     add_executable(example3 examples/example3.cpp)
#     target_link_libraries(example3 candle)

#     add_executable(example4 examples/example4.cpp)
#     target_link_libraries(example4 candle)

#     add_executable(example5 examples/example5.cpp)
#     target_link_libraries(example5 candle)

#     add_executable(example6 examples/example6.cpp)
#     target_link_libraries(example6 candle)

#     add_executable(example7 examples/example7.cpp)
#     target_link_libraries(example7 candle)

#     add_executable(example8 examples/example8.cpp)
#     target_link_libraries(example8 candle)

#     add_executable(example9 examples/example9.cpp)
#     target_link_libraries(example9 candle)
# endif()

if(${LIB_TYPE} MATCHES "SHARED")
    message("Generating project template at ${CMAKE_BINARY_DIR}/Project/")
    add_custom_command(
        TARGET candle
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy $<TARGET_FILE:candle> ../Project/libs/libcandle.so
        )
    add_custom_command(
        TARGET candle
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy ${CMAKE_HOME_DIRECTORY}/Candle/cmake_template.txt ../Project/CMakeLists.txt
        )
    add_custom_command(
        TARGET candle
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy_directory ${CMAKE_HOME_DIRECTORY}/Candle/include/ ../Project/include/
        )
    add_custom_command(
        TARGET candle
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy ${CMAKE_HOME_DIRECTORY}/Candle/examples/example1.cpp ../Project/src/main.cpp
        )
    message(STATUS "Generation complete!")
endif()
