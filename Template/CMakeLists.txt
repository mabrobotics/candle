cmake_minimum_required(VERSION 3.5)
project(template)

include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_SOURCE_DIR}/lib)


file(GLOB sources "src/*.cpp" "src/*.c")

add_executable(${PROJECT_NAME}
                ${sources})

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC md80 candle)

