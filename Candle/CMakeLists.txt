cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(candle SHARED src/usbDevice.cpp src/candle.cpp src/md80.cpp)
target_include_directories (candle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(candle pthread)
set_target_properties(candle PROPERTIES PUBLIC_HEADER candle.hpp)

add_executable(example1 examples/test.cpp)
target_link_libraries(example1 candle)

add_custom_command(TARGET candle POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:candle> ${PROJECT_BINARY_DIR}/Project/libs/libcandle.so
    )  
add_custom_command(TARGET candle POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Candle/include/candle_shared.hpp ${PROJECT_BINARY_DIR}/Project/include/candle.hpp
    )   
add_custom_command(TARGET candle POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Candle/include/mab_types.hpp ${PROJECT_BINARY_DIR}/Project/include/mab_types.hpp
    )    
add_custom_command(TARGET candle POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Candle/include/md80_shared.hpp ${PROJECT_BINARY_DIR}/Project/include/md80.hpp
    )  
add_custom_command(TARGET candle POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Candle/examples/project.cpp ${PROJECT_BINARY_DIR}/Project/src/main.cpp
    )     
add_custom_command(TARGET candle POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Candle/cmake_template.txt ${PROJECT_BINARY_DIR}/Project/CMakeLists.txt
    )               
add_custom_command(TARGET example1 POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:example1> ${PROJECT_BINARY_DIR}/examples/example1
    )  