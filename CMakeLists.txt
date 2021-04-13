cmake_minimum_required(VERSION 3.5)
project(md80)


add_compile_options(-Wall -Wextra -Wpedantic)

add_subdirectory(Candle)
add_subdirectory(Md80)
add_subdirectory(Example)
# include_directories("./")
# include_directories("./include/")
# include_directories("./include/utils")

# file(GLOB sources "src/*.cpp" "src/*.c" "src/utils/*.cpp")

# add_executable(${PROJECT_NAME}
#                 ${sources})

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)