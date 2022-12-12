include(FetchContent)

set(CMAKE_BUILD_TYPE Debug)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.11.0
)
FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)

include_directories(candle_test ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(GTest::GTest INTERFACE gtest_main gmock gmock_main)

add_executable(candle_test test.cpp ${CMAKE_SOURCE_DIR}/src/candle.cpp)

target_link_libraries(candle_test
    PRIVATE
    GTest::GTest
    candle)

add_test(candle_test candle_test)