cmake_minimum_required(VERSION 3.20)
project(robot_kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED NO_MODULE)

# biblioteca de cinematica
add_library(kinematics src/robot.cpp)
target_include_directories(kinematics PUBLIC include)
target_link_libraries(kinematics PUBLIC Eigen3::Eigen)
target_compile_options(kinematics PRIVATE -Wall -Wextra -Wpedantic)

# program demonstrativ
add_executable(demo examples/main.cpp)
target_link_libraries(demo PRIVATE kinematics)

# teste (GoogleTest se descarca automat la prima configurare)
include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(tests tests/test_forward.cpp tests/test_inverse.cpp)
target_link_libraries(tests PRIVATE kinematics GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(tests)
