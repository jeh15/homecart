cmake_minimum_required(VERSION 3.9)
project(ur_home_script)

# Include standard install locations:
include(GNUInstallDirs)

# Import library target `ur_rtde`
find_package(ur_rtde REQUIRED)
add_executable(ur_home_script ur_home_script.cpp cmake-build-debug/test.cpp cmake-build-debug/test.h ur_home_script.h InputParser.cpp InputParser.h)
target_link_libraries(ur_home_script PRIVATE ur_rtde::rtde)