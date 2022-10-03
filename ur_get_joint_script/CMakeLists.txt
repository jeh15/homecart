cmake_minimum_required(VERSION 3.9)
project(ur_get_joint_script)

# Include standard install locations:
include(GNUInstallDirs)

# Import library target `ur_rtde`
find_package(ur_rtde REQUIRED)
add_executable(ur_get_joint_script ur_get_joint_script.cpp)
target_link_libraries(ur_get_joint_script PRIVATE ur_rtde::rtde)