cmake_minimum_required(VERSION 3.9)
project(ur_home_script)

# Include standard install locations:
include(GNUInstallDirs)

# Set Variable
set(name "ur_home_script")

# Import library target `ur_rtde`
find_package(ur_rtde REQUIRED)
add_executable(${name}  ${name}.cpp)
target_link_libraries(${name} PRIVATE ur_rtde::rtde)