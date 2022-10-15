cmake_minimum_required(VERSION 3.9)
project(scripts/gripper_bazel_test/)

# Include standard install locations:
include(GNUInstallDirs)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Ugly way to switch build targets: TO DO find better method
set(name "demo")

# Import library target `ur_rtde`
find_package(ur_rtde REQUIRED)
add_executable(${name}  ${name}.cc)
target_link_libraries(${name} PRIVATE ur_rtde::rtde)

# Import library target `drake`
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
target_link_libraries(${name} PRIVATE drake::drake)