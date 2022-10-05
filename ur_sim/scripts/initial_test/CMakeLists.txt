cmake_minimum_required(VERSION 3.9)
project(ur_sim/scripts/)

# Include standard install locations:
include(GNUInstallDirs)

# Ugly way to switch build targets: TO DO find better method
# set(name "ur_sim_home_script")
# set(name "ur_sim_home_script")
set(name "ur_sim_controlled_freq")

# Import library target `ur_rtde`
find_package(ur_rtde REQUIRED)
add_executable(${name}  ${name}.cpp)
target_link_libraries(${name} PRIVATE ur_rtde::rtde)