#include <chrono>
#include <iostream>
#include <vector>
#include <thread>
#include <math.h>
#include <fstream>

// #include "ur_rtde/rtde_receive_interface.h"
// #include "ur_rtde/rtde_control_interface.h"


#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"


using drake::lcmt_schunk_wsg_command;
using drake::lcmt_schunk_wsg_status;



int main(int argc, char* argv[]){


    // Initialize LCM:
    lcm::LCM lcm;
    if(!lcm.good()){
        std::cout << "LCM Initialization Failed!" << std::endl;
        return 1;
    }
    std::cout << "LCM Initialization Success!" << std::endl;

    // Gripper Command Template:
    lcmt_schunk_wsg_command schunk_command;
    // schunk_command.target_position_mm = 38;
    schunk_command.target_position_mm = 1;
    schunk_command.force = 100;


    lcm.publish("SCHUNK_LEFT_COMMAND", &schunk_command);


    std::cout << "Script Ended..." << std::endl;

    return 0;
}