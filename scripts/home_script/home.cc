#include <chrono>
#include <iostream>
#include <vector>
#include <threads.h>
#include <math.h>

#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_control_interface.h"

#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

#include <json/json.h>
#include <json/value.h>

using drake::lcmt_schunk_wsg_command;
using drake::lcmt_schunk_wsg_status;

// Linear Pose Function: Translates End Effector Z position by linear_step in the positve Z direction
std::vector<double> set_home(const std::vector<double>& pose){
    std::vector<double> home_pose = pose;
    return home_pose; 
}

int main(int argc, char* argv[]){
    // Script for going to home position or Setting new home position:
    
    if(argc > 1){
        // Set new pose position as home:

    }

    

    return 0;
}