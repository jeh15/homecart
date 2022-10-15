#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_control_interface.h"

#include <chrono>
#include <iostream>
#include <vector>
#include <threads.h>
#include <math.h>

#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

using drake::lcmt_schunk_wsg_command;
using drake::lcmt_schunk_wsg_status;

// Linear Pose Function:
std::vector<double> get_target(const std::vector<double>& pose, double linear_step=0.1){
    std::vector<double> trajectory_target = pose;
    trajectory_target[2] = pose[2] + linear_step;
    return trajectory_target; 
}

using namespace ur_rtde;
int main(int argc, char* argv[]){
    // Initialize Control Interface and Connect to UR SIM:
    ur_rtde::RTDEControlInterface rtde_control("172.17.0.2");
    ur_rtde::RTDEReceiveInterface rtde_receive("172.17.0.2");

    // Check Initialization:
    std::cout << "Controller and Receiver Initialized!" << std::endl;

    // Check Connection:
    std::cout << "Is Connected: " << rtde_control.isConnected() << std::endl;

    // Initialize LCM:
    lcm::LCM lcm;
    if(!lcm.good()){
        std::cout << "LCM Initialization Failed!" << std::endl;
        return 1;
    }
    std::cout << "LCM Initialization Success!" << std::endl;

    // Gripper Command Template:
    lcmt_schunk_wsg_command schunk_command;
    schunk_command.target_position_mm = 100;
    schunk_command.force = 10;

    // Set Gripper to known state:
    lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);

    // Setup UR Control Parameters:
    double velocity = 0.1;
    double acceleration = 0.1;
    double dt = 1.0 / 500; // 2ms
    double lookahead_time = 0.1;
    double gain = 500;
    constexpr auto target_freq = std::chrono::milliseconds(2);
    std::vector<double> initial_pose = rtde_receive.getActualTCPPose();

    // Grab Block:
    schunk_command.target_position_mm = 10;
    schunk_command.force = 10;
    lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);

    // Move to Target Position:
    std::vector<double> init_pose = get_target(initial_pose);
    rtde_control.moveL(init_pose);
    std::this_thread::sleep_for(target_freq);

    // Stop Procedure:
    rtde_control.servoStop();
    std::cout << "Servo Ended..." << std::endl;
    rtde_control.stopScript();
    std::cout << "Script Ended..." << std::endl;

    return 0;
}