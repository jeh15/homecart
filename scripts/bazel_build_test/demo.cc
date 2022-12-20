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

// Linear Pose Function: Translates End Effector Z position by linear_step in the positve Z direction
std::vector<double> get_target(const std::vector<double>& pose, double linear_step=0.1){
    std::vector<double> trajectory_target = pose;
    trajectory_target[2] = pose[2] + linear_step;
    return trajectory_target; 
}

int main(int argc, char* argv[]){
    // Initialize Control Interface and Connect to UR SIM:
    ur_rtde::RTDEControlInterface rtde_control("192.168.4.30");
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.4.30");

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

    // Sleep and wait for gripper (2s)
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Setup UR Control Parameters:
    constexpr auto target_freq = std::chrono::milliseconds(2);
    std::vector<double> initial_pose = rtde_receive.getActualTCPPose();

    // Grab Block:
    schunk_command.target_position_mm = 10;
    schunk_command.force = 10;
    lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);

    // Sleep and wait for gripper (1s)
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Move to Target Position:
    std::vector<double> init_pose = get_target(initial_pose);
    rtde_control.moveL(init_pose);
    std::this_thread::sleep_for(target_freq);

    // Hold block for:
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Drop Block:
    schunk_command.target_position_mm = 100;
    schunk_command.force = 10;
    lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);

    // Sleep and wait for gripper (1s)
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Stop Procedure:
    rtde_control.servoStop();
    std::cout << "Servo Ended..." << std::endl;
    rtde_control.stopScript();
    std::cout << "Script Ended..." << std::endl;

    return 0;
}