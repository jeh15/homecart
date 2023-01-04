#include <chrono>
#include <iostream>
#include <vector>
#include <threads.h>
#include <math.h>
#include <fstream>

#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_control_interface.h"

#include <nlohmann/json.hpp>

#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

using json = nlohmann::json;
using drake::lcmt_schunk_wsg_command;
using drake::lcmt_schunk_wsg_status;

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
    schunk_command.target_position_mm = 110;
    schunk_command.force = 10;

    // Set Gripper to known state:
    lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);

    // Sleep and wait for gripper (2s)
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Store Poses in Vector:
    std::vector<std::vector<double>> pose_up;
    std::vector<std::vector<double>> pose_down;
    std::vector<std::vector<double>> pose_ground;

    // Go to initial positon:
    std::string path = "/home/orl/repository/homecart/scripts/demo_script/initial_position.json";
    std::ifstream file(path);
    json data = json::parse(file);
    std::vector<double> initial_pose = data["pose"];
    rtde_control.moveL(initial_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Load Poses: (UP)
    for(int i = 1; i <= 3; i++){
        std::string path = "/home/orl/repository/homecart/scripts/demo_script/block_" + std::to_string(i) + "_up.json";
        std::ifstream file(path);
        json data = json::parse(file);
        pose_up.push_back(data["pose"]);
    }

    // Load Poses: (DOWN)
    for(int i = 1; i <= 3; i++){
        std::string path = "/home/orl/repository/homecart/scripts/demo_script/block_" + std::to_string(i) + "_down.json";
        std::ifstream file(path);
        json data = json::parse(file);
        pose_down.push_back(data["pose"]);
    }

    // Load Poses: (GROUND)
    for(int i = 1; i <= 3; i++){
        std::string path = "/home/orl/repository/homecart/scripts/demo_script/ground_" + std::to_string(i) + ".json";
        std::ifstream file(path);
        json data = json::parse(file);
        pose_ground.push_back(data["pose"]);
    }

    // Control Parameters:
    double speed = 0.5;
    int move_freq = 500;
    int grip_freq = 1000;
    // Block Stacking Control Loop:
    for(size_t i = 0; i < pose_up.size(); i++){
        // Move over block:
        rtde_control.moveL(pose_up[i], speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

        // Move to grab block:
        rtde_control.moveL(pose_down[i], speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

        // Grab Block:
        schunk_command.target_position_mm = 10;
        schunk_command.force = 10;
        lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);
        std::this_thread::sleep_for(std::chrono::milliseconds(grip_freq));

        // Move back up:
        rtde_control.moveL(pose_up[i], speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

        // Go to stacking area:
        rtde_control.moveL(initial_pose, speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

        if(i == pose_up.size()-1){
            break;
        }

        // Place block at stacking area:
        rtde_control.moveL(pose_ground[i], speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

        // Release Block:
        schunk_command.target_position_mm = 110;
        schunk_command.force = 10;
        lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);
        std::this_thread::sleep_for(std::chrono::milliseconds(grip_freq));

        // Back to initial position:
        rtde_control.moveL(initial_pose, speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));
    }

    // for(size_t i = 0; i < 1; i++){
    //     std::cout << "Pose Up: ";
    //     for(auto j: pose_up[i]){
    //         std::cout << j << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << "Pose Down: ";
    //     for(auto j: pose_down[i]){
    //         std::cout << j << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << "Pose Up: ";
    //     for(auto j: pose_up[i]){
    //         std::cout << j << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << "Initial Position: ";
    //     for(auto j: initial_pose){
    //         std::cout << j << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << "Ground Pose: ";
    //     for(auto j: pose_ground[i]){
    //         std::cout << j << " ";
    //     }
    //     std::cout << std::endl;
    // }


    // Stop Procedure:
    rtde_control.servoStop();
    std::cout << "Servo Ended..." << std::endl;
    rtde_control.stopScript();
    std::cout << "Script Ended..." << std::endl;

    return 0;
}