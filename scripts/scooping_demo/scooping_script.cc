#include <chrono>
#include <iostream>
#include <vector>
#include <thread>
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

void go_to(ur_rtde::RTDEControlInterface& control_interface, const std::vector<double> pose){
    control_interface.moveL(pose);
}


int main(int argc, char* argv[]){
    // Left Arm:
    ur_rtde::RTDEControlInterface left_control("192.168.4.30");
    ur_rtde::RTDEReceiveInterface left_receive("192.168.4.30");

    // Right Arm:
    ur_rtde::RTDEControlInterface right_control("192.168.5.30");
    ur_rtde::RTDEReceiveInterface right_receive("192.168.5.30");

    // Check Initialization:
    std::cout << "Controller and Receiver Initialized!" << std::endl;

    // Check Connection:
    std::cout << "Right Arm is Connected: " << right_control.isConnected() << std::endl;
    std::cout << "Left Arm is Connected: " << left_control.isConnected() << std::endl;

    // Initialize LCM:
    lcm::LCM lcm;
    if(!lcm.good()){
        std::cout << "LCM Initialization Failed!" << std::endl;
        return 1;
    }
    std::cout << "LCM Initialization Success!" << std::endl;

    // Gripper Command Template:
    lcmt_schunk_wsg_command schunk_command;
    schunk_command.target_position_mm = 30;
    schunk_command.force = 10;

    // Set Gripper to known state: Left Arm -> "SCHUNK_LEFT_COMMAND" : Right Arm -> "SCHUNK_RIGHT_COMMAND"
    lcm.publish("SCHUNK_LEFT_COMMAND", &schunk_command);
    lcm.publish("SCHUNK_RIGHT_COMMAND", &schunk_command);

    // Thread Test:
    std::vector<std::vector<double>> right_pose;
    std::vector<std::vector<double>> left_pose;

    for(int i = 1; i <= 5; i++){
        std::string path = "/home/orl/repository/homecart/scripts/scooping_demo/right_" + std::to_string(i) + ".json";
        std::ifstream file(path);
        json data = json::parse(file);
        right_pose.push_back(data["pose"]);
    }

    for(int i = 1; i <= 4; i++){
        std::string path = "/home/orl/repository/homecart/scripts/scooping_demo/left_" + std::to_string(i) + ".json";
        std::ifstream file(path);
        json data = json::parse(file);
        left_pose.push_back(data["pose"]);
    }

    
    // Run control on multipel threads
    {
        std::thread thread_right(go_to, std::ref(right_control), right_pose[0]);
        std::thread thread_left(go_to, std::ref(left_control), left_pose[0]);

        thread_right.join();
        thread_left.join();
    }

    {
        std::thread thread_right(go_to, std::ref(right_control), right_pose[1]);
        std::thread thread_left(go_to, std::ref(left_control), left_pose[0]);

        thread_right.join();
        thread_left.join();
    }

    {
        std::thread thread_right(go_to, std::ref(right_control), right_pose[2]);
        std::thread thread_left(go_to, std::ref(left_control), left_pose[1]);

        thread_right.join();
        thread_left.join();
    }

    {
        std::thread thread_right(go_to, std::ref(right_control), right_pose[3]);
        std::thread thread_left(go_to, std::ref(left_control), left_pose[2]);

        thread_right.join();
        thread_left.join();
    }

    {
        std::thread thread_right(go_to, std::ref(right_control), right_pose[4]);
        std::thread thread_left(go_to, std::ref(left_control), left_pose[3]);

        thread_right.join();
        thread_left.join();
    }

    // Stop Procedure:
    right_control.servoStop();
    left_control.servoStop();
    std::cout << "Servo Ended..." << std::endl;
    right_control.stopScript();
    left_control.stopScript();
    std::cout << "Script Ended..." << std::endl;

    return 0;
}