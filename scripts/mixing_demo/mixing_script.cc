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

// Circle Pose Function:
std::vector<double> get_target(const std::vector<double>& pose, double timestep, double radius=0.05, double freq=1.0){
    std::vector<double> trajectory_target = pose;
    trajectory_target[0] = pose[0] + radius * cos((2 * M_PI * freq * timestep));
    trajectory_target[1] = pose[1] + radius * sin((2 * M_PI * freq * timestep));
    return trajectory_target; 
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
    schunk_command.target_position_mm = 60;
    schunk_command.force = 10;

    // Set Gripper to known state: Left Arm -> "SCHUNK_WSG_COMMAND" : Right Arm -> "SCHUNK_WSG_COMMAND_EXTRA"
    lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);
    lcm.publish("SCHUNK_WSG_COMMAND_EXTRA", &schunk_command);

    // Sleep and wait for gripper (2s)
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Store Poses in Vector:
    std::vector<std::vector<double>> right_pose;
    std::vector<std::vector<double>> left_pose;

    // Store all right poses in vector: (1: Initial, 2: Up, 3: Down, 4: Turn)
    for(int i = 1; i <= 4; i++){
        std::string path = "/home/orl/repository/homecart/scripts/mixing_demo/right_" + std::to_string(i) + ".json";
        std::ifstream file(path);
        json data = json::parse(file);
        right_pose.push_back(data["pose"]);
    }

    // Store all left poses in vector: (1: Initial, 2: Pick Up, 3: Up, 4: Down, 5: Intermediate 6: Intermediate-2)
    for(int i = 1; i <= 6; i++){
        std::string path = "/home/orl/repository/homecart/scripts/mixing_demo/left_" + std::to_string(i) + ".json";
        std::ifstream file(path);
        json data = json::parse(file);
        left_pose.push_back(data["pose"]);
    }

    // Control Parameters:
    double speed = 0.5;
    int move_freq = 10;
    int grip_freq = 1000;

    // Move to Initial Position:
    right_control.moveL(right_pose[0]);
    left_control.moveL(left_pose[0]);
    std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

    // Move over Spoon:
    right_control.moveL(right_pose[1]);
    std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

    // Move down to Spoon:
    right_control.moveL(right_pose[2]);
    std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

    // Right Arm Grab Spoon:
    schunk_command.target_position_mm = 10;
    schunk_command.force = 10;
    lcm.publish("SCHUNK_WSG_COMMAND_EXTRA", &schunk_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(grip_freq));

    // Move to up position:
    right_control.moveL(right_pose[1]);
    std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

    // Move to turn position:
    right_control.moveL(right_pose[3]);
    left_control.moveL(left_pose[1]);
    std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

    // Left Arm Grab Spoon:
    schunk_command.target_position_mm = 10;
    schunk_command.force = 10;
    lcm.publish("SCHUNK_WSG_COMMAND", &schunk_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(grip_freq));

    // Right Arm Release Spoon:
    schunk_command.target_position_mm = 100;
    schunk_command.force = 10;
    lcm.publish("SCHUNK_WSG_COMMAND_EXTRA", &schunk_command);
    std::this_thread::sleep_for(std::chrono::milliseconds(grip_freq));

    // Left Arm Move To Bowl:
    left_control.moveL(left_pose[4]);
    left_control.moveL(left_pose[5]);
    left_control.moveL(left_pose[2]);

    // Right Arm Idle:
    right_control.moveL(right_pose[0]);
    std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

    // Left Arm Move Down To Bowl:
    left_control.moveL(left_pose[3]);
    std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));

    // Parameters:
    double velocity = 0.1;
    double acceleration = 0.1;
    double dt = 1.0 / 500; // 2ms
    double lookahead_time = 0.1;
    double gain = 500;
    constexpr auto target_freq = std::chrono::milliseconds(2);
    std::vector<double> initial_pose = left_pose[3];

    // Move to Initial Position:
    double time_counter = 0.0;
    std::vector<double> pose = get_target(initial_pose, time_counter);
    left_control.moveL(pose);
    std::this_thread::sleep_for(target_freq);

    // Follow Circle Trajectory:
    for(size_t i = 0; i < 5000; i++){
        auto start = std::chrono::high_resolution_clock::now();

        // Control Commands:
        pose = get_target(initial_pose, time_counter);
        left_control.servoL(pose, velocity, acceleration, dt, lookahead_time, gain);
        time_counter += dt;

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> control_regulation = target_freq - (end - start);

        // Regulate Control Frequency:
        if(control_regulation.count() > 0){
            std::this_thread::sleep_for(control_regulation);
        }
        else{
            std::cout << "Over run... Terminating Script..." << std::endl;
            break;
        }
    }

    //TODO(jeh15): Give high-level control back
    std::this_thread::sleep_for(std::chrono::milliseconds(move_freq));
    // Move Back to Center of Bowl:
    left_control.moveL(left_pose[3]);

    // Stop Procedure:
    right_control.servoStop();
    left_control.servoStop();
    std::cout << "Servo Ended..." << std::endl;
    right_control.stopScript();
    left_control.stopScript();
    std::cout << "Script Ended..." << std::endl;

    return 0;
}