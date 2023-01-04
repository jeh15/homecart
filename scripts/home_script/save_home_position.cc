#include <chrono>
#include <iostream>
#include <vector>
#include <threads.h>
#include <math.h>
#include <ctime>
#include <fstream>
#include <filesystem>

#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_control_interface.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main(int argc, char* argv[]){
    // Saves Current Pose as "Home Position" and serializes to a JSON:

    // Initialize Control Interface and Connect to UR SIM:
    // Left arm:
    ur_rtde::RTDEControlInterface rtde_control("192.168.4.30");
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.4.30");

    // Right arm:
    // ur_rtde::RTDEControlInterface rtde_control("192.168.5.30");
    // ur_rtde::RTDEReceiveInterface rtde_receive("192.168.5.30");

    // Check Initialization:
    std::cout << "Controller and Receiver Initialized!" << std::endl;

    // Check Connection:
    std::cout << "Is Connected: " << rtde_control.isConnected() << std::endl;

    // Get Position relative to End-Effector in Cartesian Space: [x, y, z, rx, ry, rz]
    std::vector<double> pose = rtde_receive.getActualTCPPose();

    // Get Joint Positions:
    std::vector<double> joint_position = rtde_receive.getActualQ();

    // Make JSON
    json j;
    j["pose"] = pose;
    j["joint_position"] = joint_position;

    // Save "home_position.json" to bazel-bin output folder:
    std::ofstream file("home_position.json");
    file << j;
    file.close();

    // Stop Procedure:
    rtde_control.servoStop();
    std::cout << "Servo Ended..." << std::endl;
    rtde_control.stopScript();
    std::cout << "Script Ended..." << std::endl;

    return 0;
}