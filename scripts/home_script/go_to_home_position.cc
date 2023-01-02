#include <chrono>
#include <iostream>
#include <vector>
#include <threads.h>
#include <math.h>
#include <fstream>

#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_control_interface.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main(int argc, char* argv[]){
    // Load JSON:
    // TODO(jeh15): Can I somehow use relative paths?
    std::string path = "/home/orl/repository/homecart/scripts/home_script/home_position.json";
    std::ifstream file(path);
    json data = json::parse(file);

    // Initialize Control Interface and Connect to UR SIM:
    ur_rtde::RTDEControlInterface rtde_control("192.168.4.30");
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.4.30");

    // // Check Initialization:
    std::cout << "Controller and Receiver Initialized!" << std::endl;

    // // Check Connection:
    std::cout << "Is Connected: " << rtde_control.isConnected() << std::endl;

    // Move to pose from JSON:
    std::vector<double> pose = data["pose"];
    rtde_control.moveL(pose);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Stop Procedure:
    rtde_control.servoStop();
    std::cout << "Servo Ended..." << std::endl;
    rtde_control.stopScript();
    std::cout << "Script Ended..." << std::endl;

    return 0;
}