#include <ur_rtde/rtde_receive_interface.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <math.h>

int main(int argc, char* argv[]){
    // Conect to UR SIM arm:
    ur_rtde::RTDEReceiveInterface rtde_receive("172.17.0.2");

    // Get Joint Positions:
    std::vector<double> joint_positions = rtde_receive.getActualQ();

    // Stream out Joint Positions to Command Line:
    std::cout << "Joints:" << std::endl;
    for (const auto& i: joint_positions){
        std::cout << i * 180.0 / M_PI << '\n'; 
    }

    return 0;
}