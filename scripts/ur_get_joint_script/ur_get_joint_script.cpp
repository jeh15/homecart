#include <ur_rtde/rtde_receive_interface.h>
#include <chrono>
#include <iostream>
#include <vector>

int main(int argc, char* argv[]){
    // Conect to LEFT arm:
    ur_rtde::RTDEReceiveInterface rtde_receive_left("192.168.4.30");

    // Conect to Right arm:
    ur_rtde::RTDEReceiveInterface rtde_receive_right("192.168.5.30");

    // Get Joint Positions:
    std::vector<double> left_joint = rtde_receive_left.getActualQ();
    std::vector<double> right_joint = rtde_receive_right.getActualQ();

    // Stream out Joint Positions to Command Line:
    std::cout << "Left joints:" << std::endl;
    for (const auto& i: left_joint){
        std::cout << i * 180.0 / 3.14 << '\n'; 
    }

    std::cout << "Right joints:" << std::endl;
    for (const auto& i: right_joint){
        std::cout << i * 180.0 / 3.14 << '\n'; 
    }

    return 0;
}