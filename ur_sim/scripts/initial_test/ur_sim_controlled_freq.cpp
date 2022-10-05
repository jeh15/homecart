#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <threads.h>

int main(int argc, char* argv[]){
    // Initialize Control Interface and Connect to UR SIM:
    ur_rtde::RTDEControlInterface rtde_control("172.17.0.1");
    ur_rtde::RTDEReceiveInterface rtde_receive("172.17.0.1");

    // Check Initialization:
    std::cout << "Controller and Receiver Initialized!" << std::endl;

    // Check Connection:
    std::cout << "Is Connected: " << rtde_control.isConnected() << std::endl;

    // Parameters:
    double velocity = 0.1;
    double acceleration = 0.1;
    double dt = 1.0 / 500; // 2ms
    double lookahead_time = 0.1;
    double gain = 100;
    constexpr auto target_freq = std::chrono::milliseconds(2);
    std::vector<double> joint_q = rtde_receive.getActualQ();

    // Move Joints:
    for(unsigned int i=0; i<1000; i++){
        //
        auto start = std::chrono::high_resolution_clock::now();

        // Control Commands
        rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
        joint_q[0] += 0.001;
        joint_q[1] += 0.001;
        std::cout << "Iteration: " << i << std::endl;

        //
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> control_regulation = target_freq - (end - start);

        // Regulate Control Frequency:
        if(control_regulation.count() > 0){
            std::cout << "Control Regulation: "<< control_regulation.count() << std::endl;
            std::this_thread::sleep_for(control_regulation);
        }
        else{
            std::cout << "Over run... Terminating Script..." << std::endl;
            break;
        }
    }

    // Stop Procedure:
    rtde_control.servoStop();
    std::cout << "Servo Ended..." << std::endl;
    rtde_control.stopScript();
    std::cout << "Script Ended..." << std::endl;

    return 0;
}