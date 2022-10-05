#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <threads.h>
#include <math.h>

// Circle Pose Function:
std::vector<double> get_circle_target(const std::vector<double>& pose, double timestep, double radius=0.01, double freq=1.0){
    std::vector<double> trajectory_target = pose;
    trajectory_target[0] = pose[0] + radius * cos((2 * M_PI * freq * timestep));
    trajectory_target[1] = pose[1] + radius * sin((2 * M_PI * freq * timestep));
    return trajectory_target; 
}

int main(int argc, char* argv[]){
    // Initialize Control Interface and Connect to UR SIM:
    ur_rtde::RTDEControlInterface rtde_control("172.17.0.2");
    ur_rtde::RTDEReceiveInterface rtde_receive("172.17.0.2");

    // Check Initialization:
    std::cout << "Controller and Receiver Initialized!" << std::endl;

    // Check Connection:
    std::cout << "Is Connected: " << rtde_control.isConnected() << std::endl;

    // Parameters:
    double velocity = 0.1;
    double acceleration = 0.1;
    double dt = 1.0 / 100; // 10ms
    double lookahead_time = 0.1;
    double gain = 600;
    constexpr auto target_freq = std::chrono::milliseconds(10);
    std::vector<double> initial_pose = rtde_receive.getActualTCPPose();

    // Move to Initial Position:
    double time_counter = 0.0;
    std::vector<double> init_pose = get_circle_target(initial_pose, time_counter);
    rtde_control.moveL(init_pose, velocity, acceleration);
    std::this_thread::sleep_for(target_freq);

    // Follow Circle Trajectory:
    for(unsigned int i=0; i<2500; i++){
        //
        auto start = std::chrono::high_resolution_clock::now();

        // Control Commands:
        std::vector<double> servo_target = get_circle_target(initial_pose, time_counter);
        rtde_control.servoL(servo_target, velocity, acceleration, dt, lookahead_time, gain);
        time_counter += dt;

        //
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> control_regulation = target_freq - (end - start);

        // Regulate Control Frequency:
        if(control_regulation.count() > 0){
            std::cout << "Control Regulation: "<< control_regulation.count() << std::endl;
            std::this_thread::sleep_for(control_regulation);
        }
        else{
            std::cout << "Control Regulation: "<< control_regulation.count() << std::endl;
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