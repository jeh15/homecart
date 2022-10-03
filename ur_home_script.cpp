#include <ur_home_script.h>
#include "InputParser.h"

int main(int argc, char* argv[]){
    std::string robot_ip;

    InputParser input(argc, argv);
    if(input.cmdOptionExists("--robot_ip")){
        robot_ip = input.getCmdOption("--robot_ip");
    }
    else{
        robot_ip = "0.0.0.0";
    }

    // Initialize Control Interface:
    RTDEControlInterface rtde_control(robot_ip);

    // Parameters:
    double velocity = 0.1;
    double acceleration = 0.1;
    double dt = 1.0 / 500; // 2ms
    double lookahead_time = 0.1;
    double gain = 100;
    std::vector<double> joint_q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Move to Home Configuration:
    rtde_control.ServoJ(joint_q);

    // Stop Procedure:
    rtde_control.servoStop();
    rtde_control.stopScript();

    return 0;
}




