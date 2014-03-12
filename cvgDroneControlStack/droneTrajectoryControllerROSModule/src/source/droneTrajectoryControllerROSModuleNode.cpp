//////////////////////////////////////////////////////
//  droneTrajectoryControllerROSModuleNode.cpp
//
//  Created on: Dec 11, 2013
//      Author: jespestana
//
//  Last modification on: Dec 11, 2013
//      Author: jespestana
//
//////////////////////////////////////////////////////

#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "droneTrajectoryControllerROSModule.h"
#include "communication_definition.h"

int main(int argc,char **argv) {

    for (int i=0; i<argc; i++) {
        std::cout << " argv[" << i << "]: " << argv[i] << std::endl;
    }
    //std::cout << "ROS_NAMESPACE:" << std::string(std::getenv("ROS_NAMESPACE")) << std::endl;

    //Ros Init
    ros::init(argc, argv, MODULE_NAME_TRAJECTORY_CONTROLLER);
    ros::NodeHandle n;

    std::cout << "[ROSNODE] Starting droneTrajectoryController" << std::endl;
    DroneTrajectoryControllerROSModule MyDroneTrajectoryController;
    MyDroneTrajectoryController.open(n,MODULE_NAME_TRAJECTORY_CONTROLLER);
    std::cout << "[ROSNODE] droneTrajectoryController started"  << std::endl;

    try {
        while(ros::ok()) {
            //Read messages
            ros::spinOnce();

            //Run EKF State Estimator
            if(MyDroneTrajectoryController.run()) {
            }
            //Sleep
            MyDroneTrajectoryController.sleep();
        }
    } catch (std::exception &ex) {
        std::cout << "[ROSNODE] Exception :" << ex.what() << std::endl;
    }
    return 1;
}
