//////////////////////////////////////////////////////
//  droneEKFStateEstimatorROSModuleNode.cpp
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
#include "droneEKFStateEstimatorROSModule.h"
#include "communication_definition.h"

int main(int argc,char **argv) {
    //Ros Init
    ros::init(argc, argv, MODULE_NAME_ODOMETRY_STATE_ESTIMATOR);
    ros::NodeHandle n;

    std::cout << "[ROSNODE] Starting droneEKFStateEstimator" << std::endl;
    DroneEKFStateEstimatorROSModule MyDroneEKFStateEstimator;
    MyDroneEKFStateEstimator.open(n,MODULE_NAME_ODOMETRY_STATE_ESTIMATOR);
    std::cout << "[ROSNODE] droneEKFStateEstimator started"  << std::endl;

    try {
        while(ros::ok())
        {
            //Read messages
            ros::spinOnce();

            //Run EKF State Estimator
            if(MyDroneEKFStateEstimator.run())
            {
            }
            //Sleep
            MyDroneEKFStateEstimator.sleep();
        }

    } catch (std::exception &ex) {
        std::cout << "[ROSNODE] Exception :" << ex.what() << std::endl;
    }
    return 1;
}
