//////////////////////////////////////////////////////
//  droneObstacleProcessorROSModuleNode.cpp
//
//  Created on: Jul 29, 2013
//      Author: pdelapuente
//
//  Last modification on: Oct 29, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////





//I/O stream
//std::cout
#include <iostream>


// ROS
#include "ros/ros.h"

//Drone Obstacle processor ROS module
#include "droneObstacleProcessorROSModule.h"

//communication_definition
#include "communication_definition.h"



using namespace std;


int main(int argc, char **argv)
{
    //Init ros
    ros::init(argc, argv,MODULE_NAME_OBSTACLE_PROCESSOR);
  	ros::NodeHandle n;

    //Init
    cout<<"[ROSNODE] Starting Drone ObstacleProcessor..."<<endl;

    //Init module
    DroneObstacleProcessorROSModule MyDroneObstacleProcessor;
    MyDroneObstacleProcessor.open(n,MODULE_NAME_OBSTACLE_PROCESSOR);

    //Loop
    while(ros::ok())
    {
        //Read ros messages
        ros::spinOnce();
        //Run
        MyDroneObstacleProcessor.run();
        //Sleep
        MyDroneObstacleProcessor.sleep();
    }
    return 1;
}
