//////////////////////////////////////////////////////
//  droneLocalizerNode.h
//
//  Created on: Jul 29, 2013
//      Author: pdelapuente & joselusl
//
//  Last modification on: Oct 27, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////





//I/O Stream
//std::cout
#include <iostream>



// ROS
//ros::init(), ros::NodeHandle, ros::ok(), ros::spinOnce()
#include "ros/ros.h"


//Drone Localizer
#include "droneLocalizerROSModule.h"


//Communication Definition
//MODULE_NAME_LOCALIZER
#include "communication_definition.h"



using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,MODULE_NAME_LOCALIZER); //Say to ROS the name of the node and the parameters
  	ros::NodeHandle n;

    //Init
    std::cout<<"[ROSNODE] Starting Drone Localizer"<<std::endl;


    //Simulator
    DroneLocalizer MyDroneLocalizer;
    MyDroneLocalizer.open(n,MODULE_NAME_LOCALIZER);


    //Loop
    while(ros::ok())
    {
        //Read ros messages
        ros::spinOnce();

        if(!MyDroneLocalizer.run())
        {
            //cout<<"[ROSNODE] Error while running"<<endl;
        }
        //Sleep
        MyDroneLocalizer.sleep();
    }
    return 1;
}
