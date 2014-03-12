//////////////////////////////////////////////////////
//  droneSimulatorROSModuleNode.cpp
//
//  Created on: Jan 22, 2014
//      Author: joselusl
//
//  Last modification on: Jan 22, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////



//I/O stream
//std::cout
#include <iostream>


//ROS
#include "ros/ros.h"

//parrotARDrone
#include "droneSimulatorROSModule.h"

//Comunications
#include "nodes_definition.h"



using namespace std;




int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, MODULE_NAME_DRONE_SIMULATOR);
    ros::NodeHandle n;

    cout<<"[ROSNODE] Starting droneSimulator"<<endl;

    //Vars
    DroneSimulatorROSModule MyDroneSimulatorROSModule;
    MyDroneSimulatorROSModule.open(n,MODULE_NAME_DRONE_SIMULATOR);

    try
    {
        while(ros::ok())
        {
            //Read messages
            ros::spinOnce();

            //run
            MyDroneSimulatorROSModule.run();

            //Sleep
            MyDroneSimulatorROSModule.sleep();

        }
        return 1;

    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}
