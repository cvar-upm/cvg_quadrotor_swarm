//I/O stream
//std::cout
#include <iostream>
//ROS
#include "ros/ros.h"
//Aruco Eye
#include "RvizInterfaceROSModule.h"
//Communication definition
#include "communication_definition.h"

#include "RvizInteractiveMarkerDisplay.h"

using namespace std;


int main(int argc,char **argv)
{
    //Ros Init

    ros::init(argc, argv, MODULE_NAME_DRONE_ARCHITECTURE_RVIZ_INTERFACE);
    ros::NodeHandle n;

    DroneRvizDisplay drone;

    cout<<"[ROSNODE] Starting "<<MODULE_NAME_DRONE_ARCHITECTURE_RVIZ_INTERFACE<<endl;

    //Vars
    DroneArchitectureRvizInterfaceROSModule MyDroneRvizInterface;
    MyDroneRvizInterface.open( n, MODULE_NAME_DRONE_ARCHITECTURE_RVIZ_INTERFACE);
    MyDroneRvizInterface.start();



    try
    {
        while(ros::ok())
        {
            //Read messages
            ros::spinOnce();

            //Run retina
            if(MyDroneRvizInterface.run())
            {
                //Sleep
                MyDroneRvizInterface.sleep();
            }
        }
        return 1;

    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;

    }
    ROS_INFO("ok?");
    drone.ServerReset();
}
