/*
*
*
*
*
*/


/// ROS
#include "ros/ros.h"


#include <stdio.h>
#include <iostream>

//trajectory planner
#include "droneYawCommanderROSModule.h"

#include "communication_definition.h"







using namespace std;



int main(int argc, char **argv)
{

    ros::init(argc, argv, MODULE_NAME_YAW_PLANNER);
    ros::NodeHandle n; //Este nodo admite argumentos!!


//    int idDrone;
//    bool result_b = ros::param::get("~droneId",idDrone);

//    std::string stackPath;
//    ros::param::get("~stackPath",stackPath);

//    cout<<"idDrone="<<idDrone<<endl;
//    cout<<"stackPath="<<stackPath<<endl;


    //Init
    cout<<"Starting Drone Yaw Commander..."<<endl;


    //Yaw planner
    DroneYawCommander MyDroneYawCommander;
    MyDroneYawCommander.open(n,MODULE_NAME_YAW_PLANNER);


    //Loop
    while(ros::ok())
    {
        //Read ros messages
        ros::spinOnce();


        if(!MyDroneYawCommander.run())
        {
            //cout<<"!Error running"<<endl;
        }
        else
        {
            //Do nothing
        }

        MyDroneYawCommander.sleep();

    }

    return 1;

}

