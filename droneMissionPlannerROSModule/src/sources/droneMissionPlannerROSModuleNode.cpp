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






#include "droneMissionPlannerROSModule.h"




using namespace std;



int main(int argc, char **argv)
{

    ros::init(argc, argv, MODULE_NAME_MISSION_PLANNER);
  	ros::NodeHandle n;




    //Init
    cout<<"Starting drone Mission Planner..."<<endl;






    //Simulator
    DroneMissionPlanner MyDroneMissionPlanner;
    MyDroneMissionPlanner.open(n,MODULE_NAME_MISSION_PLANNER);




    //Loop
    while(ros::ok())
    {

        //double timeInitAStar=ros::Time::now().toSec();


        //Read ros messages
        ros::spinOnce();


        //cout<<"Loop"<<endl;



        MyDroneMissionPlanner.run();

        MyDroneMissionPlanner.sleep();


    }




    return 1;

}
