//////////////////////////////////////////////////////
//  droneArucoEyeROSModuleNode.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Jan 15, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////



//I/O stream
//std::cout
#include <iostream>


//Opencv
#include <opencv2/opencv.hpp>

//ROS
#include "ros/ros.h"

//Aruco Eye
#include "droneArucoEyeROSModule.h"

//Communication definition
#include "communication_definition.h"



using namespace std;


#define DISPLAY_ARUCO_EYE



int main(int argc,char **argv)
{
	//Ros Init
    ros::init(argc, argv, MODULE_NAME_ARUCO_EYE);
  	ros::NodeHandle n;
  	
    cout<<"[ROSNODE] Starting "<<MODULE_NAME_ARUCO_EYE<<endl;
  	
  	//Vars
    DroneArucoEyeROSModule MyDroneArucoEye;
    MyDroneArucoEye.open(n,MODULE_NAME_ARUCO_EYE);

#ifdef DISPLAY_ARUCO_EYE
    //Name
    std::string arucoEyeWindow="droneArucoEye";
    //Create gui
    cv::namedWindow(arucoEyeWindow,1);
#endif
  	
	try
	{
        while(ros::ok())
		{
            //Read messages
            ros::spinOnce();

            //Run retina
            if(MyDroneArucoEye.run())
            {
#ifdef DISPLAY_ARUCO_EYE
                //Draw
                MyDroneArucoEye.drawArucoCodes(arucoEyeWindow,1,true,true);
#endif
            }

            //Sleep
            MyDroneArucoEye.sleep();
		}
        return 1;

    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}
