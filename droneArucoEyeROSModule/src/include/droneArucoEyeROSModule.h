//////////////////////////////////////////////////////
//  droneArucoEyeROSModule.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Jan 15, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////


#ifndef _DRONE_ARUCO_EYE_ROS_MODULE_H
#define _DRONE_ARUCO_EYE_ROS_MODULE_H




//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>

//Vector
//std::vector
#include <vector>



//Opencv
#include <opencv2/opencv.hpp>


//Aruco
#include "aruco.h"
//Aruco JL Lib
#include "droneArucoEye.h"


//ROS
#include "ros/ros.h"


//Drone Module
#include "droneModuleROS.h"

//Drone Msgs
#include "droneMsgsROS/obsVector.h"


//ROS Images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>



#include "referenceFrames.h"

#include "communication_definition.h"



///Consts
const double DRONE_ARUCO_EYE_RATE = 30.0;



#define VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE



/////////////////////////////////////////
// Class droneArucoEyeROSModule
//
//   Description
//
/////////////////////////////////////////
class DroneArucoEyeROSModule : public DroneModule
{
    //ArucoRetina
protected:
    DroneArucoEye MyDroneArucoEye;

    //Images received
protected:
    //bool subscribe_to_rectified_front_cam;

    //Front image msgs
    cv_bridge::CvImagePtr cvDroneImage;
    //ros::Time frontImage_timesamp;
    //uint32_t frontImage_seq;
    cv::Mat droneImage;
    //Subscriber
    ros::Subscriber droneFrontImageSubs;
    void droneImageCallback(const sensor_msgs::ImageConstPtr& msg);

    //Arucos detected
protected:
    ros::Publisher droneArucoListPubl; ////Publishers
    droneMsgsROS::obsVector droneArucoListMsg; //Messages
    bool publishArucoList();

    //Constructors and destructors
public:
    DroneArucoEyeROSModule();
    ~DroneArucoEyeROSModule();

    //Init and close
public:
    void init();
    void close();

    //Open
 public:
    void open(ros::NodeHandle & nIn, std::string moduleName);

    //Reset
protected:
    bool resetValues();

    //Start
protected:
    bool startVal();

    //Stop
protected:
    bool stopVal();

    //Run
public:
    bool run();

    //Drawing
public:
    int drawArucoCodes(std::string windowName, int waitingTime=1, bool drawDetectedCodes=true, bool draw3DReconstructedCodes=true);

};




#endif
