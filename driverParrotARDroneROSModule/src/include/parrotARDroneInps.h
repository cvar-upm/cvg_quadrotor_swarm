//////////////////////////////////////////////////////
//  parrotARDroneInps.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Dec 23, 2013
//      Author: joselusl
//
//  Description: file to manage ardrone_autonomy node
//  See: https://github.com/AutonomyLab/ardrone_autonomy
//
//////////////////////////////////////////////////////


#ifndef _PARROT_ARDRONE_INPS_H
#define _PARROT_ARDRONE_INPS_H


#include "droneModuleROS.h"
#include "communication_definition.h"



////Magnetometer and RotationAngles
//#include "geometry_msgs/Vector3Stamped.h"

////Ground Speed
//#include "droneMsgs/vector2Stamped.h"

////Pressure
//#include "sensor_msgs/FluidPressure.h"

////Drone Status
//#include "droneMsgs/droneStatus.h"




/////// OpenCV ///////
//#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
//#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
//#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur




//// ROS  ///////
#include "ros/ros.h"



///// AR DRONE AUTONOMY /////
////Services
//#include "std_srvs/Empty.h"

//#include "ardrone_autonomy/CamSelect.h"
//#include "ardrone_autonomy/LedAnim.h"
//#include "ardrone_autonomy/FlightAnim.h"

////Topics
//
//#include "std_msgs/Float32.h"
//#include "std_msgs/String.h"

////images
//#include <cv_bridge/cv_bridge.h>



//#include <sensor_msgs/image_encodings.h>



//DroneCommand
#include <geometry_msgs/Twist.h>

//Empty
#include "std_msgs/Empty.h"

//Custom commands
#include "droneMsgsROS/dronePitchRollCmd.h"
#include "droneMsgsROS/droneDYawCmd.h"
#include "droneMsgsROS/droneDAltitudeCmd.h"

#include "droneMsgsROS/droneCommand.h"



//I/O stream
//std::cout
#include <iostream>

//Math
//M_PI
#include <cmath>




/////////////////////////////////////////
// Class DroneCommand
//
//   Description
//
/////////////////////////////////////////
class DroneCommandROSModule : public DroneModule
{

    //Publishers
protected:
    //Movement commands
    ros::Publisher CommandOutPubl;
    bool publishCommandValue();

    //Take off
    ros::Publisher TakeOffPub;
    bool publishTakeOff();

    //Land
    ros::Publisher LandPub;
    bool publishLand();

    //Reset
    ros::Publisher ResetPub;
    bool publishReset();



    //Subscribers
protected:
    ros::Subscriber PitchRollSubs;
    void pitchRollCallback(const droneMsgsROS::dronePitchRollCmd::ConstPtr& msg);

    ros::Subscriber AltitudeSubs;
    void dAltitudeCallback(const droneMsgsROS::droneDAltitudeCmd::ConstPtr& msg);

    ros::Subscriber YawSubs;
    void dYawCallback(const droneMsgsROS::droneDYawCmd::ConstPtr& msg);

    ros::Subscriber CommandSubs;
    void commandCallback(const droneMsgsROS::droneCommand::ConstPtr& msg);


    //Command msgs out
protected:
    geometry_msgs::Twist CommandOutMsgs;

    //HL Commands
protected:
    std_msgs::Empty EmptyMsg;


    //Constructors and destructors
public:
    DroneCommandROSModule();
    ~DroneCommandROSModule();


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


};



#endif
