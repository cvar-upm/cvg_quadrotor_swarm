//////////////////////////////////////////////////////
//  DroneTrajectoryPlannerROSModule.h
//
//  Created on: Jul 29, 2013
//      Author: pdelapuente & joselusl
//
//  Last modification on: Oct 27, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#ifndef DRONE_LOCALIZER_H
#define DRONE_LOCALIZER_H



//I/O stream
//std::cout
#include <iostream>

//Vector
//std::vector
#include <vector>

//String
//std::string, std::getline()
#include <string>

//String stream
//std::istringstream
#include <sstream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>



//
#include "Kalman_Loc2.h"


// ROS
#include "ros/ros.h"


//Drone module
#include "droneModuleROS.h"


#include "referenceFramesROS.h"



////Msgs
//drone pose
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/landmarkVector.h"
#include "droneMsgsROS/obsVector.h"

#include "communication_definition.h"

#include "referenceFramesROS.h"
#include "xmlfilereader.h"
#include "cvg_utils_library.h"
#include "control/FilteredDerivative.h"
#include "droneMsgsROS/droneSpeeds.h"


//Freq
const double FREQ_LOCALIZER = 30.0;


#define DRONE_LOCALIZER_LOGGING



/////////////////////////////////////////
// Class DroneLocalizer
//
//   Description: PALOMA
//
/////////////////////////////////////////
class DroneLocalizer : public DroneModule
{	
    //Trajectory
private:
    //Subscriber
    ros::Subscriber droneOdomPoseSubs;
    void droneOdomPoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg);
    
    ros::Subscriber droneObsVectorSubs;
    void droneObsVectorCallback(const droneMsgsROS::obsVector::ConstPtr& msg);


    //drone pose
private:
    //Publisher
    ros::Publisher dronePosePubl;
    int publishPose(droneMsgsROS::dronePose dronePoseEstimate);

    ros::Publisher droneSpeedPubl;
    int publishSpeeds(droneMsgsROS::droneSpeeds droneSpeedEstimate);
    
    ros::Publisher dronePoseNewAngNotationPubl;
    int publishPoseNewAngNotation(droneMsgsROS::dronePose dronePoseEstimateNewAngNotation);
    
    ros::Publisher mapPubl;
    int publishMap(droneMsgsROS::landmarkVector map);


public:
    DroneLocalizer();
    ~DroneLocalizer();
	
    void open(ros::NodeHandle & nIn, std::string moduleName);
	void close();

protected:
    bool init();


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
    
    void logLocalizerMsgStr();
    std::ofstream mylog;
    ros::Duration run_timestamp;

protected:
    Kalman_Loc loc;
	 
    float x_odom;
    float y_odom;
    float z_odom;
    float roll_odom;
    float pitch_odom;
    float yaw_odom;
    
    float x_predicted;
    float y_predicted;
    float z_predicted;
    float roll_predicted;
    float pitch_predicted;
    float yaw_predicted;
    
    std::vector<Observation3D> obs;
    
    bool new_odom_data;
    bool new_obs_vector;

protected:
    CVG_BlockDiagram::FilteredDerivative filter_x2vx;
    CVG_BlockDiagram::FilteredDerivative filter_y2vy;
    CVG_BlockDiagram::FilteredDerivative filter_z2dz;
    CVG_BlockDiagram::FilteredDerivative filter_yaw2dyaw;
    CVG_BlockDiagram::FilteredDerivative filter_pitch2dpitch;
    CVG_BlockDiagram::FilteredDerivative filter_roll2droll;
};






#endif
