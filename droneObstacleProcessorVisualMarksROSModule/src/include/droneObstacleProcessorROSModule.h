//////////////////////////////////////////////////////
//  droneObstacleProcessorROSModule.h
//
//  Created on: Jul 29, 2013
//      Author: pdelapuente
//
//  Last modification on: Oct 29, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////

#ifndef DRONE_OBSTACLE_PROCESSOR_ROS_MODULE_H
#define DRONE_OBSTACLE_PROCESSOR_ROS_MODULE_H





//Math
#include <math.h>


#include <cstdlib>
#include <ctime>



#include <iostream>

#include <fstream>

#include <vector>

#include <sstream>


#include "droneObstacleProcessor.h"


#include "obstacleprocessor.h"
#include "obstacle.h"
#include "rectangle.h"
#include "circle.h"



////// ROS  ///////
#include "ros/ros.h"


//Drone module
#include "droneModuleROS.h"



////Msgs
//drone pose
#include "droneMsgsROS/obstaclesTwoDim.h"
#include "droneMsgsROS/obstacleTwoDimWall.h"
#include "droneMsgsROS/obstacleTwoDimPole.h"
#include "droneMsgsROS/landmarkVector.h"

#include "communication_definition.h"


//Freq
const double FREQ_OBSTACLEPROCESSOR = 30.0;



//#define DRONE_OBSTACLES_LOGGING


/////////////////////////////////////////
// Class DroneObstacleProcessorROSModule
//
//   Description
//
/////////////////////////////////////////
class DroneObstacleProcessorROSModule : public DroneModule
{	
protected:
     DroneObstacleProcessor MyDroneObstacleProcessor;

     //std::vector<Obstacle2D> obstacles;
     //std::vector<Landmark3D> landmarks;

private:
    ros::Subscriber landmarksSubs; //Subscriber
    void landmarksCallback(const droneMsgsROS::landmarkVector::ConstPtr& msg);

private:
    ros::Publisher obstaclesPubl; //Publisher
    int publishObstacles(droneMsgsROS::obstaclesTwoDim obstacles);

public:
    DroneObstacleProcessorROSModule();
    ~DroneObstacleProcessorROSModule();

public:
    void open(ros::NodeHandle & nIn, std::string moduleName);
	void close();

#ifdef DRONE_OBSTACLES_LOGGING
protected:
    void logLandmarks(const std::vector<Landmark3D>& landmarks);
    std::ofstream mylog;
    ros::Duration run_timestamp;
#endif

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

};





#endif
