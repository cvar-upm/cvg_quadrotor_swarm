//////////////////////////////////////////////////////
//  DroneTrajectoryPlannerROSModule.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 27, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////

#ifndef DRONE_TRAJECTORY_PLANNER_ROS_MODULE_H
#define DRONE_TRAJECTORY_PLANNER_ROS_MODULE_H




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


// ROS
#include "ros/ros.h"



//Drone module
#include "droneModuleROS.h"


//Drone Trajectory planner
#include "droneTrajectoryPlanner.h"




////Msgs
//drone pose
#include "droneMsgsROS/dronePose.h"


//point fin
#include "droneMsgsROS/dronePositionRefCommand.h"

//Trajectory
#include "droneMsgsROS/dronePositionTrajectoryRefCommand.h"


//Obstacles msgs
#include "droneMsgsROS/obstacleTwoDimPole.h"
#include "droneMsgsROS/obstacleTwoDimWall.h"

#include "droneMsgsROS/obstaclesTwoDim.h"


//Others drone pose
#include "droneMsgsROS/societyPose.h"


#include "communication_definition.h"





//#define DRONE_TRAJECTORY_PLANNER_LOGGING

//Freq
const double FREQ_TRAJ_PLANNER  =  20.0; //Hz


const double OTHERS_QR_RADIUS  =  1.0; //m


//#define DRONE_TRAJECTORY_PLANNER_SAVE_LOGS



//#define _VERBOSE_DRONE_TRAJECTORY_PLANNER


/////////////////////////////////////////
// Class DroneTrajectoryPlannerROSModule
//
//   Description
//
/////////////////////////////////////////
class DroneTrajectoryPlannerROSModule : public DroneModule
{	
    //DroneTrajectoryPlanner
protected:
    DroneTrajectoryPlanner* MyDroneTrajectoryPlanner;
    //bool flagTrajectoryFound;
    TrajectoryPlanner::Result trajectoryPlannerResult;

    //Trajectory
protected:
    ros::Publisher dronePositionTrajectoryRefCommandPubl;
    virtual bool publishTrajectory();

    //point fin
protected:
    ros::Subscriber dronePositionPointFinRefCommandSubs;

    //pose (point init)
protected:
    ros::Subscriber dronePoseSubs;

    //Obstacles
protected:
    ros::Subscriber obstaclesSubs;

    //Other drones
protected:
    ros::Subscriber societyPoseSubs;


    //Logger
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
protected:
    std::ofstream mylog;
    ros::Duration run_timestamp;
#endif

public:
    DroneTrajectoryPlannerROSModule();
    ~DroneTrajectoryPlannerROSModule();
	
public:
    virtual void open(ros::NodeHandle & nIn, std::string moduleName);
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

public:
    TrajectoryPlanner::Result resultTrajectoryPlanner();

    int getTrajectory(std::vector< std::vector<double> > &trajectoryOut);
};



/////////////////////////////////////////
// Class DroneTrajectoryPlanner2dROSModule
//
//   Description
//
/////////////////////////////////////////
class DroneTrajectoryPlanner2dROSModule : public DroneTrajectoryPlannerROSModule
{

public:
    DroneTrajectoryPlanner2dROSModule();
    ~DroneTrajectoryPlanner2dROSModule();

public:
    void open(ros::NodeHandle & nIn, std::string moduleName);

    //Trajectory
protected:
    droneMsgsROS::dronePositionTrajectoryRefCommand dronePositionTrajectoryRefCommandMsg; //Messages
public:
    bool publishTrajectory();

    //point fin
protected:
    std::vector<double> pointFin;
    void dronePositionPointFinRefCommandCallback(const droneMsgsROS::dronePositionRefCommand::ConstPtr& msg);

    //pose (point init)
protected:
    void dronePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg);

    //Obstacles
protected:
    void obstaclesCallback(const droneMsgsROS::obstaclesTwoDim::ConstPtr& msg);

    //Other drones
protected:
    void societyPoseCallback(const droneMsgsROS::societyPose::ConstPtr& msg);


    //Virtual Obstacles of the trajectory planner
protected:


};



/////////////////////////////////////////
// Class DroneTrajectoryPlanner3dROSModule
//
//   Description TODO PABLO!!
//
/////////////////////////////////////////
class DroneTrajectoryPlanner3dROSModule : public DroneTrajectoryPlannerROSModule
{

};



#endif
