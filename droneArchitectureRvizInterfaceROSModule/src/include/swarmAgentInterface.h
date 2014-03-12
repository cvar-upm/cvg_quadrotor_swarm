#ifndef SWARMAGENTRVIZINTERFACE_H
#define SWARMAGENTRVIZINTERFACE_H

// ROS
#include "ros/ros.h"
// C++ standar libraries
#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <string>
// Quadrotor Stack
#include "communication_definition.h"
// Msgs
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/dronePositionRefCommand.h"
#include "droneMsgsROS/dronePositionTrajectoryRefCommand.h"
#include "droneMsgsROS/obstaclesTwoDim.h"
// (Opcional) Pose de los otros drones (publicada por droneBrain):
#include "droneMsgsROS/societyPose.h"

#include "RvizInteractiveMarkerDisplay.h"


// TODO: Obstaculos (escritos en el archivo de configuración del trajectory planner):
// preguntar a JL o JP, en archivo xml:
// ${DRONE_STACK}/configs/drone${idDrone}/configFile.xml
// Suelo (escritos en el archivo de configuración del trajectory planner):
// ${DRONE_STACK}/configs/drone${idDrone}/configFile.xml


class SwarmAgentInterface
{

protected:

    ros::NodeHandle n;
    int idDrone;

public:

    SwarmAgentInterface(int idDrone_in);
    ~SwarmAgentInterface();
    void open(ros::NodeHandle & nIn);

private:

    DroneRvizDisplay Drone;
    ros::Subscriber localizer_pose_subscriber;
    ros::Subscriber mission_planner_mission_point_reference_subscriber;
    ros::Subscriber trajectory_planner_trajectory_reference_subscriber;
    ros::Subscriber obstacle_processor_obstacle_list_subscriber;
    ros::Subscriber this_drone_society_pose_subscriber;

    void localizerPoseCallback(const droneMsgsROS::dronePose &pose_euler);
    void missionPointCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand &point);
    void trajectoryAbsRefCmdCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand &trajectory);
    void obstacleListCallback(const droneMsgsROS::obstaclesTwoDim &obstacles);
    void societyPoseSubCallback(const droneMsgsROS::societyPose::ConstPtr &msg);

};

#endif // SWARMAGENTRVIZINTERFACE_H




