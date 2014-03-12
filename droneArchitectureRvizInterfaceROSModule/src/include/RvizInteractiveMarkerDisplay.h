#ifndef RVIZINTERACTIVEMARKERDISPLAY_H
#define RVIZINTERACTIVEMARKERDISPLAY_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <math.h>
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/obstaclesTwoDim.h"
#include "droneMsgsROS/dronePositionTrajectoryRefCommand.h"
#include "droneMsgsROS/dronePositionRefCommand.h"
#include <string>
#include <cstdlib>
#include <float.h>
#include <stdio.h>
#include "boost/ref.hpp"

using namespace visualization_msgs;
using namespace interactive_markers;

class DroneRvizDisplay
{

private:

    ros::Publisher obstacles_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher mission_pub;

    int idDrone;
    int id_O, id_2;
    float x1, x2, y, y2, sx, sy, rx, ry, yaw, yaw2;




protected:

    ros::NodeHandle n;

public:

    int ActiveDroneId();

    void ServerReset();
    void ServerResetNew();
    void MenuHandlerApply();
    void ServerApplyChanges();

    void initMenu(int i);
    void makeAxesMenu(std::string name);
    void makeMovingMarker(const tf::Vector3& position , std::string name, std::string frame_id_in);

    void PoseCallback(const droneMsgsROS::dronePose &pose_euler, int idDrone);

    void ObstaclesPubCallback(const droneMsgsROS::obstaclesTwoDim obstacles, int idDrone);
    void ObstaclesPubCallbackAdd(const droneMsgsROS::obstaclesTwoDim Drone_obstacles, std::string name);
    void ObstaclesPubCallbackDelete(const droneMsgsROS::obstaclesTwoDim Drone_obstacles, std::string name);

    void MissionPointPubCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone);
    void MissionPointPubCallbackAdd(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone, std::string name);
    void MissionPointPubCallbackDelete(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone, std::string name);

    void TrajectoryPubCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone);
    void TrajectoryPubCallbackAdd(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone, std::string name);
    void TrajectoryPubCallbackDelete(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone, std::string name);


};



#endif
