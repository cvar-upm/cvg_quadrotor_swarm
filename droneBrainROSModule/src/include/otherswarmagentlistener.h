#ifndef OTHERSWARMAGENTLISTENER_H
#define OTHERSWARMAGENTLISTENER_H

// ROS
#include "ros/ros.h"

// C++ standar libraries
#include <cstdlib>
#include <stdio.h>
#include <iostream>

#include "communication_definition.h"
#include "std_msgs/Bool.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneInfo.h"

#define OTHER_SWARM_AGENT_LAN_TIMEOUT_THRESHOLD (2.0)

class OtherSwarmAgentListener {
protected:
    ros::NodeHandle n;
public:
    int idDrone;
    OtherSwarmAgentListener(int idDrone_in);
    ~OtherSwarmAgentListener();
    void open(ros::NodeHandle & nIn);
    droneMsgsROS::droneInfo getDroneInfo();
    droneMsgsROS::dronePose getEstimatedPose();
    bool isOnline();
    bool isInTheSystem();

private:
    ros::Subscriber     isInTheSystemSub;
    bool                swarmAgentIsOnline;
    ros::Time           last_isOnline_timestamp;
    bool                last_isInTheSystem;
    void isOnlineSubCallback(const std_msgs::Bool::ConstPtr &msg);

    ros::Subscriber      estimatedPoseSub;
public:
    droneMsgsROS::droneInfo last_droneInfo;
private:
    ros::Time            last_estimatedPose_timestamp;
    void estPoseSubCallback(const droneMsgsROS::dronePose::ConstPtr &msg);
};

#endif // OTHERSWARMAGENTLISTENER_H
