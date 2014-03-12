#include "otherswarmagentlistener.h"

OtherSwarmAgentListener::OtherSwarmAgentListener( int idDrone_in) {
    last_isOnline_timestamp      = ros::Time( 0, 0);
    last_estimatedPose_timestamp = ros::Time( 0, 0);
    swarmAgentIsOnline = false;
    last_isInTheSystem = false;
    last_droneInfo.id = idDrone_in;
    last_droneInfo.pose.x =   0.0;
    last_droneInfo.pose.y = -10.0;
    last_droneInfo.pose.z =   0.0;
    last_droneInfo.pose.yaw   = 90.0*(M_PI/180.0);
    last_droneInfo.pose.pitch = 0.0;
    last_droneInfo.pose.roll  = 0.0;
    idDrone = idDrone_in;
    return;
}

OtherSwarmAgentListener::~OtherSwarmAgentListener() {
    return;
}

void OtherSwarmAgentListener::open(ros::NodeHandle & nIn) {
    n = nIn;

    std::ostringstream convert;
    convert<<idDrone;
//    convert << std::setfill('0') << std::setw(2) << idDrone; // maybe we should consider doing this
    std::string idDrone_str = convert.str();

    isInTheSystemSub      = n.subscribe(std::string("/drone")+idDrone_str+"/"+DRONE_ARCHITECTURE_BRAIN_IS_IN_THE_SYSTEM_SUBSCRIBER, 1, &OtherSwarmAgentListener::isOnlineSubCallback, this);
    estimatedPoseSub = n.subscribe(std::string("/drone")+idDrone_str+"/"+DRONE_BRAIN_POSE_SUBSCRIBER, 1, &OtherSwarmAgentListener::estPoseSubCallback, this);
}

droneMsgsROS::dronePose OtherSwarmAgentListener::getEstimatedPose() {
    return last_droneInfo.pose;
}

bool OtherSwarmAgentListener::isOnline() {
    ros::Time current_time = ros::Time::now();
    if ( ((current_time - last_isOnline_timestamp).toSec() > OTHER_SWARM_AGENT_LAN_TIMEOUT_THRESHOLD) && last_isInTheSystem ) {
        swarmAgentIsOnline = false;
        last_isInTheSystem     = false;
    }

    return swarmAgentIsOnline;
}

bool OtherSwarmAgentListener::isInTheSystem() {
    isOnline();
    return last_isInTheSystem;
}

void OtherSwarmAgentListener::isOnlineSubCallback(const std_msgs::Bool::ConstPtr &msg) {
//    std::cout << "ENTERING OtherSwarmAgentListener::isOnlineSubCallback this:" << this << std::endl;
    swarmAgentIsOnline = true;
    last_isInTheSystem = msg->data;
    last_isOnline_timestamp = ros::Time::now();
//    std::cout << "LEAVING  OtherSwarmAgentListener::isOnlineSubCallback this:" << this << std::endl;
    return;
}

void OtherSwarmAgentListener::estPoseSubCallback(const droneMsgsROS::dronePose::ConstPtr &msg) {
//    std::cout << "ENTERING OtherSwarmAgentListener::estPoseSubCallback this:" << this << std::endl;
    last_estimatedPose_timestamp = ros::Time::now();
    last_droneInfo.id = idDrone;
//    last_droneInfo.pose = (*msg);
    last_droneInfo.pose.time=msg->time;
    last_droneInfo.pose.x=msg->x;
    last_droneInfo.pose.y=msg->y;
    last_droneInfo.pose.z=msg->z;
    last_droneInfo.pose.yaw=msg->yaw;
    last_droneInfo.pose.pitch=msg->pitch;
    last_droneInfo.pose.roll=msg->roll;
//    std::cout << "OtherSwarmAgentListener::estPoseSubCallback this BEFORE strings:" << this << std::endl;
    last_droneInfo.pose.reference_frame=msg->reference_frame;
    last_droneInfo.pose.target_frame=msg->target_frame;
    last_droneInfo.pose.YPR_system=msg->YPR_system;
//    std::cout << "LEAVING  OtherSwarmAgentListener::estPoseSubCallback this:" << this << std::endl;
    return;
}

droneMsgsROS::droneInfo OtherSwarmAgentListener::getDroneInfo() {
    return last_droneInfo;
}
