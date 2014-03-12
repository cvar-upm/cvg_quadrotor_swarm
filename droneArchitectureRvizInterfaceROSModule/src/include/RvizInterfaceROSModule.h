#ifndef DRONEARCHITECTURERVIZROSMODULE_H
#define DRONEARCHITECTURERVIZROSMODULE_H

#include "droneModuleROS.h"
#include "communication_definition.h"
#include "swarmAgentInterface.h"
#include "std_msgs/Int32.h"

#include "RvizInteractiveMarkerDisplay.h"
#include <interactive_markers/interactive_marker_server.h>

const double FREQ_ARCHITECTURE_RVIZ_INTERFACE = 50.0;

class DroneArchitectureRvizInterfaceROSModule : public Module
{
public:
    DroneRvizDisplay marker;
    DroneArchitectureRvizInterfaceROSModule();
    ~DroneArchitectureRvizInterfaceROSModule();
    void init();
    void open(ros::NodeHandle & nIn, std::string moduleName);
    void close();
    bool run();
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

private:
    ros::Subscriber societyBroadcastSubs; // subscription, what drones are there in the swarm?
    void societyBroadcastCallback(const std_msgs::Int32::ConstPtr &msg);
    std::list<SwarmAgentInterface> swarmAgentsList;
    std::vector<int> societyIds;

};

#endif // DRONEARCHITECTURERVIZROSMODULE_H
