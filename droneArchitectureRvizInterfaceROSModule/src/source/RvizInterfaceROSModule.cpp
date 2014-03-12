#include "RvizInterfaceROSModule.h"
#include "RvizInteractiveMarkerDisplay.h"
#include <interactive_markers/interactive_marker_server.h>

using namespace visualization_msgs;

int Num_Drone=1;

DroneArchitectureRvizInterfaceROSModule::DroneArchitectureRvizInterfaceROSModule() :
    Module( droneModule::active, FREQ_ARCHITECTURE_RVIZ_INTERFACE)
{
    init();
    swarmAgentsList.clear();
    societyIds.clear();
    return;
}

DroneArchitectureRvizInterfaceROSModule::~DroneArchitectureRvizInterfaceROSModule()
{
    close();
    return;
}

void DroneArchitectureRvizInterfaceROSModule::init()
{
    Module::init();
    return;
}

void DroneArchitectureRvizInterfaceROSModule::open(ros::NodeHandle &nIn, std::string moduleName)
{
    //Node
    Module::open(nIn,moduleName);

    societyBroadcastSubs = n.subscribe( DRONE_ARCHITECTURE_RVIZ_INTERFACE_SOCIETY_BROADCAST_SUBSCRIPTION, 100, &DroneArchitectureRvizInterfaceROSModule::societyBroadcastCallback, this);
    marker.ServerResetNew();
    ROS_INFO("Make Menu");
    marker.makeAxesMenu("GRF");
    marker.ServerApplyChanges();

    //Flag of module opened
    droneModuleOpened=true;
}

void DroneArchitectureRvizInterfaceROSModule::close()
{
    Module::close();
    return;
}

bool DroneArchitectureRvizInterfaceROSModule::run()
{
    Module::run();

    // Everything is done in SwarmAgentInterface callbacks

    return true;
}

void DroneArchitectureRvizInterfaceROSModule::societyBroadcastCallback(const std_msgs::Int32::ConstPtr &msg)
{
    // msg->data has the drones' idDrone parameter
    bool not_found = true;
    for ( auto it : societyIds ) {
        if ( it == (msg->data) ) {
            not_found = false;
        }
    }

    if ( not_found ) { // msg->data not in societyIds

        societyIds.push_back(msg->data);

        // Create marker with new Id

        tf::Vector3 position;
        position = tf::Vector3( 0,0, 0);
        marker.initMenu(msg->data);
        marker.MenuHandlerApply();
        marker.makeMovingMarker(position, std::string("Drone ")+std::to_string(msg->data), std::string("Moving ")+std::to_string(msg->data));
        ROS_INFO("Menu option added");

        marker.ServerApplyChanges();

        swarmAgentsList.push_back( SwarmAgentInterface(msg->data) );
        swarmAgentsList.back().open(n);
        Num_Drone++;
    }
    return;
}
