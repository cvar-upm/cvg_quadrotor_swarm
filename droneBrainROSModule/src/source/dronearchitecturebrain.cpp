#include "dronearchitecturebrain.h"

DroneArchitectureBrain::DroneArchitectureBrain() :
    DroneModule( droneModule::active, FREQ_ARCHITECTURE_BRAIN),
    this_drone_interface(),
    brain_state_machine( &this_drone_interface )
    {
    init();
    societyMembers.clear();
//    societyMembers.reserve(DRONEARCHITECTUREBRAIN_MAX_SWARM_FLEET_SIZE); // not necessary with std::list
    societyIds.clear();
    counter_other_agents = 0;
    return;
}

DroneArchitectureBrain::~DroneArchitectureBrain() {
    close();
    return;
}

void DroneArchitectureBrain::init() {
    //end
    DroneModule::init();
    return;
}

void DroneArchitectureBrain::open(ros::NodeHandle & nIn, std::string moduleName) {
    //Node
    DroneModule::open(nIn,moduleName);

    this_drone_interface.open(n);

    isInTheSystemPubl    = n.advertise<std_msgs::Bool>( DRONE_ARCHITECTURE_BRAIN_IS_IN_THE_SYSTEM, 1, true);
    societyBroadcastPubl = n.advertise<std_msgs::Int32>( DRONE_ARCHITECTURE_BRAIN_SOCIETY_BROADCAST, 1, true);
    societyBroadcastSubs = n.subscribe( DRONE_ARCHITECTURE_BRAIN_SOCIETY_BROADCAST, 100, &DroneArchitectureBrain::societyBroadcastCallback, this);
    societyPosePubl      = n.advertise<droneMsgsROS::societyPose>( DRONE_ARCHITECTURE_BRAIN_SOCIETY_POSE, 1, true);
    brain_state_machine.open(n);

    //Flag of module opened
    droneModuleOpened=true;

    return;
}

void DroneArchitectureBrain::close() {
    DroneModule::close();
    return;
}

bool DroneArchitectureBrain::run() {
    DroneModule::run();

    // publish this_drone's id number
    {
    std_msgs::Int32 my_id;
    my_id.data = idDrone;
    societyBroadcastPubl.publish(my_id);

    std_msgs::Bool this_drone_is_in_the_system;
    this_drone_is_in_the_system.data = brain_state_machine.getIsInTheSystemBool();
    isInTheSystemPubl.publish(this_drone_is_in_the_system);
    }
    publishSocietyPose();

#ifdef TEST_WITH_AUTONOMOUS_BRAIN
//    std::cout << "DroneArchitectureBrain::run() if (isStarted()), isStarted:" << isStarted() << std::endl;
    if (isStarted()) {
        bool everything_ok = brain_state_machine.run();
//        if (!everything_ok) {
//            stop();
//        }
    }
#endif // TEST_WITH_AUTONOMOUS_BRAIN

//    std::cout << "LEAVING DroneArchitectureBrain::run()" << isStarted() << std::endl;
    return true;
}

void DroneArchitectureBrain::societyBroadcastCallback(const std_msgs::Int32::ConstPtr &msg) {
//    std::cout << "ENTERING DroneArchitectureBrain::societyBroadcastCallback" << std::endl;
    // msg->data has the other drone's idDrone parameter
    if (msg->data == idDrone) {
        return;
    }

    bool not_found = true;
//    std::cout << "DroneArchitectureBrain::societyBroadcastCallback BEFORE societyIds.size()" << std::endl;
    for (unsigned int i=0; i<societyIds.size(); i++) {
        if ( societyIds[i] == (msg->data) ) {
            not_found = false;
        }
    }

//    std::cout << "DroneArchitectureBrain::societyBroadcastCallback BEFORE societyIds.size()" << std::endl;
    if ( not_found ) { // msg->data not in societyIds
        societyIds.push_back(msg->data);
//        std::cout << "DroneArchitectureBrain::societyBroadcastCallback BEFORE societyMembers.push_back( OtherSwarmAgentListener(msg->data) );" << std::endl;
        societyMembers.push_back( OtherSwarmAgentListener(msg->data) );
//        std::cout << "DroneArchitectureBrain::societyBroadcastCallback BEFORE societyMembers.at(societyMembers.size()-1).open(n);" << std::endl;
        societyMembers.back().open(n);
//        std::cout << "DroneArchitectureBrain::societyBroadcastCallback BEFORE counter_other_agents++;" << std::endl;
        counter_other_agents++;
    }
    return;
//    std::cout << "LEAVING DroneArchitectureBrain::societyBroadcastCallback" << std::endl;
}

void DroneArchitectureBrain::publishSocietyPose() {
//    std::cout << "DroneArchitectureBrain::publishSocietyPose() BEFORE societyPose_msg.societyDrone.clear();" << std::endl;
    societyPose_msg.societyDrone.clear();

//    std::cout << "DroneArchitectureBrain::publishSocietyPose() BEFORE societyMembers.size();" << std::endl;
    for (auto it : societyMembers) {
//        std::cout << "DroneArchitectureBrain::publishSocietyPose() BEFORE isStarted():" << (societyMembers[i].isStarted() ? std::string("TRUE") : std::string("FALSE")) << std::endl;
        if (it.isInTheSystem()) { // equivalente a isOnline() and isStarted()
//            std::cout << "DroneArchitectureBrain::publishSocietyPose() BEFORE societyMembers[i].getDroneInfo();" << std::endl;
            societyPose_msg.societyDrone.push_back( it.getDroneInfo() );
//            std::cout << "DroneArchitectureBrain::publishSocietyPose() AFTER  societyMembers[i].getDroneInfo();" << std::endl;
        }
    }

//    std::cout << "DroneArchitectureBrain::publishSocietyPose() BEFORE societyPosePubl.publish(societyPose_msg);" << std::endl;
    societyPosePubl.publish(societyPose_msg);
//    std::cout << "LEAVING DroneArchitectureBrain::publishSocietyPose();" << std::endl;
}
