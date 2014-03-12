#ifndef BRAINSTATEMACHINE_H
#define BRAINSTATEMACHINE_H

// [Task] [DONE] Define state machine states and initial state, add initial state to state machine
// [Task] [DONE] Add communication channels with the Mission Planner
// [Task] Add state sequencing

// ROS
#include "ros/ros.h"

// C++ standar libraries
#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <algorithm>
#include <sstream>

// Communications with mission_planner
#include "droneMsgsROS/droneHLCommand.h"
#include "droneMsgsROS/droneHLCommandAck.h"
#include "droneMsgsROS/droneMissionInfo.h"
#include "droneMsgsROS/droneGoTask.h"
#include "std_srvs/Empty.h"

// State-machine stuff
#include "communication_definition.h"
#include "brain_states.h"
#include "drone_utils/drone_state_enum.h"
#include "thisswarmagentinterface.h"

// SetInitDroneYaw for Controller and StateEstimator
#include "droneMsgsROS/setInitDroneYaw_srv_type.h"
// SetControlModeService for the controller, and position referece
#include "control/Controller_MidLevel_controlModes.h"
//#include "droneTrajectoryControllerROSModule/setControlMode.h"
#include "droneMsgsROS/setControlMode.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/dronePositionRefCommandStamped.h"
#include "droneMsgsROS/droneYawRefCommand.h"

//#define BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE


class BrainStateMachine {
    ros::NodeHandle   n;

    // Communications with mission_planner
private:
    ros::Publisher  droneHLCommandAckPubl;
    ros::Subscriber droneHLCommandSubs;
    ros::Publisher  droneMisionGoTaskPubl;
    ros::Subscriber droneMissionInfoSubs;
    void droneHLCommandCallback( const droneMsgsROS::droneHLCommand::ConstPtr &msg);
    void droneMissionInfoCallback( const droneMsgsROS::droneMissionInfo::ConstPtr &msg);
    void publishDroneHLCommandAck( bool ack);
public:
    droneMsgsROS::droneMissionInfo last_missionInfo;
private: //services
    ros::ServiceClient suspendClientSrv;
    ros::ServiceClient resumeClientSrv;

private:
    // setInitDroneYaw for state_estimator and trajectory_controller
    ros::ServiceClient setInitDroneYaw_srv_server;
    // SetControlModeService for the controller
    ros::ServiceClient setControlModeClientSrv;
    ros::Publisher dronePositionRefsPub;
    ros::Publisher droneYawRefCommandPub;
    void sendCurrentPositionAsPositionRef();

    // State-machine related variables
private:
    bool                     received_HL_command;
    bool                     HL_command_reception_enabled;
    int32_t                  last_HL_command;
    BrainStates::StateType   current_state, next_state;
    ThisSwarmAgentInterface *p_this_drone_interface;
    bool                     online_check, started_check, started_emergency_land_sequence;
    bool                     first_startup_sequence_done;
    int                      state_step;
    bool                     is_in_the_system;

public:
    BrainStateMachine( ThisSwarmAgentInterface *p_this_drone_interface_in);
    ~BrainStateMachine();
    void open(ros::NodeHandle & nIn);
    bool run();
private:
    void preconditionCheck();
    bool processState();
    void stateTransitionCheck();
    bool allModulesAreOnline();
    bool allModulesAreStarted( bool check_controller=false);
public:
    std::string getBrainState_str();
    std::string getBrainStateStep_str();
    bool        getOnlineCheckBool() ;
    bool        getStartedCheckBool();
    bool        getIsInTheSystemBool();
private:
#ifdef    BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    // debugging
    std::ofstream myfile;
#endif // BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
};

#endif // BRAINSTATEMACHINE_H
