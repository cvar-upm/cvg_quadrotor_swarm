#include "brainstatemachine.h"

BrainStateMachine::BrainStateMachine( ThisSwarmAgentInterface *p_this_drone_interface_in) {
    current_state = BrainStates::INIT_SEQUENCE;
    next_state    = BrainStates::INIT_SEQUENCE;
    p_this_drone_interface       = p_this_drone_interface_in;
    last_HL_command              = droneMsgsROS::droneHLCommand::UNKNOWN;
    received_HL_command          = false;
    HL_command_reception_enabled = false;
    online_check  = false;
    started_check = false;
    state_step    = 0;
    started_emergency_land_sequence = false;
    first_startup_sequence_done = false;
    is_in_the_system = false;
#ifdef    BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    myfile.open("brain_state_machine_debug.txt");
#endif // BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
}

BrainStateMachine::~BrainStateMachine() {
    p_this_drone_interface = NULL;
#ifdef    BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    myfile.close();
#endif // BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
}

void BrainStateMachine::open(ros::NodeHandle & nIn) {
    n = nIn;

    // Mission Planner related topic
    droneHLCommandAckPubl = n.advertise<droneMsgsROS::droneHLCommandAck>( DRONE_BRAIN_PLANNER_HL_COMMAND_ACK, 1, true);
    droneHLCommandSubs    = n.subscribe( DRONE_BRAIN_PLANNER_HL_COMMAND, 1, &BrainStateMachine::droneHLCommandCallback, this);
    droneMissionInfoSubs  = n.subscribe( DRONE_BRAIN_INFO_PUB, 1, &BrainStateMachine::droneMissionInfoCallback, this);
    droneMisionGoTaskPubl = n.advertise<droneMsgsROS::droneGoTask>( DRONE_BRAIN_GO_TASK_SUBS, 1, true);
    // services
    suspendClientSrv = n.serviceClient<std_srvs::Empty>(std::string(MODULE_NAME_MISSION_PLANNER)+"/suspend");
    resumeClientSrv  = n.serviceClient<std_srvs::Empty>(std::string(MODULE_NAME_MISSION_PLANNER)+"/resume");

    // EKF and controller related services and topics
    setInitDroneYaw_srv_server = n.serviceClient<droneMsgsROS::setInitDroneYaw_srv_type>(std::string(MODULE_NAME_ODOMETRY_STATE_ESTIMATOR)+"/setInitDroneYaw");
    setControlModeClientSrv=n.serviceClient<droneMsgsROS::setControlMode>(std::string(MODULE_NAME_TRAJECTORY_CONTROLLER)+"/setControlMode");
    dronePositionRefsPub   =n.advertise<droneMsgsROS::dronePositionRefCommandStamped>(DRONE_BRAIN_POSITION_REF_PUBLICATION, 1, true);
    droneYawRefCommandPub  =n.advertise<droneMsgsROS::droneYawRefCommand>(DRONE_BRAIN_YAW_REF_PUBLICATION, 1, true);
    return;
}

void BrainStateMachine::droneHLCommandCallback(const droneMsgsROS::droneHLCommand::ConstPtr &msg) {
    // msg->time; // ignored
#ifdef    BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    myfile << "Entering BrainStateMachine::droneHLCommandCallback" << std::endl;
#endif // BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    if (HL_command_reception_enabled) {
        last_HL_command     = msg->hlCommand;
        received_HL_command = true;
#ifdef    BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
        myfile << "droneMsgsROS::droneHLCommand.hlCommand = " << msg->hlCommand << std::endl;
        myfile << "HL_command_reception_enabled == true" << std::endl;
#endif // BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    } else {
        publishDroneHLCommandAck(false);
#ifdef    BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
        myfile << "HL_command_reception_enabled == false" << std::endl;
#endif // BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    }
#ifdef    BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    myfile << "Finished BrainStateMachine::droneHLCommandCallback" << std::endl;
#endif // BRAIN_STATE_MACHINE_DEBUG_LOG_IS_ACTIVE
    return;
}

void BrainStateMachine::droneMissionInfoCallback( const droneMsgsROS::droneMissionInfo::ConstPtr &msg) {
    last_missionInfo = (*msg);
    return;
}

void BrainStateMachine::publishDroneHLCommandAck( bool ack) {
    droneMsgsROS::droneHLCommandAck msg;
    msg.time = ros::Time::now();
    msg.ack  = ack;
    droneHLCommandAckPubl.publish(msg);
    return;
}

bool BrainStateMachine::run() {
    online_check  = false;
    started_check = false;
    preconditionCheck();

    // TODO_JP: What to do in any of these cases?
    if (!online_check && !started_emergency_land_sequence) { // updated in preconditionCheck();
//        started_emergency_land_sequence = true;
//        current_state = BrainStates::EMERGENCY_LAND_SEQUENCE;
//        state_step = 0;
    }
    if (!started_check) { // preconditionCheck();
        // TODO_JP, lo suyo seria mostar un mensaje de warning o algo asi
    }

    bool is_successful = true;
    is_successful = processState();
    stateTransitionCheck();
    return is_successful;
}

void BrainStateMachine::preconditionCheck()
{
    switch (current_state) {
    case BrainStates::INIT_SEQUENCE:
        online_check  = true;
        started_check = true;
        break;
    case BrainStates::LANDED:
    case BrainStates::TAKE_OFF_SEQUENCE:
        // all modules must be online
        online_check  = allModulesAreOnline();
        started_check = true;
        break;
    case BrainStates::ON_BOARD_HOVERING:
    case BrainStates::CONTROL_INIT_SEQUENCE:
    case BrainStates::CONTROL_STOP_SEQUENCE:
    case BrainStates::LAND_SEQUENCE:
        // all modules must be online
        online_check = allModulesAreOnline();
        // all modules except controller must be started
        started_check = allModulesAreStarted( false );
        break;
    case BrainStates::OFF_BOARD_CONTROL:
        // all modules must be online
        online_check = allModulesAreOnline();
        // all modules must be started
        started_check = allModulesAreStarted( true );
        break;
    case BrainStates::WIFIDOWN_EMERGENCY_SEQUENCE:
    case BrainStates::WIFIDOWN_ON_BOARD_HOVERING:
    case BrainStates::WIFIDOWN_RECOVER_SEQUENCE:
        // all modules must be online
        online_check = allModulesAreOnline();
        // all modules except controller must be started
        started_check = allModulesAreStarted( false );
        break;
    case BrainStates::EMERGENCY_LAND_SEQUENCE:
    default:
        // ??
//        online_check = allModulesAreOnline();
        online_check  = false;
        started_check = false;
        break;
    }
}

bool BrainStateMachine::processState()
{
    HL_command_reception_enabled = false;
    switch (current_state) {
    case BrainStates::INIT_SEQUENCE:
    {
        switch (state_step) {
        // [state_step=0], wait for all modules to be online
        case 0:
            if (allModulesAreOnline() && p_this_drone_interface->isWifiOk()) {
                state_step++;
            }
            break;
        // [state_step=1], start mission_planner
        case 1:
            if (p_this_drone_interface->mission_planner.start()) {
                state_step++;
            }
            break;
        // [state_step=2], wait for mission_planner to be started
        case 2:
            if (p_this_drone_interface->mission_planner.isStarted()) {
                next_state = BrainStates::LANDED;
                state_step++;
            }
            break;
        // [state_step=3], state transition step
        case 3:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::LANDED:
    {
        switch (state_step) {
        // [state_step=0], // [state_step=0], wait for take-off HL_command
        case 0:
            HL_command_reception_enabled = true;
            if ( received_HL_command ) {
                switch (last_HL_command) {
                case droneMsgsROS::droneHLCommand::TAKE_OFF:
                    next_state = BrainStates::TAKE_OFF_SEQUENCE;
                    state_step++;
                    HL_command_reception_enabled = false;
                    break;
                case droneMsgsROS::droneHLCommand::SLEEP:
                case droneMsgsROS::droneHLCommand::LAND:
                    publishDroneHLCommandAck(true);
                    break;
                case droneMsgsROS::droneHLCommand::HOVER:
                case droneMsgsROS::droneHLCommand::MOVE_TRAJECTORY:
                case droneMsgsROS::droneHLCommand::UNKNOWN:
                default:
                    publishDroneHLCommandAck(false);
                    break;
                }
            }
            received_HL_command          = false;
            break;
        // [state_step=1], state transition step
        case 1:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::TAKE_OFF_SEQUENCE:
    {
        bool start_up_services_sent = false;
        droneMsgsROS::setInitDroneYaw_srv_type setInitDroneYaw_srv_var;
        switch (state_step) {
        // [state_step=0], si el drone esta en modo de emergencia, resetearlo para ponerlo en LANDED
        case 0:
            if (p_this_drone_interface->last_drone_status_msg.status == DroneState::UNKNOWN) {
                p_this_drone_interface->drone_reset();
                state_step++;
            } else {
                state_step++;
            }
            break;
        // [state_step=1], esperar  a que el drone este en estado LANDED
        case 1:
            if (p_this_drone_interface->last_drone_status_msg.status == DroneState::LANDED) {
                state_step++;
            }
            break;
        // [state_step=2], mandar take off
        case 2:
            p_this_drone_interface->drone_takeOff();
            state_step++;
            break;
        // [state_step=3], esperar a que take-off este terminado y mandar ack al mission planner
        case 3:
            if ( p_this_drone_interface->last_drone_status_msg.status == DroneState::HOVERING ) {
                state_step++;
            }
            break;
        // [state_step=4], activar todos los modulos salvo el controlador
        case 4:
            start_up_services_sent = true;
            if (!first_startup_sequence_done) {
                start_up_services_sent = start_up_services_sent && p_this_drone_interface->state_estimator.start();
                setInitDroneYaw_srv_var.request.yaw_droneLMrT_telemetry_rad = (p_this_drone_interface->last_rotation_angles_msg.vector.z)*(M_PI/180.0);
                start_up_services_sent = start_up_services_sent && setInitDroneYaw_srv_server.call(setInitDroneYaw_srv_var);
            }
            if (start_up_services_sent) {
                state_step++;
            }
            break;
        // [state_step=5], activar todos los modulos salvo el controlador
        case 5:
            start_up_services_sent = true;
            if (!first_startup_sequence_done) {
                start_up_services_sent = start_up_services_sent && p_this_drone_interface->arucoeye.start();
                start_up_services_sent = start_up_services_sent && p_this_drone_interface->localizer.start();
                start_up_services_sent = start_up_services_sent && p_this_drone_interface->obstacle_processor.start();
                start_up_services_sent = start_up_services_sent && p_this_drone_interface->trajectory_planner.start();
                start_up_services_sent = start_up_services_sent && p_this_drone_interface->yaw_planner.start();
            }
            if (start_up_services_sent) {
                state_step++;
            }
            break;
        // [state_step=6], esperar a que todos los modulos salvo el controlador esten activos
        case 6:
            if ( allModulesAreStarted(false) ) {
                next_state = BrainStates::ON_BOARD_HOVERING;
                publishDroneHLCommandAck( true );
                first_startup_sequence_done = true;
                state_step++;
            }
            break;
        // [state_step=7], state_transition step
        case 7:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }


    }
        break;
    case BrainStates::ON_BOARD_HOVERING:
    {
        switch (state_step) {
        // [state_step=0], // [state_step=0], wait for move, hover or land HL_command
        case 0:
            HL_command_reception_enabled = true;
            if ( received_HL_command ) {
                switch (last_HL_command) {
                case droneMsgsROS::droneHLCommand::MOVE_TRAJECTORY:
                    next_state = BrainStates::CONTROL_INIT_SEQUENCE;
                    state_step++;
                    HL_command_reception_enabled = false;
                    break;
                case droneMsgsROS::droneHLCommand::LAND:
                    next_state = BrainStates::LAND_SEQUENCE;
                    state_step++;
                    HL_command_reception_enabled = false;
                    break;
                case droneMsgsROS::droneHLCommand::HOVER:
                case droneMsgsROS::droneHLCommand::SLEEP:
                case droneMsgsROS::droneHLCommand::TAKE_OFF:
                    publishDroneHLCommandAck(true);
                    break;
                case droneMsgsROS::droneHLCommand::UNKNOWN:
                default:
                    publishDroneHLCommandAck(false);
                    break;
                    break;
                }
            }
            received_HL_command          = false;
            break;
        // [state_step=1], state transition step
        case 1:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::CONTROL_INIT_SEQUENCE:
    {
        switch (state_step) {
        // [state_step=0], encender controlador
        case 0:
            sendCurrentPositionAsPositionRef();
            if (p_this_drone_interface->trajectory_controller.start()) {
                state_step++;
            }
            break;
        // [state_step=1], esperar a que el controlador este encendido
        case 1:
            if (p_this_drone_interface->trajectory_controller.isStarted()) {
                state_step++;
            }
            break;
        // [state_step=2], mandar move, y poner al controlador a hovering in position y mandar ack
        case 2:
        {
            p_this_drone_interface->drone_move();
            bool result = false;
            // result = ParrotBrain.TheDroneController.setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL)
        {
            //Prepare msg
            droneMsgsROS::setControlMode setControlModeSrv;
//            case Controller_MidLevel_controlMode::POSITION_CONTROL:
            setControlModeSrv.request.controlMode.command=Controller_MidLevel_controlMode::POSITION_CONTROL;
            //use service
            if (setControlModeClientSrv.call(setControlModeSrv)) {
                result = setControlModeSrv.response.ack;
            }
            else {
                result = false;
            }
        }
            if( result ) {
                sendCurrentPositionAsPositionRef();
                next_state = BrainStates::OFF_BOARD_CONTROL;
                publishDroneHLCommandAck( true );
                state_step++;
            }
        }
            break;
        // [state_step=3], state_transition step
        case 3:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::OFF_BOARD_CONTROL:
    {
        switch (state_step) {
        // [state_step=0], // [state_step=0], wait for land or hover HL_command
        case 0:
            HL_command_reception_enabled = true;
            if ( received_HL_command ) {
                switch (last_HL_command) {
                case droneMsgsROS::droneHLCommand::HOVER:
                case droneMsgsROS::droneHLCommand::TAKE_OFF:
                    next_state = BrainStates::CONTROL_STOP_SEQUENCE;
                    state_step++;
                    HL_command_reception_enabled = false;
                    break;
                case droneMsgsROS::droneHLCommand::LAND:
                    next_state = BrainStates::LAND_SEQUENCE;
                    state_step++;
                    HL_command_reception_enabled = false;
                    break;
                case droneMsgsROS::droneHLCommand::MOVE_TRAJECTORY:
                case droneMsgsROS::droneHLCommand::SLEEP:
                    publishDroneHLCommandAck(true);
                    break;
                case droneMsgsROS::droneHLCommand::UNKNOWN:
                default:
                    publishDroneHLCommandAck(false);
                    break;
                }
            }
            received_HL_command          = false;
            break;
        // [state_step=1], state transition step
        case 1:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::CONTROL_STOP_SEQUENCE:
    {
        switch (state_step) {
        // [state_step=0], apagar el controlador y mandar hover
        case 0:
            if (p_this_drone_interface->trajectory_controller.stop()) {
                p_this_drone_interface->drone_hover();
                state_step++;
            }
            break;
        // [state_step=1], esperar a que el controlador este apagado y mandar hover
        case 1:
            if (!(p_this_drone_interface->trajectory_controller.isStarted())) {
                state_step++;
            }
            p_this_drone_interface->drone_hover();
            break;
        // [state_step=2], mandar hover
        case 2:
            p_this_drone_interface->drone_hover();
            state_step++;
            break;
        // [state_step=3], esperar a que el AR Drone entre en hovering y mandar ack
        case 3:
            if (p_this_drone_interface->last_drone_status_msg.status == DroneState::HOVERING) {
                next_state = BrainStates::ON_BOARD_HOVERING;
                publishDroneHLCommandAck( true );
                state_step++;
            }
            break;
        // [state_step=4], state_transition step
        case 4:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::LAND_SEQUENCE:
    {
        switch (state_step) {
        // [state_step=0], apagar el controlador y mandar land
        case 0:
            if (p_this_drone_interface->trajectory_controller.stop()) {
                p_this_drone_interface->drone_land();
                state_step++;
            }
            break;
        // [state_step=1], esperar a que el controlador este apagado y mandar land
        case 1:
            if (!(p_this_drone_interface->trajectory_controller.isStarted())) {
                state_step++;
            }
            p_this_drone_interface->drone_land();
            break;
        // [state_step=2], mandar land
        case 2:
            p_this_drone_interface->drone_land();
            state_step++;
            break;
        // [state_step=3], esperar a que el AR Drone entre en hovering y mandar ack
        case 3:
            if ( (p_this_drone_interface->last_drone_status_msg.status == DroneState::LANDED )
              || (p_this_drone_interface->last_drone_status_msg.status == DroneState::UNKNOWN) ) {
                next_state = BrainStates::LANDED;
                publishDroneHLCommandAck( true );
                state_step++;
            }
            break;
        // [state_step=4], state_transition step
        case 4:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::WIFIDOWN_EMERGENCY_SEQUENCE:
    {
        switch (state_step) {
        // [state_step=0], apagar el controlador y mandar hover
        case 0:
            if (p_this_drone_interface->trajectory_controller.stop()) {
                p_this_drone_interface->drone_hover();
                state_step++;
            }
            break;
        // [state_step=1], esperar a que el controlador este apagado y mandar hover
        case 1:
            if (!(p_this_drone_interface->trajectory_controller.isStarted())) {
                state_step++;
            }
            p_this_drone_interface->drone_hover();
            break;
        // [state_step=2], mandar hover
        case 2:
            p_this_drone_interface->drone_hover();
            state_step++;
            break;
        // [state_step=3], esperar a que el AR Drone entre en hovering y mandar ack
        case 3:
            if (p_this_drone_interface->last_drone_status_msg.status == DroneState::HOVERING) {
                next_state = BrainStates::WIFIDOWN_ON_BOARD_HOVERING;
                state_step++;
            }
            break;
        // [state_step=4], state_transition step
        case 4:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::WIFIDOWN_ON_BOARD_HOVERING:
    {
        switch (state_step) {
        // [state_step=0], // [state_step=0], wait for wifi to be back
        case 0:
            p_this_drone_interface->drone_hover();
            if ( p_this_drone_interface->isWifiOk() ) {
                next_state = BrainStates::WIFIDOWN_RECOVER_SEQUENCE;
                state_step++;
            }
            break;
        // [state_step=1], state transition step
        case 1:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::WIFIDOWN_RECOVER_SEQUENCE:
    {
        switch (state_step) {
        // [state_step=0], encender controlador
        case 0:
            if (p_this_drone_interface->trajectory_controller.start()) {
                state_step++;
            }
            break;
        // [state_step=1], esperar a que el controlador este encendido
        case 1:
            if (p_this_drone_interface->trajectory_controller.isStarted()) {
                state_step++;
            }
            break;
        // [state_step=2], mandar move, y poner al controlador a hovering in position y mandar ack
        case 2:
        {
            p_this_drone_interface->drone_move();
            bool result = false;
            // result = ParrotBrain.TheDroneController.setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL)
        {
            //Prepare msg
            droneMsgsROS::setControlMode setControlModeSrv;
//            case Controller_MidLevel_controlMode::POSITION_CONTROL:
            setControlModeSrv.request.controlMode.command=Controller_MidLevel_controlMode::POSITION_CONTROL;
            //use service
            if (setControlModeClientSrv.call(setControlModeSrv)) {
                result = setControlModeSrv.response.ack;
            }
            else {
                result = false;
            }
        }
            if( result ) {
                sendCurrentPositionAsPositionRef();
                next_state = BrainStates::OFF_BOARD_CONTROL;
                publishDroneHLCommandAck( true );
                state_step++;
            }
        }
            break;
        // [state_step=3], state_transition step
        case 3:
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
        break;
    case BrainStates::EMERGENCY_LAND_SEQUENCE:
    {
        switch (state_step) {
        // [state_step=0], apagar el controlador y mandar land
        case 0:
            if (p_this_drone_interface->trajectory_controller.isOnline()) {
                if (p_this_drone_interface->trajectory_controller.stop()) {
                    p_this_drone_interface->drone_land();
                    state_step++;
                }
            }
            break;
        // [state_step=1], esperar a que el controlador este apagado y mandar land
        case 1:
            if (!(p_this_drone_interface->trajectory_controller.isStarted())) {
                state_step++;
            }
            p_this_drone_interface->drone_land();
            break;
        // [state_step=2], mandar land
        case 2:
            p_this_drone_interface->drone_land();
            state_step++;
            break;
        // [state_step=3], esperar a que el AR Drone este landed
        case 3:
            if ( (p_this_drone_interface->last_drone_status_msg.status == DroneState::LANDED )
              || (p_this_drone_interface->last_drone_status_msg.status == DroneState::UNKNOWN) ) {
                state_step++;
            }
            break;
        // [state_step=4], state_transition dead end
        case 4:
            is_in_the_system = false;
            return false;
            break;
        // [state_step=¿?], this state_step should not be reached
        default:
            state_step = 0;
            break;
        }
    }
    default:
        break;
    }
    return true;
}

void BrainStateMachine::stateTransitionCheck()
{
    switch (current_state) {
    case BrainStates::INIT_SEQUENCE:
            {
        if ( state_step == 3 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::LANDED:
    {
        if ( state_step == 1 ) {
            current_state = next_state;
            is_in_the_system = true;
            state_step = 0;
        }
    }
        break;
    case BrainStates::TAKE_OFF_SEQUENCE:
    {
        if ( state_step == 7 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::ON_BOARD_HOVERING:
    {
        if ( state_step == 1 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::CONTROL_INIT_SEQUENCE:
    {
        if ( state_step == 3 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::OFF_BOARD_CONTROL:
    {
        if ( !p_this_drone_interface->isWifiOk() ) {
            current_state = BrainStates::WIFIDOWN_EMERGENCY_SEQUENCE;
            state_step = 0;
        }
        if ( state_step == 1 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::CONTROL_STOP_SEQUENCE:
    {
        if ( state_step == 4 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::LAND_SEQUENCE:
    {
        if ( state_step == 4 ) {
            current_state = next_state;
            is_in_the_system = false;
            state_step = 0;
        }
    }
        break;
    case BrainStates::WIFIDOWN_EMERGENCY_SEQUENCE:
    {
        if ( state_step == 4 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::WIFIDOWN_ON_BOARD_HOVERING:
    {
        if ( state_step == 1 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::WIFIDOWN_RECOVER_SEQUENCE:
    {
        if ( state_step == 3 ) {
            current_state = next_state;
            state_step = 0;
        }
    }
        break;
    case BrainStates::EMERGENCY_LAND_SEQUENCE:
        // do nothing
        break;
    default:
        break;
    }
}

bool BrainStateMachine::allModulesAreOnline()
{
    bool check_result = true;
    check_result = check_result && p_this_drone_interface->state_estimator.isOnline();
    check_result = check_result && p_this_drone_interface->trajectory_controller.isOnline();
    check_result = check_result && p_this_drone_interface->arucoeye.isOnline();
    check_result = check_result && p_this_drone_interface->localizer.isOnline();
    check_result = check_result && p_this_drone_interface->obstacle_processor.isOnline();
    check_result = check_result && p_this_drone_interface->trajectory_planner.isOnline();
    check_result = check_result && p_this_drone_interface->yaw_planner.isOnline();
    check_result = check_result && p_this_drone_interface->mission_planner.isOnline();
    return check_result;
}

bool BrainStateMachine::allModulesAreStarted( bool check_controller )
{
    bool check_result = true;
    check_result = check_result && p_this_drone_interface->state_estimator.isStarted();
    if (check_controller) {
        check_result = check_result && p_this_drone_interface->trajectory_controller.isStarted();
    }
    check_result = check_result && p_this_drone_interface->arucoeye.isStarted();
    check_result = check_result && p_this_drone_interface->localizer.isStarted();
    check_result = check_result && p_this_drone_interface->obstacle_processor.isStarted();
    check_result = check_result && p_this_drone_interface->trajectory_planner.isStarted();
    check_result = check_result && p_this_drone_interface->yaw_planner.isStarted();
    check_result = check_result && p_this_drone_interface->mission_planner.isStarted();
    return check_result;
}

std::string BrainStateMachine::getBrainState_str() {
    std::string return_str = "";
    switch (current_state) {
    case BrainStates::INIT_SEQUENCE:
        return_str = "INIT_SEQUENCE";
        break;
    case BrainStates::LANDED:
        return_str = "LANDED";
        break;
    case BrainStates::TAKE_OFF_SEQUENCE:
        return_str = "TAKE_OFF_SEQUENCE";
        break;
    case BrainStates::ON_BOARD_HOVERING:
        return_str = "ON_BOARD_HOVERING";
        break;
    case BrainStates::CONTROL_INIT_SEQUENCE:
        return_str = "CONTROL_INIT_SEQUENCE";
        break;
    case BrainStates::OFF_BOARD_CONTROL:
        return_str = "OFF_BOARD_CONTROL";
        break;
    case BrainStates::CONTROL_STOP_SEQUENCE:
        return_str = "CONTROL_STOP_SEQUENCE";
        break;
    case BrainStates::LAND_SEQUENCE:
        return_str = "LAND_SEQUENCE";
        break;
    case BrainStates::WIFIDOWN_EMERGENCY_SEQUENCE:
        return_str = "WIFIDOWN_EMERGENCY_SEQUENCE";
        break;
    case BrainStates::WIFIDOWN_ON_BOARD_HOVERING:
        return_str = "WIFIDOWN_ON_BOARD_HOVERING";
        break;
    case BrainStates::WIFIDOWN_RECOVER_SEQUENCE:
        return_str = "WIFIDOWN_RECOVER_SEQUENCE";
        break;
    case BrainStates::EMERGENCY_LAND_SEQUENCE:
        return_str = "EMERGENCY_LAND_SEQUENCE";
        break;
    default:
        return_str = "UNDEFINED";
        break;
    }
    return return_str;
}

std::string BrainStateMachine::getBrainStateStep_str() {
    std::ostringstream convert;
    convert << state_step;
    std::string return_str = convert.str();
    return return_str;
}

bool BrainStateMachine::getOnlineCheckBool() {
    return online_check;
}

bool BrainStateMachine::getStartedCheckBool(){
    return started_check;
}

bool BrainStateMachine::getIsInTheSystemBool()
{
    return is_in_the_system;
}


void BrainStateMachine::sendCurrentPositionAsPositionRef() {
    // Note: p_this_drone_interface->last_estimatedPose is in droneGMR  wrt GFF
    // Note: position_ref                               is in droneLMrT wrt LMrTFF
    droneMsgsROS::dronePositionRefCommandStamped position_ref;
    position_ref.header.stamp       = ros::Time::now();
    position_ref.position_command.x = p_this_drone_interface->last_estimatedPose.x;
    position_ref.position_command.y = p_this_drone_interface->last_estimatedPose.y;
    position_ref.position_command.z = p_this_drone_interface->last_estimatedPose.z;
    droneMsgsROS::droneYawRefCommand yaw_ref;
    yaw_ref.header.stamp = ros::Time::now();
    yaw_ref.yaw          = p_this_drone_interface->last_estimatedPose.yaw;

    dronePositionRefsPub.publish(position_ref);
    droneYawRefCommandPub.publish(yaw_ref);;
}


