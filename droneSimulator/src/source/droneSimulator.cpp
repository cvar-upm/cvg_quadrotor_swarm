#include "droneSimulator.h"
#include "xmlfilereader.h"

using namespace CVG;






/////////// DroneSimulator
DroneSimulator::DroneSimulator(int idDrone, const std::string &stackPath_in)
{
    // Initialize drone state and drone state command interface variables
    current_drone_state = DroneState::LANDED;
    last_drone_state_command = DroneStateCommand::LAND;
    flag_drone_state_command_received = false;

    // Read initial position from configuration file
    try {
        XMLFileReader my_xml_reader(stackPath_in+"configs/drone"+std::to_string(idDrone)+"/ekf_state_estimator_config.xml");
        double x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux;
        x_aux = my_xml_reader.readDoubleValue( {"take_off_site","position","x"} );
        y_aux = my_xml_reader.readDoubleValue( {"take_off_site","position","y"} );
        z_aux = my_xml_reader.readDoubleValue( {"take_off_site","position","z"} );
        yaw_aux   = (M_PI/180.0)*my_xml_reader.readDoubleValue( {"take_off_site","attitude","yaw"} );
        pitch_aux = (M_PI/180.0)*my_xml_reader.readDoubleValue( {"take_off_site","attitude","pitch"} );
        roll_aux  = (M_PI/180.0)*my_xml_reader.readDoubleValue( {"take_off_site","attitude","roll"} );
        setPosition_drone_GMR_wrt_GFF( x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux);
        setSpeed_drone_GMR_wrt_GFF( 0.0, 0.0, 0.0);
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
    // Set value to unused sensors, set by default constructor
}

DroneSimulator::~DroneSimulator()
{
}



int DroneSimulator::run()
{
//    precondition_check_variables  = false;
    preconditionCheck();
//    if (!precondition_check_variables) {
//    }

    bool is_successful = true;
    is_successful = processState();
    stateTransitionCheck();

    postProcessInputs();    // Modify drone pitch/roll/dyaw/dz commands of the drone
    postProcessOutputs();   // Set sensor data

    return (int) is_successful;

    //Get commands of the drone
    std::cout<<"Drone Commands:" <<std::endl;
    double pitchCmd, rollCmd;
    DroneAutopilot.getPitchRollCommand(pitchCmd,rollCmd);
    std::cout<<" Pitch:"<<pitchCmd<<std::endl;
    std::cout<<" Roll:"<<rollCmd<<std::endl;
    std::cout<<" DAltitude:"<< DroneAutopilot.getDAltitudeCommand()<<std::endl;
    std::cout<<" DYaw:"<< DroneAutopilot.getDYawCommand()<<std::endl;



    //Simulation of the drone!!




    //Setting sensors. fill with the simulation values
    DroneBattery.setPercenaje(99.9);
    //DroneImu; //TODO, and put on droneSimulator Constructor
    DroneTermometer.setTemperature(27.0,0.0);
    DroneMagnetometer.setMagnetometer(-25.0,5.0,65.0);
    DroneAltitudeSensor.setAltitude(0.5,0.0);
    DroneAltitudeSensor.setAltitudeSpeed(0.1,0.0);
    DroneRotationAnglesSensor.setRotationAngles(0.21,0.001,0.258);
    DroneGroundSpeedSensor.setGroundSpeed(0.1,-0.1);
    DronePressureSensor.setPressure(780,0.0);


    //end
    return 1;
}

void DroneSimulator::setPosition_drone_GMR_wrt_GFF(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in)
{
    current_xg = x_in;     current_yg = y_in;         current_zg = z_in;
    current_yawg = yaw_in; current_pitchg = pitch_in; current_rollg = roll_in;

    current_xl = x_in;      current_yl = -y_in;         current_zl = -z_in;
    current_yawl = -yaw_in; current_pitchl = -pitch_in; current_rolll = roll_in;
}

void DroneSimulator::setPosition_drone_LMrT_wrt_LMrTFF(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in)
{
    current_xg = x_in;      current_yg = -y_in;         current_zg = -z_in;
    current_yawg = -yaw_in; current_pitchg = -pitch_in; current_rollg = roll_in;

    current_xl = x_in;     current_yl = y_in;         current_zl = z_in;
    current_yawl = yaw_in; current_pitchl = pitch_in; current_rolll = roll_in;
}

void DroneSimulator::setSpeed_drone_GMR_wrt_GFF(double vx_in, double vy_in, double vz_in)
{
    current_vxg = vx_in; current_vyg = vy_in;  current_vzg = vz_in;
    current_vxl = vx_in; current_vyl = -vy_in; current_vzl = -vz_in;
}

void DroneSimulator::setSpeed_droneLMrT_wrt_LMrTFF(double vx_in, double vy_in, double vz_in)
{
    current_vxg = vx_in; current_vyg = -vy_in; current_vzg = -vz_in;
    current_vxl = vx_in; current_vyl = vy_in;  current_vzl = vz_in;
}

void DroneSimulator::getPosition_drone_GMR_wrt_GFF(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in)
{
    x_in = current_xg;     y_in = current_yg;         z_in = current_zg;
    yaw_in = current_yawg; pitch_in = current_pitchg; roll_in = current_rollg;
}

void DroneSimulator::getPosition_drone_LMrT_wrt_LMrTFF(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in)
{
    x_in = current_xl;     y_in = current_yl;         z_in = current_zl;
    yaw_in = current_yawl; pitch_in = current_pitchl; roll_in = current_rolll;
}

void DroneSimulator::getSpeed_drone_GMR_wrt_GFF(double &vx_in, double &vy_in, double &vz_in)
{
    vx_in = current_vxg;     vy_in = current_vyg;         vz_in = current_vzg;
}

void DroneSimulator::getSpeed_droneLMrT_wrt_LMrTFF(double &vx_in, double &vy_in, double &vz_in)
{
    vx_in = current_vxl;     vy_in = current_vyl;         vz_in = current_vzl;
}

//void DroneSimulator::getPosition_drone_GMR_wrt_GFF_noisy(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in)
//{
//    x_in = current_xg+distribution(generator);     y_in = current_yg+distribution(generator);         z_in = current_zg+distribution(generator);
//    yaw_in = current_yawg+distribution(generator); pitch_in = current_pitchg+distribution(generator); roll_in = current_rollg+distribution(generator);
//}


void DroneSimulator::preconditionCheck()
{
    //    precondition_check_variables  = true;
}

bool DroneSimulator::processState()
{
    bool is_successful = true;

    switch (current_drone_state) {
    case DroneState::UNKNOWN:
    {
        setPosition_drone_GMR_wrt_GFF( current_xg, current_yg, 0.0, current_yawg, 0.0, 0.0);
        setSpeed_drone_GMR_wrt_GFF(    0.0, 0.0, 0.0);
    } break;
    case DroneState::LANDED:
    {
        setPosition_drone_GMR_wrt_GFF( current_xg, current_yg, 0.0, current_yawg, 0.0, 0.0);
        setSpeed_drone_GMR_wrt_GFF(    0.0, 0.0, 0.0);
    } break;
    case DroneState::TAKING_OFF:
    {
        double elapsed_seconds = timer.getElapsedSeconds();
        setPosition_drone_GMR_wrt_GFF( current_xg, current_yg,
                                       DRONESIMULATOR_TAKING_OFF_ALTITUDE*(elapsed_seconds/DRONESIMULATOR_TAKING_OFF_TIME),
                                       current_yawg, 0.0, 0.0);
        setSpeed_drone_GMR_wrt_GFF(    0.0, 0.0,
                                       DRONESIMULATOR_TAKING_OFF_ALTITUDE/DRONESIMULATOR_TAKING_OFF_TIME);
        if (elapsed_seconds > DRONESIMULATOR_TAKING_OFF_TIME)
            flag_drone_state_command_received = true;
    } break;
    case DroneState::HOVERING:
    {
        setPosition_drone_GMR_wrt_GFF( current_xg, current_yg, current_zg, current_yawg, 0.0, 0.0);
        setSpeed_drone_GMR_wrt_GFF(    0.0, 0.0, 0.0);
    } break;
    case DroneState::FLYING:
    {
        double pitch, roll, dyaw, dz;
        DroneAutopilot.getPitchRollCommand( pitch, roll);
        dyaw = DroneAutopilot.getDYawCommand( );
        dz   = DroneAutopilot.getDAltitudeCommand( );

        quadrotor_model.setInputs( pitch, roll, dyaw, dz);
        quadrotor_model.runSimulation();

        double current_dyawl;
        quadrotor_model.getObservation( current_xl, current_yl, current_zl,
                                        current_yawl, current_pitchl, current_rolll,
                                        current_vxl,  current_vyl, current_vzl,
                                        current_dyawl,
                                        current_vxm, current_vym);

        setPosition_drone_LMrT_wrt_LMrTFF( current_xl, current_yl, current_zl,
                                           current_yawl, current_pitchl, current_rolll);
        setSpeed_droneLMrT_wrt_LMrTFF( current_vxl,  current_vyl, current_vzl);
    } break;
    case DroneState::LANDING:
    {
        double elapsed_seconds = timer.getElapsedSeconds();
        timer.restart(true);
        setPosition_drone_GMR_wrt_GFF( current_xg, current_yg,
                                       current_zg - elapsed_seconds*DRONESIMULATOR_LANDING_SPEED,
                                       current_yawg, 0.0, 0.0);
        setSpeed_drone_GMR_wrt_GFF(    0.0, 0.0,
                                       - DRONESIMULATOR_LANDING_SPEED);
        if (current_zg < 0.0)
            flag_drone_state_command_received = true;
    } break;
    default:
    {
        //    case DroneState::INIT:
        //    case DroneState::TEST:
        //    case DroneState::GOTO_FIX_POINT:
        //    case DroneState::LOOPING:
        // same as DroneState::UNKNOWN
        setPosition_drone_GMR_wrt_GFF( current_xg, current_yg, 0.0, current_yawg, 0.0, 0.0);
        setSpeed_drone_GMR_wrt_GFF(    0.0, 0.0, 0.0);
    } break;
    }
    return is_successful;
}

void DroneSimulator::stateTransitionCheck()
{
    if (flag_drone_state_command_received) {
        switch (current_drone_state) {
        case DroneState::UNKNOWN:
        {
            switch (last_drone_state_command) {
            case DroneStateCommand::RESET:
            {
                startLandedState();
            } break;
            default: {} break;
            }
        } break;
        case DroneState::LANDED:
        {
            switch (last_drone_state_command) {
            case DroneStateCommand::RESET:
            {
                startUnknownState();
            } break;
            case DroneStateCommand::TAKE_OFF:
            {
                startTakingOffState();
            } break;
            default: {} break;
            }
        } break;
        case DroneState::TAKING_OFF:
        {
            switch (last_drone_state_command) {
            case DroneStateCommand::RESET:
            {
                startUnknownState();
            } break;
            case DroneStateCommand::LAND:
            {
                startLandingState();
            } break;
            case DroneStateCommand::MOVE:
            {
                double elapsed_seconds = timer.getElapsedSeconds();
                if (elapsed_seconds > DRONESIMULATOR_TAKING_OFF_TIME)
                    startFlyingState();
            } break;
            default: {
                double elapsed_seconds = timer.getElapsedSeconds();
                if (elapsed_seconds > DRONESIMULATOR_TAKING_OFF_TIME)
                    startHoveringState();
            } break;
            }
        } break;
        case DroneState::HOVERING:
        {
            switch (last_drone_state_command) {
            case DroneStateCommand::RESET:
            {
                startUnknownState();
            } break;
            case DroneStateCommand::LAND:
            {
                startLandingState();
            } break;
            case DroneStateCommand::MOVE:
            {
                startFlyingState();
            } break;
            default: {} break;
            }
        } break;
        case DroneState::FLYING:
        {
            switch (last_drone_state_command) {
            case DroneStateCommand::RESET:
            {
                startUnknownState();
                quadrotor_model.stop();
                current_vxm = 0.0;
                current_vym = 0.0;
            } break;
            case DroneStateCommand::LAND:
            {
                startLandingState();
                quadrotor_model.stop();
                current_vxm = 0.0;
                current_vym = 0.0;
            } break;
            case DroneStateCommand::HOVER:
            {
                startHoveringState();
                quadrotor_model.stop();
                current_vxm = 0.0;
                current_vym = 0.0;
            } break;
            default: {} break;
            }
        } break;
        case DroneState::LANDING:
        {
            switch (last_drone_state_command) {
            case DroneStateCommand::RESET:
            {
                startUnknownState();
            } break;
            default: {
                if (current_zg < 0.0)
                    startLandedState();
            } break;
            }
        } break;
        default:
        {
            //    case DroneState::INIT:
            //    case DroneState::TEST:
            //    case DroneState::GOTO_FIX_POINT:
            //    case DroneState::LOOPING:
            startUnknownState();
        } break;
        }
    }

    flag_drone_state_command_received = false;
}

void DroneSimulator::postProcessInputs()
{
    switch (current_drone_state) {
    case DroneState::TAKING_OFF:
    {
    } break;
    case DroneState::FLYING:
    {
    } break;
    case DroneState::UNKNOWN:
    case DroneState::LANDED:
    case DroneState::HOVERING:
    case DroneState::LANDING:
    default:
    {
        //    case DroneState::INIT:
        //    case DroneState::TEST:
        //    case DroneState::GOTO_FIX_POINT:
        //    case DroneState::LOOPING:
        DroneAutopilot.setPitchRollCommand( 0.0, 0.0);
        DroneAutopilot.setDYawCommand(0.0);
        DroneAutopilot.setDAltitudeCommand(0.0);
    } break;
    }
}

void DroneSimulator::postProcessOutputs()
{
    DroneAltitudeSensor.setAltitude(      current_zl, 0.01*0.01);
    DroneAltitudeSensor.setAltitudeSpeed( current_vzl, 0.01*0.01);
    DroneRotationAnglesSensor.setRotationAngles( current_yawl  *(180.0/M_PI),
                                                 current_pitchl*(180.0/M_PI),
                                                 current_rolll *(180.0/M_PI));
    DroneGroundSpeedSensor.setGroundSpeed( current_vxm,
                                           current_vym);
}

void DroneSimulator::startUnknownState()
{
    current_drone_state = DroneState::UNKNOWN;
}

void DroneSimulator::startLandedState()
{
    current_drone_state = DroneState::LANDED;
}

void DroneSimulator::startTakingOffState()
{
    current_drone_state = DroneState::TAKING_OFF;

    timer.restart(false);
}

void DroneSimulator::startHoveringState()
{
    current_drone_state = DroneState::HOVERING;
}

void DroneSimulator::startFlyingState()
{
    current_drone_state = DroneState::FLYING;

    quadrotor_model.start( current_xl, current_yl, current_zl, current_yawl, current_pitchl, current_rolll, current_vxl, current_vyl, current_vzl, 0.0);
}

void DroneSimulator::startLandingState()
{
    current_drone_state = DroneState::LANDING;

    timer.restart(false);
}

void DroneSimulator::commandDrone(DroneStateCommand::StateCommand drone_state_command)
{
    last_drone_state_command = drone_state_command;
    flag_drone_state_command_received = true;
}


