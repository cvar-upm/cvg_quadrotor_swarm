#include "droneTrajectoryController.h"
#include "xmlfilereader.h"

DroneTrajectoryController::DroneTrajectoryController(int idDrone, const std::string &stackPath_in) :
    current_idDrone(idDrone),
    state_machine(idDrone, stackPath_in),
    midlevel_controller(idDrone, stackPath_in)
    {
    std::cout << "Constructor: DroneTrajectoryController" << std::endl;
    try {
        XMLFileReader my_xml_reader( stackPath_in+"configs/drone"+ std::to_string(idDrone)+"/trajectory_controller_config.xml");
        double pitch_max, roll_max, dyaw_max, dz_max;
        double tilt_tr, dyaw_tr, dz_tr;
        pitch_max = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","pitch_max"} );
        roll_max  = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","roll_max"} );
        dyaw_max  = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","dyaw_max"} );
        dz_max    = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","dz_max"} );
        tilt_tr   = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","low_pass_filters","tilt_tr"} );
        dyaw_tr   = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","low_pass_filters","dyaw_tr"} );
        dz_tr     = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","low_pass_filters","dz_tr"} );

        //Filters
        pitch_lowpassfilter.setResponseTime(tilt_tr);
        pitch_lowpassfilter.enableSaturation( true, -pitch_max, +pitch_max);
        pitch_lowpassfilter.reset();
        roll_lowpassfilter.setResponseTime(tilt_tr);
        roll_lowpassfilter.enableSaturation( true, -roll_max, +roll_max);
        roll_lowpassfilter.reset();
        dyaw_lowpassfilter.setResponseTime(dyaw_tr);
        dyaw_lowpassfilter.enableSaturation( true, -dyaw_max, +dyaw_max);
        dyaw_lowpassfilter.reset();
        dz_lowpassfilter.setResponseTime(dz_tr);
        dz_lowpassfilter.enableSaturation( true, -dz_max, +dz_max);
        dz_lowpassfilter.reset();

        std::string init_control_mode_str;
        init_control_mode_str = my_xml_reader.readStringValue( {"trajectory_controller_config","init_control_mode"} );
        if ( init_control_mode_str.compare("speed") == 0 ) {
            init_control_mode = Controller_MidLevel_controlMode::SPEED_CONTROL;
        } else { if ( init_control_mode_str.compare("position") == 0 ) {
            init_control_mode = Controller_MidLevel_controlMode::POSITION_CONTROL;
        } else {                             // "trajectory"
            init_control_mode = Controller_MidLevel_controlMode::SPEED_CONTROL;
            throw std::runtime_error("Controller_MidLevel_SpeedLoop::Controller_MidLevel_SpeedLoop, initial control_mode cannot be TRAJECTORY_CONTROL");
            return;
            }
        }

        resetValues();

        // Transformation between different reference frames, see documentation for more information
        matHomog_drone_LMrT_wrt_drone_GMR   = cv::Mat::eye(4,4,CV_32F);
        matHomog_drone_GMR_wrt_drone_LMrT   = cv::Mat::eye(4,4,CV_32F);
        matHomog_drone_GMR_wrt_GFF          = cv::Mat::eye(4,4,CV_32F);
        matHomog_drone_LMrT_wrt_LMrTFF       = cv::Mat::eye(4,4,CV_32F);
        referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_LMrT_wrt_drone_GMR,    0.0,   0.0,   0.0,     0.0,       0.0,      M_PI);
        referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_GMR_wrt_drone_LMrT,    0.0,   0.0,   0.0,     0.0,       0.0,     -M_PI);

        XMLFileReader my_xml_reader_ekfstuff( stackPath_in+"configs/drone"+ std::to_string(idDrone)+"/ekf_state_estimator_config.xml");
        double tr_vx, saturation_value_vx, tr_vy, saturation_value_vy, tr_dyaw, saturation_value_dyaw, tr_dz, saturation_value_dz;
        tr_vx               = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","tr_v"} );
        tr_vy               = tr_vx;
        tr_dyaw             = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","tr_dyaw"} );
        tr_dz               = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","tr_dz"} );
        saturation_value_vx   = my_xml_reader_ekfstuff.readDoubleValue( {"ekf_state_estimator_config","ground_optical_flow_maximum_speed"} );
        saturation_value_vy   = saturation_value_vx;
        saturation_value_dyaw = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","saturation_value_dyaw"} );
        saturation_value_dz   = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","saturation_value_dz"} );
        filter_x2vx.setResponseTime(tr_vx);
        filter_x2vx.enableSaturation( true, -saturation_value_vx, +saturation_value_vx);
        filter_y2vy.setResponseTime(tr_vy);
        filter_y2vy.enableSaturation( true, -saturation_value_vy, +saturation_value_vy);
        filter_yaw2dyaw.setResponseTime(tr_dyaw);
        filter_yaw2dyaw.enableSaturation( true, -saturation_value_dyaw, +saturation_value_dyaw);
        filter_z2dz.setResponseTime(tr_dz);
        filter_z2dz.enableSaturation( true, -saturation_value_dz, +saturation_value_dz);

        logControllerGains();
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}

DroneTrajectoryController::~DroneTrajectoryController() {}

void DroneTrajectoryController::resetValues() {
    control_mode = init_control_mode;

    // ci = s, fi = 0, co = 0;
    xci = xs; yci = ys; yawci = yaws; zci = zs;
    pitchfi = 0.0; rollfi = 0.0;
    vxfi = 0.0; vyfi = 0.0; dyawfi = 0.0; dzfi = 0.0;
    pitchco = 0.0; rollco = 0.0; dyawco = 0.0; dzco = 0.0;

    midlevel_controller.reset();
    state_machine.reset();
    pitch_lowpassfilter.reset();
    roll_lowpassfilter.reset();
    dyaw_lowpassfilter.reset();
    dz_lowpassfilter.reset();
}

void DroneTrajectoryController::setPositionControl() {
    state_machine.activatePositionControl();
    midlevel_controller.setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL);
    setAllReferences_droneLMrT_wrt_LMrTFF(xs, ys, yaws, zs);
}

void DroneTrajectoryController::setSpeedControl() {
    state_machine.activateSpeedControl();
    midlevel_controller.setControlMode(Controller_MidLevel_controlMode::SPEED_CONTROL);
    setAllReferences_droneLMrT_wrt_LMrTFF(xs, ys, yaws, zs, 0.0, 0.0);
}

void DroneTrajectoryController::setTrajectory_droneLMrT_wrt_LMrTFF(DroneTrajectory &trajectory, TrajectoryConfiguration &traj_config) {
    state_machine.setTrajectory(trajectory, traj_config);
}

bool DroneTrajectoryController::setControlMode(Controller_MidLevel_controlMode::controlMode mode) {
    bool error_ocurred = false;
        switch (mode) {
        case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
            // Requires a trayectory first!
            error_ocurred = state_machine.activateTrajectoryControl();
            if (!error_ocurred) {
                midlevel_controller.setControlMode(mode);
                setAllReferences_droneLMrT_wrt_LMrTFF(xs, ys, yaws, zs);
            } else {
                setControlMode(init_control_mode);
                return false;
            }
            break;
        case Controller_MidLevel_controlMode::POSITION_CONTROL:
            setPositionControl();
            break;
        case Controller_MidLevel_controlMode::SPEED_CONTROL:
            setSpeedControl();
            break;
        default: // Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE
            return false;
            break;
        }

        if ( control_mode != mode )
            setAllReferences_droneLMrT_wrt_LMrTFF(xs, ys, yaws, zs);
        control_mode = mode;
        return true;
}

void DroneTrajectoryController::setFeedback_drone_pose_GMR_wrt_GFF(double xs_t, double ys_t, double zs_t, double yaws_t, double pitchs_t, double rolls_t)
{
    double x2, y2, z2, yaw2, pitch2, roll2;
    referenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF( xs_t, ys_t, zs_t, yaws_t, pitchs_t, rolls_t, x2, y2, z2, yaw2, pitch2, roll2);
    setFeedback_drone_pose_LMrT_wrt_LMrTFF( x2, y2, z2, yaw2, pitch2, roll2);
}

void DroneTrajectoryController::setFeedback_drone_speeds_GMR_wrt_GFF(double vxs_t, double vys_t, double vzs_t, double dyaws_t, double dpitchs_t, double drolls_t)
{
    double vx2, vy2, vz2, dyaw2, dpitch2, droll2;
    speedReferenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF( vxs_t, vys_t, vzs_t, dyaws_t, dpitchs_t, drolls_t,
                                                                            vx2, vy2, vz2, dyaw2, dpitch2, droll2);
    setFeedback_drone_speeds_LMrT_wrt_LMrTFF( vx2, vy2, vz2, dyaw2, dpitch2, droll2);
}

void DroneTrajectoryController::setFeedback_drone_speeds_LMrT_wrt_LMrTFF(double vxs_t, double vys_t, double vzs_t, double dyaws_t, double dpitchs_t, double drolls_t)
{
    vxs  = vxs_t;
    vys  = vys_t;
    vzs  = vzs_t;
    dyaws = dyaws_t;
//    dpitchs = dpitchs_t;  // unused, undefined
//    drolls  = drolls_t;   // unused, undefined
}
















void DroneTrajectoryController::setAllReferences_droneLMrT_wrt_LMrTFF( double xci_in, double yci_in, double yawci_in, double zci_in,
        double vxfi_in , double vyfi_in , double dyawfi_in , double dzfi_in ,
        double pitchfi_in , double rollfi_in ) {
    xci     = xci_in;
    yci     = yci_in;
    yawci   = yawci_in;
    zci     = zci_in;
    vxfi    = vxfi_in;
    vyfi    = vyfi_in;
    dyawfi  = dyawfi_in;
    dzfi    = dzfi_in;
    pitchfi = pitchfi_in;
    rollfi  = rollfi_in;
}

void DroneTrajectoryController::setPositionRefs_drone_GMR_wrt_GFF(double xci_in, double yci_in, double zci_in, double yawci_in, double pitchfi_in, double rollfi_in) {
    double x2 = 0.0, y2 = 0.0, z2 = 0.0, yaw2 = 0.0, pitch2 = 0.0, roll2 = 0.0;
    referenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF(
                xci_in, yci_in, zci_in, yawci_in, pitchfi_in, rollfi_in,
                x2, y2, z2, yaw2, pitch2, roll2);
    setPositionRefs_drone_LMrT_wrt_LMrTFF(x2, y2, z2, yaw2, pitch2, roll2);
}

void DroneTrajectoryController::setPositionRefs_drone_LMrT_wrt_LMrTFF(double xci_in, double yci_in, double zci_in, double yawci_in, double pitchfi_in, double rollfi_in) {
    xci   = xci_in;
    yci   = yci_in;
    zci   = zci_in;
    yawci = yawci_in;
    pitchfi = pitchfi_in;
    rollfi = rollfi_in;
}



void DroneTrajectoryController::setPositionRefs_drone_GMR_wrt_GFF(double xci_in, double yci_in, double zci_in)
{
    setPositionRefs_drone_LMrT_wrt_LMrTFF( xci_in, -yci_in, -zci_in);
}

void DroneTrajectoryController::setPositionRefs_drone_LMrT_wrt_LMrTFF(double xci_in, double yci_in, double zci_in)
{
    xci   = xci_in;
    yci   = yci_in;
    zci   = zci_in;
}


void DroneTrajectoryController::setYawRef_drone_GMR_wrt_GFF(double yawci_in_rad) {
    setYawRef_drone_LMrT_wrt_LMrTFF( -yawci_in_rad );
}

void DroneTrajectoryController::setYawRef_drone_LMrT_wrt_LMrTFF(double yawci_in_rad) {
    yawci = yawci_in_rad;
}

void DroneTrajectoryController::setHorizontalSpeedRefs_drone_GMR_wrt_GFF(double vxfi_in, double vyfi_in)
{
    setHorizontalSpeedRefs_drone_LMrT_wrt_LMrTFF( vxfi_in, -vyfi_in);
}

void DroneTrajectoryController::setHorizontalSpeedRefs_drone_LMrT_wrt_LMrTFF(double vxfi_in, double vyfi_in)
{
    vxfi = vxfi_in;
    vyfi = vyfi_in;
}

void DroneTrajectoryController::setSpeedRefs_drone_GMR_wrt_GFF( double vxfi_in, double vyfi_in, double dzfi_in) {
    setSpeedRefs_droneLMrT_wrt_LMrTFF( vxfi_in, -vyfi_in, -dzfi_in);
}

void DroneTrajectoryController::setSpeedRefs_droneLMrT_wrt_LMrTFF( double vxfi_in, double vyfi_in, double dzfi_in) {
    vxfi = vxfi_in;
    vyfi = vyfi_in;
    dzfi = dzfi_in;
}

void DroneTrajectoryController::setDYawFIReference_drone_GMR_wrt_GFF( double dyawfi_in) {
    setDYawFIReference_droneLMrT_wrt_LMrTFF(-dyawfi_in);
}

void DroneTrajectoryController::setDYawFIReference_droneLMrT_wrt_LMrTFF( double dyawfi_in) {
    dyawfi =  dyawfi_in;
}

void DroneTrajectoryController::getOutput( double *p_pitchco, double *p_rollco, double *p_dyawco, double *p_dzco) {
    switch (control_mode)
    {
    case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
        state_machine.setInputs( xs, ys, zs, vxs, vys, vzs, yaws, dyaws);
        double xci2, yci2, zci2, vxfi2, vyfi2, dzfi2, yawci2, dyawfi2, pitchfi2, rollfi2;
        state_machine.getOutput( xci2, yci2, zci2, vxfi2, vyfi2, dzfi2, yawci2, dyawfi2, pitchfi2, rollfi2);
        setAllReferences_droneLMrT_wrt_LMrTFF( xci2, yci2, yawci, zci2, vxfi2, vyfi2, 0.0, dzfi2, pitchfi2, rollfi2);
        midlevel_controller.setFeedback( xs, ys, vxs, vys, yaws, zs);
        midlevel_controller.setReference( xci, yci, yawci, zci, vxfi, vyfi, dyawfi, dzfi, pitchfi, rollfi);
        midlevel_controller.getOutput( &pitchco_hf, &rollco_hf, &dyawco_hf, &dzco_hf);
        break;
    case Controller_MidLevel_controlMode::SPEED_CONTROL:
        setAllReferences_droneLMrT_wrt_LMrTFF( xs, ys, yawci, zci, vxfi, vyfi);
        midlevel_controller.setFeedback( xs, ys, vxs, vys, yaws, zs);
        midlevel_controller.setReference( xci, yci, yawci, zci, vxfi, vyfi);
        midlevel_controller.getOutput( &pitchco_hf, &rollco_hf, &dyawco_hf, &dzco_hf);
        break;
    case Controller_MidLevel_controlMode::POSITION_CONTROL:
        midlevel_controller.setFeedback( xs, ys, vxs, vys, yaws, zs);
        midlevel_controller.setReference( xci, yci, yawci, zci);
        midlevel_controller.getOutput( &pitchco_hf, &rollco_hf, &dyawco_hf, &dzco_hf);
        break;
    }

    // (low pass) filter the command outputs
    pitch_lowpassfilter.setInput(pitchco_hf);
    pitchco = pitch_lowpassfilter.getOutput();
    roll_lowpassfilter.setInput(rollco_hf);
    rollco = roll_lowpassfilter.getOutput();
    dyaw_lowpassfilter.setInput(dyawco_hf);
    dyawco = dyaw_lowpassfilter.getOutput();
    dz_lowpassfilter.setInput(dzco_hf);
    dzco = dz_lowpassfilter.getOutput();

    *p_pitchco = pitchco;
    *p_rollco  = rollco;
    *p_dyawco  = dyawco;
    *p_dzco    = dzco;
}

SM_stateNames::stateNames DroneTrajectoryController::getCurrentState() {
    return state_machine.getCurrentState();
}

int DroneTrajectoryController::getCheckpoint() {
    return state_machine.getCheckpoint();
}

int DroneTrajectoryController::getTrueCheckpoint() {
    return state_machine.getTrueCheckpoint();
}

void DroneTrajectoryController::referenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF(
        double x1, double y1, double z1, double yaw1, double pitch1, double roll1,
        double &x2, double &y2, double &z2, double &yaw2, double &pitch2, double &roll2) {
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_GMR_wrt_GFF, x1, y1, z1, yaw1, pitch1, roll1);
    // Note that, matHomog_drone_GMR_wrt_drone_LMrT = matHomog_GFF_wrt_LMrTFF
    matHomog_drone_LMrT_wrt_LMrTFF = matHomog_drone_GMR_wrt_drone_LMrT*matHomog_drone_GMR_wrt_GFF*matHomog_drone_LMrT_wrt_drone_GMR;
    referenceFrames::getxyzYPRfromHomogMatrix_wYvPuR( matHomog_drone_LMrT_wrt_LMrTFF, &x2, &y2, &z2, &yaw2, &pitch2, &roll2);
}

void DroneTrajectoryController::referenceChange_from1_drone_LMrT_wrt_LMrTFF_to2_drone_GMR_wrt_GFF(
        double x1, double y1, double z1, double yaw1, double pitch1, double roll1,
        double &x2, double &y2, double &z2, double &yaw2, double &pitch2, double &roll2) {
    // This function is still unused, but it would be necessary if we needed to get information out of the controller
    // For instance for logging purposes...
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_LMrT_wrt_LMrTFF, x1, y1, z1, yaw1, pitch1, roll1);
    // Note that, matHomog_LMrtFF_wrt_GFF = matHomog_drone_LMrT_wrt_drone_GMR
    matHomog_drone_GMR_wrt_GFF = matHomog_drone_LMrT_wrt_drone_GMR*matHomog_drone_LMrT_wrt_LMrTFF*matHomog_drone_GMR_wrt_drone_LMrT;
    referenceFrames::getxyzYPRfromHomogMatrix_wYvPuR( matHomog_drone_GMR_wrt_GFF, &x2, &y2, &z2, &yaw2, &pitch2, &roll2);
}

void DroneTrajectoryController::speedReferenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF(
        double vx1, double vy1, double vz1, double dyaw1, double dpitch1, double droll1,
        double &vx2, double &vy2, double &vz2, double &dyaw2, double &dpitch2, double &droll2) {
    vx2     = +vx1;
    vy2     = -vy1;
    vz2     = -vz1;
    dyaw2   = -dyaw1;
    dpitch2 = -dpitch1;
    droll2  = +droll1;
    return;
}

void DroneTrajectoryController::speedReferenceChange_from1_drone_LMrT_wrt_LMrTFF_to2_drone_GMR_wrt_GFF(
        double vx1, double vy1, double vz1, double dyaw1, double dpitch1, double droll1,
        double &vx2, double &vy2, double &vz2, double &dyaw2, double &dpitch2, double &droll2) {
    vx2     = +vx1;
    vy2     = -vy1;
    vz2     = -vz1;
    dyaw2   = -dyaw1;
    dpitch2 = -dpitch1;
    droll2  = +droll1;
    return;
}

std::string DroneTrajectoryController::controlMode2String() {

    switch (control_mode)
    {
    case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
        return ("trajectory");
        break;
    case Controller_MidLevel_controlMode::SPEED_CONTROL:
        return ("speed");
        break;
    case Controller_MidLevel_controlMode::POSITION_CONTROL:
        return ("position");
        break;
    default:
        return ("unknown");
        break;
    }

}

void DroneTrajectoryController::logControllerState(bool is_started) {

    SM_stateNames::stateNames current_sm_state = state_machine.getCurrentState();
    midlevel_controller.getIntermediateVars( &vxco_int, &vyco_int, &yawco_int, &zco_int, &vxfi, &vyfi, &dzfi, &dyawfi);

    float xci_out, yci_out, zci_out, yawci_out, pitchfi_out, rollfi_out;
    getCurrentPositionReference( &xci_out, &yci_out, &zci_out, &yawci_out, &pitchfi_out, &rollfi_out);
    float vxfi_out, vyfi_out, dzfi_out, dyawfi_out;
    getCurrentSpeedReference( &vxfi_out, &vyfi_out, &dzfi_out, &dyawfi_out);
    double vxco_int_out, vyco_int_out, dzco_hf_out, dyawco_hf_out, dpitch2_aux, droll2_aux;
    speedReferenceChange_from1_drone_LMrT_wrt_LMrTFF_to2_drone_GMR_wrt_GFF( vxco_int,     vyco_int,     dzco_hf,     dyawco_hf,     0.0,         0.0,
                                                                            vxco_int_out, vyco_int_out, dzco_hf_out, dyawco_hf_out, dpitch2_aux, droll2_aux);

    debug_string_stacker
//              /* timestamp   NOTE: done in drone logger */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
        /* tag         */   << "[ctrlr;state]"
        /* isStarted   */   << " started:" << is_started
        /* controlMode */   << " mode:"    << controlMode2String()
        /* StateMachineState */ << " sm_mode:" << state_machine.getCurrentStateName(current_sm_state)
        /* references  */
        /* xci, yci, zci, yawci */      << " xci:"   << xci_out
                                        << " yci:"   << yci_out
                                        << " zci:"   << zci_out
                                        << " yawci:" << yawci_out
        /* vxfi, vyfi, dzfi, dyawfi */  << " vxfi:"   << vxfi_out
                                        << " vyfi:"   << vyfi_out
                                        << " dzfi:"   << dzfi_out
                                        << " dyawfi:" << dyawfi_out
        /* vxco_int, vyco_int       */  << " vxco_int:" <<  vxco_int_out
        /* dzco_total, dyawco_total */  << " vyco_int:" <<  vyco_int_out
                                        << " dzco_t:"   << -dzco_hf_out     // sign is correct, I am no longer sure, requires debug to know
                                        << " dyawco_t:" <<  dyawco_hf_out
        /* pitchfi, rollfi*/            << " pitchfi:" << pitchfi_out
                                        << " rollfi:"  << rollfi_out
        /* commands    */
        /* pitchco, rollco */   << " Pco:"  << pitchco
                                << " Rco:"  << rollco
        /* dzco, dyawco */      << " dzco:" << dzco
                                << " dYco:" << dyawco
        /* EKF_feedback*/
        /* xs, ys, zs  */   << " xs:" << xs
                            << " ys:" << ys
                            << " zs:" << zs
        /* vxs, vys, vzs */ << " vxs:" << vxs
                            << " vys:" << vys
                            << " vzs:" << vzs
        /* yaws, dyaws */   << " yaws:"  << yaws
                            << " dyaws:" << dyaws
                            << std::endl;
}

void DroneTrajectoryController::logControllerGains() {
    double Kp_vxm2P, Ki_vxm2P, Kd_vxm2P,
           Kp_vym2R, Ki_vym2R, Kd_vym2R,
           Kp_z2vz,  Ki_z2vz,  Kd_z2vz,
           Kp_Y2dY,  Ki_Y2dY,  Kd_Y2dY,
           Kp_x2vxm, Ki_x2vxm, Kd_x2vxm,
           Kp_y2vym, Ki_y2vym, Kd_y2vym;

    midlevel_controller.getGains(Kp_vxm2P, Ki_vxm2P, Kd_vxm2P,
               Kp_vym2R, Ki_vym2R, Kd_vym2R,
               Kp_z2vz,  Ki_z2vz,  Kd_z2vz,
               Kp_Y2dY,  Ki_Y2dY,  Kd_Y2dY,
               Kp_x2vxm, Ki_x2vxm, Kd_x2vxm,
               Kp_y2vym, Ki_y2vym, Kd_y2vym);

        debug_string_stacker
//              /* timestamp   NOTE: done in drone logger */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
                /* tag         */   << " [ctrlr;gains]"
                /* ctrlr x2vxm */   /* Kp_x2vxm, Ki_x2vxm, Kd_x2vxm */
                                    << " Kp_x2vxm:" << Kp_x2vxm
                                    << " Ki_x2vxm:" << Ki_x2vxm
                                    << " Kd_x2vxm:" << Kd_x2vxm
                /* ctrlr y2vym */   /* Kp_y2vym, Ki_y2vym, Kd_y2vym */
                                    << " Kp_y2vym:" << Kp_y2vym
                                    << " Ki_y2vym:" << Ki_y2vym
                                    << " Kd_y2vym:" << Kd_y2vym
                /* ctrlr vxm2P */   /* Kp_vxm2P, Ki_vxm2P, Kd_vxm2P */
                                    << " Kp_vxm2P:" << Kp_vxm2P
                                    << " Ki_vxm2P:" << Ki_vxm2P
                                    << " Kd_vxm2P:" << Kd_vxm2P
                /* ctrlr vym2R */   /* Kp_vym2R, Ki_vym2R, Kd_vym2R */
                                    << " Kp_vym2R:" << Kp_vym2R
                                    << " Ki_vym2R:" << Ki_vym2R
                                    << " Kd_vym2R:" << Kd_vym2R
                /* ctrlr z2vz  */   /* Kp_z2vz,  Ki_z2vz,  Kd_z2vz  */
                                    << " Kp_z2vz:" << Kp_z2vz
                                    << " Ki_z2vz:" << Ki_z2vz
                                    << " Kd_z2vz:" << Kd_z2vz
                /* ctrlr Y2dY  */   /* Kp_Y2dY,  Ki_Y2dY,  Kd_Y2dY  */
                                    << " Kp_Y2dY:" << Kp_Y2dY
                                    << " Ki_Y2dY:" << Ki_Y2dY
                                    << " Kd_Y2dY:" << Kd_Y2dY
                                    << std::endl;
        debug_string_stacker.setPriorityFlag();
}

void DroneTrajectoryController::getCurrentPositionReference( float *x_out, float *y_out, float *z_out, float *yaw_out, float *pitch_out, float *roll_out) {
    double x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux;
    double x2_aux, y2_aux, z2_aux, yaw2_aux, pitch2_aux, roll2_aux;
    midlevel_controller.getCurrentPositionReference( &x_aux, &y_aux, &z_aux, &yaw_aux, &pitch_aux, &roll_aux);
    referenceChange_from1_drone_LMrT_wrt_LMrTFF_to2_drone_GMR_wrt_GFF( x_aux,  y_aux,  z_aux,  yaw_aux,  pitch_aux,  roll_aux,
                                                                       x2_aux, y2_aux, z2_aux, yaw2_aux, pitch2_aux, roll2_aux);
    (*x_out)     = x2_aux;
    (*y_out)     = y2_aux;
    (*z_out)     = z2_aux;
    (*yaw_out)   = yaw2_aux;
    (*pitch_out) = pitch2_aux;
    (*roll_out)  = roll2_aux;
}

void DroneTrajectoryController::getCurrentSpeedReference( float *dx_out, float *dy_out, float *dz_out, float *dyaw_out) {
    double dx_aux, dy_aux, dz_aux, dyaw_aux;
    double dx2_aux, dy2_aux, dz2_aux, dyaw2_aux, dpitch2_aux, droll2_aux;
    midlevel_controller.getCurrentSpeedReference( &dx_aux, &dy_aux, &dz_aux, &dyaw_aux);
    speedReferenceChange_from1_drone_LMrT_wrt_LMrTFF_to2_drone_GMR_wrt_GFF( dx_aux,  dy_aux,  dz_aux,  dyaw_aux,   0.0,        0.0,
                                                                            dx2_aux, dy2_aux, dz2_aux, dyaw2_aux, dpitch2_aux, droll2_aux);

    (*dx_out)   = dx2_aux;
    (*dy_out)   = dy2_aux;
    (*dz_out)   = dz2_aux;
    (*dyaw_out) = dyaw2_aux;
}

void DroneTrajectoryController::getCurrentTrajectoryReference( std::vector<SimpleTrajectoryWaypoint> *trajectory_waypoints_out, int *initial_checkpoint_out, bool *is_periodic_out) {
    switch (control_mode) {
    case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL: {
        state_machine.getCurrentTrajectoryReference( trajectory_waypoints_out,
                                                     initial_checkpoint_out,
                                                     is_periodic_out);
//        std::cout << "void DroneTrajectoryController::getCurrentTrajectoryReference( ... )" << std::endl;
//        int checkpoint_count = 0;
//        for ( auto it : (*trajectory_waypoints_out) ) {
//            std::cout << "drone_LMrT_wrt_LMrTFF, chk:" << checkpoint_count << " x:" << it.x << " y:" << it.y << " z:" << it.z << std::endl;
//            checkpoint_count++;
//        }

//        checkpoint_count = 0;
        for ( std::vector<SimpleTrajectoryWaypoint>::iterator it = (*trajectory_waypoints_out).begin();
              it != (*trajectory_waypoints_out).end();
              it++) {
            double x_aux, y_aux, z_aux, yaw_aux = 0.0, pitch_aux = 0.0, roll_aux = 0.0;
            double x2_aux, y2_aux, z2_aux, yaw2_aux, pitch2_aux, roll2_aux;
            x_aux = (*it).x;
            y_aux = (*it).y;
            z_aux = (*it).z;
//            std::cout << "drone_LMrT_wrt_LMrTFF, chk:" << checkpoint_count << " x:" << (*it).x << " y:" << (*it).y << " z:" << (*it).z << std::endl;
            referenceChange_from1_drone_LMrT_wrt_LMrTFF_to2_drone_GMR_wrt_GFF( x_aux,  y_aux,  z_aux,  yaw_aux,  pitch_aux,  roll_aux,
                                                                               x2_aux, y2_aux, z2_aux, yaw2_aux, pitch2_aux, roll2_aux);
            (*it).x = x2_aux;
            (*it).y = y2_aux;
            (*it).z = z2_aux;
//            std::cout << "drone_GMR_wrt_GFF,     chk:" << checkpoint_count << " x:" << (*it).x << " y:" << (*it).y << " z:" << (*it).z << std::endl;
//            checkpoint_count++;
        }

//        checkpoint_count = 0;
//        for ( auto it : (*trajectory_waypoints_out) ) {
//            std::cout << "drone_GMR_wrt_GFF,     chk:" << checkpoint_count << " x:" << it.x << " y:" << it.y << " z:" << it.z << std::endl;
//            checkpoint_count++;
//        }
    } break;
    case Controller_MidLevel_controlMode::POSITION_CONTROL:
    case Controller_MidLevel_controlMode::SPEED_CONTROL:
    case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
    default:
        trajectory_waypoints_out->clear();
        (*initial_checkpoint_out) = 0;
        (*is_periodic_out)        = false;
        break;
    }
}

void DroneTrajectoryController::setFeedback_drone_pose_LMrT_wrt_LMrTFF(double xs_t, double ys_t, double zs_t, double yaws_t, double pitchs_t, double rolls_t)
{
    xs = xs_t;
    ys = ys_t;
    yaws = yaws_t;
    zs = zs_t;
//    pitchs = pitchs_t; // unused, undefined
//    rolls = rolls_t;   // unused, undefined
}
