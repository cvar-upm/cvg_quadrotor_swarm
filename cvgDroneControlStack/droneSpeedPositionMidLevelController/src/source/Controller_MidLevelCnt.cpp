/*
 * Controller_MidLevelCnt.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#include "Controller_MidLevelCnt.h"

Controller_MidLevelCnt::Controller_MidLevelCnt(int idDrone, const std::string &stackPath_in) :
    speedController(idDrone, stackPath_in),
    started(false)
    {
        std::cout << "Constructor: Controller_MidLevelCnt" << std::endl;
        try {
        XMLFileReader my_xml_reader( stackPath_in+"configs/drone"+std::to_string(idDrone)+"/trajectory_controller_config.xml");

        // Just in case, I set to zero all the variables of the controller object
        xci = 0; yci = 0; yawci = 0; zci = 0;
        vxfi = 0.0; vyfi = 0.0; dyawfi = 0.0; dzfi = 0.0; pitchfi = 0.0; rollfi = 0.0;
        xs = 0; ys = 0; vxs = 0; vys = 0; yaws = 0; zs = 0;
        eps_x = 0; eps_y = 0;
        vxc_int = 0; vyc_int = 0;
        vxco_int = 0; vyco_int = 0; yawco_int = 0; zco_int = 0;
        pitchco = 0; rollco = 0; dyawco = 0; dzco = 0;

        speedController.reset();

        double vxy_at_ratio;
        vxy_max = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","speed_control_loop","vxy_max"} );
        vxy_at_ratio = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","speed_control_loop","vxy_at_ratio"} );
        vxy_at_max =    vxy_at_ratio  * vxy_max;
        vxy_ct_max = (1-vxy_at_ratio) * vxy_max;
        double dz_at_ratio;
        dz_max    = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","dz_max"} );
        dz_at_ratio = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","speed_control_loop","dz_at_ratio"} );
        dz_at_max =    dz_at_ratio  * dz_max;
        dz_ct_max = (1-dz_at_ratio) * dz_max;
        double xy_Kp, xy_Ki, xy_Kd, xy_DeltaKp;
        xy_DeltaKp = my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","position_controller","xy","DeltaKp"} );
        xy_Kp = xy_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","position_controller","xy","Kp"} );
        xy_Ki = xy_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","position_controller","xy","Ki"} );
        xy_Kd = xy_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","position_controller","xy","Kd"} );

        pid_x.setGains( xy_Kp, xy_Ki, xy_Kd);
        pid_x.enableMaxOutput(true,  vxy_max);
        pid_x.enableAntiWindup(true, vxy_max);

        pid_y.setGains( xy_Kp, xy_Ki, xy_Kd);
        pid_y.enableMaxOutput( true, vxy_max);
        pid_y.enableAntiWindup(true, vxy_max);

        std::string init_control_mode_str = my_xml_reader.readStringValue( {"trajectory_controller_config","init_control_mode"} );
        if ( init_control_mode_str.compare("speed") == 0 ) {
            init_control_mode = Controller_MidLevel_controlMode::SPEED_CONTROL;
        } else { if ( init_control_mode_str.compare("position") == 0 ) {
            init_control_mode = Controller_MidLevel_controlMode::POSITION_CONTROL;
        } else {                             // "trajectory"
            init_control_mode = Controller_MidLevel_controlMode::SPEED_CONTROL;
            throw std::runtime_error("Controller_MidLevelCnt::Controller_MidLevelCnt, inital control_mode cannot be TRAJECTORY_CONTROL");
            return;
            }
        }
        setControlMode(init_control_mode);
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}

Controller_MidLevelCnt::~Controller_MidLevelCnt() {
}

void Controller_MidLevelCnt::setFeedback( double xs_t, double ys_t, double vxs_t, double vys_t, double yaws_t, double zs_t) {
	xs = xs_t; ys = ys_t;
	vxs = vxs_t; vys = vys_t;
	yaws = yaws_t;
	zs = zs_t;
}

void Controller_MidLevelCnt::setReference( double xci_t, double yci_t, double yawci_t, double zci_t,
        double vxfi_t , double vyfi_t , double dyawfi_t , double dzfi_t ,
        double pitchfi_t , double rollfi_t ) {
	xci = xci_t; yci = yci_t; yawci = yawci_t; zci = zci_t;
	vxfi = vxfi_t; vyfi = vyfi_t; dyawfi = dyawfi_t; dzfi = dzfi_t;
	pitchfi = pitchfi_t; rollfi = rollfi_t;
}

void Controller_MidLevelCnt::getOutput( double *pitchco_out, double *rollco_out, double *dyawco_out, double *dzco_out) {

	if (!started) {
		reset();
		*pitchco_out = 0.0;
		*rollco_out  = 0.0;
		*dyawco_out  = 0.0;
		*dzco_out    = 0.0;
		started = true;
		return;
	}

	// Calculate vxc for speed controller
	pid_x.setReference(xci);
	pid_x.setFeedback(xs);
	vxc_int = pid_x.getOutput();

	// Calculate vyc for speed controller
	pid_y.setReference(yci);
	pid_y.setFeedback(ys);
	vyc_int = pid_y.getOutput();

	switch (control_mode) {
	case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
        saturation_2D( vxc_int, vyc_int, &vxc_int, &vyc_int, vxy_ct_max);
        saturation_2D( vxfi, vyfi, &vxfi, &vyfi, vxy_at_max);
		vxco_int = vxc_int + vxfi;
		vyco_int = vyc_int + vyfi;
        dzfi = cvg_utils_library::saturate(dzfi, -dz_at_max, dz_at_max);
		// No yaw AT, CT saturation considered for now; dyawfi will usually be 0.0
		break;
	case Controller_MidLevel_controlMode::POSITION_CONTROL:
		vxfi = 0.0;
		vyfi = 0.0;
		dzfi = 0.0;
		// No yaw AT, CT saturation considered for now; dyawfi will usually be 0.0
		// No saturation command here, why?: saturation of speed reference on speed controller
		vxco_int = vxc_int;
		vyco_int = vyc_int;
		break;
	case Controller_MidLevel_controlMode::SPEED_CONTROL:
		// No saturation command here, why?: saturation of speed reference on speed controller
		vxco_int = vxfi;
		vyco_int = vyfi;
		dzfi = 0.0;	// altitude is also position controlled in this mode
		// No yaw AT, CT saturation considered for now; dyawfi will usually be 0.0
		break;
	}

	// Yaw and Z commands pass directly to speed controller
	yawco_int = yawci;
	zco_int = zci;

	// Enter references to speed controller
    speedController.setReference( vxco_int, vyco_int, yawco_int, zco_int);
	// Enter feedback measurements/estimations to speed controller
	speedController.setFeedback( vxs, vys, yaws, zs);
	// Obtain controller outputs (control commands for the pelican proxy)
	speedController.getOutput( &pitchco, &rollco, &dyawco, &dzco);

	*pitchco_out = pitchco + pitchfi;
	*rollco_out  = rollco  + rollfi;
    *dyawco_out  = dyawco  + dyawfi;
    *dzco_out    = dzco    - dzfi;
    return;
}

void Controller_MidLevelCnt::reset() {

	// Just in case, I set to zero all the variables of the controller object
    xci = 0.0; yci = 0.0; yawci = 0.0; zci = 0.0;
	vxfi = 0.0; vyfi = 0.0; dyawfi = 0.0; dzfi = 0.0; pitchfi = 0.0; rollfi = 0.0;
    xs = 0.0; ys = 0.0; vxs = 0.0; vys = 0.0; yaws = 0.0; zs = 0.0;
    eps_x = 0.0; eps_y = 0.0;
    vxc_int = 0.0; vyc_int = 0.0;
    vxco_int = 0.0; vyco_int = 0.0; yawco_int = 0.0; zco_int = 0.0;
    pitchco = 0.0; rollco = 0.0; dyawco = 0.0; dzco = 0.0;

	started = false;
	// Reset speed controller
	speedController.reset();

	// Reset internal PIDs
	pid_x.reset();
	pid_y.reset();

    setControlMode(init_control_mode);
}

void Controller_MidLevelCnt::getIntermediateVars( double *vxco_int_out, double *vyco_int_out, double *yawco_int_out, double *zco_int_out,
                                                  double *vxfi_out, double *vyfi_out, double *dzfi_out, double *dyawfi_out) {
	*vxco_int_out  = vxco_int;
	*vyco_int_out  = vyco_int;
    if ( yawco_int_out != NULL )
        *yawco_int_out  = yawco_int;
    if ( zco_int_out != NULL )
        *zco_int_out  = zco_int;
    if ( vxfi_out != NULL )
        *vxfi_out  = vxfi;
    if ( vyfi_out != NULL )
        *vyfi_out  = vyfi;
    if ( dzfi_out != NULL )
        *dzfi_out  = dzfi;
    if ( dyawfi_out != NULL )
        *dyawfi_out  = dyawfi;
}

void Controller_MidLevelCnt::saturation_2D(double x1, double x2, double *y1, double *y2, double max) {

    double modulus = sqrt( pow( x1 ,2) + pow( x2 ,2) );

	if ( fabs(modulus) > max ) {
		*y1 = (max/modulus)*x1;
		*y2 = (max/modulus)*x2;
	} else {
		*y1 = x1;
		*y2 = x2;
	}

}

void Controller_MidLevelCnt::setControlMode(Controller_MidLevel_controlMode::controlMode mode) {

	switch (mode) {
	case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
        pid_x.enableMaxOutput( true, vxy_ct_max);
        pid_x.enableAntiWindup(true, vxy_ct_max);
        pid_y.enableMaxOutput( true, vxy_ct_max);
        pid_y.enableAntiWindup(true, vxy_ct_max);
		break;
	case Controller_MidLevel_controlMode::POSITION_CONTROL:
	case Controller_MidLevel_controlMode::SPEED_CONTROL:
        pid_x.enableMaxOutput(true,  vxy_max);
        pid_x.enableAntiWindup(true, vxy_max);
        pid_y.enableMaxOutput( true, vxy_max);
        pid_y.enableAntiWindup(true, vxy_max);
		break;
	}

	speedController.setControlMode(mode);

	control_mode = mode;
}

Controller_MidLevel_controlMode::controlMode Controller_MidLevelCnt::getControlMode() {
	return control_mode;
}


void Controller_MidLevelCnt::getCurrentPositionReference(double *xci_out, double *yci_out, double *zci_out, double *yawci_out, double *pitchfi_out, double *rollfi_out) {
    (*xci_out)     = xci;
    (*yci_out)     = yci;
    (*zci_out)     = zci;
    (*yawci_out)   = yawci;
    (*pitchfi_out) = pitchfi;
    (*rollfi_out)  = rollfi;
}

void Controller_MidLevelCnt::getCurrentSpeedReference( double *dxfi_out, double *dyfi_out, double *dzfi_out, double *dyawfi_out) {
    (*dxfi_out) = vxfi;
    (*dyfi_out)   = vyfi;
    (*dzfi_out)   = dzfi;
    (*dyawfi_out) = dyawfi;
}
