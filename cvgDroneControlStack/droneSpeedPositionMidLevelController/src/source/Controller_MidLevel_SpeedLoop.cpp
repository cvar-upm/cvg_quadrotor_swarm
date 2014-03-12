/*
 * Controller_MidLevel_SpeedLoop.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#include "Controller_MidLevel_SpeedLoop.h"

Controller_MidLevel_SpeedLoop::Controller_MidLevel_SpeedLoop(int idDrone, const std::string &stackPath_in) /* throw(std::runtime_error) */ :  started(false) {
    std::cout << "Constructor: Controller_MidLevel_SpeedLoop" << std::endl;
    try {
        XMLFileReader my_xml_reader( stackPath_in+"configs/drone"+std::to_string(idDrone)+"/trajectory_controller_config.xml");

        // Just in case, I set to zero all the internal variables of the controller object
        vxci = 0.0; vyci = 0.0; yawci = 0.0; zci = 0.0;
        vxs = 0.0; vys = 0.0; yaws = 0.0; zs = 0.0;
        eps_vx = 0.0; eps_vy = 0.0; eps_yaw = 0.0; eps_z = 0.0;
        vxd_int = 0.0; vyd_int = 0.0;
        pitchd = 0.0; rolld = 0.0;
        pitchc_int = 0.0; rollc_int = 0.0; dyawc_int = 0.0; dzc_int = 0.0;
        pitchco = 0.0; rollco = 0.0; dyawco = 0.0; dzco = 0.0;
        nlf_is_activated = false;

        // Nacho's PID algorithm >>>> u[k] = Kp*e[k] + Ki*i[k] + Kd*de[k]

        // read configuration file
        double vxy_Kp, vxy_Ki, vxy_Kd, vxy_DeltaKp;
        vxy_DeltaKp = my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","vxy","DeltaKp"} );
        vxy_Kp = vxy_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","vxy","Kp"} );
        vxy_Ki = vxy_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","vxy","Ki"} );
        vxy_Kd = vxy_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","vxy","Kd"} );
        nlf_is_activated = my_xml_reader.readIntValue( {"trajectory_controller_config","middle_level_controller","speed_controller","vxy","nlf_is_activated"} );
        vxy_max = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","speed_control_loop","vxy_max"} );
        double pitch_max, roll_max, dyaw_max, dz_at_ratio;
        pitch_max = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","pitch_max"} );
        roll_max  = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","roll_max"} );
        dyaw_max  = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","dyaw_max"} );
        dz_max    = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","dz_max"} );
        dz_at_ratio = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","speed_control_loop","dz_at_ratio"} );
        dz_ct_max = (1-dz_at_ratio) * dz_max;
        double yaw_Kp, yaw_Ki, yaw_Kd, yaw_DeltaKp;
        yaw_DeltaKp = my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","yaw","DeltaKp"} );
        yaw_Kp = yaw_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","yaw","Kp"} );
        yaw_Ki = yaw_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","yaw","Ki"} );
        yaw_Kd = yaw_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","yaw","Kd"} );
        double z_Kp, z_Ki, z_Kd, z_DeltaKp;
        z_DeltaKp = my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","altitude","DeltaKp"} );
        z_Kp = z_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","altitude","Kp"} );
        z_Ki = z_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","altitude","Ki"} );
        z_Kd = z_DeltaKp*my_xml_reader.readDoubleValue( {"trajectory_controller_config","middle_level_controller","speed_controller","altitude","Kd"} );


        // Vx PID gains: Kp, Ki = Kp/Ti, Kd = Kp*Td
        pid_vx.setGains( vxy_Kp, vxy_Ki, vxy_Kd); // set Kp, Ki = Kp/Ti, Kd = Kp*Td
        if (!nlf_is_activated) {
            // Saturation specified by config file
            pid_vx.enableAntiWindup(true, pitch_max);
            pid_vx.enableMaxOutput( true, pitch_max); // set enable, value. Maximum output is vx_max = 2.5-2.75 m/s
        } else {
        //  Saturation performed by non-linear function
        // (Parrot 1) Maximum output is vx_max = 2.5-2.75 m/s
        pid_vx.enableAntiWindup(true, vxy_max);
        pid_vx.enableMaxOutput( true, vxy_max);
        }

        // Vy PID gains: Kp, Ki = Kp/Ti, Kd = Kp*Td
        pid_vy.setGains( vxy_Kp, vxy_Ki, vxy_Kd);
        if (!nlf_is_activated) {
            // Saturation specified by config file
            pid_vy.enableAntiWindup(true, roll_max);
            pid_vy.enableMaxOutput( true, roll_max);
        } else {
        //  Saturation performed by non-linear function
        // (Parrot 1) Maximum output is vy_max = 2.0-2.30 m/s
        pid_vy.enableAntiWindup(true, vxy_max);
        pid_vy.enableMaxOutput( true, vxy_max);
        }

        // yaw PID gains: Kp, Ki = Kp/Ti, Kd = Kp*Td
        pid_yaw.setGains( 	yaw_Kp, yaw_Ki, yaw_Kd);
        pid_yaw.enableMaxOutput(true,  dyaw_max);
        pid_yaw.enableMaxOutput( true, dyaw_max);

        // z PID gains: Kp, Ki = Kp/Ti, Kd = Kp*Td
        pid_z.setGains( z_Kp, z_Ki, z_Kd);
        pid_z.enableMaxOutput( true,  dz_ct_max);
        pid_z.enableMaxOutput( true,  dz_ct_max);

        std::string init_control_mode_str = my_xml_reader.readStringValue( {"trajectory_controller_config","init_control_mode"} );
        if ( init_control_mode_str.compare("speed") == 0 ) {
            init_control_mode = Controller_MidLevel_controlMode::SPEED_CONTROL;
        } else { if ( init_control_mode_str.compare("position") == 0 ) {
            init_control_mode = Controller_MidLevel_controlMode::POSITION_CONTROL;
        } else {                             // "trajectory"
            init_control_mode = Controller_MidLevel_controlMode::SPEED_CONTROL;
            throw std::runtime_error("Controller_MidLevel_SpeedLoop::Controller_MidLevel_SpeedLoop, inital control_mode cannot be TRAJECTORY_CONTROL");
            return;
            }
        }
        setControlMode(init_control_mode);
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}

Controller_MidLevel_SpeedLoop::~Controller_MidLevel_SpeedLoop() {
}

void Controller_MidLevel_SpeedLoop::getOutput( double *pitchco_out, double *rollco_out, double *dyawco_out, double *dzco_out) {


	if (!started) {
		reset();
		*pitchco_out = 0.0;
		*rollco_out  = 0.0;
		*dyawco_out  = 0.0;
		*dzco_out    = 0.0;
		started = true;
		return;
	}

    // Saturate speed reference input to MULTIROTOR_SPEEDCONTROLLER_VXY_MAX
    double vrci = sqrt( vxci*vxci + vyci*vyci);
    if ( vrci  > vxy_max ) {
        double Kaux = vxy_max / vrci;
		vxci = Kaux * vxci;
		vyci = Kaux * vyci;
	}

	// Perform PID_VX calculations and obtain vxd_int
	pid_vx.setReference(vxci);
	pid_vx.setFeedback(vxs);
	vxd_int = pid_vx.getOutput();
	// Perform PID_VY calculations and obtain vyd_int
	pid_vy.setReference(vyci);
	pid_vy.setFeedback(vys); // I put 0.0 because eps_vym is already (Reference - Feedback)
	vyd_int = pid_vy.getOutput();

	// Compute {Pitchd, Rolld} desired values from {vxd, vyd}
	referenceChangeFixed2Moving( vxd_int, vyd_int, yaws, &pitchd, &rolld);
	// Compute {Pitch} command from {Pitchd}
    if (!nlf_is_activated) {
        pitchc_int = (-1)*pitchd;
    } else {
        nlf_p2vx( pitchd, &pitchc_int);
    }
//		pitchco_saturation( pitchc_int, &pitchco); // Commented because: Saturation already performed either by NLF or by PID
	pitchco = pitchc_int;

	// Compute {Roll} command from {Rolld}
    if (!nlf_is_activated) {
        rollc_int = rolld;
    } else {
        nlf_r2vy( rolld, &rollc_int);
    }
	//	rollco_saturation( rollc_int, &rollco); // Commented because: Saturation already performed either by NLF or by PID
	rollco = rollc_int;

	// Compute {dYawdt} output command from {Yawc} input command
    double yaw_error = cvg_utils_library::getAngleError( yawci, yaws);
    pid_yaw.setReference(yaws+yaw_error); // pid_yaw.setReference(yawci);
	pid_yaw.setFeedback(yaws);
	dyawc_int = pid_yaw.getOutput();
	//	dyawco_saturation( dyawc_int, &dyawco); // Commented because: Saturation already performed either by PID
	dyawco = dyawc_int;

	// Compute {Z} output command from {Z} input command
	pid_z.setReference(zci);
	pid_z.setFeedback(zs);
	dzc_int = pid_z.getOutput();
	//	dzco_saturation( dzc_int, &dzco); // Commented because: Saturation already performed either by PID
	dzco = dzc_int;

	*pitchco_out = pitchco;
	*rollco_out  = rollco;
	*dyawco_out  = dyawco;
	*dzco_out    = dzco;

}

void Controller_MidLevel_SpeedLoop::reset() {

		// Just in case, I set to zero all the variables of the controller object
//		setYawAndZ2ActualValues(); // yawci = 0.0; zci = 0.0;
		vxci = 0.0; vyci = 0.0; vxs = 0.0; vys = 0.0; yaws = 0.0; zs = 0.0;
		pitchd = 0.0; rolld = 0.0; eps_yaw = 0.0; eps_z = 0.0; vxd_int = 0.0; vyd_int = 0.0;
		pitchc_int = 0.0; rollc_int = 0.0; dyawc_int = 0.0; dzc_int = 0.0;
		pitchco = 0.0; rollco = 0.0; dyawco = 0.0; dzco = 0.0;

		started = false;

		pid_vx.reset();
		pid_vy.reset();
		pid_z.reset();
		pid_yaw.reset();

        setControlMode(init_control_mode);
}

void Controller_MidLevel_SpeedLoop::referenceChangeFixed2Moving(double eps_vxs_f, double eps_vys_f, double yaws, double *eps_vxm, double *eps_vym) {

	*eps_vxm =  cos(yaws)*eps_vxs_f + sin(yaws)*eps_vys_f;
	*eps_vym = -sin(yaws)*eps_vxs_f + cos(yaws)*eps_vys_f;

}

void Controller_MidLevel_SpeedLoop::nlf_p2vx(double vxmd_int, double *pitchc_int) {
    double vx_max = 2.625; // m/s
    (*pitchc_int) = -(1.0/2.703)*cvg_utils_library::asin_ws(vxmd_int/vx_max);

//    double sin_beta = cvg_utils_library::saturate(vxmd_int/vx_max, -1, 1);
//    *pitchc_int = -(1.0/2.703)*asin(sin_beta);
}

void Controller_MidLevel_SpeedLoop::pitchco_saturation(double pitchc_int, double *pitchco) {

	// double cvg_utils_library::saturate(double x, double x_lim_inf, double x_lim_sup);
	*pitchco = cvg_utils_library::saturate(pitchc_int, -1.0, 1.0);

}

void Controller_MidLevel_SpeedLoop::nlf_r2vy(double vymd_int, double *rollc_int) {
    double vy_max = 2.172; // m/s
    (*rollc_int) = (1/2.961)*cvg_utils_library::asin_ws( vymd_int/vy_max );

//    double sin_beta = cvg_utils_library::saturate(vymd_int/vy_max, -1, 1);
//    *rollc_int = (1/2.961)*asin(sin_beta);
}

void Controller_MidLevel_SpeedLoop::rollco_saturation(double rollc_int, double *rollco) {

	// double cvg_utils_library::saturate(double x, double x_lim_inf, double x_lim_sup);
	*rollco = cvg_utils_library::saturate(rollc_int, -1.0, 1.0);

}

void Controller_MidLevel_SpeedLoop::dyawco_saturation(double dyawc_int, double *dyawco) {

	// double cvg_utils_library::saturate(double x, double x_lim_inf, double x_lim_sup);
	*dyawco = cvg_utils_library::saturate(dyawc_int, -1.0, 1.0);

}

void Controller_MidLevel_SpeedLoop::dzco_saturation(double dzc_int, double *dzco) {

	// double cvg_utils_library::saturate(double x, double x_lim_inf, double x_lim_sup);
//	*dzco = cvg_utils_library::saturate(dzc_int, -1.0, 1.0);
	*dzco = cvg_utils_library::saturate(dzc_int, -0.4, 0.4);

}

void Controller_MidLevel_SpeedLoop::setControlMode(Controller_MidLevel_controlMode::controlMode mode){

	switch (mode) {
	case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
        pid_z.enableMaxOutput( true,  dz_ct_max);
        pid_z.enableAntiWindup( true,  dz_ct_max);
		break;
	case Controller_MidLevel_controlMode::POSITION_CONTROL:
	case Controller_MidLevel_controlMode::SPEED_CONTROL:
        pid_z.enableMaxOutput( true,  dz_max);
        pid_z.enableAntiWindup( true,  dz_max);
		break;
	}

	control_mode = mode;
}

Controller_MidLevel_controlMode::controlMode Controller_MidLevel_SpeedLoop::getControlMode() {
	return control_mode;
}
