/*
 * Controller_MidLevel_SpeedLoop.h
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_MIDLEVEL_SPEEDLOOP_H_
#define CONTROLLER_MIDLEVEL_SPEEDLOOP_H_

#include "cvg_utils_library.h"
#include "control/PID.h"
#include <math.h>
#include "control/Controller_MidLevel_controlModes.h"
#include "xmlfilereader.h"
#include <string>

class Controller_MidLevel_SpeedLoop {
private:
	// Controller command inputs
    double vxci, vyci, yawci, zci;			// ci ~ command inputs
	// Controller state estimate inputs (controller feedback)
    double vxs, vys, yaws, zs;
	// Intermediate variables between controller blocks
    double eps_vx, eps_vy, eps_yaw, eps_z;// eps ~ epsilon to denote the control error in the commanded variable
    double vxd_int, vyd_int;				// d ~ desired value; int ~ internal variable
    double pitchd, rolld;
    double pitchc_int, rollc_int, dyawc_int, dzc_int;
	// Next control layer commands, in this case these commands are to be sent directly to the drone using the parrot proxy
    double pitchco, rollco, dyawco, dzco;	// co ~ command outputs
    bool   nlf_is_activated;
    double dz_max, dz_ct_max, vxy_max;

	// Internal blocks of the controller diagram, PID and others
	CVG_BlockDiagram::PID pid_vx, pid_vy, pid_z, pid_yaw;
    void referenceChangeFixed2Moving(double eps_vxs_f, double eps_vys_f, double yaws, double *eps_vxm, double *eps_vym);
    void nlf_p2vx(double vxmd_int, double *pitchc_int);
    void pitchco_saturation(double pitchc_int, double *pitchco);
    void nlf_r2vy(double vymd_int, double *rollc_int);
    void rollco_saturation(double rollc_int, double *rollco);
    void dyawco_saturation(double dyawc_int, double *dyawco);
    void dzco_saturation(double dzc_int, double *dzco);

    volatile bool started;
    Controller_MidLevel_controlMode::controlMode control_mode, init_control_mode;

protected:

public:
    Controller_MidLevel_SpeedLoop(int idDrone, const std::string &stackPath_in); // throw(std::runtime_error);
	~Controller_MidLevel_SpeedLoop();

	void reset();

    inline void setFeedback( double vxs_t, double vys_t, double yaws_t, double zs_t) {vxs = vxs_t; vys = vys_t; yaws = yaws_t; zs = zs_t;}
    inline void setReference( double vxci_t, double vyci_t, double yawci_t, double zci_t) {vxci = vxci_t; vyci = vyci_t; yawci = yawci_t; zci = zci_t;}
    void getOutput( double *pitchco, double *rollco, double *dyawco, double *dzco);

	void setControlMode(Controller_MidLevel_controlMode::controlMode mode);
	Controller_MidLevel_controlMode::controlMode getControlMode();

    inline void getGains( double &Kp_vxm2P, double &Ki_vxm2P, double &Kd_vxm2P,
                          double &Kp_vym2R, double &Ki_vym2R, double &Kd_vym2R,
                          double &Kp_z2vz,  double &Ki_z2vz,  double &Kd_z2vz,
                          double &Kp_Y2dY,  double &Ki_Y2dY,  double &Kd_Y2dY) {
        pid_vx.getGains( Kp_vxm2P, Ki_vxm2P, Kd_vxm2P);
        pid_vy.getGains( Kp_vym2R, Ki_vym2R, Kd_vym2R);
        pid_z.getGains(  Kp_z2vz,  Ki_z2vz,  Kd_z2vz );
        pid_yaw.getGains(Kp_Y2dY,  Ki_Y2dY,  Kd_Y2dY );
    }
};

#endif /* CONTROLLER_MIDLEVEL_SPEEDLOOP_H_ */
