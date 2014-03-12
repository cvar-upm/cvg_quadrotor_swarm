/*
 * Controller_MidLevelCnt.h
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_MIDLEVELCNT_H_
#define CONTROLLER_MIDLEVELCNT_H_

#include "Controller_MidLevel_SpeedLoop.h"
#include "control/Controller_MidLevel_controlModes.h"
#include <math.h>

class Controller_MidLevelCnt
{
private:
	// Controller command inputs
    double xci, yci, yawci, zci;			// ci ~ command inputs
    double vxfi, vyfi, dyawfi, dzfi, pitchfi, rollfi;		// fi ~ feedforward inputs
	// Controller state estimate inputs (controller feedback)
    double xs, ys, vxs, vys, yaws, zs;
	// Intermediate variables between controller blocks
    double eps_x, eps_y;					// eps ~ epsilon to denote the control error in the commanded variable
    double vxc_int, vyc_int;				// d ~ desired value; int ~ internal variable
	// Next control layer commands, in this case these commands are to be sent directly to the drone using the pelican proxy
    double vxco_int, vyco_int, yawco_int, zco_int;	// co_int ~ command outputs internal (to speed controller)
    double pitchco, rollco, dyawco, dzco;			// co ~ command outputs (to pelican proxy)
    double vxy_max, vxy_at_max, vxy_ct_max;
    double dz_max,  dz_at_max,  dz_ct_max;

	// Internal PID controllers
	CVG_BlockDiagram::PID pid_x, pid_y;
    void saturation_2D(double x1, double x2, double *y1, double *y2, double max);

	// Internal speed controller
	Controller_MidLevel_SpeedLoop speedController;

    volatile bool started;
    Controller_MidLevel_controlMode::controlMode control_mode, init_control_mode;

public:
    Controller_MidLevelCnt(int idDrone, const std::string &stackPath_in);
	~Controller_MidLevelCnt();

	void reset();

    void setFeedback( double xs_t, double ys_t, double vxs_t, double vys_t, double yaws_t, double zs_t);
    void setReference( double xci_t, double yci_t, double yawci_t, double zci_t,
            double vxfi_t = 0.0, double vyfi_t = 0.0, double dyawfi_t = 0.0, double dzfi_t = 0.0,
            double pitchfi_t = 0.0, double rollfi_t = 0.0);
    void getOutput( double *pitchco, double *rollco, double *dyawco, double *dzco);

	void setControlMode(Controller_MidLevel_controlMode::controlMode mode);
	Controller_MidLevel_controlMode::controlMode getControlMode();

    void getIntermediateVars( double *vxco_int_out, double *vyco_int_out, double *yawco_int_out = NULL, double *zco_int_out = NULL,
                              double *vxfi_out = NULL, double *vyfi_out = NULL, double *dzfi_out = NULL, double *dyawfi_out = NULL);

    inline void getGains( double &Kp_vxm2P, double &Ki_vxm2P, double &Kd_vxm2P,
                          double &Kp_vym2R, double &Ki_vym2R, double &Kd_vym2R,
                          double &Kp_z2vz,  double &Ki_z2vz,  double &Kd_z2vz,
                          double &Kp_Y2dY,  double &Ki_Y2dY,  double &Kd_Y2dY,
                          double &Kp_x2vxm, double &Ki_x2vxm, double &Kd_x2vxm,
                          double &Kp_y2vym, double &Ki_y2vym, double &Kd_y2vym) {
        speedController.getGains( Kp_vxm2P, Ki_vxm2P, Kd_vxm2P,
                                  Kp_vym2R, Ki_vym2R, Kd_vym2R,
                                  Kp_z2vz,  Ki_z2vz,  Kd_z2vz ,
                                  Kp_Y2dY,  Ki_Y2dY,  Kd_Y2dY );
        pid_x.getGains( Kp_x2vxm, Ki_x2vxm, Kd_x2vxm);
        pid_y.getGains( Kp_y2vym, Ki_y2vym, Kd_y2vym);
    }

public:
    void getCurrentPositionReference(double *xci_out, double *yci_out, double *zci_out, double *yawci_out, double *pitchfi_out, double *rollfi_out);
    void getCurrentSpeedReference(double *dxfi_out, double *dyfi_out, double *dzfi_out, double *dyawfi_out);
};


#endif /* CONTROLLER_MIDLEVELCNT_H_ */
