/*
 * Controller_StateMachine_v2.h
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_STATEMACHINE_V2_H_
#define CONTROLLER_STATEMACHINE_V2_H_

#include <math.h>
#include "matrixLib.h"
#include "cvg_utils_library.h"
#include "Timer.h"
#include "Controller_SM_stateNames.h"
#include "Controller_SM_Trajectory_v2.h"
#include "control/FilteredDerivative.h"
#include "xmlfilereader.h"

class Controller_StateMachine_v2
{
private:

	// ***************************** General State Machine Data *****************************
	// SM_Inputs
    double xei, yei, zei, vxei, vyei, vzei, yawei, dyawei; 			// ei  ~ estimation input

	// SM_Outputs
	// Note that all this data is passed through the trajectory controller
	//       (even in cases where it, in turn, passes directly the data to the parrot)
    double vxfo, vyfo;							// fo ~ feedforward output (to Speed controller)
	CVG_BlockDiagram::FilteredDerivative derivBlock_vxfo, derivBlock_vyfo;
    double dvxfo, dvyfo;	// derivatives of speed references
    double pitchfo, rollfo, vzfo, dyawfo;		// fo ~ feedforward output (directly to parrot)
    double xrefo, yrefo, zrefo, yawfo;			// refo ~ (position) reference outputs (to position controller)

	// SM trajectory related information
    DroneTrajectory trajectory; // trajectory.traj_config has the trajectory configuration parameters
    int pr_checkpoint, checkpoint, true_checkpoint;

	// SM state
	bool started;
    SM_stateNames::stateNames current_state;
	bool justChangedState;			// Stores whether a state change has just ocurred

	int debug_counter;

    // Related to loading the configuration from an XML instead of from #defines.
    SM_stateNames::stateNames init_state;
    bool   tiltfo_is_activated; // SM_TRAJECTORYMODE_ACTIVATE_TILTFO
//    double tiltfo_tr;           // SM_TRAJECTORYMODE_TILTFO_TR
    double tiltfo_rad2tiltref;  // SM_TRAJECTORYMODE_TILTFO_RAD2TILTREF
    int current_idDrone;
    // SM_STATEMACHINE_DEBUG
    // SM_STATEMACHINE_DEBUG_OLD


	// ***************************** General State Machine Functions ************************************
public:
    Controller_StateMachine_v2( int idDrone, const std::string &stackPath_in); // throw(std::runtime_error);
	virtual ~Controller_StateMachine_v2();
    bool reset();
	inline int getCheckpoint() 			{ return checkpoint; }
	inline int getTrueCheckpoint() 		{ return true_checkpoint; }
	const char *getCurrentStateName(SM_stateNames::stateNames state_in);
	inline SM_stateNames::stateNames getCurrentState() 	{ return current_state; }
    void setInputs( double xei_i, double yei_i, double zei_i,
        double vxei_i, double vyei_i, double vzei_i, double yawei_i, double dyawei_i);
    void getOutput( double &xrefo_o, double &yrefo_o, double &zrefo_o,
            double &vxfo_o, double &vyfo_o, double &vzfo_o,
            double &yawfo_o, double &dyawfo_o,
            double &pitchfo_o, double &rollfo_o);

    bool setTrajectory( DroneTrajectory &trajectory, TrajectoryConfiguration traj_config);
    inline DroneTrajectory getTrajectory() { return trajectory; }
    bool activateTrajectoryControl();
	inline void activatePositionControl() 					{ current_state = SM_stateNames::POSITION_CONTROL; }
	inline void activateSpeedControl() 						{ current_state = SM_stateNames::SPEED_CONTROL; }
private:
    void initSM( double x_act, double y_act, double z_act, double yaw_act);

private:
	void process();
//	void processMandatoryCode();
	void updateTrueCheckpointValue();
	void processPositionControl();
	void processSpeedControl();

	// ***************************** Straight line state: data and functions ****************************
	void processStraight();
	void isStraightFinished();
	SM_stateNames::stateNames r_nextState;
    CVG::Vector r_p0, r_ur, r_ur2;		// initial point and direction of straight line (recta)
    double rs_end;		// distance from p0 that indicates the end of the straight line (recta)
    double rs_act;		// actual value of s;


	// ***************************** Perform turn state: data and functions *****************************
    CVG::Vector 		c_pinit, c_pc, c_pend;	// Initial point of turn
    double 	c_alim; 		// limit yaw angle from pinit to "pend" that indicates the end of the turn
    CVG::Vector		c_u0;			// +1: clockwise turn, -1: counter_clockwise turn
    double  c_vc, c_Rt; 	// Speed reference during turn and radius of turn circle
	bool 		c_changeState;
	SM_stateNames::stateNames  c_nextState;
	void processTurn();			// "during:"
	void isTurnFinished();

	// ***************************** Hover to checkpoint state: data and functions **********************
	// This state only uses the information of "checkpoint"
	void processHover();		// "during:"
	void isHoverFinished();		// transition
    CVG::Vector h_checkpoint; // dim3
    bool h_stay_in_last_checkpoint;

public:
    void getCurrentTrajectoryReference( std::vector<SimpleTrajectoryWaypoint> *trajectory_waypoints_out, int *initial_checkpoint_out, bool *is_periodic_out);
};

#endif /* CONTROLLER_STATEMACHINE_V2_H_ */
