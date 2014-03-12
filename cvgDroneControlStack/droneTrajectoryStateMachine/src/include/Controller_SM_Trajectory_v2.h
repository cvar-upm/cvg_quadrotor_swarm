/*
 * Controller_SM_Trajectory_v2.h
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_SM_TRAJECTORY_V2_H_
#define CONTROLLER_SM_TRAJECTORY_V2_H_

#include "matrixLib.h"
#include "cvg_utils_library.h"
#include "xmlfilereader.h"
#include <math.h>
#include <iostream>
#include "control/simpletrajectorywaypoint.h"

class TrajectoryWaypoint
{
public:
    double x, y, z;
    bool achieved_checkpoint, achieved_true_checkpoint; // keep track of trajectory progress

    TrajectoryWaypoint(double x, double y, double z);
////////	TrajectoryWaypoint() { x = 0;  y = 0;  z = 0; }
    virtual ~TrajectoryWaypoint();
    void convert2Vector(CVG::Vector& lv);
public:
    /* ***** TrajectoryWaypoint - Trajectory planning ***** */
	// SM_stateNames::STRAIGHT mode parameters
    double L, s_end;
	// SM_stateNames::TURN mode parameters
    double R, vc, alim;
    CVG::Vector u0, r_ur;
    /* END: ***** TrajectoryWaypoint - Trajectory planning ***** */
};


class TrajectoryConfiguration
{
public:
    double chk_clearance_R;
    double chk_R, Rt_min, Rt_max;
    double vmax_xy, vmax_z;
    double vstall_turn, amax, num_chk_speed_plan, speed_tr;
    double straighmode_safetyzone_radius_m, turnmode_safetyzone_radius_m,
			turnmode_safetyzone_altitude_m, turnmode_safetyzone_negalpha_rad;

    TrajectoryConfiguration(int idDrone) {
        std::cout << "Constructor:  TrajectoryConfiguration" << std::endl;
        set2defaultValues(idDrone); }
    void set2defaultValues(int idDrone);


    // Default configuration values
public:
    static void setStackPath(const std::string &stackPath_in) { TrajectoryConfiguration::stackPath = stackPath_in; }
private:
    static std::string stackPath;
    static bool xml_file_is_read;
    static int  current_idDrone;
    static double default_chk_clearance_R;
    static double default_chk_R, default_Rt_min, default_Rt_max;
    static double default_vmax_xy, default_vmax_z;
    static double default_vstall_turn, default_amax, default_num_chk_speed_plan, default_speed_tr;
    static double default_straighmode_safetyzone_radius_m, default_turnmode_safetyzone_radius_m,
            default_turnmode_safetyzone_altitude_m, default_turnmode_safetyzone_negalpha_rad;
};


#include <deque>
typedef std::deque<TrajectoryWaypoint> trajectory_container;
//#include <vector>
//typedef std::vector<TrajectoryWaypoint> trajectory_container;

class DroneTrajectory
{
private:
	trajectory_container waypoints;
    int initial_checkpoint;
    bool isPeriodic;
    TrajectoryWaypoint error_waypoint;
//	SM_stateNames::stateNames initial_state; // I think that this is not useful.

public:
    DroneTrajectory(int idDrone);
    DroneTrajectory(int idDrone, double x_0, double y_0, double z_0);
    DroneTrajectory(int idDrone, double *routeX, double *routeY, double *routeZ, int num_waypoints);
    virtual ~DroneTrajectory();

    bool check();	// check that internal variables are consistent: initial_checkpoint, length >= 1.
    bool reset();	// prepare trajectory to be introduced into the state machine

    bool setInitialCheckpoint(int initial_checkpoint);
    inline void setPeriodic(bool isPeriodic) 					{ this->isPeriodic = isPeriodic; }
    inline int getInitialCheckpoint()					 		{ return initial_checkpoint; }
    int incrementCheckpoint(int checkpoint);
    int decrementCheckpoint(int checkpoint);
    bool routeFinished(int checkpoint);
    inline bool getIsPeriodic()		 							{ if ( getLength() < 2 ) isPeriodic = false;  return isPeriodic; }
	inline int getLength()		 									{ return waypoints.size(); }
    inline bool empty()											{ return waypoints.empty(); }
    inline bool checkpointValueIsValid(int chk)		{ return ( (chk >= 0) && (chk < getLength()) ); }
    inline void achievedCheckpoint(int chk)				{ if (checkpointValueIsValid(chk)) waypoints[chk].achieved_checkpoint = true; }
    inline void achievedTrueCheckpoint(int t_chk)		{ if (checkpointValueIsValid(t_chk)) waypoints[t_chk].achieved_true_checkpoint = true; }
//private:
    inline bool isAchievedCheckpoint(int chk)		{ if (checkpointValueIsValid(chk)) return waypoints[chk].achieved_checkpoint;  else  return false;}
    inline bool isAchievedTrueCheckpoint(int t_chk){ if (checkpointValueIsValid(t_chk)) return waypoints[t_chk].achieved_true_checkpoint;  else  return false;}
public:
    TrajectoryWaypoint& operator[](int i);
    // int operator==(const DroneTrajectory& right) const;  // I think this might not be necessary.

    void addWaypoint(double x, double y, double z);
    bool deleteLastWaypoint();
	void deleteFirstWaypoints(int until_checkpoint = 0);
//	inline void clear() 											{ waypoints_my_clear();  initial_checkpoint = 0;  isPeriodic = false;}
	void clear();

    /* ***** DroneTrajectory - Trajectory planning ***** */
	// planify trajectory from chk = checkpoint to chk = last_checkpoint "as if periodic"
public:
    TrajectoryConfiguration traj_config;
    bool planify_trajectory(TrajectoryConfiguration *configuration, int segment = 1, bool initialize_planning_variables = false);
    bool calculate_straight( int segment, CVG::Vector &r_p0, CVG::Vector &r_ur, CVG::Vector &r_ur2, double &rs_end, double &c_Rt);
    void calculate_turn(int turn, CVG::Vector &c_pinit, CVG::Vector &c_pend, CVG::Vector &c_pc, double &c_alim, CVG::Vector &c_u0);
    double obtainPlannedSpeed( int chk, double x, double y, double z, double v_current);
private:
	void initializeAuxiliarPlanningVariables();
	void destroyAuxiliarPlanningVariables();
    bool deleteWaypointInPlanning(int checkpoint);

	// planning variables
    CVG::Vector aux_vector;
    bool pr_isPeriodic;
    CVG::Vector r_p0, r_p1, r_p2;
    CVG::Vector r_ur, r_ur2;
    double rs_end;
    double c_Rt, c_vc;
    CVG::Vector c_u0;
    /* END: ***** DroneTrajectory - Trajectory planning ***** */
};

#endif /* CONTROLLER_SM_TRAJECTORY_V2_H_ */
