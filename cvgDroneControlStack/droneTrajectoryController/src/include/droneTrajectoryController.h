#ifndef DRONETRAJECTORYCONTROLLER_H
#define DRONETRAJECTORYCONTROLLER_H

#include <math.h>
#include "Controller_MidLevelCnt.h"
#include "Controller_StateMachine_v2.h"
#include "cvg_utils_library.h"
#include "control/LowPassFilter.h"
#include "referenceFrames.h"
#include "control/FilteredDerivative.h"
#include "debugstringstacker.h"

class DroneTrajectoryController
{
public:
    DroneTrajectoryController(int idDrone, const std::string &stackPath_in);
    ~DroneTrajectoryController();
    void resetValues();
    int current_idDrone;

    //
public:
    inline Controller_MidLevel_controlMode::controlMode getControlMode() { return control_mode; }
    inline Controller_MidLevel_controlMode::controlMode getInitControlMode() { return init_control_mode; }
    void setPositionControl();
    void setSpeedControl();
    void setTrajectory_droneLMrT_wrt_LMrTFF(DroneTrajectory &trajectory, TrajectoryConfiguration &traj_config);
    bool setControlMode(Controller_MidLevel_controlMode::controlMode mode);

    // Input of sensor feedback
private:
    CVG_BlockDiagram::FilteredDerivative filter_x2vx;
    CVG_BlockDiagram::FilteredDerivative filter_y2vy;
    CVG_BlockDiagram::FilteredDerivative filter_yaw2dyaw;
    CVG_BlockDiagram::FilteredDerivative filter_z2dz;
public:
    void setFeedback_drone_pose_GMR_wrt_GFF( double xs_t, double ys_t, double zs_t, double yaws_t, double pitchs_t = 0.0, double rolls_t = 0.0);
    void setFeedback_drone_speeds_GMR_wrt_GFF( double vxs_t, double vys_t, double vzs_t, double dyaws_t, double dpitchs_t = 0.0, double drolls_t = 0.0);
    void setFeedback_drone_pose_LMrT_wrt_LMrTFF( double xs_t, double ys_t, double zs_t, double yaws_t, double pitchs_t = 0.0, double rolls_t = 0.0);
    void setFeedback_drone_speeds_LMrT_wrt_LMrTFF( double vxs_t, double vys_t, double vzs_t, double dyaws_t, double dpitchs_t = 0.0, double drolls_t = 0.0);

    // Input of controller references
public:
    void setAllReferences_droneLMrT_wrt_LMrTFF( double xci_in, double yci_in, double yawci_in, double zci_in,
            double vxfi_t = 0.0, double vyfi_t = 0.0, double dyawfi_t = 0.0, double dzfi_t = 0.0,
            double pitchfi_t = 0.0, double rollfi_t = 0.0);
    // Pose references, currently unused
    void setPositionRefs_drone_GMR_wrt_GFF(double xci_in, double yci_in, double zci_in, double yawci_in, double pitchfi_in = 0.0, double rollfi_in = 0.0);
    void setPositionRefs_drone_LMrT_wrt_LMrTFF(double xci_in, double yci_in, double zci_in, double yawci_in, double pitchfi_in = 0.0, double rollfi_in = 0.0);
    // speed references, currently unused
    void setSpeedRefs_drone_GMR_wrt_GFF( double vxfi_in, double vyfi_in, double dzfi_in);
    void setSpeedRefs_droneLMrT_wrt_LMrTFF( double vxfi_in, double vyfi_in, double dzfi_in);
    void setDYawFIReference_drone_GMR_wrt_GFF( double dyawfi_in);
    void setDYawFIReference_droneLMrT_wrt_LMrTFF( double dyawfi_in);
    // Position references
    void setPositionRefs_drone_GMR_wrt_GFF(double xci_in, double yci_in, double zci_in);
    void setPositionRefs_drone_LMrT_wrt_LMrTFF(double xci_in, double yci_in, double zci_in);
    // yaw reference
    void setYawRef_drone_GMR_wrt_GFF( double yawci_in_rad);
    void setYawRef_drone_LMrT_wrt_LMrTFF( double yawci_in_rad);
    // Horizontal speed references
    void setHorizontalSpeedRefs_drone_GMR_wrt_GFF(double vxfi_in, double vyfi_in);
    void setHorizontalSpeedRefs_drone_LMrT_wrt_LMrTFF(double vxfi_in, double vyfi_in);

    // Output of droneCommands
public:
    void getOutput( double *p_pitchco, double *p_rollco, double *p_dyawco, double *p_dzco);

    // ***** Internal state of state_machine *****
public:
    SM_stateNames::stateNames getCurrentState();
    int getCheckpoint();
    int getTrueCheckpoint();
    //  END: ***** Scheduler part of MyDrone *****

    // ***** Internal variables in the controller block diagram *****
private:
    // Controller command inputs
    // ci ~ command inputs
    double xci, yci, yawci, zci;
    // fi ~ feedforward inputs
    double pitchfi, rollfi; //Position
    double vxfi, vyfi, dyawfi, dzfi; //=Speed ci

    // Controller state estimate inputs (controller feedback)
    //Position
    double xs, ys, zs, yaws;
    //Speeds
    double vxs, vys, vzs, dyaws;

    // Next control layer commands, in this case these commands are to be sent directly to the drone using the pelican proxy
    double vxco_int, vyco_int, yawco_int, zco_int;	// co_int ~ command outputs internal (to speed controller)
    double pitchco_hf, rollco_hf, dyawco_hf, dzco_hf; // hf ~ "high frequency", before (low pass) filtering

    // co ~ command outputs (to pelican proxy)
    double pitchco, rollco, dyawco, dzco;

    //Others
    Controller_MidLevel_controlMode::controlMode control_mode, init_control_mode;// Active and initial control modes
    Controller_StateMachine_v2 		state_machine;	// enables switching speed, position and trajectory control
    Controller_MidLevelCnt			midlevel_controller;
    CVG_BlockDiagram::LowPassFilter pitch_lowpassfilter, roll_lowpassfilter, dyaw_lowpassfilter, dz_lowpassfilter;
    // ***** END: Internal variables in the controller block diagram *****

    // Transformation between different reference frames, see documentation for more information
public:
    void referenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF(
            double x1, double y1, double z1, double yaw1, double pitch1, double roll1,
            double &x2, double &y2, double &z2, double &yaw2, double &pitch2, double &roll2);
    void referenceChange_from1_drone_LMrT_wrt_LMrTFF_to2_drone_GMR_wrt_GFF(
            double x1, double y1, double z1, double yaw1, double pitch1, double roll1,
            double &x2, double &y2, double &z2, double &yaw2, double &pitch2, double &roll2);
    void speedReferenceChange_from1_drone_GMR_wrt_GFF_to2_drone_LMrT_wrt_LMrTFF(
            double vx1, double vy1, double vz1, double dyaw1, double dpitch1, double droll1,
            double &vx2, double &vy2, double &vz2, double &dyaw2, double &dpitch2, double &droll2);
    void speedReferenceChange_from1_drone_LMrT_wrt_LMrTFF_to2_drone_GMR_wrt_GFF(
            double vx1, double vy1, double vz1, double dyaw1, double dpitch1, double droll1,
            double &vx2, double &vy2, double &vz2, double &dyaw2, double &dpitch2, double &droll2);
private:
    cv::Mat matHomog_drone_LMrT_wrt_drone_GMR;      // constant, see reference frames' definition (see pdf)
    cv::Mat matHomog_drone_GMR_wrt_drone_LMrT;      // constant, see reference frames' definition (see pdf)
    cv::Mat matHomog_drone_GMR_wrt_GFF;             // given by arucoSlam
    cv::Mat matHomog_drone_LMrT_wrt_LMrTFF;         // required by droneController

    // DroneLogger - Logging functions
private:
    DebugStringStacker debug_string_stacker;
public:
    void logControllerState(bool is_started);   // called from ROSModule after getOutput(...) call
private:
    void logControllerGains();                  // called internally
    std::string controlMode2String();
public:
    void getCurrentPositionReference( float *x_out, float *y_out, float *z_out, float *yaw_out, float *pitch_out, float *roll_out);
    void getCurrentSpeedReference( float *dx_out, float *dy_out, float *dz_out, float *dyaw_out);
    void getCurrentTrajectoryReference( std::vector<SimpleTrajectoryWaypoint> *trajectory_waypoints_out, int *initial_checkpoint_out, bool *is_periodic_out);
};

#endif // DRONETRAJECTORYCONTROLLER_H
