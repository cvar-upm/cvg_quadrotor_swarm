// Jesus' to-do list:
// TODO: Cosas que me gustaría añadir al EKF,
//			- TODO: Hacer pruebas con diferentes constantes para las varianzas de medida, de ruido a la entrada, etc
//			- TODO: habilitar/permitir cambios de configuracion del EKF online

#ifndef DRONESIMULATOR_H
#define DRONESIMULATOR_H

//Other includes
#include <math.h>
#include "cvg_utils_library.h"      // from lib_cvgutils package
#include "Timer.h"
#include "drone_utils/drone_state_enum.h"
#include "drone_utils/drone_state_command_enum.h"
#include "quadrotor_model.h"



//DroneElements
#include "droneElements.h"

#define DRONESIMULATOR_TAKING_OFF_TIME     (3.0)
#define DRONESIMULATOR_TAKING_OFF_ALTITUDE (0.7)
#define DRONESIMULATOR_LANDING_SPEED       (0.5)
#define DRONESIMULATOR_
#define DRONESIMULATOR_

/////////////////////////////////////////
// Class DroneSimulator
//
//   Description
//
/////////////////////////////////////////
class DroneSimulator
{
    //Drone elements
public:
    Battery DroneBattery;
    IMU DroneImu;
    Termometer DroneTermometer;
    Magnetometer DroneMagnetometer;
    AltitudeSensor DroneAltitudeSensor;
    RotationAnglesSensor DroneRotationAnglesSensor;
    GroundSpeedSensor DroneGroundSpeedSensor;
    PressureSensor DronePressureSensor;

    Autopilot DroneAutopilot;
    //HLAutopilot DroneHLAutopilot;



    // Constructors, Destructor and related functions
public:
    DroneSimulator(int idDrone, const std::string &stackPath_in);
    ~DroneSimulator();


public:
    int run();

private: // Current drone position, and speed
    // drone_GMR_wrt_GFF,     g ~ global
    double current_xg, current_yg, current_zg, current_yawg, current_pitchg, current_rollg;
    double current_vxg, current_vyg, current_vzg;
    // drone_LMrT_wrt_LMrTFF, l ~ local
    double current_xl, current_yl, current_zl, current_yawl, current_pitchl, current_rolll;
    double current_vxl, current_vyl, current_vzl;
    // local_groundspeed
    double current_vxm, current_vym;

private:  // set position
    void setPosition_drone_GMR_wrt_GFF(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in);
    void setPosition_drone_LMrT_wrt_LMrTFF(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in);
    void setSpeed_drone_GMR_wrt_GFF( double vx_in, double vy_in, double vz_in);
    void setSpeed_droneLMrT_wrt_LMrTFF( double vx_in, double vy_in, double vz_in);
public:   // get position
    void getPosition_drone_GMR_wrt_GFF(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in);
    void getPosition_drone_LMrT_wrt_LMrTFF(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in);
    void getSpeed_drone_GMR_wrt_GFF( double &vx_in, double &vy_in, double &vz_in);
    void getSpeed_droneLMrT_wrt_LMrTFF( double &vx_in, double &vy_in, double &vz_in);
//public: //get noisy
    //void getPosition_drone_GMR_wrt_GFF_noisy(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in);


private: // drone state
    DroneState::ModeType current_drone_state;
public:
    inline DroneState::ModeType getCurrentDroneState() { return current_drone_state; }
private:
    Timer      timer;
    QuadrotorModel quadrotor_model;
//    bool precondition_check_variables;
    void preconditionCheck();
    bool processState();
    void stateTransitionCheck();
    void postProcessInputs();  // Modify drone pitch/roll/dyaw/dz commands of the drone
    void postProcessOutputs(); // Set sensor data

    void startUnknownState();
    void startLandedState();
    void startTakingOffState();
    void startHoveringState();
    void startFlyingState();
    void startLandingState();

public:  // drone state command interface
    void commandDrone( DroneStateCommand::StateCommand drone_state_command);
private:
    DroneStateCommand::StateCommand last_drone_state_command;
    bool flag_drone_state_command_received;
};

#endif /* DRONESIMULATOR_H */
