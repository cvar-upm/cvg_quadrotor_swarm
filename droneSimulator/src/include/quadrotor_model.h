#ifndef _QUADROTOR_MODEL_H
#define _QUADROTOR_MODEL_H


#include "model.h"
#include "Timer.h"

// Integration timestep
// extern float timeIntegration;

// Saturation of model input in pitch, roll, dyaw and dz
//#define LIM_PITCH_REF      0.5
//#define LIM_ROLL_REF       0.5
//#define LIM_DYAW_REF       0.5
//#define LIM_DZ_REF         0.5

#define QUADROTOR_MODEL_STATE_LENGTH 12
#define QUADROTOR_MODEL_OBSERVATION_LENGTH 13
#define QUADROTOR_MODEL_INPUT_LENGTH 4

//First model
class QuadrotorModel : public ContinuousModel {
	// Valores por defecto durante 2012:
	//		Input_gains = diag([10*pi/180, 10*pi/180, 100*pi/180, -1]);
	//		Input_gains = diag([gain_pitch, gain_roll, gain_dyaw, gain_dz]);
private:
	double gain_pitch, gain_roll, gain_dyaw, gain_dz;
public:
	// Default constructor
    QuadrotorModel();
	//User process model
    virtual void processModel(CVG::Vector* Statek1, CVG::Vector* Statek, CVG::Vector* Inputs);
	//User Observation model
    virtual void observationModel(CVG::Vector* Output, CVG::Vector* State);
	//User Jacobian process model
    virtual void jacobiansProcessModel(CVG::Matrix* MatJacFx, CVG::Matrix* MatJacFu, CVG::Vector* State, CVG::Vector* Inputs);
	//User Jacobian process model
    virtual void jacobiansObservationModel(CVG::Matrix* MatJacHx, CVG::Vector* State);
    virtual ~QuadrotorModel() {}

	inline void setIntputGains(double gain_pitch, double gain_roll, double gain_dyaw, double gain_dz) {
		this->gain_pitch = gain_pitch;
		this->gain_roll  = gain_roll;
		this->gain_dyaw  = gain_dyaw;
		this->gain_dz    = gain_dz;
	}

private:
    Timer timer;
    CVG::Vector Statek;         // Current State
    CVG::Vector Statek1;        // State in k+1
    CVG::Vector Inputs;         // Commands: pitch/roll/dyaw/daltitude
    CVG::Vector Observation;    //

public:
    void setInputs( double pitch_in, double roll_in, double dyaw_in, double dz_in);
    void getObservation( double &x_out, double &y_out, double &z_out,
                         double &yaw_out, double &pitch_out, double &roll_out,
                         double &vx_out, double &vy_out, double &vz_out, double &dyaw_out,
                         double &vxm_out, double &vym_out);
    void runSimulation();
    void start( double x_in, double y_in, double z_in,
                double yaw_in, double pitch_in, double roll_in,
                double vx_in, double vy_in, double vz_in, double dyaw_in);
    inline void stop() {}
};

#endif // _QUADROTOR_MODEL_H
