#pragma once

#if !defined(_KALMAN_LOC_INCLUDED_)
#define _KALMAN_LOC_INCLUDED_

#define  NOMINMAX 

#define WANT_STREAM                  // include.h will get stream fns
#define WANT_MATH      
#include "newmatap.h"                // need matrix applications
#include "newmatio.h"                // need matrix output routines
#ifdef use_namespace
using namespace NEWMAT;              // access NEWMAT namespace
#endif  

#include "tools.h"
#include "observation3d.h"
#include "landmark3d.h"

#include <fstream>
#include <string>

class Kalman_Loc 
{
public:
	//Kalman_Loc(int num_lm_wind=8,double x_0=0, double y_0=0, double z_0=0, double roll_0=0, double pitch_0=0, double yaw_0=0);
	Kalman_Loc(void);
	~Kalman_Loc(void);

	Matrix KalmanPredict(float inc_odom_x, float inc_odom_y, float inc_odom_z, float inc_odom_roll, float inc_odom_pitch, float inc_odom_yaw);
	Matrix KalmanUpdate(const std::vector<Observation3D>& v);//se le pasan obs y corrige pos y cov Kalman
	
    void initialization(std::string paramsFileName);
    void mapInitialization(std::string paramsFileName);
	
	Matrix computeInnovation(Observation3D obs, Landmark3D map_landmark);
	Matrix computeInnovation2(Observation3D obs, Landmark3D map_landmark);
	
	Matrix computeHx(Observation3D obs);
	Matrix computeHx2(Observation3D obs);
	
	Matrix computeHz(Observation3D obs);
	Matrix computeHz2(Observation3D obs);
	
	void obtainObstacles();
	
	std::vector<Landmark3D> map;
	Matrix kalman_pos;
	Matrix cov_kalman;
	
	//visualization purposes?
	//Matrix odom_pos;
	//vector<Matrix> odom_path;
	
	//Matrix x_tot;
	//Matrix P_tot;

	Matrix R;
	
	//odom uncertainty
	float sigma_odom_x;
	float sigma_odom_y;
	float sigma_odom_z;
	float sigma_odom_roll;
	float sigma_odom_pitch;
	float sigma_odom_yaw;
	
    ColumnVector prev_odom_pose;
	
	// pose
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;


	float off_x;
	float off_y;
	
	int known_obs;
	
	double wall_width;
	double wall_length;
	double known_pole_rad;
	
	int num_landmarks_wind;
	
	std::ofstream debug_output; //debug;

};
	

#endif
