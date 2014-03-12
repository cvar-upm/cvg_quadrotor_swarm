#include "Kalman_Loc2.h"
#include <math.h>
#include <assert.h>

#include "pugixml.hpp"

/*Kalman_Loc::Kalman_Loc(int num_lm_wind, double x_0, double y_0, double z_0, double roll_0, double pitch_0, double yaw_0)
{	
	kalman_pos.ReSize(6,1);
	kalman_pos=0.0;
	//odom_pos.ReSize(6);
	//odom_pos=0.0;
	
	prev_odom_pose.ReSize(6,1);
	prev_odom_pose=0.0;
	
	kalman_pos(1,1) = x_0; kalman_pos(2,1) = y_0; kalman_pos(3,1) = z_0;
	kalman_pos(4,1) = roll_0; kalman_pos(5,1) = pitch_0; kalman_pos(6,1) = yaw_0;
	
	cov_kalman.ReSize(6,6);
	cov_kalman=0.0;

	//odom covariance
	//float k = 0.1;
	//sigma_odom_x = 0.3*k;
	//sigma_odom_y = 0.3*k;
	//sigma_odom_z = 0.1*k;
	//sigma_odom_roll = M_PI/10*k;
	//sigma_odom_pitch = M_PI/10*k;
	//sigma_odom_yaw = M_PI/6*k;
	
	sigma_odom_x = 0.5;
	sigma_odom_y = 0;
	sigma_odom_z = 0;
	sigma_odom_roll = 0*DEG2RAD;
	sigma_odom_pitch = 0;
	sigma_odom_yaw = 0;
	

	//obs covariance
	R.ReSize(6,6);
	R = 0.0;
	R(1,1) = pow(0.00001,2);
	R(2,2) = pow(0.00001,2);
	R(3,3) = pow(0.00001,2);
	R(4,4) = pow(0.00001*DEG2RAD,2);
	R(5,5) = pow(0.00001*DEG2RAD,2);
	R(6,6) = pow(0.00001*DEG2RAD,2);
	
	//FIXME, check!!
	wall_width = 0.2; //not given
	wall_length = 8.0;
	known_pole_rad = 0.20; //not given
	
	num_landmarks_wind = num_lm_wind;
	mapInitialization();
	

}*/

Kalman_Loc::Kalman_Loc(void)
{

}

void Kalman_Loc::initialization(std::string paramsFileName)
{

	debug_output.open("localization_debugging.txt",std::ios::out);
	debug_output << "Localization debugging " << endl;

	cout<<paramsFileName<<endl;
	kalman_pos.ReSize(6,1);
	kalman_pos=0.0;
	
	cov_kalman.ReSize(6,6);
	cov_kalman=0.0;


    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(paramsFileName);
    pugi::xml_parse_result result = doc.load(nameFile);

   if (!result) //FIXME, pass as argument
   {
   	cout << "Xml file missing " << endl;
   	assert(0);
    
   }
   
   /////////////////////////////////////////////////////////////////////////////////////
    
   // initial pose 
   kalman_pos(1,1) = atof(doc.child("main").child("params").child_value("x_0"));
   kalman_pos(2,1) = atof(doc.child("main").child("params").child_value("y_0"));
   kalman_pos(3,1) = atof(doc.child("main").child("params").child_value("z_0"));
   kalman_pos(4,1) = atof(doc.child("main").child("params").child_value("yaw_0"))*DEG2RAD;
   kalman_pos(5,1) = atof(doc.child("main").child("params").child_value("pitch_0"))*DEG2RAD;
   kalman_pos(6,1) = atof(doc.child("main").child("params").child_value("roll_0"))*DEG2RAD;

	debug_output << "Initial pose from file " << endl;
	debug_output << "kalman_pos(1,1) " << kalman_pos(1,1) << endl;
	debug_output << "kalman_pos(2,1) " << kalman_pos(2,1) << endl;
	debug_output << "kalman_pos(3,1) " << kalman_pos(3,1) << endl;
	debug_output << "kalman_pos(4,1) " << kalman_pos(4,1) << endl;
	debug_output << "kalman_pos(5,1) " << kalman_pos(5,1) << endl;
	debug_output << "kalman_pos(6,1) " << kalman_pos(6,1) << endl;

    cout << "Initial pose from file " << endl;
    cout << "kalman_pos(1,1) " << kalman_pos(1,1) << endl;
    cout << "kalman_pos(2,1) " << kalman_pos(2,1) << endl;
    cout << "kalman_pos(3,1) " << kalman_pos(3,1) << endl;
    cout << "kalman_pos(4,1) " << kalman_pos(4,1) << endl;
    cout << "kalman_pos(5,1) " << kalman_pos(5,1) << endl;
    cout << "kalman_pos(6,1) " << kalman_pos(6,1) << endl;
	
	prev_odom_pose.ReSize(6,1);
	//prev_odom_pose=0.0;
	prev_odom_pose=kalman_pos; //gets updated from the pose estimator node!!
	
   /*prev_odom_pose(1) =  2.07; //FIXME!! x
   prev_odom_pose(2) = 0.07; // y
   prev_odom_pose(4) = 90*DEG2RAD; // yaw, my roll */
	
	
	/////////////////////////////////////////////////////////////////////////////////////
	
	//covariance values
	
	//odom covariance
	sigma_odom_x = atof(doc.child("main").child("params").child_value("sigma_odom_x"))*DEG2RAD;
	sigma_odom_y = atof(doc.child("main").child("params").child_value("sigma_odom_y"))*DEG2RAD;
	sigma_odom_z = atof(doc.child("main").child("params").child_value("sigma_odom_z"))*DEG2RAD;
	sigma_odom_roll = atof(doc.child("main").child("params").child_value("sigma_odom_roll"))*DEG2RAD;
	sigma_odom_pitch = atof(doc.child("main").child("params").child_value("sigma_odom_pitch"))*DEG2RAD;
	sigma_odom_yaw = atof(doc.child("main").child("params").child_value("sigma_odom_yaw"))*DEG2RAD;
	
	//obs covariance
	R.ReSize(6,6);
	R = 0.0;
	double sigma_obs_x = atof(doc.child("main").child("params").child_value("sigma_obs_x"));
	R(1,1) = pow(sigma_obs_x,2);
	
	double sigma_obs_y = atof(doc.child("main").child("params").child_value("sigma_obs_y"));
	R(2,2) = pow(sigma_obs_y,2);
	
	double sigma_obs_z = atof(doc.child("main").child("params").child_value("sigma_obs_z"));
	R(3,3) = pow(sigma_obs_z,2);
	
	double sigma_obs_roll = atof(doc.child("main").child("params").child_value("sigma_obs_roll"));
	R(4,4) = pow(sigma_obs_roll*DEG2RAD,2);
	
	double sigma_obs_pitch = atof(doc.child("main").child("params").child_value("sigma_obs_pitch"));
	R(5,5) = pow(sigma_obs_pitch*DEG2RAD,2);
	
	double sigma_obs_yaw = atof(doc.child("main").child("params").child_value("sigma_obs_yaw"));
	R(6,6) = pow(sigma_obs_yaw*DEG2RAD,2);
	
	/////////////////////////////////////////////////////////////////////////////////////
	
	// given dimensions
	
	wall_width = atof(doc.child("main").child("params").child_value("wall_width"));
	wall_length = atof(doc.child("main").child("params").child_value("wall_length"));
	known_pole_rad = atof(doc.child("main").child("params").child_value("known_pole_rad"));
	
	
	/////////////////////////////////////////////////////////////////////////////////////
    mapInitialization(paramsFileName);

}



Kalman_Loc::~Kalman_Loc(void)
{
}

void Kalman_Loc::mapInitialization(string paramsFileName)
{

	//FIXME, give real values if the windows are known

	/*
	
	//////////////////////////////////////////////////////////////
	Option 1
	// large window
	Landmark3D large_window_1(1,x1,0,2,-M_PI/2,0,0); //FIXME
	large_window_1.is_known = true;
	map.push_back(large_window_1);
	
	Landmark3D large_window_2(2,x1,0,1,-M_PI/2,0,0); //FIXME
	large_window_2.is_known = true;
	map.push_back(large_window_2);
	
	Landmark3D large_window_3(3,x2,0,1,-M_PI/2,0,0); //FIXME
	large_window_3.is_known = true;
	map.push_back(large_window_3);
	num_landmarks_wind ++;
	
	Landmark3D large_window_4(4,x2,0,2,-M_PI/2,0,0); //FIXME
	large_window_4.is_known = true;
	map.push_back(large_window_4);
	
	//////////////////////////////////////////////////////////////
	// small window
	Landmark3D small_window_1(5,x3,0,2,-M_PI/2,0,0); //FIXME
	small_window_1.is_known = true;
	map.push_back(small_window_1);
	
	Landmark3D small_window_2(6,x3,0,1,-M_PI/2,0,0); //FIXME 
	small_window_2.is_known = true;
	map.push_back(small_window_2);
	
	Landmark3D small_window_3(7,x4,0,1,-M_PI/2,0,0); //FIXME 
	small_window_3.is_known = true;
	map.push_back(small_window_3);
	
	Landmark3D small_window_4(8,x4,0,2,-M_PI/2,0,0); //FIXME  
	small_window_4.is_known = true;
	map.push_back(small_window_4);
	
	/*
	//////////////////////////////////////////////////////////////
	Option 2
	// large window
	Landmark3D large_window_1(1,x1,0,1.5,-M_PI/2,0,0); //FIXME
	large_window_1.is_known = true;
	map.push_back(large_window_1);
	
	Landmark3D large_window_2(2,x2,0,1,-M_PI/2,0,0); //FIXME
	large_window_2.is_known = true;
	map.push_back(large_window_2);
	
	Landmark3D large_window_3(3,x3,0,1,-M_PI/2,0,0); //FIXME
	large_window_3.is_known = true;
	map.push_back(large_window_3);
	
	Landmark3D large_window_4(4,x4,0,1.5,-M_PI/2,0,0); //FIXME
	large_window_4.is_known = true;
	map.push_back(large_window_4);
	num_landmarks_wind ++;
	
	//////////////////////////////////////////////////////////////
	// small window
	Landmark3D small_window_1(5,x5,0,1.5,-M_PI/2,0,0); //FIXME
	small_window_1.is_known = true;
	map.push_back(small_window_1);
	
	Landmark3D small_window_2(6,x6,0,1,-M_PI/2,0,0); //FIXME 
	small_window_2.is_known = true;
	map.push_back(small_window_2);
	
	Landmark3D small_window_3(7,x7,0,1,-M_PI/2,0,0); //FIXME 
	small_window_3.is_known = true;
	map.push_back(small_window_3);
	
	Landmark3D small_window_4(8,x8,0,1.5,-M_PI/2,0,0); //FIXME  
	small_window_4.is_known = true;
	map.push_back(small_window_4); */



    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(paramsFileName);
    pugi::xml_parse_result result = doc.load(nameFile);

   if (!result) //FIXME, pass as argument
   {
   	cout << "Xml file missing " << endl;
   	assert(0);
    
   }
   
   /////////////////////////////////////////////////////////////////////////////////////
    
   // known Arucos 

	pugi::xml_node Arucos = doc.child("main").child("params").child("Arucos");
	for (pugi::xml_node aruco = Arucos.child("Aruco"); aruco; aruco = aruco.next_sibling("Aruco"))
	{
		 
		int id = atoi(aruco.attribute("id").value());
		double x = atof(aruco.attribute("x").value());
		double y = atof(aruco.attribute("y").value());
		double z = atof(aruco.attribute("z").value());
		double roll = atof(aruco.attribute("yaw").value())*DEG2RAD;
		double pitch = atof(aruco.attribute("pitch").value())*DEG2RAD;
		double yaw = atof(aruco.attribute("roll").value())*DEG2RAD;
		
		debug_output << " known aruco from file " << id << " " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << endl;

		Landmark3D  known_pole(id,x,y,z,roll,pitch,yaw);
		known_pole.is_known = true;
		map.push_back( known_pole);
	}

	
	//FIXME, read from xml file
	//////////////////////////////////////////////////////////////
	
	
	//////////////////////////////////////////////////////////////
	//first known pole
/*	Landmark3D  known_pole1_1(9,1.20,3.20-known_pole_rad,2,-M_PI/2,0,0);
	 known_pole1_1.is_known = true;
	map.push_back( known_pole1_1);
	
	Landmark3D  known_pole1_2(10,1.20+known_pole_rad,3.20,2,0,0,0);
	known_pole1_2.is_known = true;
	map.push_back(known_pole1_2);
	
	Landmark3D known_pole1_3(11,1.20,3.20+known_pole_rad,2,M_PI/2,0,0);
	known_pole1_3.is_known = true;
	map.push_back(known_pole1_3);
	
	Landmark3D known_pole1_4(12,1.20-known_pole_rad,3.20,2,M_PI,0,0);
	known_pole1_4.is_known = true;
	map.push_back(known_pole1_4);
	
	//////////////////////////////////////////////////////////////
	//second known pole
	Landmark3D  known_pole2_1(13,6.8,3.20-known_pole_rad,2,-M_PI/2,0,0);
	known_pole2_1.is_known = true;
	map.push_back(known_pole2_1);
	
	Landmark3D  known_pole2_2(14,6.8+known_pole_rad,3.20,2,0,0,0);
	known_pole2_2.is_known = true;
	map.push_back(known_pole2_2);
	
	Landmark3D known_pole2_3(15,6.8,3.20+known_pole_rad,2,M_PI/2,0,0);
	known_pole2_3.is_known = true;
	map.push_back(known_pole2_3);
	
	Landmark3D known_pole2_4(16,6.8-known_pole_rad,3.20,2,M_PI,0,0);
	known_pole2_4.is_known = true;
	map.push_back(known_pole2_4);
	
	//////////////////////////////////////////////////////////////
	//third known pole
	Landmark3D  known_pole3_1(17,6.8,8.8-known_pole_rad,2,-M_PI/2,0,0);
	 known_pole3_1.is_known = true;
	map.push_back( known_pole3_1);
	
	Landmark3D  known_pole3_2(18,6.8+known_pole_rad,8.8,2,0,0,0);
	known_pole3_2.is_known = true;
	map.push_back(known_pole3_2);
	
	Landmark3D known_pole3_3(19,6.8,8.8+known_pole_rad,2,M_PI/2,0,0);
	known_pole3_3.is_known = true;
	map.push_back(known_pole3_3);
	
	Landmark3D known_pole3_4(20,6.8-known_pole_rad,8.8,2,M_PI,0,0);
	known_pole3_4.is_known = true;
	map.push_back(known_pole3_4);
	
	//////////////////////////////////////////////////////////////
	//fourth known pole
	Landmark3D  known_pole4_1(21,1.2,8.8-known_pole_rad,2,-M_PI/2,0,0);
	known_pole4_1.is_known = true;
	map.push_back( known_pole4_1);
	
	Landmark3D  known_pole5_2(22,1.2+known_pole_rad,8.8,2,0,0,0);
	known_pole5_2.is_known = true;
	map.push_back(known_pole5_2);
	
	Landmark3D known_pole5_3(23,1.2,8.8+known_pole_rad,2,M_PI/2,0,0);
	known_pole5_3.is_known = true;
	map.push_back(known_pole5_3);
	
	Landmark3D known_pole6_4(24,1.2-known_pole_rad,8.8,2,M_PI,0,0);
	known_pole6_4.is_known = true;
	map.push_back(known_pole6_4);*/
	//////////////////////////////////////////////////////////////

	
	// preliminar tests
	/*Landmark3D l1(1,4,2,2,0,0,0);
	map.push_back(l1);
	
	Landmark3D l2(2,8,4,2,-M_PI/2,0,0);
	map.push_back(l2);
	
	Landmark3D l3(3,4,6,2,0,0,0);
	map.push_back(l3);
	
	Landmark3D l4(4,8,6,2,0,0,M_PI/2);
	map.push_back(l3);
	
	known_obs = map.size();*/
	

}

Matrix Kalman_Loc::KalmanPredict(float odom_pose_x, float odom_pose_y, float odom_pose_z, float odom_pose_roll, float odom_pose_pitch, float odom_pose_yaw)//Next kalman position
{
	ColumnVector current_odom_pose(6);
	current_odom_pose << odom_pose_x << odom_pose_y << odom_pose_z << odom_pose_roll << odom_pose_pitch << odom_pose_yaw;

    debug_output << "prev_odom_pose " << prev_odom_pose(1) << " " <<prev_odom_pose(2) <<  " " <<prev_odom_pose(3) <<  " " <<prev_odom_pose(4)*RAD2DEG  <<  " " << prev_odom_pose(5)*RAD2DEG   <<  " " <<prev_odom_pose(6)*RAD2DEG  << endl;
    debug_output << "current_odom_pose " << current_odom_pose(1) << " " << current_odom_pose(2) <<  " " <<current_odom_pose(3) <<  " " <<current_odom_pose(4)*RAD2DEG  <<  " " << current_odom_pose(5)*RAD2DEG   <<  " " <<current_odom_pose(6)*RAD2DEG  << endl;

	ColumnVector inc_odom(6);
	inc_odom = Comp(Inv(prev_odom_pose), current_odom_pose);

    debug_output << "odom inc " << inc_odom(1) << " " << inc_odom(2) <<  " " <<inc_odom(3) <<  " " <<inc_odom(4)*RAD2DEG  <<  " " << inc_odom(5)*RAD2DEG   <<  " " <<inc_odom(6)*RAD2DEG  << endl;

	kalman_pos << Comp(kalman_pos, inc_odom);		//the prediction
	
    debug_output << "prediction " << kalman_pos(1,1) << " " << kalman_pos(2,1) <<  " " <<kalman_pos(3,1) <<  " " <<kalman_pos(4,1)*RAD2DEG  <<  " " << kalman_pos(5,1)*RAD2DEG   <<  " " << kalman_pos(6,1)*RAD2DEG  << endl;

	// odom pose
	//odom_pos << Comp(odom_pos, inc_odom);
	//odom_path.push_back(odom_pos);

    prev_odom_pose = current_odom_pose;

	Matrix Q(6,6);
	IdentityMatrix eye(6);
	Q << eye;
	Q(1,1)=pow(sigma_odom_x,2);
	Q(2,2)=pow(sigma_odom_y,2);
	Q(3,3)=pow(sigma_odom_z,2);
	Q(4,4)=pow(sigma_odom_roll,2);
	Q(5,5)=pow(sigma_odom_pitch,2);
	Q(6,6)=pow(sigma_odom_yaw,2);
	
	Matrix Fx(6,6);
	Matrix Fu(6,6);

	Fx << J1_n(kalman_pos,inc_odom);
	Fu << J2_n(kalman_pos,inc_odom);

	cov_kalman=Fx*cov_kalman*Fx.t()+Fu*Q*Fu.t();
	
	x = kalman_pos(1,1);
	y = kalman_pos(2,1);
	z = kalman_pos(3,1);
	roll = kalman_pos(4,1);
	pitch = kalman_pos(5,1);
	yaw = kalman_pos(6,1);

	return kalman_pos;
}
	
Matrix Kalman_Loc::KalmanUpdate(const std::vector<Observation3D>& v)//
{	

	int obs_size = v.size();
    int tam_max=6*(v.size());
	
    //debug_output << "tam max " << tam_max << endl;
	
	Matrix h(tam_max,1);
	Matrix Hx(tam_max,6);
	//Matrix Hz(tam_max,6);
	
	std::vector<Matrix> Hz_v;

	std::vector<int> new_landmarks;//
	
	Matrix hi(6,1);
	int tam=0;

	for(int i=0;i<v.size();i+=1) //the observations
	{
		Observation3D obs = v[i];
		int obs_id = obs.id;
       // cout << "obs id " << obs_id << endl;
		
		bool recognized_landmark = false;
		int j=0;
		for (j=0; j<map.size();j++) // the map landmarks
		{
			int landmark_id = map[j].id;       
			if (obs_id == landmark_id)
			{
				recognized_landmark = true;
				//cout << "recognized id " << landmark_id << endl;
				break;
				
			
			}
		}

		if (recognized_landmark)   //
		{	
			Landmark3D map_landmark = map[j];

            debug_output << "recognized landmark " << map_landmark.id << " " << map_landmark.x << " " << map_landmark.y << " " << map_landmark.z << " " << map_landmark.roll*RAD2DEG << " " << map_landmark.pitch*RAD2DEG << " " << map_landmark.yaw*RAD2DEG << endl;
            debug_output << "observation " << obs.id << " " << obs.x << " " << obs.y << " " << obs.z << " " << obs.roll*RAD2DEG << " " << obs.pitch*RAD2DEG << " " << obs.yaw*RAD2DEG << endl;

			hi = computeInnovation2(obs,map_landmark);
			// update innovation h
			h.SubMatrix(tam+1,tam+6,1,1) << hi;
			debug_output << "innovation " << hi(1,1) << " " << hi(2,1) << " " << hi(3,1) << " " << hi(4,1)*RAD2DEG << " " << hi(5,1)*RAD2DEG << " " << hi(6,1)*RAD2DEG << endl;
			// compute jacobians
			Matrix Hxi(6,6);
			Hxi << computeHx2(obs);
			Hx.SubMatrix(tam+1,tam+6,1,6) << Hxi;
			Matrix Hzi(6,6);
			Hzi << computeHz2(obs);
			//Hz.SubMatrix(tam+1,tam+6,tam+1,tam+6) << Hzi;
			Hz_v.push_back(Hzi);

			tam+=6;	

		}
		else
		{
			new_landmarks.push_back(i);
		}
		
	}

	if(tam>0)//Kalman correction step
	{	
		cout << "tam " << tam << endl;
		Matrix PHt(6,tam);
		Matrix Hx_resized(tam,6);
		Hx_resized << Hx.submatrix(1,tam,1,6);
		
		PHt << cov_kalman * Hx_resized.t();
		Matrix S(tam,tam);
		S << Hx_resized*PHt;

		for(int i=0;i<Hz_v.size();i++)
		{
			Matrix Hzi(6,6);
			Hzi << Hz_v[i];
			Matrix aux1(6,6);
			aux1 << Hzi*R*Hzi.t();
			Matrix aux2 = S.SubMatrix(i*6+1,i*6+6,i*6+1,i*6+6);
			S.SubMatrix(i*6+1,i*6+6,i*6+1,i*6+6) << aux1 + aux2;
		}
		
		Matrix K(6,tam); 
		K << PHt*S.i();
		
		Matrix h_resized(tam,1);
		h_resized << h.submatrix(1,tam,1,1);
		
		kalman_pos << kalman_pos-K*h_resized;
		IdentityMatrix eye(6);
		cov_kalman << (eye-K*Hx_resized)*cov_kalman;
	
	}
	
	//add new landmarks to the map
	for(int k=0;k<new_landmarks.size();k++)
	{
		int ind=new_landmarks[k];
		Observation3D new_obs = v[ind];
		debug_output << "new landmark " << new_obs.id << endl;

        cout << "new landmark " << new_obs.id << endl;
		
        Matrix obs(6,1);obs(1,1)=new_obs.x;obs(2,1)=new_obs.y;obs(3,1)=new_obs.z;
        obs(4,1)=new_obs.roll; obs(5,1)=new_obs.pitch; obs(6,1)=new_obs.yaw;

        debug_output << "obs " << new_obs.id << " " << new_obs.x << " " <<new_obs.y << " " << new_obs.z << " " << new_obs.roll*RAD2DEG << " " << new_obs.pitch*RAD2DEG << " " << new_obs.yaw*RAD2DEG << endl;
		
		if (fabs(new_obs.pitch) > 10*DEG2RAD) cout << "Pitch of new landmark greater than 10º" << endl;
		if (fabs(new_obs.yaw) > 10*DEG2RAD) cout << "Yaw of new landmark greater than 10º" << endl;
		
		//obs(3,0)=new_obs.roll; obs(4,0)=new_obs.pitch; obs(5,0)=new_obs.yaw;

		//Correct the pose and add the new landmark to the map
		Matrix pos_abs(6,1);
		pos_abs=Comp(kalman_pos,obs);
        Landmark3D lp(new_obs.id, pos_abs(1,1), pos_abs(2,1), pos_abs(3,1), pos_abs(4,1), pos_abs(5,1), pos_abs(6,1));
        cout << "lp " << lp.id << " " << lp.x << " " <<lp.y << " " << lp.z << " " << lp.roll*RAD2DEG << " " << lp.pitch*RAD2DEG << " " << lp.yaw*RAD2DEG << endl;
        debug_output << "lp " << lp.id << " " << lp.x << " " <<lp.y << " " << lp.z << " " << lp.roll*RAD2DEG << " " << lp.pitch*RAD2DEG << " " << lp.yaw*RAD2DEG << endl;
        map.push_back(lp);

         cout << "landmark added " << endl;
		
	}
	
	debug_output << "correction " << kalman_pos(1,1) << " " << kalman_pos(2,1) <<  " " <<kalman_pos(3,1) <<  " " <<kalman_pos(4,1) <<  " " << kalman_pos(5,1)  <<  " " << kalman_pos(6,1) << endl;
	
	x = kalman_pos(1,1);
	y = kalman_pos(2,1);
	z = kalman_pos(3,1);
	roll = kalman_pos(4,1);
	pitch = kalman_pos(5,1);
	yaw = kalman_pos(6,1);
	
	return kalman_pos;
	
	
}

Matrix Kalman_Loc::computeInnovation2(Observation3D obs, Landmark3D map_landmark)
{
	Matrix hi(6,1);
	
	Matrix obs_v(6,1);
	obs_v(1,1) = obs.x;    obs_v(2,1) = obs.y;     obs_v(3,1) = obs.z;
	obs_v(4,1) = obs.roll; obs_v(5,1) = obs.pitch; obs_v(6,1) = obs.yaw;
	
    Matrix Xhz = Comp(kalman_pos,obs_v);

    debug_output << "Xhz " << Xhz(1,1) << " " << Xhz(2,1) <<  " " <<Xhz(3,1) <<  " " <<Xhz(4,1)*RAD2DEG <<  " " << Xhz(5,1)*RAD2DEG   <<  " " << Xhz(6,1)*RAD2DEG  << endl;

    Matrix Xb(6,1);
	Xb(1,1) = map_landmark.x;    Xb(2,1) = map_landmark.y;     Xb(3,1) = map_landmark.z;
	Xb(4,1) = map_landmark.roll; Xb(5,1) = map_landmark.pitch; Xb(6,1) = map_landmark.yaw;

	hi=Xhz-Xb;

	hi(4,1) = AngRango(hi(4,1));
	hi(5,1) = AngRango(hi(5,1));
	hi(6,1) = AngRango(hi(6,1));

	return hi;
}

Matrix Kalman_Loc::computeInnovation(Observation3D obs, Landmark3D map_landmark)
{

	Matrix hi(6,1);
	//invertir=[0;0;0];

	float ca=cos(kalman_pos(4,1));
	float sa=sin(kalman_pos(4,1));
	float cb=cos(kalman_pos(5,1));
	float sb=sin(kalman_pos(5,1));
	float cg=cos(kalman_pos(6,1));
	float sg=sin(kalman_pos(6,1));


	float cf=cos(obs.roll);
	float sf=sin(obs.roll);
	float ct=cos(obs.pitch);
	float st=sin(obs.pitch);
	float cp=cos(obs.yaw);
	float sp=sin(obs.yaw);

	Matrix Rh(3,3);
   Rh.Row(1) << cg*cb  << cg*sb*sa-sg*ca << cg*sb*ca+sg*sa;
	Rh.Row(2) <<  sg*cb <<  sg*sb*sa+cg*ca << sg*sb*ca-cg*sa;
	Rh.Row(3) <<   -sb  <<   cb*sa     <<      cb*ca;

	Matrix Rz(3,3);
	Rz.Row(1) <<  cp*ct <<  cp*st*sf-sp*cf << cp*st*cf+sp*sf;
	Rz.Row(2) <<  sp*ct <<  sp*st*sf+cp*cf << sp*st*cf-cp*sf;
	Rz.Row(3) <<   -st  <<   ct*sf         <<  ct*cf;

	Matrix Rhz (3,3);
	Rhz << Rh*Rz;


	if((Rhz(3,3)==0 && Rhz(3,2)==0)|| fabs(Rhz(3,1))==1)    
	{

		 cout << "Beta_hz 90º or 270º in computeInnovation" << endl;
	}

	Matrix Xhz(6,1);
	
	Matrix Z_aux(3,1);
	Z_aux(1,1) = obs.x;      Z_aux(2,1) = obs.y;       Z_aux(3,1) = obs.z;
	
	Matrix X_aux(3,1);
	X_aux(1,1) = kalman_pos(1,1);      X_aux(2,1) = kalman_pos(2,1);       X_aux(3,1) = kalman_pos(3,1);
	
	Matrix aux(3,1); 
	aux << Rh*Z_aux + X_aux;

	Xhz(1,1) = aux(1,1);
	Xhz(2,1) = aux(2,1);
	Xhz(3,1) = aux(3,1);

	Xhz(4,1)=atan2(Rhz(3,2),Rhz(3,3));

	int signCosBeta;
	if(fabs(Rhz(3,2))>fabs(Rhz(3,3)))
		signCosBeta=(Rhz(3,2)/sin(Xhz(4,1)) >= 0)?1:-1;
	else
		signCosBeta=(Rhz(3,3)/cos(Xhz(4,1))>=0)?1:-1;   

	Xhz(5,1)=atan2(-Rhz(3,1),sqrt(pow(Rhz(3,3),2)+pow(Rhz(3,2),2))*signCosBeta);
	Xhz(6,1)=atan2(Rhz(2,1),Rhz(1,1));

	Matrix Xb(6,1);
	Xb(1,1) = map_landmark.x;    Xb(2,1) = map_landmark.y;     Xb(3,1) = map_landmark.z;
	Xb(4,1) = map_landmark.roll; Xb(5,1) = map_landmark.pitch; Xb(6,1) = map_landmark.yaw;

	hi=Xhz-Xb;

	hi(4,1) = AngRango(hi(4,1));
	hi(5,1) = AngRango(hi(5,1));
	hi(6,1) = AngRango(hi(6,1));

	return hi;

}

/***************************************************************************************/


Matrix Kalman_Loc::computeHx2(Observation3D obs)
{
	Matrix Hx(6,6);
	Hx = 0.0;
	
	Matrix obs_v(6,1);
	obs_v(1,1) = obs.x;    obs_v(2,1) = obs.y;     obs_v(3,1) = obs.z;
	obs_v(4,1) = obs.roll; obs_v(5,1) = obs.pitch; obs_v(6,1) = obs.yaw;
	
	Hx = J1_n(kalman_pos, obs_v);
	
	return Hx;
	
}

Matrix Kalman_Loc::computeHx(Observation3D obs)
{
	Matrix Hx(6,6);
	Hx = 0.0;

	Matrix Z(6,1);
	Z(1,1) = obs.x;      Z(2,1) = obs.y;       Z(3,1) = obs.z;
	Z(4,1) = obs.roll;   Z(5,1) = obs.pitch;   Z(6,1) = obs.yaw;

	double ca=cos(kalman_pos(4,1));
	double sa=sin(kalman_pos(4,1));
	double cb=cos(kalman_pos(5,1));
	double sb=sin(kalman_pos(5,1));
	double cg=cos(kalman_pos(6,1));
	double sg=sin(kalman_pos(6,1));

	double xz=Z(1,1);
	double yz=Z(2,1);
	double zz=Z(3,1);
	double cf=cos(Z(4,1));
	double sf=sin(Z(4,1));
	double ct=cos(Z(5,1));
	double st=sin(Z(5,1));
	double cp=cos(Z(6,1));
	double sp=sin(Z(6,1));

	Matrix Rh(3,3);
	Rh.Row(1) << cg*cb <<  cg*sb*sa-sg*ca << cg*sb*ca+sg*sa;
	Rh.Row(2) <<   sg*cb <<  sg*sb*sa+cg*ca << sg*sb*ca-cg*sa;
	Rh.Row(3) <<   -sb << cb*sa << cb*ca;

	Matrix Rz(3,3);
	Rz.Row(1) << cp*ct << cp*st*sf-sp*cf << cp*st*cf+sp*sf;
	Rz.Row(2) << sp*ct << sp*st*sf+cp*cf << sp*st*cf-cp*sf;
	Rz.Row(3) <<   -st <<    ct*sf       <<    ct*cf;

	Matrix Rhz(3,3);
	Rhz << Rh*Rz;

	if((Rhz(3,3)==0 && Rhz(3,2)==0)|| fabs(Rhz(3,1))==1)    
	{
		 cout << "Beta_hz 90º or 270º in Hx" << endl;
	}

	double alfa=atan2(Rhz(3,2),Rhz(3,3));

	int signCosBeta;
	if(fabs(Rhz(3,2))>fabs(Rhz(3,3)))
		signCosBeta=(Rhz(3,2)/sin(alfa) >= 0)?1:-1;
	else
		signCosBeta=(Rhz(3,3)/cos(alfa) >= 0)?1:-1;   


	double den=sqrt(1-pow(Rhz(3,1),2))*signCosBeta;

	IdentityMatrix eye(3);
	Hx.SubMatrix(1,3,1,3) << eye;

	Hx(1,4)=(cg*sb*ca+sg*sa)*yz+(-cg*sb*sa+sg*ca)*zz;
	Hx(2,4)=(sg*sb*ca-cg*sa)*yz-(sg*sb*sa+cg*ca)*zz;
	Hx(3,4)=(cb*ca)*yz-(cb*sa)*zz;

	Hx(4,4)=(((sp*st*sf+cp*cf)*cb*ca-(ct*sf)*cb*sa)*Rhz(3,3)-((sp*st*cf-cp*sf)*cb*ca-(ct*cf)*cb*sa)*Rhz(3,2))/(den*den);

	Hx(5,4)=(-(sp*ct)*cb*ca-(st)*cb*sa)/den;
	Hx(6,4)=(((sg*sb*ca-cg*sa)*sp*ct+(sg*sb*sa+cg*ca)*st)*Rhz(1,1)-((cg*sb*ca+sg*sa)*sp*ct+(cg*sb*sa-sg*ca)*st)*Rhz(2,1))/(den*den);

	Hx(1,5)=(-cg*sb)*xz+(cg*cb*sa)*yz+(cg*cb*ca)*zz;
	Hx(2,5)=(-sg*sb)*xz+(sg*cb*sa)*yz+(sg*cb*ca)*zz;
	Hx(3,5)=(-cb)*xz-(sb*sa)*yz-(sb*ca)*zz;
	Hx(4,5)=((-(cp*st*sf-sp*cf)*cb-(sp*st*sf+cp*cf)*sb*sa-(ct*sf)*sb*ca)*Rhz(3,3)+((cp*st*cf+sp*sf)*cb+(sp*st*cf-cp*sf)*sb*sa+(ct*cf)*sb*ca)*Rhz(3,2))/(den*den);
	Hx(5,5)=((cp*ct)*cb+(sp*ct)*sb*sa-(st)*sb*ca)/den;
	Hx(6,5)=(((-sg*sb)*cp*ct+(sg*cb*sa)*sp*ct-(sg*cb*ca)*st)*Rhz(1,1)-((-cg*sb)*cp*ct+(cg*cb*sa)*sp*ct-(cg*cb*ca)*st)*Rhz(2,1))/(den*den);

	Hx(1,6)=(-sg*cb)*xz+(-sg*sb*sa-cg*ca)*yz+(-sg*sb*ca+cg*sa)*zz;
	Hx(2,6)=(cg*cb)*xz+(cg*sb*sa-sg*ca)*yz+(cg*sb*ca+sg*sa)*zz;
	Hx(3,6)=0;
	Hx(4,6)=0;
	Hx(5,6)=0;
	Hx(6,6)=(((cg*cb)*cp*ct+(cg*sb*sa-sg*ca)*sp*ct-(cg*sb*ca+sg*sa)*st)*Rhz(1,1)+((sg*cb)*cp*ct+(sg*sb*sa+cg*ca)*sp*ct-(sg*sb*ca-cg*sa)*st)*Rhz(2,1))/(den*den);

	return Hx;

}

/***************************************************************************************/

Matrix Kalman_Loc::computeHz2(Observation3D obs)
{

	Matrix Hz(6,6);
	Hz = 0.0;
	
	Matrix obs_v(6,1);
	obs_v(1,1) = obs.x;    obs_v(2,1) = obs.y;     obs_v(3,1) = obs.z;
	obs_v(4,1) = obs.roll; obs_v(5,1) = obs.pitch; obs_v(6,1) = obs.yaw;
	
	Hz = J2_n(kalman_pos, obs_v);
	
	return Hz;
}


Matrix Kalman_Loc::computeHz(Observation3D obs)
{

	Matrix Hz(6,6);
	Hz = 0.0;

	Matrix Z(6,1);
	Z(1,1) = obs.x;      Z(2,1) = obs.y;       Z(3,1) = obs.z;
	Z(4,1) = obs.roll;   Z(5,1) = obs.pitch;   Z(6,1) = obs.yaw;
	
	double ca=cos(kalman_pos(4,1));
	double sa=sin(kalman_pos(4,1));
	double cb=cos(kalman_pos(5,1));
	double sb=sin(kalman_pos(5,1));
	double cg=cos(kalman_pos(6,1));
	double sg=sin(kalman_pos(6,1));

	double xz=Z(1,1);
	double yz=Z(2,1);
	double zz=Z(3,1);
	double cf=cos(Z(4,1));
	double sf=sin(Z(4,1));
	double ct=cos(Z(5,1));
	double st=sin(Z(5,1));
	double cp=cos(Z(6,1));
	double sp=sin(Z(6,1));
	
	Matrix Rh(3,3);
	Rh.Row(1) << cg*cb <<  cg*sb*sa-sg*ca << cg*sb*ca+sg*sa;
	Rh.Row(2) <<   sg*cb <<  sg*sb*sa+cg*ca << sg*sb*ca-cg*sa;
	Rh.Row(3) <<   -sb << cb*sa << cb*ca;

	Matrix Rz(3,3);
	Rz.Row(1) << cp*ct << cp*st*sf-sp*cf << cp*st*cf+sp*sf;
	Rz.Row(2) << sp*ct << sp*st*sf+cp*cf << sp*st*cf-cp*sf;
	Rz.Row(3) <<   -st <<    ct*sf       <<    ct*cf;

	Matrix Rhz(3,3);
	Rhz << Rh*Rz;

	if((Rhz(3,3)==0 && Rhz(3,2)==0)|| fabs(Rhz(3,1))==1)    
	{
		 cout << "Beta_hz 90º or 270º in Hz" << endl;
	}
	
	double alfa=atan2(Rhz(3,2),Rhz(3,3));

	int signCosBeta;
	if(fabs(Rhz(3,2))>fabs(Rhz(3,3)))
		signCosBeta=(Rhz(3,2)/sin(alfa) >= 0)?1:-1;
	else
		signCosBeta=(Rhz(3,3)/cos(alfa) >= 0)?1:-1; 

	double den=sqrt(1-Rhz(3,1)*Rhz(3,1))*signCosBeta;


	Hz(1,1)=cg*cb;
	Hz(2,1)=sg*cb;
	Hz(3,1)=-sb;
	Hz(4,1)=0;
	Hz(5,1)=0;
	Hz(6,1)=0;

	Hz(1,2)=cg*sb*sa-sg*ca;
	Hz(2,2)=sg*sb*sa+cg*ca;
	Hz(3,2)=cb*sa;
	Hz(4,2)=0;
	Hz(5,2)=0;
	Hz(6,2)=0;

	Hz(1,3)=cg*sb*ca+sg*sa;
	Hz(2,3)=sg*sb*ca-cg*sa;
	Hz(3,3)=cb*ca;
	Hz(4,3)=0;
	Hz(5,3)=0;
	Hz(6,3)=0;

	Hz(1,4)=0;
	Hz(2,4)=0;
	Hz(3,4)=0;
	Hz(4,4)=((-(cp*st*cf+sp*sf)*sb+(sp*st*cf-cp*sf)*cb*sa+(ct*cf)*cb*ca)*Rhz(3,3)-((cp*st*sf-sp*cf)*sb-(sp*st*sf+cp*cf)*cb*sa-(ct*sf)*cb*ca)*Rhz(3,2))/(den*den);
	Hz(5,4)=0;
	Hz(6,4)=0;

	Hz(1,5)=0;
	Hz(2,5)=0;
	Hz(3,5)=0;
	Hz(4,5)=((-(cp*ct*sf)*sb+(sp*ct*sf)*cb*sa-(st*sf)*cb*ca)*Rhz(3,3)-(-(cp*ct*cf)*sb+(sp*ct*cf)*cb*sa-(st*cf)*cb*ca)*Rhz(3,2))/(den*den);
	Hz(5,5)=(-(cp*st)*sb+(sp*st)*cb*sa+(ct)*cb*ca)/den;
	Hz(6,5)=((-(sg*cb)*cp*st-(sg*sb*sa+cg*ca)*sp*st-(sg*sb*ca-cg*sa)*ct)*Rhz(1,1)+((cg*cb)*cp*st+(cg*sb*sa-sg*ca)*sp*st+(cg*sb*ca+sg*sa)*ct)*Rhz(2,1))/(den*den);

	Hz(1,6)=0;
	Hz(2,6)=0;
	Hz(3,6)=0;
	Hz(4,6)=(((sp*st*sf+cp*cf)*sb+(cp*st*sf-sp*cf)*cb*sa)*Rhz(3,3)-((sp*st*cf-cp*sf)*sb+(cp*st*cf+sp*sf)*cb*sa)*Rhz(3,2))/(den*den);
	Hz(5,6)=((-sp*ct)*sb-(cp*ct)*cb*sa)/den;
	Hz(6,6)=((-(sg*cb)*sp*ct+(sg*sb*sa+cg*ca)*cp*ct)*Rhz(1,1)-(-(cg*cb)*sp*ct+(cg*sb*sa-sg*ca)*cp*ct)*Rhz(2,1))/(den*den);

	return Hz;


}

void Kalman_Loc::obtainObstacles()
{
	for (int i=0; i< map.size(); i++)
	{
		Landmark3D lm = map[i];
	
	
	
	}


}

