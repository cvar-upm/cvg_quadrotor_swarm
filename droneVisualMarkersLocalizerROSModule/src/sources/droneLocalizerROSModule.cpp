//////////////////////////////////////////////////////
//  DroneTrajectoryPlannerROSModule.h
//
//  Created on: Jul 29, 2013
//      Author: pdelapuente & joselusl
//
//  Last modification on: Oct 27, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneLocalizerROSModule.h"

using namespace std;


DroneLocalizer::DroneLocalizer() : DroneModule(droneModule::active,FREQ_LOCALIZER)
{
    if(!init())
        cout<<"Error init"<<endl;
        
    return;
}



DroneLocalizer::~DroneLocalizer()
{
	close();
	return;
}

void DroneLocalizer::open(ros::NodeHandle & nIn, std::string moduleName)
{
	//Node
    DroneModule::open(nIn,moduleName);

#ifdef DRONE_LOCALIZER_LOGGING
    mylog.open(stackPath+"logs/localizerLog.dat");
    cout<<"loging in: "<<stackPath+"logs/localizerLog.dat"<<endl;
#endif // DRONE_LOCALIZER_LOGGING

    loc.initialization(stackPath+"configs/drone"+stringId+"/params_localization_obs.xml");

	
    ////subscribers
    droneOdomPoseSubs = n.subscribe(DRONE_LOCALIZER_POSE_SUBSCRIPTION, 1, &DroneLocalizer::droneOdomPoseCallback, this);
    droneObsVectorSubs = n.subscribe(DRONE_LOCALIZER_ARUCO_OBSERVATIONVEC_LIST, 1, &DroneLocalizer::droneObsVectorCallback, this);


    //////Publishers
    //drone pose
    // TODO_P: eliminar el publisher que sobra
    dronePosePubl = n.advertise<droneMsgsROS::dronePose>(DRONE_LOCALIZER_POSE_PUBLICATION, 1, true);

    droneSpeedPubl = n.advertise<droneMsgsROS::droneSpeeds>(DRONE_LOCALIZER_SPEEDS_PUBLICATION, 1, true);

    dronePoseNewAngNotationPubl = n.advertise<droneMsgsROS::dronePose>(DRONE_LOCALIZER_POSE_PUBLICATION_2ND_YPR_CONVENTION, 1, true);

    mapPubl = n.advertise<droneMsgsROS::landmarkVector>(DRONE_LOCALIZER_LANDMARK_LIST, 1, true);

    // read configuration of the FilteredDerivative blocks (speed estimation)
    try {
        XMLFileReader my_xml_reader(stackPath+"configs/drone"+ std::to_string(idDrone)+"/trajectory_controller_config.xml");
        XMLFileReader my_xml_reader_ekfstuff(stackPath+"configs/drone"+ std::to_string(idDrone)+"/ekf_state_estimator_config.xml");
        double tr_vx, saturation_value_vx, tr_vy, saturation_value_vy, tr_dz, saturation_value_dz;
        double tr_dyaw, saturation_value_dyaw, tr_dpitch, saturation_value_dpitch, tr_droll, saturation_value_droll;
        tr_vx               = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","tr_v"} );
        tr_vy               = tr_vx;
        tr_dz               = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","tr_dz"} );
        tr_dyaw             = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","tr_dyaw"} );
        tr_dpitch = tr_dyaw;
        tr_droll  = tr_dyaw;
        saturation_value_vx   = my_xml_reader_ekfstuff.readDoubleValue( {"ekf_state_estimator_config","ground_optical_flow_maximum_speed"} );
        saturation_value_vy   = saturation_value_vx;
        saturation_value_dz   = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","saturation_value_dz"} );
        saturation_value_dyaw = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_pose_feedback_low_pass_filters","saturation_value_dyaw"} );
        saturation_value_dpitch = saturation_value_dyaw;
        saturation_value_droll  = saturation_value_dyaw;
        filter_x2vx.setResponseTime(tr_vx);
        filter_x2vx.enableSaturation( true, -saturation_value_vx, +saturation_value_vx);
        filter_y2vy.setResponseTime(tr_vy);
        filter_y2vy.enableSaturation( true, -saturation_value_vy, +saturation_value_vy);
        filter_z2dz.setResponseTime(tr_dz);
        filter_z2dz.enableSaturation( true, -saturation_value_dz, +saturation_value_dz);
        filter_yaw2dyaw.setResponseTime(tr_dyaw);
        filter_yaw2dyaw.enableSaturation( true, -saturation_value_dyaw, +saturation_value_dyaw);
        filter_pitch2dpitch.setResponseTime(tr_dpitch);
        filter_pitch2dpitch.enableSaturation( true, -saturation_value_dpitch, +saturation_value_dpitch);
        filter_roll2droll.setResponseTime(tr_droll);
        filter_roll2droll.enableSaturation( true, -saturation_value_droll, +saturation_value_droll);
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }

    //Flag of module opened
    droneModuleOpened=true;
	
	//End
	return;
}




void DroneLocalizer::droneOdomPoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg)
{   

	// input coordinates in common reference frame, changed to internal reference frame
    droneMsgsROS::dronePose mobr_ref_frame_msg;
    //referenceFrames::refFrameChangeZYX2XYZ(*msg, &mobr_ref_frame_msg); //common ref frame not used anymore!!!
    
    droneMsgsROS::dronePose input_msg;
    input_msg.time=msg->time;
    input_msg.x=msg->x;
    input_msg.y=msg->y;
    input_msg.z=msg->z;
    input_msg.yaw=msg->yaw;
    input_msg.pitch=msg->pitch;
    input_msg.roll=msg->roll;


    referenceFrames::wYvPuR2xYyPzR(input_msg, &mobr_ref_frame_msg); // GMR wYvPuR 2 GMR xYyPzR
    
    x_odom =mobr_ref_frame_msg.x;
    y_odom =mobr_ref_frame_msg.y;
    z_odom =mobr_ref_frame_msg.z;
    roll_odom =mobr_ref_frame_msg.roll;
    pitch_odom =mobr_ref_frame_msg.pitch;
    yaw_odom =mobr_ref_frame_msg.yaw;

    /*cout<<"odom pose msg converted"<<endl;
    {
        cout<<"  x="<<x_odom<<"; y="<<y_odom<<"; z="<<z_odom <<"; roll="<<roll_odom<<"; pitch="<<pitch_odom<<"; yaw="<<yaw_odom <<endl;
    }*/
    
    //prev_odom initialization (in GMR xYyPzR)
    if (!isStarted())
    {
    	loc.prev_odom_pose(1) = x_odom;
    	loc.prev_odom_pose(2) = y_odom;
    	loc.prev_odom_pose(3) = z_odom;
    	loc.prev_odom_pose(4) = roll_odom;
    	loc.prev_odom_pose(5) = pitch_odom;
    	loc.prev_odom_pose(6) = yaw_odom;
    }

	new_odom_data = true;

    return;
}

void DroneLocalizer::droneObsVectorCallback(const droneMsgsROS::obsVector::ConstPtr& msg)
{
	obs.clear();
	// no transformation or change of convention
    for(unsigned int i=0; i<msg->obs.size();i++)
	{
        droneMsgsROS::Observation3D obs_msg = msg->obs[i];

        // [*]
        // msg in wYvPuR to
        // localization lib which is in xYyPzR
        // yaw_xYyPzR  = roll_wYvPuR
        // roll_xYyPzR = yaw_wYvPuR
		Observation3D observation;
		observation.x = obs_msg.x;
		observation.y = obs_msg.y;
		observation.z = obs_msg.z;
        observation.roll = obs_msg.yaw;     // [*] This is not a bug!!
        observation.pitch = obs_msg.pitch;
        observation.yaw = obs_msg.roll;     // [*] This is not a bug!!
		observation.id = obs_msg.id;
        /*
		if (obs_msg.is_known)
			observation.is_known = true;
		else
			observation.is_known = false;
        */
		
		
   	obs.push_back(observation);
	}
    
    //cout<<"obs size " << obs.size() <<endl;

	new_obs_vector = true;

    return;
}



int DroneLocalizer::publishPose(droneMsgsROS::dronePose dronePoseEstimate)
{
    if(droneModuleOpened==false)
        return 0;


    dronePosePubl.publish(dronePoseEstimate);

    return 1;
}

int DroneLocalizer::publishSpeeds(droneMsgsROS::droneSpeeds droneSpeedEstimate)
{
    droneSpeedPubl.publish(droneSpeedEstimate);

    return 1;
}

int DroneLocalizer::publishPoseNewAngNotation(droneMsgsROS::dronePose dronePoseEstimateNewAngNotation)
{
    if(droneModuleOpened==false)
        return 0;


    dronePoseNewAngNotationPubl.publish(dronePoseEstimateNewAngNotation);


    return 1;
}

int DroneLocalizer::publishMap(droneMsgsROS::landmarkVector map_msg)
{
    if(droneModuleOpened==false)
        return 0;


    mapPubl.publish(map_msg);


    return 1;
}



bool DroneLocalizer::init()
{
    //Init pose
    x_odom=0.0;
    y_odom=0.0;
    z_odom=0.0;
    roll_odom=0.0;
    pitch_odom=0.0;
    yaw_odom=0.0;

	new_odom_data = false;
    new_obs_vector = false;
   
//   localizerLogMsgStrm.str(std::string());
    run_timestamp  = ros::Duration(0.0);

    //end
    return true;
}



void DroneLocalizer::close()
{
    DroneModule::close();
    //log
    mylog.close();
    return;
}


bool DroneLocalizer::resetValues()
{

    return true;
}


bool DroneLocalizer::startVal()
{
    //Reset time

    //time
    //double initTime = ros::Time::now().toSec();


    //End
    return DroneModule::startVal();
}



bool DroneLocalizer::stopVal()
{
    return DroneModule::stopVal();
}



bool DroneLocalizer::run()
{
    if(!DroneModule::run())
    {
        //cout<<"Module not started yet!"<<endl;
        return false;
    }

    if(droneModuleOpened==false)
        return false;

	bool new_estimated_pose = false;
    /////////////////////////////////////////////////////////
	//predict
	if (new_odom_data)
	{
		loc.KalmanPredict(x_odom, y_odom, z_odom, roll_odom, pitch_odom, yaw_odom);
		new_odom_data = false;
		new_estimated_pose = true;
	}
	
	x_predicted = loc.x;
	y_predicted = loc.y;
	z_predicted = loc.z;
	yaw_predicted = loc.roll;
	pitch_predicted = loc.pitch;
	roll_predicted = loc.yaw;
	
	
	cout << "predict (xyz)" << loc.x << " " << loc.y << " " << loc.z << " " << loc.roll*RAD2DEG << " " << loc.pitch*RAD2DEG << " " << loc.yaw*RAD2DEG << endl;
	
	/////////////////////////////////////////////////////////
	//correct
	if (new_obs_vector)
	{
		loc.KalmanUpdate(obs);
		obs.clear();
		new_obs_vector = false;
		new_estimated_pose = true;
	}

	/////////////////////////////////////////////////////////
	// publish
	
    droneMsgsROS::dronePose estimatedPose;

    estimatedPose.time = ros::Time::now().toSec();
    estimatedPose.x=loc.x;
    estimatedPose.y=loc.y;
    estimatedPose.z=loc.z;
    estimatedPose.roll=loc.yaw;
    estimatedPose.pitch=loc.pitch;
    estimatedPose.yaw=loc.roll;

	cout << "update " << loc.x << " " << loc.y << " " << loc.z << " " << loc.roll*RAD2DEG << " " << loc.pitch*RAD2DEG << " " << loc.yaw*RAD2DEG << endl;


    
   if (new_estimated_pose)
   {
    	//Publish pose mine
    	publishPose(estimatedPose);

        double xs_t, ys_t, zs_t, yaws_t, pitchs_t, rolls_t;
        double vxs_t, vys_t, vzs_t, dyaws_t, dpitchs_t, drolls_t;
        xs_t = estimatedPose.x;
        ys_t = estimatedPose.y;
        zs_t = estimatedPose.z;
        yaws_t = estimatedPose.yaw;
        pitchs_t = estimatedPose.pitch;
        rolls_t = estimatedPose.roll;
        filter_x2vx.setInput(xs_t);
        filter_y2vy.setInput(ys_t);
        filter_z2dz.setInput(zs_t);
        vxs_t   = filter_x2vx.getOutput();
        vys_t   = filter_y2vy.getOutput();
        vzs_t   = filter_z2dz.getOutput();
        double yaw_kminus1 = filter_yaw2dyaw.getInput();
        double Dyaw = cvg_utils_library::getAngleError( yaws_t, yaw_kminus1);
        filter_yaw2dyaw.setInput(yaw_kminus1+Dyaw);
        double pitch_kminus1 = filter_pitch2dpitch.getInput();
        double Dpitch = cvg_utils_library::getAngleError( pitchs_t, pitch_kminus1);
        filter_pitch2dpitch.setInput(pitch_kminus1+Dpitch);
        double roll_kminus1 = filter_roll2droll.getInput();
        double Droll = cvg_utils_library::getAngleError( rolls_t, roll_kminus1);
        filter_roll2droll.setInput(roll_kminus1+Droll);
        dyaws_t   = filter_yaw2dyaw.getOutput();
        dpitchs_t = filter_pitch2dpitch.getOutput();
        drolls_t  = filter_roll2droll.getOutput();

        droneMsgsROS::droneSpeeds estimatedSpeed;
        estimatedSpeed.dx = vxs_t;
        estimatedSpeed.dy = vys_t;
        estimatedSpeed.dz = vzs_t;
        estimatedSpeed.dyaw = dyaws_t;
        estimatedSpeed.dpitch = dpitchs_t;
        estimatedSpeed.droll = drolls_t;

        publishSpeeds(estimatedSpeed);

		//////////////////////////////////////////////
		 //Publish pose comm ref frame
        droneMsgsROS::dronePose new_ang_notation_msg;
		//referenceFrames::refFrameChangeXYZ2ZYX(estimatedPose,&comm_ref_frame_msg); //common ref frame not used anymore!!!
		 
		referenceFrames::xYyPzR2wYvPuR(estimatedPose,&new_ang_notation_msg);
		publishPoseNewAngNotation(new_ang_notation_msg);
   }
	
	//////////////////////////////////////////////
	
    droneMsgsROS::landmarkVector map_msg;
	
    for (unsigned int i=0;i<loc.map.size();i++)
	{
        droneMsgsROS::Landmark3D lm_msg;
		lm_msg.x = loc.map[i].x;
		lm_msg.y = loc.map[i].y;
		lm_msg.z = loc.map[i].z;
        lm_msg.roll = loc.map[i].yaw;
		lm_msg.pitch = loc.map[i].pitch;
        lm_msg.yaw = loc.map[i].roll;
		lm_msg.id = loc.map[i].id;
		if (loc.map[i].is_known)
			lm_msg.is_known = 1;
		else
			lm_msg.is_known = 0;
		
		map_msg.landmark_vector.push_back(lm_msg);
	
	
	}
	
	 //Publish map	
	 publishMap(map_msg);
	//////////////////////////////////////////////

     run_timestamp = ros::Time::now() - ros::Time(0,0);
#ifdef DRONE_LOCALIZER_LOGGING
     logLocalizerMsgStr();
#endif // DRONE_LOCALIZER_LOGGING



    return false;
}


void DroneLocalizer::logLocalizerMsgStr()
{

    #ifdef DRONE_LOCALIZER_LOGGING
    if(mylog.is_open())
    {
        mylog
            /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
            /* tag         */   << " [localization;state]"
            /* isStarted   */   << " started:" << isStarted()
            /* prev_odom_x, prev_odom_y, prev_odom_z, prev_odom_yaw, prev_odom_pitch, prev_odom_roll */ 
                                << " prev_odom_x:" << loc.prev_odom_pose(1)
                                                      << " prev_odom_y:" << loc.prev_odom_pose(2)
                                                      << " prev_odom_z:" << loc.prev_odom_pose(3)
                                                      << " prev_odom_yaw:" << loc.prev_odom_pose(6)
                                                      << " prev_odom_pitch:" << loc.prev_odom_pose(5)
                                                      << " prev_odom_roll:" << loc.prev_odom_pose(4)
            /* current_odom_x, current_odom_y, current_odom_z, current_odom_yaw, current_odom_pitch, current_odom_roll */ 
            										  << " current_odom_x:" << x_odom
            										  << " current_odom_y:" << y_odom
            										  << " current_odom_z:" << z_odom
            										  << " current_odom_yaw:" << roll_odom
            										  << " current_odom_pitch:" << pitch_odom
            										  << " current_odom_roll:" << yaw_odom
            /* isStarted   */   << " num_obs:" << obs.size()
            /* x_predicted, y_predicted, z_predicted, yaw_predicted, pitch_predicted, roll_predicted */ 
            											<< " x_predicted:"   << x_predicted
                                            << " y_predicted:"   << y_predicted
                                            << " z_predicted:"   << z_predicted
                                            << " yaw_predicted:" << yaw_predicted
                                            << " pitch_predicted:" << pitch_predicted
                                            << " roll_predicted:" << roll_predicted  
            /* x_corrected, y_corrected, z_corrected, yaw_corrected, pitch_corrected, roll_corrected */ 
            										  << " x_corrected:"   << loc.x
                                            << " y_corrected:"   << loc.y
                                            << " z_corrected:"   << loc.z
                                            << " yaw_corrected:" << loc.roll
                                            << " pitch_corrected:" << loc.pitch
                                            << " roll_corrected:" << loc.yaw  
                                << std::endl;
    }
    #endif // DRONE_LOCALIZER_LOGGING
}

