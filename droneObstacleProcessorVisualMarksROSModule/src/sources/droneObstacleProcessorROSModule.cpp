//////////////////////////////////////////////////////
//  droneObstacleProcessorROSModule.cpp
//
//  Created on: Jul 29, 2013
//      Author: pdelapuente
//
//  Last modification on: Oct 29, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneObstacleProcessorROSModule.h"



using namespace std;




DroneObstacleProcessorROSModule::DroneObstacleProcessorROSModule() : DroneModule(droneModule::active,FREQ_OBSTACLEPROCESSOR)
{
    if(!init())
    {
        cout<<"[DOP-ROS] Error init"<<endl;
    }
        

    return;
}



DroneObstacleProcessorROSModule::~DroneObstacleProcessorROSModule()
{
	close();
	return;
}

bool DroneObstacleProcessorROSModule::init()
{
    DroneModule::init();

    if(!MyDroneObstacleProcessor.init())
        return false;

    return true;
}

void DroneObstacleProcessorROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
	//Node
    DroneModule::open(nIn,moduleName);
    
#ifdef DRONE_OBSTACLES_LOGGING
    mylog.open(stackPath+"logs/ObstaclesLog.dat");
    cout<<"loging in: "<<stackPath+"logs/ObstaclesLog.dat"<<endl;
#endif // DRONE_OBSTACLES_LOGGING


    if(!MyDroneObstacleProcessor.open(stackPath+"configs/drone"+stringId+"/params_localization_obs.xml"))
    {
        cout<<"[DOP-ROS] error opening droneObstacleProcessor"<<endl;
        return;
    }

	
    //// Topics ///

    //////subscribers
    landmarksSubs = n.subscribe(DRONE_OBSTACLE_PROCESSOR_LANDMARK_LIST, 1, &DroneObstacleProcessorROSModule::landmarksCallback, this);


    //////Publishers
    //obstacles
    obstaclesPubl = n.advertise<droneMsgsROS::obstaclesTwoDim>(DRONE_OBSTACLE_PROCESSOR_OBSTACLE_LIST, 1, true);


    //Flag of module opened
    droneModuleOpened=true;
	
	//End
	return;
}




void DroneObstacleProcessorROSModule::landmarksCallback(const droneMsgsROS::landmarkVector::ConstPtr& msg)
{
    std::vector<Landmark3D> landmarks;
    //landmarks.clear();
    for (unsigned int i=0;i<msg->landmark_vector.size(); i++)
    {
        droneMsgsROS::Landmark3D lm_msg = msg->landmark_vector[i];
    	
    	Landmark3D lm;
    	lm.x = lm_msg.x;
    	lm.y = lm_msg.y;
    	lm.z = lm_msg.z;
        lm.roll = lm_msg.yaw;
    	lm.pitch = lm_msg.pitch;
        lm.yaw = lm_msg.roll;
    	
    	lm.id = lm_msg.id;
    	if (lm_msg.is_known)
			lm.is_known = true;
		else
			lm.is_known = false;
    	
        landmarks.push_back(lm);
    	
    
    }

    //set
    if(!MyDroneObstacleProcessor.setLandmarks(landmarks))
        return;


    //Logging
#ifdef DRONE_OBSTACLES_LOGGING
     logLandmarks(landmarks);
#endif // DRONE_OBSTACLES_LOGGING



    return;
}



int DroneObstacleProcessorROSModule::publishObstacles(droneMsgsROS::obstaclesTwoDim obstacles)
{
    if(droneModuleOpened==false)
        return 0;


    obstaclesPubl.publish(obstacles);


    return 1;
}



void DroneObstacleProcessorROSModule::close()
{
    DroneModule::close();
#ifdef DRONE_OBSTACLES_LOGGING
    mylog.close();
#endif

    if(!MyDroneObstacleProcessor.close())
        return;

	 return;
}



bool DroneObstacleProcessorROSModule::resetValues()
{
    if(!MyDroneObstacleProcessor.reset())
        return false;


    return true;

}



bool DroneObstacleProcessorROSModule::startVal()
{
    if(!MyDroneObstacleProcessor.start())
        return false;


    //End
    return DroneModule::startVal();
}



bool DroneObstacleProcessorROSModule::stopVal()
{
    if(!MyDroneObstacleProcessor.stop())
        return false;

    return DroneModule::stopVal();
}



bool DroneObstacleProcessorROSModule::run()
{
    if(!DroneModule::run())
        return false;


    if(droneModuleOpened==false)
        return false;

    /////////////////////////////////////////////////////////
	//obtain obstacles
    MyDroneObstacleProcessor.run();

	/////////////////////////////////////////////////////////
	// publish

    //Receive obstacles
    std::vector<Obstacle2D*> obstacles;
    MyDroneObstacleProcessor.getObstacles(obstacles);

	
    droneMsgsROS::obstaclesTwoDim obstacles_msg;
	
    for (unsigned int i=0;i<obstacles.size();i++)
	{

        cout<<"Publishing:"<<endl;

        Obstacle2D* obstacle = obstacles[i];
	
        if (obstacle->type == RECTANGLE)
		{
            Rectangle* obs_r = dynamic_cast<Rectangle*> (obstacle);
            droneMsgsROS::obstacleTwoDimWall wall;
            wall.id=obs_r->id;
			wall.centerX = obs_r->pose.x;
			wall.centerY = obs_r->pose.y;
         wall.sizeX = obs_r->xl;
         wall.sizeY = obs_r->yl;
			wall.yawAngle = obs_r->pose.theta.getValue();
			
			obstacles_msg.walls.push_back(wall);

            cout<<"-wall added!"<<endl;
		}
			
        if (obstacle->type == CIRCLE)
		{
            Circle* obs_c = dynamic_cast<Circle*> (obstacle);
            droneMsgsROS::obstacleTwoDimPole pole;
            pole.id=obs_c->id;
			pole.centerX = obs_c->center.x;
			pole.centerY = obs_c->center.y;
			pole.radiusX = obs_c->radius;
			pole.radiusY = obs_c->radius;
			pole.yawAngle = 0;
		
			obstacles_msg.poles.push_back(pole);

            cout<<"-pole added!"<<endl;
		}
	
	
	}
	
     //Publish
    obstacles_msg.time=ros::Time::now().toSec();

     publishObstacles(obstacles_msg);
     
     	//////////////////////////////////////////////








    return false;
}
#ifdef DRONE_OBSTACLES_LOGGING
void DroneObstacleProcessorROSModule::logLandmarks(const std::vector<Landmark3D>& landmarks)
{


    if(mylog.is_open())
    {
        run_timestamp = ros::Time::now() - ros::Time(0,0);
        mylog
            /* timestamp   */   << run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec
            /* tag         */   << " [obstacles;state]"
            /* isStarted   */   << " started:" << isStarted();
            /* map landmarks */
            mylog<<" id:[";
            for (unsigned int i=0;i<landmarks.size();i++) {
                mylog<<landmarks[i].id<<";";
            }
            mylog<<"]";
            mylog<<" x:[";
            for (unsigned int i=0;i<landmarks.size();i++) {
                mylog<<landmarks[i].x<<";";
            }
            mylog<<"]";
            mylog<<" y:[";
            for (unsigned int i=0;i<landmarks.size();i++) {
                mylog<<landmarks[i].y<<";";
            }
            mylog<<"]";
            mylog<<" z:[";
            for (unsigned int i=0;i<landmarks.size();i++) {
                mylog<<landmarks[i].z<<";";
            }
            mylog<<"]";
            mylog<<" yaw:[";
            for (unsigned int i=0;i<landmarks.size();i++) {
                mylog<<landmarks[i].roll<<";";
            }
            mylog<<"]";
            mylog<<" pitch:[";
            for (unsigned int i=0;i<landmarks.size();i++) {
                mylog<<landmarks[i].pitch<<";";
            }
            mylog<<"]";
            mylog<<" roll:[";
            for (unsigned int i=0;i<landmarks.size();i++) {
                mylog<<landmarks[i].yaw<<";";
            }
            mylog<<"]";
//            for (int i=0;i<landmarks.size();i++) {
//            	Landmark3D lm = landmarks[i];
//                mylog
//            	/* lm_x, lm_y, lm_z, lm_yaw, lm_pitch, lm_roll */
//            										  << " lm_id:" <<lm.id
//            										  << " lm_x:" <<lm.x
//            										  << " lm_y:" << lm.y
//            										  << " lm_z:" << lm.z
//            										  << " lm_yaw:" << lm.roll
//            										  << " lm_pitch:" << lm.pitch
//                                                      << " lm_roll:" << lm.yaw;
//            }
//            /* detected obstacles */
//            for (int i=0;i<obstacle_processor.detectedObstacles.size();i++) {
//            	Obstacle2D* obstacle = obstacle_processor.detectedObstacles[i];
//            	if (obstacle->type == CIRCLE)
//					{
//            		Circle* obs_c = dynamic_cast<Circle*> (obstacle);
//                    mylog
//            	   /* obstacle center */
//                                << " pole_center:" << i << " " << obs_c->center.x << " " << obs_c->center.y;
//                }
//            }
            mylog << std::endl;
    }

}
#endif // DRONE_OBSTACLES_LOGGING



