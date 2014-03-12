//////////////////////////////////////////////////////
//  droneArucoEyeROSModule.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Jan 15, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneArucoEyeROSModule.h"



using namespace std;


////////// Drone Aruco Eye ///////////
DroneArucoEyeROSModule::DroneArucoEyeROSModule(): DroneModule(droneModule::active,DRONE_ARUCO_EYE_RATE)
{
    init();
    return;
}


DroneArucoEyeROSModule::~DroneArucoEyeROSModule()
{
    close();
    return;
}

void DroneArucoEyeROSModule::init()
{
//    MyDroneArucoEye.init(); // corrected double initialization

    //subscribe_to_rectified_front_cam=false;

    droneArucoListMsg.time=ros::Time::now().toSec();

    return;
}


void DroneArucoEyeROSModule::close()
{
    if(!MyDroneArucoEye.close())
        return;

    return;
}


void DroneArucoEyeROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //configure droneArucoEye
    if(!MyDroneArucoEye.configureArucoEye(stackPath+"configs/drone"+stringId+"/arUcoList.xml",stackPath+"configs/drone"+stringId+"/ardrone_front.yaml"))
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
        cout<<"[DAE-ROS] Error configuring Aruco Eye"<<endl;
#endif
        return;
    }

    //Configure camera in the drone
    if(!MyDroneArucoEye.setCameraInTheDrone(stackPath+"configs/drone"+stringId+"/droneConfiguration.xml",0))
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
        cout<<"[DAE-ROS] Error setting camera in the drone!!!!"<<endl;
#endif
        return;
    }
//    if(!MyDroneArucoEye.setCameraInTheDrone(MULTIROTOR_FRONTCAM_POSITION_DRONE_GMR_X,MULTIROTOR_FRONTCAM_POSITION_DRONE_GMR_Y,MULTIROTOR_FRONTCAM_POSITION_DRONE_GMR_Z,
//                                            (M_PI/180.0)*(-90.0),0.0,(M_PI/180.0)*(-90.0+MULTIROTOR_FRONTCAM_POSITION_DRONE_GMR_PITCH)))
//        return;

    //Subscriber to image
//    if(subscribe_to_rectified_front_cam)
//        droneFrontImageSubs = n.subscribe(DRONE_ARUCO_EYE_FRONT_IMAGE_RECT, 1, &DroneArucoEyeROSModule::droneImageCallback, this);
//    else
        droneFrontImageSubs = n.subscribe(DRONE_ARUCO_EYE_FRONT_IMAGE_RAW, 1, &DroneArucoEyeROSModule::droneImageCallback, this);

    //Publisher aruco 3D pose
    droneArucoListPubl = n.advertise<droneMsgsROS::obsVector>(DRONE_ARUCO_EYE_OBSERVATIONVEC_LIST, 1, true);

    //Flag of module opened
    droneModuleOpened=true;

    //Autostart the module
    //moduleStarted=true; //JL to remove!

    //End
    return;
}


void DroneArucoEyeROSModule::droneImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //Transform message to Opencv
    try
    {
        cvDroneImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
        ROS_ERROR("cv_bridge exception: %s", e.what());
#endif
        return;
    }

    droneImage=cvDroneImage->image;
//    frontImage_timesamp = msg->header.stamp;
//    frontImage_seq = msg->header.seq;

    //Set image to aruco retina
    if(!MyDroneArucoEye.setInputImage(droneImage))
        return;

    return;
}



//Reset
bool DroneArucoEyeROSModule::resetValues()
{
    if(!DroneModule::resetValues())
        return false;

    if(!MyDroneArucoEye.reset())
        return false;

    return true;
}

//Start
bool DroneArucoEyeROSModule::startVal()
{
    if(!DroneModule::startVal())
        return false;

    if(!MyDroneArucoEye.start())
        return false;

    return true;
}

//Stop
bool DroneArucoEyeROSModule::stopVal()
{
    if(!DroneModule::stopVal())
        return false;

    if(!MyDroneArucoEye.stop())
        return false;

    return true;
}


//Run
bool DroneArucoEyeROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    //Run aruco retina
    unsigned int numCodesReconstructed;
    if(!MyDroneArucoEye.run(numCodesReconstructed))
        return false;
#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
    cout<<"[DAE-ROS] numCodesReconstructed="<<numCodesReconstructed<<endl;
#endif


    //Prepare message
    droneArucoListMsg.obs.clear();
    //Time
    droneArucoListMsg.time=ros::Time::now().toSec();
    droneArucoListMsg.YPR_system      = "wYvPuR";
    droneArucoListMsg.target_frame    = "aruco_GMR";
    droneArucoListMsg.reference_frame = "drone_GMR";


    //Get codes reconstructed
    int idMarker;
    cv::Mat matHomog_aruco_GMR_wrt_drone_GMR;
    for(unsigned int i=0; i<numCodesReconstructed;i++)
    {
        if(!MyDroneArucoEye.getDroneMarkerI(i,idMarker,matHomog_aruco_GMR_wrt_drone_GMR))
        {
            continue;
        }

        double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0;
        referenceFrames::getxyzYPRfromHomogMatrix_wYvPuR( matHomog_aruco_GMR_wrt_drone_GMR, &x, &y, &z, &yaw, &pitch, &roll);

#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
        cout<<"[DAE-ROS] Marker id="<<idMarker<<"; HomogTransInWorld="<<matHomog_aruco_GMR_wrt_drone_GMR<<endl;

        std::cout<<"[DAE-ROS]  Homog_aruco_GMR_wrt_drone_GMR_wYvPuR =\n"<<
                   "    x = "   << x   << " y = "     << y     << " z = "   << z    << endl <<
                   "    yaw = " << yaw*(180.0/M_PI) << " pitch = " << pitch*(180.0/M_PI) << " roll = "<< roll*(180.0/M_PI) << endl;
#endif

        //message
        droneArucoListMsg.obs.push_back( droneMsgsROS::Observation3D() );
        droneArucoListMsg.obs[i].id = idMarker;
        droneArucoListMsg.obs[i].x = x;
        droneArucoListMsg.obs[i].y = y;
        droneArucoListMsg.obs[i].z = z;
        droneArucoListMsg.obs[i].yaw   = yaw;
        droneArucoListMsg.obs[i].pitch = pitch;
        droneArucoListMsg.obs[i].roll  = roll;
    }

    //Publish
    if(!publishArucoList())
        return false;

    //end
    return true;
}



bool DroneArucoEyeROSModule::publishArucoList()
{
    if(droneModuleOpened==false)
        return 0;

    //publish
    droneArucoListPubl.publish(droneArucoListMsg);

    //end
    return 1;
}



int DroneArucoEyeROSModule::drawArucoCodes(std::string windowName, int waitingTime, bool drawDetectedCodes, bool draw3DReconstructedCodes)
{
    return MyDroneArucoEye.drawArucoCodes(windowName,waitingTime,drawDetectedCodes,draw3DReconstructedCodes);
}
