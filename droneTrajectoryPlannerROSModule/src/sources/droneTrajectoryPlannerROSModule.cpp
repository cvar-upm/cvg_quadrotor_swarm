//////////////////////////////////////////////////////
//  DroneTrajectoryPlannerROSModule.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 27, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneTrajectoryPlannerROSModule.h"



using namespace std;




DroneTrajectoryPlannerROSModule::DroneTrajectoryPlannerROSModule() : DroneModule(droneModule::active,FREQ_TRAJ_PLANNER)
{

    return;
}


DroneTrajectoryPlannerROSModule::~DroneTrajectoryPlannerROSModule()
{
	close();
	return;
}


bool DroneTrajectoryPlannerROSModule::init()
{
    DroneModule::init();

    //flagTrajectoryFound=false;

    //end
    return true;
}


void DroneTrajectoryPlannerROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
	//Node
    DroneModule::open(nIn,moduleName);

    //Init
    if(!init())
        cout<<"Error init"<<endl;
	
	//End
	return;
}


void DroneTrajectoryPlannerROSModule::close()
{
    DroneModule::close();

    //Logging
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
    //log
    mylog.close();
#endif // DRONE_TRAJECTORY_PLANNER_LOGGING

    MyDroneTrajectoryPlanner->close();

    return;
}


bool DroneTrajectoryPlannerROSModule::resetValues()
{
    if(!MyDroneTrajectoryPlanner->reset())
        return false;

    //flagTrajectoryFound=false;

    if(!publishTrajectory())
        return false;

    return true;
}


bool DroneTrajectoryPlannerROSModule::startVal()
{
    if(!MyDroneTrajectoryPlanner->start())
        return false;

    //flagTrajectoryFound=false;

    //End
    return DroneModule::startVal();
}


bool DroneTrajectoryPlannerROSModule::stopVal()
{
    if(!MyDroneTrajectoryPlanner->stop())
        return false;

    return DroneModule::stopVal();
}


bool DroneTrajectoryPlannerROSModule::run()
{
    if(!DroneModule::run())
        return false;

    if(droneModuleOpened==false)
        return false;

    //Find Trajectory
    //flagTrajectoryFound=false;
    if(!MyDroneTrajectoryPlanner->run(trajectoryPlannerResult))
    {
#ifdef _VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP-ROS] Error finding trajectory"<<endl;
#endif
        return false;
    }

#ifdef _VERBOSE_DRONE_TRAJECTORY_PLANNER
    switch(trajectoryPlannerResult)
    {
    case TrajectoryPlanner::Result::NEW_TRAJECTORY_FOUND:
        cout<<"[DTP-ROS] New trajectory found"<<endl;
        break;
    case TrajectoryPlanner::Result::NO_NEED_FOR_NEW_TRAJECTORY:
        cout<<"[DTP-ROS] No need for new trajectory"<<endl;
        break;
    case TrajectoryPlanner::Result::UNABLE_TO_FIND_TRAJECTORY:
        cout<<"[DTP-ROS] Unable to find trajectory"<<endl;
        break;
    case TrajectoryPlanner::Result::TRAJECTORY_PARAMETERS_NOT_DEFINED:
        cout<<"[DTP-ROS] Trajectory parameters not defined"<<endl;
        break;
    case TrajectoryPlanner::Result::ERROR_FINDING_TRAJECTORY:
        cout<<"[DTP-ROS] Error finding trajectory"<<endl;
        break;
    case TrajectoryPlanner::Result::ERROR_SETTING_POINT_INIT:
        cout<<"[DTP-ROS] Error setting point init"<<endl;
        break;
    case TrajectoryPlanner::Result::ERROR_SETTING_POINT_FIN:
        cout<<"[DTP-ROS] Error setting point fin"<<endl;
        break;
    case TrajectoryPlanner::Result::ERROR_RECOVERING_TRAJECTORY:
        cout<<"[DTP-ROS] Error recovering trajectory"<<endl;
        break;
    default:
        cout<<"[DTP-ROS] nothing"<<endl;
        break;

    }

#endif


    //Publish trajectory if new
    if(trajectoryPlannerResult==TrajectoryPlanner::Result::NEW_TRAJECTORY_FOUND)
        if(!publishTrajectory())
            return false;

    return true;
}


TrajectoryPlanner::Result DroneTrajectoryPlannerROSModule::resultTrajectoryPlanner()
{
    return trajectoryPlannerResult;
}


int DroneTrajectoryPlannerROSModule::getTrajectory(std::vector<std::vector<double> > &trajectoryOut)
{
    if(!MyDroneTrajectoryPlanner->getTrajectory(trajectoryOut))
        return 0;

    return 1;
}


bool DroneTrajectoryPlannerROSModule::publishTrajectory()
{
    if(droneModuleOpened==false)
        return false;


    return false;
}









DroneTrajectoryPlanner2dROSModule::DroneTrajectoryPlanner2dROSModule() : DroneTrajectoryPlannerROSModule()
{
    //Polymorphism
    DroneTrajectoryPlanner2d* MyDroneTrajectoryPlanner2d=new DroneTrajectoryPlanner2d;
    MyDroneTrajectoryPlanner=static_cast<DroneTrajectoryPlanner2d*>(MyDroneTrajectoryPlanner2d);

    return;
}



DroneTrajectoryPlanner2dROSModule::~DroneTrajectoryPlanner2dROSModule()
{
    //Polymorphism
    DroneTrajectoryPlanner2d* MyDroneTrajectoryPlanner2d=static_cast<DroneTrajectoryPlanner2d*>(MyDroneTrajectoryPlanner);
    delete MyDroneTrajectoryPlanner2d;

    return;
}


void DroneTrajectoryPlanner2dROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{



    //Node
    DroneModule::open(nIn,moduleName);

    //Init
    if(!init())
        cout<<"Error init"<<endl;


    //DroneTrajectoryPlanner
    //Setting DroneTrajectoryPlanner
#ifndef DRONE_TRAJECTORY_PLANNER_SAVE_LOGS
    MyDroneTrajectoryPlanner->configure(false);
#else
    MyDroneTrajectoryPlanner->configure(true);
#endif
    //Open
    string configFile=stackPath+"configs/drone"+stringId+"/configFile.xml";

#ifndef DRONE_TRAJECTORY_PLANNER_SAVE_LOGS
    vector<string> prmFiles={stackPath+"configs/drone"+stringId+"/nodesList.dat",stackPath+"configs/drone"+stringId+"/nodesRelation.dat"};
    MyDroneTrajectoryPlanner->open(idDrone,configFile,prmFiles);
#else
    vector<string> prmFiles={stackPath+"configs/drone"+stringId+"/nodesList.dat",stackPath+"configs/drone"+stringId+"/nodesRelation.dat"};
    vector<string> logFiles={stackPath+"logs/drone"+stringId+"/nodesList.dat",stackPath+"logs/drone"+stringId+"/nodesRelation.dat",
                             stackPath+"logs/drone"+stringId+"/solutionNodes.dat",stackPath+"logs/drone"+stringId+"/simplifSolNodes.dat"};
    MyDroneTrajectoryPlanner->open(idDrone,configFile,prmFiles,logFiles);
#endif

    //Logging
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
    mylog.open(stackPath+"logs/trajectoryPlannerLog.dat");
    run_timestamp  = ros::Duration(0.0);
    cout<<"loging in: "<<stackPath+"logs/trajectoryPlannerLog.dat"<<endl;
#endif // DRONE_TRAJECTORY_PLANNER_LOGGING



    //// Topics ///
    //////Publisers
    //Trajectory
    dronePositionTrajectoryRefCommandPubl = n.advertise<droneMsgsROS::dronePositionTrajectoryRefCommand>(DRONE_TRAJECTORY_PLANNER_TRAJ_REF_COM, 1, true);

    //////subscribers
    //drone pose
    // rostopic pub -1 /drone0/ArucoSlam_EstimatedPose droneMsgsROS/dronePose -- 1.0 0.0 0.0 0.0 0.0 0.0 0.0 "a" "b" "c"
    dronePoseSubs = n.subscribe(DRONE_TRAJECTORY_PLANNER_POSE_SUBSCRIPTION, 1, &DroneTrajectoryPlanner2dROSModule::dronePoseCallback, this);

    //drone ref
    // rostopic pub -1 /drone0/droneMissionPoint droneMsgsROS/dronePositionRefCommand -- 2.0 2.0 1.25
    dronePositionPointFinRefCommandSubs = n.subscribe(DRONE_TRAJECTORY_PLANNER_MISSION_POINT_REF, 1, &DroneTrajectoryPlanner2dROSModule::dronePositionPointFinRefCommandCallback, this);

    //Obstacles
    //rostopic pub -1 /drone0/obstacles
    obstaclesSubs = n.subscribe(DRONE_TRAJECTORY_PLANNER_OBSTACLE_LIST, 1, &DroneTrajectoryPlanner2dROSModule::obstaclesCallback, this);

    //Society
    //rostopic pub -1 /drone0/societyPose droneMsgsROS/societyPose -- [2 '[1.0 0.0 0.0 0.0 0.0 0.0 0.0 "a" "b" "c"]']
    //rostopic pub -1 /drone0/societyPose droneMsgsROS/societyPose -- " {societyDrone: {id:2, pose:[1.0 0.0 0.0 0.0 0.0 0.0 0.0 "a" "b" "c"]' }  } "
    //rostopic pub -1 /drone0/societyPose droneMsgsROS/societyPose -- " {societyDrone: [ {} ]  } "    //Funciona, vacio
    //rostopic pub -1 /drone0/societyPose droneMsgsROS/societyPose -- " {societyDrone: [ {  }, {} ]  } " //Funciona, dos elementos vacios
    societyPoseSubs=n.subscribe(DRONE_TRAJECTORY_PLANNER_SOCIETY_POSE, 1, &DroneTrajectoryPlanner2dROSModule::societyPoseCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //End
    return;
}


bool DroneTrajectoryPlanner2dROSModule::publishTrajectory()
{
    if(droneModuleOpened==false)
        return 0;


    std::vector< std::vector<double> > trajectory;
    if(!MyDroneTrajectoryPlanner->getTrajectory(trajectory))
        return false;




    droneMsgsROS::dronePositionRefCommand dronePositionRefCommandAux;
    dronePositionRefCommandAux.x=0.0;
    dronePositionRefCommandAux.y=0.0;

    //The z is sent using the point Fin
    //dronePositionRefCommandAux.z=1.15; //JL TODO change this!!!
    dronePositionRefCommandAux.z=pointFin.at(2);

    //droneRefCommandAux.yaw=0.0;


    dronePositionTrajectoryRefCommandMsg.droneTrajectory.clear();


    //Logging
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
    run_timestamp = ros::Time::now() - ros::Time(0,0);
    std::ostringstream xmat_trajectoryLogMsgStrm;
    std::ostringstream ymat_trajectoryLogMsgStrm;
    std::ostringstream zmat_trajectoryLogMsgStrm;
    if(mylog.is_open())
    {
        mylog<<run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec<<" ";
        mylog<<"[trajectoryPlanner;droneTrajectory] ";

        xmat_trajectoryLogMsgStrm.str(std::string());
        xmat_trajectoryLogMsgStrm << "[";

        ymat_trajectoryLogMsgStrm.str(std::string());
        ymat_trajectoryLogMsgStrm << "[";

        zmat_trajectoryLogMsgStrm.str(std::string());
        zmat_trajectoryLogMsgStrm << "[";
    }
#endif

    //cout<<"Trajectory:"<<endl;
    for(unsigned int i=0;i<trajectory.size();i++)
    {
        /*
        for(unsigned int j=0;j<trajectory[i].size();j++)
        {
            cout<<trajectory[i][j]<<" ";
        }
        cout<<endl;
        */

        dronePositionRefCommandAux.x=trajectory[i][0];
        dronePositionRefCommandAux.y=trajectory[i][1];



        //Logging
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
        if(mylog.is_open())
        {
            xmat_trajectoryLogMsgStrm << dronePositionRefCommandAux.x;
            ymat_trajectoryLogMsgStrm << dronePositionRefCommandAux.y;
            zmat_trajectoryLogMsgStrm << dronePositionRefCommandAux.z;
            xmat_trajectoryLogMsgStrm<<";";
            ymat_trajectoryLogMsgStrm<<";";
            zmat_trajectoryLogMsgStrm<<";";
        }
#endif



        dronePositionTrajectoryRefCommandMsg.droneTrajectory.push_back(dronePositionRefCommandAux);
    }
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
        if(mylog.is_open())
        {
            xmat_trajectoryLogMsgStrm<<"]";
            ymat_trajectoryLogMsgStrm<<"]";
            zmat_trajectoryLogMsgStrm<<"]";
        }
#endif


    //worldTriDimTrajectoryMsg.time=ros::Time::now().toSec();


    //Header
    dronePositionTrajectoryRefCommandMsg.header.stamp=ros::Time::now();


    //Other stuff
    dronePositionTrajectoryRefCommandMsg.is_periodic=false;
    dronePositionTrajectoryRefCommandMsg.initial_checkpoint=1;


    //Logging
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
    if(mylog.is_open())
    {
        mylog<<"x:"<<xmat_trajectoryLogMsgStrm.str()<<" ";
        mylog<<"y:"<<ymat_trajectoryLogMsgStrm.str()<<" ";
        mylog<<"z:"<<zmat_trajectoryLogMsgStrm.str()<<" ";
        int is_periodic_int = dronePositionTrajectoryRefCommandMsg.is_periodic ? 1 : 0;
        mylog<<"is_periodic:"<<is_periodic_int<<" ";
        mylog<<"initial_checkpoint:"<<dronePositionTrajectoryRefCommandMsg.initial_checkpoint<<"";
        mylog<<endl;
    }
#endif


    dronePositionTrajectoryRefCommandPubl.publish(dronePositionTrajectoryRefCommandMsg);


    return true;
}


void DroneTrajectoryPlanner2dROSModule::dronePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg)
{
    //Point Init
    //Using only information of the x and y!!
    std::vector<double> pointInit(2);
    pointInit[0]=msg->x;
    pointInit[1]=msg->y;

    if(!MyDroneTrajectoryPlanner->setPointInit(pointInit))
        return;


    //Logging
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
    run_timestamp = ros::Time::now() - ros::Time(0,0);
    if(mylog.is_open())
    {
        mylog<< run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec <<" ";
        mylog<<"[trajectoryPlanner;dronePose] ";
        mylog<<"x:"<<msg->x<<" ";
        mylog<<"y:"<<msg->y<<" ";
        mylog<<"z:"<<msg->z<<" ";
        mylog<<"yaw:"<<msg->yaw<<" ";
        mylog<<"pitch:"<<msg->pitch<<" ";
        mylog<<"roll:"<<msg->roll<<"";
        mylog<<endl;
    }
#endif



    return;
}


void DroneTrajectoryPlanner2dROSModule::dronePositionPointFinRefCommandCallback(const droneMsgsROS::dronePositionRefCommand::ConstPtr& msg)
{
    //Point Init
    //Using only information of the x and y!!
    pointFin.resize(3);
    pointFin[0]=msg->x;
    pointFin[1]=msg->y;
    //We store the z
    pointFin[2]=msg->z;



    //Introduce the point in trajectory planner
    std::vector<double> pointFinIn(2);
    for(unsigned int i=0;i<2;i++)
        pointFinIn[i]=pointFin[i];

    if(!MyDroneTrajectoryPlanner->setPointFin(pointFinIn))
        return;



    //Logging
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
    run_timestamp = ros::Time::now() - ros::Time(0,0);
    if(mylog.is_open())
    {
        mylog<<run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec<<" ";
        mylog<<"[trajectoryPlanner;dronePointFinRef] ";
        mylog<<"x:"<<msg->x<<" ";
        mylog<<"y:"<<msg->y<<" ";
        mylog<<"z:"<<msg->z<<"";
        mylog<<endl;
    }
#endif

    return;
}


void DroneTrajectoryPlanner2dROSModule::obstaclesCallback(const droneMsgsROS::obstaclesTwoDim::ConstPtr& msg)
{
    //Set
    droneMsgsROS::obstaclesTwoDim obstaclesMsg; //Message
    //obstaclesMsg.time=msg->time;
    obstaclesMsg.walls=msg->walls;
    obstaclesMsg.poles=msg->poles;


    //Set obstacles to drone trajectory planner
    //Aux vars
    unsigned int idObstacle;
    std::vector<double> centerPointIn(2);
    std::vector<double> radiusIn(2);
    std::vector<double> sizeIn(2);
    double yawAngleIn;

    //Ellipses
    std::vector<EllipseObstacle2d> EllipsesIn;

    for(unsigned int i=0;i<obstaclesMsg.poles.size();i++)
    {
        EllipseObstacle2d OneEllipse;
        idObstacle=obstaclesMsg.poles[i].id;
        centerPointIn[0]=obstaclesMsg.poles[i].centerX;
        centerPointIn[1]=obstaclesMsg.poles[i].centerY;
        radiusIn[0]=obstaclesMsg.poles[i].radiusX;
        radiusIn[0]=obstaclesMsg.poles[i].radiusY;
        yawAngleIn=obstaclesMsg.poles[i].yawAngle;

        OneEllipse.define(idObstacle,centerPointIn,radiusIn,yawAngleIn);

        EllipsesIn.push_back(OneEllipse);
    }


    //Rectangles
    std::vector<RectangleObstacle2d> RectanglesIn;

    for(unsigned int i=0;i<obstaclesMsg.walls.size();i++)
    {
        RectangleObstacle2d OneRectangle;
        idObstacle=obstaclesMsg.walls[i].id;
        centerPointIn[0]=obstaclesMsg.walls[i].centerX;
        centerPointIn[1]=obstaclesMsg.walls[i].centerY;
        sizeIn[0]=obstaclesMsg.walls[i].sizeX;
        sizeIn[0]=obstaclesMsg.walls[i].sizeY;
        yawAngleIn=obstaclesMsg.walls[i].yawAngle;

        OneRectangle.define(idObstacle,centerPointIn,sizeIn,yawAngleIn);

        RectanglesIn.push_back(OneRectangle);
    }


    DroneTrajectoryPlanner2d* MyDroneTrajectoryPlanner2d=static_cast<DroneTrajectoryPlanner2d*>(MyDroneTrajectoryPlanner);
    if(!MyDroneTrajectoryPlanner2d->setObstacles(EllipsesIn,RectanglesIn))
        return;



    //Logging
#ifdef DRONE_TRAJECTORY_PLANNER_LOGGING
    run_timestamp = ros::Time::now() - ros::Time(0,0);
    if(mylog.is_open())
    {
        mylog<<run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec<<" ";
        mylog<<"[trajectoryPlanner;obstacles] ";
        //obstaclesMsg.walls
        mylog<<"walls_id:[";
        for (unsigned int i=0; i<obstaclesMsg.walls.size(); i++) {
            mylog<<obstaclesMsg.walls[i].id<<";";
        }
        mylog<<"] ";
        mylog<<"walls_center:[";
        for (unsigned int i=0; i<obstaclesMsg.walls.size(); i++) {
            mylog<<obstaclesMsg.walls[i].centerX<<",";
            mylog<<obstaclesMsg.walls[i].centerY<<";";
        }
        mylog<<"] ";

        mylog<<"walls_size:[";
        for (unsigned int i=0; i<obstaclesMsg.walls.size(); i++) {
            mylog<<obstaclesMsg.walls[i].sizeX<<",";
            mylog<<obstaclesMsg.walls[i].sizeY<<";";
        }
        mylog<<"] ";

        mylog<<"walls_yaw:[";
        for (unsigned int i=0; i<obstaclesMsg.walls.size(); i++) {
            mylog<<obstaclesMsg.walls[i].yawAngle<<";";
        }
        mylog<<"] ";

        // obstaclesMsg.poles
        mylog<<"poles_id:[";
        for (unsigned int i=0; i<obstaclesMsg.poles.size(); i++) {
            mylog<<obstaclesMsg.poles[i].id<<";";
        }
        mylog<<"] ";
        mylog<<"poles_center:[";
        for (unsigned int i=0; i<obstaclesMsg.poles.size(); i++) {
            mylog<<obstaclesMsg.poles[i].centerX<<",";
            mylog<<obstaclesMsg.poles[i].centerY<<";";
        }
        mylog<<"] ";

        mylog<<"poles_size:[";
        for (unsigned int i=0; i<obstaclesMsg.poles.size(); i++) {
            mylog<<obstaclesMsg.poles[i].radiusX<<",";
            mylog<<obstaclesMsg.poles[i].radiusY<<";";
        }
        mylog<<"] ";

        mylog<<"poles_yaw:[";
        for (unsigned int i=0; i<obstaclesMsg.poles.size(); i++) {
            mylog<<obstaclesMsg.poles[i].yawAngle<<";";
        }
        mylog<<"]";
        mylog<<endl;
    }
#endif


    return;
}


void DroneTrajectoryPlanner2dROSModule::societyPoseCallback(const droneMsgsROS::societyPose::ConstPtr& msg)
{

    //Vars
    //Society pose
    unsigned int idRobot;
    vector<double> centerRobot(2);
    vector<double> sizeRobot={OTHERS_QR_RADIUS, OTHERS_QR_RADIUS};
    double yawAngleRobot;
    std::vector<Robot2d> SocietyPose;


    //Set society pose to drone Trajectory Planner
    for(unsigned int i=0;i<msg->societyDrone.size();i++)
    {
        Robot2d OneRobot;
        idRobot=msg->societyDrone[i].id;
        centerRobot[0]=msg->societyDrone[i].pose.x;
        centerRobot[1]=msg->societyDrone[i].pose.y;
        yawAngleRobot=msg->societyDrone[i].pose.yaw;

        OneRobot.setRobotParameters(idRobot,centerRobot,sizeRobot,yawAngleRobot);
        SocietyPose.push_back(OneRobot);
    }


    DroneTrajectoryPlanner2d* MyDroneTrajectoryPlanner2d=static_cast<DroneTrajectoryPlanner2d*>(MyDroneTrajectoryPlanner);
    if(!MyDroneTrajectoryPlanner2d->setSocietyPose(SocietyPose))
        return;

/*
    //Logging
    mylog<<run_timestamp.sec << "." << std::setfill('0') << std::setw(9) << run_timestamp.nsec<<" ";
    mylog<<"[trajectoryPlanner;societyPose] ";

    mylog<<"walls_id:[";
    for (unsigned int i=0; i<obstaclesMsg.walls.size(); i++) {
        mylog<<obstaclesMsg.walls[i].id<<";";
    }
    mylog<<"] ";
    mylog<<"walls_center:[";
    for (unsigned int i=0; i<obstaclesMsg.walls.size(); i++) {
        mylog<<obstaclesMsg.walls[i].centerX<<",";
        mylog<<obstaclesMsg.walls[i].centerY<<";";
    }
    mylog<<"] ";

    mylog<<"walls_size:[";
    for (unsigned int i=0; i<obstaclesMsg.walls.size(); i++) {
        mylog<<obstaclesMsg.walls[i].sizeX<<",";
        mylog<<obstaclesMsg.walls[i].sizeY<<";";
    }
    mylog<<"] ";

    mylog<<"walls_yaw:[";
    for (unsigned int i=0; i<obstaclesMsg.walls.size(); i++) {
        mylog<<obstaclesMsg.walls[i].yawAngle<<";";
    }
    mylog<<"] ";
    */


    return;
}

