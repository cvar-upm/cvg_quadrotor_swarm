//////////////////////////////////////////////////////
//  droneTrajectoryPlanner.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneTrajectoryPlanner.h"



using namespace std;




DroneTrajectoryPlanner::DroneTrajectoryPlanner()
{
    init();
    return;
}



DroneTrajectoryPlanner::~DroneTrajectoryPlanner()
{
    clear();
	return;
}


bool DroneTrajectoryPlanner::init()
{

    //Flags
    newPointInit=false;
    newPointFin=false;
    newObstacles=false;
    newSocietyPose=false;

    flagFindTrajectory=false;


    //IdDrone
    idDrone=0;


    //trajectory
    trajectory.clear();



    //Default Settings
    flagLogging=false;



    //end
    return true;
}

bool DroneTrajectoryPlanner::clear()
{
    trajectory.clear();

    return true;
}

int DroneTrajectoryPlanner::configure(bool setLogs)
{
    flagLogging=setLogs;

    return 1;
}

int DroneTrajectoryPlanner::open(int idDroneIn, std::string configFileIn, std::vector<std::string> prmFilesIn)
{
    //Id Drone
    idDrone=idDroneIn;

    //Set files
    configFile=configFileIn;
    prmFiles=prmFilesIn;
    if(prmFiles.size()!=2)
        return 0;

    /////Trajectory planner
    //Init components
    if(!MyRobotTrajectoryPlanner->initComponents(configFile,prmFiles[0],prmFiles[1]))
    {
        return 0;
    }
#ifdef SAVE_PRM_DRONE_TRAJECTORY_PLANNER
    if(!MyRobotTrajectoryPlanner->savePRM(prmFiles[0],prmFiles[1]))
    {
        return 0;
    }
#endif
    //MAP fixed known obstacles
    if(!MyRobotTrajectoryPlanner->createMap(configFile))
    {
        return 0;
    }


    //End
    return 1;
}

int DroneTrajectoryPlanner::open(int idDroneIn, std::string configFileIn, std::vector<std::string> prmFilesIn, std::vector<std::string> logFilesIn)
{
    if(!open(idDroneIn,configFileIn,prmFilesIn))
        return 0;

    //set log files
    logFiles=logFilesIn;
    if(logFiles.size()!=4 && flagLogging)
    {
        return 0;
    }


    //Save logs
    if(flagLogging)
    {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP] Saving logs"<<endl;
#endif
        if(!MyRobotTrajectoryPlanner->savePRM(logFiles[0],logFiles[1]))
        {
            return 0;
        }
        if(!MyRobotTrajectoryPlanner->savePlannerResult(logFiles[2],logFiles[3]))
        {
            return 0;
        }
    }

	
	//End
    return 1;
}


int DroneTrajectoryPlanner::close()
{
    trajectory.clear();

    return 1;
}




int DroneTrajectoryPlanner::setPointInit(std::vector<double> pointInitIn)
{
    //flag
    newPointInit=true;

    //Using only information of the x and y!!
    pointInit.resize(pointInitIn.size());

    for(unsigned int i=0;i<pointInit.size();i++)
        pointInit[i]=pointInitIn[i];


    return 1;
}


int DroneTrajectoryPlanner::setPointFin(std::vector<double> pointFinIn)
{
    //flag
    newPointFin=true;


    //Using information of the x, y, z!!
    pointFin.resize(pointFinIn.size());

    for(unsigned int i=0;i<pointFin.size();i++)
        pointFin[i]=pointFinIn[i];


    return 1;
}







bool DroneTrajectoryPlanner::reset()
{

    trajectory.clear();


    return true;

}



bool DroneTrajectoryPlanner::start()
{

    trajectory.clear();


    //End
    return true;
}



bool DroneTrajectoryPlanner::stop()
{
    return true;
}



bool DroneTrajectoryPlanner::run(TrajectoryPlanner::Result &resultOut, bool flagReplanifyAlways)
{
    //resultOut=false;
    resultOut=TrajectoryPlanner::Result::ERROR_NOTHING_DONE;

#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP] Running"<<endl;
#endif

    //Check if points are defined!
    if(pointInit.size()==0 || pointFin.size()==0)
    {
        //cout<<" !No points defined"<<endl;
        resultOut=TrajectoryPlanner::Result::TRAJECTORY_PARAMETERS_NOT_DEFINED;
        return false;
    }


    //// Point fin
    if(newPointFin)
    {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP] New point fin: ";
        cout<<"[";
        for(unsigned int j=0;j<pointFin.size();j++)
        {
            cout<<pointFin[j];
            if(j!=pointFin.size()-1)
                cout<<" ";
        }
        cout<<"];"<<endl;
#endif

        //set
        if(!MyRobotTrajectoryPlanner->setPointFin(pointFin))
        {
            //cout<<" -error setting point fin in MyRobotTrajectoryPlanner";
            resultOut=TrajectoryPlanner::Result::ERROR_SETTING_POINT_FIN;
            return false;
        }
        //flags
        newPointFin=false;
        flagFindTrajectory=true;
    }


    /////Point init
    if(newPointInit)
    {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP] New point init: ";
        cout<<"[";
        for(unsigned int j=0;j<pointInit.size();j++)
        {
            cout<<pointInit[j];
            if(j!=pointInit.size()-1)
                cout<<" ";
        }
        cout<<"];"<<endl;
#endif

        //set
        if(!MyRobotTrajectoryPlanner->setPointInit(pointInit))
        {
            //cout<<" -error setting point init in MyRobotTrajectoryPlanner";
            resultOut=TrajectoryPlanner::Result::ERROR_SETTING_POINT_INIT;
            return false;
        }

        //flags
        newPointInit=false;
        flagFindTrajectory=true;
    }



    ////Obstacles
    if(newObstacles)
    {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP] New obstacles"<<endl;
#endif


        //Set obstacles
        //JL TODO
        //flags
        newObstacles=false;
        flagFindTrajectory=true;
    }



    ////Society Pose
    //JL: TODO
    if(newSocietyPose)
    {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP] New Society Pose"<<endl;
#endif


        //Set obstacles
        //JL TODO
        //flags
        newSocietyPose=false;
        flagFindTrajectory=true;
    }



    //Find trajectory
    if(flagFindTrajectory)
    {
        //////////////////////////////////////////
        //////Checking last trajectory if available
        if(trajectory.size()>0)
        {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
            cout<<"[DTP] -Checking last trajectory"<<endl;
#endif


            //Check if the point fin is the same. JL Not necesary??
            bool flagCorrectPointFin=false;
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
            cout<<"[DTP]  -Checking if target point is in last trajectory"<<endl;
#endif
            if(trajectory[trajectory.size()-1]!=pointFin)
            {
                //Flag
                flagCorrectPointFin=false;
                //Limpiamos trajectoria
                trajectory.clear();
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                cout<<"[DTP]   +Target point is not in trajectory. Need to replan!"<<endl;
#endif
            }
            else
            {
                //Flag
                flagCorrectPointFin=true;
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                cout<<"[DTP]   +Target point is in trajectory"<<endl;
#endif
            }



            //Check if the robot is still in the trajectory
            bool flagDroneInTheTrajectory=false;
            if(trajectory.size()>=2) //At least it has two points: init and final
            {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                cout<<"[DTP]  -Checking if the robot is still in the trajectory"<<endl;
#endif
                double minDistance=99999.9;
                for(unsigned int i=0;i<trajectory.size()-1;i++)
                {
                    double distance=0.0;
                    std::vector<double> pointProjected;
                    if(MyRobotTrajectoryPlanner->distancePointToRect(distance,pointProjected,pointInit,trajectory[i],trajectory[i+1]))
                    {
                        if(distance<minDistance)
                            minDistance=distance;
                    }
//                    else
//                    {
//                        cout<<"[DTP]   *Error calculating distances in trajectory"<<endl;
//                    }
                }

#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                if(minDistance==99999.9)
                    cout<<"[DTP]   *Error calculating distances in trajectory"<<endl;
#endif


                if(minDistance>PRECISSION_IN_TRAJECTORY)
                {
                    //Flag
                    flagDroneInTheTrajectory=false;
                    //Limpiamos trajectoria
                    trajectory.clear();
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                    cout<<"[DTP]   +Robot is not in trajectory. Need to replan!"<<endl;
                    cout<<"[DTP]    *Distance out of trajectory="<<minDistance<<endl;
#endif
                }
                else
                {
                    //Flag
                    flagDroneInTheTrajectory=true;
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                    cout<<"[DTP]   +Robot is in trajectory"<<endl;
#endif
                }
            }


            //Check if the last trajectory is collision free
            bool flagLastTrajectoryStillCollisionFree=false;
            if(trajectory.size()>=2) //At least it has two points: init and final
            {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                cout<<"[DTP]  -Checking if last trajectory is collision free"<<endl;
#endif

                //trajectory check
                if(!MyRobotTrajectoryPlanner->checkTrajectory(flagLastTrajectoryStillCollisionFree,trajectory,pointInit))
                //if(!MyRobotTrajectoryPlanner.checkTrajectory(flagLastTrajectoryStillCollisionFree,trajectory))
                {
                    //flag. pedimos que recalcule
                    flagLastTrajectoryStillCollisionFree=false;
                    //Limpiamos trajectoria
                    trajectory.clear();
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                    cout<<"[DTP]   +Error checking Trajectory is not collision free. Need to replan!"<<endl;
#endif
                }

                if(!flagLastTrajectoryStillCollisionFree)
                {
                    //Limpiamos trajectoria
                    trajectory.clear();
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                    cout<<"[DTP]   +Trajectory is not collision free. Need to replan!"<<endl;
#endif
                }
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                else
                {
                    cout<<"[DTP]   +Trajectory is collision free."<<endl;
                }
#endif
            }


            //checkings check
            if(!flagReplanifyAlways && flagCorrectPointFin && flagLastTrajectoryStillCollisionFree && flagDroneInTheTrajectory)
            {
                //Flag. Not needed
                flagFindTrajectory=false;
                //trajectoryFound=false; //To return ok
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                cout<<"[DTP] Trajectory doesn't need to be replanned"<<endl;
#endif
                //end

                resultOut=TrajectoryPlanner::Result::NO_NEED_FOR_NEW_TRAJECTORY;

                return true;
            }

            else
            {
                //Flag. Not needed
                flagFindTrajectory=true;

                if(trajectory.size()!=0)
                {
                    //Limpiamos trajectoria
                    trajectory.clear();
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                    cout<<"[DTP] Trajectory needs to be replanned"<<endl;
#endif

                    resultOut=TrajectoryPlanner::Result::NEW_TRAJECTORY_FOUND; //Trayectoria vacia

                    return true;
                }
                else
                {
                    //Limpiamos trajectoria
                    trajectory.clear();
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
                    cout<<"[DTP] Trajectory needs to be replanned. Now replanning."<<endl;
#endif
                }
            }
        }


        //////////////////////////////////////
        ///////Calculate trajectory if needed
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP] Planifying Trajectory."<<endl;
#endif

        bool resultOfFindTrajectory;
        if(!MyRobotTrajectoryPlanner->findTrajectory(resultOfFindTrajectory))
        {
            reset();
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
            cout<<"[DTP]  +Error while trying to find trajectory."<<endl;
#endif
            resultOut=TrajectoryPlanner::Result::ERROR_FINDING_TRAJECTORY;
            return false;
        }
        else
        {
            resultOut=TrajectoryPlanner::Result::NEW_TRAJECTORY_FOUND;

        }
        //cout<<"aqui"<<endl;

        if(resultOfFindTrajectory)
        {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
            cout<<"[DTP]  +Trajectory planed."<<endl;
#endif
            //Flag Trajectory found!
            flagFindTrajectory=false;
        }
        else
        {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
            cout<<"[DTP]  +Unable to Planify Trajectory."<<endl;
#endif
        }


        //Save trajectory
        if(!MyRobotTrajectoryPlanner->getTrajectory(trajectory))
        {
            reset();
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
            cout<<"[DTP] Error recovering Trajectory."<<endl;
#endif
            resultOut=TrajectoryPlanner::Result::ERROR_RECOVERING_TRAJECTORY;
            return false;
        }


        //Save logs
        if(flagLogging)
        {
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
            cout<<"[DTP] Saving logs"<<endl;
#endif
            if(!MyRobotTrajectoryPlanner->savePRM(logFiles[0],logFiles[1]))
            {
                resultOut=TrajectoryPlanner::Result::ERROR_FINDING_TRAJECTORY;
                return false;
            }
            if(!MyRobotTrajectoryPlanner->savePlannerResult(logFiles[2],logFiles[3]))
            {
                resultOut=TrajectoryPlanner::Result::ERROR_FINDING_TRAJECTORY;
                return false;
            }
        }



        return true;
    }



    //No need to do anything. same points
    //trajectoryFound=false;
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        cout<<"[DTP] It's not necesary to replan. Nothing has changed."<<endl;
#endif

    resultOut=TrajectoryPlanner::Result::NO_NEED_FOR_NEW_TRAJECTORY;

    return true;
}


int DroneTrajectoryPlanner::getTrajectory(std::vector< std::vector<double> > &trajectoryOut)
{
    trajectoryOut.clear();
    std::vector<double> pointOfTrajectory;
    for(unsigned int i=0;i<trajectory.size();i++)
    {
        pointOfTrajectory.resize(trajectory[i].size());
        for(unsigned j=0;j<trajectory[i].size();j++)
        {
            pointOfTrajectory[j]=trajectory[i][j];
        }
        trajectoryOut.push_back(pointOfTrajectory);
    }

    return 1;
}






DroneTrajectoryPlanner2d::DroneTrajectoryPlanner2d()
{
    //Polymorphism
    RobotTrajectoryPlanner2d* MyRobotTrajectoryPlanner2d=new RobotTrajectoryPlanner2d;
    MyRobotTrajectoryPlanner=static_cast<RobotTrajectoryPlanner2d*>(MyRobotTrajectoryPlanner2d);

    return;
}

DroneTrajectoryPlanner2d::~DroneTrajectoryPlanner2d()
{
    //Be tidy
    RobotTrajectoryPlanner2d* MyRobotTrajectoryPlanner2d=static_cast<RobotTrajectoryPlanner2d*>(MyRobotTrajectoryPlanner);
    delete MyRobotTrajectoryPlanner2d;

    return;
}


int DroneTrajectoryPlanner2d::setObstacles(std::vector<EllipseObstacle2d> EllipsesIn, std::vector<RectangleObstacle2d> RectanglesIn)
{
    //flag
    newObstacles=true;

    std::vector<EllipseObstacle2d> Ellipses=EllipsesIn;
    std::vector<RectangleObstacle2d> Rectangles=RectanglesIn;



    //Borramos los obstaculos dinamicos que ya no se han recibido
    for(unsigned int i=0;i<dynamicObstaclesAdded.size();i++)
    {
        //Lo borramos
        if(!MyRobotTrajectoryPlanner->deleteObstacle(dynamicObstaclesAdded[i]))
            return 0;

    }

    //Limpiamos
    dynamicObstaclesAdded.clear();




    //Aux vars
    unsigned int idObstacle;
    std::vector<double> centerPointIn(2);
    std::vector<double> radiusIn(2);
    std::vector<double> sizeIn(2);
    double yawAngleIn;

    //Needed because of polimorphysm
    RobotTrajectoryPlanner2d* MyRobotTrajectoryPlanner2d=static_cast<RobotTrajectoryPlanner2d*>(MyRobotTrajectoryPlanner);

    //Ellipses
    for(unsigned int i=0;i<Ellipses.size();i++)
    {
        Ellipses[i].getParameters(idObstacle,centerPointIn,radiusIn,yawAngleIn);

        if(!MyRobotTrajectoryPlanner2d->setEllipse(idObstacle,centerPointIn,radiusIn,yawAngleIn))
            return 0;


        dynamicObstaclesAdded.push_back(idObstacle);

    }

    //Rectangles
    for(unsigned int i=0;i<Rectangles.size();i++)
    {
        Rectangles[i].getParameters(idObstacle,centerPointIn,sizeIn,yawAngleIn);

        if(!MyRobotTrajectoryPlanner2d->setRectangle(idObstacle,centerPointIn,sizeIn,yawAngleIn))
            return 0;

        dynamicObstaclesAdded.push_back(idObstacle);
    }



    return 1;
}



int DroneTrajectoryPlanner2d::setSocietyPose(std::vector<Robot2d> societyIn)
{
    //flags
    newSocietyPose=true;

    societyIdsAdded.clear();

    //delete all robots received in the previous callback
    if(!MyRobotTrajectoryPlanner->clearQuadRotors())
    {
        //cout<<"error al limpiar drones!"<<endl;
        return 0;
    }


    unsigned int idDroneSoc;
    vector<double> centerPointDroneSoc(2);
    vector<double> sizeDimensionsDroneSoc(2);
    double yawAngleDroneSoc;


    //Needed because of polimorphysm
    RobotTrajectoryPlanner2d* MyRobotTrajectoryPlanner2d=static_cast<RobotTrajectoryPlanner2d*>(MyRobotTrajectoryPlanner);
    //Add robots received
    for(unsigned int i=0;i<societyIn.size();i++)
    {
        societyIn[i].getRobotParameters(idDroneSoc,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc);
        if(idDroneSoc==idDrone)
            continue;

        if(!MyRobotTrajectoryPlanner2d->setQuadRotor(idDroneSoc,centerPointDroneSoc,sizeDimensionsDroneSoc,yawAngleDroneSoc))
        {
            return 0;
        }
#ifdef VERBOSE_DRONE_TRAJECTORY_PLANNER
        else
        {
            cout<<"[DTP] (setSocietyPose) set drone "<<idDroneSoc<<". pos="<<centerPointDroneSoc[0]<<";"<<centerPointDroneSoc[1]<<". Size="<<sizeDimensionsDroneSoc[0]<<";"<<sizeDimensionsDroneSoc[1]<<endl;
        }
#endif

        societyIdsAdded.push_back(idDroneSoc);
    }

    return 1;

}
