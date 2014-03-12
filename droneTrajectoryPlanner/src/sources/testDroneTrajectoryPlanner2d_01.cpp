//////////////////////////////////////////////////////
//  testDroneTrajectoryPlanner2d_01.cpp
//
//  Created on: Oct 24, 2013
//      Author: joselusl
//
//  Last modification on: Oct 26, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


//I/O Stream
//std::cout
#include <iostream>

//Vector
//std::vector
#include <vector>

//Chrono
//
#include <chrono>



//drone trajectory planner
#include "droneTrajectoryPlanner.h"


using namespace std;

int main(void)
{

    cout<<"[TEST] Init test program"<<endl;


    //Class declaration
    DroneTrajectoryPlanner2d MyDroneTrajectoryPlanner;

    //bool trajectoryFound=false;

    TrajectoryPlanner::Result resultTrajectory;


    //Point init
    vector<double> pointInit(2);
    //Point fin
    vector<double> pointFin(2);
    //Obstacles
    //Aux vars
    unsigned int idObstacle;
    std::vector<double> centerPointObstacle(2);
    vector<double> sizeObstacle(2);
    vector<double> radiusObstacle(2);
    double yawAngleObstacle;
    //Ellipses
    std::vector<EllipseObstacle2d> Ellipses;
    EllipseObstacle2d OneEllipse;
    //Rectangles
    std::vector<RectangleObstacle2d> Rectangles;
    RectangleObstacle2d OneRectangle;
    //Society pose
    unsigned int idRobot;
    vector<double> centerRobot(2);
    vector<double> sizeRobot(2);
    double yawAngleRobot;
    std::vector<Robot2d> SocietyPose;
    Robot2d OneRobot;

    //out trajectory
    vector< vector<double> > trajectory;

    //Times
    std::chrono::time_point<std::chrono::system_clock> start, end;


    //Setting DroneTrajectoryPlanner
    MyDroneTrajectoryPlanner.configure(true);

    //Init DroneTrajectoryPlanner
    vector<string> prmFiles={"/home/joselu/workspace/ros/quadrotor/stack/configs/drone0/nodesList.dat","/home/joselu/workspace/ros/quadrotor/stack/configs/drone0/nodesRelation.dat"};
    vector<string> logFiles={"/home/joselu/workspace/ros/quadrotor/stack/logs/drone0/nodesList.dat","/home/joselu/workspace/ros/quadrotor/stack/logs/drone0/nodesRelation.dat","/home/joselu/workspace/ros/quadrotor/stack/logs/drone0/solutionNodes.dat","/home/joselu/workspace/ros/quadrotor/stack/logs/drone0/simplifSolNodes.dat"};
    if(!MyDroneTrajectoryPlanner.open(0,"/home/joselu/workspace/ros/quadrotor/stack/configs/drone0/configFile.xml",prmFiles,logFiles))
    {
        cout<<"[TEST] Unable to start Trajectory planner"<<endl;
        return 0;
    }
    else
        cout<<"[TEST] Trajectory planner started"<<endl;


    //////Simulation Loop
    char command;
    bool setPointInit=false;
    bool setPointFin=false;
    bool setObstacles=false;
    bool setSocietyPose=false;
    while(1)
    {
        //Set point init
        if(setPointInit)
        {
            //flag
            setPointInit=false;
            //Set point
            if(!MyDroneTrajectoryPlanner.setPointInit(pointInit))
            {
                cout<<"[TEST] Error setting point init"<<endl;
                return 0;
            }
            else
            {
                cout<<"[TEST] Point init set to: ";
                cout<<"[";
                for(unsigned int j=0;j<pointInit.size();j++)
                {
                    cout<<pointInit[j];
                    if(j!=pointInit.size()-1)
                        cout<<" ";
                }
                cout<<"];"<<endl;
            }
        }


        //Set point fin
        if(setPointFin)
        {
            //Flag
            setPointFin=false;
            if(!MyDroneTrajectoryPlanner.setPointFin(pointFin))
            {
                cout<<"[TEST] Error setting point fin"<<endl;
                return 0;
            }
            else
            {
                cout<<"[TEST] Point fin set to: ";
                cout<<"[";
                for(unsigned int j=0;j<pointFin.size();j++)
                {
                    cout<<pointFin[j];
                    if(j!=pointFin.size()-1)
                        cout<<" ";
                }
                cout<<"];"<<endl;
            }
        }

        //Set obstacles
        if(setObstacles)
        {
            //Flag
            setObstacles=false;
            if(!MyDroneTrajectoryPlanner.setObstacles(Ellipses,Rectangles))
            {
                cout<<"[TEST] Error setting obstacles"<<endl;
                return 0;
            }
            else
            {
                cout<<"[TEST] Obstacles set"<<endl;
                cout<<"[TEST] +Number of ellipses="<<Ellipses.size()<<endl;
                cout<<"[TEST] +Number of rectangles="<<Rectangles.size()<<endl;
            }
            //Clear obstacles
            Ellipses.clear();
            Rectangles.clear();
        }


        //Set society pose
        if(setSocietyPose)
        {
            //flag
            setSocietyPose=false;
            if(!MyDroneTrajectoryPlanner.setSocietyPose(SocietyPose))
            {
                cout<<"[TEST] Error setting society pose"<<endl;
                return 0;
            }
            else
            {
                cout<<"[TEST] Society pose set ok"<<endl;
            }
            //clear
            SocietyPose.clear();
        }


        //Time ant
        start = std::chrono::system_clock::now();

        //Calculate trayectory
        //trajectoryFound=false;
        if(!MyDroneTrajectoryPlanner.run(resultTrajectory))
        {
            cout<<"[TEST] error running DTP"<<endl;
        }

        if(resultTrajectory==TrajectoryPlanner::Result::ERROR_FINDING_TRAJECTORY)
        {
            cout<<"[TEST] Unable to find trajectory"<<endl;
        }
        else if(resultTrajectory==TrajectoryPlanner::Result::NEW_TRAJECTORY_FOUND)
        {
            cout<<"[TEST] Trajectory planified"<<endl;
        }


        //Get trajectory
        if(!MyDroneTrajectoryPlanner.getTrajectory(trajectory))
        {
            cout<<"[TEST] Unable to get trajectory"<<endl;
            break;
        }
        else
        {
            cout<<"[TEST] Trajectory is:"<<endl;
            cout<<"\t{ ";
            for(unsigned int i=0;i<trajectory.size();i++)
            {
                if(i==0)
                    cout<<"[";
                else
                    cout<<"\t  [";
                for(unsigned int j=0;j<trajectory[i].size();j++)
                {
                    cout<<trajectory[i][j];
                    if(j!=trajectory[i].size()-1)
                        cout<<" ";
                }
                if(i==trajectory.size()-1)
                    cout<<"]";
                else
                    cout<<"];"<<endl;
            }
            cout<<" }"<<endl;
        }

        //Time sig
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        cout<<"[TEST] Time for plannify="<<elapsed_seconds.count()<<". Rate="<<1.0/elapsed_seconds.count()<<endl;


        //Command
        cout<<"command=";
        cin.tie( NULL );
        command=cin.get();
        cin.ignore();


        //End simulation
        bool stopSimulation=false;
        switch(command)
        {
        //Continue
        case ' ':
            break;
        //Stop
        case 'q':
            stopSimulation=true;
            break;
        //Point init
        case 'e':
            pointInit[0]=1.0;
            pointInit[1]=1.0;
            setPointInit=true;
            break;
        case 'd':
            pointInit[0]=7.0;
            pointInit[1]=5.0;
            setPointInit=true;
            break;
        //Point fin
        case 'r':
            pointFin[0]=2.0;
            pointFin[1]=20.0;
            setPointFin=true;
            break;
        case 'f':
            pointFin[0]=10.0;
            pointFin[1]=20.0;
            setPointFin=true;
            break;
        //Obstacles
        case 't':
        {
            idObstacle=1000;
            centerPointObstacle[0]=5.0;
            centerPointObstacle[1]=5.0;
            radiusObstacle[0]=0.5;
            radiusObstacle[1]=0.5;
            yawAngleObstacle=0.0;
            OneEllipse.define(idObstacle,centerPointObstacle,radiusObstacle,yawAngleObstacle);
            Ellipses.push_back(OneEllipse);
            setObstacles=true;
            break;
        }
        case 'g':
        {
            idObstacle=1001;
            centerPointObstacle[0]=7.0;
            centerPointObstacle[1]=18.0;
            sizeObstacle[0]=8.0;
            sizeObstacle[1]=0.2;
            yawAngleObstacle=0.0;
            OneRectangle.define(idObstacle,centerPointObstacle,sizeObstacle,yawAngleObstacle);
            Rectangles.push_back(OneRectangle);
            setObstacles=true;
            break;
        }
        //Society Pose
        case 'y':
        {
            //Robot 0
            idRobot=0;
            centerRobot[0]=pointInit[0];
            centerRobot[1]=pointInit[1];
            sizeRobot[0]=1.5;
            sizeRobot[1]=1.5;
            yawAngleRobot=0.0;
            OneRobot.setRobotParameters(idRobot,centerRobot,sizeRobot,yawAngleRobot);
            SocietyPose.push_back(OneRobot);
            //Robot 1
            idRobot=1;
            centerRobot[0]=7.7;
            centerRobot[1]=7.2;
            sizeRobot[0]=1.5;
            sizeRobot[1]=1.5;
            yawAngleRobot=0.0;
            OneRobot.setRobotParameters(idRobot,centerRobot,sizeRobot,yawAngleRobot);
            SocietyPose.push_back(OneRobot);

            setSocietyPose=true;
            break;
        }
        case 'h':
        {
            //Robot 0
            idRobot=0;
            centerRobot[0]=pointInit[0];
            centerRobot[1]=pointInit[1];
            sizeRobot[0]=1.5;
            sizeRobot[1]=1.5;
            yawAngleRobot=0.0;
            OneRobot.setRobotParameters(idRobot,centerRobot,sizeRobot,yawAngleRobot);
            SocietyPose.push_back(OneRobot);
            //Robot 1
            idRobot=1;
            centerRobot[0]=13.0;
            centerRobot[1]=5.0;
            sizeRobot[0]=1.5;
            sizeRobot[1]=1.5;
            yawAngleRobot=0.0;
            OneRobot.setRobotParameters(idRobot,centerRobot,sizeRobot,yawAngleRobot);
            SocietyPose.push_back(OneRobot);

            setSocietyPose=true;
            break;
        }
        }


        if(stopSimulation)
            break;
    }

    //End
    return 1;
}
