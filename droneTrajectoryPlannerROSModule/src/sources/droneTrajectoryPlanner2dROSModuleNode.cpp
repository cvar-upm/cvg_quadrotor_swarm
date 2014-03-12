//////////////////////////////////////////////////////
//  DroneTrajectoryPlannerROSModuleNode.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 23, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////



//I/O Stream
//std::cout
#include <iostream>



// ROS
//ros::init(), ros::NodeHandle, ros::ok(), ros::spinOnce()
#include "ros/ros.h"


//Drone Module
//#include "droneModuleROS.h"

//trajectory planner
#include "droneTrajectoryPlannerROSModule.h"

//Communication Definition
//MODULE_NAME_TRAJECTORY_PLANNER
#include "communication_definition.h"



#define _VERBOSE_DRONE_TRAJECTORY_PLANNER_ROSNODE


using namespace std;

int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, MODULE_NAME_TRAJECTORY_PLANNER); //Say to ROS the name of the node and the parameters
    ros::NodeHandle n; //Este nodo admite argumentos!!

    cout<<"[ROSNODE] Starting Drone Trajectory Planner"<<endl;

    //Trajectory planner
    DroneTrajectoryPlanner2dROSModule MyDroneTrajectoryPlannerROSModule;
    MyDroneTrajectoryPlannerROSModule.open(n,MODULE_NAME_TRAJECTORY_PLANNER);

    //Loop
    while(ros::ok())
    {
        //Time init
        //double timeTrajectoryPlannerInit=ros::Time::now().toSec();
        //Read ros messages
        ros::spinOnce();
        //Run
        if(!MyDroneTrajectoryPlannerROSModule.run())
        {
            //cout<<"[ROSNODE] Error finding trajectory"<<endl;
            //continue;
        }
        else
        {
            TrajectoryPlanner::Result result=MyDroneTrajectoryPlannerROSModule.resultTrajectoryPlanner();

#ifdef _VERBOSE_DRONE_TRAJECTORY_PLANNER_ROSNODE
            switch(result)
            {
            case TrajectoryPlanner::Result::NEW_TRAJECTORY_FOUND:
            {
                cout<<"[ROSNODE] New trajectory found:"<<endl;
                std::vector< std::vector<double> > trajectory;
                MyDroneTrajectoryPlannerROSModule.getTrajectory(trajectory);
                cout<<"{";
                for(unsigned int i=0;i<trajectory.size();i++)
                {
                    cout<<"["<<trajectory[i][0]<<";"<<trajectory[i][1]<<"]";
                    if(i!=trajectory.size()-1)
                        cout<<";";
                }
                cout<<"}"<<endl;
                break;
            }
            case TrajectoryPlanner::Result::NO_NEED_FOR_NEW_TRAJECTORY:
                //cout<<"[ROSNODE] No need for new trajectory"<<endl;
                break;
            case TrajectoryPlanner::Result::UNABLE_TO_FIND_TRAJECTORY:
                cout<<"[ROSNODE] Unable to find trajectory"<<endl;
                break;
            case TrajectoryPlanner::Result::TRAJECTORY_PARAMETERS_NOT_DEFINED:
                cout<<"[ROSNODE] Trajectory parameters not defined"<<endl;
                break;
            case TrajectoryPlanner::Result::ERROR_FINDING_TRAJECTORY:
                cout<<"[ROSNODE] Error finding trajectory"<<endl;
                break;
            case TrajectoryPlanner::Result::ERROR_SETTING_POINT_INIT:
                cout<<"[ROSNODE] Error setting point init"<<endl;
                break;
            case TrajectoryPlanner::Result::ERROR_SETTING_POINT_FIN:
                cout<<"[ROSNODE] Error setting point fin"<<endl;
                break;
            case TrajectoryPlanner::Result::ERROR_RECOVERING_TRAJECTORY:
                cout<<"[ROSNODE] Error recovering trajectory"<<endl;
                break;
            default:
                cout<<"[ROSNODE] Error because of nothing"<<endl;
                break;

            }

#endif

        }

        //Sleep
        MyDroneTrajectoryPlannerROSModule.sleep();
        //Time fin
        //double timeTrajectoryPlannerFin=ros::Time::now().toSec();
        //cout<<"[ROSNODE] +Trajectory Planner ended in "<<timeTrajectoryPlannerFin-timeTrajectoryPlannerInit<<" seconds!"<<endl;

    }

    return 1;
}

