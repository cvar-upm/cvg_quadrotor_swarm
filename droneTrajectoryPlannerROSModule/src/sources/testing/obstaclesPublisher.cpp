//////////////////////////////////////////////////////
//  code_tests3.cpp
//
//  Created on: Jan 21, 2013
//      Author: joselusl
//
//  Last modification on: Jan 21, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////

#include <iostream>


#include "ros/ros.h"

#include "droneMsgsROS/obstaclesTwoDim.h"
#include "droneMsgsROS/obstacleTwoDimPole.h"
#include "droneMsgsROS/obstacleTwoDimWall.h"




int main(int argc,char **argv) {

    //Ros Init
    ros::init(argc, argv, "testing_obstaclesPub");
    ros::NodeHandle n;


    ros::Publisher publisher;
    publisher=n.advertise<droneMsgsROS::obstaclesTwoDim>("/drone0/obstacles",1,true);

    droneMsgsROS::obstaclesTwoDim msg;

    ros::Rate rate(0.1);

    try {
        while(ros::ok())
        {
            //Read messages
            ros::spinOnce();

            //Clear
            msg.poles.clear();
            msg.walls.clear();


            //Fill the msg
            droneMsgsROS::obstacleTwoDimPole OnePole;
            droneMsgsROS::obstacleTwoDimWall OneWall;
            //One pole
            OnePole.id=40001;
            OnePole.centerX=6.0;
            OnePole.centerY=13.0;
            OnePole.radiusX=0.15;
            OnePole.radiusY=0.15;
            OnePole.yawAngle=0;


            msg.poles.push_back(OnePole);


            msg.time=ros::Time::now().toSec();



            //Publish
            publisher.publish(msg);
            //ros::spinOnce();

            std::cout<<"new obstacles published"<<std::endl;

            //sleep
            rate.sleep();


        }
    } catch (std::exception &ex) {
        std::cout << "[ROSNODE] Exception :" << ex.what() << std::endl;
    }

    return 1;
}
