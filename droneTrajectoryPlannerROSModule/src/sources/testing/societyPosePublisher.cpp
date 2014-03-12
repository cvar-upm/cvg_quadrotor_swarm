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

#include "droneMsgsROS/societyPose.h"
#include "droneMsgsROS/droneInfo.h"
#include "droneMsgsROS/dronePose.h"




int main(int argc,char **argv) {

    //Ros Init
    ros::init(argc, argv, "testing_societyPose");
    ros::NodeHandle n;


    ros::Publisher publisher;
    publisher=n.advertise<droneMsgsROS::societyPose>("/drone0/societyPose",1,true);

    droneMsgsROS::societyPose msg;
    //unsigned int seq=0;

    ros::Rate rate(0.1);

    try {
        while(ros::ok())
        {
            //Read messages
            ros::spinOnce();

            //Clear
            msg.societyDrone.clear();


            //Fill the msg
            droneMsgsROS::droneInfo droneInfo;
            droneMsgsROS::dronePose dronePose;
            //Pose
            dronePose.x=5.0;
            dronePose.y=10.0;
            dronePose.z=1.2;
            dronePose.yaw=0.0;
            dronePose.pitch=0.0;
            dronePose.roll=0.0;
            //Others
            dronePose.reference_frame="a";
            dronePose.target_frame="a";
            dronePose.YPR_system="b";
            dronePose.time=ros::Time::now().toSec();



            droneInfo.id=2;
            droneInfo.pose=dronePose;

            msg.societyDrone.push_back(droneInfo);



            //Publish
            publisher.publish(msg);
            //ros::spinOnce();

            std::cout<<"new societyPose published"<<std::endl;

            //sleep
            rate.sleep();


        }
    } catch (std::exception &ex) {
        std::cout << "[ROSNODE] Exception :" << ex.what() << std::endl;
    }

    return 1;
}
