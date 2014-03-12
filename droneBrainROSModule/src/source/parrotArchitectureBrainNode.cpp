/*
*
*
*
*
*/

// ROS
#include "ros/ros.h"

// C++ standar library
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "dronearchitecturebrain.h"

#include "communication_definition.h"
#include "drone_utils/drone_state_enum.h"

//Console
#include <curses.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, MODULE_NAME_ARCHITECTURE_BRAIN);
    ros::NodeHandle n;

    //Init
    cout<<"Starting "<<MODULE_NAME_ARCHITECTURE_BRAIN<<endl;

    //Trajectory planner
    DroneArchitectureBrain myDroneArchitectureBrain;
    myDroneArchitectureBrain.open(n,MODULE_NAME_ARCHITECTURE_BRAIN);

#ifndef   TEST_WITH_AUTONOMOUS_BRAIN
    myDroneArchitectureBrain.start();
#endif // TEST_WITH_AUTONOMOUS_BRAIN

    // ncurses, initialization and title
    int scr_line = 0, scr_column = 0;
    {
        initscr();
        curs_set(0);
        noecho();
        nodelay(stdscr, TRUE);
        erase(); refresh();
        move(scr_line,scr_column);
        printw("Drone Architecture Brain Interface:"); //refresh();
    }


    //Loop
    char command=0;
    int  counter = 0;
    while(ros::ok()) {
//        std::cout << "Before ros::spinOnce()" << counter << std::endl;
        ros::spinOnce();
//        std::cout << "After ros::spinOnce()" << counter << std::endl;

#ifdef TEST_WITH_AUTONOMOUS_BRAIN
        // DONE: wait for start button 's'
        command=getch();
        if ( (command == 's') && (!myDroneArchitectureBrain.isStarted())) {
            myDroneArchitectureBrain.start();
        }
#endif // TEST_WITH_AUTONOMOUS_BRAIN

        if(!myDroneArchitectureBrain.run()) {
        }
        else {
        }

        // ncurses, [this_drone] ARDroneMode+Telemetry+droneNavdata+estimatedPose
        {
            scr_line = 1, scr_column = 0;
            move(scr_line,scr_column);
            printw("WiFi status: %s", myDroneArchitectureBrain.this_drone_interface.isWifiOk() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            std::string ardrone_mode = "";
            switch ( myDroneArchitectureBrain.this_drone_interface.last_drone_status_msg.status ) {
            case DroneState::INIT:
                ardrone_mode = "INIT          ";
                break;
            case DroneState::LANDED:
                ardrone_mode = "LANDED        ";
                break;
            case DroneState::FLYING:
                ardrone_mode = "FLYING        ";
                break;
            case DroneState::HOVERING:
                ardrone_mode = "HOVERING      ";
                break;
//            case DroneState::TEST:
//                ardrone_mode = "TEST          ";
//                break;
            case DroneState::TAKING_OFF:
                ardrone_mode = "TAKING_OFF    ";
                break;
//            case DroneState::GOTO_FIX_POINT:
//                ardrone_mode = "GOTO_FIX_POINT";
//                break;
            case DroneState::LANDING:
                ardrone_mode = "LANDING       ";
                break;
            case DroneState::LOOPING:
                ardrone_mode = "LOOPING       ";
                break;
            default: // ARDroneModes::UNKNOWN:
                ardrone_mode = "UNKNOWN       ";
                break;
            }
            printw("ARDroneMode: %s", ardrone_mode.c_str());
            move(++scr_line,scr_column);
            printw("Telemetry data:");
            move(++scr_line,scr_column);
            printw(" Battery=%6.2f%%\n",myDroneArchitectureBrain.this_drone_interface.last_battery_msg.batteryPercent);
            move(++scr_line,scr_column);
            // Attitude, YPR_wYvPuR
            printw(" Yaw    =%6.2fdeg\n", myDroneArchitectureBrain.this_drone_interface.last_rotation_angles_msg.vector.z);
            move(++scr_line,scr_column);
            printw(" Pitch  =%6.2fdeg\n", myDroneArchitectureBrain.this_drone_interface.last_rotation_angles_msg.vector.y);
            move(++scr_line,scr_column);
            printw(" Roll   =%6.2fdeg\n", myDroneArchitectureBrain.this_drone_interface.last_rotation_angles_msg.vector.x);
            move(++scr_line,scr_column);
            //altitude
            printw(" Alttd  =%6.2fm\n",   -myDroneArchitectureBrain.this_drone_interface.last_altitude_msg.altitude);
            move(++scr_line,scr_column);
            //Speeds
            printw(" Vx     =%6.2fm/s\n",myDroneArchitectureBrain.this_drone_interface.last_ground_optical_flow_msg.vector.x); // speedX
            move(++scr_line,scr_column);
            printw(" Vy     =%6.2fm/s\n",myDroneArchitectureBrain.this_drone_interface.last_ground_optical_flow_msg.vector.y); // speedY
            move(++scr_line,scr_column);
            printw(" Vz     =%6.2fm/s\n",myDroneArchitectureBrain.this_drone_interface.last_altitude_msg.altitude_speed);
            move(++scr_line,scr_column);
            // Estimated Pose
            printw("Estimated Pose:");
            move(++scr_line,scr_column);
            printw(" x    =%6.2fm\n",myDroneArchitectureBrain.this_drone_interface.last_estimatedPose.x);
            move(++scr_line,scr_column);
            printw(" y    =%6.2fm\n",myDroneArchitectureBrain.this_drone_interface.last_estimatedPose.y);
            move(++scr_line,scr_column);
            printw(" z    =%6.2fm\n",myDroneArchitectureBrain.this_drone_interface.last_estimatedPose.z);
            move(++scr_line,scr_column);
            printw(" yaw  =%6.2fdeg\n",myDroneArchitectureBrain.this_drone_interface.last_estimatedPose.yaw*(180.0/M_PI));
            move(++scr_line,scr_column);
            printw(" pitch=%6.2fdeg\n",myDroneArchitectureBrain.this_drone_interface.last_estimatedPose.pitch*(180.0/M_PI));
            move(++scr_line,scr_column);
            printw(" roll =%6.2fdeg\n",myDroneArchitectureBrain.this_drone_interface.last_estimatedPose.roll*(180.0/M_PI));
            move(++scr_line,scr_column);
        }
        // ncurses, [this_drone] module_status(isStarted isOnline)
        {
            scr_line = 1, scr_column = 25;
            move(scr_line,scr_column);
            printw("module_status:");
            move(++scr_line,scr_column);
            printw(" state_estimator:      %s %s\n", myDroneArchitectureBrain.this_drone_interface.state_estimator.isStarted() ? "ON " : "OFF", myDroneArchitectureBrain.this_drone_interface.state_estimator.isOnline() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            printw(" trajectory_controller:%s %s\n", myDroneArchitectureBrain.this_drone_interface.trajectory_controller.isStarted() ? "ON " : "OFF", myDroneArchitectureBrain.this_drone_interface.trajectory_controller.isOnline() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            printw(" arucoeye:             %s %s\n", myDroneArchitectureBrain.this_drone_interface.arucoeye.isStarted() ? "ON " : "OFF", myDroneArchitectureBrain.this_drone_interface.arucoeye.isOnline() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            printw(" localizer:            %s %s\n", myDroneArchitectureBrain.this_drone_interface.localizer.isStarted() ? "ON " : "OFF", myDroneArchitectureBrain.this_drone_interface.localizer.isOnline() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            printw(" obstacle_processor:   %s %s\n", myDroneArchitectureBrain.this_drone_interface.obstacle_processor.isStarted() ? "ON " : "OFF", myDroneArchitectureBrain.this_drone_interface.obstacle_processor.isOnline() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            printw(" trajectory_planner:   %s %s\n", myDroneArchitectureBrain.this_drone_interface.trajectory_planner.isStarted() ? "ON " : "OFF", myDroneArchitectureBrain.this_drone_interface.trajectory_planner.isOnline() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            printw(" yaw_planner:          %s %s\n", myDroneArchitectureBrain.this_drone_interface.yaw_planner.isStarted() ? "ON " : "OFF", myDroneArchitectureBrain.this_drone_interface.yaw_planner.isOnline() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            printw(" mission_planner:      %s %s\n", myDroneArchitectureBrain.this_drone_interface.mission_planner.isStarted() ? "ON " : "OFF", myDroneArchitectureBrain.this_drone_interface.mission_planner.isOnline() ? "ONLINE " : "OFFLINE");
            move(++scr_line,scr_column);
            move(++scr_line,scr_column);
            printw(" Brain State: %s\n", myDroneArchitectureBrain.brain_state_machine.getBrainState_str().c_str());
            move(++scr_line,scr_column);
            printw(" State Step:  %s\n", myDroneArchitectureBrain.brain_state_machine.getBrainStateStep_str().c_str());
            move(++scr_line,scr_column);
            printw(" Online check:             %s\n", myDroneArchitectureBrain.brain_state_machine.getOnlineCheckBool() ? "PASSED": "FAILED");
            move(++scr_line,scr_column);
            printw(" Started check:            %s\n", myDroneArchitectureBrain.brain_state_machine.getStartedCheckBool()? "PASSED": "FAILED");
            move(++scr_line,scr_column);
            printw(" Battery check, >%2.0f%%: %s\n", myDroneArchitectureBrain.this_drone_interface.battery_threshold,
                                                    myDroneArchitectureBrain.this_drone_interface.batteryCheckIsOk()?   "PASSED": "FAILED");
            move(++scr_line,scr_column);
        }
        // ncurses, [this_drone] module_status(idDrone isOnline)
        {
            scr_line = 1, scr_column = 60;
            move(scr_line,scr_column);
            printw("Other drones status:");
            move(++scr_line,scr_column);
            for (auto it : myDroneArchitectureBrain.societyMembers) {
                printw(" drone%02d: %s\n", it.last_droneInfo.id, it.isInTheSystem() ? "ONLINE " : "OFFLINE");
                move(++scr_line,scr_column);
                printw("  x  =%6.2fm\n",it.last_droneInfo.pose.x);
                move(++scr_line,scr_column);
                printw("  y  =%6.2fm\n",it.last_droneInfo.pose.y);
                move(++scr_line,scr_column);
                printw("  z  =%6.2fm\n",it.last_droneInfo.pose.z);
                move(++scr_line,scr_column);
                printw("  yaw=%6.2fdeg\n",it.last_droneInfo.pose.yaw*(180.0/M_PI));
                move(++scr_line,scr_column);
            }
        }
        // ncurses, refresh window
        {
        refresh();
        }

        counter ++;
//        printf("while(ros::ok()) BEFORE myDroneArchitectureBrain.sleep()\n");
        myDroneArchitectureBrain.sleep();
//        printf("LEAVING while(ros::ok())\n");
    }

    endwin();
    printf("Parrot Architecture Brain Interface ended..\n");
    return 1;
}
