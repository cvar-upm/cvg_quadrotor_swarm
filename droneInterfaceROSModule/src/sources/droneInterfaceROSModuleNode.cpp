
/// ROS
#include "ros/ros.h"


#include <sstream>


#include <stdio.h>



//Console
#include <curses.h>


//Drone
#include "droneInterfaceROSModule.h"


#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O


//http://www.asciitable.com/
#define ASCII_KEY_UP      65
#define ASCII_KEY_DOWN    66

#define ASCII_KEY_RIGHT   67
#define ASCII_KEY_LEFT    68

#define ASCII_KEY_DEL      127


//Define step commands
#define STEP_COMMAND_YAW    0.1
#define STEP_COMMAND_PITCH  0.1
#define STEP_COMMAND_ROLL   0.1
#define STEP_COMMAND_HEIGHT 0.1



#define DISPLAY_DRONE_IMAGE
//#define DISPLAY_FRONT_IMAGE
//#define DISPLAY_BOTTOM_IMAGE



using namespace ros;



int main(int argc, char **argv)
{

    ros::init(argc, argv, MODULE_NAME_DRONE_CONSOLE_INTERFACE);
  	ros::NodeHandle n;


    DroneInterfaceROSModule MyDroneInterfaceROSModule;
    MyDroneInterfaceROSModule.open(n,MODULE_NAME_DRONE_CONSOLE_INTERFACE);

   
    printf("Starting Drone interface...\n");
	
	
	
	bool endProgram=false;
	
	
	initscr();
    curs_set(0);
    noecho();
    nodelay(stdscr, TRUE);
	erase(); refresh();
	

    printw("Drone interface"); //refresh();



   
    char command=0;

    //Loop
	while (ros::ok())
	{
	
        //Read messages
		ros::spinOnce();


        //Images
        //MyDrone.displayFrontImage("Image");
        //MyDrone.displayBottomImage("Image");

		
#ifdef DISPLAY_FRONT_IMAGE
        if(MyDroneInterfaceROSModule.newFrontImage)
        {
            MyDroneInterfaceROSModule.newFrontImage=false;
            cv::imshow("Front Image",MyDroneInterfaceROSModule.frontImage);
            cv::waitKey(1);
        }
#endif

#ifdef DISPLAY_BOTTOM_IMAGE
        if(MyDroneInterfaceROSModule.newBottomImage)
        {
            MyDroneInterfaceROSModule.newBottomImage=false;
            cv::imshow("Bottom Image",MyDroneInterfaceROSModule.bottomImage);
            cv::waitKey(1);
        }
#endif

#ifdef DISPLAY_DRONE_IMAGE
        if(MyDroneInterfaceROSModule.newBottomImage)
        {
            MyDroneInterfaceROSModule.newBottomImage=false;
            cv::imshow("Drone Image",MyDroneInterfaceROSModule.bottomImage);
            cv::waitKey(1);
        }
        else if(MyDroneInterfaceROSModule.newFrontImage)
        {
            MyDroneInterfaceROSModule.newFrontImage=false;
            cv::imshow("Drone Image",MyDroneInterfaceROSModule.frontImage);
            cv::waitKey(1);
        }
#endif

		
        //Odometry measures
        move(1,0);
        printw("+Odometry measures:");
        int lineOdometry=2; int columOdometry=0;
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Battery=%3.1f%%",MyDroneInterfaceROSModule.BatteryMsgs.batteryPercent); //refresh();

        //Rotations
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -RotX=% -2.2fº",MyDroneInterfaceROSModule.RotationAnglesMsgs.vector.x); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -RotY=% -2.2fº",MyDroneInterfaceROSModule.RotationAnglesMsgs.vector.y); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -RotZ=% -2.2fº",MyDroneInterfaceROSModule.RotationAnglesMsgs.vector.z); //refresh();

        //altitude
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -h=% 2.3f m",MyDroneInterfaceROSModule.AltitudeMsgs.altitude); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Vz=% 3.5f m/s",MyDroneInterfaceROSModule.AltitudeMsgs.altitude_speed); //refresh();

        //Speeds
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Vx=% 2.5f m/s",MyDroneInterfaceROSModule.GroundSpeedMsgs.vector.x); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Vy=% 2.5f m/s",MyDroneInterfaceROSModule.GroundSpeedMsgs.vector.y); //refresh();

        //acelerations
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Ax=% 2.5f m/s2",MyDroneInterfaceROSModule.ImuMsgs.linear_acceleration.x); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Ay=% 2.5f m/s2",MyDroneInterfaceROSModule.ImuMsgs.linear_acceleration.y); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Az=% 2.5f m/s2",MyDroneInterfaceROSModule.ImuMsgs.linear_acceleration.z); //refresh();


        //Magnetometer
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -MagX=% 3.5f (TBA)",MyDroneInterfaceROSModule.MagnetometerMsgs.vector.x); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -MagY=% 3.5f (TBA)",MyDroneInterfaceROSModule.MagnetometerMsgs.vector.y); //refresh();
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -MagZ=% 3.5f (TBA)",MyDroneInterfaceROSModule.MagnetometerMsgs.vector.z); //refresh();


        //Pressure
        move(lineOdometry++,columOdometry); //refresh();
        printw(" -Press=% 3.0f (TBA)",MyDroneInterfaceROSModule.PressureMsgs.fluid_pressure); //refresh();



        //Command measures
        int lineCommands=2; int columCommands=35;
        move(1,columCommands);
        printw("+Commands:");

        move(lineCommands++,columCommands); //refresh();
        printw(" -Pitch=%0.3f",MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd); //refresh();
        move(lineCommands++,columCommands); //refresh();
        printw(" -Roll=%0.3f",MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd); //refresh();

        move(lineCommands++,columCommands); //refresh();
        printw(" -dz=%0.3f",MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd); //refresh();

        move(lineCommands++,columCommands); //refresh();
        printw(" -dYaw=%0.3f",MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd); //refresh();
    


    		
        move(20,0);
        printw("Command: "); //refresh();
		
		
        command=getch();
	
	
		
        switch(command)
        {
//		//Led Animations -> Service
//        case 'z':

//            printw("LED animation\n");

//            if (MyDrone.setLedAnimation(1,10.0,10))
//            {
//                //ROS_INFO("Sum: %d", (bool)srv.response.result);
//            }
//            else
//            {
//                move(22,0); //refresh();
//                //ROS_ERROR("Failed to call service ledanimation\n");
//                printw("Failed to call service ledanimation"); refresh();
//            }
//			break;
	
        //Takeoff -> Publish Topic
        case '1':
            //Clear Cmd
            MyDroneInterfaceROSModule.clearCmd();
            //Reset if needed
            if(MyDroneInterfaceROSModule.DroneStatusMsgs.status==MyDroneInterfaceROSModule.DroneStatusMsgs.UNKNOWN)
            {
                printw("Reseting. ");
                MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.RESET;
                MyDroneInterfaceROSModule.publishDroneCmd();
                ros::spinOnce();
            }
            //Wait until reseted!
            while(MyDroneInterfaceROSModule.DroneStatusMsgs.status==MyDroneInterfaceROSModule.DroneStatusMsgs.UNKNOWN)
            {
                ros::spinOnce();
            }
            //Send takeoff
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.TAKE_OFF;
            MyDroneInterfaceROSModule.publishDroneCmd();
            printw("Taking off"); //refresh();
            clrtoeol();
            break;
			
        //Land -> Publish Topic
        case '0':
            //Clear Cmd
            MyDroneInterfaceROSModule.clearCmd();
            //Send land
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.LAND;
            MyDroneInterfaceROSModule.publishDroneCmd();
            printw("Landing"); //refresh();
            clrtoeol();
            break;
			
//		//Emergency ??
//		case ' ':
//            MyDrone.setNavCommandToZero();
//            MyDrone.land();
//            printw("Emergency -> Landing\n"); //refresh();
//			break;
			
        //Reset -> Publish Topic
        case '9':
            //Clear cmd
            MyDroneInterfaceROSModule.clearCmd();
            //Send reset
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.RESET;
            MyDroneInterfaceROSModule.publishDroneCmd();
            printw("Reseting"); //refresh();
            clrtoeol();
            break;


        //Reset commands
        case '2':
            //Clear cmd
            MyDroneInterfaceROSModule.clearCmd();
            //Send reseted commands
            MyDroneInterfaceROSModule.publishDAltitudeCmd();
            MyDroneInterfaceROSModule.publishDYawCmd();
            MyDroneInterfaceROSModule.publishPitchRollCmd();
            printw("Commands set to zero");
            clrtoeol();
            break;


        //Hover
        case '\n':
            //Clear cmd
            MyDroneInterfaceROSModule.clearCmd();
            //Send Hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.HOVER;
            MyDroneInterfaceROSModule.publishDroneCmd();
            printw("Hover");
            clrtoeol();
            break;


        //Move. -> Exit hover mode
        case 'p':
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            printw("Move"); //refresh();
            clrtoeol();
            break;


        //Altitude movs
        case 'w':
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            //Command
            MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd+=STEP_COMMAND_HEIGHT; //Move up
            if(MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd>1.0)
                MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd=1.0;
            MyDroneInterfaceROSModule.publishDAltitudeCmd();
            printw("DAltitude=%f",MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd);
            clrtoeol();
            break;

        case 's':
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            //Command
            MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd-=STEP_COMMAND_HEIGHT; //Move down
            if(MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd<-1.0)
                MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd=-1.0;
            MyDroneInterfaceROSModule.publishDAltitudeCmd();
            printw("DAltitude=%f",MyDroneInterfaceROSModule.DroneDAltitudeCmdMsgs.dAltitudeCmd);
            clrtoeol();
            break;

        //Yaw
        case 'd':
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            //Command
            MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd+=STEP_COMMAND_YAW; //Turn right
            if(MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd>1.0)
                MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd=1.0;
            MyDroneInterfaceROSModule.publishDYawCmd();
            printw("DYaw=%f",MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd); //refresh();
            clrtoeol();
            break;

        case 'a':
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            //Command
            MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd-=STEP_COMMAND_YAW; //Turn left
            if(MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd<-1.0)
                MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd=-1.0;
            MyDroneInterfaceROSModule.publishDYawCmd();
            printw("DYaw=%f",MyDroneInterfaceROSModule.DroneDYawCmdMsgs.dYawCmd); //refresh();
            clrtoeol();
            break;


        //Roll
        case ASCII_KEY_RIGHT:
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            //Command
            MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd+=STEP_COMMAND_ROLL; //Move rigth
            if(MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd>1.0)
                MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd=1.0;
            MyDroneInterfaceROSModule.publishPitchRollCmd();
            printw("Roll=%f",MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd); //refresh();
            clrtoeol();
            break;

        case ASCII_KEY_LEFT:
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            //Command
            MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd-=STEP_COMMAND_ROLL; //Move left
            if(MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd<-1.0)
                MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd=-1.0;
            MyDroneInterfaceROSModule.publishPitchRollCmd();
            printw("Roll=%f",MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.rollCmd); //refresh();
            clrtoeol();
            break;


        //Pitch
        case ASCII_KEY_UP:
        {
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            //Command
            MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd+=STEP_COMMAND_PITCH; //Move forward
            if(MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd>1.0)
                MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd=1.0;
            MyDroneInterfaceROSModule.publishPitchRollCmd();
            printw("Pitch=%f",MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd); //refresh();
            clrtoeol();
            break;
        }

        case ASCII_KEY_DOWN:
            //Exit auto-hover
            MyDroneInterfaceROSModule.DroneCommandMsgs.command=MyDroneInterfaceROSModule.DroneCommandMsgs.MOVE;
            MyDroneInterfaceROSModule.publishDroneCmd();
            //Command
            MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd-=STEP_COMMAND_PITCH; //Move backward
            if(MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd<-1.0)
                MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd=-1.0;
            MyDroneInterfaceROSModule.publishPitchRollCmd();
            printw("Pitch=%f",MyDroneInterfaceROSModule.DronePitchRollCmdMsgs.pitchCmd); //refresh();
            clrtoeol();
            break;



//        //Set front camera
//        case 'n':
//            MyDrone.setCamChannel(0);
//            printw("Camera set to front\n"); //refresh();
//            break;


//        //Set bottom camera
//        case 'm':
//            MyDrone.setCamChannel(1);
//            printw("Camera set to bottom\n"); //refresh();
//            break;


        case ASCII_KEY_DEL:
            endProgram=true;
            printw("Ending..\n"); //refresh();
            break;




        }

		

		
		
	
	
		//State info
		move(18,0); //refresh();
        printw("State: "); //refresh();
		
        switch(MyDroneInterfaceROSModule.DroneStatusMsgs.status)
        {
            case MyDroneInterfaceROSModule.DroneStatusMsgs.UNKNOWN:
                printw("Unknown"); //refresh();
                break;
            case MyDroneInterfaceROSModule.DroneStatusMsgs.INITED:
                printw("Init"); //refresh();
                break;
            case MyDroneInterfaceROSModule.DroneStatusMsgs.LANDED:
                printw("Landed"); //refresh();
                break;
            case MyDroneInterfaceROSModule.DroneStatusMsgs.FLYING:
                printw("Flying"); //refresh();
                break;
            case MyDroneInterfaceROSModule.DroneStatusMsgs.HOVERING:
                printw("Hovering"); //refresh();
                break;
            case MyDroneInterfaceROSModule.DroneStatusMsgs.TAKING_OFF:
                printw("Taking off"); //refresh();
                break;
            case MyDroneInterfaceROSModule.DroneStatusMsgs.LANDING:
                printw("Landing"); //refresh();
                break;
            case MyDroneInterfaceROSModule.DroneStatusMsgs.LOOPING:
                printw("Looping"); //refresh();
                break;
        }
        clrtoeol();
	
        //Refresh
        refresh();

		if(endProgram)
			break;

        MyDroneInterfaceROSModule.sleep();

	}

	endwin();

    printf("Drone Interface ended..\n");

	return 0;
}
