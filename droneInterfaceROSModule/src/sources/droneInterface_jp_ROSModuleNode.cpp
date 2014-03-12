/// ROS
#include "ros/ros.h"
#include <sstream>
#include <stdio.h>

//Console
#include <curses.h>
//Drone
#include "droneInterfaceROSModule.h"
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

#include "dronetrajectorycontrollerinterface.h"
#include "dronelocalizerinterface.h"
#include "droneekfstateestimatorinterface.h"
#include "droneMsgsROS/setInitDroneYaw_srv_type.h"
#include "control/simpletrajectorywaypoint.h"

//http://www.asciitable.com/
#define ASCII_KEY_UP      65
#define ASCII_KEY_DOWN    66
#define ASCII_KEY_RIGHT   67
#define ASCII_KEY_LEFT    68

#define ASCII_KEY_DEL      127

//Define step commands
#define CTE_COMMAND_YAW    0.40
#define CTE_COMMAND_PITCH  0.30
#define CTE_COMMAND_ROLL   0.30
#define CTE_COMMAND_HEIGHT 0.70

// Define controller commands define constants
#define CONTROLLER_CTE_COMMAND_SPEED        ( 1.00 )
#define CONTROLLER_STEP_COMMAND_POSITTION   ( 0.25 )
#define CONTROLLER_STEP_COMMAND_ALTITUDE    ( 0.25 )
#define CONTROLLER_STEP_COMMAND_YAW         ( 10.0 * (M_PI/180.0) )

#define FREQ_INTERFACE  200.0

#define DISPLAY_DRONE_IMAGE

#define DISPLAY_COLUMN_SIZE 22

// see keybindings_interfacejp_tentative.txt for a summary of keybindings

void printout_stream( std::stringstream *pinterface_printout_stream, int *lineCommands, int *columCommands);

int main(int argc, char **argv)
{

    ros::init(argc, argv, MODULE_NAME_DRONE_CONSOLE_INTERFACE);
    ros::NodeHandle n;

    DroneInterfaceROSModule drone_interface;
    drone_interface.open(n,MODULE_NAME_DRONE_CONSOLE_INTERFACE);

    DroneTrajectoryControllerInterface drone_trajectory_controller_interface(MODULE_NAME_TRAJECTORY_CONTROLLER, ModuleNames::TRAJECTORY_CONTROLLER);
    drone_trajectory_controller_interface.open(n);

    DroneEKFStateEstimatorInterface drone_ekf_state_estimator_interface(MODULE_NAME_ODOMETRY_STATE_ESTIMATOR, ModuleNames::ODOMETRY_STATE_ESTIMATOR);
    drone_ekf_state_estimator_interface.open(n);


    DroneLocalizerInterface drone_localizer_interface(MODULE_NAME_LOCALIZER, ModuleNames::LOCALIZER);
    drone_localizer_interface.open(n);

    printf("Starting Drone interface...\n");
    bool endProgram=false;

    // ncurses initialization
    initscr();
    curs_set(0);
    noecho();
    nodelay(stdscr, TRUE);
    erase(); refresh();

    printw("Drone interface"); //refresh();

    char command=0;
    int num_state_line = 18;
    int num_command_line = 20;
    //Loop
    while (ros::ok()) {
        //Read messages
        ros::spinOnce();

#ifdef DISPLAY_DRONE_IMAGE
        if(drone_interface.newBottomImage)
        {
            drone_interface.newBottomImage=false;
            cv::imshow("Drone Image",drone_interface.bottomImage);
            cv::waitKey(1);
        }
        else if(drone_interface.newFrontImage)
        {
            drone_interface.newFrontImage=false;
            cv::imshow("Drone Image",drone_interface.frontImage);
            cv::waitKey(1);
        }
#endif

        int lineCommands=1, columCommands=0;
        {
        std::stringstream *pinterface_printout_stream = drone_interface.getOdometryStream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        pinterface_printout_stream = drone_interface.getDroneCommandsStream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        num_state_line   = lineCommands + 1;
        num_command_line = num_state_line + 2;

#ifdef TEST_WITHOUT_AURCOSLAM
        lineCommands=2, columCommands=DISPLAY_COLUMN_SIZE;
        pinterface_printout_stream = drone_ekf_state_estimator_interface.getStateEstimatorState_Stream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        pinterface_printout_stream = drone_ekf_state_estimator_interface.getPositionEstimates_GMRwrtGFF_Stream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        pinterface_printout_stream = drone_ekf_state_estimator_interface.getSpeedEstimates_GMRwrtGFF_Stream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
#else  // TEST_WITHOUT_AURCOSLAM
#ifdef TEST_WITH_AURCOSLAM
        lineCommands=2, columCommands=DISPLAY_COLUMN_SIZE;
        pinterface_printout_stream = drone_localizer_interface.getLocalizerState_Stream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        pinterface_printout_stream = drone_localizer_interface.getPositionEstimates_GMRwrtGFF_Stream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        pinterface_printout_stream = drone_localizer_interface.getSpeedEstimates_GMRwrtGFF_Stream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
#endif // TEST_WITH_AURCOSLAM
#endif // TEST_WITHOUT_AURCOSLAM


        lineCommands=2, columCommands=2*DISPLAY_COLUMN_SIZE;
        pinterface_printout_stream = drone_trajectory_controller_interface.getControllerState();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        pinterface_printout_stream = drone_trajectory_controller_interface.getPositionReferences_GMRwrtGFF_Stream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        pinterface_printout_stream = drone_trajectory_controller_interface.getSpeedReferences_GMRwrtGFF_Stream();
        printout_stream(pinterface_printout_stream, &lineCommands, &columCommands);
        }

        // Drone console information print
        {

            // Drone telemetry
            {
//                //Odometry measures
//                move(1,0);
//                printw("+Odometry measures:");
//                int lineOdometry=2; int columOdometry=0;
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -Battery=%3.1f%%",drone_interface.BatteryMsgs.batteryPercent); //refresh();

//                //Rotations
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -RotX=% -2.2fº",drone_interface.RotationAnglesMsgs.vector.x); //refresh();
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -RotY=% -2.2fº",drone_interface.RotationAnglesMsgs.vector.y); //refresh();
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -RotZ=% -2.2fº",drone_interface.RotationAnglesMsgs.vector.z); //refresh();

//                //altitude
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -h=% 2.3f m",drone_interface.AltitudeMsgs.altitude); //refresh();
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -Vz=% 3.5f m/s",drone_interface.AltitudeMsgs.altitude_speed); //refresh();

//                //Speeds
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -Vx=% 2.5f m/s",drone_interface.GroundSpeedMsgs.vector.x); //refresh();
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -Vy=% 2.5f m/s",drone_interface.GroundSpeedMsgs.vector.y); //refresh();

//                //acelerations
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -Ax=% 2.5f m/s2",drone_interface.ImuMsgs.linear_acceleration.x); //refresh();
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -Ay=% 2.5f m/s2",drone_interface.ImuMsgs.linear_acceleration.y); //refresh();
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -Az=% 2.5f m/s2",drone_interface.ImuMsgs.linear_acceleration.z); //refresh();


//                //Magnetometer
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -MagX=% 3.5f (TBA)",drone_interface.MagnetometerMsgs.vector.x); //refresh();
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -MagY=% 3.5f (TBA)",drone_interface.MagnetometerMsgs.vector.y); //refresh();
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -MagZ=% 3.5f (TBA)",drone_interface.MagnetometerMsgs.vector.z); //refresh();


//                //Pressure
//                move(lineOdometry++,columOdometry); //refresh();
//                printw(" -Press=% 3.0f (TBA)",drone_interface.PressureMsgs.fluid_pressure); //refresh();
            }

            // Drone navcommand
            {
//                //Command measures
//                int lineCommands=2; int columCommands=35;
//                move(1,columCommands);
//                printw("+Commands:");

//                move(lineCommands++,columCommands); //refresh();
//                printw(" -Pitch=%0.3f",drone_interface.DronePitchRollCmdMsgs.pitchCmd); //refresh();
//                move(lineCommands++,columCommands); //refresh();
//                printw(" -Roll=%0.3f",drone_interface.DronePitchRollCmdMsgs.rollCmd); //refresh();

//                move(lineCommands++,columCommands); //refresh();
//                printw(" -dz=%0.3f",drone_interface.DroneDAltitudeCmdMsgs.dAltitudeCmd); //refresh();

//                move(lineCommands++,columCommands); //refresh();
//                printw(" -dYaw=%0.3f",drone_interface.DroneDYawCmdMsgs.dYawCmd); //refresh();
            }

        }

        double current_xs, current_ys, current_zs, current_yaws;
        {
#ifdef TEST_WITHOUT_AURCOSLAM
            const droneMsgsROS::dronePose &last_drone_estimated_GMRwrtGFF_pose_msg = drone_ekf_state_estimator_interface.last_drone_estimated_GMRwrtGFF_pose_msg();
            current_xs   = last_drone_estimated_GMRwrtGFF_pose_msg.x;
            current_ys   = last_drone_estimated_GMRwrtGFF_pose_msg.y;
            current_zs   = last_drone_estimated_GMRwrtGFF_pose_msg.z;
            current_yaws = last_drone_estimated_GMRwrtGFF_pose_msg.yaw;
#else  // TEST_WITHOUT_AURCOSLAM
#ifdef TEST_WITH_AURCOSLAM
            const droneMsgsROS::dronePose &last_drone_estimated_GMRwrtGFF_pose_msg = drone_localizer_interface.last_drone_estimated_GMRwrtGFF_pose_msg();
            current_xs   = last_drone_estimated_GMRwrtGFF_pose_msg.x;
            current_ys   = last_drone_estimated_GMRwrtGFF_pose_msg.y;
            current_zs   = last_drone_estimated_GMRwrtGFF_pose_msg.z;
            current_yaws = last_drone_estimated_GMRwrtGFF_pose_msg.yaw;
#endif // TEST_WITH_AURCOSLAM
#endif // TEST_WITHOUT_AURCOSLAM
        }

        double current_xci, current_yci, current_zci, current_yawci;
        {
            const droneMsgsROS::dronePose &current_drone_position_reference = drone_trajectory_controller_interface.current_drone_position_reference();
            current_xci = current_drone_position_reference.x;
            current_yci = current_drone_position_reference.y;
            current_zci = current_drone_position_reference.z;
            current_yawci = current_drone_position_reference.yaw;
        }

        double current_vxfi, current_vyfi; // , current_vzfi, current_dyawfi;
        {
            const droneMsgsROS::droneSpeeds &current_drone_speed_reference = drone_trajectory_controller_interface.current_drone_speed_reference();
            current_vxfi = current_drone_speed_reference.dx;
            current_vyfi = current_drone_speed_reference.dy;
            // current_vzfi = current_drone_speed_reference.dz;
            // current_dyawfi = current_drone_speed_reference.dyaw;
        }

        move(num_command_line,0);
        printw("Command: "); //refresh();

        command=getch();
        switch(command) {
        case 't': // take off: take off
            //Clear Cmd
            drone_interface.drone_take_off();
            printw("Taking off"); clrtoeol();
            break;
        case 'y': // land: hover > land > stop controller
            drone_interface.drone_hover();
            drone_interface.drone_land();
            if ( drone_trajectory_controller_interface.isStarted() )
                drone_trajectory_controller_interface.stop();
            printw("Landing"); clrtoeol();
            break;
        case ' ': // emergency stop: emergency stop > stop controller
            drone_interface.drone_emergency_stop();
            if ( drone_trajectory_controller_interface.isStarted() )
                drone_trajectory_controller_interface.stop();
            printw("Resetting"); clrtoeol();
            break;
        case 'h': // hover: hover > stop controller
            drone_interface.drone_hover();
            if ( drone_trajectory_controller_interface.isStarted() )
                drone_trajectory_controller_interface.stop();
            printw("Hover"); clrtoeol();
            break;
        case 'm': // move: move (set flying mode)
            drone_interface.drone_move();
            printw("Move"); clrtoeol();
            break;
        case 's': // set commands to zero: [open loop] hover, [closed loop] (non internal - with controller) hover
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.drone_hover();
                printw("Hover"); clrtoeol();
            } else {
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL:
                    drone_trajectory_controller_interface.publishDroneSpeedsReference( 0.0, 0.0);
                    printw("control mode: speed, stop command"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL:
                    drone_trajectory_controller_interface.publishDroneSpeedsReference( 0.0, 0.0);
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xs, current_ys, current_zs);
                    drone_trajectory_controller_interface.publishDroneYawReference(current_yaws);
                    printw("control mode: position, stop command"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
                    if( drone_trajectory_controller_interface.setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL) ) {
                        drone_trajectory_controller_interface.publishDroneSpeedsReference( 0.0, 0.0);
                        drone_trajectory_controller_interface.publishDronePositionReference( current_xs, current_ys, current_zs);
                        drone_trajectory_controller_interface.publishDroneYawReference(current_yaws);
                        printw("control mode: trajectory >> position, stop command"); clrtoeol();
                    } else {
                        printw("error changing to POSITION_CONTROL"); clrtoeol();
                    }
                    break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            // printw("Hover (keep control strategy)"); clrtoeol();
            break;

        case 'q': // move upwards: [open loop] upwards speed command; [speed, position control] move altitude reference upwards
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.DroneDAltitudeCmdMsgs.dAltitudeCmd = CTE_COMMAND_HEIGHT;
                drone_interface.publishDAltitudeCmd();
                printw("DAltitude=%f",drone_interface.DroneDAltitudeCmdMsgs.dAltitudeCmd); clrtoeol();
            } else {
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL: {
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xci, current_yci, current_zci + CONTROLLER_STEP_COMMAND_ALTITUDE);
                    printw("control mode: speed, upwards altitude step %f", +CONTROLLER_STEP_COMMAND_ALTITUDE); clrtoeol();
                }   break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL: {
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xci, current_yci, current_zci + CONTROLLER_STEP_COMMAND_ALTITUDE);
                    printw("control mode: position, upwards altitude step %f", +CONTROLLER_STEP_COMMAND_ALTITUDE); clrtoeol();
                }   break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL: {
                    printw("control mode: trajectory, command ignored"); clrtoeol();
                }   break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case 'a': // move downwards: same as before but downwards
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.DroneDAltitudeCmdMsgs.dAltitudeCmd = -CTE_COMMAND_HEIGHT;
                drone_interface.publishDAltitudeCmd();
                printw("DAltitude=%f",drone_interface.DroneDAltitudeCmdMsgs.dAltitudeCmd); clrtoeol();
            } else {
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL: {
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xci, current_yci, current_zci - CONTROLLER_STEP_COMMAND_ALTITUDE);
                    printw("control mode: speed, downwards altitude step %f", -CONTROLLER_STEP_COMMAND_ALTITUDE); clrtoeol();
                }   break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL: {
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xci, current_yci, current_zci - CONTROLLER_STEP_COMMAND_ALTITUDE);
                    printw("control mode: position, downwards altitude step %f", -CONTROLLER_STEP_COMMAND_ALTITUDE); clrtoeol();
                }   break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL: {
                    printw("control mode: trajectory, command ignored"); clrtoeol();
                }   break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case 'z': // (yaw) turn counter-clockwise: [open loop] turn with speed command; [any controller] move yaw reference counter-clockwise
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.DroneDYawCmdMsgs.dYawCmd = -CTE_COMMAND_YAW;
                drone_interface.publishDYawCmd();
                printw("DYaw=%f",drone_interface.DroneDYawCmdMsgs.dYawCmd); clrtoeol();
            } else {
                double new_yawci = current_yawci + CONTROLLER_STEP_COMMAND_YAW;
                drone_trajectory_controller_interface.publishDroneYawReference(new_yawci);
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL:
                    printw("control mode: speed, yaw leftwards rotation %f", +CONTROLLER_STEP_COMMAND_YAW); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL:
                    printw("control mode: position, yaw leftwards rotation %f", +CONTROLLER_STEP_COMMAND_YAW); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
                    printw("control mode: trajectory, yaw leftwards rotation %f", +CONTROLLER_STEP_COMMAND_YAW); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case 'x': // (yaw) turn clockwise: same as before but clockwise
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.DroneDYawCmdMsgs.dYawCmd = +CTE_COMMAND_YAW;
                drone_interface.publishDYawCmd();
                printw("DYaw=%f",drone_interface.DroneDYawCmdMsgs.dYawCmd); clrtoeol();
            } else {
                double new_yawci = current_yawci - CONTROLLER_STEP_COMMAND_YAW;
                drone_trajectory_controller_interface.publishDroneYawReference(new_yawci);
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL:
                    printw("control mode: speed, yaw rightwards rotation %f", -CONTROLLER_STEP_COMMAND_YAW); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL:
                    printw("control mode: position, yaw rightwards rotation %f", -CONTROLLER_STEP_COMMAND_YAW); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
                    printw("control mode: trajectory, yaw rightwards rotation %f", -CONTROLLER_STEP_COMMAND_YAW); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case '\'': // set yaw reference to 0 (look parallel to x-axis)
            if ( drone_trajectory_controller_interface.isStarted() ) {
                double new_yawci = 0.0;
                drone_trajectory_controller_interface.publishDroneYawReference(new_yawci);
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL:
                    printw("control mode: speed, head yaw north"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL:
                    printw("control mode: position, head yaw north"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
                    printw("control mode: trajectory, head yaw north"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case ASCII_KEY_RIGHT: // move right, or to[+]y_axis: [open loop] or [speed command] or [position command], ignored during trayectory control
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.DronePitchRollCmdMsgs.rollCmd = CTE_COMMAND_ROLL;
                drone_interface.publishPitchRollCmd();
                printw("Roll=%f", drone_interface.DronePitchRollCmdMsgs.rollCmd); clrtoeol();
            } else {
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL: {
                    double new_vyfi = - CONTROLLER_CTE_COMMAND_SPEED;
                    drone_trajectory_controller_interface.publishDroneSpeedsReference( current_vxfi, new_vyfi);
                    printw("control mode: speed, vyc =%f", new_vyfi); clrtoeol();
                } break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL: {
                    double new_yci = current_yci - CONTROLLER_STEP_COMMAND_POSITTION;
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xci, new_yci, current_zci);
                    printw("control mode: position, yc =%f", new_yci); clrtoeol();
                } break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
                    printw("control mode: trajectory, command ignored"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case ASCII_KEY_LEFT:  // move left: [open loop] or [speed command] or [position command], ignored during trayectory control
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.DronePitchRollCmdMsgs.rollCmd = -CTE_COMMAND_ROLL;
                drone_interface.publishPitchRollCmd();
                printw("Roll=%f", drone_interface.DronePitchRollCmdMsgs.rollCmd); clrtoeol();
            } else {
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL: {
                    double new_vyfi = + CONTROLLER_CTE_COMMAND_SPEED;
                    drone_trajectory_controller_interface.publishDroneSpeedsReference( current_vxfi, new_vyfi);
                    printw("control mode: speed, vyc =%f", new_vyfi); clrtoeol();
                } break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL: {
                    double new_yci = current_yci + CONTROLLER_STEP_COMMAND_POSITTION;
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xci, new_yci, current_zci);
                    printw("control mode: position, yc =%f", new_yci); clrtoeol();
                } break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
                    printw("control mode: trajectory, command ignored"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case ASCII_KEY_DOWN:  // move backwards: [open loop] or [speed command] or [position command], ignored during trayectory control
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.DronePitchRollCmdMsgs.pitchCmd = +CTE_COMMAND_PITCH;
                drone_interface.publishPitchRollCmd();
                printw("Pitch=%f", drone_interface.DronePitchRollCmdMsgs.pitchCmd); clrtoeol();
            } else {
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL: {
                    double new_vxfi = - CONTROLLER_CTE_COMMAND_SPEED;
                    drone_trajectory_controller_interface.publishDroneSpeedsReference( new_vxfi, current_vyfi);
                    printw("control mode: speed, vxc =%f", new_vxfi); clrtoeol();
                } break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL: {
                    double new_xci = current_xci - CONTROLLER_STEP_COMMAND_POSITTION;
                    drone_trajectory_controller_interface.publishDronePositionReference( new_xci, current_yci, current_zci);
                    printw("control mode: position, xc =%f", new_xci); clrtoeol();
                } break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
                    printw("control mode: trajectory, command ignored"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case ASCII_KEY_UP:    // move frontwards; [open loop] or [speed command] or [position command], ignored during trayectory control
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.DronePitchRollCmdMsgs.pitchCmd = -CTE_COMMAND_PITCH;
                drone_interface.publishPitchRollCmd();
                printw("Pitch=%f", drone_interface.DronePitchRollCmdMsgs.pitchCmd); clrtoeol();
            } else {
                switch ( drone_trajectory_controller_interface.getControlMode() ) {
                case Controller_MidLevel_controlMode::SPEED_CONTROL: {
                    double new_vxfi = + CONTROLLER_CTE_COMMAND_SPEED;
                    drone_trajectory_controller_interface.publishDroneSpeedsReference( new_vxfi, current_vyfi);
                    printw("control mode: speed, vxc =%f", new_vxfi); clrtoeol();
                } break;
                case Controller_MidLevel_controlMode::POSITION_CONTROL: {
                    double new_xci = current_xci + CONTROLLER_STEP_COMMAND_POSITTION;
                    drone_trajectory_controller_interface.publishDronePositionReference( new_xci, current_yci, current_zci);
                    printw("control mode: position, xc =%f", new_xci); clrtoeol();
                } break;
                case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
                    printw("control mode: trajectory, command ignored"); clrtoeol();
                    break;
                case Controller_MidLevel_controlMode::UNKNOWN_CONTROL_MODE:
                default:
                    drone_interface.drone_hover();
                    if ( drone_trajectory_controller_interface.isStarted() )
                        drone_trajectory_controller_interface.stop();
                    printw("current control_mode unknown: Hover"); clrtoeol();
                    break;
                }
            }
            break;
        case 'j': { // reset EKF: stop controller > reset EKF
            if ( drone_trajectory_controller_interface.isStarted() )
                drone_trajectory_controller_interface.stop();
            drone_ekf_state_estimator_interface.reset();
            printw("reset EKF"); clrtoeol();
        } break;
        case 'k': { // stop  EKF: stop controller > stop EKF
            if ( drone_trajectory_controller_interface.isStarted() )
                drone_trajectory_controller_interface.stop();
            drone_ekf_state_estimator_interface.stop();
            printw("stop  EKF"); clrtoeol();
        } break;
        case 'l': { // start EKF: start EKF and setInitDroneYaw
            drone_ekf_state_estimator_interface.start();
            double current_yaw_droneLMrT_telemetry_rad = drone_interface.RotationAnglesMsgs.vector.z * (M_PI/180.0);
            if ( drone_ekf_state_estimator_interface.sendInitDroneYaw(current_yaw_droneLMrT_telemetry_rad) ) {
            }
            printw("start EKF"); clrtoeol();
        } break;
        case 'u': { // reset controller
            drone_trajectory_controller_interface.reset();
            printw("reset controller"); clrtoeol();
        } break;
        case 'i': { // stop  controller: stop controller > drone_hover
            drone_trajectory_controller_interface.stop();
            drone_interface.drone_hover();
            printw("stop  controller"); clrtoeol();
        } break;
        case 'o': { // start controller: start EKF > start controller
            if ( !drone_ekf_state_estimator_interface.isStarted() )
                drone_ekf_state_estimator_interface.start(true);
            drone_interface.drone_move();
            if ( !drone_trajectory_controller_interface.isStarted() )
                drone_trajectory_controller_interface.start();
            printw("start controller"); clrtoeol();
        } break;
        case '7': { // start speed controller: if controller started > drone_move > start controller (speed)
            if ( drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.drone_move();
                if ( drone_trajectory_controller_interface.setControlMode(Controller_MidLevel_controlMode::SPEED_CONTROL) ) {
                    drone_trajectory_controller_interface.publishDroneSpeedsReference( 0.0, 0.0);
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xs, current_ys, current_zs);
                    drone_trajectory_controller_interface.publishDroneYawReference( current_yaws);
                    printw("control mode: speed"); clrtoeol();
                } else {
                    printw("could not set control mode");
                }
            }
        } break;
        case '8': { // start position controller: if controller started > drone_move > start controller (position)
            if ( drone_trajectory_controller_interface.isStarted() ) {
                drone_interface.drone_move();
                if ( drone_trajectory_controller_interface.setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL) ) {
                    drone_trajectory_controller_interface.publishDroneSpeedsReference( 0.0, 0.0);
                    drone_trajectory_controller_interface.publishDronePositionReference( current_xs, current_ys, current_zs);
                    drone_trajectory_controller_interface.publishDroneYawReference( current_yaws);
                    printw("control mode: position"); clrtoeol();
                } else {
                    printw("could not set control mode");
                }
            }
        } break;
        case '9': { // start trajectory controller (3D quadrilateral) using abs_trajectory_ref channel
            if ( drone_trajectory_controller_interface.isStarted() ) {
                std::vector<SimpleTrajectoryWaypoint> trajectory_waypoints_out;
                int initial_checkpoint_out = 0;
                bool is_periodic_out = true;
                trajectory_waypoints_out.push_back( SimpleTrajectoryWaypoint( current_xs+0.0, current_ys+0.0, current_zs+0.0) );
                trajectory_waypoints_out.push_back( SimpleTrajectoryWaypoint( current_xs+0.0, current_ys+3.0, current_zs+0.0) );
                trajectory_waypoints_out.push_back( SimpleTrajectoryWaypoint( current_xs+1.0, current_ys+3.0, current_zs+3.0) );
                trajectory_waypoints_out.push_back( SimpleTrajectoryWaypoint( current_xs+3.0, current_ys+0.0, current_zs+1.5) );
                drone_trajectory_controller_interface.publishDroneAbsTrajecotryReference( &trajectory_waypoints_out, initial_checkpoint_out, is_periodic_out);
            }
        } break;
        case '0': { // start trajectory controller (3D quadrilateral) using rel_trajectory_ref channel
            if ( drone_trajectory_controller_interface.isStarted() ) {
                std::vector<SimpleTrajectoryWaypoint> trajectory_waypoints_out;
                int initial_checkpoint_out = 0;
                bool is_periodic_out = true;
                trajectory_waypoints_out.push_back( SimpleTrajectoryWaypoint( 0.0, 0.0, 0.0) );
                trajectory_waypoints_out.push_back( SimpleTrajectoryWaypoint( 0.0, 3.0, 0.0) );
                trajectory_waypoints_out.push_back( SimpleTrajectoryWaypoint( 1.0, 3.0, 3.0) );
                trajectory_waypoints_out.push_back( SimpleTrajectoryWaypoint( 3.0, 0.0, 1.5) );
                drone_trajectory_controller_interface.publishDroneRelTrajecotryReference( &trajectory_waypoints_out, initial_checkpoint_out, is_periodic_out);
            }
        } break;
            //                case ASCII_KEY_DEL:
            //            endProgram=true;
            //            printw("Ending..\n"); //refresh();
            //            break;
        }







        //State info
        move(num_state_line,0); //refresh();
        printw("State: "); //refresh();

        switch(drone_interface.DroneStatusMsgs.status)
        {
        case drone_interface.DroneStatusMsgs.UNKNOWN:
            printw("Unknown"); //refresh();
            break;
        case drone_interface.DroneStatusMsgs.INITED:
            printw("Init"); //refresh();
            break;
        case drone_interface.DroneStatusMsgs.LANDED:
            printw("Landed"); //refresh();
            break;
        case drone_interface.DroneStatusMsgs.FLYING:
            printw("Flying"); //refresh();
            break;
        case drone_interface.DroneStatusMsgs.HOVERING:
            printw("Hovering"); //refresh();
            break;
        case drone_interface.DroneStatusMsgs.TAKING_OFF:
            printw("Taking off"); //refresh();
            break;
        case drone_interface.DroneStatusMsgs.LANDING:
            printw("Landing"); //refresh();
            break;
        case drone_interface.DroneStatusMsgs.LOOPING:
            printw("Looping"); //refresh();
            break;
        }
        clrtoeol();

        //Refresh
        refresh();

        if(endProgram)
            break;

        drone_interface.sleep();

    }

    endwin();

    printf("Drone Interface ended..\n");

    return 0;
}

void printout_stream( std::stringstream *pinterface_printout_stream, int *lineCommands, int *columCommands) {
    std::string     line;
    move((*lineCommands),(*columCommands));
    while( std::getline( *pinterface_printout_stream, line, '\n') ) {
        for (int i = line.size(); i < DISPLAY_COLUMN_SIZE; i++)
            line += " ";
        printw(line.c_str());
        move(++(*lineCommands),(*columCommands));
    }
}
