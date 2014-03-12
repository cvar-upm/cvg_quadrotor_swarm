#ifndef NODES_DEFINITION_H
#define NODES_DEFINITION_H



#include <string>




// ***** BEGIN: MODULE NAMES *****
const std::string MODULE_NAME_ODOMETRY_STATE_ESTIMATOR   = "droneStateEstimator";
const std::string MODULE_NAME_TRAJECTORY_CONTROLLER      = "droneController";
const std::string MODULE_NAME_ARUCO_EYE                  = "droneArucoEyeROSModule";
const std::string MODULE_NAME_LOCALIZER                  = "droneLocalizer";
const std::string MODULE_NAME_OBSTACLE_PROCESSOR         = "droneObstacleProcessor";
const std::string MODULE_NAME_TRAJECTORY_PLANNER         = "droneTrajectoryPlanner";
const std::string MODULE_NAME_MISSION_PLANNER            = "droneMissionPlanner";
const std::string MODULE_NAME_YAW_PLANNER				 = "droneYawPlanner";
const std::string MODULE_NAME_ARCHITECTURE_BRAIN         = "droneArchitectureBrain";
const std::string MODULE_NAME_DRONE_LOGGER               = "droneLogger";

const std::string MODULE_NAME_DRONE_CONSOLE_INTERFACE    = "droneInterface";

const std::string MODULE_NAME_DRIVER_PARROT_ALTITUDE    = "droneAltitude";
const std::string MODULE_NAME_DRIVER_PARROT_STATUS    = "droneDroneStatus";
const std::string MODULE_NAME_DRIVER_PARROT_BATTERY    = "droneBattery";
const std::string MODULE_NAME_DRIVER_PARROT_GROUND_SPEED    = "droneGroundSpeed";
const std::string MODULE_NAME_DRIVER_PARROT_IMU    = "droneImu";
const std::string MODULE_NAME_DRIVER_PARROT_MAGNETOMETER    = "droneMagnetometer";
const std::string MODULE_NAME_DRIVER_PARROT_PRESSURE    = "dronePressure";
const std::string MODULE_NAME_DRIVER_PARROT_ROTATION_ANGLES    = "droneRotationAngles";
const std::string MODULE_NAME_DRIVER_PARROT_TEMPERATURE    = "droneTemperature";
const std::string MODULE_NAME_DRIVER_PARROT_FRONT_CAMERA    = "droneFrontCamera";
const std::string MODULE_NAME_DRIVER_PARROT_BOTTOM_CAMERA    = "droneBottomCamera";
const std::string MODULE_NAME_DRIVER_PARROT_DRONE_COMMAND    = "droneCommand";

const std::string MODULE_NAME_DRIVER_PELICAN_ALTITUDE    = "droneAltitude";
const std::string MODULE_NAME_DRIVER_PELICAN_BATTERY    = "droneBattery";
const std::string MODULE_NAME_DRIVER_PELICAN_GROUND_SPEED    = "droneGroundSpeed";
const std::string MODULE_NAME_DRIVER_PELICAN_ROTATION_ANGLES    = "droneRotationAngles";

const std::string MODULE_NAME_DRONE_SIMULATOR    = "droneSimulator";
const std::string MODULE_NAME_DRONE_VISUAL_MARKERS_EYE_SIMULATOR    = "droneArucoEyeROSModule";

const std::string MODULE_NAME_DRONE_ARCHITECTURE_RVIZ_INTERFACE    = "droneArchitectureRvizInterfaceROSModule";

namespace ModuleNames {
    enum name { ODOMETRY_STATE_ESTIMATOR = 1,
                TRAJECTORY_CONTROLLER,
                ARUCO_EYE,
                LOCALIZER,
                OBSTACLE_PROCESSOR,
                TRAJECTORY_PLANNER,
                MISSION_PLANNER,
                YAW_PLANNER,
                ARCHITECTURE_BRAIN
              };
}
// ***** END:   MODULE NAMES *****







#endif
