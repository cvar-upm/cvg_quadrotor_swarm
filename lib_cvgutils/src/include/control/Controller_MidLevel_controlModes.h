/*
 * control/Controller_MidLevel_controlModes.h
 *
 *  Created on: Nov 26, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_MIDLEVEL_CONTROLMODES_H_
#define CONTROLLER_MIDLEVEL_CONTROLMODES_H_

// Control mode names declaration
// To avoid problems make sure that the corresponding enum<->int values
//   are the same as in droneMsgsROS/droneTrajectoryControllerControlMode.h

namespace Controller_MidLevel_controlMode
{
    enum controlMode
    {
		TRAJECTORY_CONTROL = 1,
        POSITION_CONTROL   = 2,
        SPEED_CONTROL      = 3,
        UNKNOWN_CONTROL_MODE = 1000
	};
}


#endif /* CONTROLLER_MIDLEVEL_CONTROLMODES_H_ */
