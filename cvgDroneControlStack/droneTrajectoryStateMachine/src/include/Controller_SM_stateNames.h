/*
 * SM_stateNames.h
 *
 *  Created on: Jun 26, 2012
 *      Author: jespestana
 */

#ifndef SM_STATENAMES_H_
#define SM_STATENAMES_H_

// State names declaration
namespace SM_stateNames {
	enum stateNames {
		STRAIGHT = 1,
		TURN,
		HOVER,
		POSITION_CONTROL,
		SPEED_CONTROL
	};
}


#endif /* SM_STATENAMES_H_ */
