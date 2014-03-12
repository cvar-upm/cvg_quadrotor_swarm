#ifndef BRAIN_STATES_H
#define BRAIN_STATES_H

namespace BrainStates {
    enum StateType {
        INIT_SEQUENCE               =  0,
        LANDED                      =  1,
        TAKE_OFF_SEQUENCE           =  2,
        ON_BOARD_HOVERING           =  3,
        CONTROL_INIT_SEQUENCE       =  4,
        OFF_BOARD_CONTROL           =  5,
        CONTROL_STOP_SEQUENCE       =  6,
        LAND_SEQUENCE               =  7,
        WIFIDOWN_EMERGENCY_SEQUENCE =  8,
        WIFIDOWN_ON_BOARD_HOVERING  =  9,
        WIFIDOWN_RECOVER_SEQUENCE   = 10,
        EMERGENCY_LAND_SEQUENCE     = 11
    };
}

#endif // BRAIN_STATES_H
