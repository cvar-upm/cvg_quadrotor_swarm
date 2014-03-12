#ifndef DRONE_STATE_COMMAND_ENUM_H
#define DRONE_STATE_COMMAND_ENUM_H

namespace DroneStateCommand {
    enum StateCommand {
        TAKE_OFF = 1,
        HOVER    = 2,
        LAND     = 3,
        MOVE     = 4,
        RESET    = 5
    };
}

#endif // DRONE_STATE_COMMAND_ENUM_H
