#time
Header header

#HL command
int32 command

# Control mode names declaration
# To avoid problems make sure that the corresponding enum<->int values
#   are the same as in lib_cvgutils/src/include/control/Controller_MidLevel_controlModes.h

int32 TRAJECTORY_CONTROL = 1
int32 POSITION_CONTROL   = 2
int32 SPEED_CONTROL      = 3
int32 UNKNOWN_CONTROL_MODE = 1000
