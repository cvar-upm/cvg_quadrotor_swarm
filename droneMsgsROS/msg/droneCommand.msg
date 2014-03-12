#time
Header header


#HL command
int32 command 

# Drone State Command names declaration:
#  [*] To avoid problems make sure that the corresponding enum<->int values
#  are the same as in lib_cvgutils/src/include/drone_utils/drone_state_command_enum.h

int32 TAKE_OFF = 1
int32 HOVER    = 2
int32 LAND     = 3
int32 MOVE     = 4
int32 RESET    = 5


int32 UNKNOWN=1000
