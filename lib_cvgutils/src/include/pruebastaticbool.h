#ifndef PRUEBASTATICBOOL_H
#define PRUEBASTATICBOOL_H

#include <iostream>
#include "xmlfilereader.h"
#include <math.h>

class PruebaStaticBool
{
    // Default configuration values
private:
    static bool xml_file_is_read;
    static int  current_idDrone;
    static double default_chk_clearance_R;
    static double default_chk_R, default_Rt_min, default_Rt_max;
    static double default_vmax_xy, default_vmax_z;
    static double default_vstall_turn, default_amax, default_num_chk_speed_plan, default_speed_tr;
    static double default_straighmode_safetyzone_radius_m, default_turnmode_safetyzone_radius_m,
            default_turnmode_safetyzone_altitude_m, default_turnmode_safetyzone_negalpha_rad;

public:
    PruebaStaticBool(int idDrone);
};

#endif // PRUEBASTATICBOOL_H
