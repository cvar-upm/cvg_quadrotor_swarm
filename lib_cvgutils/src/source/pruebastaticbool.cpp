#include "pruebastaticbool.h"

PruebaStaticBool::PruebaStaticBool(int idDrone) {
    if ( (!xml_file_is_read) || (current_idDrone!=idDrone) ) {
        current_idDrone = idDrone;
        xml_file_is_read = true;
        std::cout << "Reading XML file, idDrone:" << current_idDrone << std::endl;
        XMLFileReader my_xml_reader(std::string(std::getenv("DRONE_STACK"))+"/configs/drone"+std::to_string(current_idDrone)+"/trajectory_controller_config.xml");
        default_chk_clearance_R = my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","checkpoint_clearance_radius"} );

        default_chk_R  = my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","trajectory_planning_params","checkpoint_radius"} );
        double turn_alpha_min;
        turn_alpha_min = my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","trajectory_planning_params","turn_alpha_min"} );
        double turn_alpha_max;
        turn_alpha_max = my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","trajectory_planning_params","turn_alpha_max"} );
        default_Rt_min = ( (default_chk_R/2.0) * sin(( turn_alpha_min *M_PI/180.0)/2.0)/(1-sin(( turn_alpha_min *M_PI/180.0)/2.0)) );
        default_Rt_max = ( (default_chk_R/2.0) * sin(( turn_alpha_max *M_PI/180.0)/2.0)/(1-sin(( turn_alpha_max *M_PI/180.0)/2.0)) );

        double dz_at_ratio, dz_max;
        dz_max    = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","drone_commands","dz_max"} );
        dz_at_ratio = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","speed_control_loop","dz_at_ratio"} );
        double vxy_at_ratio, vxy_max;
        vxy_max = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","speed_control_loop","vxy_max"} );
        vxy_at_ratio = my_xml_reader.readDoubleValue( {"trajectory_controller_config","general_saturations","speed_control_loop","vxy_at_ratio"} );

        default_vmax_xy = vxy_at_ratio*vxy_max; // vxy_at_max
        default_vmax_z  = dz_at_ratio*dz_max;   // dz_at_max

        default_vstall_turn 		= my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","speed_plan_saturations","stall_turn_speed_max"} );
        default_amax   				= my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","speed_plan_saturations","accel_max"} );
        default_num_chk_speed_plan 	= my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","speed_plan_saturations","delta_checkpoint"} );
        default_speed_tr 			= my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","speed_plan_saturations","speed_tr"} );

        default_straighmode_safetyzone_radius_m = my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","trajectory_mode_safety_zones_thresholds","straight_mode","distance_to_trajectory"} );
        default_turnmode_safetyzone_radius_m	= my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","trajectory_mode_safety_zones_thresholds","turn_mode","radius_distance"} );
        default_turnmode_safetyzone_altitude_m	= my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","trajectory_mode_safety_zones_thresholds","turn_mode","altitude_distance"} );
        default_turnmode_safetyzone_negalpha_rad= (M_PI/180.0)*my_xml_reader.readDoubleValue( {"trajectory_controller_config","state_machine","trajectory_mode_safety_zones_thresholds","turn_mode","negative_angle_position"} );
    }
}

// private static member variables initialization
bool PruebaStaticBool::xml_file_is_read = false;
int  PruebaStaticBool::current_idDrone  = -1;

double PruebaStaticBool::default_chk_clearance_R = -1.0;

double PruebaStaticBool::default_chk_R  = -1.0;
double PruebaStaticBool::default_Rt_min = -1.0;
double PruebaStaticBool::default_Rt_max = -1.0;

double PruebaStaticBool::default_vmax_xy= -1.0;
double PruebaStaticBool::default_vmax_z = -1.0;

double PruebaStaticBool::default_vstall_turn        = -1.0;
double PruebaStaticBool::default_amax               = -1.0;
double PruebaStaticBool::default_num_chk_speed_plan = -1.0;
double PruebaStaticBool::default_speed_tr           = -1.0;

double PruebaStaticBool::default_straighmode_safetyzone_radius_m  = -1.0;
double PruebaStaticBool::default_turnmode_safetyzone_radius_m     = -1.0;
double PruebaStaticBool::default_turnmode_safetyzone_altitude_m   = -1.0;
double PruebaStaticBool::default_turnmode_safetyzone_negalpha_rad = -1.0;
