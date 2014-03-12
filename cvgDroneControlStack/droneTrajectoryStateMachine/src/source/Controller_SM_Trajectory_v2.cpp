/*
 * Controller_SM_Trajectory_v2.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#include "Controller_SM_Trajectory_v2.h"

using namespace CVG;

/* ***** TrajectoryWaypoint functions ***** */

// DONE: solve - Problema de glibc:
// 	  1) double free or corruption (fasttop)
//    2) habia otro que ponia corrupted linked list o algo asi que no consigo volver a provocar
//  3) poniendo  u0(0), r_ur(0) en el conscrutor initialization list no me pasa el problema.
// Posibles soluciones en internet:
//  http://cboard.cprogramming.com/c-programming/111240-double-free-corruption-fasttop.html
//  http://stackoverflow.com/questions/2403020/why-doesnt-the-c-default-destructor-destroy-my-objects
//  http://stackoverflow.com/questions/106508/what-is-a-smart-pointer-and-when-should-i-use-one
// Por lo que he leido puede ser que tengo que definir el comando delete y new para la clase Vector. Dbe ser que hay veces que
// se llama solo al destructor y otras que se llama a delete y al destructor causando doble destruccion... pero no estoy seguro.
// A parte, creo que es un problema con objetos temporales de los que crea C++.
// Dandole mas vueltas creo que es un problema con las shallow copies de JL.
// Relacionados con el problema de shallow copy:
//  - http://stackoverflow.com/questions/7297024/call-default-copy-constructor-from-within-overloaded-copy-constructor
//  - http://stackoverflow.com/questions/3279543/what-is-the-copy-and-swap-idiom
//  - http://stackoverflow.com/questions/4172722/what-is-the-rule-of-three
//  - http://en.wikipedia.org/wiki/Single_responsibility_principle ; "Sometimes you need to implement a class that manages
//     a resource. (Never manage multiple resources in a single class, this will only lead to pain.) "
TrajectoryWaypoint::TrajectoryWaypoint(double x, double y, double z) : u0(3), r_ur(3) {
	this->x = x;
	this->y = y;
	this->z = z;

	achieved_checkpoint = false;
	achieved_true_checkpoint = false;

	// default "empty" values
	// SM_stateNames::STRAIGHT mode parameters
	L = 0.0;
	s_end = 0.0;
//	r_ur(3);

	// SM_stateNames::TURN mode parameters
	R = 0.0;
	vc = 0.0;
	alim = 0.0;
//	u0(3);
}

TrajectoryWaypoint::~TrajectoryWaypoint() {
}

void TrajectoryWaypoint::convert2Vector(Vector& lv) {
	if ( lv.length() != 3 ) {
		lv.deletion();
		lv.creation(3);
	}
	lv.setValueData(this->x,1);
	lv.setValueData(this->y,2);
	lv.setValueData(this->z,3);
}

/* END: ***** TrajectoryWaypoint functions ***** */

/* ***** TrajectoryConfiguration functions ***** */
void TrajectoryConfiguration::set2defaultValues(int idDrone) {
    if ( (!xml_file_is_read) || (current_idDrone!=idDrone) ) {
        try {
            current_idDrone = idDrone;
            xml_file_is_read = true;
            std::cout << "Reading XML file, idDrone:" << current_idDrone << std::endl;
            XMLFileReader my_xml_reader( TrajectoryConfiguration::stackPath +"configs/drone"+std::to_string(current_idDrone)+"/trajectory_controller_config.xml");
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
        } catch ( cvg_XMLFileReader_exception &e) {
            throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
        }
    }

    chk_clearance_R		= default_chk_clearance_R;

    chk_R   	= default_chk_R;
    Rt_min  	= default_Rt_min;
    Rt_max  	= default_Rt_max;

    vmax_xy 	= default_vmax_xy;
    vmax_z  	= default_vmax_z;

    vstall_turn 		= default_vstall_turn;
    amax   				= default_amax;
    num_chk_speed_plan 	= default_num_chk_speed_plan;
    speed_tr 			= default_speed_tr;

    straighmode_safetyzone_radius_m = default_straighmode_safetyzone_radius_m;
    turnmode_safetyzone_radius_m	= default_turnmode_safetyzone_radius_m;
    turnmode_safetyzone_altitude_m	= default_turnmode_safetyzone_altitude_m;
    turnmode_safetyzone_negalpha_rad= default_turnmode_safetyzone_negalpha_rad;
}

// private static member variables initialization
bool TrajectoryConfiguration::xml_file_is_read = false;
int  TrajectoryConfiguration::current_idDrone  = -1;

std::string TrajectoryConfiguration::stackPath = std::string(std::getenv("DRONE_STACK"))+"/";

double TrajectoryConfiguration::default_chk_clearance_R = -1.0;

double TrajectoryConfiguration::default_chk_R  = -1.0;
double TrajectoryConfiguration::default_Rt_min = -1.0;
double TrajectoryConfiguration::default_Rt_max = -1.0;

double TrajectoryConfiguration::default_vmax_xy= -1.0;
double TrajectoryConfiguration::default_vmax_z = -1.0;

double TrajectoryConfiguration::default_vstall_turn        = -1.0;
double TrajectoryConfiguration::default_amax               = -1.0;
double TrajectoryConfiguration::default_num_chk_speed_plan = -1.0;
double TrajectoryConfiguration::default_speed_tr           = -1.0;

double TrajectoryConfiguration::default_straighmode_safetyzone_radius_m  = -1.0;
double TrajectoryConfiguration::default_turnmode_safetyzone_radius_m     = -1.0;
double TrajectoryConfiguration::default_turnmode_safetyzone_altitude_m   = -1.0;
double TrajectoryConfiguration::default_turnmode_safetyzone_negalpha_rad = -1.0;
/* END: ***** TrajectoryConfiguration functions ***** */

/* ***** DroneTrajectory functions ***** */

DroneTrajectory::DroneTrajectory(int idDrone) :
    error_waypoint(0.0, 0.0, -1000.0),
    traj_config(idDrone)
    {
	initial_checkpoint = 0;
	isPeriodic 		   = false;
}

DroneTrajectory::DroneTrajectory(int idDrone, double x_0, double y_0, double z_0) :
    error_waypoint(0.0, 0.0, -1000.0),
    traj_config(idDrone)
    {
	initial_checkpoint = 0;
	isPeriodic 		   = false;
	this->addWaypoint( x_0, y_0, z_0);
}

DroneTrajectory::DroneTrajectory(int idDrone, double *routeX, double *routeY, double *routeZ, int num_waypoints) :
    error_waypoint(0.0, 0.0, -1000.0),
    traj_config(idDrone)
    {
	initial_checkpoint = 0;
	isPeriodic 		   = false;
    for (int i = 0; i<num_waypoints; i++) {
		this->addWaypoint( routeX[i], routeY[i], routeZ[i]);
	}
}

DroneTrajectory::~DroneTrajectory() {
	this->clear();
}

bool DroneTrajectory::check() {
	return setInitialCheckpoint( initial_checkpoint );
}

bool DroneTrajectory::reset() {
    bool error_ocurred = this->check();

	for (int i = 0; i < (this->getInitialCheckpoint()-1); i++) {
		waypoints[i].achieved_checkpoint = true;
		waypoints[i].achieved_true_checkpoint = true;
	}

	return error_ocurred;
}

bool DroneTrajectory::setInitialCheckpoint(int initial_checkpoint) {
    bool error_ocurred = false;

    int N = getLength();
	if (initial_checkpoint > (N-1))
		initial_checkpoint = N-1;

	if (initial_checkpoint < 0)
		initial_checkpoint = 0;

	if ( N == 0 )
		error_ocurred = true; // trajectory is empty

	this->initial_checkpoint = initial_checkpoint;
	return error_ocurred;
}

int DroneTrajectory::incrementCheckpoint(int checkpoint) {

    int N = getLength();

	if ( checkpoint < 0 ) {
		checkpoint = 0;
		return checkpoint;
	}

	if (!isPeriodic) {
		if ( (checkpoint < N-1) )
			return cvg_utils_library::fmod( checkpoint+1, N);
	} else { // isPeriodic
		return cvg_utils_library::fmod( checkpoint+1, N);
	}

	return checkpoint;
}

int  DroneTrajectory::decrementCheckpoint(int checkpoint) {

    int N = getLength();

	if ( checkpoint < 0 ) {
		checkpoint = 0;
		return checkpoint;
	}

	if (!isPeriodic) {
		if ( (checkpoint > 0 ) )
			return cvg_utils_library::fmod( checkpoint-1, N);
		else    // checkpoint == 0
			return checkpoint;
	} else {    // isPeriodic
		return cvg_utils_library::fmod( checkpoint-1, N);
	}

	return checkpoint;
}

bool DroneTrajectory::routeFinished(int checkpoint) {

	if (isPeriodic)
		return false;
	else {
        int last_checkpoint = (this->getLength() - 1);
//		std::cout << "(chk == last_chk): " << ( checkpoint == last_checkpoint ) <<
//					" achieved(lst_chk): " << isAchievedCheckpoint(last_checkpoint) <<
//					" achievedTrue(lst_chk): " << isAchievedTrueCheckpoint(last_checkpoint) << std::endl;
		if ( ( checkpoint == last_checkpoint ) && isAchievedCheckpoint(last_checkpoint) && isAchievedTrueCheckpoint(last_checkpoint) )
			return true;
		else
			return false;
	}
}

TrajectoryWaypoint& DroneTrajectory::operator[](int i)
{
	if (this->empty()) {
		return error_waypoint; // waypoint_error
//		TrajectoryWaypoint waypoint_error( 0.0, 0.0, -1.0);
//		return waypoint_error;
	}
//	if (isPeriodic) {
//		int index = i % this->getLength();
//		index = index > 0 ? index : index + this->getLength();
//		return waypoints[index];
//	} else {
//		if (i<0)
//			return waypoints[0];
//		if (i >= (int) this->getLength())
//			return waypoints[this->getLength()-1];
//		return waypoints[i];
//	}

    int index = cvg_utils_library::fmod( i, this->getLength());
	return waypoints[index];
}

void DroneTrajectory::addWaypoint(double x, double y, double z) {
    waypoints.push_back(TrajectoryWaypoint( x, y, z));
	check();
}

bool DroneTrajectory::deleteLastWaypoint() {
    bool error_ocurred = false;
	if (this->empty()) {
		error_ocurred = true;
		return error_ocurred;
	}

	if (this->getLength() > 1)
		waypoints.pop_back();
	check();
	error_ocurred = false;
	return error_ocurred;
}

void DroneTrajectory::deleteFirstWaypoints(int until_checkpoint) {
	if (until_checkpoint < 0)
		return;
	if (until_checkpoint > (int) (this->getLength()-1) )
		return;

	waypoints.erase( waypoints.begin(), waypoints.begin()+(until_checkpoint+1));
	check();
}

void DroneTrajectory::clear() {
	waypoints.clear();
	initial_checkpoint = 0;
	isPeriodic = false;
}

bool DroneTrajectory::planify_trajectory(TrajectoryConfiguration *config, int segment, bool func_input_initialize_planning_variables) {
	// This is a recursive function that just to be called:
	//		- planify_trajectory(&config); // to planify whole trajectory with default checkpoint radius
	//		- planify trajectory(&config, segment_number, true); // planify trajectory from segment#segment_number onwards
	//		- the trajectory is always planified as if it was periodic (segment#(N-1) goes from chk#(N-1) to chk#0

//	std::cout << "DroneTrajectory::planify_trajectory( segment = " << segment << " )" << std::endl;

    bool error_ocurred;

    int N = getLength();
	if (N < 1) // error, trayectoria vacia
		return (error_ocurred = true);

	if ( segment < 1)
		return planify_trajectory( config, 1);

    bool initialize_planning_variables;
	if ( (segment == 1) || func_input_initialize_planning_variables ) // DO: initialize_planning_variables
		initializeAuxiliarPlanningVariables();
	else // DO NOT: initialize_planning_variables (do nothing)
		initialize_planning_variables = false;

	if ( N == 1 ) {
        int target_checkpoint = 0;
		waypoints[target_checkpoint].L     = -1.0;
		waypoints[target_checkpoint].s_end = -1.0;
		waypoints[target_checkpoint].R     = -1.0;
		waypoints[target_checkpoint].vc    = -1.0;
		waypoints[target_checkpoint].alim  = -1.0;
		if (waypoints[target_checkpoint].u0.length() != 3)
			waypoints[target_checkpoint].u0.creation(3);
		waypoints[target_checkpoint].u0.setValueData( 1.0, 3);
		if (waypoints[target_checkpoint].r_ur.length() != 3)
			waypoints[target_checkpoint].r_ur.creation(3);
		waypoints[target_checkpoint].r_ur.setValueData( 1.0, 3);
		traj_config = *config;
		error_ocurred = false;
		return error_ocurred;
	}

	// 1º) Calculate r_p0, r_p1, r_p2, r_ur, r_ur2, L, L2, rs_end <-si_no_hay_curva
    double L, L2;
	(*this)[segment-1].convert2Vector(r_p0);
	(*this)[segment].convert2Vector(r_p1);
	(*this)[segment+1].convert2Vector(r_p2);

//	cvg_utils_library::unitarizeVector(r_ur);
	L  = cvg_utils_library::unitaryVectorFrom2Points( r_ur, r_p0, r_p1);
	if ( L < config->chk_R ) {
        bool avoided2delete_waypoint0 = deleteWaypointInPlanning(segment);
//		return planify_trajectory( config, segment); // This case only occurs when segment == 1
		return planify_trajectory( config, 1);       // This case only occurs when segment == 1
	}

	L2 = cvg_utils_library::unitaryVectorFrom2Points( r_ur2, r_p1, r_p2);
	if ( L2 < config->chk_R ) {
        bool avoided2delete_waypoint0 = deleteWaypointInPlanning(incrementCheckpoint(segment));
		if (avoided2delete_waypoint0)
			return planify_trajectory( config, segment-1);
		else
			return planify_trajectory( config, segment);
	}

	rs_end = L - config->chk_R; // si no hay curva

	// 3º) Calcular c_u0, c_alim
	cvg_utils_library::crossProduct( c_u0, r_ur, r_ur2);
	cvg_utils_library::unitarizeVector( c_u0);

    double alpha = M_PI - cvg_utils_library::acos_ws(cvg_utils_library::dotProduct( r_ur, r_ur2));
    double d = config->chk_R/2.0;

	// 2º) Calcular c_Rt, c_vc
	c_Rt = d*sin(alpha/2.0)/(1-sin(alpha/2.0));

	if ( (c_Rt < config->Rt_min) || (c_Rt > config->Rt_max) ) {
		if (c_Rt < config->Rt_min)
			c_vc = config->vstall_turn;
		if (c_Rt > config->Rt_max)
			c_vc = cvg_utils_library::calculateVmax( r_ur, config->vmax_xy, config->vmax_z);
		c_Rt = -1.0;
		rs_end = L - config->chk_R;
	} else {
		c_vc = pow( c_Rt*config->amax, 0.5);
        double vaux = cvg_utils_library::calculateVmax( r_ur, config->vmax_xy, config->vmax_z);
		if (c_vc > vaux)
			c_vc = vaux;
		vaux = cvg_utils_library::calculateVmax( r_ur2, config->vmax_xy, config->vmax_z);
		if (c_vc > vaux)
			c_vc = vaux;
		rs_end = L - (c_Rt+d)*cos(alpha/2.0);
	}

	// 4º) Copy turn parameters to waypoint data
    int target_checkpoint = cvg_utils_library::fmod( segment, N);
	waypoints[target_checkpoint].L  = L;
	waypoints[target_checkpoint].s_end = rs_end;
	if (waypoints[target_checkpoint].r_ur.length() != 3)
		waypoints[target_checkpoint].r_ur.creation(3);
	waypoints[target_checkpoint].r_ur.copy(&r_ur);
	waypoints[target_checkpoint].R  = c_Rt;
	waypoints[target_checkpoint].vc = c_vc;
	waypoints[target_checkpoint].alim = M_PI - alpha;
	if (waypoints[target_checkpoint].u0.length() != 3)
		waypoints[target_checkpoint].u0.creation(3);
	waypoints[target_checkpoint].u0.copy(&c_u0);

	if ( segment > (N-1) ) {
		destroyAuxiliarPlanningVariables();
		traj_config = (*config);
		error_ocurred = false;
		return error_ocurred;
	} else { // then recursive function call
		return planify_trajectory( config, segment+1);
	}
}

bool DroneTrajectory::calculate_straight( int segment, Vector &r_p0, Vector &r_ur, Vector &r_ur2, double &rs_end, double &c_Rt) {
	segment = cvg_utils_library::fmod( segment, this->getLength() );
	// retrieve r_p0
	(*this)[segment-1].convert2Vector(r_p0);
	(*this)[segment].convert2Vector(r_p1);
	(*this)[segment+1].convert2Vector(r_p2);
	// r_ur, r_ur2
	cvg_utils_library::unitaryVectorFrom2Points( r_ur, r_p0, r_p1);
	cvg_utils_library::unitaryVectorFrom2Points( r_ur2, r_p1, r_p2);
	// rs_end
	rs_end = (*this)[segment].s_end;
	// c_Rt
	c_Rt = (*this)[segment].R;

    int N = getLength();
    bool is_last_checkpoint; // tells the caller whether this is the last straight segment
	if ( isPeriodic )
		is_last_checkpoint = false;
	else
		if ( segment == N-1 )
			is_last_checkpoint = true;
		else
			is_last_checkpoint = false;
	return is_last_checkpoint;
}

void DroneTrajectory::calculate_turn(int turn, Vector &c_pinit, Vector &c_pend, Vector &c_pc, double &c_alim, Vector &c_u0) {
	turn = cvg_utils_library::fmod( turn, this->getLength() );
	(*this)[turn-1].convert2Vector(r_p0);
	(*this)[turn].convert2Vector(r_p1);
	(*this)[turn+1].convert2Vector(r_p2);
	cvg_utils_library::unitaryVectorFrom2Points( r_ur, r_p0, r_p1);
	cvg_utils_library::unitaryVectorFrom2Points( r_ur2, r_p1, r_p2);

	// c_alim, c_u0
	c_alim = (*this)[turn].alim;
	c_u0.copy( &( (*this)[turn].u0 ) );

	// c_pinit
	aux_vector.copy(&r_ur);
	cvg_utils_library::multiplyDoubleVsVector( (*this)[turn].s_end, aux_vector);
	c_pinit.addition( &r_p0, &aux_vector);

	// c_pend
	aux_vector.copy(&r_ur2);
	cvg_utils_library::multiplyDoubleVsVector( (*this)[turn].L - (*this)[turn].s_end, aux_vector);
	c_pend.addition( &r_p1, &aux_vector);

	// c_pc
	cvg_utils_library::crossProduct( aux_vector, c_u0, r_ur);
	cvg_utils_library::multiplyDoubleVsVector( (*this)[turn].R, aux_vector);
	c_pc.addition( &c_pinit, &aux_vector);
}

double DroneTrajectory::obtainPlannedSpeed( int chk, double x, double y, double z, double v_current) {
	chk = cvg_utils_library::fmod( chk, getLength() );
    double v_max  = cvg_utils_library::calculateVmax( waypoints[chk].r_ur, traj_config.vmax_xy, traj_config.vmax_z);
    double v_plan = v_max, v_plani = v_max;

    double cummL = 0.0, dL_turn = 0.0, dL_trv = 0.0;
    double cummL_exit = ( v_current*v_current/(2*traj_config.amax) ) * 10.0;
    bool first_iteration = true;
    int  i_aux;

	for ( int i = chk; i < chk + traj_config.num_chk_speed_plan; i++ ) {
		i_aux = cvg_utils_library::fmod( i, getLength());
		if ( first_iteration ) {
			cummL = pow( pow( waypoints[chk].x - x, 2) + pow( waypoints[chk].y - y, 2) + pow( waypoints[chk].z - z, 2), 0.5);
			first_iteration = false;
		} else {
			cummL += waypoints[i_aux].L;
		}
		if ( waypoints[i_aux].R > 0 )
			dL_turn = ( waypoints[i_aux].R + traj_config.chk_R/2.0 )*sin( waypoints[i_aux].alim/2.0 );
		else
			dL_turn = 0.0;
		dL_trv = traj_config.speed_tr*fabs(v_current);

        double aux_cummL = (cummL - dL_turn - dL_trv);
		if ( aux_cummL < 0 )
			v_plani = fabs(waypoints[i_aux].vc);
		else
			v_plani = pow( pow(fabs(waypoints[i_aux].vc),2) + 2*traj_config.amax*aux_cummL, 0.5);

		if ( v_plani < v_plan )
			v_plan = v_plani;

		if ( aux_cummL > cummL_exit )
			break;
	}

	first_iteration = true;
	i_aux = cvg_utils_library::fmod( chk-1, getLength());
	for ( int i = i_aux; i > (chk-1) - traj_config.num_chk_speed_plan; i--) {
		i_aux = cvg_utils_library::fmod( i, getLength());

		if ( first_iteration ) {
			cummL = pow( pow( waypoints[i_aux].x - x, 2) + pow( waypoints[i_aux].y - y, 2) + pow( waypoints[i_aux].z - z, 2), 0.5);
			first_iteration = false;
		} else {
			cummL += waypoints[i_aux].L;
		}

		if ( waypoints[i_aux].R > 0 )
			dL_turn = ( waypoints[i_aux].R + traj_config.chk_R/2.0 )*sin( waypoints[i_aux].alim/2.0 );
		else
			dL_turn = 0.0;
//		dL_trv = traj_config.speed_tr*fabs(v_current);
//		double aux_cummL = (cummL - dL_turn - dL_trv);
		dL_trv = 0.0;
        double aux_cummL = (cummL - dL_turn);
//		double aux_cummL = cummL;
		if ( aux_cummL < 0 )
			v_plani = fabs(waypoints[i_aux].vc);
		else
			v_plani = pow( pow(fabs(waypoints[i_aux].vc),2) + 2*traj_config.amax*aux_cummL, 0.5);
//		v_plani = pow( pow(fabs(waypoints[i_aux].vc),2) + 2*traj_config.amax*cummL, 0.5);

		if ( v_plani < v_plan )
			v_plan = v_plani;

		if ( aux_cummL > cummL_exit )
			break;
	}

	return v_plan;
}

bool DroneTrajectory::deleteWaypointInPlanning(int checkpoint) {

    bool avoided2delete_waypoint0;

	if ( checkpoint == 0 ) {
		deleteLastWaypoint();
		avoided2delete_waypoint0 = true;
	} else {
		waypoints.erase( waypoints.begin() + checkpoint );
		avoided2delete_waypoint0 = false;
	}
	return avoided2delete_waypoint0;
}

void DroneTrajectory::initializeAuxiliarPlanningVariables() {
	pr_isPeriodic = isPeriodic;
//	isPeriodic = true; // 29 January 2014
    isPeriodic = false;

	aux_vector.creation(3);
	r_p0.creation(3); r_p1.creation(3); r_p2.creation(3);
	r_ur.creation(3); r_ur2.creation(3);
	c_u0.creation(3);
	rs_end = -1.0;
	c_Rt =   -1.0;
	c_vc =    0.0;
}

void DroneTrajectory::destroyAuxiliarPlanningVariables() {
	isPeriodic 	  = pr_isPeriodic;
	pr_isPeriodic = false;

//	aux_vector.deletion();
//	r_p0.deletion(); r_p1.deletion(); r_p2.deletion();	// used in calculate_straight(...)
//	r_ur.deletion(); r_ur2.deletion();					// used in calculate_turn(...)
	c_u0.deletion();
	rs_end = -1.0;
	c_Rt =   -1.0;
	c_vc =    0.0;
}

/* END: ***** DroneTrajectory functions ***** */

