#include "droneEKFStateEstimator.h"
#include "xmlfilereader.h"
#define DEG2RAD (M_PI/180.0)

using namespace CVG;

// Constructor, Destructor, Reset and configuration
DroneEKFStateEstimator::DroneEKFStateEstimator(int idDrone, const std::string &stackPath_in) {
    try {
        XMLFileReader my_xml_reader(stackPath_in+"configs/drone"+std::to_string(idDrone)+"/ekf_state_estimator_config.xml");
        double xml_read_double = 0.0;

        maximum_ground_optical_flow_measurement = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","ground_optical_flow_maximum_speed"} );

        //Init State Estimator
        numStates   = my_xml_reader.readIntValue( {"ekf_state_estimator_config","state_model","number_of_states"} );
        numInputs   = my_xml_reader.readIntValue( {"ekf_state_estimator_config","state_model","commands","number_of_inputs"} );
        numMeasures = my_xml_reader.readIntValue( {"ekf_state_estimator_config","observation_model","number_of_measurements"} );
        EKF.creation( numStates, numInputs, numMeasures);

        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","integration_time"} );
        multirotorModel.setMaxIntegrationTime(xml_read_double);
        EKF.initModel(&multirotorModel);

        //Inicializamos otras variables
        EstimatedState.creation(numStates);
        RealActuation.creation(numInputs);
        FlagObservers.creation(numMeasures);    // Flags to be activated when a measurement is obtained
        RealObservation.creation(numMeasures);
        EstimatedObservation.creation(numMeasures);

        /////// Observation variances
        VarObservation_EKF.creation(numMeasures);
        // Odometry measurements
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","pitch"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double 	*M_PI/180.0)      , 2 ) ,1); // Pitch
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","roll"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double 	*M_PI/180.0)      , 2 ) ,2); // Roll
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","yaw_odometry"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double*M_PI/180.0)  , 2 ) ,4); // Yaw
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","z"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double					 )      , 2 ) ,6); // Z
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","vx_odometry"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double				 )      , 2 ) ,11);// Vxm
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","vy_odometry"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double 			 )      , 2 ) ,12);// Vym
        // Position measurements
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","x"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double					 )      , 2 ) ,7); // X position (PX)
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","y"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double					 )      , 2 ) ,8); // Y position (PY)
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","yaw_position"} );
        VarObservation_EKF.setValueData( pow( ( xml_read_double*M_PI/180.0)     , 2 ) ,13); // Yaw
        // Other measurements
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","observation_model","standard_deviation","unused"} );
        VarObservation_EKF.setValueData( xml_read_double ,3); // dYaw/dt
        VarObservation_EKF.setValueData( xml_read_double ,5); // dZ/dt
        VarObservation_EKF.setValueData( xml_read_double ,9); // Vx
        VarObservation_EKF.setValueData( xml_read_double ,10);// Vy

        /////// Command variances
        VarActuation_EKF.creation(numInputs);
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","commands","standard_deviation","pitch"} );
        VarActuation_EKF.setValueData( pow( ( xml_read_double		/24)   , 2 ) ,1);  // Pitch
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","commands","standard_deviation","roll"} );
        VarActuation_EKF.setValueData( pow( ( xml_read_double 		/24)   , 2 ) ,2);  // Roll
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","commands","standard_deviation","dyaw"} );
        VarActuation_EKF.setValueData( pow( ( xml_read_double	    /100)  , 2 ) ,3);  // dYaw/dt
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","commands","standard_deviation","dz"} );
        VarActuation_EKF.setValueData( pow( ( xml_read_double		/1)    , 2 ) ,4);  // dZ/dt

        /////// Initial state estimation variances
        VarStateModel_EKF.creation(numStates);
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","standard_deviation","yaw"} );
        VarStateModel_EKF.setValueData( pow( xml_read_double*M_PI/180.0 ,2) , 5);
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","standard_deviation","z"} );
        VarStateModel_EKF.setValueData( pow( xml_read_double	 ,2) , 8);
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","standard_deviation","x"} );
        VarStateModel_EKF.setValueData( pow( xml_read_double	 ,2) , 9);
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","standard_deviation","y"} );
        VarStateModel_EKF.setValueData( pow( xml_read_double	 ,2) ,10);
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","standard_deviation","vx"} );
        VarStateModel_EKF.setValueData( pow( xml_read_double	 ,2) ,11);
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","standard_deviation","vy"} );
        VarStateModel_EKF.setValueData( pow( xml_read_double	 ,2) ,12);

        MatPInit.creation(numStates,numStates);
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","pitch"} );
        MatPInit.setValueData( pow( (( xml_read_double  *M_PI/180)/58.227) , 2 ) ,1 ,1 ); //  X1 : proportional to pitch
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","roll"} );
        MatPInit.setValueData( pow( (( xml_read_double   *M_PI/180)/38.575) , 2 ) ,2 ,2 ); //  X2 : proportional to roll
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","dyaw1"} );
        MatPInit.setValueData( pow( (( xml_read_double*M_PI/180)/67.213) , 2 ) ,3 ,3 ); //  X3 : internal variable related to d(yaw)/dt
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","dyaw2"} );
        MatPInit.setValueData( pow( (( xml_read_double*M_PI/180)/67.213) , 2 ) ,4 ,4 ); //  X4 : proportional to d(yaw)/dt
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","yaw"} );
        MatPInit.setValueData( pow( (( xml_read_double    *M_PI/180)       ) , 2 ) ,5 ,5 ); //  X5 : yaw
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","dz1"} );
        MatPInit.setValueData( pow( (  xml_read_double   /14.189)          , 2 ) ,6 ,6 ); //  X6 : internal variable related to d(Z)/dt
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","dz2"} );
        MatPInit.setValueData( pow( (  xml_read_double   /14.189)          , 2 ) ,7 ,7 ); //  X7 : proportional to d(Z)/dt
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","z"} );
        MatPInit.setValueData( pow( (  xml_read_double       )                 , 2 ) ,8 ,8 ); //  X8 : Z
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","x"} );
        MatPInit.setValueData( pow( (  xml_read_double       )                 , 2 ) ,9 ,9 );	//  X9 : X
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","y"} );
        MatPInit.setValueData( pow( (  xml_read_double       )                 , 2 ) ,10,10);	//  X10: Y
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","vx"} );
        MatPInit.setValueData( pow( (  xml_read_double    )                 , 2 ) ,11,11);	//  X11: VX
        xml_read_double = my_xml_reader.readDoubleValue( {"ekf_state_estimator_config","state_model","initial_state","standard_deviation","vy"} );
        MatPInit.setValueData( pow( (  xml_read_double    )                 , 2 ) ,12,12);	//  X12: VY

        MatVarEstimatedState.creation(numStates,numStates);
        MatVarOutput.creation(numMeasures,numMeasures);

        EKF.setVarActuation(&VarActuation_EKF);
        EKF.setVarObservation(&VarObservation_EKF);
        EKF.setVarState(&VarStateModel_EKF);

        //initial state + time
        resetStateEstimator();

        // Transformation between different reference frames, see documentation for more information
        matHomog_drone_GMR_t0_wrt_GFF       = cv::Mat::eye(4,4,CV_32F);
        matHomog_drone_LMrT_wrt_drone_GMR   = cv::Mat::eye(4,4,CV_32F);
        matHomog_drone_GMR_wrt_drone_LMrT   = cv::Mat::eye(4,4,CV_32F);
        matHomog_EKF_LMrT_wrt_GFF           = cv::Mat::eye(4,4,CV_32F);
        matHomog_EKF_LMrT_wrt_drone_LMrT_t0 = cv::Mat::eye(4,4,CV_32F);
        matHomog_droneLMrT_wrt_EKF_LMrT     = cv::Mat::eye(4,4,CV_32F);
        matHomog_drone_GMR_wrt_GFF          = cv::Mat::eye(4,4,CV_32F);
        matRotation_EKF_LMrT_wrt_GFF        = cv::Mat::eye(3,3,CV_32F);
        matRotation_EKF_LMrT_wrt_GFF_only_yaw=cv::Mat::eye(3,3,CV_32F);
        vecSpeed_drone_LMrT_wrt_EKF_LMrT    = cv::Mat::zeros(3,1,CV_32F);
        vecSpeed_drone_LMrT_wrt_GFF         = cv::Mat::zeros(3,1,CV_32F);

        float x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux;
        x_aux = my_xml_reader.readDoubleValue( {"take_off_site","position","x"} );
        y_aux = my_xml_reader.readDoubleValue( {"take_off_site","position","y"} );
        z_aux = my_xml_reader.readDoubleValue( {"take_off_site","position","z"} );
        yaw_aux = my_xml_reader.readDoubleValue( {"take_off_site","attitude","yaw"} );
        pitch_aux = my_xml_reader.readDoubleValue( {"take_off_site","attitude","pitch"} );
        roll_aux = my_xml_reader.readDoubleValue( {"take_off_site","attitude","roll"} );
        yaw_aux   = (M_PI/180.0)*yaw_aux;
        pitch_aux = (M_PI/180.0)*pitch_aux;
        roll_aux  = (M_PI/180.0)*roll_aux;
        //    // Homogeneous tranforms
        //    cv::Mat matHomog_drone_GMR_t0_wrt_GFF;          // from droneXX.xml configFile
        //    cv::Mat matHomog_drone_LMrT_wrt_drone_GMR;      // constant, see reference frames' definition (see pdf)
        //    cv::Mat matHomog_drone_GMR_wrt_drone_LMrT;      // constant, see reference frames' definition (see pdf)
        //    cv::Mat matHomog_EKF_LMrT_wrt_drone_LMrT_t0;    // from initial yaw
        //    cv::Mat matHomog_EKF_LMrT_wrt_GFF;              // to obtain SLAMs required transform from EKF data
        //    cv::Mat matHomog_droneLMrT_wrt_EKF_LMrT;        // from EKF
        referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_GMR_t0_wrt_GFF,      x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux);
        referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_LMrT_wrt_drone_GMR,    0.0,   0.0,   0.0,     0.0,       0.0,      M_PI);
        referenceFrames::createHomogMatrix_wYvPuR( &matHomog_drone_GMR_wrt_drone_LMrT,    0.0,   0.0,   0.0,     0.0,       0.0,     -M_PI);
    //  matHomog_EKF_LMrT_wrt_drone_LMrT_t0;  // is calculated calling the service setDroneYawInitSrv
        matHomog_EKF_LMrT_wrt_GFF = matHomog_drone_GMR_t0_wrt_GFF*matHomog_drone_LMrT_wrt_drone_GMR*matHomog_EKF_LMrT_wrt_drone_LMrT_t0;
        matRotation_EKF_LMrT_wrt_GFF = matHomog_EKF_LMrT_wrt_GFF(cv::Rect(0,0,3,3));
        double yaw = atan2(  matRotation_EKF_LMrT_wrt_GFF.at<float>(1,0), matRotation_EKF_LMrT_wrt_GFF.at<float>(0,0));
        referenceFrames::createRotMatrix_wYvPuR(&matRotation_EKF_LMrT_wrt_GFF_only_yaw, yaw, 0.0, M_PI);
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}

DroneEKFStateEstimator::~DroneEKFStateEstimator() {}

bool DroneEKFStateEstimator::resetStateEstimator() {
    //Init State
    EstimatedState.setZeros();
    EKF.setInitialState(&EstimatedState);
    EKF.setVarInitState(&MatPInit);

    //time
    timer.restart(false);
    double initTime = timer.getElapsedSeconds();
    EKF.setInitTime(initTime);

    return true;
}

bool DroneEKFStateEstimator::startStateEstimator() {
    //time
    double initTime = timer.getElapsedSeconds();
    EKF.setInitTime(initTime);

    return true;
}

inline void DroneEKFStateEstimator::setModelIntputGains(double gain_pitch, double gain_roll, double gain_dyaw, double gain_dz) {
    multirotorModel.setIntputGains(gain_pitch, gain_roll, gain_dyaw, gain_dz);
}


// EKF State Estimator inputs and state estimation funtion
//  Measurements:
//  01 pitch: C(1,1)*X1
//  02 roll : C(2,2)*X2
//  03 dYaw : C(3,4)*X4
//  04 Yaw  : X5
//  05 dZ   : C(5,7)*X7
//  06 Z    : X8
//  07 X    : X9					"VISION"
//  08 Y    : X10					"VISION"
//  09 Vx   : X11
//  10 Vy   : X12
//  11 [Vxm;: [R_Y R_Y]' * [X11;
//  12 Vym] : [R_Y R_Y]     X12]
//  13 Yaw  : X5					"VISION"
int DroneEKFStateEstimator::setDroneMeasurementRotationAngles(float roll, float pitch, float yaw) {
    float pitch_rad = pitch*DEG2RAD;
    float roll_rad  = roll*DEG2RAD;
    float yaw_rad   = yaw*DEG2RAD;

    // |yaw_EKF_rad - yaw_rad| < 2*M_PI
    EKF.getEstimatedOutput(&EstimatedObservation);
    float yaw_EKF_rad = EstimatedObservation.getValueData( 4 );
    yaw_EKF_rad = cvg_utils_library::mapAnglesToBeNear_PIrads( yaw_EKF_rad, yaw_rad);
    EstimatedState.setValueData(yaw_EKF_rad, 5);
    EKF.setInitialState(&EstimatedState);

    RealObservation.setValueData( yaw_rad    , 4);
    RealObservation.setValueData( pitch_rad  , 1);
    RealObservation.setValueData( roll_rad   , 2);
    FlagObservers.setValueData( 1.0, 4 );
    FlagObservers.setValueData( 1.0, 1 );
    FlagObservers.setValueData( 1.0, 2 );
    return 1;
}
int DroneEKFStateEstimator::setDroneMeasurementGroundOpticalFlow(float vx, float vy) {
    // droneMsgs::droneNavData reference frame convention
    // Speed limit -> to avoid bad measurements in the speed
    float v = sqrt(vx*vx + vy*vy);
    if ( (maximum_ground_optical_flow_measurement > 0.0) || (v <= maximum_ground_optical_flow_measurement) ) {
        RealObservation.setValueData( vx, 11);
        FlagObservers.setValueData( 1.0, 11 );
        RealObservation.setValueData( vy, 12);
        FlagObservers.setValueData( 1.0, 12 );
    } else {
        FlagObservers.setValueData( 0.0, 11 );
        FlagObservers.setValueData( 0.0, 12 );
    }
    return 1;
}
int DroneEKFStateEstimator::setDroneMeasurementAltitude(float altitude) {
    RealObservation.setValueData( altitude,6);
    FlagObservers.setValueData( 1.0, 6 );
    return 1;
}

//  Input commands:
//  01 pitch: U1
//  02 roll : U2
//  03 dYaw : U3
//  04 dAlt : U4
int DroneEKFStateEstimator::setDroneCommands( float pitch, float roll, float dyaw, float dalt) {                        // ROSdatatype ??::??
    // Modify RealActuation
    setDroneCommandPitch(pitch);
    setDroneCommandRoll( roll);
    setDroneCommandDYaw( dyaw);
    setDroneCommandDAlt( dalt);
    return 1;
}
void DroneEKFStateEstimator::setDroneCommandPitch(float pitch) {
    RealActuation.setValueData( pitch, 1);
}
void DroneEKFStateEstimator::setDroneCommandRoll( float roll) {
    RealActuation.setValueData( roll,  2);
}
void DroneEKFStateEstimator::setDroneCommandDYaw( float dyaw) {
    RealActuation.setValueData( dyaw,  3);
}
void DroneEKFStateEstimator::setDroneCommandDAlt( float dalt) {
    RealActuation.setValueData( dalt,   4);
}

int DroneEKFStateEstimator::stateEstimation(float mahalanobisDistance) {
    EKF.setSystemMeasures(&RealObservation);
    EKF.setSystemInputs(&RealActuation);

    double actual_time = timer.getElapsedSeconds();
    EKF.setActualTime(actual_time);

    // Activate or deactivate every measurement
    for( int i=1; i<=numMeasures; i++) {
        if ( FlagObservers.getValueData(i) == 1.0 ) {
            EKF.activateSystemMeasure(i);
            FlagObservers.setValueData(0.0, i); // Deactivate flag, since we have already read the measurement
        } else {
            EKF.deactivateSystemMeasure(i);
        }
    }

    int output = EKF.stateEstimation(mahalanobisDistance);

    if(output) {
        EKF.getEstimatedState(&EstimatedState);
        EKF.getEstimatedStateVariances(&MatVarEstimatedState);
        EKF.getEstimatedOutput(&EstimatedObservation);
        EKF.getEstimatedOutputVariances(&MatVarOutput);

        float x, y, z, yaw, pitch, roll;
        x = EstimatedObservation.getValueData(7);
        y = EstimatedObservation.getValueData(8);
        z = EstimatedObservation.getValueData(6);
        yaw   = EstimatedObservation.getValueData(4);
        pitch = EstimatedObservation.getValueData(1);
        roll  = EstimatedObservation.getValueData(2);
        referenceFrames::createHomogMatrix_wYvPuR( &matHomog_droneLMrT_wrt_EKF_LMrT, x, y, z, yaw, pitch, roll);
        matHomog_drone_GMR_wrt_GFF = matHomog_EKF_LMrT_wrt_GFF*matHomog_droneLMrT_wrt_EKF_LMrT*matHomog_drone_GMR_wrt_drone_LMrT;
        matRotation_EKF_LMrT_wrt_GFF = matHomog_EKF_LMrT_wrt_GFF(cv::Rect(0,0,3,3));
        double yaw2 = atan2(  matRotation_EKF_LMrT_wrt_GFF.at<float>(1,0), matRotation_EKF_LMrT_wrt_GFF.at<float>(0,0));
        referenceFrames::createRotMatrix_wYvPuR(&matRotation_EKF_LMrT_wrt_GFF_only_yaw, yaw2, 0.0, M_PI);
        vecSpeed_drone_LMrT_wrt_EKF_LMrT.at<float>(0,0) = EstimatedObservation.getValueData( 9);
        vecSpeed_drone_LMrT_wrt_EKF_LMrT.at<float>(1,0) = EstimatedObservation.getValueData(10);
        vecSpeed_drone_LMrT_wrt_EKF_LMrT.at<float>(2,0) = EstimatedObservation.getValueData( 5);
        vecSpeed_drone_LMrT_wrt_GFF = matRotation_EKF_LMrT_wrt_GFF_only_yaw*vecSpeed_drone_LMrT_wrt_EKF_LMrT;
    }
    return output;
}

// EKF State Estimator outputs
void DroneEKFStateEstimator::getDroneEstimatedPose_LMrTwrtEKF(float *x, float *y, float *z, float *yaw, float *pitch, float *roll) {
    *x = EstimatedObservation.getValueData(7);
    *y = EstimatedObservation.getValueData(8);
    *z = EstimatedObservation.getValueData(6);
    *yaw   = EstimatedObservation.getValueData(4);
    *pitch = EstimatedObservation.getValueData(1);
    *roll  = EstimatedObservation.getValueData(2);
}

void DroneEKFStateEstimator::getDroneEstimatedSpeed_LMrTwrtEKF(float *dx, float *dy, float *dz, float *dyaw, float *dpitch, float *droll) {
    *dx = EstimatedObservation.getValueData(9);
    *dy = EstimatedObservation.getValueData(10);
    *dz = EstimatedObservation.getValueData(5);
    *dyaw   = EstimatedObservation.getValueData(3); // Never used!!
    *dpitch = 0.0; // Not defined!!! Never used!!
    *droll  = 0.0; // Not defined!!! Never used!!
}

void DroneEKFStateEstimator::getDroneEstimatedPose_GMRwrtGFF(float *x, float *y, float *z, float *yaw, float *pitch, float *roll) {
    double x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux;
    referenceFrames::getxyzYPRfromHomogMatrix_wYvPuR( matHomog_drone_GMR_wrt_GFF, &x_aux, &y_aux, &z_aux, &yaw_aux, &pitch_aux, &roll_aux);
    *x = x_aux; *y = y_aux; *z = z_aux;
    *yaw = yaw_aux; *pitch = pitch_aux; *roll = roll_aux;
}

void DroneEKFStateEstimator::getDroneEstimatedSpeed_GMRwrtGFF(float *dx, float *dy, float *dz, float *dyaw, float *dpitch, float *droll) {
    *dx = vecSpeed_drone_LMrT_wrt_GFF.at<float>(0,0);
    *dy = vecSpeed_drone_LMrT_wrt_GFF.at<float>(1,0);
    *dz = vecSpeed_drone_LMrT_wrt_GFF.at<float>(2,0);
    *dyaw   = -EstimatedObservation.getValueData(3); // Never used!!
    *dpitch =  0.0; // Not defined!!! Never used!!
    *droll  =  0.0; // Not defined!!! Never used!!
}

void DroneEKFStateEstimator::getEstimatedObservation(Vector *estObserv) {
    estObserv->copy(&EstimatedObservation);
    return;
}

void DroneEKFStateEstimator::setInitDroneYaw(double init_telemetry_yaw) {
    referenceFrames::createHomogMatrix_wYvPuR( &matHomog_EKF_LMrT_wrt_drone_LMrT_t0, 0.0, 0.0, 0.0, -init_telemetry_yaw, 0.0, 0.0);
    matHomog_EKF_LMrT_wrt_GFF = matHomog_drone_GMR_t0_wrt_GFF*matHomog_drone_LMrT_wrt_drone_GMR*matHomog_EKF_LMrT_wrt_drone_LMrT_t0;
    matRotation_EKF_LMrT_wrt_GFF = matHomog_EKF_LMrT_wrt_GFF(cv::Rect(0,0,3,3));
    double yaw = atan2(  matRotation_EKF_LMrT_wrt_GFF.at<float>(1,0), matRotation_EKF_LMrT_wrt_GFF.at<float>(0,0));
    referenceFrames::createRotMatrix_wYvPuR(&matRotation_EKF_LMrT_wrt_GFF_only_yaw, yaw, 0.0, M_PI);
}
