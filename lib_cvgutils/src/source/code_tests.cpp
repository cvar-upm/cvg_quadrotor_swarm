#include "xmlfilereader.h"
#include <cstdlib>
#include <exception>
#include <vector>
#include <string>
#include "matrixLib.h"
#include "pruebastaticbool.h"
#include <unistd.h>
#include "cvg_utils_library.h"

template <typename T>
T my_function(int a, T t) {
    std::cout << "a: " << a << std::endl;
    return T();
}

enum class myPrivateEnum { RED = 0, YELLOW = 1, GREEN = -1};

int main(int argc,char **argv) {
    std::cout << "Compile test" << std::to_string(10) << std::endl;

    // general tests
    {
//    std::cout << BOOST_CURRENT_FUNCTION << std::endl;
//    std::cout << __func__ << std::endl;

//    int b = 0;
//    std::cout << "b++" << b++ << std::endl;
//    b = 0;
//    std::cout << "++b" << ++b << std::endl;

//    for ( int i = 0; i<1; i++) {
//        std::cout << "i:" << i << std::endl;
//    }

//    char c = '\'';
//    if ( c == '\'' )
//        std::cout << "c == '\''" << std::endl;

//        int i = 0, j = 0;
//        int* pi = &i;
//        const int* pi = &i; // pointer to (const inst)
//        int const* pi = &i; // pointer to (const inst)
//        int* const pi = &i; // (const pointer) to int
//        const int* const pi = &i; // (const pointer) to (const inst)
//        (*pi) = 3;
//        pi = &j;
    }



    // Pruebas con XMLFileReader
    {
//        std::cout << "Your DRONE_STACK is located in: " << std::getenv("DRONE_STACK") << std::endl;

//        try {
//            XMLFileReader my_xml_reader(std::string(std::getenv("DRONE_STACK"))+"/configs/drone0/ekf_state_estimator_config.xml");
//            int result_int;
//            std::string result_string;
//            double result_double;
//            {
//                result_string = my_xml_reader.readStringValue( {"ekf_state_estimator_config","description"} );
//            std::cout << "result: " << result_string << std::endl;
//        }
//        {
//            static const std::vector<std::string> vr = {"ekf_state_estimator_config","module_frequency"};
//            result_double = my_xml_reader.readDoubleValue(vr);
//            std::cout << "result: " << result_double << std::endl;
//        }
//        {
//            static const std::vector<std::string> vr = {"ekf_state_estimator_config","state_model","number_of_states"};
//            result_int = my_xml_reader.readIntValue(vr);
//            std::cout << "result: " << result_int << std::endl;
//        }
//        {
//            static const std::vector<std::string> vr = {"ekf_state_estimator_config","state_model","delete_this_parameter"};
//            try {
////                result_string = my_xml_reader.readStringValue(vr);
//                result_int    = my_xml_reader.readIntValue(vr);
//                std::cout << "result_string: " << result_string << " result_string.size(): " << result_string.size() << std::endl;
//            } catch ( cvg_XMLFileReader_exception &e) {
//                throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
//            }

//            std::cout << "xml::value_location: {";
//            for (auto it : vr) {
//                std::cout << "\'" << it << "\';";
//            }   std::cout << "}" << std::endl;

//            std::cout << BOOST_CURRENT_FUNCTION << std::endl;
//        }
//    } catch (std::exception &e) {
//        std::cout << "lib_cvgutils:code_tests, exception: " << e.what() << std::endl;
//        return 0;
//    }

        // Prueba, como se leerian varios doubles de una lista de xmls
//        std::string prueba_varios_doubles = "6.30 3.55";
//        std::size_t pos;
//        double my_d = std::stod(prueba_varios_doubles, &pos);
//        std::cout << "my_d:" << my_d << " pos:" << pos << std::endl;
//        prueba_varios_doubles = prueba_varios_doubles.substr( pos);
//        my_d = std::stod(prueba_varios_doubles, &pos);
//        std::cout << "my_d:" << my_d << " pos:" << pos  << std::endl;
//        prueba_varios_doubles = prueba_varios_doubles.substr( pos);
//        my_d = std::stod(prueba_varios_doubles, &pos);
//        std::cout << "my_d:" << my_d << " pos:" << pos  << std::endl;


    }


    // prueba a definir un static const vector
    {
//    static const std::vector<std::string> vr = {"2", "3", "4"};
//    for (int i=0; i < vr.size(); i++) {
//        std::cout << vr[i] << " ";
//    } std::cout << std::endl;
    }


    // test OpenCV matrix multiplication
    {
//    cv::Mat A, B, C, D, t;
////    A = cv::Mat::eye(3,3,CV_32F);
//    B = cv::Mat::eye(4,4,CV_32F);
//    C = cv::Mat::zeros(3,1,CV_32F);
//    D = cv::Mat::zeros(3,1,CV_32F);
//    B.at<float>(0,0) = 2;
//    B.at<float>(0,2) = 3;
//    std::cout << "C: " << C << std::endl;
//    C.at<float>(0,0) = 1;
//    C.at<float>(1,0) = 0;
//    C.at<float>(2,0) = 3;
//    std::cout << "B: " << B << std::endl;
//    A = B(cv::Rect(0,0,3,3));
//    std::cout << "A: " << A << std::endl;
//    std::cout << "C: " << C << std::endl;
//    std::cout << "D: " << D << std::endl;
//    D = A*C;
//    std::cout << "D: " << D << std::endl;

//    A = B(cv::Rect(0,0,3,3));
//    t = B(cv::Rect(3,0,1,3));
//    A.at<float>(0,0) = 5;
//    B.at<float>(0,3) = 5;
//    std::cout << "B: " << B << std::endl;
//    std::cout << "A: " << A << std::endl;
//    std::cout << "t: " << t << std::endl;
//    A = A.t();
//    t = -A*t;
//    std::cout << "B, At: " << B << std::endl;
//    std::cout << "At: " << A << std::endl;
//    std::cout << "t, At*t: " << t << std::endl;
    }


    // prueba bool to integer conversion and viceversa
    {
//    int a;
//    bool a_b;
//    a = 1;
//    a_b = a;
//    std::cout << "(bool) = (int,1): " << a_b << std::endl;
//    a = 0;
//    a_b = a;
//    std::cout << "(bool) = (int,0): " << a_b << std::endl;
//    a_b = true;
//    a = a_b;
//    std::cout << "(int) = (bool,true): " << a_b << std::endl;
//    a_b = false;
//    a = a_b;
//    std::cout << "(int) = (bool,false): " << a_b << std::endl;
    }

    // prueba string comparison
    {
//        int idDrone = 0;
//        XMLFileReader my_xml_reader(std::string(std::getenv("DRONE_STACK"))+"/configs/drone"+std::to_string(idDrone)+"/trajectory_controller_config.xml");

//        std::string init_control_mode_str, control_mode;
//        init_control_mode_str = my_xml_reader.readStringValue( {"trajectory_controller_config","init_control_mode"} );
//    std::cout << "init_control_mode_str.compare(\"speed\"):"<< init_control_mode_str.compare("speed") << std::endl;
//    std::cout << "init_control_mode_str.compare(\"position\"):"<< init_control_mode_str.compare("position") << std::endl;
//    std::cout << "init_control_mode_str.compare(\"trajectory\"):"<< init_control_mode_str.compare("trajectory") << std::endl;
//    if ( init_control_mode_str.compare("speed") == 0 )
//        { control_mode = "speed"; }
//    else {
//        if ( init_control_mode_str.compare("position") == 0 )
//            { control_mode = "position";   }
//        else                            // "trajectory"
//            { control_mode = "trajectory"; }
//    }
//    std::cout << "init_control_mode_str: " << init_control_mode_str << " control_mode: " << control_mode << std::endl;
    }

    // Prueba initialization from xml_file
    {
//    for (int i=0; i<10; i++) {
//        std::cout << "i:" << i <<std::endl;
//        PruebaStaticBool my_trajectory_config(0);
//        PruebaStaticBool my_trajectory_config2(0);
//        usleep(1000000);
//    }
//        PruebaStaticBool my_trajectory_config(0);
//        PruebaStaticBool my_trajectory_config2(0);
//        PruebaStaticBool my_trajectory_config3(1);
    }

    // Prueba yaws, yawci
    {
        double yawci = ( 185.0)*M_PI/180.0;
        double yaws  = (   6.0)*M_PI/180.0;
        std::cout << "Yaw_Error: " << cvg_utils_library::getAngleError( yawci, yaws)*(180.0/M_PI) << std::endl;
//        std::cout << cvg_utils_library::getAngleError( NAN, NAN) << std::endl;

//        double yaw_test;
//        yaw_test = NAN;
//        std::cout << "yaw_test:"<< yaw_test << std::endl;
//        bool my_float_is_nan = false;
//        if ( isnan(yaw_test) ) {
//            my_float_is_nan = true;
//            std::cout << "yaw_test:"<< yaw_test << std::endl;
//            std::cout << "yaw_test:"<< yaw_test << std::endl;
//            std::cout << "yaw_test:"<< yaw_test << std::endl;
//        }
//        std::cout << "yaw_test:"<< yaw_test << std::endl;
    }

    // Prueba reference variables
    {
//        double x = 3.0;
//        double &rx = x;
//        std::cout << "x:" << x << " rx:" << rx << std::endl;
//        rx = 6.0;
//        std::cout << "x:" << x << " rx:" << rx << std::endl;
//        for (int i = 0; i<0; i++) {
//            std::cout << "i:" << i << std::endl;
//        }
    }

    // enum class
    {
//        myPrivateEnum my_enum = myPrivateEnum::RED;                             // enum class
////        std::cout << "my_enum = myPrivateEnum::RED:" << my_enum << std::endl; // no compila
////        my_enum = 1;                                                          // no compila
////        int a = my_enum;                                                      // no compila
//        std::cout << "(my_enum < myPrivateEnum::GREEN):" << (my_enum < myPrivateEnum::GREEN) << std::endl;
    }



    return 1;
}
