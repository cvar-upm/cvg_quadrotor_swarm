#include <cstdlib>
#include <exception>
#include <vector>
#include <string>
#include "referenceFrames.h"

int main(int argc,char **argv) {
    std::cout << "Compile test" << std::to_string(10) << std::endl;


    // test OpenCV matrix multiplication
    {
    cv::Mat T1, T2;
    T1 = cv::Mat::eye(4,4,CV_32F);
    T2 = cv::Mat::eye(4,4,CV_32F);

    referenceFrames::createHomogMatrix_wYvPuR( &T1, 1.0, 0.0, 0.0, 90*(M_PI/180.0), 0.0, 0.0);
    referenceFrames::invertHomogMatrix( T1, T2);
    std::cout << "T1:\n" << T1 << std::endl;
    std::cout << "T2:\n" << T2 << std::endl;
    }



    return 1;
}

