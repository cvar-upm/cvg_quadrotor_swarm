#include "xmlfilereader.h"
#include <cstdlib>
#include <exception>
#include <vector>
#include <string>
#include "matrixLib.h"
#include "pruebastaticbool.h"
#include <unistd.h>

int main(int argc,char **argv) {

    // Prueba initialization from xml_file
    {
    for (int i=0; i<10; i++) {
        std::cout << "i:" << i <<std::endl;
        PruebaStaticBool my_trajectory_config(1);
        PruebaStaticBool my_trajectory_config2(1);
        usleep(1000000);
    }
    }

    return 1;

}
