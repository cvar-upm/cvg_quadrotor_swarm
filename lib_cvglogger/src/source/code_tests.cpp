//////////////////////////////////////////////////////
//  code_tests.cpp
//
//  Created on: Dec 11, 2013
//      Author: jespestana
//
//  Last modification on: Dec 11, 2013
//      Author: jespestana
//
//////////////////////////////////////////////////////

#include <iostream>
//#include <math.h>
//#include <cstdlib>
//#include <unistd.h>

#include "dronelogfileswritter.h"

#include <list>
#include <iostream>
#include <vector>

#include "debugstringstacker.h"
#include "stringstacker.h"

class Box
{
    // http://stackoverflow.com/questions/16501419/creating-a-class-with-a-static-member-container-holding-all-instances-of-said-cl
private:
    std::list <Box*>::iterator iter;
public:
    static std::list <Box*> List;
    Box() {
        List.push_front(this);
        iter = List.begin();
        std::cout << "Constructing box." << std::endl;
    }
    ~Box() {
        std::cout << "Trashing box." << std::endl;
        List.erase(iter);
    }
    void lookInside() {
        std::cout << "It's empty." << std::endl;
    };

    // JVC: added next two overloads:
    Box &operator=(Box const &src) {
        // don't assign anything.
        return *this;
    }
    Box(Box const &other) {
        List.push_front(this);
        iter = List.begin();
        std::cout << "Copy constructing box.\n";
    }
};

std::list <Box*> Box::List;

int main(int argc,char **argv) {
//        std::string stack_location_str(std::getenv("DRONE_STACK"));
//        stack_location_str += "/";
//        std::cout << "Your DRONE_STACK is located in: " << stack_location_str << std::endl;
//        DroneLogfilesWritter drone_logfile_writter(stack_location_str);
//        drone_logfile_writter.writeString2Logfile("Hola\nAdios\n"); // this works

    /* Prueba iteradores*/ {
//        {
//            Box Box_1;
//            Box Box_2;
//            {
//                Box Box_3;

//                Box_3 = Box_1; // No longer causes problem!

//                for (auto iter : Box::List) {
//                    iter->lookInside();
//                }
//                std::cout << "The list contains " << Box::List.size() << " boxes." << std::endl;
//            }

//            for (auto iter : Box::List) {
//                iter->lookInside();
//            }
//            std::cout << "The list contains " << Box::List.size() << " boxes." << std::endl;
//        }

//        for (auto iter : Box::List) {
//            iter->lookInside();
//        }
//        std::cout << "The list contains " << Box::List.size() << " boxes." << std::endl;

//        std::vector<int> v(10);
//        std::vector<int>::iterator it = v.begin() + 5;
//        std::insert_iterator<std::vector<int> > it_ins(v, it);

//        for (unsigned n = 20; n > 0; --n)
//            *it_ins++ = rand();
    }

    /* PRUEBA DebugStringStacker */
    {
//        std::string all_my_debug_strings;
//        bool priority_flag = false;
//        {
//            DebugStringStacker debug_string_stacker1;
//            all_my_debug_strings = DebugStringStacker::getAllStackedDebugStrings( priority_flag, false);
//            std::cout << "PRUEBA DebugStringStacker" << std::endl;
//            std::cout << all_my_debug_strings << "priority_flag:" << priority_flag << std::endl << std::endl;
//        }
//        DebugStringStacker debug_string_stacker2(1);
//        DebugStringStacker debug_string_stacker3;

//        {
//            DebugStringStacker debug_string_stacker4;
//            DebugStringStacker debug_string_stacker5;

//            //    debug_string_stacker1 << "prueba 1" << std::endl;
//            debug_string_stacker2 << "prueba 2, important data" << std::endl; debug_string_stacker2.setPriorityFlag();
//            debug_string_stacker3 << "prueba 3" << std::endl;
//            debug_string_stacker4 << "prueba 4" << std::endl;
//            debug_string_stacker5 << "prueba 5" << std::endl;

//            all_my_debug_strings = DebugStringStacker::getAllStackedDebugStrings( priority_flag, false);
//            std::cout << "PRUEBA DebugStringStacker" << std::endl;
//            std::cout << all_my_debug_strings << "priority_flag:" << priority_flag << std::endl << std::endl;

//            //    debug_string_stacker1 << "end prueba 1" << std::endl;
//            debug_string_stacker2 << "end prueba 2" << std::endl;
//            debug_string_stacker3 << "end prueba 3" << std::endl;
//            debug_string_stacker4 << "end prueba 4, important data" << std::endl; debug_string_stacker4.setPriorityFlag();
//            debug_string_stacker5 << "end prueba 5" << std::endl;
//            all_my_debug_strings = DebugStringStacker::getAllStackedDebugStrings( priority_flag, false);
//            std::cout << "PRUEBA DebugStringStacker" << std::endl;
//            std::cout << all_my_debug_strings << "priority_flag:" << priority_flag << std::endl << std::endl;
//        }

//        all_my_debug_strings = DebugStringStacker::getAllStackedDebugStrings(priority_flag);
//        std::cout << "PRUEBA DebugStringStacker" << std::endl;
//        std::cout << all_my_debug_strings << "priority_flag:" << priority_flag << std::endl << std::endl;

//        all_my_debug_strings = DebugStringStacker::getAllStackedDebugStrings(priority_flag, true, 1);
//        std::cout << "PRUEBA DebugStringStacker" << std::endl;
//        std::cout << all_my_debug_strings << "priority_flag:" << priority_flag << std::endl << std::endl;
    }

    StringStacker string_stacker;
    string_stacker << std::string("Esto es una prueba") << std::endl;
    string_stacker << std::string("Esto es una prueba otra vez") << std::endl;

    std::cout << string_stacker.getStackedString();

    string_stacker << std::string("Esto es una prueba 2") << std::endl;
    string_stacker << std::string("Esto es una prueba otra vez 2") << std::endl;

    std::cout << string_stacker.getStackedString();

    string_stacker << std::string("Esto es una prueba") << std::endl;
    string_stacker << std::string("Esto es una prueba otra vez") << std::endl;
    string_stacker << std::string("Esto es una prueba 2") << std::endl;
    string_stacker << std::string("Esto es una prueba otra vez 2") << std::endl;
    std::string line;
    for (int i=0; i<4; i++) {
        std::cout << std::string("Inside while loop") << std::endl;
        std::getline( string_stacker, line, '\n');
        std::cout << line << std::endl;
    }

    string_stacker << std::string("Esto es una prueba") << std::endl;
    string_stacker << std::string("Esto es una prueba otra vez") << std::endl;
    string_stacker << std::string("Esto es una prueba 2") << std::endl;
    string_stacker << std::string("Esto es una prueba otra vez 2") << std::endl;
    for (int i=0; i<4; i++) {
        std::cout << std::string("Inside while loop") << std::endl;
        std::getline( string_stacker, line, '\n');
        std::cout << line << std::endl;
    }

    return 0;
}
