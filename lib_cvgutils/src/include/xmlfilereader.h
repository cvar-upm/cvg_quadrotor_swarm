#ifndef XMLFILEREADER_H
#define XMLFILEREADER_H

#include "pugixml.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <string>
#include <boost/current_function.hpp>

// example ussage:
//     try {
//          XMLFileReader my_xml_reader(std::string(std::getenv("DRONE_STACK"))+"/configs/drone"+ std::to_string(idDrone)+"/trajectory_controller_config.xml");
//          set_moduleRate(my_xml_reader.readDoubleValue( {"trajectory_controller_config","module_frequency"} ));
//      } catch ( cvg_XMLFileReader_exception &e) {
//          throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
//      }

class cvg_XMLFileReader_exception : public std::runtime_error {
public:
//    using std::runtime_error::runtime_error;
    cvg_XMLFileReader_exception( std::string exception_string = "") : std::runtime_error(exception_string) {}
//    ~cvg_XMLFileReader_exception() {}
};

class XMLFileReader
{
public:
    XMLFileReader(std::string xml_file_str_in)                                  ;// throw(cvg_XMLFileReader_exception);
    double readDoubleValue(const std::vector<std::string> &value_location)      ;// throw(cvg_XMLFileReader_exception);
    int readIntValue(const std::vector<std::string> &value_location)            ;// throw(cvg_XMLFileReader_exception);
    std::string readStringValue(const std::vector<std::string> &value_location, std::string xmlfilereader_caller_function =
                    "std::string XMLFileReader::readStringValue(const std::vector<std::string>&, std::string)") ;// throw(cvg_XMLFileReader_exception);
private:
    pugi::xml_document doc;
    std::string xml_file_str;
    std::string exception_string(std::string xmlfilereader_caller_function, const std::vector<std::string> &value_location, std::string aditional_exception_description = "no additional information");
};



#endif // XMLFILEREADER_H
