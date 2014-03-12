#include "xmlfilereader.h"

XMLFileReader::XMLFileReader(std::string xml_file_str_in) { // throw(cvg_XMLFileReader_exception) {
    std::cout<< "XMLFileReader, opening file: " << xml_file_str << std::endl;

    xml_file_str = xml_file_str_in;
    std::ifstream xml_fstream(xml_file_str);
    pugi::xml_parse_result result = doc.load(xml_fstream);
    if(!result) {throw cvg_XMLFileReader_exception( std::string("XMLFileReader, could not open file \'")+xml_file_str+"\'"); return;}
    return;
}

double XMLFileReader::readDoubleValue(const std::vector<std::string> &value_location) { // throw(cvg_XMLFileReader_exception) {
    std::string readingValue = readStringValue(value_location, BOOST_CURRENT_FUNCTION);
    double result;
    try {
        result = std::stod(readingValue);
    } catch(std::invalid_argument &e) {
        throw cvg_XMLFileReader_exception( exception_string( BOOST_CURRENT_FUNCTION, value_location, "value is not a double") );
        return 0;
    }
    return result;
}

int XMLFileReader::readIntValue(const std::vector<std::string> &value_location) { // throw(cvg_XMLFileReader_exception) {
    std::string readingValue = readStringValue(value_location, BOOST_CURRENT_FUNCTION);
    int result;
    try {
        result = std::stoi(readingValue);
    } catch(std::invalid_argument &e) {
        throw cvg_XMLFileReader_exception( exception_string( BOOST_CURRENT_FUNCTION, value_location, "value is not an integer") );
        return 0;
    }
    return result;
}

std::string XMLFileReader::readStringValue(const std::vector<std::string> &value_location, std::string xmlfilereader_caller_function) { // throw(cvg_XMLFileReader_exception) {
    unsigned int n=value_location.size();
    if (n < 1) {throw cvg_XMLFileReader_exception("XMLFileReader::readStringValue, std::vector<std::string> value_location argument too short (length<1)."); return "";};

    std::string readingValue;
    if (n == 1) {
        readingValue=doc.child_value(value_location[n-1].c_str());
    } else {
        pugi::xml_node doc_node;
        for (unsigned int i=0; i<n-1;i++) {
            if (i==0)
                doc_node = doc.child(value_location[i].c_str());
            else
                doc_node = doc_node.child(value_location[i].c_str());
//            std::cout << "doc_node==pugi::xml_node(): "<< (doc_node!=pugi::xml_node()) << std::endl;
        }
        readingValue = doc_node.child_value(value_location[n-1].c_str());
    }

    // cvg_XMLFileReader_exception is thrown if the returned std::string is empty
    //                                   or if the specified path to the parameter is invalid
    if (readingValue.size() == 0)
        throw cvg_XMLFileReader_exception( exception_string( xmlfilereader_caller_function, value_location, "parameter empty or inexistent (specified xml_path to the parameter might be invalid)") );
    return readingValue;

}

std::string XMLFileReader::exception_string(std::string xmlfilereader_caller_function, const std::vector<std::string> &value_location, std::string aditional_exception_description) {
    std::string exception_description = "";
    exception_description += ", xmlfilereader_function: " + xmlfilereader_caller_function;
    exception_description += ", xml_file_path: " + xml_file_str;
    exception_description += ", parameter_location_in_xml: {";
    for (auto it : value_location) {
        exception_description += '\"' + it + '\"';
        if ( it != value_location.back())
            exception_description += ",";
    }   exception_description += "}";
    exception_description += ", aditional_exception_description: " + aditional_exception_description;
    return exception_description;
}
