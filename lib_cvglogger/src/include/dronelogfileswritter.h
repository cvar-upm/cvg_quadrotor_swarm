#ifndef DRONELOGFILESWRITTER_H
#define DRONELOGFILESWRITTER_H

#include <iostream>
#include <string>
#include <sstream>
#include <ostream>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"
// Boost::date_time :
// http://www.boost.org/doc/libs/1_51_0/doc/html/date_time/posix_time.html#date_time.posix_time.ptime_class
#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o
//#include "boost/date_time/posix_time/posix_time_types.hpp" //no i/o just types
// http://www.boost.org/doc/libs/1_42_0/doc/html/date_time/examples.html
#include "boost/date_time/gregorian/gregorian.hpp"

// Class Description:
//  - This class creates two files, events_log.txt and flight_diary.txt.
//  - The idea is to encapsulate I/O for the DroneLoggerROSModule class and node
//  - The class automatically creates two folders <date>/<time_of_day> inside ${DRONE_STACK}/logs/drone_logger

namespace fs = boost::filesystem;
namespace io = boost::iostreams;

class DroneLogfilesWritter {
private:
    boost::gregorian::date      init_date;
    boost::posix_time::ptime    init_time;

    fs::path currentlog_path;
    io::stream_buffer<io::file_sink> eventslog_buf;
    std::ostream                     eventslog_out;
    io::stream_buffer<io::file_sink> flight_diary_buf;
    std::ostream                     flight_diary_out;

public:
    DroneLogfilesWritter(std::string stackPath);
    ~DroneLogfilesWritter();
    void writeString2Logfile(std::string string_in);

    inline std::string getCurrentLogPath() { return currentlog_path.string(); }
};

#endif // DRONELOGFILESWRITTER_H
