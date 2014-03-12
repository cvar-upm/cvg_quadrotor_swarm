#include "dronelogfileswritter.h"

DroneLogfilesWritter::DroneLogfilesWritter(std::string stackPath) :
    init_date(boost::gregorian::day_clock::local_day()),
    init_time(boost::posix_time::microsec_clock::local_time()),
    eventslog_out(&eventslog_buf),
    flight_diary_out(&flight_diary_buf)
    {
    // creating events_logfile and fligth_diary_logfile
    std::stringstream s0, s1, s2, s3, s4, msg;
    s0 << stackPath << "logs/drone_logger";
    s1 << s0.str()
       << "/date_"
       << "y" << (int) init_date.year()
       << "m" << std::setfill('0') << std::setw(2) << (int) init_date.month()
       << "d" << std::setfill('0') << std::setw(2) << (int) init_date.day();
    s2 << s1.str() << "/time_" << init_time.time_of_day();
    s3 << s2.str() << "/frontImage";
    s4 << s2.str() << "/bottomImage";

    fs::path logs_path; logs_path = fs::path(s0.str());
//    std::cout << "is_directory " << logs_path   << ": " << fs::is_directory(logs_path) << std::endl;
    if ( !(fs::is_directory(logs_path)) ) {
        fs::create_directory(logs_path);
    }

    fs::path daylogs_path = fs::path(s1.str());
//    std::cout << "is_directory " << daylogs_path << ": " << fs::is_directory(daylogs_path) << std::endl;
    if ( !(fs::is_directory(daylogs_path)) ) {
        fs::create_directory(daylogs_path);
    }

    currentlog_path = fs::path(s2.str());
//    std::cout << "is_directory " << currentlog_path << ": " << fs::is_directory(currentlog_path) << std::endl;
    if ( !(fs::is_directory(currentlog_path)) ) {
        fs::create_directory(currentlog_path);
    }


    if ( !(fs::is_directory(fs::path(s3.str()))) )
    {
        fs::create_directory(fs::path(s3.str()));
    }

    if ( !(fs::is_directory(fs::path(s4.str()))) )
    {
        fs::create_directory(fs::path(s4.str()));
    }

    eventslog_buf.open( (currentlog_path / fs::path("events_log.txt")).string() );

    flight_diary_buf.open( (currentlog_path / fs::path("flight_diary.txt")).string() );
    flight_diary_out << "Start time: " << init_time.time_of_day()
                     << " of"
                     << " year:" << (int) init_date.year()
                     << " month:" << std::setfill('0') << std::setw(2) << (int) init_date.month()
                     << " day:" << std::setfill('0') << std::setw(2) << (int) init_date.day()
                     << std::endl;
    return;
}

DroneLogfilesWritter::~DroneLogfilesWritter() {
    // Note: for the buffer to be emptied into the logfile, the program must be closed with control+c
    eventslog_buf.close();
    flight_diary_buf.close();
}

void DroneLogfilesWritter::writeString2Logfile(std::string string_in) {
    eventslog_out << string_in;
}
