#include <Logger.hpp>


Logger::Logger(std::string _logFileName,bool _debug, bool _error, bool _show ):
                debug_flag(_debug),error_flag(_error),show_flag(_show)
{
    logfile.open(_logFileName.append(".log"));
    if (!logfile.is_open())
        std::cerr << "Error in Opening logfile" << std::endl;
    
}

Logger::~Logger()
{
    if (logfile.is_open())
        logfile.close();
    logfile.flush();
}