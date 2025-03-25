#ifndef _FOO_H 
#define _FOO_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>

class Logger
{
private:
    std::ofstream logfile;
    bool debug_flag;
    bool error_flag;
    bool show_flag;
    std::string delimiter = ",";
    enum Color {BLACK = 90,RED,GREEN,YELLOW,BLUE,MAGENTA,CYAN,WHITE};
    enum LogLevel{ DEBUG = 0,INFO, WARNING, ERROR, CRITICAL};
    std::string BOLDRED = "\033[1m\033[31m";
    std::string RESET = "\033[0m";
    std::ostringstream logstream;
    char timestamp[50];
public:
    /**
     *   Logger constructor.
     *  args:
     *      _logFile: log filename;
     *      bool debug : whether to show debug logs to stdout 
     *      bool error : whether to show error logs to stdout
     *      bool show : whether to show logs to stdout or just save to file;
    */
    Logger(std::string _logFile = "Mylog.log",bool debug = true, bool error = true, bool show = true);

    /**
     * Logger constructor;
    */
    Logger();

    /**
     * Change logfile name;
    */
    void changeLogFilename(std::string _filename);
    /**
        Log level info.
    */
    template <typename... Args>
    void info(Args &&...args);
    /**
     * Logger level : Debug
    */
    template <typename... Args>
    void debug(Args &&...args);

    /**
     * Logger level : error
    */
    template <typename... Args>
    void error(Args &&...args);

    /**
     * Logger level : warning
    */
    template <typename... Args>
    void warning(Args &&...args);

    /**
     * Logger level : fatal
    */
    template <typename... Args>
    void fatal(Args &&...args);

    ~Logger();
};

// Logger::Logger(std::string _logFileName,bool _debug, bool _error, bool _show ):
//                 debug_flag(_debug),error_flag(_error),show_flag(_show)
// {
//     logfile.open(_logFileName.append(".log"));
//     if (!logfile.is_open())
//         std::cerr << "Error in Opening logfile" << std::endl;
    
// }

/**
        Log level info.
*/
template <typename... Args>
void Logger::info(Args &&...args)
{
    // find the current timestamp.    
    time_t now = time(0);
    tm* timeinfo = localtime(&now);
    strftime(timestamp,sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);

    // create log entry.
    logstream <<  "[" << timestamp << "]:" << "[INFO]: " ;
    ((logstream << args << " " ), ...);
    logstream << "\n";
    // write to the log file.
    logfile << logstream.str();
    // change the console text color.
    // std::cout << "\033["<<CYAN<<"m";
    // show output to console output if show output is true.
    if (show_flag)
    {
        std::cout << logstream.str();
    }
    // revert the console text color to white.
    // std::cout << "\033[" << WHITE << "m";

    // reset the stringstream to empty.
    logstream.str("");
}

/**
     * Logger level : debug
*/
template <typename... Args>
void Logger::debug(Args &&...args)
{
    // check for flags.
    if (!debug_flag)
        return;
    // find the current timestamp.    
    time_t now = time(0);
    tm* timeinfo = localtime(&now);
    strftime(timestamp,sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);

    // create log entry.
    logstream <<  "[" << timestamp << "]:" << "[DEBUG]: " ;
    ((logstream << args << " " ), ...);
    logstream << "\n";
    
    // write to the log file.
    logfile << logstream.str();
    // change the console text color.
    std::cout << "\033["<<GREEN<<"m";
    // show output to console output if show output is true.
    if (show_flag)
    {
        std::cout << logstream.str();
    }
    // revert the console text color to white.
    std::cout << "\033[" << WHITE << "m";
    logstream.str("");
}

/**
     * Logger level : error.
     * Show only error messages both in console and log file depending
     * on error_flag value
*/
template <typename... Args>
void Logger::error(Args &&...args)
{
    // check for flags.
    if (!error_flag)
        return;
     // find the current timestamp.    
     time_t now = time(0);
     tm* timeinfo = localtime(&now);
     strftime(timestamp,sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
 
     // create log entry.
     logstream <<  "[" << timestamp << "]:" << "[ERROR]: " ;
     ((logstream << args << " " ), ...);
     logstream << "\n";
     
     // write to the log file.
    logfile << logstream.str();
     // change the console text color.
     std::cout << "\033["<<MAGENTA<<"m";
     // show output to console output if show output is true.
     if (show_flag)
     {
         std::cout << logstream.str();
     }
     // revert the console text color to white.
     std::cout << "\033[" << WHITE << "m";
     logstream.str("");
    
}

/**
     * Logger level : warning.
       Show warning messages both in console and log file
*/
template <typename... Args>
void Logger::warning(Args &&...args)
{
    // find the current timestamp.    
    time_t now = time(0);
    tm* timeinfo = localtime(&now);
    strftime(timestamp,sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);

    // create log entry.
    logstream <<  "[" << timestamp << "]:" << "[WARNING]: " ;
    ((logstream << args << " " ), ...);
    logstream << "\n";
    
    // write to the log file.
    logfile << logstream.str();
    // change the console text color.
    std::cout << "\033["<<YELLOW<<"m";
    // show output to console output if show output is true.
    if (show_flag)
    {
        std::cout << logstream.str();
    }
    // revert the console text color to white.
    std::cout << "\033[" << WHITE << "m";
    logstream.str("");
}

/**
     * Logger level : fatal.
       Show fatal messages both in console and log file
*/
template <typename... Args>
void Logger::fatal(Args &&...args)
{
     // find the current timestamp.    
     time_t now = time(0);
     tm* timeinfo = localtime(&now);
     strftime(timestamp,sizeof(timestamp), "%Y-%m-%d %H:%M:%S", timeinfo);
 
     // create log entry.
     logstream <<  "[" << timestamp << "]:" << "[FATAL]: " ;
     ((logstream << args << " " ), ...);
     logstream << "\n";
     
     // write to the log file.
     logfile << logstream.str();
     // change the console text color.
    //  std::cout << "\033["<<RED<<"m";

    // if bold red color needed.
     std::cout << BOLDRED;
     // show output to console output if show output is true.
     if (show_flag)
     {
         std::cout << logstream.str();
     }
     // revert the console text color to white.
     std::cout << RESET;
    //  std::cout << "\033[" << WHITE << "m";
     logstream.str("");
}

// Logger::~Logger()
// {
//     if (logfile.is_open())
//         logfile.close();
//     logfile.flush();
// }

#endif /* _FOO_H */