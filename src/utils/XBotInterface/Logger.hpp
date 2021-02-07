/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef __XBOT_LOGGER__
#define __XBOT_LOGGER__

#include <ostream>
#include <iostream>
#include <sstream>

#include <unordered_map>

#include <eigen3/Eigen/Dense>

#include <spdlog/spdlog.h>

#include <signal.h>

#include <time.h>

#include <matio.h>

#define ASYNC_QUEUE_SIZE_BIT 65536

#define M_PREALLOCATION_SIZE 8192






namespace XBot {

class SSLogger;

/**
 * @brief Logger severity level
 *
 */
enum class LoggerLevel {
    INFO,
    WARNING,
    ERROR
};

/**
 * @brief Endl abstraction to control the print of the SSLogger
 *
 */
class LoggerEndl {

public:

    LoggerEndl ( SSLogger * sslogger ) : _ss_logger ( sslogger ) {}
    friend void operator<< ( std::ostream& os, LoggerEndl& log );

private:

    void print ( std::ostream& os );
    SSLogger * _ss_logger;

};



/**
 * @brief String Stream Logger
 *
 */
class SSLogger  {

    // LoggerEndl can access SSLogger private print
    friend LoggerEndl;

public:

    /**
     * @brief SSLogger constructor
     *
     *
     */
    SSLogger ( const std::string& logger_name,
               const std::string& log_filename,
               int log_file_size,
               int log_max_files )
        : _logger_name(logger_name),
          _log_filename(log_filename),
          _log_file_size(log_file_size),
          _log_max_files(log_max_files),
          _endl ( this )
    {
    }

    void setSeverityLevel ( XBot::LoggerLevel severity ) {
        _severity = severity;
    }

    void setHeader ( const std::string& header ) {
        _header = header;
    }

    std::stringstream& log() {
        // put the header in the stringstream
        _ss << _header;
        // give it back to <<
        return _ss;
    }

    LoggerEndl& endl() {
        return _endl;
    }

    friend void operator<< ( std::ostream& ss, LoggerEndl& log );

protected:

    virtual void info_internal( const std::string& s) = 0;
    virtual void warning_internal( const std::string& s) = 0;
    virtual void error_internal( const std::string& s) = 0;

    std::string _logger_name;
    std::string _log_filename;
    int _log_file_size;
    int _log_max_files;

private:

    void print ( std::ostream& os ) {

        // from ostream to stringstream
        std::stringstream ss;
        ss << os.rdbuf();



        // chose the right severity level
        switch ( _severity ) {
        case LoggerLevel::INFO : {
            info_internal( ss.str() );
            break;
        }
        case LoggerLevel::WARNING : {
            warning_internal( ss.str() );
            break;
        }
        case LoggerLevel::ERROR : {
            error_internal( ss.str() );
            break;
        }
        default : {
            break;
        }
        }

        // empty the stringstream
        _ss.str(std::string());
    }

    std::stringstream _ss;
    std::string _header;

    LoggerEndl _endl;
    LoggerLevel _severity;

};

class SSFileLogger : public SSLogger {

public:
    /**
    * @brief SSLogger constructor
    *
    *
    */
    SSFileLogger ( const std::string& logger_name,
                   const std::string& log_filename,
                   int log_file_size,
                   int log_max_files ) :
        SSLogger(logger_name,
                 log_filename,
                 log_file_size,
                 log_max_files)
    {

    }


};

class SPDFileLogger : public SSFileLogger {

public:
    /**
    * @brief SSLogger constructor
    *
    *
    */
    SPDFileLogger ( const std::string& logger_name,
                    const std::string& log_filename,
                    int log_file_size,
                    int log_max_files ) :
        SSFileLogger( logger_name,
                      log_filename,
                      log_file_size,
                      log_max_files)
    {

        // by default use the async mode to be RT safe
        size_t q_size = ASYNC_QUEUE_SIZE_BIT;
        spdlog::set_async_mode ( q_size );

        // rotating file logger
        _rotating_logger = spdlog::rotating_logger_mt ( logger_name, log_filename, log_file_size, log_max_files );
    }

protected:

    virtual void info_internal( const std::string& s) {
        _rotating_logger->info ( s );
    }

    virtual void warning_internal( const std::string& s) {
        _rotating_logger->warn ( s );
    }

    virtual void error_internal( const std::string& s) {
        _rotating_logger->error ( s );
    }


private:

    std::shared_ptr<spdlog::logger> _rotating_logger;

};

class SSConsoleLogger : public SSLogger {

public:
    /**
    * @brief SSLogger constructor
    *
    *
    */
    SSConsoleLogger ( const std::string& logger_name ) :
        SSLogger(logger_name,
                 "/tmp/console_log",
                 1048576 * 5,
                 5)
    {

    }

};

class SPDConsoleLogger : public SSConsoleLogger {

public:
    /**
    * @brief SSLogger constructor
    *
    *
    */
    SPDConsoleLogger ( const std::string& logger_name) :
        SSConsoleLogger( logger_name)
    {
        // console logger
        _console = spdlog::stdout_logger_mt ( "console" );
        // rotating file logger
        _rotating_logger = spdlog::rotating_logger_mt ( logger_name, _log_filename, _log_file_size, _log_max_files );
    }

protected:

    virtual void info_internal( const std::string& s) {
        _console->info ( s );
        _rotating_logger->info ( s );
    }

    virtual void warning_internal( const std::string& s) {
        _console->warn ( s );
        _rotating_logger->warn ( s );
    }

    virtual void error_internal( const std::string& s) {
        _console->error ( s );
        _rotating_logger->error ( s );
    }


private:

    std::shared_ptr<spdlog::logger> _console;
    std::shared_ptr<spdlog::logger> _rotating_logger;

};


class Logger {

public:


    /**
    * @brief Info log
    *
    * @return XBot::SSLogger&
    */
    virtual std::stringstream& info() {

        _ss_logger->setSeverityLevel ( XBot::LoggerLevel::INFO );
        return _ss_logger->log();
    }

    /**
     * @brief Warning log
     *
     * @return XBot::SSLogger&
     */
    virtual std::stringstream& warning() {
        _ss_logger->setSeverityLevel ( XBot::LoggerLevel::WARNING );
        return _ss_logger->log();
    }

    /**
     * @brief Error log
     *
     * @return XBot::SSLogger&
     */
    virtual std::stringstream& error() {
        _ss_logger->setSeverityLevel ( XBot::LoggerLevel::ERROR );
        return _ss_logger->log();
    }

    /**
     * @brief Smart endl for XBot Logger
     *
     * @return XBot::LoggerEndl&
     */
    LoggerEndl& endl() {
        return _ss_logger->endl();
    }

protected:

    std::shared_ptr<SSLogger> _ss_logger;

};

class FileLogger : public Logger {

public:

    typedef std::shared_ptr<FileLogger> Ptr;

    static Ptr getLogger( const std::string& logger_name,
                          const std::string& log_filename,
                          int log_file_size = 1048576 * 5,
                          int log_max_files = 5 ) {

        if (_instance_map.count(logger_name)) return _instance_map.at(logger_name);
        else {
            Ptr ptr(new FileLogger(logger_name, log_filename, log_file_size, log_max_files));
            _instance_map[logger_name] = ptr;
            return ptr;
        }

    }



private:

    FileLogger ( const std::string& logger_name,
                 const std::string& log_filename,
                 int log_file_size = 1048576 * 5,
                 int log_max_files = 5 )
    {
        _ss_logger = std::make_shared<SPDFileLogger>(logger_name, log_filename, log_file_size, log_max_files);
    }

    static std::map<std::string, Ptr> _instance_map;

};


class ConsoleLogger : public Logger {

public:

    typedef std::shared_ptr<ConsoleLogger> Ptr;

    static Ptr getLogger(const std::string& logger_name = "console_logger") {
        if (_instance) return _instance;
        else {
            _instance = Ptr( new ConsoleLogger(logger_name) );
            return _instance;
        }
    }

private:

    /**
     * @brief XBot Logger constructor: you can specify the logger name, the path for the log,
     * the file_size and the maximum number of file to use for the log.
     *
     * By default it creates the log using file /tmp/XBotLog with 5Mb and a maximum of 5 files.
     *
     */

    ConsoleLogger ( const std::string& logger_name)
    {
        _ss_logger = std::make_shared<SPDConsoleLogger>(logger_name);
    }

    static Ptr _instance;


};


#define DEFAULT_BUFFER_SIZE 13421772 // 12.8 MB

/**
 * @brief The MatLogger class provides functionality to log numerical
 * data to binary .mat files which can be easily imported in MATLAB/
 * OCTAVE/scipy/...
 *
 * Usage:
 *  - obtain a pointer to the MatLogger by calling the factory method
 * MatLogger::getLogger(filename_without_extension)
 *  - during the initialization phase, preallocate memory for the
 * variables to be logged (not mandatory, but makes the logger RT
 * safe) with methods createScalarVariable(), createVectorVariable(),
 * createMatrixVariable()
 *  - inside the loop, log data with the method add()
 *  - you can actually dump data to the mat file manually by calling flush(),
 *    otherwise the dumping will be done inside the destructor
 *
 */
class MatLogger {



protected: enum class VariableType { Scalar, Vector, Matrix };

protected: struct VariableInfo {

        std::string name;
        int interleave = 1;
        int count = 0;
        VariableType type;
        Eigen::MatrixXd data;
        int rows, cols;
        int buffer_capacity;
        int head = 0, tail = 0;
        bool empty = true;
        bool IsCircShift = false;

        void rearrange()
        {

            if ( empty ) {
                data.resize(0, 0);
                return;
            }

            if ( tail > head ) {
                data.conservativeResize(data.rows(), cols * (tail - head));
                return;
            }

            auto log = XBot::ConsoleLogger::getLogger();
            log->info() << "Data of " << name << " are circ-shifted, take care while reading it..." << log->endl();

//         Eigen::MatrixXd tmp(data.rows(), data.cols());
//         tmp.leftCols(cols*(buffer_capacity-head)) = data.rightCols(cols*(buffer_capacity-head));
//         tmp.rightCols(cols*head) = data.leftCols(cols*head);
//         data = tmp;

//         for( int i = 0; i < (buffer_capacity-tail)*cols; i++ ){
//             auto log = XBot::ConsoleLogger::getLogger();
//             log->info() << "Data of " << name << " are circ-shifted, take care!from circular array...be patient! [TBD improve performance]" << log->endl();
//             circshift();
//         }
        }

    private:

        void circshift() {
            for ( int i = 1; i < data.cols(); i++) {
                data.col(i).swap(data.col(0));
            }
        }

    };

public:

    typedef std::shared_ptr<MatLogger> Ptr;

    /**
     * @brief Factory method which returns a matlogger which
     * saves on the mat file provided as an argument.
     *
     * @return A shared pointer to the requested MatLogger
     */
    static Ptr getLogger(std::string filename)
    {
        if ( _instances.count(filename) ) {
            return _instances.at(filename);
        }
        else {
            _instances[filename] = Ptr(new MatLogger(filename));
            return _instances.at(filename);
        }
    }


    /**
     * @brief Logs a single variable to the mat file. Each call will overwrite
     * the variable (each time data dimensions must remain the same)
     *
     * @param name The name of the variable
     * @return True if variable with provided name either does not exist yet, or
     * has matching dimensions.
     */
    template <typename Derived>
    bool log(const std::string& name, const Eigen::MatrixBase<Derived>& data) {

        if ( _var_idx_map.count(name) ) {
            return false;
        }

        auto it = _single_var_map.find(name);

        if ( it != _single_var_map.end() ) {
            if ( it->second.rows() == data.rows() && it->second.cols() == data.cols() ) {
                it->second = data;
                return true;
            }
            else {
                return false;
            }
        }
        else {
            _single_var_map[name] = data;
            return true;
        }

    }

    bool log(const std::string& name, const std::vector<double>& data) {
        Eigen::Map<Eigen::MatrixXd> map((double *)data.data(), (int)data.size(), 1);
        return add(name, map);
    }

    /**
     * @brief Allocate memory for logging a scalar variable.
     *
     * @param name The name of the variable to be logged.
     * @param interleave The variable will be actually logged every interleave calls to the method add() (default is 1)
     * @param buffer_size Max number of samples that will be logged before overwriting the oldest ones (default one million)
     * @return True if the requested name is available.
     */
    bool createScalarVariable(std::string name, int interleave = 1, int buffer_size = -1)
    {
        if ( buffer_size < 0 ) {
            buffer_size = 1024 * 1024;
        }

        if (_var_idx_map.count(name) || _single_var_map.count(name)) {
            return false;
        }

        _var_idx_map[name] = VariableInfo();

        VariableInfo& varinfo = _var_idx_map.at(name);

        varinfo.name = name;
        varinfo.interleave = interleave;
        varinfo.count = -1;
        varinfo.type = VariableType::Scalar;
        varinfo.data = Eigen::RowVectorXd::Zero(buffer_size);
        varinfo.rows = 1;
        varinfo.cols = 1;
        varinfo.buffer_capacity = buffer_size;


        return true;

    }

    /**
     * @brief Allocate memory for logging a scalar variable.
     *
     * @param name The name of the variable to be logged.
     * @param size The size of the vector to be logged (i.e. its number of elements)
     * @param interleave The variable will be actually logged every interleave calls to the method add() (default is 1)
     * @param buffer_size Max number of samples that will be logged before overwriting the oldest ones (by default 12.8 MB of memory are allocated)
     * @return True if the requested name is available.
     */
    bool createVectorVariable(std::string name, int size, int interleave = 1, int buffer_size = -1)
    {
        if ( size <= 0 ) {
            return false;
        }

        if ( buffer_size < 0 ) {
            buffer_size = DEFAULT_BUFFER_SIZE / (size * 8);
        }

        if (_var_idx_map.count(name)) {
            return false;
        }

        _var_idx_map[name] = VariableInfo();

        VariableInfo& varinfo = _var_idx_map.at(name);

        varinfo.name = name;
        varinfo.interleave = interleave;
        varinfo.count = 0;
        varinfo.type = VariableType::Vector;
        varinfo.data = Eigen::MatrixXd::Zero(size, buffer_size);
        varinfo.rows = size;
        varinfo.cols = 1;
        varinfo.buffer_capacity = buffer_size;


        return true;
    }

    /**
     * @brief Allocate memory for logging a scalar variable.
     *
     * @param name The name of the variable to be logged.
     * @param rows The number of rows of the vector to be logged
     * @param cols The number of columns of the vector to be logged
     * @param interleave The variable will be actually logged every interleave calls to the method add() (default is 1)
     * @param buffer_size Max number of samples that will be logged before overwriting the oldest ones (by default 12.8 MB of memory are allocated)
     * @return True if the requested name is available.
     */
    bool createMatrixVariable(std::string name, int rows, int cols, int interleave = 1, int buffer_size = -1)
    {

        if ( rows <= 0 || cols <= 0 ) {
            return false;
        }

        if ( buffer_size < 0 ) {
            buffer_size = DEFAULT_BUFFER_SIZE / (rows * cols * 8);
        }

        if (_var_idx_map.count(name)) {
            return false;
        }


        _var_idx_map[name] = VariableInfo();

        VariableInfo& varinfo = _var_idx_map.at(name);

        varinfo.name = name;
        varinfo.interleave = interleave;
        varinfo.count = -1;
        varinfo.type = VariableType::Matrix;
        varinfo.data = Eigen::MatrixXd::Zero(rows, cols * buffer_size);
        varinfo.rows = rows;
        varinfo.cols = cols;
        varinfo.buffer_capacity = buffer_size;


        return true;
    }


    /**
     * @brief Logs the provided data to the MAT variable with the provided name.
     * If such a variable was not defined by calling a createVariable() method,
     * the call to add() causes memory allocation and is not RT safe.
     *
     * @param name MAT variable name.
     * @param data The Eigen varible to be logged.
     * @return True if
     */
    template <typename Derived>
    bool add(const std::string& name, const Eigen::MatrixBase<Derived>& data, int interleave = 1, int buffer_capacity = -1)
    {
        auto it = _var_idx_map.find(name);

        if ( it == _var_idx_map.end() ) {

            std::cout << " in " << __func__ << "! Variable with name " << name << " has NOT been created yet! This will cause memory allocation!" << std::endl;

            if ( data.cols() == 1 ) {
                if (createVectorVariable(name, data.size(), interleave, buffer_capacity)) {
                    return add(name, data);
                }
                else return false;
            }
            else {
                if (createMatrixVariable(name, data.rows(), data.cols(), interleave, buffer_capacity)) {
                    return add(name, data);
                }
                else return false;
            }

        }

        VariableInfo& varinfo = it->second;

        if ( data.rows() != varinfo.rows || data.cols() != varinfo.cols ) {
            std::cout << " in " << __func__ << "! Provided data has unmatching dimensions!" << std::endl;
            return false;
        }


        varinfo.count = (varinfo.count + 1) % varinfo.interleave;

        if ( varinfo.count != 0 ) {
            return true;
        }

        varinfo.tail = varinfo.tail % varinfo.buffer_capacity;

        // if buffer is not empty and head = tail, increment head since we are going to overwrite an element
        if ( !varinfo.empty && varinfo.head == varinfo.tail ) {
            varinfo.head = (varinfo.head + 1) % varinfo.buffer_capacity;
        }

        // write to tail position
        varinfo.data.block(0, varinfo.tail * varinfo.cols, varinfo.rows, varinfo.cols) = data.template cast<double>();
        varinfo.empty = false;

        // increment tail position
        varinfo.tail = (varinfo.tail + 1) ;

        if (!varinfo.IsCircShift && varinfo.tail == varinfo.buffer_capacity) {
            varinfo.IsCircShift = true;
        }
    }

    bool add(const std::string& name, double data)
    {
        Eigen::Matrix<double, 1, 1> eigen_data;
        eigen_data(0) = data;
        return add(name, eigen_data);
    }

    bool add(const std::string& name, const std::vector<double>& data) {
        Eigen::Map<Eigen::MatrixXd> map((double *)data.data(), (int)data.size(), 1);
        return add(name, map);
    }

    bool add(const std::string& name, const std::vector<int>& data) {
        Eigen::Map< Eigen::MatrixXi> map((int *)data.data(), (int)data.size(), 1);
        return add(name, map);
    }

    template <size_t Size>
    bool add(const std::string& name, const std::array<double, Size>& data) {
        Eigen::Map<Eigen::MatrixXd> map((double *)data.data(), (int)data.size(), 1);
        return add(name, map);
    }

    template <size_t Size>
    bool add(const std::string& name, const std::array<int, Size>& data) {
        Eigen::Map< Eigen::MatrixXi> map((int *)data.data(), (int)data.size(), 1);
        return add(name, map);
    }

    template <typename EigenVectorType>
    bool add(const std::string& name, const std::vector<EigenVectorType>& data) {

        if ( data.size() == 0 ) return false;

        for ( const auto& vec : data ) {
            if (vec.cols() != 1 && vec.size() != data[0].size()) {
                std::cout << "in " << __PRETTY_FUNCTION__ << "! All elements of the vector to be logged must be column vectors of the same size!" << std::endl;
                return false;
            }
        }

        Eigen::MatrixXd tmp(data[0].size(), data.size());

        int i = 0;
        for ( const auto& vec : data ) {
            tmp.col(i++) = vec;
        }

        add(name, tmp);

    }


    template <typename EigenVectorType, size_t Size>
    bool add(const std::string& name, const std::array<EigenVectorType, Size>& data) {

        if ( data.size() == 0 ) return false;

        for ( const auto& vec : data ) {
            if (vec.cols() != 1 && vec.size() != data[0].size()) {
                std::cout << "in " << __PRETTY_FUNCTION__ << "! All elements of the vector to be logged must be column vectors of the same size!" << std::endl;
                return false;
            }
        }

        Eigen::MatrixXd tmp(data[0].size(), data.size());

        int i = 0;
        for ( const auto& vec : data ) {
            tmp.col(i++) = vec;
        }

        add(name, tmp);

    }

    /**
     * @brief Does the actual work of saving data to disk. Since
     * this is a time-consuming operation, should be done outside of
     * any high performance loop. If not explicitly called in the code,
     * flush() is anyway performed in the class destructor.
     */
    void flush() {

        if (_flushed) return;

        _flushed = true;

        std::cout << "Dumping data to mat file " << _file_name << std::endl;

        mat_t * mat_file = Mat_CreateVer(_file_name.c_str(), nullptr, MAT_FT_MAT5);

        if (!mat_file) {
            std::cout << "ERROR creating MAT file!" << std::endl;
        }

        for ( auto& pair : _single_var_map ) {

            std::cout << "Writing variable " << pair.first << " to mat file..." << std::endl;

            int n_dims = 2;
            std::size_t dims[2];
            dims[0] = pair.second.rows();
            dims[1] = pair.second.cols();


            matvar_t * mat_var = Mat_VarCreate(pair.first.c_str(),
                                               MAT_C_DOUBLE,
                                               MAT_T_DOUBLE,
                                               n_dims,
                                               dims,
                                               (void *)pair.second.data(),
                                               0 );

            Mat_VarWrite(mat_file, mat_var, MAT_COMPRESSION_NONE);
            Mat_VarFree(mat_var);

        }

        for ( auto& pair : _var_idx_map ) {

            std::cout << "Writing variable " << pair.first << " to mat file..." << std::endl;

            VariableInfo& varinfo = pair.second;

            // varinfo.rearrange();

            int n_dims = 2;
            std::size_t dims[3];

            if ( varinfo.type == VariableType::Matrix ) {
                n_dims = 3;
                dims[0] = varinfo.rows;
                dims[1] = varinfo.cols;
                if(varinfo.IsCircShift) {
                    dims[2] = varinfo.data.cols()/varinfo.cols;
                }
                else {
                    dims[2] = varinfo.tail;
                }
            }
            else {
                dims[0] = varinfo.data.rows();
                if(varinfo.IsCircShift) {
                    dims[1] = varinfo.data.cols();
                }
                else {
                    dims[1] = varinfo.tail;
                }
            }

            matvar_t * mat_var = Mat_VarCreate(pair.first.c_str(),
                                               MAT_C_DOUBLE,
                                               MAT_T_DOUBLE,
                                               n_dims,
                                               dims,
                                               (void *)varinfo.data.data(),
                                               0 );

            Mat_VarWrite(mat_file, mat_var, MAT_COMPRESSION_ZLIB);
            Mat_VarFree(mat_var);

        }

        std::cout << "Flushing to " << _file_name << " complete!" << std::endl;



    }


    ~MatLogger() {
        flush();
    }

protected:

    MatLogger(std::string file_name):
//         _clog(ConsoleLogger::getLogger()),
        _flushed(false)
    {
        // retrieve time
        time_t rawtime;
        struct tm * timeinfo;
        char buffer [80];

        std::time (&rawtime);
        timeinfo = localtime (&rawtime);

        strftime(buffer, 80, "__%Y_%m_%d__%H_%M_%S.mat", timeinfo);
        puts (buffer);

        // rotating file logger
        std::string file_name_extended;
        file_name_extended = file_name + std::string(buffer);

        _file_name = file_name_extended;
    }

    std::unordered_map<std::string, VariableInfo> _var_idx_map;
    std::unordered_map<std::string, Eigen::MatrixXd> _single_var_map;
    std::string _file_name;

private:

    static std::unordered_map<std::string, Ptr> _instances;
//     ConsoleLogger::Ptr _clog;
    bool _flushed;

};



}

#endif //__XBOT_LOGGER__
