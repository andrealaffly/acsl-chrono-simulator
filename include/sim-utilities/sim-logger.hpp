/***********************************************************************************************************************
 * Copyright (c) 2025 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

 /***********************************************************************************************************************
 * File:        sim-logger.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 26, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the common logger class that is then used to instantiate the logging
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_LOGGER_HPP_
#define SIM_LOGGER_HPP_

#include <atomic>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <ctime>

/// ------------- Boost Includes ------------- /// 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/basic_logger.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/value_ref.hpp>
#include <boost/phoenix/bind.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include "sim-platforms.hpp"    // Import the header file with all the platforms
#include "fkYAML/node.hpp"      // Import the header file for the yaml functionality

namespace logging = boost::log;
namespace sinks = boost::log::sinks;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace attrs = boost::log::attributes;
namespace keywords = boost::log::keywords;

namespace _acsl_
{

namespace _logger_
{

// -----------------------------------------------------------------------------------------------------
// Loggers structure
//
// Groups Boost.Log logger objects for physics and control logging channels.
// Can be used to filter, route, or format output for each simulation subsystem.
// -----------------------------------------------------------------------------------------------------
struct Loggers
{
    src::logger physics;
    src::logger control;
};

// =====================================================================================================================
// simlog
//
// Purpose:
//   Manages the logging infrastructure for the simulator, providing setup, configuration,
//   and logging of runtime data using Boost.Log. Handles both physics and control logs,
//   and exposes interfaces for header configuration and log initialization.
//
// Overview:
//   - The class provides functions to configure log file headers, initialize logging, and record data.
//   - Maintains two distinct loggers: one for physics and one for control, grouped in the Loggers struct.
//   - Uses Boost.Log for flexible, multi-level logging and formatting.
//   - Encapsulates logger management to provide a single interface to the simulation code.
//
// Public Interface:
//   - simlog(): Constructor for logger setup routine.
//   - ~simlog(): Destructor to handle resource cleanup if needed.
//   - void ConfigureLogDirectory(): Creates and configures log directory structure.
//   - src::logger GetPhysicsLogger(): Returns the logger for physics channel.
//   - src::logger GetControlLogger(): Returns the logger for control channel.
//   - void SetLogDirectory(const std::filesystem::path&): Sets log directory for simulation instance.
//   - std::filesystem::path GetLogDirectory() const: Gets current log directory path.
//
// Notes:
//   - Logging is performed in a thread-safe manner using synchronous Boost.Log backends.
//   - All log data can be formatted, filtered, and routed according to the logger's configuration.
//   - Intended to be used as a central logging facility for different simulator subsystems.
// =====================================================================================================================
class simlog
{
public:
    // ------------------------------------------------------------------------
    // Constructor:
    //   - Initializes the logging directory configuration.
    // ------------------------------------------------------------------------
    simlog() 
    {
        // Setup the log directory
        ConfigureLogDirectory();
    }

    // Destructor
    ~simlog() {}

    // ------------------------------------------------------------------------
    // Creates a directory structure for simulator log files:
    //   - Base directory is relative: "../sim-log/"
    //   - Appends platform folder (e.g., UAV type)
    //   - Adds date in "YYYY_MM_DD" format
    //   - Adds run folder "flight_run_HH_MM_SS" from current local time
    //
    // Implementation: sim-bridge.cpp
    // ------------------------------------------------------------------------
    void ConfigureLogDirectory();

    // ------------------------------------------------------------------------
    // Accessor: Returns references to physics and control loggers.
    // ------------------------------------------------------------------------
    src::logger& GetPhysicsLogger() { return this->loggers.physics; }
    src::logger& GetControlLogger() { return this->loggers.control; }

    // ------------------------------------------------------------------------
    // Setter: Assigns the log directory for this simulation instance.
    // ------------------------------------------------------------------------
    void SetLogDirectory(const std::filesystem::path& dir) { this->log_dir = dir; }

    // ------------------------------------------------------------------------
    // Accessor: Retrieves the current log directory for this simulation instance.
    // ------------------------------------------------------------------------
    std::filesystem::path GetLogDirectory() const { return this->log_dir; }

private:
    // ------------------------------------------------------------------------
    // Hardcoded path to the simulation configuration YAML.
    // Stores settings for:
    //   - Active simulation mode (model/software/hardware-in-loop)
    //   - Active UAV platform
    // ------------------------------------------------------------------------
    const std::string sim_config_filepath = "../config/sim-config.yaml";

    // ------------------------------------------------------------------------
    // Group of Boost.Log logger objects for physics and control logging.
    // ------------------------------------------------------------------------
    Loggers loggers;

    // ------------------------------------------------------------------------
    // Filesystem path storing the base log directory for this simulation run.
    // ------------------------------------------------------------------------
    std::filesystem::path log_dir;

    // ------------------------------------------------------------------------
    // Structure from sim-platforms that holds all the platforms that are 
    // available. - Borrowed from the bridge class and will be used to just
    // figure out which platform is currently active. No other functionality
    // of the platforms structure is used.
    // ------------------------------------------------------------------------
    ::_acsl_::_bridge_::platforms available_uavs;
    std::string active_platform;
};


// =====================================================================================================================
// blackbox (Abstract Base Class)
//
// Purpose:
//   Defines a generic, inheritable interface for simulator logging systems with
//   three fundamental operations: configuring headers, initializing logging,
//   and recording data. Classes inheriting from blackbox must implement each method.
//
// Usage:
//   - Inherit from this base class to implement custom logging strategies or backends.
//   - Recommended to use with simulator loggers where a common interface is needed.
// =====================================================================================================================
class blackbox
{
public:
    // ------------------------------------------------------------------------
    // Virtual destructor for proper cleanup in derived classes.
    // ------------------------------------------------------------------------
    virtual ~blackbox() {}

    // ------------------------------------------------------------------------
    // Configure log file headers for new simulation runs.
    // Must be overridden by derived logger implementations.
    // ------------------------------------------------------------------------
    virtual void ConfigureHeaders() = 0;

    // ------------------------------------------------------------------------
    // Initialize logging systems and open log outputs.
    // Must be overridden by derived logger implementations.
    // Returns true on success.
    // ------------------------------------------------------------------------
    virtual bool InitiateLogging() = 0;

    // ------------------------------------------------------------------------
    // Record simulation or control data to log channels.
    // Must be overridden by derived logger implementations.
    // ------------------------------------------------------------------------
    virtual void LogData() = 0;
};


// ============================================================================================================
// Namespace: _filesystem_
// Description: Helper functions for filesystem operations
// ============================================================================================================
namespace _filesystem_
{
    
/**
 * @brief Sets up the Boost.Log controller logging infrastructure for a specific controller.
 *
 * This function prepares the logging environment for a controller by creating its log directory,
 * initializing a log file, and configuring Boost.Log sinks and formatters. It should be called
 * during controller initialization to ensure diagnostic logging support.
 *
 * @param m_logger Reference to the simulator logging instance used for log directory access and attribute management.
 * @param platform_name   Name of the platform; becomes the name of the subdirctory to search for the gains.
 * @param controller_name The name of the controller (used as subfolder under the log directory, e.g., "PID_OMEGA").
 *
 * @return true if log directory and log file creation, stream initialization, and Boost.Log sink setup succeed; false otherwise.
 *
 * @details
 * This function creates a dedicated directory for the given controller, 
 * sets up a log file, and configures a synchronous Boost.Log sink to 
 * stream log messages to the designated file. This function also copies
 * over the gains for the controller to the log structure.
 * Proper error handling ensures robust operation and reliable diagnostics.
 */
bool setupControllerLogging(_acsl_::_logger_::simlog& m_logger, const std::string& platform_name, const std::string& controller_name) ;

} // namespace _filesystem_

}   // namespace _logger_

}   // namespace _acsl_


#endif  // SIM_LOGGER_HPP_