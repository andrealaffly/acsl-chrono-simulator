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
 * File:        sim-logger.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 26, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation file for the common logger class that is then used to instantiate the logging
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-logger.hpp"

namespace _acsl_
{

namespace _logger_
{

// =====================================================================================================================
// ConfigureLogDirectory()
//
// Purpose:
//   Creates a directory structure for flight simulation logs, organizing output files by platform and run time.
//   Copies main config files needed for reproducibility into the log directory.
//
// Workflow:
//   1. Get current local system time to uniquely timestamp the log directory for archival.
//   2. Format date ("YYYY_MM_DD") and time ("HH_MM_SS") strings for use in folder names.
//   3. Read simulator config YAML, extract platform selections, and validate exclusive platform choice.
//   4. Assemble the full log directory: base path, platform folder, date folder, and flight run folder.
//   5. Create the directory tree recursively using std::filesystem and handle errors.
//   6. Print diagnostic info for success or errors during directory creation.
//   7. Ensure a "config" subdirectory exists under the new log directory for storing reproducibility-critical files.
//   8. Copy required config files from "../config" to the log's config subdirectory; log outcome of each copy.
//
// Notes:
//   - Only "phy-config.yaml", "sim-config.yaml", and "vis-config.yaml" are copied per run.
//   - Any file or directory creation issues are logged as errors and do not halt subsequent file copies.
//   - Layout enables full reproducibility and traceability for each simulation run.
//   - Uses std::filesystem for portability and robust error handling.
// =====================================================================================================================
void simlog::ConfigureLogDirectory()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Get current local system time for this simulator run
    // ------------------------------------------------------------------------
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_result;
    localtime_r(&t, &tm_result);

    // ------------------------------------------------------------------------
    // STEP 2 – Format date and time strings for unique directory naming
    // ------------------------------------------------------------------------
    std::ostringstream dateStream, timeStream;
    dateStream << std::put_time(&tm_result, "%Y_%m_%d");
    timeStream << std::put_time(&tm_result, "%H_%M_%S");
    std::string dateStr = dateStream.str();
    std::string timeStr = timeStream.str();

    // ------------------------------------------------------------------------
    // STEP 3 – Read sim config file and extract platform selection from YAML
    //   - Open sim_config_filepath for reading
    //   - Populate available_uavs from YAML dynamically without hardcoding
    //   - Ensure only one platform is selected
    // ------------------------------------------------------------------------
    this->log_dir = "../sim-log";  // base directory
    std::ifstream ifs(this->sim_config_filepath);
    if (!ifs) {
        _message_::SIMULATOR_ERROR(
            "[SIMLOG]: COULD NOT OPEN SIMULATOR CONFIG FILE: " + this->sim_config_filepath
        );
        return;
    }
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);
    for (auto& [name, ref] : available_uavs.asVectorRef()) {
        ref = config_file["platform"][name].as_bool();
    }
    this->active_platform = available_uavs.validateExclusiveSelection();
    this->log_dir /= this->active_platform;
    this->log_dir /= dateStr;
    this->log_dir /= "flight_run_" + timeStr;

    // ------------------------------------------------------------------------
    // STEP 4 – Create required log directory tree recursively; handle errors
    // ------------------------------------------------------------------------
    std::error_code ec;
    std::filesystem::create_directories(this->log_dir, ec);

    // ------------------------------------------------------------------------
    // STEP 5 – Report outcome of directory creation
    // ------------------------------------------------------------------------
    if (ec) {
        _message_::SIMULATOR_ERROR("[SIMLOG]: ERROR CREATING DIRECTORIES: ", ec.message());
        return;
    } else {
        _message_::SIMULATOR_INFO("[SIMLOG]: CREATED LOG DIRECTORY: " + this->log_dir.string());
    }

    // ------------------------------------------------------------------------
    // STEP 6 – Ensure log/config subdirectory exists for config archival
    // ------------------------------------------------------------------------
    std::filesystem::path dst_config = this->log_dir / "config";
    std::error_code ec_config;
    std::filesystem::create_directories(dst_config, ec_config);

    // ------------------------------------------------------------------------
    // STEP 7 – Copy config files ("phy-config.yaml", "sim-config.yaml", "vis-config.yaml") to log/config subdir
    //   - Each file is copied with overwrite enabled
    //   - Every outcome is logged, errors do not halt processing
    // ------------------------------------------------------------------------
    const char* files_to_copy[] = {"phy-config.yaml", "sim-config.yaml", "vis-config.yaml"};
    std::filesystem::path src_config_dir = "../config";
    for (const auto& fname : files_to_copy) {
        std::filesystem::path src = src_config_dir / fname;
        std::filesystem::path dst = dst_config / fname;
        std::error_code copy_ec;
        std::filesystem::copy_file(
            src, dst,
            std::filesystem::copy_options::overwrite_existing,
            copy_ec
        );
        if (copy_ec) {
            _message_::SIMULATOR_ERROR(
                "[SIMLOG]: ERROR COPYING CONFIG FILE: " + src.string() + " -> " + dst.string(),
                copy_ec.message()
            );
        } else {
            _message_::SIMULATOR_INFO("[SIMLOG]: COPIED: " + src.string() + " to " + dst.string());
        }
    }
}


// ============================================================================================================
// _filesystem_ : Helper functions for filesystem operations
// ============================================================================================================
namespace _filesystem_
{
/**
 * @brief Configures the logging system for a controller using Boost.Log.
 * 
 * @param m_logger        Reference to the logger instance managing application-wide logging.
 * @param platform_name   Name of the platform; becomes the name of the subdirctory to search for the gains.
 * @param controller_name Name of the controller; becomes the name of the subdirectory for log storage.
 * 
 * @return true on successful setup and file opening, false if any step fails.
 * 
 * @details
 * This function creates a dedicated directory for the given controller, 
 * sets up a log file, and configures a synchronous Boost.Log sink to 
 * stream log messages to the designated file. This function also copies
 * over the gains for the controller to the log structure.
 * Proper error handling ensures robust operation and reliable diagnostics.
 */
bool setupControllerLogging(_acsl_::_logger_::simlog& m_logger, const std::string& platform_name, const std::string& controller_name) 
{
    try {
        // Get the root logging directory from the logger instance.
        std::filesystem::path directory = m_logger.GetLogDirectory();
        // Create the controller-specific log directory under the root directory.
        std::filesystem::path control_log_directory = directory / controller_name;

        // Try to create the log directory (and any needed parent directories).
        std::error_code ec;
        std::filesystem::create_directories(control_log_directory, ec);
        if (ec) {
            // If directory creation fails, log an error and exit.
            ::_acsl_::_message_::SIMULATOR_ERROR("[SIMCTL]: ERROR CREATING DIRECTORY: ", ec.message());
            return false;
        } else {
            // Log successful creation of the controller log directory.
            ::_acsl_::_message_::SIMULATOR_INFO("[SIMCTL]: CREATED CONTROLLER LOG DIRECTORY: " + control_log_directory.string());
        }

        // Build the full path to the controller's log file.
        std::filesystem::path log_file_path = control_log_directory / "controller_log.log";
        ::_acsl_::_message_::SIMULATOR_INFO("[SIMCTL]: ATTEMPTING FILE CREATION AT: " + log_file_path.string());

        // Attempt to open the log file stream for writing.
        auto file_stream = boost::make_shared<std::ofstream>(log_file_path.string());
        if (!file_stream->is_open()) {
            // If file cannot be opened, log an error and exit.
            ::_acsl_::_message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO OPEN LOG FILE: ", log_file_path.string());
            return false;
        }

        // Add a "Tag" attribute for filtering controller log messages, if not present.
        if (!m_logger.GetControlLogger().get_attributes().count("Tag")) {
            m_logger.GetControlLogger().add_attribute("Tag",
                    attrs::constant<std::string>("ControllerTag"));
        }

        // Define and set up a synchronous Boost.Log sink for text file output.
        typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
        boost::shared_ptr<text_sink> controller_sink = boost::make_shared<text_sink>();
        // Ensure immediate flushing of log messages to the file.
        controller_sink->locked_backend()->auto_flush(true);
        // Add the log file stream to the sink backend.
        controller_sink->locked_backend()->add_stream(file_stream);
        // Configure the sink with a detailed log message formatter.
        controller_sink->set_formatter(
            expr::stream
                << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] "
                << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] "
                << "[" << expr::attr<std::string>("Tag") << "] "
                << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] "
                << "[" << expr::attr<unsigned int>("LineID") << "] "
                << expr::smessage
        );
        // Register the sink with the core logging system.
        logging::core::get()->add_sink(controller_sink);
        // Only allow log messages with "Tag" == "ControllerTag" to be written to this file.
        controller_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "ControllerTag");
        // Add common log attributes (timestamps, line IDs, etc.).
        logging::add_common_attributes();
        // Log successful log file opening.
        ::_acsl_::_message_::SIMULATOR_INFO("[SIMCTL]: CONTROLLER LOG FILE OPENED");

        // Copy over the parameter files.
        // 1. Build path to the source parameter file
        std::filesystem::path param_source = "../chrono-assets/parameters";
        param_source /= platform_name;
        param_source /= controller_name;
        param_source /= "gains_" + controller_name + ".json";

        // 2. Check if the source file exists
        if (!std::filesystem::exists(param_source)) {
            ::_acsl_::_message_::SIMULATOR_ERROR("[SIMCTL]: GAINS FILE NOT FOUND: ", param_source.string());
        } else {
            // 3. Create the params folder within the controller log directory
            std::filesystem::path param_dest_dir = control_log_directory / "params";
            std::error_code param_ec;
            std::filesystem::create_directories(param_dest_dir, param_ec);
            if (param_ec) {
                ::_acsl_::_message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO CREATE PARAMETER DIRECTORY: ", param_dest_dir.string());
            } else {
                // 4. Build the destination file path
                std::filesystem::path param_dest = param_dest_dir / ("gains_" + controller_name + ".json");
                // 5. Copy the file
                std::error_code copy_ec;
                std::filesystem::copy_file(param_source, param_dest, std::filesystem::copy_options::overwrite_existing, copy_ec);
                if (copy_ec) {
                    ::_acsl_::_message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO COPY GAINS FILE: ", copy_ec.message());
                } else {
                    ::_acsl_::_message_::SIMULATOR_INFO("[SIMCTL]: COPIED GAINS FILE TO: " + param_dest.string());
                }
            }
        }

        // Return success if all steps completed.
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        // Log any filesystem-specific exception and exit.
        ::_acsl_::_message_::SIMULATOR_ERROR("[SIMCTL]: FILESYSTEM ERROR: ", e.what());
    } catch (const std::exception& e) {
        // Log any other exception and exit.
        ::_acsl_::_message_::SIMULATOR_ERROR("[SIMCTL]: EXCEPTION: ", e.what());
    }

    // Return failure if an error occurred.
    return false;
}

} // namespace _filesystem_


}   // namespace _logger_
    
}   // namespace _acsl_


