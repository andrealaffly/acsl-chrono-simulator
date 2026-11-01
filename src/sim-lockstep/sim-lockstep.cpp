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
 * File:        sim-lockstep.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 31, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Implementation of the lockstep client to the ACSL flightstack (see sim-lockstep.hpp).
 *
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-lockstep.hpp"
#include "sim-messages.hpp"

#include <cstdlib>
#include <algorithm>    // For std::sort (round-trip percentiles)
#include <iomanip>      // For std::setprecision
#include <numeric>      // For std::accumulate
#include <sstream>      // For the timing summary

namespace _acsl_
{
namespace _lockstep_
{   

// =====================================================================================================================
// Constructor: start ROS 2 (no signal handlers), create the private node, the two clients and the executor
// =====================================================================================================================
simlockstep::simlockstep(size_t domain_id, const std::filesystem::path& log_dir) : log_dir_(log_dir)
{
    // Make the lockstep message type support findable without sourcing anything (works from a terminal, the GUI or
    // the wrapper). ROS 2 looks the DDS type support library up in LD_LIBRARY_PATH when the clients are created and
    // reads the variable at that moment, so prepending it here is enough. Only flightstack-loop runs get here.
    const char* ld_path = std::getenv("LD_LIBRARY_PATH");
    std::string lockstep_ld_path = LOCKSTEP_INTERFACES_LIB_DIR;
    if (ld_path && *ld_path) {
        lockstep_ld_path += ":" + std::string(ld_path);
    }
    setenv("LD_LIBRARY_PATH", lockstep_ld_path.c_str(), 1);

    // SignalHandlerOptions::None: ROS must not take over SIGINT, the simulator keeps its own Ctrl+C / window close
    if (!rclcpp::ok()) {
        // Domain from sim-config.yaml, not from ~/.bashrc: only the simulator's own ROS context uses it
        rclcpp::InitOptions init_options;
        init_options.set_domain_id(domain_id);
        rclcpp::init(0, nullptr, init_options, rclcpp::SignalHandlerOptions::None);
        owns_ros_init_ = true;
    }

    // Create the node.
    node_ = std::make_shared<rclcpp::Node>("acsl_sim_lockstep");

    // create_client throws if the message type support cannot be loaded (overlay not sourced)
    try {
        // No QoS argument on purpose: services default QoS, same form on every ROS 2 distro
        handshake_client_ = node_->create_client<HandshakeSrv>("/acsl_lockstep/handshake");
        step_client_      = node_->create_client<StepSrv>("/acsl_lockstep/step");
        finish_client_    = node_->create_client<FinishSrv>("/acsl_lockstep/finish");
    } catch (const std::exception& e) {
        ::_acsl_::_message_::SIMULATOR_ERROR(
            "[SIMLKS]: Could not create the lockstep clients. Is libraries/chrono-ros-messages built "
            "(acsl_lockstep_interfaces)? ROS 2 says: ",
            std::string(e.what()));
    }

    // Created here, after rclcpp::init: the executor's constructor needs an initialized ROS 2 context
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Get the domain id, it should match.
    ::_acsl_::_message_::SIMULATOR_INFO("[SIMLKS]: Lockstep client ready. ROS domain ID:", domain_id);

    // Network log next to physics.log. Buffered (no flush per line), flushed on a Step failure and closed at Finish
    if (!log_dir_.empty()) {
        network_log_.open(log_dir_ / "network.log");
        if (network_log_.is_open()) {
            network_log_ << std::setprecision(9)
                         << "time, seq, rtt_us, motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8\n";
            ::_acsl_::_message_::SIMULATOR_INFO("[SIMLKS]: Network log:", (log_dir_ / "network.log").string());
        } else {
            ::_acsl_::_message_::SIMULATOR_WARNING("[SIMLKS]: Could not open the network log in:", log_dir_.string());
        }
    }
    rtt_us_.reserve(1 << 16);
}

// =====================================================================================================================
// Destructor: release the ROS entities before shutting ROS 2 down
// =====================================================================================================================
simlockstep::~simlockstep()
{
    // Wokaround so that simulator owns the SIGINT handler, and ROS 2 does not take over Ctrl+C / window close
    executor_->remove_node(node_);
    executor_.reset();
    handshake_client_.reset();
    step_client_.reset();
    finish_client_.reset();
    node_.reset();

    if (owns_ros_init_ && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

// =====================================================================================================================
// IsServerUp: non-blocking. Discovery runs in the DDS background threads, no spinning needed
// =====================================================================================================================
bool simlockstep::IsServerUp() const
{
    return handshake_client_->service_is_ready() && 
           step_client_->service_is_ready() && 
           finish_client_->service_is_ready();
}

// =====================================================================================================================
// Handshake: send protocol version, platform, controller and step size, wait for the verdict
// =====================================================================================================================
HandshakeResult simlockstep::Handshake(const std::string& platform, const std::string& controller, double step_size,
                                       std::string& message)
{
    // Package the handshake request. The flightstack will check the protocol version, platform, controller and step size.
    auto request = std::make_shared<HandshakeSrv::Request>();
    request->protocol_version = HandshakeSrv::Request::PROTOCOL_VERSION;
    request->platform         = platform;
    request->controller       = controller;
    request->step_size        = step_size;

    auto result = handshake_client_->async_send_request(request);

    // Spin only this private executor until the reply arrives or the timeout expires
    if (executor_->spin_until_future_complete(result.future, handshake_timeout_) != rclcpp::FutureReturnCode::SUCCESS) {
        handshake_client_->remove_pending_request(result.request_id);
        message = "No handshake reply from the flightstack within " + std::to_string(handshake_timeout_.count()) + " ms.";
        return HandshakeResult::TIMEOUT;
    }

    auto response = result.future.get();

    // Rejected: the flightstack's own reason (it exits right after sending it)
    if (!response->accepted) {
        message = response->message;
        return HandshakeResult::REJECTED;
    }

    message = "Flightstack platform: " + response->platform + ", controller: " + response->controller +
              ", protocol version: " + std::to_string(response->protocol_version);
    return HandshakeResult::ACCEPTED;
}

// =====================================================================================================================
// Step: one controller evaluation on the flightstack. Blocks until the motors come back (strict lockstep)
// =====================================================================================================================
bool simlockstep::Step(const StepRequest& request, std::array<double, 8>& motors, std::string& message)
{
    // Copy the caller's request (time, state, reference) and stamp the sequence number
    auto step_request = std::make_shared<StepRequest>(request);
    step_request->seq = seq_;

    // Round trip: from just before sending to the reply (DDS both ways + the flightstack's controller compute)
    const auto t_send = std::chrono::steady_clock::now();
    auto result = step_client_->async_send_request(step_request);

    if (executor_->spin_until_future_complete(result.future, step_timeout_) != rclcpp::FutureReturnCode::SUCCESS) {
        step_client_->remove_pending_request(result.request_id);
        message = "No step reply from the flightstack within " + std::to_string(step_timeout_.count()) +
                  " ms (seq " + std::to_string(seq_) + ", t = " + std::to_string(request.time) + " s).";
        network_log_.flush();   // keep the rows leading up to the failure
        return false;
    }

    auto response = result.future.get();

    // The reply must belong to this request
    if (response->seq != step_request->seq) {
        message = "Step reply out of order: sent seq " + std::to_string(step_request->seq) +
                  ", got seq " + std::to_string(response->seq) + ".";
        network_log_.flush();   // keep the rows leading up to the failure
        return false;
    }

    // ok == false: non-finite controller output, or no accepted handshake on the flightstack side
    if (!response->ok) {
        message = "Flightstack returned ok == false at seq " + std::to_string(seq_) + ", t = " +
                  std::to_string(request.time) + " s. See the flightstack terminal.";
        network_log_.flush();   // keep the rows leading up to the failure
        return false;
    }

    motors = response->motors;

    // Timing + network log row
    const double rtt_us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t_send).count();
    if (rtt_us_.empty()) {
        first_step_wall_ = t_send;
    }
    rtt_us_.push_back(rtt_us);

    if (network_log_.is_open()) {
        network_log_ << request.time << ", " << step_request->seq << ", " << rtt_us;
        for (const double m : motors) {
            network_log_ << ", " << m;
        }
        network_log_ << '\n';
    }

    ++seq_;
    return true;
}

// =====================================================================================================================
// Finish: tell the flightstack the simulation is over and compare step counts
// =====================================================================================================================
bool simlockstep::Finish(double time, std::string& message)
{
    auto request = std::make_shared<FinishSrv::Request>();
    request->steps_sent = seq_;
    request->time       = time;

    // Close the network log and write the timing summary first, so they exist even if the reply never comes
    if (network_log_.is_open()) {
        network_log_.close();
        std::ofstream summary(log_dir_ / "network_summary.log");
        summary << TimingSummary(time) << '\n';
    }

    auto result = finish_client_->async_send_request(request);

    if (executor_->spin_until_future_complete(result.future, handshake_timeout_) != rclcpp::FutureReturnCode::SUCCESS) {
        finish_client_->remove_pending_request(result.request_id);
        message = "No finish reply from the flightstack within " + std::to_string(handshake_timeout_.count()) + " ms.";
        return false;
    }

    auto response = result.future.get();

    message = "Steps sent: " + std::to_string(seq_) + ", computed by the flightstack: " +
              std::to_string(response->steps_done);
    return response->ok;
}

// =====================================================================================================================
// TimingSummary: round-trip statistics of all Steps (simulator side)
// =====================================================================================================================
std::string simlockstep::TimingSummary(double sim_time) const
{
    if (rtt_us_.empty()) {
        return "No Steps were timed.";
    }

    std::vector<double> sorted(rtt_us_);
    std::sort(sorted.begin(), sorted.end());

    const size_t n      = sorted.size();
    const double sum_us = std::accumulate(sorted.begin(), sorted.end(), 0.0);
    const double p99_us = sorted[std::min(n - 1, static_cast<size_t>(0.99 * static_cast<double>(n)))];
    const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - first_step_wall_).count();

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1)
        << "steps: " << n
        << " | RTT [us] min " << sorted.front() << ", mean " << sum_us / static_cast<double>(n)
        << ", p99 " << p99_us << ", max " << sorted.back()
        << " | waiting on the flightstack: " << std::setprecision(3) << sum_us * 1e-6 << " s of " << wall_s << " s wall"
        << " | real-time factor: " << sim_time / wall_s;
    return oss.str();
}

} // namespace _lockstep_
} // namespace _acsl_