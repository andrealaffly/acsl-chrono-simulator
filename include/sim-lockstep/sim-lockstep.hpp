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
 * File:        sim-lockstep.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 31, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Lockstep client to the ACSL flightstack (HIL / SIL, mode.enable_flightstack_loop in sim-config.yaml).
 *              The flightstack is the server (lockstep_node), this class is the client:
 *              - Handshake : once, before the physics clock starts. Sends protocol version, platform, controller and
 *                            the step size. Rejected -> the flightstack's reason is returned to the caller.
 *              - Step      : every simulator step. Sends time + state + reference, blocks until the 8 normalized
 *                            motor thrusts come back (strict lockstep, the simulator owns the clock).
 *              Constructed only in flightstack-loop mode, so a normal simulation never starts ROS 2.
 *
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_LOCKSTEP_HPP_
#define SIM_LOCKSTEP_HPP_

#include <array>                                        // For the 8 motor outputs
#include <chrono>                                       // For the service timeouts
#include <cstdint>                                      // For uint64_t
#include <memory>                                       // For std::shared_ptr
#include <string>                                       // For std::string
#include <filesystem>                                   // For the run log directory
#include <fstream>                                      // For the network log
#include <vector>                                       // For the round-trip time samples

#include "rclcpp/rclcpp.hpp"                            // ROS2 Client Library C++ header
#include "acsl_lockstep_interfaces/srv/handshake.hpp"   // Lockstep handshake service (identical copy in both repos)
#include "acsl_lockstep_interfaces/srv/step.hpp"        // Lockstep step service (identical copy in both repos)
#include "acsl_lockstep_interfaces/srv/finish.hpp"      // Lockstep finish service (identical copy in both repos)

namespace _acsl_
{
namespace _lockstep_
{

// Result of a handshake attempt. TIMEOUT is worth a retry, REJECTED is final (the flightstack exits)
enum class HandshakeResult { ACCEPTED, REJECTED, TIMEOUT };

// =====================================================================================================================
// class simlockstep
//
// Purpose:
//   - Owns a private ROS 2 node + executor with the two service clients to the flightstack.
//   - IsServerUp() : non-blocking probe, used by the wait gate (render-only loop) before the handshake.
//   - Handshake()  : blocking, returns true if the flightstack accepted this platform / controller / step size.
//   - Step()       : blocking, one controller evaluation on the flightstack per simulator step.
//
// Usage:
//   simlockstep client;
//   while (!client.IsServerUp()) { /* render only, no physics step */ }
//   if (!client.Handshake("quadm", "mrac_geometric", 0.001, msg)) { /* print msg, exit */ }
//   client.Step(request, motors, msg);   // every step
//
// Notes:
//   - Runtime: the terminal running the simulator must source libraries/chrono-ros-messages/install/local_setup.bash
//     (the message type support is loaded at runtime).
//   - ROS 2 is started WITHOUT its signal handlers, so Ctrl+C / closing the window behave as before.
// =====================================================================================================================
class simlockstep
{

    public:
        using HandshakeSrv = acsl_lockstep_interfaces::srv::Handshake;
        using StepSrv      = acsl_lockstep_interfaces::srv::Step;
        using StepRequest  = StepSrv::Request;
        using FinishSrv    = acsl_lockstep_interfaces::srv::Finish;

        // domain_id: ROS 2 domain of the SIL/HIL link (mode.ros_domain_id in sim-config.yaml)
        // log_dir: run log folder for network.log and network_summary.log (empty = no network log)
        simlockstep(size_t domain_id, const std::filesystem::path& log_dir);
        ~simlockstep();

        // One client per simulation: no copies
        simlockstep(const simlockstep&) = delete;
        simlockstep& operator=(const simlockstep&) = delete;

        // Non-blocking: true once BOTH flightstack services are discovered
        bool IsServerUp() const;

        // Blocking (up to handshake_timeout_). message = flightstack summary (ACCEPTED) or the reason (REJECTED / TIMEOUT)
        HandshakeResult Handshake(const std::string& platform, const std::string& controller, 
                                  double step_size, std::string& message);

        // Blocking (up to step_timeout_). seq is filled here. false = timeout / seq mismatch / ok == false (see message)
        bool Step(const StepRequest& request, std::array<double, 8>& motors, std::string& message);

        // Blocking (up to handshake_timeout_). Tells the flightstack the simulation is over; it replies, then exits.
        // true = the flightstack computed exactly as many Steps as were sent. message = summary or reason
        bool Finish(double time, std::string& message);

        // Number of Steps completed (= seq of the next Step)
        uint64_t GetSeq() const { return seq_; }

        // Round-trip timing summary of all Steps so far (sim_time = simulation time reached, for the real-time factor)
        std::string TimingSummary(double sim_time) const;

    private:
        // ROS 2
        rclcpp::Node::SharedPtr node_;                                   // private node "acsl_sim_lockstep"
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;  // private executor, spun only while waiting for a reply.
                                                                         // A pointer: it must be created AFTER rclcpp::init
        rclcpp::Client<HandshakeSrv>::SharedPtr handshake_client_;       // /acsl_lockstep/handshake
        rclcpp::Client<StepSrv>::SharedPtr step_client_;                 // /acsl_lockstep/step
         rclcpp::Client<FinishSrv>::SharedPtr finish_client_;            // /acsl_lockstep/finish
        bool owns_ros_init_ = false;                                     // true if this class called rclcpp::init
        
        // Lockstep state
        uint64_t seq_ = 0;                                               // seq of the next Step (the flightstack checks it)

        // Network log and round-trip timing (simulator side)
        std::filesystem::path log_dir_;                                  // run log folder (empty = no network log)
        std::ofstream network_log_;                                      // network.log: time, seq, rtt_us, motor1..motor8
        std::vector<double> rtt_us_;                                     // round-trip time of every Step [us]
        std::chrono::steady_clock::time_point first_step_wall_;          // wall time when the first Step was sent

        // Timeouts. Handshake: first contact. Step: generous for HIL over the LAN (a normal reply is ~1 ms)
        static constexpr std::chrono::milliseconds handshake_timeout_{2000};
        static constexpr std::chrono::milliseconds step_timeout_{1000};

};

}   // namespace _lockstep_
}   // namespace _acsl_

#endif // SIM_LOCKSTEP_HPP_