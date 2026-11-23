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
 * File:        sim-controllers.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 05, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header only file to include all the platforms' controllers.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_CONTROLLERS_HPP_
#define SIM_CONTROLLERS_HPP_

#include "sim-messages.hpp"
#include "sim-control-base.hpp"
#include "sim-trajectory.hpp"
#include "sim-logger.hpp"

/* =====================================================================================================================
  sim-controllers.hpp

  Purpose:
    - Central registry for all UAV controller class includes.
    - This ensures that `sim-bridge.hpp` (and related simulation code)
      can discover, instantiate, and use all available controller definitions for each platform.

  How to add a new controller:
    1. Include your derived controller class header in the "Includes" section below.
    2. Add a new `bool <controller_name>{};` member to the appropriate platform struct inside `controllers`.
       !! The member name **must exactly match** the name used in your YAML config. !!
    3. Update the `asVectorRef()` helper inside `controllers` with a tuple for the new entry,
       so that dynamic iteration and assignment can be performed without hardcoding.
    4. Update the factory logic in `createSelectedController()` to handle instantiation and
       initialization for the new controller type.
    5. Verify the controller is listed in the YAML configuration file for its platform,
       and that a matching implementation exists in your codebase.

  Notes:
    - Keep includes grouped and sorted alphabetically for maintainability.
    - This file is intentionally limited to includes and the **controllers** struct —
      no functional or simulation logic belongs here.
==================================================================================================================== */


// -----------------------------------------------------------------------------
// Includes for the different UAV platforms' controllers (add more as needed)
// -----------------------------------------------------------------------------
#include "pid-omega-qrbp.hpp"
#include "mrac-omega-qrbp.hpp"
#include "mrac-observer-qrbp.hpp"
#include "mrac-long-lat-qrbp.hpp"
#include "pid-quaternion-qrbp.hpp"
#include "pid-geometric-qrbp.hpp"
#include "mrac-geometric-qrbp.hpp"
#include "mrac-geometric-tailsitter.hpp"
#include "mrac-hybrid-tailsitter.hpp"
#include "mrac-omega-quadm.hpp"
#include "mrac-geometric-quadm.hpp"
#include "mrac-geometric-na-ebci-quadm.hpp"
#include "mrac-omega-x8.hpp"
#include "mrac-geometric-x8.hpp"

namespace _acsl_
{

namespace _bridge_
{

// -----------------------------------------------------------------------------
// Structure: controllers
//
// Purpose:
//   Holds grouped controller selection flags for each UAV platform. Each inner
//   struct (qrbp, x8copter, tailsitter) contains boolean flags for available
//   control algorithms (e.g., PID, MRAC).
//
// Pattern:
//   1) For each platform (e.g., qrbp, x8copter), add controller flags inside
//      the matching inner struct, named for clarity (e.g., pid, mrac, tlmrac).
//   2) Use asVectorRef() for dynamic access and assignment, regardless of the
//      concrete platform or controller set.
//
// How to add a new platform or controller flag:
//   a) Extend the relevant platform struct with a new bool member.
//   b) Add a corresponding entry to asVectorRef()—the tuple includes platform,
//      controller, and the reference to the flag.
//   c) Update createSelectedController() with the logic to instantiate the
//      matching controller, passing required arguments.
//
// How to remove a platform or controller flag:
//   a) Remove the bool from its struct.
//   b) Remove the tuple entry from asVectorRef().
//   c) Remove/adjust factories and logic in createSelectedController().
//
// Notes:
//   - All controller flags are expected to be mutually exclusive: exactly one
//     controller should be active (set to true) per simulation run.
//   - Mismatches between controller request and selected platform are detected
//     and reported via errors in validateExclusiveSelection().
// -----------------------------------------------------------------------------

struct controllers
{
    // -------------------------------------------------------------------------
    // Platform group: QRBP
    // Holds controller flags specific to quadrotor biplane platform.
    // -------------------------------------------------------------------------
    struct qrbp
    {
        bool pid_omega{};       // PID omega controller for QRBP
        bool mrac_omega{};      // MRAC omega controller for QRBP
        bool mrac_observer{};   // MRAC omega controller with adaptive observer
        bool mrac_long_lat{};   // MRAC longitudinal/lateral controller for QRBP
        bool pid_quaternion{};  // PID with quaternion control for QRBP
        bool pid_geometric{};   // PID with geometric control for QRBP
        bool mrac_geometric{};  // MRAC with geometric control for QRBP
    } qrbp;

    // -------------------------------------------------------------------------
    // Platform group: tailsitter
    // Holds controller flags specific to tailsitter.
    // -------------------------------------------------------------------------
    struct tailsitter
    {
        bool mrac_geometric{};  // MRAC with geometric control for TAILSITTER
        bool mrac_hybrid{};     // MRAC with geometric il and hybrid ol
    } tailsitter;

    // -------------------------------------------------------------------------
    // Platform group: quadm
    // Holds controller flags specific to quadm.
    // -------------------------------------------------------------------------
    struct quadm
    {
        bool mrac_omega{};              // MRAC omega controller for QUADM
        bool mrac_geometric{};          // MRAC geometric controller for QUADM
        bool mrac_geometric_na_ebci{};  // MRAC geometric with non-adaptive ebci controller for QUADM
    } quadm;
    
    // -------------------------------------------------------------------------
    // Platform group: x8
    // Holds controller flags specific to x8.
    // -------------------------------------------------------------------------
    struct x8
    {
        bool mrac_omega{};      // MRAC omega controller for X8
        bool mrac_geometric{};  // MRAC geometric controller for X8
    } x8;

    // -------------------------------------------------------------------------
    // Returns all controller selection flags as:
    // 	 {platform, controller, reference} tuples.
    //
    // Usage:
    //   Mutable iteration (can assign to flags):
    //     for (auto& [platform_name, controller_name, ref] : ctrls.asVectorRef()) { ref = true; }
    // -------------------------------------------------------------------------
    std::vector<std::tuple<std::string, std::string, bool&>> asVectorRef() {
        return {
            {"qrbp", "pid_omega", qrbp.pid_omega},
            {"qrbp", "mrac_omega", qrbp.mrac_omega},
            {"qrbp", "mrac_observer", qrbp.mrac_observer},
            {"qrbp", "mrac_long_lat", qrbp.mrac_long_lat},
            {"qrbp", "pid_quaternion", qrbp.pid_quaternion},
            {"qrbp", "pid_geometric", qrbp.pid_geometric},
            {"qrbp", "mrac_geometric", qrbp.mrac_geometric},
            {"tailsitter", "mrac_geometric", tailsitter.mrac_geometric},
            {"tailsitter", "mrac_hybrid", tailsitter.mrac_hybrid},
            {"quadm", "mrac_omega", quadm.mrac_omega},
            {"quadm", "mrac_geometric", quadm.mrac_geometric},
            {"quadm", "mrac_geometric_na_ebci", quadm.mrac_geometric_na_ebci},
            {"x8", "mrac_omega", x8.mrac_omega},
            {"x8", "mrac_geometric", x8.mrac_geometric},
        };
    }

    // -------------------------------------------------------------------------
    // Factory: createSelectedController()
    //
    // Purpose:
    //   Instantiates and returns a unique pointer to the selected controller
    //   implementation, based on which controller flag is active for the
    //   specified platform.
    //
    // Parameters:
    //   platform    - String name ("qrbp", "x8copter", "tailsitter") indicating
		// 							   current active platform attached to the system
    //   logger      - Reference to the simulator's logging instance.
    //   trajectory  - Reference to the currently active trajectory object.
    //
    // Returns:
    //   std::unique_ptr<controller_base> to the newly created controller.
    //   Returns nullptr if selection is invalid or not implemented.
    //
    // Workflow:
    //   1. Calls validateExclusiveSelection(platform) to verify the selection,
    //      and retrieves the (platform, controller) pair.
    //   2. Based on the result, instantiates the correct controller, passing
    //      necessary log and trajectory objects to the constructor.
    //   3. Logs the attachment action for traceability.
    //   4. Returns a base-class pointer for polymorphic control.
    //
    // Notes:
    //   - Return nullptr for unimplemented controller types or mismatches.
    // -------------------------------------------------------------------------
    std::unique_ptr< ::_acsl_::_control_::controller_base >
    createSelectedController(std::string platform,
                            ::_acsl_::_logger_::simlog& logger,
                            ::_acsl_::_trajectory_::trajectorybase& trajectory) const
    {
        auto active = this->validateExclusiveSelection(platform);

        if (active.first == "qrbp" && active.second == "pid_omega") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING PID OMEGA CONTROLLER TO QRBP");
            return std::make_unique< ::_acsl_::_qrbp_::_pid_omega_::pid_omega >(logger, trajectory);
        }
        else if (active.first == "qrbp" && active.second == "mrac_omega") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC OMEGA CONTROLLER TO QRBP");
            return std::make_unique< ::_acsl_::_qrbp_::_mrac_omega_::mrac_omega >(logger, trajectory);
        }
        else if (active.first == "qrbp" && active.second == "mrac_observer") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC OMEGA W/ ADAPTIVE OBSERVER TO QRBP");
            return std::make_unique< ::_acsl_::_qrbp_::_mrac_observer_::mrac_observer >(logger, trajectory);
        }
        else if (active.first == "qrbp" && active.second == "mrac_long_lat") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC LONGITUDINAL/LATERAL CONTROLLER TO QRBP");
            return std::make_unique< ::_acsl_::_qrbp_::_mrac_long_lat_::mrac_long_lat >(logger, trajectory);
        }
        else if (active.first == "qrbp" && active.second == "pid_quaternion") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING PID QUATERNION CONTROLLER TO QRBP");
            return std::make_unique< ::_acsl_::_qrbp_::_pid_quaternion_::pid_quaternion >(logger, trajectory);
        }
        else if (active.first == "qrbp" && active.second == "pid_geometric") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING PID GEOMETRIC CONTROLLER TO QRBP");
            return std::make_unique< ::_acsl_::_qrbp_::_pid_geometric_::pid_geometric >(logger, trajectory);
        }
        else if (active.first == "qrbp" && active.second == "mrac_geometric") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC GEOMETRIC CONTROLLER TO QRBP");
            return std::make_unique< ::_acsl_::_qrbp_::_mrac_geometric_::mrac_geometric >(logger, trajectory);
        }
        else if (active.first == "tailsitter" && active.second == "mrac_geometric") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC_GEOMETRIC CONTROLLER TO TAILSITTER");
            return std::make_unique< ::_acsl_::_tailsitter_::_mrac_geometric_::mrac_geometric >(logger, trajectory);
        }
        else if (active.first == "tailsitter" && active.second == "mrac_hybrid") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC_HYBRID CONTROLLER TO TAILSITTER");
            return std::make_unique< ::_acsl_::_tailsitter_::_mrac_hybrid_::mrac_hybrid >(logger, trajectory);
        }
        else if (active.first == "quadm" && active.second == "mrac_omega") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC OMEGA CONTROLLER TO QUADM");
            return std::make_unique< ::_acsl_::_quadm_::_mrac_omega_::mrac_omega >(logger, trajectory);
        }
        else if (active.first == "quadm" && active.second == "mrac_geometric") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC GEOMETRIC CONTROLLER TO QUADM");
            return std::make_unique< ::_acsl_::_quadm_::_mrac_geometric_::mrac_geometric >(logger, trajectory);
        }
        else if (active.first == "quadm" && active.second == "mrac_geometric_na_ebci") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC GEOMETRIC NON-ADAPTIVE EBCI CONTROLLER TO QUADM");
            return std::make_unique< ::_acsl_::_quadm_::_mrac_geometric_na_ebci_::mrac_geometric_na_ebci >(logger, trajectory);
        }
        else if (active.first == "x8" && active.second == "mrac_omega") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC OMEGA CONTROLLER TO X8");
            return std::make_unique< ::_acsl_::_x8_::_mrac_omega_::mrac_omega >(logger, trajectory);
        }
        else if (active.first == "x8" && active.second == "mrac_geometric") {
            _message_::SIMULATOR_INFO("[SIMCTL]: ATTACHING MRAC GEOMETRIC CONTROLLER TO X8");
            return std::make_unique< ::_acsl_::_x8_::_mrac_geometric_::mrac_geometric >(logger, trajectory);
        }
        // [Add more else if branches for other controllers as needed]
        // e.g., x8copter and tailsitter controllers
        // else if (active.first == "x8copter" && active.second == "pid") { ... }

        return nullptr;
    }

    // -------------------------------------------------------------------------
    // Validates mutual exclusivity for controller selection, ensuring that
    // exactly one controller is set to true, and returns its (platform, controller)
    // name pair. Reports errors if none or multiple are selected, or if the platform
    // passed doesn't match the selected controller's platform.
    //
    // Parameters:
    //   platform  - String for the expected platform to match against selected controller.
    //
    // Returns:
    //   pair of strings {platform_name, controller_name} representing active controller.
    //
    // Workflow:
    //   1. Iterates all controller flags using asVectorRef().
    //   2. Counts and identifies which controller is selected.
    //   3. Error if count != 1, or if platform does not match.
    // -------------------------------------------------------------------------
    std::pair<std::string, std::string> validateExclusiveSelection(std::string platform) const
    {
        std::string crrplatform;
        std::string crrcontroller;
        int count = 0;
        for (const auto& [p, c, val] : const_cast<controllers*>(this)->asVectorRef()) {
            if (val) {
                crrplatform = p;
                crrcontroller = c;
                count++;
            }
        }
        if (count == 0) {
            _message_::SIMULATOR_ERROR("[SIMCTL]: NO CONTROLLER SELECTED - EXACTLY ONE MUST BE SET TO TRUE.");
        } else if (count > 1) {
            _message_::SIMULATOR_ERROR("[SIMCTL]: MULTIPLE CONTROLLERS SELECTED - EXACTLY ONE MUST BE SET TO TRUE.");
        } else if (crrplatform != platform) {
            std::ostringstream oss;
            oss << "[SIMCTL]: PLATFORM & CONTROLLER MISMATCH, PICKED PLATFORM: "
                << crrplatform << ", PICKED CONTROLLER: " << crrcontroller;
            _message_::SIMULATOR_ERROR(oss.str());
        }
        return {crrplatform, crrcontroller};
    }
};


} // namespace _bridge_

} // namespace _acsl_


#endif  // SIM_CONTROLLERS_HPP_