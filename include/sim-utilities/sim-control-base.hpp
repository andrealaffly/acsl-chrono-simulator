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
 * File:        sim-control-base.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the baseclass of the controller. It is the same for all the controllers to be used and
 *              is platform agnostic. It has the same input and the output. Input - States at time t. Output - Control
 *              action at time t.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_CONTROL_BASE_HPP_
#define SIM_CONTROL_BASE_HPP_

// ====================================================================================
//  ACSL Physics Sim includes
// ====================================================================================
#include "sim-helpers.hpp"                  // Import for the helper functions
#include "sim-templated-helpers.hpp"        // Import for the templated headers
#include "sim-logger.hpp"                   // Import the header for the Logger API
#include "sim-trajectory.hpp"               // Import for the trajectory API
#include "continuous-lyapunov-equation.hpp" // Import API for solving Lyapunov eqns
#include "deadzone-operator.hpp"            // Import API for dead zone operator
#include "projection-operator.hpp"          // Import API for projection operator
#include "adaptive-laws.hpp"                // Import API for adaptive laws

// ====================================================================================
// Boost includes
// ====================================================================================
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>           // Include the correct header for bind
#include <boost/bind/placeholders.hpp>   // Include the placeholders
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/interval.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/math/tools/polynomial.hpp>

namespace bph = boost::placeholders;    // Use the correct namespace

namespace _acsl_
{

namespace _control_
{

// =====================================================================================================================
// class controller_base
//
// Purpose:
//   - Abstract base class for all UAV controller implementations.
//   - Encapsulates actuator thrust commands, parameter reading, initialization,
//     state updating, control execution, and logging integration.
//   - Designed to be subclassed for specific controller algorithms.
//   - Maintains a reference to a simlog instance for telemetry and debugging.
//   - Maintains a reference to a trajectory instace for getting the user defined trajectory.
//
// Notes:
//   - Only thrust indices t1-t4 are typically used for quadcopters; t5-t8 for larger platforms (hex/oct).
//   - All interface methods are virtual and should be overridden in derived classes as needed.
//   - Logger instance should exist for the whole simulation lifecycle.
//   - Trajectory instance should exist for the whole simulation lifecycle.
// =====================================================================================================================
class controller_base
{

public:
    // -------------------------------------------------------------------------------------------------
    // Constructor:
    //   - Initializes the control input vector to zero.
    //   - Stores a reference to the external simlog logger for use across control routines.
    // -------------------------------------------------------------------------------------------------
    controller_base(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
        : m_logger(logger), m_traj(trajectory), 
          control_input(Eigen::Matrix<float, 8, 1>::Zero()), 
          com_u(Eigen::Matrix<double, 4, 1>::Zero())
    {}

    // -------------------------------------------------------------------------------------------------
    // Virtual Destructor:
    //   - Ensures proper cleanup in derived classes due to polymorphic usage.
    // -------------------------------------------------------------------------------------------------
    virtual ~controller_base() = default;

    // -------------------------------------------------------------------------------------------------
    // Function: read_params
    //   - Reads and loads controller parameters from the specified JSON file.
    //   - To be overridden as needed for controller-specific configuration.
    //   - Default implementation: no operation.
    //
    // @param jsonFile Path to parameter file (optional).
    // -------------------------------------------------------------------------------------------------
    virtual void read_params([[maybe_unused]] const std::string& jsonFile) {}

    // -------------------------------------------------------------------------------------------------
    // Function: init
    //   - Initializes controller state and internal conditions.
    //   - Override as needed in derived classes.
    //   - Default implementation: no operation.
    // -------------------------------------------------------------------------------------------------
    virtual void init() {}

    // -------------------------------------------------------------------------------------------------
    // Function: update
    //   - Updates controller state using system and trajectory data.
    //   - To be overridden for platform-specific control logic.
    //   - All arguments may be unused in base class.
    //
    // @param time  Current simulation time [seconds].
    // @param x, y, z  Position (meters).
    // @param vx, vy, vz  Velocity (meters/sec).
    // @param q0, q1, q2, q3  Quaternion orientation (w, x, y, z).
    // @param roll, pitch, yaw  Euler angles (radians).
    // @param w_x, w_y, w_z  Angular velocities (rad/sec).
    // -------------------------------------------------------------------------------------------------
    virtual void update([[maybe_unused]] double time,
                        [[maybe_unused]] double x,
                        [[maybe_unused]] double y,
                        [[maybe_unused]] double z,
                        [[maybe_unused]] double vx,
                        [[maybe_unused]] double vy,
                        [[maybe_unused]] double vz,
                        [[maybe_unused]] double q0,
                        [[maybe_unused]] double q1,
                        [[maybe_unused]] double q2,
                        [[maybe_unused]] double q3,
                        [[maybe_unused]] double roll,
                        [[maybe_unused]] double pitch,
                        [[maybe_unused]] double yaw,
                        [[maybe_unused]] double w_x,
                        [[maybe_unused]] double w_y,
                        [[maybe_unused]] double w_z) {}

    // -------------------------------------------------------------------------------------------------
    // Function: run
    //   - Executes the controller logic for the control cycle or integration time step.
    //   - Override as needed for real-time or timestep-based control.
    //
    // @param time_step_rk4_ Integration step (e.g. RK4 delta time).
    // -------------------------------------------------------------------------------------------------
    virtual void run([[maybe_unused]] const double time_step_rk4_) {}

    // -------------------------------------------------------------------------------------------------
    // Thrust Getter Functions
    //   - Returns commanded thrust for each actuator [t1-t8].
    //   - Useful for forwarding outputs to the physics/simulation layers.
    //
    //   For quadcopters: t1-t4. For hex/oct: t1-t6 or t1-t8.
    // -------------------------------------------------------------------------------------------------
    float get_t1() const { return control_input(0); }
    float get_t2() const { return control_input(1); }
    float get_t3() const { return control_input(2); }
    float get_t4() const { return control_input(3); }
    float get_t5() const { return control_input(4); }
    float get_t6() const { return control_input(5); }
    float get_t7() const { return control_input(6); }
    float get_t8() const { return control_input(7); }

    // -------------------------------------------------------------------------------------------------
    // Torque and moment Getter Functions
    //   - Returns commanded thrust at com, mx, my, mz torques at com
    //   - Useful for debugging control allocation and tuning control parameters.
    // 
    // Fixed size for any configuration as it has the sim treat it as a rigid body
    // -------------------------------------------------------------------------------------------------
    double get_u1() const { return com_u(0); }
    double get_u2() const { return com_u(1); }
    double get_u3() const { return com_u(2); }
    double get_u4() const { return com_u(3); }

protected:
    // -------------------------------------------------------------------------------------------------
    // Control input vector storing actuator thrusts [size 8].
    //   - Written by derived controller logic; read by simulation system.
    // -------------------------------------------------------------------------------------------------
    Eigen::Matrix<float, 8, 1> control_input;

    // -------------------------------------------------------------------------------------------------
    // Control input vector storing the thrust, mx, my, mz at the COM [size 4 (always fixed)]
    //   - Written by derived controller logic; read by simulation system.
    // -------------------------------------------------------------------------------------------------
    Eigen::Matrix<double, 4, 1> com_u;

    // -------------------------------------------------------------------------------------------------
    // Reference to simlog logger instance.
    //   - Allows controllers to write diagnostics/events consistently.
    //   - Should remain valid for the entire simulation duration.
    // -------------------------------------------------------------------------------------------------
    _acsl_::_logger_::simlog& m_logger;

    // -------------------------------------------------------------------------------------------------
    // Reference to trajectory object used by the controller.
    //   - Provides access to the system’s current trajectory for trajectory tracking/feedforward algorithms.
    //   - The referenced trajectory must outlive this controller instance.
    // -------------------------------------------------------------------------------------------------
    ::_acsl_::_trajectory_::trajectorybase& m_traj;
};



}   // namespace _control_

}   // namespace _acsl_


#endif  // SIM_CONTROL_BASE_HPP_