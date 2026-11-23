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
 * File:        nmpc-quadm.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 12, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Header file for the nonlinear model predictive controller (NMPC) for the QUADM. The attitude is kept
 *              on SO(3) as a rotation matrix and the optimal control problem is solved with a Gauss-Newton/iLQR
 *              Riccati backward pass followed by a line search on the nonlinear model. The decision variables are
 *              the four individual motor thrusts, so the per motor thrust limits are enforced exactly.
 *
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef NMPC_QUADM_HPP_
#define NMPC_QUADM_HPP_

#include "sim-control-base.hpp"      // Include for the base class of a controller defined in the simualtor
#include "quadm-parameter-file.hpp"  // Include for the hardcoded quadm parameters that are common for all controllers

#include <array>

namespace _acsl_
{

namespace _quadm_
{

namespace _nmpc_
{

// Define the number of states in the boost array for integration (only the 3 position error integrals)
constexpr int NSI = 3;

// Define the number of steps in the prediction horizon (horizon length [s] = NH * dt_mpc)
constexpr int NH = 30;

// Define the size of the error state [dp, dv, dtheta, domega] and the input [f1, f2, f3, f4]
// NOTE: The full state has 18 numbers (R is 3x3) but the optimizer works on the 12 element error state, where
//       dtheta is a small rotation vector applied on the right (R = R_bar Exp(dtheta)).
constexpr int NX = 12;
constexpr int NU = 4;

// Eigen aliases used by the optimal control problem
using Vec12   = Eigen::Matrix<double, NX, 1>;     // Error state / cost gradient
using Mat12   = Eigen::Matrix<double, NX, NX>;    // State Jacobian A / cost Hessian
using Mat12x4 = Eigen::Matrix<double, NX, NU>;    // Input Jacobian B
using Mat4x12 = Eigen::Matrix<double, NU, NX>;    // Feedback gain K

// Structure for a single quadrotor state. p, v, w are vectors and R lives on SO(3)
struct mpc_state {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();      // Position in the inertial NED frame         [m]
    Eigen::Vector3d v = Eigen::Vector3d::Zero();      // Velocity in the inertial NED frame         [m/s]
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // Rotation from the body to the inertial frame
    Eigen::Vector3d w = Eigen::Vector3d::Zero();      // Angular velocity in the body frame          [rad/s]
};

// Structure for all parameter members of the controller
struct controller_internal_parameters {
    double dt_mpc;                                    // Step length of the prediction model          [s]
    int max_iterations;                               // Max Gauss-Newton iterations per control tick
    int line_search_steps;                            // Max step halvings in the line search
    double regularization;                            // Tikhonov term added to Quu to keep it invertible
    Eigen::Matrix<double, NX, NX> Q;                  // Stage cost on the error state
    Eigen::Matrix<double, NX, NX> Qf;                 // Terminal cost on the error state
    Eigen::Matrix<double, NU, NU> R_in;               // Stage cost on the deviation from the reference motor thrusts
    Eigen::Matrix<double, 3, 3> Ki_tran;              // Integral gain that drives the disturbance estimate
    Eigen::Matrix<double, 3, 1> d_hat_max;            // Saturation on the disturbance estimate        [m/s^2]
    Eigen::Matrix<double, 3, 3> J;                    // Inertia matrix                               [kg m^2]
    Eigen::Matrix<double, 3, 3> J_inv;                // Inverse of the inertia matrix
    Eigen::Matrix<double, 4, 4> mixer_inv;            // Maps the motor thrusts to [thrust; mx; my; mz]
    Eigen::Matrix<double, NU, 1> f_min;               // Min allowed thrust per motor                 [N]
    Eigen::Matrix<double, NU, 1> f_max;               // Max allowed thrust per motor                 [N]
};

// Structure for all the members that are mapped to the rk4 vector AFTER integration
struct controller_integrated_state_members {
    Eigen::Matrix<double, 3, 1> e_tran_pos_I;         // Translational integral error
};

// Structure for all the internal members of the controller
struct controller_internal_members {
    double t;                                                      // Time
    Eigen::Quaterniond q;                                          // Quaternion
    Eigen::Matrix<double, 3, 3> Rji;                               // Rotation from the body to the inertial frame
    Eigen::Matrix<double, 3, 3> Rij;                               // Rotation from the inertial to the body frame
    Eigen::Matrix<double, 3, 1> x_tran_pos;                        // Translational position
    Eigen::Matrix<double, 3, 1> x_tran_vel;                        // Translational velocity
    Eigen::Matrix<double, 3, 1> eta_rot;                           // Euler Angles
    Eigen::Matrix<double, 3, 1> omega;                             // Angular velocities
    Eigen::Matrix<double, 3, 1> r_user;                            // Translational user position command
    Eigen::Matrix<double, 3, 1> r_dot_user;                        // Translational user veloctiy command
    Eigen::Matrix<double, 3, 1> r_ddot_user;                       // Translational user acceleration command
    double psi_user;                                               // User yaw command
    Eigen::Matrix<double, 3, 1> e_tran_pos;                        // Translational error in position
    Eigen::Matrix<double, 3, 1> e_tran_vel;                        // Translational error in velocity
    Eigen::Matrix<double, 3, 1> d_hat;                             // Disturbance acceleration estimate in I [m/s^2]
    mpc_state x0;                                                  // Measured state handed to the NMPC
    std::array<mpc_state, NH + 1> X_ref;                           // Reference state over the horizon
    std::array<Eigen::Matrix<double, NU, 1>, NH> U_ref;            // Reference motor thrusts over the horizon
    std::array<mpc_state, NH + 1> X;                               // Planned state over the horizon (warm start)
    std::array<Eigen::Matrix<double, NU, 1>, NH> U;                // Planned motor thrusts over the horizon
    std::array<mpc_state, NH + 1> X_new;                           // Candidate state during the line search
    std::array<Eigen::Matrix<double, NU, 1>, NH> U_new;            // Candidate motor thrusts during the line search
    std::array<Mat4x12, NH> K_fb;                                  // Feedback gain per step
    std::array<Eigen::Matrix<double, NU, 1>, NH> k_ff;             // Feed-forward correction per step
    double cost_initial;                                           // Cost of the warm start rollout
    double cost;                                                   // Cost after the optimization
    int iterations;                                                // Accepted Gauss-Newton iterations this tick
    double alpha;                                                  // Last accepted line search step
    double shift_timer;                                            // Time elapsed since the last warm start shift (unused)
    Eigen::Matrix<double, 4, 1> u;                                 // [thrust; mx; my; mz]
    Eigen::Matrix<double, 4, 1> Thrust;                            // Newtons
    Eigen::Matrix<double, 4, 1> Sat_Thrust;                        // Saturated thrust
    double alg_duration;                                           // Control execution duration
    std::chrono::high_resolution_clock::time_point alg_start_time; // Algorithm Start timepoint
    std::chrono::high_resolution_clock::time_point alg_end_time;   // Algorithm End timepoint
};


// =========================================================================================================
// SO(3) helpers
//   - skewSymmetric: 3-vector to skew matrix such that skewSymmetric(a) * b = a x b  (from sim-helpers)
//   - expSO3:        rotation vector (axis * angle) to rotation matrix (Rodrigues)   (from sim-helpers)
//   - logSO3:        rotation matrix to rotation vector (inverse of expSO3)          (from sim-helpers)
//   - box_minus:     12 element error between two states [p_a - p_b; v_a - v_b; Log(R_b^T R_a); w_a - w_b]
// =========================================================================================================
using ::_shared_::_transformations_::skewSymmetric;
using ::_shared_::_transformations_::expSO3;
using ::_shared_::_transformations_::logSO3;

// Error state of a relative to b. The rotation part is expressed in b's body frame, which matches the right
// perturbation R = R_bar Exp(dtheta) used by the linearization.
inline Vec12 box_minus(const mpc_state& a, const mpc_state& b)
{
    Vec12 d;
    d << a.p - b.p,
         a.v - b.v,
         logSO3(b.R.transpose() * a.R),
         a.w - b.w;
    return d;
}


// =========================================================================================================
// nmpc.hpp   -- QUADM Nonlinear Model Predictive Controller on SO(3)
//   - Implements a NMPC that tracks the user trajectory with the motor thrusts as inputs.
//   - Uses an integral of the position error as a constant disturbance estimate in the
//     prediction model for offset free tracking.
//   - Inherits base routines and actuator interface from controller_base.
//   - Inherits base routines from blackbox to setup the logging.
// =========================================================================================================
class nmpc : public ::_acsl_::_control_::controller_base, public ::_acsl_::_logger_::blackbox
{

public:
    // -------------------------------------------------------------------------
    // Constructor:
    //   - Accepts simlog reference for logging duties.
    //   - Passes simlog to controller_base for unified logging.
    // -------------------------------------------------------------------------
    nmpc(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory);

    // -------------------------------------------------------------------------
    // Destructor:
    //   - Allows cleanup in derived class.
    // -------------------------------------------------------------------------
    virtual ~nmpc() = default;

    // -------------------------------------------------------------------------
    // Override: update()
    //   - Update controller state (to be implemented in nmpc-quadm.cpp).
    //   - Provide logic in the source file.
    // -------------------------------------------------------------------------
    void update([[maybe_unused]] double time,
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
                [[maybe_unused]] double w_z) override;

    // -------------------------------------------------------------------------
    // Override: init()
    //   - Provides the initiation of the controller. This includes reading the
    //     parameter files.
    // -------------------------------------------------------------------------
    void init() override;

    // ------------------------------------------------------------------------
    // Blackbox functions for the physics logger.
    // ------------------------------------------------------------------------
    // -------------------------------------------
    // Function: InitiateLogging
    //   - Initiates the logging for controller.
    //   - Creates the log folder structure.
    //   - Copies the gains to the logs.
    // -------------------------------------------
    bool InitiateLogging() override;

    // -------------------------------------------
    // Function: ConfigureHeaders
    //   - Simple function that has an oss stream
    //     and writes the headers to the log file
    // -------------------------------------------
    void ConfigureHeaders() override;

    // -------------------------------------------
    // Function: LogData
    //   - Simple function that has an oss stream
    //     and writes the data corresponding to
    //     the headers to the log file.
    // ------------------------------------------
    void LogData() override;

    // -------------------------------------------------------------------------
    // Override: run()
    //   - Provides the entire logic of the controller.
    // -------------------------------------------------------------------------
    void run([[maybe_unused]] const double time_step_rk4_) override;

    // -------------------------------------------------------------------------
    // Override: read_params()
    //   - Reads in the paramters for the controller
    // -------------------------------------------------------------------------
    void read_params([[maybe_unused]] const std::string& jsonFile) override;

private:
    // -------------------------------------------------------------------------
    // NON API FUNCTIONS NECESSARY FOR ALL CONTROLLERS - MUST BE IMPLEMENTED
    // -------------------------------------------------------------------------
    // Define the state vector (y) and its derivative (dy) for the rk4 integrator
    _control_::rk4_array<double, NSI> y;
    _control_::rk4_array<double, NSI> dy;

    // Define the model (dy = position error, so y holds its integral)
    void model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t);

    // Create a RungeKutta object for integration functionality.
    boost::numeric::odeint::runge_kutta4<_control_::rk4_array<double, NSI>> rk4;

    // Define the internal parameter members of the controller
    ::_acsl_::_quadm_::_nmpc_::controller_internal_parameters cip;

    // Define the internal members of the controller
    ::_acsl_::_quadm_::_nmpc_::controller_internal_members cim;

    // Define the internal integrated state members of the controller
    ::_acsl_::_quadm_::_nmpc_::controller_integrated_state_members csm;

private:
    // -------------------------------------------------------------------------
    // NON API CONTROLLER SPECIFIC FUNCTIONS - WILL BE DIFF FOR DIFF CONTROLLERS
    // -------------------------------------------------------------------------
    // Function to compute the disturbance estimate from the integral of the position error
    void compute_disturbance_estimate();

    // Function to sample the user trajectory over the horizon and build the reference states and inputs
    void compute_reference_horizon();

    // Function to propagate the nonlinear prediction model by one step
    mpc_state predict_step(const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f) const;

    // Function to linearize the prediction model about (x, f) in the error state
    void linearize(const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f, Mat12& A, Mat12x4& B) const;

    // Function to compute the stage cost at step k
    double stage_cost(const int k, const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f) const;

    // Function to compute the terminal cost
    double terminal_cost(const mpc_state& x) const;

    // Function to simulate the warm start inputs from the measured state and return its cost
    double rollout();

    // Function to compute the feedback and feed-forward gains with a Riccati backward pass
    void backward_pass();

    // Function to simulate the nonlinear model with the new gains and return the candidate cost
    double forward_pass(const double alpha);

    // Function to solve the optimal control problem (rollout + Gauss-Newton iterations)
    void solve_ocp();

    // Function to shift the warm start every time a full dt_mpc has elapsed
    void shift_warm_start(const double time_step);

    // Function to compute the normalized thrust
    void compute_normalized_thrusts();

    // Assign the values from rk4 to controller internal members
    void assign_from_rk4();

};

} // namespace _nmpc_

} // namespace _quadm_

} // namespace _acsl_

#endif  // NMPC_QUADM_HPP_
