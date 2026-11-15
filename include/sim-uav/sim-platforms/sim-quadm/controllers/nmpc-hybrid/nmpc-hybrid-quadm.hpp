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
 * File:        nmpc-hybrid-quadm.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 9, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Header file for the interconnected nonlinear MPC and hybrid MRAC controller for the QUADM.
 *              - Outer loop: a translational NMPC (input = inertial force mu) is the baseline of a hybrid MRAC on
 *                [p; v]. The NMPC plan also generates the command of the MRAC reference model.
 *              - Inner loop: an attitude NMPC on SO(3) (input = torque) is the baseline of an MRAC on omega.
 *              - The hybrid resets are applied to the outer loop reference model only.
 *              For reference: G. M. Kumar, X. E. Casanova Pabon, A. L'Afflitto, "Interconnected Nonlinear MPC and
 *              Hybrid MRAC with Applications to Multi-Rotor UAVs".
 *
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef NMPC_HYBRID_QUADM_HPP_
#define NMPC_HYBRID_QUADM_HPP_

#include "sim-control-base.hpp"      // Include for the base class of a controller defined in the simualtor
#include "quadm-parameter-file.hpp"  // Include for the hardcoded quadm parameters that are common for all controllers

#include <array>

namespace _acsl_
{

namespace _quadm_
{

namespace _nmpc_hybrid_
{

// Define the number of states in the boost array for integration
//   mu filters (6) + x_tran_ref (6) + K_hat_x_tran (18) + K_hat_r_tran (9) + Theta_hat_tran (21)
//   + omega_ref (3) + K_hat_x_rot (9) + K_hat_r_rot (9) + Theta_hat_rot (21) + e_tran_pos_I (3)
constexpr int NSI = 105;

// Define the number of steps in the prediction horizons (horizon length [s] = NH * dt)
constexpr int NH_O = 30;    // Outer (translational) NMPC
constexpr int NH_I = 20;    // Inner (attitude) NMPC

// Define the size of the error state and the input of both NMPCs
//   - Outer: error state [dp, dv], input mu (force in the inertial frame)
//   - Inner: error state [dtheta, domega] with R = R_bar Exp(dtheta), input tau (torque in the body frame)
constexpr int NX = 6;
constexpr int NU = 3;

// Define the size of the regressors (paper Eqs. (67) and (74)) and of the augmented regressors (Eq. (29))
constexpr int NPHI_TRAN = 4;                    // [1; ||v - w_hat|| (v - w_hat)]
constexpr int NPHI_ROT = 4;                     // [wx wy, wx wz, wy wz, wz^2 - wx^2]
constexpr int NTHETA_TRAN = NPHI_TRAN + 3;      // [Phi_o; mu_baseline]
constexpr int NTHETA_ROT = NPHI_ROT + 3;        // [Phi_in; tau_baseline]

// Eigen aliases used by the optimal control problems
using Vec6   = Eigen::Matrix<double, NX, 1>;    // Error state / cost gradient
using Mat6   = Eigen::Matrix<double, NX, NX>;   // State Jacobian A / cost Hessian
using Mat6x3 = Eigen::Matrix<double, NX, NU>;   // Input Jacobian B
using Mat3x6 = Eigen::Matrix<double, NU, NX>;   // Feedback gain K

// -------------------------------------------------------------------------------------------------------------------- //
// NMPC STATES AND SO(3) HELPERS
// -------------------------------------------------------------------------------------------------------------------- //

// Structure for the outer (translational) NMPC state
struct outer_state {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();      // Position in the inertial NED frame         [m]
    Eigen::Vector3d v = Eigen::Vector3d::Zero();      // Velocity in the inertial NED frame         [m/s]
};

// Structure for the inner (attitude) NMPC state. R lives on SO(3)
struct inner_state {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // Rotation from the body to the inertial frame
    Eigen::Vector3d w = Eigen::Vector3d::Zero();      // Angular velocity in the body frame          [rad/s]
};

// =========================================================================================================
// SO(3) helpers
//   - skewSymmetric: 3-vector to skew matrix such that skewSymmetric(a) * b = a x b  (from sim-helpers)
//   - veeMap:        skew matrix to 3-vector (inverse of skewSymmetric)              (from sim-helpers)
//   - expSO3:        rotation vector (axis * angle) to rotation matrix (Rodrigues)   (from sim-helpers)
//   - logSO3:        rotation matrix to rotation vector (inverse of expSO3)          (from sim-helpers)
//   - box_minus_outer: 6 element error between two outer states [p_a - p_b; v_a - v_b]
//   - box_minus_inner: 6 element error between two inner states [Log(R_b^T R_a); w_a - w_b]
// =========================================================================================================
using ::_shared_::_transformations_::skewSymmetric;
using ::_shared_::_transformations_::veeMap;
using ::_shared_::_transformations_::expSO3;
using ::_shared_::_transformations_::logSO3;

// Error state of a relative to b for the outer NMPC
inline Vec6 box_minus_outer(const outer_state& a, const outer_state& b)
{
    Vec6 d;
    d << a.p - b.p,
         a.v - b.v;
    return d;
}

// Error state of a relative to b for the inner NMPC. The rotation part is expressed in b's body frame, which matches
// the right perturbation R = R_bar Exp(dtheta) used by the linearization.
inline Vec6 box_minus_inner(const inner_state& a, const inner_state& b)
{
    Vec6 d;
    d << logSO3(b.R.transpose() * a.R),
         a.w - b.w;
    return d;
}

// -------------------------------------------------------------------------------------------------------------------- //
// CONTROLLER STRUCTURES
// -------------------------------------------------------------------------------------------------------------------- //

// Structure for all parameter members of the controller
struct controller_internal_parameters {
    // Outer (translational) NMPC
    double dt_outer;                                        // Step length (= sampling period) of the outer NMPC  [s]
    int max_iterations_outer;                               // Max Gauss-Newton iterations per outer solve
    int line_search_steps_outer;                            // Max step halvings in the outer line search
    double regularization_outer;                            // Tikhonov term added to Quu (outer)
    Mat6 Q_outer;                                           // Outer stage cost on the error state
    Mat6 Qf_outer;                                          // Outer terminal cost on the error state
    Eigen::Matrix<double, NU, NU> R_outer;                  // Outer stage cost on the deviation from the reference force
    double thrust_min;                                      // Min allowed total thrust in the outer NMPC            [N]
    double thrust_max;                                      // Max allowed total thrust in the outer NMPC            [N]
    double cos_tilt_max;                                    // Cosine of the max allowed tilt in the outer NMPC
    bool terminal_cost_from_lqr_outer;                      // Use the hover LQR (Riccati) solution as Qf_outer

    // Inner (attitude) NMPC
    double dt_inner;                                        // Step length (= sampling period) of the inner NMPC  [s]
    int max_iterations_inner;                               // Max Gauss-Newton iterations per inner solve
    int line_search_steps_inner;                            // Max step halvings in the inner line search
    double regularization_inner;                            // Tikhonov term added to Quu (inner)
    Mat6 Q_inner;                                           // Inner stage cost on [eta; e_omega]
    Mat6 Qf_inner;                                          // Inner terminal cost on [eta; e_omega]
    Eigen::Matrix<double, NU, NU> R_inner;                  // Inner stage cost on the deviation from the feedforward torque
    bool terminal_cost_from_lqr_inner;                      // Use the hover LQR (Riccati) solution as Qf_inner

    // Disturbance estimate injected in the outer NMPC model (not in the paper, switched off by default)
    bool use_nmpc_disturbance_estimate;                     // Switch for the disturbance estimate
    Eigen::Matrix<double, 3, 3> Ki_disturbance;             // Integral gain that drives the disturbance estimate
    Eigen::Matrix<double, 3, 1> d_hat_max;                  // Saturation on the disturbance estimate               [m/s^2]

    // Outer (translational) MRAC - paper Eqs. (66)-(72)
    Eigen::Matrix<double, 3, 3> Kp_refmod_tran;             // Proportional gains of the translational reference model
    Eigen::Matrix<double, 3, 3> Kv_refmod_tran;             // Velocity gains of the translational reference model
    Eigen::Matrix<double, 3, 3> Kp_refmod_tran_inv;         // Inverse of Kp_refmod_tran (reference command Eq. (71))
    Eigen::Matrix<double, 6, 6> Gamma_x_tran;               // Adaptive Gains for the translational control
    Eigen::Matrix<double, 3, 3> Gamma_r_tran;               // Adaptive Gains for the translational control
    Eigen::Matrix<double, NTHETA_TRAN, NTHETA_TRAN> Gamma_Theta_tran;   // Adaptive Gains for the translational control
    Eigen::Matrix<double, 6, 6> Q_tran;                     // Lyapunov weighting matrix
    Eigen::Matrix<double, 6, 6> P_tran;                     // Solution matrix to continuous Lyapunov equation
    Eigen::Matrix<double, 6, 6> A_tran;                     // Translational system matrix
    Eigen::Matrix<double, 6, 3> B_tran;                     // Translational input matrix
    Eigen::Matrix<double, 6, 6> A_ref_tran;                 // Reference model system matrix
    Eigen::Matrix<double, 6, 3> B_ref_tran;                 // Reference model input matrix
    Eigen::Matrix<double, 3, 1> wind_estimate;              // Wind estimate used in the drag regressor              [m/s]

    // Inner (rotational) MRAC - paper Eqs. (73)-(80)
    Eigen::Matrix<double, 3, 3> K_omega_ref;                // Gains of the angular velocity reference model
    double k_R;                                             // Attitude error gain in the angular velocity command
    Eigen::Matrix<double, 3, 3> Gamma_x_rot;                // Adaptive gain for state feedback parameters
    Eigen::Matrix<double, 3, 3> Gamma_r_rot;                // Adaptive gain for command tracking parameters
    Eigen::Matrix<double, NTHETA_ROT, NTHETA_ROT> Gamma_Theta_rot;  // Adaptive gain for dynamic regression parameters
    Eigen::Matrix<double, 3, 3> Q_rot;                      // Lyapunov weighting matrix (rotation)
    Eigen::Matrix<double, 3, 3> P_rot;                      // Lyapunov solution matrix (rotation)
    Eigen::Matrix<double, 3, 3> B_rot;                      // Rotational input matrix
    Eigen::Matrix<double, 3, 3> A_ref_rot;                  // Rotational reference model matrix
    Eigen::Matrix<double, 3, 3> B_ref_rot;                  // Rotational reference input matrix

    // Command generator (filter on mu to get mu_dot)
    Eigen::Matrix<double, 2, 2> A_filter_mu;                // Differentiator A matrix for mu
    Eigen::Matrix<double, 2, 1> B_filter_mu;                // Differentiator B matrix for mu
    Eigen::Matrix<double, 1, 2> C_filter_mu;                // Differentiator C matrix for mu (output = filtered mu_dot)
    Eigen::Matrix<double, 1, 2> C_filter_mu_lowpass;        // Low pass C matrix for mu (output = filtered mu)

    // Hybrid (outer loop only)
    bool use_hybrid;                                        // Switch for the hybrid reference model resets
    double hybrid_error_norm_threshold;                     // Position error norm that triggers a reset            [m]
    double hybrid_rho;                                      // Contraction of the tracking error at a reset         [-]
    double hybrid_dwell_time;                               // Min time between two resets                         [s]

    // Robustification (the paper uses projection only, so dead-zone and e-mod are switched off by default)
    bool use_dead_zone_emod;                                // Switch for the dead-zone and e-modification
    double dead_zone_delta_translational;                   // Translational deadzone delta radius
    double dead_zone_e0_translational;                      // Translational deadzone error tolerance
    double sigma_x_translational;                           // Translational E-mod gain for x (states)
    double sigma_r_translational;                           // Translational E-mod gain for r (commands)
    double sigma_Theta_translational;                       // Translational E-mod gain for Theta (parameters)
    double projection_x_max_x_translational;                // Translational Projection limit for Kx_hat
    double projection_epsilon_x_translational;              // Translational Projection tolerance for Kx_hat
    double projection_x_max_r_translational;                // Translational Projection limit for Kr_hat
    double projection_epsilon_r_translational;              // Translational Projection tolerance for Kr_hat
    double projection_x_max_Theta_translational;            // Translational Projection limit for Theta_hat
    double projection_epsilon_Theta_translational;          // Translational Projection tolerance for Theta_hat
    double dead_zone_delta_rotational;                      // Rotational deadzone delta radius
    double dead_zone_e0_rotational;                         // Rotational deadzone error tolerance
    double sigma_x_rotational;                              // Rotational E-mod gain for x (states)
    double sigma_r_rotational;                              // Rotational E-mod gain for r (commands)
    double sigma_Theta_rotational;                          // Rotational E-mod gain for Theta (parameters)
    double projection_x_max_x_rotational;                   // Rotational Projection limit for Kx_hat
    double projection_epsilon_x_rotational;                 // Rotational Projection tolerance for Kx_hat
    double projection_x_max_r_rotational;                   // Rotational Projection limit for Kr_hat
    double projection_epsilon_r_rotational;                 // Rotational Projection tolerance for Kr_hat
    double projection_x_max_Theta_rotational;               // Rotational Projection limit for Theta_hat
    double projection_epsilon_Theta_rotational;             // Rotational Projection tolerance for Theta_hat

    // Physical parameters
    Eigen::Matrix<double, 3, 3> J;                          // Inertia matrix                                       [kg m^2]
    Eigen::Matrix<double, 3, 3> J_inv;                      // Inverse of the inertia matrix
    Eigen::Matrix<double, 4, 4> mixer_inv;                  // Maps the motor thrusts to [thrust; mx; my; mz]
};

// Structure for all the members that are mapped to the rk4 vector AFTER integration
struct controller_integrated_state_members {
    Eigen::Matrix<double, 2, 1> state_mu_x_filter;                          // States for filter
    Eigen::Matrix<double, 2, 1> state_mu_y_filter;                          // States for filter
    Eigen::Matrix<double, 2, 1> state_mu_z_filter;                          // States for filter
    Eigen::Matrix<double, 6, 1> x_tran_ref;                                 // Reference model in I
    Eigen::Matrix<double, 6, 3> K_hat_x_tran;                               // Translational Adaptive gains for x
    Eigen::Matrix<double, 3, 3> K_hat_r_tran;                               // Translational Adaptive gains for r
    Eigen::Matrix<double, NTHETA_TRAN, 3> Theta_hat_tran;                   // Translational Adaptive gains for Theta
    Eigen::Matrix<double, 3, 1> omega_ref;                                  // Angular Velocity Reference model
    Eigen::Matrix<double, 3, 3> K_hat_x_rot;                                // Rotational Adaptive gains for x
    Eigen::Matrix<double, 3, 3> K_hat_r_rot;                                // Rotational Adaptive gains for r
    Eigen::Matrix<double, NTHETA_ROT, 3> Theta_hat_rot;                     // Rotational Adaptive gains for Theta
    Eigen::Matrix<double, 3, 1> e_tran_pos_I;                               // Integral of (p - p_ref) for d_hat
};

// Structure for all the members of the outer (translational) NMPC
struct outer_nmpc_members {
    outer_state x0;                                                         // Measured state handed to the outer NMPC
    std::array<outer_state, NH_O + 1> X_ref;                                // Reference state over the horizon
    std::array<Eigen::Vector3d, NH_O> U_ref;                                // Reference force over the horizon
    std::array<outer_state, NH_O + 1> X;                                    // Planned state over the horizon (warm start)
    std::array<Eigen::Vector3d, NH_O> U;                                    // Planned force over the horizon (warm start)
    std::array<Eigen::Vector3d, NH_O> U_plan;                               // Accepted plan of the last solve (not shifted)
    std::array<outer_state, NH_O + 1> X_new;                                // Candidate state during the line search
    std::array<Eigen::Vector3d, NH_O> U_new;                                // Candidate force during the line search
    std::array<Mat3x6, NH_O> K_fb;                                          // Feedback gain per step
    std::array<Eigen::Vector3d, NH_O> k_ff;                                 // Feed-forward correction per step
    double cost_initial;                                                    // Cost of the warm start rollout
    double cost;                                                            // Cost after the optimization
    int iterations;                                                         // Accepted Gauss-Newton iterations
    double alpha;                                                           // Last accepted line search step
    bool first_solve;                                                       // True until the first solve
    bool solved_this_tick;                                                  // True on the ticks with a new solve
    double t_solve;                                                         // Time of the last solve s_k            [s]
    Eigen::Vector3d p_solve;                                                // Position at the last solve           [m]
    Eigen::Vector3d v_solve;                                                // Velocity at the last solve           [m/s]
    Eigen::Vector3d a_solve;                                                // Nominal acceleration of the held force [m/s^2]
    Eigen::Vector3d mu_hold;                                                // Held first input of the plan (beta)  [N]
};

// Structure for all the members of the inner (attitude) NMPC
struct inner_nmpc_members {
    inner_state x0;                                                         // Measured state handed to the inner NMPC
    std::array<inner_state, NH_I + 1> X_ref;                                // Preview [R_d; Omega_d] over the horizon
    std::array<Eigen::Vector3d, NH_I> Omega_d_dot;                          // Preview of the desired angular acceleration
    std::array<inner_state, NH_I + 1> X;                                    // Planned state over the horizon (warm start)
    std::array<Eigen::Vector3d, NH_I> U;                                    // Planned torque over the horizon (warm start)
    std::array<inner_state, NH_I + 1> X_new;                                // Candidate state during the line search
    std::array<Eigen::Vector3d, NH_I> U_new;                                // Candidate torque during the line search
    std::array<Mat3x6, NH_I> K_fb;                                          // Feedback gain per step
    std::array<Eigen::Vector3d, NH_I> k_ff;                                 // Feed-forward correction per step
    double cost_initial;                                                    // Cost of the warm start rollout
    double cost;                                                            // Cost after the optimization
    int iterations;                                                         // Accepted Gauss-Newton iterations
    double alpha;                                                           // Last accepted line search step
    bool first_solve;                                                       // True until the first solve
    bool solved_this_tick;                                                  // True on the ticks with a new solve
    double t_solve;                                                         // Time of the last solve               [s]
    double thrust_preview;                                                  // Total thrust used for the torque limits [N]
    Eigen::Vector3d tau_hold;                                               // Held first input of the plan (beta)  [N m]
};

// Structure for all the internal members of the controller
struct controller_internal_members {
    double t;                                                               // Time
    bool first_run;                                                         // True until the first update
    Eigen::Quaterniond q;                                                   // Quaternion
    Eigen::Matrix<double, 3, 3> Rji;                                        // Rotation from the body to the inertial frame
    Eigen::Matrix<double, 3, 3> Rij;                                        // Rotation from the inertial to the body frame
    Eigen::Matrix<double, 3, 1> x_tran_pos;                                 // Translational position
    Eigen::Matrix<double, 3, 1> x_tran_vel;                                 // Translational velocity
    Eigen::Matrix<double, 6, 1> x_tran;                                     // Translational states [position; velocity]
    Eigen::Matrix<double, 3, 1> eta_rot;                                    // Euler Angles
    Eigen::Matrix<double, 3, 1> omega;                                      // Angular velocities
    Eigen::Matrix<double, 3, 1> r_user;                                     // Translational user position command
    Eigen::Matrix<double, 3, 1> r_dot_user;                                 // Translational user veloctiy command
    Eigen::Matrix<double, 3, 1> r_ddot_user;                                // Translational user acceleration command
    double psi_user;                                                        // User yaw command
    double psi_dot_user;                                                    // User yaw rate command
    Eigen::Matrix<double, 3, 1> d_hat;                                      // Disturbance acceleration estimate in I [m/s^2]
    Eigen::Matrix<double, 3, 1> e_tran_pos_integrand;                       // Integrand of e_tran_pos_I (zero if d_hat is off)

    // Outer (translational) MRAC
    Eigen::Matrix<double, 3, 1> r_cmd_tran;                                 // Translational reference command (Eq. (52))
    Eigen::Matrix<double, 6, 1> x_tran_ref_dot;                             // Translational reference model
    Eigen::Matrix<double, 6, 1> e_tran;                                     // Translational error in states [e_pos; e_vel]
    Eigen::Matrix<double, 3, 1> e_tran_pos;                                 // Translational error in position
    Eigen::Matrix<double, 3, 1> e_tran_vel;                                 // Translational error in velocity
    Eigen::Matrix<double, 6, 3> K_hat_x_tran_dot;                           // Adaptive gain to be integrated
    Eigen::Matrix<double, 3, 3> K_hat_r_tran_dot;                           // Adaptive gain to be integrated
    Eigen::Matrix<double, NTHETA_TRAN, 3> Theta_hat_tran_dot;               // Adaptive gain to be integrated
    Eigen::Matrix<double, NPHI_TRAN, 1> outer_loop_regressor;               // Outer loop regressor
    Eigen::Matrix<double, NTHETA_TRAN, 1> augmented_outer_loop_regressor;   // Outer loop augmented regressor
    double dead_zone_value_translational;                                   // Dead zone val - OL
    bool proj_op_activated_K_hat_x_translational;                           // Projection activation boolean - OL - K_hat_x
    bool proj_op_activated_K_hat_r_translational;                           // Projection activation boolean - OL - K_hat_r
    bool proj_op_activated_Theta_hat_translational;                         // Projection activation boolean - OL - Theta_hat
    Eigen::Matrix<double, 3, 1> mu_tran_baseline;                           // Baseline control input (outer NMPC)
    Eigen::Matrix<double, 3, 1> mu_tran_adaptive;                           // Adaptive control input
    Eigen::Matrix<double, 3, 1> mu_tran_I;                                  // Virtual control action in the inertial frame
    Eigen::Matrix<double, 3, 1> mu_tran_I_dot;                              // Rate of the filtered virtual control action
    Eigen::Matrix<double, 3, 1> mu_tran_I_filtered;                         // Filtered virtual control action (command generator)

    // Hybrid (outer loop only)
    bool hybrid_reset_triggered;                                            // True on the ticks with a reset
    int number_of_hybrid_resets;                                            // Number of resets since the start
    double time_of_last_hybrid_reset;                                       // Time of the last reset                [s]

    // Command generator
    Eigen::Matrix<double, 2, 1> internal_state_mu_x_filter;                 // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_mu_y_filter;                 // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_mu_z_filter;                 // Internal States for filter
    Eigen::Matrix<double, 3, 1> c1;                                         // Desired "heading" vector in the inertial frame
    Eigen::Matrix<double, 3, 1> b3d;                                        // Desired body z axis
    Eigen::Matrix<double, 3, 1> b2d;                                        // Desired body y axis
    Eigen::Matrix<double, 3, 1> b1d;                                        // Desired body x axis
    Eigen::Matrix<double, 3, 3> R_d;                                        // Desired rotation matrix
    Eigen::Matrix<double, 3, 1> c1_dot;                                     // Desired "heading rate" vector in the inertial frame
    Eigen::Matrix<double, 3, 1> n_hat;                                      // Intermediate variable to compute b2d_dot
    Eigen::Matrix<double, 3, 1> n_hat_dot;                                  // Intermediate variable to compute b2d_dot
    double sigma;                                                           // Intermediate variable to compute b2d_dot
    Eigen::Matrix<double, 3, 1> b3d_dot;                                    // Differential of Desired body z axis
    Eigen::Matrix<double, 3, 1> b2d_dot;                                    // Differential of Desired body y axis
    Eigen::Matrix<double, 3, 1> b1d_dot;                                    // Differential of Desired body x axis
    Eigen::Matrix<double, 3, 3> R_d_dot;                                    // Differential of Desired rotation matrix
    Eigen::Matrix<double, 3, 1> Omega_d;                                    // Desired angular velocity in the desired frame (Eq. (63))
    Eigen::Matrix<double, 3, 1> omega_d;                                    // Desired angular velocity in the body frame R^T R_d Omega_d

    // Inner (rotational) MRAC
    Eigen::Matrix<double, 3, 1> e_R;                                        // Attitude error (Eq. (78))
    Eigen::Matrix<double, 3, 1> omega_c;                                    // Attitude corrected angular velocity command (Eq. (78))
    Eigen::Matrix<double, 3, 1> r_cmd_rot;                                  // Rotational reference command K_omega omega_c (Eq. (79))
    Eigen::Matrix<double, 3, 1> omega_ref_dot;                              // Angular velocity reference model
    Eigen::Matrix<double, 3, 1> omega_e;                                    // Error in the angular velocities
    Eigen::Matrix<double, NPHI_ROT, 1> inner_loop_regressor;                // Inner loop regressor vector
    Eigen::Matrix<double, NTHETA_ROT, 1> augmented_inner_loop_regressor;    // Inner loop augmented regressor vector
    Eigen::Matrix<double, 3, 3> K_hat_x_rot_dot;                            // Adaptive gain to be integrated
    Eigen::Matrix<double, 3, 3> K_hat_r_rot_dot;                            // Adaptive gain to be integrated
    Eigen::Matrix<double, NTHETA_ROT, 3> Theta_hat_rot_dot;                 // Adaptive gain to be integrated
    double dead_zone_value_rotational;                                      // Dead zone val - IL
    bool proj_op_activated_K_hat_x_rotational;                              // Projection activation boolean - IL - K_hat_x
    bool proj_op_activated_K_hat_r_rotational;                              // Projection activation boolean - IL - K_hat_r
    bool proj_op_activated_Theta_hat_rotational;                            // Projection activation boolean - IL - Theta_hat
    Eigen::Matrix<double, 3, 1> tau_rot_baseline;                           // Baseline rotational control input (inner NMPC)
    Eigen::Matrix<double, 3, 1> tau_rot_adaptive;                           // Adaptive rotational control input
    Eigen::Matrix<double, 3, 1> tau_rot;                                    // Rotational Control action

    // Offsets of the members in the rk4 vector (recorded in assign_from_rk4, never hard coded)
    int rk4_index_state_mu_x_filter;                                        // Offset of the first mu filter state
    int rk4_index_x_tran_ref;                                               // Offset of the translational reference model
    int rk4_index_omega_ref;                                                // Offset of the angular velocity reference model

    // Actuation
    Eigen::Matrix<double, 4, 1> u;                                          // [thrust; mx; my; mz]
    Eigen::Matrix<double, 4, 1> Thrust;                                     // Newtons
    Eigen::Matrix<double, 4, 1> Sat_Thrust;                                 // Saturated thrust
    double alg_duration;                                                    // Control execution duration
    std::chrono::high_resolution_clock::time_point alg_start_time;          // Algorithm Start timepoint
    std::chrono::high_resolution_clock::time_point alg_end_time;            // Algorithm End timepoint
};


// =========================================================================================================
// nmpc_hybrid.hpp   -- QUADM interconnected NMPC and hybrid MRAC
//   - Outer loop: translational NMPC baseline + hybrid MRAC (reference resets on the position error norm).
//   - Inner loop: attitude NMPC baseline on SO(3) + MRAC on the angular velocity.
//   - Inherits base routines and actuator interface from controller_base.
//   - Inherits base routines from blackbox to setup the logging.
// =========================================================================================================
class nmpc_hybrid : public ::_acsl_::_control_::controller_base, public ::_acsl_::_logger_::blackbox
{

public:
    // -------------------------------------------------------------------------
    // Constructor:
    //   - Accepts simlog reference for logging duties.
    //   - Passes simlog to controller_base for unified logging.
    // -------------------------------------------------------------------------
    nmpc_hybrid(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory);

    // -------------------------------------------------------------------------
    // Destructor:
    //   - Allows cleanup in derived class.
    // -------------------------------------------------------------------------
    virtual ~nmpc_hybrid() = default;

    // -------------------------------------------------------------------------
    // Override: update()
    //   - Update controller state (to be implemented in nmpc-hybrid-quadm.cpp).
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

    // Define the model
    void model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t);

    // Create a RungeKutta object for integration functionality.
    boost::numeric::odeint::runge_kutta4<_control_::rk4_array<double, NSI>> rk4;

    // Define the internal parameter members of the controller
    ::_acsl_::_quadm_::_nmpc_hybrid_::controller_internal_parameters cip;

    // Define the internal members of the controller
    ::_acsl_::_quadm_::_nmpc_hybrid_::controller_internal_members cim;

    // Define the internal integrated state members of the controller
    ::_acsl_::_quadm_::_nmpc_hybrid_::controller_integrated_state_members csm;

    // Define the members of the outer (translational) NMPC
    ::_acsl_::_quadm_::_nmpc_hybrid_::outer_nmpc_members onm;

    // Define the members of the inner (attitude) NMPC
    ::_acsl_::_quadm_::_nmpc_hybrid_::inner_nmpc_members inm;

private:
    // -------------------------------------------------------------------------
    // NON API CONTROLLER SPECIFIC FUNCTIONS - WILL BE DIFF FOR DIFF CONTROLLERS
    // -------------------------------------------------------------------------
    // Function to initialize the reference models and the filters at the measured state (first tick only)
    void initialize_from_measured_state();

    // Function to reset the outer reference model with the hybrid algorithm (error norm trigger)
    void compute_hybrid_reference_reset();

    // Function to compute the disturbance estimate injected in the outer NMPC model
    void compute_disturbance_estimate();

    // ---------------------------------- Outer (translational) NMPC ---------------------------------- //
    // Function to sample, solve and hold the outer NMPC and to compute the reference command from its plan
    void compute_outer_nmpc();

    // Function to sample the user trajectory over the outer horizon
    void compute_outer_reference_horizon();

    // Function to propagate the outer nominal model by one step
    outer_state outer_predict_step(const outer_state& x, const Eigen::Vector3d& mu) const;

    // Function to linearize the outer nominal model (linear, so independent of the state)
    void outer_linearize(Mat6& A, Mat6x3& B) const;

    // Function to project the force onto the thrust magnitude and tilt limits
    Eigen::Vector3d outer_project_force(const Eigen::Vector3d& mu) const;

    // Function to compute the outer stage cost at step k
    double outer_stage_cost(const int k, const outer_state& x, const Eigen::Vector3d& mu) const;

    // Function to compute the outer terminal cost
    double outer_terminal_cost(const outer_state& x) const;

    // Function to simulate the outer warm start from the measured state and return its cost
    double outer_rollout();

    // Function to compute the outer feedback and feed-forward gains with a Riccati backward pass
    void outer_backward_pass();

    // Function to simulate the outer nominal model with the new gains and return the candidate cost
    double outer_forward_pass(const double alpha);

    // Function to solve the outer optimal control problem
    void outer_solve_ocp();

    // Function to shift the outer warm start by one step after a solve
    void outer_shift_warm_start();

    // ---------------------------------- Outer (translational) MRAC ---------------------------------- //
    // Function to compute the translational control in the inertial frame (hybrid MRAC on top of the outer NMPC)
    void compute_translational_control_in_I();

    // ---------------------------------------- Command generator ---------------------------------------- //
    // Function to observe the rate of the outerloop control input
    void compute_translational_control_rate();

    // Function to compute the thrust (u1), the desired rotation matrix and the desired angular velocity
    void compute_u1_R_d();

    // Function to map a force and a heading to a desired rotation matrix (Eqs. (60)-(62))
    Eigen::Matrix3d compute_R_d_from_force(const Eigen::Vector3d& mu, const double psi) const;

    // Function to propagate one axis of the mu filter (command generator model) by dt with a held input (RK4)
    Eigen::Matrix<double, 2, 1> propagate_mu_filter(const Eigen::Matrix<double, 2, 1>& state,
                                                    const double u,
                                                    const double dt) const;

    // ----------------------------------- Inner (attitude) NMPC ----------------------------------- //
    // Function to sample, solve and hold the inner NMPC
    void compute_inner_nmpc();

    // Function to build the inner preview [R_d; Omega_d] from the outer plan and the held adaptive force
    void compute_inner_reference_horizon();

    // Function to propagate the inner nominal model by one step (Eqs. (82)-(83))
    inner_state inner_predict_step(const inner_state& x, const Eigen::Vector3d& tau) const;

    // Function to linearize the inner nominal model about (x, tau) in the error state
    void inner_linearize(const inner_state& x, Mat6& A, Mat6x3& B) const;

    // Function to compute the inner residual [eta; e_omega] at step k and its Jacobian E (Eq. (84))
    Vec6 inner_residual(const int k, const inner_state& x, Mat6& E) const;

    // Function to compute the inner feedforward torque at step k (Eq. (86))
    Eigen::Vector3d inner_feedforward_torque(const int k, const inner_state& x) const;

    // Function to scale the torque so that every motor stays within its thrust limits
    Eigen::Vector3d inner_clamp_torque(const Eigen::Vector3d& tau) const;

    // Function to compute the inner stage cost at step k (Eq. (85))
    double inner_stage_cost(const int k, const inner_state& x, const Eigen::Vector3d& tau) const;

    // Function to compute the inner terminal cost
    double inner_terminal_cost(const inner_state& x) const;

    // Function to simulate the inner warm start from the measured state and return its cost
    double inner_rollout();

    // Function to compute the inner feedback and feed-forward gains with a Riccati backward pass
    void inner_backward_pass();

    // Function to simulate the inner nominal model with the new gains and return the candidate cost
    double inner_forward_pass(const double alpha);

    // Function to solve the inner optimal control problem
    void inner_solve_ocp();

    // Function to shift the inner warm start by one step after a solve
    void inner_shift_warm_start();

    // Function to compute the infinite horizon LQR cost-to-go (discrete Riccati) used as the NMPC terminal cost
    Mat6 compute_lqr_terminal_cost(const Mat6& A, const Mat6x3& B, const Mat6& Q, const Eigen::Matrix3d& R) const;

    // ----------------------------------- Inner (rotational) MRAC ----------------------------------- //
    // Function to compute the rotational control input (MRAC on omega on top of the inner NMPC)
    void compute_rotational_control();

    // Function to compute the normalized thrust
    void compute_normalized_thrusts();

    // Assign the values from rk4 to controller internal members
    void assign_from_rk4();

};

}   // namespace _nmpc_hybrid_

}   // namespace _quadm_

}   // namespace _acsl_

#endif // NMPC_HYBRID_QUADM_HPP_
