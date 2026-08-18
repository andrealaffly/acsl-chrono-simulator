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
 * File:        mrac-geometric-tailsitter.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 23, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for mrac geometric controller for the tailsitter.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef MRAC_GEOMETRIC_TAILSITTER_HPP_
#define MRAC_GEOMETRIC_TAILSITTER_HPP_

#include "sim-control-base.hpp"     // Include for the base class of a controller defined in the simualtor 
#include "tailsitter-parameter-file.hpp"  // Include for the hardcoded tailsitter parameters that are common for all controllers

namespace _acsl_
{

namespace _tailsitter_
{

namespace _mrac_geometric_
{

// Define the number of states in the boost array for integration
constexpr int NSI = 243;

// -------------------------------------------------------------------------------------------------------------------- //
// CONTROLLER STRUCTURES
// -------------------------------------------------------------------------------------------------------------------- //

// Structure for all parameter members of the controller
struct controller_internal_parameters {
    Eigen::Matrix<double, 3, 3> Kp_refmod_tran;     // Proportional Gains for the translational reference model
    Eigen::Matrix<double, 3, 3> Kd_refmod_tran;     // Derivative Gains for the translational reference model
    Eigen::Matrix<double, 3, 3> Ki_refmod_tran;     // Integral Gains for the translational reference command
    Eigen::Matrix<double, 3, 3> Kp_tran;            // Proportional Gains for the translational control
    Eigen::Matrix<double, 3, 3> Ki_tran;            // Integral Gains for the translational control
    Eigen::Matrix<double, 3, 3> Kd_tran;            // Derivative Gains for the translational control
    Eigen::Matrix<double, 6, 6> Gamma_x_tran;       // Adaptive Gains for the translational control
    Eigen::Matrix<double, 3, 3> Gamma_r_tran;       // Adaptive Gains for the translational control
    Eigen::Matrix<double, 30, 30> Gamma_Theta_tran; // Adaptive Gains for the translational control
    Eigen::Matrix<double, 6, 6> Q_tran;          	// Lyapunov weighting matrix
    Eigen::Matrix<double, 6, 6> P_tran;          	// Solution matrix to continuous Lyapunov equation
    Eigen::Matrix<double, 6, 6> A_tran;          	// Translational system matrix
    Eigen::Matrix<double, 6, 3> B_tran;          	// Translational input matrix
    Eigen::Matrix<double, 6, 6> A_ref_tran;      	// Reference model system matrix
    Eigen::Matrix<double, 6, 3> B_ref_tran;      	// Reference model input matrix
    double dead_zone_delta_translational;        	// Translational deadzone delta radius
    double dead_zone_e0_translational;           	// Translational deadzone error tolerance
    double sigma_x_translational;                	// Translational E-mod gain for x (states)
    double sigma_r_translational;                	// Translational E-mod gain for r (commands)
    double sigma_Theta_translational;            	// Translational E-mod gain for Theta (parameters)
    double projection_x_max_x_translational;     	// Translational Projection limit for Kx_hat
    double projection_epsilon_x_translational;   	// Translational Projection tolerance for Kx_hat
    double projection_x_max_r_translational;     	// Translational Projection limit for Kr_hat
    double projection_epsilon_r_translational;   	// Translational Projection tolerance for Kr_hat
    double projection_x_max_Theta_translational; 	// Translational Projection limit for Theta_hat
    double projection_epsilon_Theta_translational; 	// Translational Projection tolerance for Theta_hat
    
    bool use_filter;                                // Boolean to switch between the second order filter or the 2L VS MRAD
    Eigen::Matrix<double, 2, 2> A_filter_mu;        // Differentiator A matrix for \mu
    Eigen::Matrix<double, 2, 1> B_filter_mu;        // Differentiator B matrix for \mu
    Eigen::Matrix<double, 1, 2> C_filter_mu;        // Differentiator C matrix for q_d
    Eigen::Matrix<double, 2, 2> A_filter_omega_d;   // Differentiator A matrix for omega_d
    Eigen::Matrix<double, 2, 1> B_filter_omega_d;   // Differentiator B matrix for omega_d
    Eigen::Matrix<double, 1, 2> C_filter_omega_d;   // Differentiator C matrix for omega_d

    Eigen::Matrix<double, 3, 3> Kp_omega_ref;    	// Proportional rotational reference model gains
    Eigen::Matrix<double, 3, 3> Ki_omega_ref;    	// Integral rotational reference model gains
    Eigen::Matrix<double, 3, 3> Kp_att;             // Proportional Gains for the rotational control
    Eigen::Matrix<double, 3, 3> Kd_att;             // Derivative Gains for the rotational control
    Eigen::Matrix<double, 3, 1> Ka_att;             // The axis scaling gaisn for the rotational control
    Eigen::Matrix<double, 3, 3>  Gamma_x_rot;    	// Adaptive gain for state feedback parameters
    Eigen::Matrix<double, 3, 3>  Gamma_r_rot;    	// Adaptive gain for command tracking parameters
    Eigen::Matrix<double, 12, 12> Gamma_Theta_rot;  // Adaptive gain for dynamic regression parameters
    Eigen::Matrix<double, 3, 3> Q_rot;           	// Lyapunov weighting matrix (rotation)
    Eigen::Matrix<double, 3, 3> P_rot;           	// Lyapunov solution matrix (rotation)
    Eigen::Matrix<double, 3, 3> A_rot;           	// Rotational system matrix
    Eigen::Matrix<double, 3, 3> B_rot;           	// Rotational input matrix
    Eigen::Matrix<double, 3, 3> A_ref_rot;       	// Rotational reference model matrix
    Eigen::Matrix<double, 3, 3> B_ref_rot;       	// Rotational reference input matrix
    double dead_zone_delta_rotational;           	// Rotational deadzone delta radius
    double dead_zone_e0_rotational;              	// Rotational deadzone error tolerance
    double sigma_x_rotational;                   	// Rotational E-mod gain for x (states)
    double sigma_r_rotational;                   	// Rotational E-mod gain for r (commands)
    double sigma_Theta_rotational;               	// Rotational E-mod gain for Theta (parameters)
    double projection_x_max_x_rotational;        	// Rotational Projection limit for Kx_hat
    double projection_epsilon_x_rotational;      	// Rotational Projection tolerance for Kx_hat
    double projection_x_max_r_rotational;        	// Rotational Projection limit for Kr_hat
    double projection_epsilon_r_rotational;      	// Rotational Projection tolerance for Kr_hat
    double projection_x_max_Theta_rotational;    	// Rotational Projection limit for Theta_hat
    double projection_epsilon_Theta_rotational;  	// Rotational Projection tolerance for Theta_hat 
    
};

// Structure for all the members that are mapped to the rk4 vector AFTER integration
struct controller_integrated_state_members {
    Eigen::Matrix<double, 3, 1> e_tran_pos_I;               // Translational integral error
    Eigen::Matrix<double, 3, 1> e_tran_pos_ref_I;		    // Translational Integral error (reference model - user cmd)
	Eigen::Matrix<double, 6, 1> x_tran_ref;				    // Reference model in I
	Eigen::Matrix<double, 6, 3> K_hat_x_tran;   			// Translational Adaptive gains for x
	Eigen::Matrix<double, 3, 3> K_hat_r_tran;	    		// Translational Adaptive gains for r
	Eigen::Matrix<double, 30, 3> Theta_hat_tran;	    	// Translational Adaptive gains for Theta
	
    Eigen::Matrix<double, 2, 1> state_mu_x_filter;          // States for filter
    Eigen::Matrix<double, 2, 1> state_mu_y_filter;          // States for filter
    Eigen::Matrix<double, 2, 1> state_mu_z_filter;          // States for filter
    Eigen::Matrix<double, 2, 1> state_omega_x_d_filter;     // States for filter
    Eigen::Matrix<double, 2, 1> state_omega_y_d_filter;     // States for filter
    Eigen::Matrix<double, 2, 1> state_omega_z_d_filter;     // States for filter

    Eigen::Matrix<double, 3, 1> e_omega_ref_I;   	        // Integral error (ang vel ref model - desired angular velocity)
	Eigen::Matrix<double, 3, 1> omega_ref;  			    // Angular Velocity Reference model
	Eigen::Matrix<double, 3, 3> K_hat_x_rot;			    // Rotational Adaptive gains for x
	Eigen::Matrix<double, 3, 3> K_hat_r_rot;			    // Rotational Adaptive gains for r
	Eigen::Matrix<double, 12, 3> Theta_hat_rot;			    // Rotational Adaptive gains for Theta
};

// Structure for all the internal members of the controller
struct controller_internal_members {
    double t;                                                      // Time
    Eigen::Matrix<double, 3, 1> r_user;                            // Translational user position command
    Eigen::Matrix<double, 3, 1> r_dot_user;                        // Translational user veloctiy command
    Eigen::Matrix<double, 3, 1> r_ddot_user;                       // Translational user acceleration command 
    double psi_user;                                               // Rotational command for psi
    double psi_user_unwrapped;                                     // Rotational command for psi without the -pi to pi jumps but as a continuous signal
    double psi_dot_user;                                           // Rotational command for psi rate
    Eigen::Matrix<double, 3, 1> r_cmd_tran;                        // Translational reference command
    Eigen::Matrix<double, 6, 1> x_tran_ref_dot;                    // Translational reference model
    Eigen::Matrix<double, 6, 1> x_tran;                            // Translational states [position; velocity]
    Eigen::Matrix<double, 3, 1> x_tran_pos;                        // Translational position
    Eigen::Matrix<double, 3, 1> x_tran_vel;                        // Translational velocity
    Eigen::Matrix<double, 3, 1> e_tran_pos_ref;                    // Translational reference model error
    Eigen::Matrix<double, 6, 1> e_tran;                            // Translational error in states [e_pos; e_vel]
    Eigen::Matrix<double, 3, 1> e_tran_pos;                        // Translational error in position
    Eigen::Matrix<double, 3, 1> e_tran_vel;                        // Translational error in velocity
    Eigen::Matrix<double, 6, 3> K_hat_x_tran_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 3, 3> K_hat_r_tran_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 30, 3> Theta_hat_tran_dot;			   // Adaptive gain to be integrated
    Eigen::Matrix<double, 27, 1> outer_loop_regressor;			   // Outer loop regressor
	Eigen::Matrix<double, 30, 1> augmented_outer_loop_regressor;   // Outer loop augmented regressor
    double dead_zone_value_translational;						   // Dead zone val - OL
    bool proj_op_activated_K_hat_x_translational;				   // Projection activation boolean - OL - K_hat_x
	bool proj_op_activated_K_hat_r_translational;				   // Projection activation boolean - OL - K_hat_r
	bool proj_op_activated_Theta_hat_translational;				   // Projection activation boolean - OL - Theta_hat
    Eigen::Matrix<double, 3, 1> mu_tran_baseline;                  // Baseline control input
    Eigen::Matrix<double, 3, 1> mu_tran_adaptive;                  // Adaptive control input
    Eigen::Matrix<double, 3, 1> mu_tran_I;                         // Virtual control action in the inertial frame
    Eigen::Matrix<double, 3, 1> mu_tran_I_dot;                     // Rate of virtual control action in the inertial frame
    Eigen::Matrix<double, 3, 1> mu_tran_J;                         // Virtual control action in the body frame
    
    Eigen::Matrix<double, 3, 3> Rji;                               // Rotation matrix from the body to the inertial frame
    Eigen::Matrix<double, 3, 3> Rij;                               // Rotation matrix from the inertial to the body frame
    Eigen::Matrix<double, 3, 1> c1;                                // Desired "heading" vector in the inertial frame
    Eigen::Matrix<double, 3, 1> b3d;                               // Desired body z axis
    Eigen::Matrix<double, 3, 1> b2d;                               // Desired body y axis
    Eigen::Matrix<double, 3, 1> b1d;                               // Desired body x axis
    Eigen::Matrix<double, 3, 3> R_d;                               // Desired rotation matrix

    Eigen::Matrix<double, 3, 1> c1_dot;                            // Desired "heading rate" vector in the inertial frame
    Eigen::Matrix<double, 3, 1> n_hat;                             // Intermediate variable to compute b2d_dot
    Eigen::Matrix<double, 3, 1> n_hat_dot;                         // Intermediate variable to compute b2d_dot
    double sigma;                                                  // Intermediate variable to compute b2d_dot
    Eigen::Matrix<double, 3, 1> b3d_dot;                           // Differential of Desired body z axis
    Eigen::Matrix<double, 3, 1> b2d_dot;                           // Differential of Desired body y axis
    Eigen::Matrix<double, 3, 1> b1d_dot;                           // Differential of Desired body x axis
    Eigen::Matrix<double, 3, 3> R_d_dot;                           // Differential of Desired rotation matrix

    Eigen::Matrix<double, 2, 1> internal_state_mu_x_filter;        // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_mu_y_filter;        // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_mu_z_filter;        // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_omega_x_d_filter;   // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_omega_y_d_filter;   // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_omega_z_d_filter;   // Internal States for filter


    Eigen::Matrix<double, 3, 1> omega_d_in_K;                      // Desired angular velocity in the desired frame
    Eigen::Matrix<double, 3, 1> omega_d;                           // Desired angular velocity 
    Eigen::Matrix<double, 3, 1> alpha_d;                           // Desired angular acceleration

    Eigen::Quaterniond q;                                          // Quaternion
    Eigen::Matrix<double, 3, 1> omega;                             // Angular velocity
    Eigen::Matrix<double, 3, 1> omega_cmd;                         // Angular velocity reference command
    Eigen::Matrix<double, 3, 1> omega_ref_dot;                     // Angular velocity reference model
    Eigen::Matrix<double, 3, 1> e_omega_ref;                       // Angular velocity reference model error
    Eigen::Matrix<double, 3, 1> omega_e;                           // Error in the angular velocities
    Eigen::Matrix<double, 3, 1> Xi_e;                              // Error in the attitude states
    Eigen::Matrix<double, 9, 1> inner_loop_regressor;			   // Inner loop regressor vector
	Eigen::Matrix<double, 12, 1> augmented_inner_loop_regressor;   // Inner loop augmented regressor vector    
    Eigen::Matrix<double, 3, 1> tau_rot_baseline;                  // Baseline rotational control input 
    Eigen::Matrix<double, 3, 1> tau_rot_adaptive;                  // Adaptive rotational control input 
    Eigen::Matrix<double, 3, 1> tau_rot;                           // Rotational Control action
    Eigen::Matrix<double, 3, 3> K_hat_x_rot_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 3, 3> K_hat_r_rot_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 12, 3> Theta_hat_rot_dot;				   // Adaptive gain to be integrated
    double dead_zone_value_rotational;							   // Dead zone val - IL
    bool proj_op_activated_K_hat_x_rotational;					   // Projection activation boolean - IL - K_hat_x
	bool proj_op_activated_K_hat_r_rotational;					   // Projection activation boolean - IL - K_hat_r
	bool proj_op_activated_Theta_hat_rotational;				   // Projection activation boolean - IL - Theta_hat

    Eigen::Matrix<double, 4, 1> u;                                 // [thrust; mx; my; mz]
    Eigen::Matrix<double, 4, 1> Thrust;                            // Newtons
    Eigen::Matrix<double, 4, 1> Sat_Thrust;                        // Saturated thrust 
    double alg_duration;                                           // Control execution duration
    std::chrono::high_resolution_clock::time_point alg_start_time; // Algorithm Start timepoint
    std::chrono::high_resolution_clock::time_point alg_end_time;   // Algorithm End timepoint
};

// Structure for all parameter members of the differentiators
struct differentiator_internal_paramters
{
   Eigen::Matrix<double, 3, 6> C_diff;              // C matrix for the differentiator plant
    Eigen::Matrix<double, 6, 3> B_diff;             // B matrix for the differentiator plant  
    Eigen::Matrix<double, 6, 3> L_diff;             // L matrix for the differentiator plant
    Eigen::Matrix<double, 6, 6> A_ref_y_diff;       // A matrix for the differentiator reference model
    Eigen::Matrix<double, 6, 6> A_tran_y_diff;      // A matrix for the transient error plant
    Eigen::Matrix<double, 3, 3> Gamma_y_diff;       // Adaptive gain matrix for the differentiator
    Eigen::Matrix<double, 3, 3> Gamma_Theta_diff;   // Adaptive gain matrix for the differentiator
    Eigen::Matrix<double, 3, 3> Gamma_g_y_diff;     // Adaptive gain matrix for the differentiator
    
    double projection_x_max_K_hat_y_diff;           // Projection params
    double projection_epsilon_K_hat_y_diff;         // Projection params
    
    double projection_x_max_Theta_hat_diff;         // Projection params
    double projection_epsilon_Theta_hat_diff;       // Projection params
    
    double projection_x_max_K_hat_g_y_diff;         // Projection params
    double projection_epsilon_K_hat_g_y_diff;       // Projection params

    double lambda_bar_diff;                         // Norm of the Lambda matrix for the differentiator plant
    double theta_bar_diff;                          // Norm of the Theta matrix for the differentiator plant

    Eigen::Matrix<double, 3, 3> K_ye_diff;          // Initial gains for the differentiator
    Eigen::Matrix<double, 3, 3> Theta_e_diff;       // Initial gains for the differentiator
    Eigen::Matrix<double, 3, 3> K_gye_diff;         // Initial gains for the differentiator
};

// Structure for the members that are mapped to the rk4 vector after integration
struct differentiator_integrated_state_members
{
  Eigen::Matrix<double, 3, 1> int_mu_tran_I;             // The integral of the translational control input
  Eigen::Matrix<double, 6, 1> x_hat_vs_2l_mrad;          // Estimated translational control input for the VS 2L MRAD
  Eigen::Matrix<double, 3, 3> K_hat_y_vs_2l_mrad;        // Adaptive gain matrix for the VS 2L MRAD
  Eigen::Matrix<double, 3, 3> Theta_hat_vs_2l_mrad;      // Adaptive gain matrix for the VS 2L MRAD
  Eigen::Matrix<double, 3, 3> K_hat_g_y_vs_2l_mrad;      // Adaptive gain matrix for the VS 2L MRAD transient term
  Eigen::Matrix<double, 6, 1> eta_vs_2l_mrad;            // Transient error model for the VS 2L MRAD
};

// Structure for the internal members of the observers
struct differentiator_internal_members
{
  Eigen::Matrix<double, 6, 1> x_hat_vs_2l_mrad_dot;     // Estimated translational input states for the VS 2L MRAD
  Eigen::Matrix<double, 3, 3> K_hat_y_vs_2l_mrad_dot;   // Adaptive gain matrix for the VS 2L MRAD
  Eigen::Matrix<double, 3, 3> Theta_hat_vs_2l_mrad_dot; // Adaptive gain matrix for the VS 2L MRAD
  Eigen::Matrix<double, 3, 3> K_hat_g_y_vs_2l_mrad_dot; // Adaptive gain matrix for the VS 2L MRAD transient term
  Eigen::Matrix<double, 6, 1> eta_vs_2l_mrad_dot;       // Transient error model for the VS 2L MRAD
  Eigen::Matrix<double, 3, 1> y_measured_vs_2l_mrad;    // Measured outputs for the VS 2L MRAD
  Eigen::Matrix<double, 6, 1> z_measured_vs_2l_mrad;    // Measured full outputs for the VS 2L MRAD
  Eigen::Matrix<double, 3, 1> Phi_y_vs_2l_mrad;         // Regressor vector for the VS 2L MRAD
  Eigen::Matrix<double, 6, 1> nu_vs_2l_mrad;            // Transient error for the VS 2L MRAD
  Eigen::Matrix<double, 3, 1> u_vs_2l_mrad;             // Virtual control input of the VS 2L MRAD
  double rho_vs_2l_mrad;                                // rho term for the VS 2L MRAD
  Eigen::Matrix<double, 3, 1> beta_vs_2l_mrad;          // beta term for the VS 2L MRAD
  Eigen::Matrix<double, 3, 1> obs_error_vs_2l_mrad;     // (y_measured - y_estimated) for VS 2L MRAD
  bool proj_op_activated_K_hat_y_vs_2l_mrad;            // Boolean to record projection operator activation - VS 2L MRAD
  bool proj_op_activated_Theta_hat_vs_2l_mrad;          // Boolean to record projection operator activation - VS 2L MRAD
  bool proj_op_activated_K_hat_g_y_vs_2l_mrad;          // Boolean to record projection operator activation - VS 2L MRAD
  bool first_run_differentiator = false;                 // First run boolean to initialize the differentiator state
};

// =========================================================================================================
// mrac_geometric.hpp   -- TAILSITTER MRAC geometric controller
//   - Implements a MRAC controller for rotation matrices and angular rates on the TAILSITTER platform.
//   - Inherits base routines and actuator interface from controller_base.
//   - Inherits base routines from blackbox to setup the logging.
// =========================================================================================================
class mrac_geometric : public ::_acsl_::_control_::controller_base , public ::_acsl_::_logger_::blackbox
{

public:
    // -------------------------------------------------------------------------
    // Constructor:
    //   - Accepts simlog reference for logging duties.
    //   - Passes simlog to controller_base for unified logging.
    // -------------------------------------------------------------------------
    mrac_geometric(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory);

    // -------------------------------------------------------------------------
    // Destructor:
    //   - Allows cleanup in derived class.
    // -------------------------------------------------------------------------
    virtual ~mrac_geometric() = default;

    
    // -------------------------------------------------------------------------
    // Override: update()
    //   - Update controller state (to be implemented in mrac_geometric.cpp).
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
    // Define the state vector
    _control_::rk4_array<double, NSI> y;
    _control_::rk4_array<double, NSI> dy;

    // Define the model
    void model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t);

    // Create a RungeKutta object for integration functionality.
    boost::numeric::odeint::runge_kutta4<_control_::rk4_array<double, NSI>> rk4;

    // Define the internal parameter members of the controller 
    ::_acsl_::_tailsitter_::_mrac_geometric_::controller_internal_parameters cip;

    // Define the internal members of the controller
    ::_acsl_::_tailsitter_::_mrac_geometric_::controller_internal_members cim;   

    // Define the internal integrated state members of the controller
    ::_acsl_::_tailsitter_::_mrac_geometric_::controller_integrated_state_members csm;

    // Member to unwrap the heading for heading command
    ::_shared_::_compute_::SimplePsiUnwrapState psiState;

    // Define the internal parameter members of the differentiator
    ::_acsl_::_tailsitter_::_mrac_geometric_::differentiator_internal_paramters dip;

    // Define the internal members of the differentiator
    ::_acsl_::_tailsitter_::_mrac_geometric_::differentiator_internal_members dim;

    // Define the internal integrated state members of the differentiator
    ::_acsl_::_tailsitter_::_mrac_geometric_::differentiator_integrated_state_members dsm;

private:
    // -------------------------------------------------------------------------
    // NON API CONTROLLER SPECIFIC FUNCTIONS - WILL BE DIFF FOR DIFF CONTROLLERS
    // -------------------------------------------------------------------------
    // Function to compute the translational control in the inertial frame
    void compute_translational_control_in_I();

    // Function to compute the thrust (u1) and the desired rotation matrix
    void compute_u1_R_d();

    // Function to observe the outerloop control input
    void compute_translational_control_rate();

    // Function to compute the rotational control input
    void compute_rotational_control();

    // Function to compute the normalized thrust
    void compute_normalized_thrusts();

    // Assign the values from rk4 to controller internal members
    void assign_from_rk4();

};

}   // namespace _mrac_geometric_

}   // namespace _tailsitter_
    
}   // namespace _acsl_


#endif  //  MRAC_GEOMETRIC_TAILSITTER_HPP_

