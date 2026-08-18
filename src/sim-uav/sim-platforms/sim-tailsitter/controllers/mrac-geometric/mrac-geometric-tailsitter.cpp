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
 * File:        mrac-geometric-tailsitter.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 23, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: MRAC with geometric and angular velocities for the TAILSITTER. Inherts the class controller_base for the
 *              basic functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "mrac-geometric-tailsitter.hpp"

namespace _acsl_
{

namespace _tailsitter_
{

namespace _mrac_geometric_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
mrac_geometric::mrac_geometric(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                              : ::_acsl_::_control_::controller_base(logger, trajectory)
{
    // Initial Conditions
    init();   
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void mrac_geometric::read_params(const std::string& jsonFile)
{
    // Implementation here
    std::ifstream file(jsonFile);
    nlohmann::json j;
    file >> j;

    // Controller matrices ----------------------------------------------------------------------------------------------
    // Translational parameters
    cip.Kp_refmod_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KP_refmod_translational"], 3, 3);
	cip.Ki_refmod_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KI_refmod_translational"], 3, 3);
	cip.Kd_refmod_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KD_refmod_translational"], 3, 3);
    cip.Kp_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KP_translational"], 3, 3);
    cip.Ki_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KI_translational"], 3, 3);
    cip.Kd_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KD_translational"], 3, 3);
    cip.Gamma_x_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_x_translational"], 6, 6);
	cip.Gamma_r_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_r_translational"], 3, 3);
	cip.Gamma_Theta_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_Theta_translational"], 30, 30);
	cip.Q_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Q_translational"], 6, 6);

    // Rotational parameters
    cip.Kp_omega_ref = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KP_omega_ref"], 3, 3);
	cip.Ki_omega_ref = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KI_omega_ref"], 3, 3);
    cip.Kp_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KP_rotational"], 3, 3);
    cip.Kd_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["KD_rotational"], 3, 3);
    cip.Ka_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Ka_rotational"], 3, 1);
    cip.Gamma_x_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_x_rotational"], 3, 3);
	cip.Gamma_r_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_r_rotational"], 3, 3);
	cip.Gamma_Theta_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_Theta_rotational"], 12, 12);
	cip.Q_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["ROTATIONAL"]["Q_rotational"], 3, 3);

    // Differentiator matrices
    cip.use_filter = j["FILTERS"]["use_filter"];
    cip.A_filter_mu = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["FILTERS"]["A_filter_mu"], 2, 2);
    cip.B_filter_mu = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["FILTERS"]["B_filter_mu"], 2, 1);
    cip.C_filter_mu = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["FILTERS"]["C_filter_mu"], 1, 2);
    cip.A_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["FILTERS"]["A_filter_omega_d"], 2, 2);
    cip.B_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["FILTERS"]["B_filter_omega_d"], 2, 1);
    cip.C_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["FILTERS"]["C_filter_omega_d"], 1, 2);

    // Robustification Parameters
	cip.dead_zone_delta_translational = j["ROBUSTIFICATION"]["dead_zone_delta_translational"];
	cip.dead_zone_e0_translational = j["ROBUSTIFICATION"]["dead_zone_e0_translational"];
	cip.sigma_x_translational = j["ROBUSTIFICATION"]["sigma_x_translational"];
	cip.sigma_r_translational = j["ROBUSTIFICATION"]["sigma_r_translational"];
	cip.sigma_Theta_translational = j["ROBUSTIFICATION"]["sigma_Theta_translational"];
    cip.projection_x_max_x_translational = j["ROBUSTIFICATION"]["projection_x_max_x_translational"];
	cip.projection_epsilon_x_translational = j["ROBUSTIFICATION"]["projection_epsilon_x_translational"];
    cip.projection_x_max_r_translational = j["ROBUSTIFICATION"]["projection_x_max_r_translational"];
	cip.projection_epsilon_r_translational = j["ROBUSTIFICATION"]["projection_epsilon_r_translational"];
    cip.projection_x_max_Theta_translational = j["ROBUSTIFICATION"]["projection_x_max_Theta_translational"];
	cip.projection_epsilon_Theta_translational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_translational"];

    cip.dead_zone_delta_rotational = j["ROBUSTIFICATION"]["dead_zone_delta_rotational"];
	cip.dead_zone_e0_rotational = j["ROBUSTIFICATION"]["dead_zone_e0_rotational"];
    cip.sigma_x_rotational = j["ROBUSTIFICATION"]["sigma_x_rotational"];
	cip.sigma_r_rotational = j["ROBUSTIFICATION"]["sigma_r_rotational"];
	cip.sigma_Theta_rotational = j["ROBUSTIFICATION"]["sigma_Theta_rotational"];
    cip.projection_x_max_x_rotational = j["ROBUSTIFICATION"]["projection_x_max_x_rotational"]; 
	cip.projection_epsilon_x_rotational = j["ROBUSTIFICATION"]["projection_epsilon_x_rotational"];
    cip.projection_x_max_r_rotational = j["ROBUSTIFICATION"]["projection_x_max_r_rotational"];
	cip.projection_epsilon_r_rotational = j["ROBUSTIFICATION"]["projection_epsilon_r_rotational"];
    cip.projection_x_max_Theta_rotational = j["ROBUSTIFICATION"]["projection_x_max_Theta_rotational"];
	cip.projection_epsilon_Theta_rotational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_rotational"];

    // Adaptive differentiator gains and parameter matrices
    dip.C_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["C_differentiator"], 3, 6);
    dip.B_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["B_differentiator"], 6, 3);
    dip.L_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["L_differentiator"], 6, 3);

    dip.A_ref_y_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["A_ref_y_differentiator"], 6, 6);
    dip.A_tran_y_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["A_tran_y_differentiator"], 6, 6);

    dip.Gamma_y_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["Gamma_y_differentiator"], 3, 3);
    dip.Gamma_Theta_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["Gamma_Theta_differentiator"], 3, 3);
    dip.Gamma_g_y_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["Gamma_g_y_differentiator"], 3, 3);

    dip.projection_x_max_K_hat_y_diff = j["DIFFERENTIATOR"]["projection_x_max_K_hat_y_differentiator"];
    dip.projection_epsilon_K_hat_y_diff = j["DIFFERENTIATOR"]["projection_epsilon_K_hat_y_differentiator"];
    
    dip.projection_x_max_Theta_hat_diff = j["DIFFERENTIATOR"]["projection_x_max_Theta_hat_differentiator"];
    dip.projection_epsilon_Theta_hat_diff = j["DIFFERENTIATOR"]["projection_epsilon_Theta_hat_differentiator"];
    
    dip.projection_x_max_K_hat_g_y_diff = j["DIFFERENTIATOR"]["projection_x_max_K_hat_g_y_differentiator"];
    dip.projection_epsilon_K_hat_g_y_diff = j["DIFFERENTIATOR"]["projection_epsilon_K_hat_g_y_differentiator"];

    dip.lambda_bar_diff = j["DIFFERENTIATOR"]["lambda_bar_differentiator"];
    dip.theta_bar_diff = j["DIFFERENTIATOR"]["theta_bar_differentiator"];

    dip.K_ye_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["K_ye_differentiator"], 3, 3);
    dip.Theta_e_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["Theta_e_differentiator"], 3, 3);
    dip.K_gye_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["K_gye_differentiator"], 3, 3);

}

// Implementing virutal functions from controller_base
void mrac_geometric::init(){
    // Reading in the parameters
    read_params("../chrono-assets/parameters/tailsitter/MRAC_GEOMETRIC/gains_MRAC_GEOMETRIC.json");

    // Set the inital conditions
    y.fill(0.0);
    dy.fill(0.0);

    // Setup the logging
    InitiateLogging();
    ConfigureHeaders();

    /// Translational - Initialize matrices
	// Initialize to zero the 6x6 matrix and set the top-right 3x3 block as follows
	::_shared_::_initiate_::initMat(cip.A_tran);
	cip.A_tran.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

	// Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
	::_shared_::_initiate_::initMat(cip.B_tran);
	cip.B_tran.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();

	// Initialize to zero the 6x6 matrix
	::_shared_::_initiate_::initMat(cip.A_ref_tran);
	// Set the top-right 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
	// Set the bottom-left 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(3, 0) = -cip.Kp_refmod_tran;
	// Set the bottom-right 3x3 block as follows
	cip.A_ref_tran.block<3, 3>(3, 3) = -cip.Kd_refmod_tran;

	// Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
	::_shared_::_initiate_::initMat(cip.B_ref_tran);
	cip.B_ref_tran.block<3, 3>(3, 0) = (1.0 / MASS) * Eigen::Matrix3d::Identity();

	// Solve the continuous Lyapunov equation to compute P_translational
	cip.P_tran = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_tran, cip.Q_tran);

    /// Rotational - Initialize matrices
	// Intialize to zero the 3x3 matrix
	::_shared_::_initiate_::initMat(cip.A_rot);

	// Initlize to identity the 3x3 matrix 
	cip.B_rot = Eigen::Matrix3d::Identity();
	
	// Set the 3x3 matrix as follows
	cip.A_ref_rot << -cip.Kp_omega_ref;

	// Set the 3x3 matrix as follows
	cip.B_ref_rot = Eigen::Matrix3d::Identity();

	// Solve the continuous Lyapunov eequation to compute P_rotational
	cip.P_rot = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_rot, cip.Q_rot);

    // Initiate the rk4 vector for the differentiator gain matrices
    int index = 201;
    ::_shared_::_serialize_::assignElementsToDxdt(dip.K_ye_diff, this->y, index);           // For VS 2L MRAD
    ::_shared_::_serialize_::assignElementsToDxdt(dip.Theta_e_diff, this->y, index);        // For VS 2L MRAD
    ::_shared_::_serialize_::assignElementsToDxdt(dip.K_gye_diff, this->y, index);          // For VS 2L MRAD

    // DISPLAY A DEBUG MESSAGE
    _message_::SIMULATOR_INFO("[SIMCTL]: INITIAL PARAMETERS COMPUTED FOR MRAD DIFFERENTIATOR");
}

// Update function for the controller
void mrac_geometric::update(double time, 
                            double x,
                            double y,
                            double z,
                            double vx,
                            double vy,
                            double vz,
                            double q0,
                            double q1,
                            double q2,
                            double q3,
                            double roll,
                            double pitch,
                            double yaw,
                            double w_x,
                            double w_y,
                            double w_z)     
{
    // 1. Assign all the states ----------------------------------------------------
    cim.t = time;

    // Assign the Translational States
    cim.x_tran_pos << x,y,z;
    cim.x_tran_vel << vx, vy, vz;
    cim.x_tran << cim.x_tran_pos,
                  cim.x_tran_vel;

    // Assign the quaternions and ensure it is normalized
    // q0 = w, q1 = x, q2 = y, q3 = z
    cim.q = Eigen::Quaterniond(q0, q1, q2, q3).normalized();

    // Assign the rotation matrices
    cim.Rji = cim.q.toRotationMatrix(); // rotation from the body to the inertial frame
    cim.Rij = cim.Rji.transpose();      // rotation from the inertial to the body frame

    // Asign the angular velocity
    cim.omega << w_x, w_y, w_z;

    // 2. Get the reference trajectory ---------------------------------------------
    m_traj.UpdateModule(time);
    cim.r_user = m_traj.GetPosition();
    cim.r_dot_user = m_traj.GetVelocity();
    cim.r_ddot_user = m_traj.GetAcceleration();   
    cim.psi_user = m_traj.GetEulerAngle()(2);
    cim.psi_user_unwrapped = ::_shared_::_compute_::unwrapPsiSimple(cim.psi_user, this->psiState);
    // cim.psi_dot_user = m_traj.GetEulerRate()(2);
    cim.psi_dot_user = 0.0;

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Assign the values from the integrator ------------------------------------
    assign_from_rk4();

    // 5. Initialize the differentiator's estimated state vector -------------------
    if(!dim.first_run_differentiator) {
        // Initate the x_hat vector
        dsm.x_hat_vs_2l_mrad << 0.0, 0.0, 0.0, 6.56683, -2.05528, -9.0455;
        // dsm.x_hat_vs_2l_mrad << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        dim.x_hat_vs_2l_mrad_dot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 

        // Initiate the corresponding rk4 vector
        int index = 231; // For VS 2L MRAD states
        ::_shared_::_serialize_::assignElementsToDxdt(dsm.x_hat_vs_2l_mrad, this->y, index);
        _message_::SIMULATOR_INFO("[SIMCTL]: INITIAL X HAT VECTOR SET FOR DIFFERENTIATOR");

        // Set the initialization condition to true
        dim.first_run_differentiator = true;
    }
}

// Function to assign elements from the rk4 integrator
void mrac_geometric::assign_from_rk4()
{
    int index = 0;

    //------------ assign after integration  ------------//
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_mu_x_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_mu_y_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_mu_z_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_z_d_filter, y, index);

    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_ref_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.x_tran_ref, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_x_tran, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_r_tran, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.Theta_hat_tran, y, index);

    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_omega_ref_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.omega_ref, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_x_rot, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_r_rot, y, index);
	::_shared_::_deserialize_::assignElementsToMembers(csm.Theta_hat_rot, y, index);

    //------------ assign elements of the differentiator after integration  ------------//
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.K_hat_y_vs_2l_mrad, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.Theta_hat_vs_2l_mrad, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.K_hat_g_y_vs_2l_mrad, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.int_mu_tran_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.x_hat_vs_2l_mrad, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.eta_vs_2l_mrad, y, index);  
}

// Model function for integration
void mrac_geometric::model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t)
{
    int index = 0;
    
    //------------ Fill up the dy for integration  ------------//
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_mu_x_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_mu_y_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_mu_z_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);

    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos_ref, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.x_tran_ref_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_x_tran_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_r_tran_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.Theta_hat_tran_dot, dy, index);

    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_omega_ref, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.omega_ref_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_x_rot_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_r_rot_dot, dy, index);
	::_shared_::_serialize_::assignElementsToDxdt(cim.Theta_hat_rot_dot, dy, index);

    //------------ Fill up the dy for integration for the differentiator ------------//
    ::_shared_::_serialize_::assignElementsToDxdt(dim.K_hat_y_vs_2l_mrad_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.Theta_hat_vs_2l_mrad_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.K_hat_g_y_vs_2l_mrad_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.mu_tran_I, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.x_hat_vs_2l_mrad_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.eta_vs_2l_mrad_dot, dy, index);

    
}


// Function to compute the outerloop control in I
void mrac_geometric::compute_translational_control_in_I()
{
    // Compute the error in the states
    cim.e_tran_pos << cim.x_tran_pos - csm.x_tran_ref.head<3>();
    cim.e_tran_vel << cim.x_tran_vel - csm.x_tran_ref.tail<3>();
    cim.e_tran << cim.e_tran_pos,
                  cim.e_tran_vel;

    // Compute the translational position error between the reference model and the user defined trajectory
    cim.e_tran_pos_ref << csm.x_tran_ref.head<3>() - cim.r_user;

    // Compute the refernce command input [reference model - user_defined_trajectory]
    cim.r_cmd_tran << MASS * ( - cip.Ki_refmod_tran * csm.e_tran_pos_ref_I      // Integral term
                               + cip.Kp_refmod_tran * cim.r_user                // Proportional term
                               + cip.Kd_refmod_tran * cim.r_dot_user            // Derivative term
                               + cim.r_ddot_user );                             // Feedforward term

    // Reference model
    cim.x_tran_ref_dot << cip.A_ref_tran * csm.x_tran_ref
                        + cip.B_ref_tran * cim.r_cmd_tran;

    // Compute the baseline control input
    cim.mu_tran_baseline << MASS*( - cip.Kp_tran * cim.e_tran_pos               // Proportional term
                                   - cip.Kd_tran * cim.e_tran_vel               // Derivative term
                                   - cip.Ki_tran * csm.e_tran_pos_I             // Integral term
                                   + cim.x_tran_ref_dot.tail<3>() )             // Feedforward term
                                   - MASS * G * e3_basis;                       // Weight Dynamic inversion term

    // Temp regressor
    cim.outer_loop_regressor.setZero();
                                   
    // Compute the augmented regressor vector
    cim.augmented_outer_loop_regressor << cim.mu_tran_baseline,
										  cim.outer_loop_regressor;

    // Cache the transpose of the tracking error * P * B
	Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.e_tran.transpose() * cip.P_tran * cip.B_tran;

	// Computing the scalar value output from the dead-zone modification modulation function
	cim.dead_zone_value_translational = ::_shared_::_deadzone_operator_::deadZoneModulationFunction(cim.e_tran.transpose(),
																   						 cip.dead_zone_delta_translational,
																   							cip.dead_zone_e0_translational);

	// Adaptive laws
	cim.K_hat_x_tran_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_x_tran,
																				cim.dead_zone_value_translational,
																				cim.x_tran,
																				e_transpose_p_b,
																				cip.sigma_x_translational,
																				csm.K_hat_x_tran);

	cim.K_hat_r_tran_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_r_tran,
																				cim.dead_zone_value_translational,
																				cim.r_cmd_tran,
																				e_transpose_p_b,
																				cip.sigma_r_translational,
																				csm.K_hat_r_tran);

	cim.Theta_hat_tran_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_tran,
																				  cim.dead_zone_value_translational,
																				  cim.augmented_outer_loop_regressor,
																				  e_transpose_p_b,
																				  cip.sigma_Theta_translational,
																				  csm.Theta_hat_tran);

    // Projection operator - Ball
    // Projection operator K_hat_x
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_x_tran)> proj_op_output_K_hat_x_translational = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_x_tran, 
                                                                    cim.K_hat_x_tran_dot,
                                                                    cip.projection_x_max_x_translational,
                                                                    cip.projection_epsilon_x_translational);

    cim.K_hat_x_tran_dot = proj_op_output_K_hat_x_translational.projected_matrix;
    cim.proj_op_activated_K_hat_x_translational = proj_op_output_K_hat_x_translational.projection_operator_activated;
    
    // Projection operator K_hat_r
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_r_tran)> proj_op_output_K_hat_r_translational = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_r_tran,
                                                                    cim.K_hat_r_tran_dot,
                                                                    cip.projection_x_max_r_translational,
                                                                    cip.projection_epsilon_r_translational);

    cim.K_hat_r_tran_dot = proj_op_output_K_hat_r_translational.projected_matrix;
    cim.proj_op_activated_K_hat_r_translational = proj_op_output_K_hat_r_translational.projection_operator_activated;															 

    // Projection operator Theta_hat
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.Theta_hat_tran)> proj_op_output_Theta_hat_translational =
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.Theta_hat_tran,
                                                                    cim.Theta_hat_tran_dot,
                                                                    cip.projection_x_max_Theta_translational,
                                                                    cip.projection_epsilon_Theta_translational);

    cim.Theta_hat_tran_dot = proj_op_output_Theta_hat_translational.projected_matrix;
    cim.proj_op_activated_Theta_hat_translational = proj_op_output_Theta_hat_translational.projection_operator_activated;

    // Adaptive control law
    cim.mu_tran_adaptive << csm.K_hat_x_tran.transpose() * cim.x_tran
                          + csm.K_hat_r_tran.transpose() * cim.r_cmd_tran;
                          - csm.Theta_hat_tran.transpose() * cim.augmented_outer_loop_regressor;

    // Compute with the dynamic inversion without aerodynamics
    cim.mu_tran_I << cim.mu_tran_baseline + cim.mu_tran_adaptive;
}

// Function to compute the outerloop translational control rate in I using the differentiator
void mrac_geometric::compute_translational_control_rate()
{
    // Filter computations --------------------------------------------------------------
    // Compute the internal state for rate of change of mu
    cim.internal_state_mu_x_filter << cip.A_filter_mu * csm.state_mu_x_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(0);

    cim.internal_state_mu_y_filter << cip.A_filter_mu * csm.state_mu_y_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(1);

    cim.internal_state_mu_z_filter << cip.A_filter_mu * csm.state_mu_z_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(2);

    // MRAD 2L VS computations ----------------------------------------------------------
    // Assign the total z_measured vector (use actual state vector)
    dim.z_measured_vs_2l_mrad << dsm.int_mu_tran_I(0),
                                 dsm.int_mu_tran_I(1),
                                 dsm.int_mu_tran_I(2),
                                 cim.mu_tran_I(0),
                                 cim.mu_tran_I(1),
                                 cim.mu_tran_I(2);

    // Assign the measured output (use actual state vector)
    dim.y_measured_vs_2l_mrad << dip.C_diff * dim.z_measured_vs_2l_mrad;

    // Compute the observation error
    dim.obs_error_vs_2l_mrad << dim.y_measured_vs_2l_mrad - dip.C_diff * dsm.x_hat_vs_2l_mrad;

    // Compute the transient error
    dim.nu_vs_2l_mrad << dim.z_measured_vs_2l_mrad - dsm.x_hat_vs_2l_mrad - dsm.eta_vs_2l_mrad;

    // Regressor vector for the observer - It is the signal itself as we assume some noise in it
    dim.Phi_y_vs_2l_mrad << cim.mu_tran_I;

    // Compute rho Eq. (40)
    dim.rho_vs_2l_mrad = dip.lambda_bar_diff * dip.theta_bar_diff * dim.Phi_y_vs_2l_mrad.norm();

    // Compute beta Eq. (40)
    const Eigen::Vector3d nu_y_vs_2l_mrad = dip.C_diff * dim.nu_vs_2l_mrad;

    const double nu_y_norm_vs_2l_mrad = nu_y_vs_2l_mrad.norm();

    if (nu_y_norm_vs_2l_mrad <= 1e-6) { 
        dim.beta_vs_2l_mrad.setZero();
    }
    else {
        dim.beta_vs_2l_mrad = 0.002 * dim.rho_vs_2l_mrad * (nu_y_vs_2l_mrad / nu_y_norm_vs_2l_mrad);
    }

    // Compute the virtual control input for the differentitator
    dim.u_vs_2l_mrad << cim.mu_tran_I - dsm.K_hat_y_vs_2l_mrad.transpose() * dim.y_measured_vs_2l_mrad 
                                      + dsm.Theta_hat_vs_2l_mrad.transpose() * dim.Phi_y_vs_2l_mrad
                                      - dsm.K_hat_g_y_vs_2l_mrad.transpose() * dim.obs_error_vs_2l_mrad
                                      + dim.beta_vs_2l_mrad;

    // Compute the estimated state to be integrated
    dim.x_hat_vs_2l_mrad_dot << dip.A_ref_y_diff * dsm.x_hat_vs_2l_mrad + dip.B_diff * dim.u_vs_2l_mrad
                              + dip.L_diff * dip.C_diff * dim.nu_vs_2l_mrad;

    // Compute the transient error model
    dim.eta_vs_2l_mrad_dot << dip.A_tran_y_diff * dsm.eta_vs_2l_mrad;
    
    // Compute the derivative of the differentiator gains to be integrated
    dim.K_hat_y_vs_2l_mrad_dot << -dip.Gamma_y_diff * dim.y_measured_vs_2l_mrad * dim.nu_vs_2l_mrad.transpose() * dip.C_diff.transpose();
    dim.Theta_hat_vs_2l_mrad_dot << -dip.Gamma_Theta_diff * dim.Phi_y_vs_2l_mrad * dim.nu_vs_2l_mrad.transpose() * dip.C_diff.transpose();
    dim.K_hat_g_y_vs_2l_mrad_dot << -dip.Gamma_g_y_diff * dim.obs_error_vs_2l_mrad * dim.nu_vs_2l_mrad.transpose() * dip.C_diff.transpose();

    // Projection operator - Ball - NO boolean to switch off projection. It is always on.

    // Projection operator K_hat_y_vs_2l_mrad
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(dsm.K_hat_y_vs_2l_mrad)> proj_op_output_K_hat_y_vs_2l_mrad = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(dsm.K_hat_y_vs_2l_mrad,
                                                                    dim.K_hat_y_vs_2l_mrad_dot,
                                                                    dip.projection_x_max_K_hat_y_diff,
                                                                    dip.projection_epsilon_K_hat_y_diff);
                                                                    
    dim.K_hat_y_vs_2l_mrad_dot = proj_op_output_K_hat_y_vs_2l_mrad.projected_matrix;
    dim.proj_op_activated_K_hat_y_vs_2l_mrad = proj_op_output_K_hat_y_vs_2l_mrad.projection_operator_activated;

    // Projection operator Theta_hat_vs_2l_mrad
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(dsm.Theta_hat_vs_2l_mrad)> proj_op_output_Theta_hat_vs_2l_mrad = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(dsm.Theta_hat_vs_2l_mrad,
                                                                    dim.Theta_hat_vs_2l_mrad_dot,
                                                                    dip.projection_x_max_Theta_hat_diff,
                                                                    dip.projection_epsilon_Theta_hat_diff);

    dim.Theta_hat_vs_2l_mrad_dot = proj_op_output_Theta_hat_vs_2l_mrad.projected_matrix;
    dim.proj_op_activated_Theta_hat_vs_2l_mrad = proj_op_output_Theta_hat_vs_2l_mrad.projection_operator_activated;

    // Projection operator K_hat_g_y_mrad
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(dsm.K_hat_g_y_vs_2l_mrad)> proj_op_output_K_hat_g_y_vs_2l_mrad = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(dsm.K_hat_g_y_vs_2l_mrad,
                                                                    dim.K_hat_g_y_vs_2l_mrad_dot,
                                                                    dip.projection_x_max_K_hat_g_y_diff,
                                                                    dip.projection_epsilon_K_hat_g_y_diff);

    dim.K_hat_g_y_vs_2l_mrad_dot = proj_op_output_K_hat_g_y_vs_2l_mrad.projected_matrix;
    dim.proj_op_activated_K_hat_g_y_vs_2l_mrad = proj_op_output_K_hat_g_y_vs_2l_mrad.projection_operator_activated; // Assign the measured output (use actual state vector)


    // Compute the rate of change of the translational virtual control input
    if (cip.use_filter) {
        cim.mu_tran_I_dot(0) = cip.C_filter_mu * csm.state_mu_x_filter;
        cim.mu_tran_I_dot(1) = cip.C_filter_mu * csm.state_mu_y_filter;
        cim.mu_tran_I_dot(2) = cip.C_filter_mu * csm.state_mu_z_filter;
    }
    else {
        if (cim.t < 4.0) {
            cim.mu_tran_I_dot(0) = cip.C_filter_mu * csm.state_mu_x_filter;
            cim.mu_tran_I_dot(1) = cip.C_filter_mu * csm.state_mu_y_filter;
            cim.mu_tran_I_dot(2) = cip.C_filter_mu * csm.state_mu_z_filter;
        }
        else {
            cim.mu_tran_I_dot(0) = dim.x_hat_vs_2l_mrad_dot(3);
            cim.mu_tran_I_dot(1) = dim.x_hat_vs_2l_mrad_dot(4);
            cim.mu_tran_I_dot(2) = dim.x_hat_vs_2l_mrad_dot(5);
        }
        
    }

}

// Compute the orientation commands and the desired total thrust
void mrac_geometric::compute_u1_R_d()
{
	// Compute the desired total thrust
    cim.u(0) = cim.mu_tran_I.norm();

    // Compute the desired body frame z axis
    cim.b3d = -cim.mu_tran_I / cim.u(0);

    // Compute the desired "heading" vector 
    cim.c1 << std::cos(cim.psi_user),
              std::sin(cim.psi_user),
               0.0;

    // Construct an orthonormal triad
    cim.n_hat = cim.b3d.cross(cim.c1);
    cim.sigma = cim.n_hat.norm();
    cim.b2d =  cim.n_hat / cim.sigma;

    cim.b1d = cim.b2d.cross(cim.b3d);

    // Construct the desired rotation matrix
    cim.R_d.col(0) = cim.b1d;
    cim.R_d.col(1) = cim.b2d;
    cim.R_d.col(2) = cim.b3d;

    // Compute R_d_dot
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    
    // compute b3d_dot
    cim.b3d_dot = -1.0 * (1/cim.mu_tran_I.norm()) * (I - cim.b3d * cim.b3d.transpose()) * cim.mu_tran_I_dot;
    
    // compute b2d_dot
    cim.c1_dot << -1.0 * cim.psi_dot_user * std::sin(cim.psi_user),
                   cim.psi_dot_user * std::cos(cim.psi_user),
                   0.0;

    cim.n_hat_dot = cim.b3d_dot.cross(cim.c1) + cim.b3d.cross(cim.c1_dot);

    cim.b2d_dot = (1/cim.sigma) * (I - cim.b2d * cim.b2d.transpose()) * cim.n_hat_dot;

    // compute b1d_dot
    cim.b1d_dot = cim.b2d_dot.cross(cim.b3d) + cim.b2d.cross(cim.b3d_dot);

    // assign to R_d_dot
    cim.R_d_dot.col(0) = cim.b1d_dot;
    cim.R_d_dot.col(1) = cim.b2d_dot;
    cim.R_d_dot.col(2) = cim.b3d_dot;

    Eigen::Matrix3d omega_d_K_skew;
    omega_d_K_skew = cim.R_d.transpose() * cim.R_d_dot;

    // build the desired angular velocity in the desired frame R_d from the skew symmetric matric omega_d_K_skew
    // omega_d_K_skew = [   0 -wd3  wd2]
    //                  [ wd3    0 -wd1]
    //                  [-wd2  wd1    0]
    cim.omega_d_in_K(0) = omega_d_K_skew(2, 1);
    cim.omega_d_in_K(1) = omega_d_K_skew(0, 2);
    cim.omega_d_in_K(2) = omega_d_K_skew(1, 0);

    // transform the desired angular velocity in the desired frame R_d to the current body frame J
    cim.omega_d = cim.Rji.transpose() * cim.R_d * cim.omega_d_in_K;

    // Compute the internal state for angular acceleration
    cim.internal_state_omega_x_d_filter << cip.A_filter_omega_d * csm.state_omega_x_d_filter
                                         + cip.B_filter_omega_d * cim.omega_d(0);

    cim.internal_state_omega_y_d_filter << cip.A_filter_omega_d * csm.state_omega_y_d_filter
                                         + cip.B_filter_omega_d * cim.omega_d(1);

    cim.internal_state_omega_z_d_filter << cip.A_filter_omega_d * csm.state_omega_z_d_filter
                                         + cip.B_filter_omega_d * cim.omega_d(2);

    // Compute the desired angular acceleration
    Eigen::Matrix<double, 3, 1> omega_dot_d_J;
    omega_dot_d_J(0) = cip.C_filter_omega_d * csm.state_omega_x_d_filter;
    omega_dot_d_J(1) = cip.C_filter_omega_d * csm.state_omega_y_d_filter;
    omega_dot_d_J(2) = cip.C_filter_omega_d * csm.state_omega_z_d_filter;

    cim.alpha_d = omega_dot_d_J + cim.omega.cross(cim.omega_d);
}


// Compute the rotational control
void mrac_geometric::compute_rotational_control()
{
    // Compute the error in the attitude
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 1> e1 = I.col(0);
    Eigen::Matrix<double, 3, 1> e2 = I.col(1);
    Eigen::Matrix<double, 3, 1> e3 = I.col(2);
    Eigen::Vector3d a1e1 = cip.Ka_att(0) * e1;
    Eigen::Vector3d a2e2 = cip.Ka_att(1) * e2;
    Eigen::Vector3d a3e3 = cip.Ka_att(2) * e3;
    Eigen::Vector3d r1 = cim.R_d.transpose() * cim.Rji * e1;
    Eigen::Vector3d r2 = cim.R_d.transpose() * cim.Rji * e2;
    Eigen::Vector3d r3 = cim.R_d.transpose() * cim.Rji * e3;

    cim.Xi_e = a1e1.cross(r1) + a2e2.cross(r2) + a3e3.cross(r3);

    // Compute the error in the angular velocities
    cim.omega_e << cim.omega - csm.omega_ref;

    // Compute the error between the angular velocity reference model and the desired angular velocity
    cim.e_omega_ref << csm.omega_ref - cim.omega_d;

    // Compute the command angular velocity
    cim.omega_cmd << cip.Kp_omega_ref * cim.omega_d - cip.Ki_omega_ref * csm.e_omega_ref_I + cim.alpha_d;

    // Compute the derivative of the refernce angular velocity
    cim.omega_ref_dot << cip.A_ref_rot * csm.omega_ref + cip.B_ref_rot * cim.omega_cmd;

    // Cache the feedforward term
    Eigen::Vector3d fft;
    fft = inertia_matrix_q * ( cim.omega.cross(inertia_matrix_q * cim.omega) - cim.alpha_d );
    
    // Compute the baseline control input
    cim.tau_rot_baseline << inertia_matrix_q * ( - cip.Kp_att * cim.Xi_e        // Proportional term
                                                 - cip.Kd_att * cim.omega_e)    // Derivative term
                                                 + fft;                         // Feedforward term

    // Temp regressor
    cim.inner_loop_regressor.setZero();

    // Compute the augmented regressor vector
    cim.augmented_inner_loop_regressor << cim.tau_rot_baseline, cim.inner_loop_regressor;

    // Cache the transpose of the tracking error * P * B
    Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.omega_e.transpose() * cip.P_rot * cip.B_rot;

    // Computing the scalar value output from the dead-zone modification modulation function
	cim.dead_zone_value_rotational = ::_shared_::_deadzone_operator_::deadZoneModulationFunction(cim.omega_e.transpose(),
																								 cip.dead_zone_delta_rotational,
																								 cip.dead_zone_e0_rotational);

	// Adaptive laws
	cim.K_hat_x_rot_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_x_rot,
																				cim.dead_zone_value_rotational,
																				cim.omega,
																				e_transpose_p_b,
																				cip.sigma_x_rotational,
																				csm.K_hat_x_rot);

	cim.K_hat_r_rot_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_r_rot,
																				cim.dead_zone_value_rotational,
																				cim.omega_cmd,
																				e_transpose_p_b,
																				cip.sigma_r_rotational,
																				csm.K_hat_r_rot);

	cim.Theta_hat_rot_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_rot,
																				  cim.dead_zone_value_rotational,
																				  cim.augmented_inner_loop_regressor,
																				  e_transpose_p_b,
																				  cip.sigma_Theta_rotational,
																				  csm.Theta_hat_rot);	

    // Projection operator - Ball
    // Projection operator K_hat_x
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_x_rot)> proj_op_output_K_hat_x_rotational = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_x_rot,
                                                                    cim.K_hat_x_rot_dot,
                                                                    cip.projection_x_max_x_rotational,
                                                                    cip.projection_epsilon_x_rotational);

    cim.K_hat_x_rot_dot = proj_op_output_K_hat_x_rotational.projected_matrix;
    cim.proj_op_activated_K_hat_x_rotational = proj_op_output_K_hat_x_rotational.projection_operator_activated;

    // Projection operator K_hat_r
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_r_rot)> proj_op_output_K_hat_r_rotational = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_r_rot,
                                                                    cim.K_hat_r_rot_dot,
                                                                    cip.projection_x_max_r_rotational,
                                                                    cip.projection_epsilon_r_rotational);
                                
    cim.K_hat_r_rot_dot = proj_op_output_K_hat_r_rotational.projected_matrix;
    cim.proj_op_activated_K_hat_r_rotational = proj_op_output_K_hat_r_rotational.projection_operator_activated;

    // Projection operator Theta_hat
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.Theta_hat_rot)> proj_op_output_Theta_hat_rotational = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.Theta_hat_rot,
                                                                    cim.Theta_hat_rot_dot,
                                                                    cip.projection_x_max_Theta_rotational,
                                                                    cip.projection_epsilon_Theta_rotational);

    cim.Theta_hat_rot_dot = proj_op_output_Theta_hat_rotational.projected_matrix;
    cim.proj_op_activated_Theta_hat_rotational = proj_op_output_Theta_hat_rotational.projection_operator_activated;

    // Adaptive control law
	cim.tau_rot_adaptive << csm.K_hat_x_rot.transpose() * cim.omega
						  + csm.K_hat_r_rot.transpose() * cim.omega_cmd
						  - csm.Theta_hat_rot.transpose() * cim.augmented_inner_loop_regressor;

	// Total rotational control input
	cim.tau_rot << cim.tau_rot_baseline + cim.tau_rot_adaptive;	

    // Assing the control inputs
    cim.u(1) = cim.tau_rot(0);
    cim.u(2) = cim.tau_rot(1);
    cim.u(3) = cim.tau_rot(2);
}

// Function to compute the normalized thrusts
void mrac_geometric::compute_normalized_thrusts()
{
    // Compute the individual thrusts in Newtons
    cim.Thrust << mixer_matrix_tailsitter * cim.u;

    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs
    control_input(0) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_tailsitter, cim.Sat_Thrust(0));
    control_input(1) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_tailsitter, cim.Sat_Thrust(1));
    control_input(2) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_tailsitter, cim.Sat_Thrust(2));
    control_input(3) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_tailsitter, cim.Sat_Thrust(3));

    // std::cout << "T1: " << control_input(0) 
    //           << "| T2: " << control_input(1)
    //           << "| T3: " << control_input(2)
    //           << "| T4: " << control_input(3) 
    //           << std::endl;
}

// Function that is called in sim-bridge.cpp
void mrac_geometric::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Compute the aerodynamics

    // 2. Compute the translational control input
    compute_translational_control_in_I();

    // 3. Compute the rate of change of the translational control input
    compute_translational_control_rate();

    // 4. Compute the thrust needed and the desired rotaion matrix
    compute_u1_R_d();

    // 5. Compute the rotational control input
    compute_rotational_control();

    // 6. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 7. Do the integration
    rk4.do_step(boost::bind(&mrac_geometric::model, this, bph::_1, bph::_2, bph::_3),
                y, cim.t, time_step_rk4_);
    
    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration of the execution 
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 8. Log the Data after all the calculataions
    this->LogData();
    
}

// Function that is called during the constructor. 
bool mrac_geometric::InitiateLogging()
{
    auto status = _logger_::_filesystem_::setupControllerLogging(this->m_logger, "tailsitter" ,"MRAC_GEOMETRIC");
    return status;
}

// Funciton that setups up the headers for the log file
void mrac_geometric::ConfigureHeaders()
{

    // Create the oss object
    std::ostringstream oss;

    oss << ", "
        << "Controller Time [s], "
        << "Algorithm Execution Time [us], "
        << "Position x [m], "
        << "Position y [m], "
        << "Position z [m], "
        << "Velocity x [m/s], "
        << "Velocity y [m/s], "
        << "Velocity z [m/s], "
		<< "Reference Position x [m], "
        << "Reference Position y [m], "
        << "Reference Position z [m], "
        << "Reference Velocity x [m/s], "
        << "Reference Velocity y [m/s], "
        << "Reference Velocity z [m/s], "
		<< "Reference Acceleration x [m/s^2], "
        << "Reference Acceleration y [m/s^2], "
        << "Reference Acceleration z [m/s^2], "
        << "User Position x [m], "
        << "User Position y [m], "
        << "User Position z [m], "
        << "User Velocity x [m/s], "
        << "User Velocity y [m/s], "
        << "User Velocity z [m/s], "
        << "User Acceleration x [m/s^2], "
        << "User Acceleration y [m/s^2], "
        << "User Acceleration z [m/s^2], "
		<< "Error in x [m], "
        << "Error in y [m], "
        << "Error in z [m], "
        << "Error in vx [m/s], "
        << "Error in vy [m/s], "
        << "Error in vz [m/s], "
        << "Integral Error in x [m], "
        << "Integral Error in y [m], "
        << "Integral Error in z [m], "
        << "Reference Error in x [m], "
        << "Reference Error in y [m], "
        << "Reference Error in z [m], "
        << "Integral Reference Error in x [m], "
        << "Integral Reference Error in y [m], "
        << "Integral Reference Error in z [m], "
        << "Reference Command in x [-], "
        << "Reference Command in y [-], "
        << "Reference Command in z [-], "
        << "mu_baseline x [N], " 
        << "mu_baseline y [N], " 
        << "mu_baseline z [N], " 
        << "mu_adaptive x [N], "
        << "mu_adaptive y [N], "
        << "mu_adaptive z [N], "
        << "mu_I x [N], "
        << "mu_I y [N], "
        << "mu_I z [N], "
        << "Desired w_x [rad/s], "
        << "Desired w_y [rad/s], "
        << "Desired w_z [rad/s], "
        << "Desired w_x_dot [rad/s^2], "
        << "Desired w_y_dot [rad/s^2], "
        << "Desired w_z_dot [rad/s^2], "
        << "w_x [rad/s], "
        << "w_y [rad/s], "
        << "w_z [rad/s], "
        << "Command w_x [rad/s], "
        << "Command w_y [rad/s], "
        << "Command w_z [rad/s], "
        << "Reference w_x [rad/s], "
        << "Reference w_y [rad/s], "
        << "Reference w_z [rad/s], "
        << "Reference alpha_x [rad/s^2], "
        << "Reference alpha_y [rad/s^2], "
        << "Reference alpha_z [rad/s^2], "
        << "Reference error in w_x [rad/s], "
        << "Reference error in w_y [rad/s], "
        << "Reference error in w_z [rad/s], "
        << "Xi_e x [-], "
        << "Xi_e y [-], "
        << "Xi_e z [-], "
        << "Error in w_x [rad/s], "
        << "Error in w_y [rad/s], "
        << "Error in w_z [rad/s], "
        << "Integral reference error in w_x [rad/s], "
        << "Integral reference error in w_y [rad/s], "
        << "Integral reference error in w_z [rad/s], "
        << "Tau x baseline [Nm], "
        << "Tau y baseline [Nm], "
        << "Tau z baseline [Nm], "
        << "Tau x adaptive [Nm], "
        << "Tau y adaptive [Nm], "
        << "Tau z adaptive [Nm], "
        << "Control input 1 [N], "
        << "Control input 2 [N], "
        << "Control input 3 [N], "
        << "Control input 4 [N], "
        << "Thrust Motor 1 [N], "
        << "Thrust Motor 2 [N], "
        << "Thrust Motor 3 [N], "
        << "Thrust Motor 4 [N], "
        << "Normalized Thrust 1 [-], "
        << "Normalized Thrust 2 [-], "
        << "Normalized Thrust 3 [-], "
        << "Normalized Thrust 4 [-], "
        << "dead_zone_value_translational [-], "
        << "dead_zone_value_rotational [-], "
        << "proj_op_activated_K_hat_x_translational [-], "
        << "proj_op_activated_K_hat_r_translational [-], "
        << "proj_op_activated_Theta_hat_translational [-], "
        << "proj_op_activated_K_hat_x_rotational [-], "
        << "proj_op_activated_K_hat_r_rotational [-], "
        << "proj_op_activated_Theta_hat_rotational [-], "
        ;

        // Use the helper function to create the header for the matrix data
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "R_d", cim.R_d, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "R_d_dot", cim.R_d_dot, "[-]" );
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "R_ji", cim.Rji, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "K_hat_x_translational", csm.K_hat_x_tran, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "K_hat_r_translational", csm.K_hat_r_tran, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "Theta_hat_translational", csm.Theta_hat_tran, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "K_hat_x_rotational", csm.K_hat_x_rot, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "K_hat_r_rotational", csm.K_hat_r_rot, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "Theta_hat_rotational", csm.Theta_hat_rot, "[-]");    

        oss << "mu_dot_x Filter [N/s], "
            << "mu_dot_y Filter [N/s], "
            << "mu_dot_z Filter [N/s], "
            << "mu_est_x Differentiator [N], "
            << "mu_est_y Differentiator [N], "
            << "mu_est_z Differentiator [N], "
            << "mu_dot_est_x Differentiator [N], "
            << "mu_dot_est_y Differentiator [N], "
            << "mu_dot_est_z Differentiator [N], "
            ;         

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();

        _message_::SIMULATOR_INFO("[SIMCTL]: WROTE MRAC GEOMETRIC LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE MRAC GEOMETRIC LOG HEADER DATA", e.what());
    }

}

void mrac_geometric::LogData()
{
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << cim.t << ", "
        << cim.alg_duration << ", "
        << cim.x_tran(0) << ", "
        << cim.x_tran(1) << ", "
        << cim.x_tran(2) << ", "
        << cim.x_tran(3) << ", "
        << cim.x_tran(4) << ", "
        << cim.x_tran(5) << ", "
        << csm.x_tran_ref(0) << ", "
        << csm.x_tran_ref(1) << ", "
        << csm.x_tran_ref(2) << ", "
        << csm.x_tran_ref(3) << ", "
        << csm.x_tran_ref(4) << ", "
        << csm.x_tran_ref(5) << ", "
        << cim.x_tran_ref_dot(3) << ", "
        << cim.x_tran_ref_dot(4) << ", "
        << cim.x_tran_ref_dot(5) << ", "
        << cim.r_user(0) << ", "
        << cim.r_user(1) << ", "
        << cim.r_user(2) << ", "
        << cim.r_dot_user(0) << ", "
        << cim.r_dot_user(1) << ", "
        << cim.r_dot_user(2) << ", "
        << cim.r_ddot_user(0) << ", "
        << cim.r_ddot_user(1) << ", "
        << cim.r_ddot_user(2) << ", "
        << cim.e_tran(0) << ", "
        << cim.e_tran(1) << ", "
        << cim.e_tran(2) << ", "
        << cim.e_tran(3) << ", "
        << cim.e_tran(4) << ", "
        << cim.e_tran(5) << ", "
        << csm.e_tran_pos_I(0) << ", "
        << csm.e_tran_pos_I(1) << ", "
        << csm.e_tran_pos_I(2) << ", "
        << cim.e_tran_pos_ref(0) << ", "
        << cim.e_tran_pos_ref(1) << ", "
        << cim.e_tran_pos_ref(2) << ", "
        << csm.e_tran_pos_ref_I(0) << ", "
        << csm.e_tran_pos_ref_I(1) << ", "
        << csm.e_tran_pos_ref_I(2) << ", "
        << cim.r_cmd_tran(0) << ", "
        << cim.r_cmd_tran(1) << ", "
        << cim.r_cmd_tran(2) << ", "
        << cim.mu_tran_baseline(0) << ", "
        << cim.mu_tran_baseline(1) << ", "
        << cim.mu_tran_baseline(2) << ", "
        << cim.mu_tran_adaptive(0) << ", "
        << cim.mu_tran_adaptive(1) << ", "
        << cim.mu_tran_adaptive(2) << ", "
        << cim.mu_tran_I(0) << ", "
        << cim.mu_tran_I(1) << ", "
        << cim.mu_tran_I(2) << ", "
        << cim.omega_d(0) << ", "
        << cim.omega_d(1) << ", "
        << cim.omega_d(2) << ", "
        << cim.alpha_d(0) << ", "
        << cim.alpha_d(1) << ", "
        << cim.alpha_d(2) << ", "
        << cim.omega(0) << ", "
        << cim.omega(1) << ", "
        << cim.omega(2) << ", "
        << cim.omega_cmd(0) << ", "
        << cim.omega_cmd(1) << ", "
        << cim.omega_cmd(2) << ", "
        << csm.omega_ref(0) << ", "
        << csm.omega_ref(1) << ", "
        << csm.omega_ref(2) << ", "
        << cim.omega_ref_dot(0) << ", "
        << cim.omega_ref_dot(1) << ", "
        << cim.omega_ref_dot(2) << ", "
        << cim.e_omega_ref(0) << ", "
        << cim.e_omega_ref(1) << ", "
        << cim.e_omega_ref(2) << ", "
        << cim.Xi_e(0) << ", "
        << cim.Xi_e(1) << ", "
        << cim.Xi_e(2) << ", "
        << cim.omega_e(0) << ", "
        << cim.omega_e(1) << ", "
        << cim.omega_e(2) << ", "
        << csm.e_omega_ref_I(0) << ", "
        << csm.e_omega_ref_I(1) << ", "
        << csm.e_omega_ref_I(2) << ", "
        << cim.tau_rot_baseline(0) << ", "
        << cim.tau_rot_baseline(1) << ", "
        << cim.tau_rot_baseline(2) << ", "
        << cim.tau_rot_adaptive(0) << ", "
        << cim.tau_rot_adaptive(1) << ", "
        << cim.tau_rot_adaptive(2) << ", "
        << cim.u(0) << ", "
        << cim.u(1) << ", "
        << cim.u(2) << ", "
        << cim.u(3) << ", "
        << cim.Sat_Thrust(0) << ", "
        << cim.Sat_Thrust(1) << ", "
        << cim.Sat_Thrust(2) << ", "
        << cim.Sat_Thrust(3) << ", "
        << this->control_input(0) << ", "
        << this->control_input(1) << ", "
        << this->control_input(2) << ", "
        << this->control_input(3) << ", "
        << cim.dead_zone_value_translational << ", "
        << cim.dead_zone_value_rotational << ", "
        << cim.proj_op_activated_K_hat_x_translational << ", "
        << cim.proj_op_activated_K_hat_r_translational << ", "
        << cim.proj_op_activated_Theta_hat_translational << ", "
        << cim.proj_op_activated_K_hat_x_rotational << ", "
        << cim.proj_op_activated_K_hat_r_rotational << ", "
        << cim.proj_op_activated_Theta_hat_rotational << ", "
        ;

        // User helper functions to output the matrix data
        ::_shared_::_serialize_::appendEigenData(oss, cim.R_d);
        ::_shared_::_serialize_::appendEigenData(oss, cim.R_d_dot);
        ::_shared_::_serialize_::appendEigenData(oss, cim.Rji);
        ::_shared_::_serialize_::appendEigenData(oss, csm.K_hat_x_tran);
        ::_shared_::_serialize_::appendEigenData(oss, csm.K_hat_r_tran);
        ::_shared_::_serialize_::appendEigenData(oss, csm.Theta_hat_tran);
        ::_shared_::_serialize_::appendEigenData(oss, csm.K_hat_x_rot);
        ::_shared_::_serialize_::appendEigenData(oss, csm.K_hat_r_rot);
        ::_shared_::_serialize_::appendEigenData(oss, csm.Theta_hat_rot);

        oss << cip.C_filter_mu * csm.state_mu_x_filter << ", "
            << cip.C_filter_mu * csm.state_mu_y_filter << ", "
            << cip.C_filter_mu * csm.state_mu_z_filter << ", "
            << dim.x_hat_vs_2l_mrad_dot(0) << ", "
            << dim.x_hat_vs_2l_mrad_dot(1) << ", "
            << dim.x_hat_vs_2l_mrad_dot(2) << ", "
            << dim.x_hat_vs_2l_mrad_dot(3) << ", "
            << dim.x_hat_vs_2l_mrad_dot(4) << ", "
            << dim.x_hat_vs_2l_mrad_dot(5) << ", "
            ;


    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE MRAC GEOMETRIC LOG DATA", e.what());
    }
}


}   // namespace _mrac_geometric_

}   // namespace _tailsitter_
    
}   // namespace _acsl_
