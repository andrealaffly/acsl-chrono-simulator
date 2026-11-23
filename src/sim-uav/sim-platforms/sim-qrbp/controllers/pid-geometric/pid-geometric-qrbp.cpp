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
 * File:        pid-geometric-qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        March 25, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID with geometric and angular velocities for the QRBP. Inherts the class controller_base for the basic 
 *              functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "pid-geometric-qrbp.hpp"

namespace _acsl_
{

namespace _qrbp_
{

namespace _pid_geometric_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
pid_geometric::pid_geometric(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                            : ::_acsl_::_control_::controller_base(logger, trajectory)
{
    // Initial Conditions
    init();   
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void pid_geometric::read_params(const std::string& jsonFile)
{
    // Implementation here
    std::ifstream file(jsonFile);
    nlohmann::json j;
    file >> j;

    // Controller matrices ----------------------------------------------------------------------------------------------
    // Translational parameters
    cip.Kp_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_translational"], 3, 3);
    cip.Ki_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KI_translational"], 3, 3);
    cip.Kd_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_translational"], 3, 3);

    // Rotational parameters
    cip.Kp_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_rotational"], 3, 3);
    cip.Kd_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_rotational"], 3, 3);
    cip.Ka_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["Ka_rotational"], 3, 1);

    // Differentiator matrices
    cip.A_filter_mu = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["A_filter_mu"], 2, 2);
    cip.B_filter_mu = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["B_filter_mu"], 2, 1);
    cip.C_filter_mu = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["C_filter_mu"], 1, 2);
    cip.A_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["A_filter_omega_d"], 2, 2);
    cip.B_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["B_filter_omega_d"], 2, 1);
    cip.C_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["C_filter_omega_d"], 1, 2);

}

// Implementing virutal functions from controller_base
void pid_geometric::init(){
    // Reading in the parameters
    read_params("../chrono-assets/parameters/qrbp/PID_GEOMETRIC/gains_PID_GEOMETRIC.json");

    // Set the inital conditions
    y.fill(0.0);
    dy.fill(0.0);

    // Setup the logging
    InitiateLogging();
    ConfigureHeaders();
}

// Update function for the controller
void pid_geometric::update(double time, 
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
}

// Function to assign elements from the rk4 integrator
void pid_geometric::assign_from_rk4()
{
    int index = 0;

    //------------ assign after integration  ------------//
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_mu_x_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_mu_y_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_mu_z_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_z_d_filter, y, index);
}

// Model function for integration
void pid_geometric::model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t)
{
    int index = 0;
    
    //------------ Fill up the dy for integration  ------------//
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_mu_x_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_mu_y_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_mu_z_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);
}


// Function to compute the outerloop control in I
void pid_geometric::compute_translational_control_in_I()
{
    // Compute the error in the states
    cim.e_tran_pos << cim.x_tran_pos - cim.r_user;
    cim.e_tran_vel << cim.x_tran_vel - cim.r_dot_user;

    // Compute the baseline control input
    cim.mu_tran_baseline << MASS*( - cip.Kp_tran * cim.e_tran_pos 
                                   - cip.Kd_tran * cim.e_tran_vel
                                   - cip.Ki_tran * csm.e_tran_pos_I
                                   + cim.r_ddot_user );

    // Compute with the dynamic inversion without aerodynamics
    cim.mu_tran_I << cim.mu_tran_baseline - MASS * G * e3_basis;
}

// Function to compute the outerloop translational control rate in I using the differentiator
void pid_geometric::compute_translational_control_rate()
{
    // Compute the internal state for rate of change of mu
    cim.internal_state_mu_x_filter << cip.A_filter_mu * csm.state_mu_x_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(0);

    cim.internal_state_mu_y_filter << cip.A_filter_mu * csm.state_mu_y_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(1);

    cim.internal_state_mu_z_filter << cip.A_filter_mu * csm.state_mu_z_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(2);

    // Compute the rate of change of the translational virtual control input
    cim.mu_tran_I_dot(0) = cip.C_filter_mu * csm.state_mu_x_filter;
    cim.mu_tran_I_dot(1) = cip.C_filter_mu * csm.state_mu_y_filter;
    cim.mu_tran_I_dot(2) = cip.C_filter_mu * csm.state_mu_z_filter;
}

// Compute the orientation commands and the desired total thrust
void pid_geometric::compute_u1_R_d()
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
    cim.alpha_d(0) = cip.C_filter_omega_d * csm.state_omega_x_d_filter;
    cim.alpha_d(1) = cip.C_filter_omega_d * csm.state_omega_y_d_filter;
    cim.alpha_d(2) = cip.C_filter_omega_d * csm.state_omega_z_d_filter;
}

// Compute the rotational control
void pid_geometric::compute_rotational_control()
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
    cim.omega_e << cim.omega - cim.omega_d;

    // Compute the baseline control input
    cim.tau_rot_baseline << inertia_matrix_q * ( - cip.Kp_att * cim.Xi_e
                                                 - cip.Kd_att * cim.omega_e);

    // Cache the feedforward term
    Eigen::Vector3d fft;
    fft = inertia_matrix_q * ( cim.omega.cross(inertia_matrix_q * cim.omega) - cim.alpha_d );

    // Compute with dynamic inversion
    cim.tau_rot << cim.tau_rot_baseline + fft;

    // Assing the control inputs
    cim.u(1) = cim.tau_rot(0);
    cim.u(2) = cim.tau_rot(1);
    cim.u(3) = cim.tau_rot(2);
}

// Function to compute the normalized thrusts
void pid_geometric::compute_normalized_thrusts()
{
    // Compute the individual thrusts in Newtons
    cim.Thrust << mixer_matrix_qrbp * cim.u;

    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs
    control_input(0) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(0));
    control_input(1) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(1));
    control_input(2) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(2));
    control_input(3) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(3));
}

// Function that is called in sim-bridge.cpp
void pid_geometric::run(const double time_step_rk4_) {

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
    rk4.do_step(boost::bind(&pid_geometric::model, this, bph::_1, bph::_2, bph::_3),
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
bool pid_geometric::InitiateLogging()
{
    auto status = _logger_::_filesystem_::setupControllerLogging(this->m_logger, "qrbp" ,"PID_GEOMETRIC");
    return status;
}

// Funciton that setups up the headers for the log file
void pid_geometric::ConfigureHeaders()
{

    // Create the oss object
    std::ostringstream oss;

    oss << ", "
        << "Controller Time [s], "
        << "Algorithm Execution Time [us], "
        << "user x [m], "
        << "user y [m], "
        << "user z [m], "
        << "user vx [m/s], "
        << "user vy [m/s], "
        << "user vz [m/s], "
        << "user ax [m/s2], "
        << "user ay [m/s2], "
        << "user az [m/s2], "
        << "user psi [rad], "
        << "x [m], "
        << "y [m], "
        << "z [m], "
        << "vx [m/s], "
        << "vy [m/s], "
        << "vz [m/s], "
        << "omega x [rad/s], "
        << "omega y [rad/s], "
        << "omega z [rad/s], "
        << "Error in x [m], "
        << "Error in y [m], "
        << "Error in z [m], "
        << "Error in vx [m/s], "
        << "Error in vy [m/s], "
        << "Error in vz [m/s], "
        << "e_tran_pos_I x [-], "
        << "e_tran_pos_I y [-], "
        << "e_tran_pos_I z [-], "
        << "mu_tran_baseline x [N], "
        << "mu_tran_baseline y [N], "
        << "mu_tran_baseline z [N], "
        << "mu_tran_I x [N], "
        << "mu_tran_I y [N], "
        << "mu_tran_I z [N], "
        << "mu_tran_I_dot x [N/s], "
        << "mu_tran_I_dot y [N/s], "
        << "mu_tran_I_dot z [N/s], "
        << "omega_d x [rad/s], "
        << "omega_d y [rad/s], "
        << "omega_d z [rad/s], "
        << "alpha_d x [rad/s2], "
        << "alpha_d y [rad/s2], "
        << "alpha_d z [rad/s2], "
        << "Xi_e x [-], "
        << "Xi_e y [-], "
        << "Xi_e z [-], "
        << "omega_e x [-], "
        << "omega_e y [-], "
        << "omega_e z [-], "
        << "tau_rot_baseline x [-], "
        << "tau_rot_baseline y [-], "
        << "tau_rot_baseline z [-], "
        << "tau_rot x [-], "
        << "tau_rot y [-], "
        << "tau_rot z [-], "
        << "u1 [N], "
        << "u2 [Nm], "
        << "u3 [Nm], "
        << "u4 [Nm], "
        << "T1 [N], "
        << "T2 [N], "
        << "T3 [N], "
        << "T4 [N], "
        << "Normalized Thrust 1 [-], "
        << "Normalized Thrust 2 [-], "
        << "Normalized Thrust 3 [-], "
        << "Normalized Thrust 4 [-], "
        ;

        // Use the helper function to create the header for the matrix data
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "R_d", cim.R_d, "[-]");
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "R_d_dot", cim.R_d_dot, "[-]" );
        ::_shared_::_serialize_::generateMatrixHeaders(oss, "R_ji", cim.Rji, "[-]");


    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();

        _message_::SIMULATOR_INFO("[SIMCTL]: WROTE PID GEOMETRIC LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID GEOMETRIC LOG HEADER DATA", e.what());
    }

}

void pid_geometric::LogData()
{
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << cim.t << ", "
        << cim.alg_duration << ", "
        << cim.r_user(0) << ", "
        << cim.r_user(1) << ", "
        << cim.r_user(2) << ", "
        << cim.r_dot_user(0) << ", "
        << cim.r_dot_user(1) << ", "
        << cim.r_dot_user(2) << ", "
        << cim.r_ddot_user(0) << ", "
        << cim.r_ddot_user(1) << ", "
        << cim.r_ddot_user(2) << ", "
        << cim.psi_user << ", "
        << cim.x_tran_pos(0) << ", "
        << cim.x_tran_pos(1) << ", "
        << cim.x_tran_pos(2) << ", "
        << cim.x_tran_vel(0) << ", "
        << cim.x_tran_vel(1) << ", "
        << cim.x_tran_vel(2) << ", "
        << cim.omega(0) << ", "
        << cim.omega(1) << ", "
        << cim.omega(2) << ", "
        << cim.e_tran_pos(0) << ", "
        << cim.e_tran_pos(1) << ", "
        << cim.e_tran_pos(2) << ", "
        << cim.e_tran_vel(0) << ", "
        << cim.e_tran_vel(1) << ", "
        << cim.e_tran_vel(2) << ", "
        << csm.e_tran_pos_I(0) << ", "
        << csm.e_tran_pos_I(1) << ", "
        << csm.e_tran_pos_I(2) << ", "
        << cim.mu_tran_baseline(0) << ", "
        << cim.mu_tran_baseline(1) << ", "
        << cim.mu_tran_baseline(2) << ", "
        << cim.mu_tran_I(0) << ", "
        << cim.mu_tran_I(1) << ", "
        << cim.mu_tran_I(2) << ", "
        << cim.mu_tran_I_dot(0) << ", "
        << cim.mu_tran_I_dot(1) << ", "
        << cim.mu_tran_I_dot(2) << ", "
        << cim.omega_d(0) << ", "
        << cim.omega_d(1) << ", "
        << cim.omega_d(2) << ", "
        << cim.alpha_d(0) << ", "
        << cim.alpha_d(1) << ", "
        << cim.alpha_d(2) << ", "
        << cim.Xi_e(0) << ", "
        << cim.Xi_e(1) << ", "
        << cim.Xi_e(2) << ", "
        << cim.omega_e(0) << ", "
        << cim.omega_e(1) << ", "
        << cim.omega_e(2) << ", "
        << cim.tau_rot_baseline(0) << ", "
        << cim.tau_rot_baseline(1) << ", "
        << cim.tau_rot_baseline(2) << ", "
        << cim.tau_rot(0) << ", "
        << cim.tau_rot(1) << ", "
        << cim.tau_rot(2) << ", "
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
        ;

        // User helper functions to output the matrix data
        ::_shared_::_serialize_::appendEigenData(oss, cim.R_d);
        ::_shared_::_serialize_::appendEigenData(oss, cim.R_d_dot);
        ::_shared_::_serialize_::appendEigenData(oss, cim.Rji);

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID GEOMETRIC LOG DATA", e.what());
    }
}


}   // namespace _pid_geometric_

}   // namespace _qrbp_
    
}   // namespace _acsl_
