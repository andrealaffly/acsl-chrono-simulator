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
 * File:        pid-omega-qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID  with angular velocities for the QRBP. Inherts the class controller_base for the basic 
 *              functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "pid-omega-qrbp.hpp"

namespace _acsl_
{

namespace _qrbp_
{

namespace _pid_omega_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
pid_omega::pid_omega(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                    : ::_acsl_::_control_::controller_base(logger, trajectory)
{
    // Initial Conditions
    init();   
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void pid_omega::read_params(const std::string& jsonFile)
{
    // Implementation here
    std::ifstream file(jsonFile);
    nlohmann::json j;
    file >> j;

    // Translational parameters
    cip.Kp_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_translational"], 3, 3);
    cip.Ki_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KI_translational"], 3, 3);
    cip.Kd_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_translational"], 3, 3);

    // Rotational parameters
    cip.Kp_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_rotational"], 3, 3);
    cip.Ki_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KI_rotational"], 3, 3);
    cip.Kd_rot = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_rotational"], 3, 3);

}

// Implementing virutal functions from controller_base
void pid_omega::init(){
    // Reading in the parameters
    read_params("../chrono-assets/parameters/qrbp/PID_OMEGA/gains_PID_OMEGA.json");

    // Set the inital conditions
    y.fill(0.0);
    dy.fill(0.0);

    // Setup the logging
    InitiateLogging();
    ConfigureHeaders();
}

// Update function for the controller
void pid_omega::update(double time, 
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

    // Assign the Rotational States
    cim.eta_rot << roll, pitch, yaw;
    cim.omega_rot << w_x, w_y, w_z;

    // Assign the quaternions
    cim.q.w() = q0;
    cim.q.z() = q1;
    cim.q.y() = q2;
    cim.q.z() = q3;

    // 2. Get the reference trajectory ---------------------------------------------
    m_traj.UpdateModule(time);
    cim.r_user = m_traj.GetPosition();
    cim.r_dot_user = m_traj.GetVelocity();
    cim.r_ddot_user = m_traj.GetAcceleration();   

    // Set the desired yaw and yaw_rate; we don't need the acceleration as 
    // we are going to compute it from the filter.
    cim.eta_rot_d(2) = m_traj.GetEulerAngle()(2);
    cim.eta_rot_rate_d(2) = m_traj.GetEulerRate()(2);

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Compute the rotation matrices --------------------------------------------
    cim.R_I_J = _shared_::_transformations_::rotationMatrix321GlobalToLocal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));
    cim.R_J_I = _shared_::_transformations_::rotationMatrix321LocalToGlobal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));

    // 5. Compute the body velocities in the quadcopter frame from the inertial velocities
    cim.x_tran_vel_J = cim.R_I_J * cim.x_tran_vel;

    // 6. Assign the values from the integrator ------------------------------------
    assign_from_rk4();
}

// Function to assign elements from the rk4 integrator
void pid_omega::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_roll_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_pitch_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_z_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_rot_eta_I, y, index);
}

// Model function for integration
void pid_omega::model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t)
{
    //------------ Fill up the dy for integration  ------------//
    int index = 0;
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_roll_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_pitch_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_rot_eta, dy, index);
}


// Function to compute the outerloop control in I
void pid_omega::compute_translational_control_in_I()
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

// Compute the orientation commands and the desired total thrust
void pid_omega::compute_u1_eta_d()
{
	// Compute the virtual forces in J
	cim.mu_tran_J << cim.R_I_J * cim.mu_tran_I;

	// Compute u1 - Total Thrust
	cim.u(0) = sqrt(  pow(cim.mu_tran_J(0), 2)
							 		+ pow(cim.mu_tran_J(1), 2)
								 	+ pow(cim.mu_tran_J(2), 2));

	// Compute the desired roll
	double r1 = cim.mu_tran_J(1) / cim.u(0);
	cim.eta_rot_d(0) = atan2(r1, sqrt(1 - pow(r1,2)));

	// Compute the desired pitch
	cim.eta_rot_d(1) = atan2( -cim.mu_tran_J(0), -cim.mu_tran_J(2) );

	// Compute the internal state for angular rates
	cim.internal_state_roll_d_filter << A_filter_roll_ref * csm.state_roll_d_filter
                                      + B_filter_roll_ref * cim.eta_rot_d(0);

	cim.internal_state_pitch_d_filter << A_filter_pitch_ref * csm.state_pitch_d_filter
                                       + B_filter_pitch_ref * cim.eta_rot_d(1);


	// Compute the desired roll and pitch rates
	cim.eta_rot_rate_d(0) = C_filter_roll_ref * csm.state_roll_d_filter;    
	cim.eta_rot_rate_d(1) = C_filter_pitch_ref * csm.state_pitch_d_filter;	

	// Compute the Jacobian with the current orientation
	cim.Jacobian = ::_shared_::_transformations_::jacobianMatrix(cim.eta_rot(0), cim.eta_rot(1));

	// Compute the desired angular velocities
	cim.omega_rot_d << cim.Jacobian * cim.eta_rot_rate_d;
	
	// Compute the internal state for angular accelration
	cim.internal_state_omega_x_d_filter << A_filter_roll_dot_ref * csm.state_omega_x_d_filter
																				+ B_filter_roll_dot_ref * cim.omega_rot_d(0);

	cim.internal_state_omega_y_d_filter << A_filter_pitch_dot_ref * csm.state_omega_y_d_filter
																				+ B_filter_pitch_dot_ref * cim.omega_rot_d(1);

	cim.internal_state_omega_z_d_filter << A_filter_yaw_dot_ref * csm.state_omega_z_d_filter
																				+ B_filter_yaw_dot_ref * cim.omega_rot_d(2);

	// Compute the desired angular acceleration
	cim.alpha_rot_d(0) = C_filter_roll_dot_ref * csm.state_omega_x_d_filter;
	cim.alpha_rot_d(1) = C_filter_pitch_dot_ref * csm.state_omega_y_d_filter;
	cim.alpha_rot_d(2) = C_filter_yaw_dot_ref * csm.state_omega_z_d_filter;	
}


// Compute the rotational control
void pid_omega::compute_rotational_control()
{
    // Compute the error in the rotational states
    cim.e_rot_eta(0) = cim.eta_rot(0) - cim.eta_rot_d(0);
    cim.e_rot_eta(1) = cim.eta_rot(1) - cim.eta_rot_d(1);
    cim.e_rot_eta(2) = ::_shared_::_compute_::makeYawAngularErrorContinuous(cim.eta_rot(2), cim.eta_rot_d(2)); 

    cim.e_rot_omega << cim.omega_rot - cim.omega_rot_d;

    // Compute the baseline control input
    cim.tau_rot_baseline << inertia_matrix_q * ( - cip.Kp_rot * cim.e_rot_eta 
                                                 - cip.Kd_rot * cim.e_rot_omega
                                                 - cip.Ki_rot * csm.e_rot_eta_I
                                                 + cim.alpha_rot_d);

    // Compute with dynamic inversion without aerodynamics
    cim.tau_rot << cim.tau_rot_baseline 
                   + cim.omega_rot.cross(inertia_matrix_q * cim.omega_rot);

    // Assign the control inputs
    cim.u(1) = cim.tau_rot(0);
    cim.u(2) = cim.tau_rot(1);
    cim.u(3) = cim.tau_rot(2);                                             
}

// Function to compute the normalized thrusts
void pid_omega::compute_normalized_thrusts()
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
void pid_omega::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Compute the aerodynamics

    // 2. Compute the translational control input
    compute_translational_control_in_I();

    // 3. Compute the thrust needed and the desired angles
    compute_u1_eta_d();

    // 4. Compute the rotational control input
    compute_rotational_control();

    // 5. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 6. Do the integration
    rk4.do_step(boost::bind(&pid_omega::model, this, bph::_1, bph::_2, bph::_3),
                y, cim.t, time_step_rk4_);
    
    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration of the execution 
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 7. Log the Data after all the calculataions
    this->LogData();
    
}

// Function that is called during the constructor. 
bool pid_omega::InitiateLogging()
{
    auto status = _logger_::_filesystem_::setupControllerLogging(this->m_logger, "qrbp" ,"PID_OMEGA");
    return status;
}

// Funciton that setups up the headers for the log file
void pid_omega::ConfigureHeaders()
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
        << "Phi [rad], "
        << "Theta [rad], "
        << "Psi [rad], "
        << "omega_x [rad/s], "
        << "omega_y [rad/s], "
        << "omega_z [rad/s], "
        << "User Position x [m], "
        << "User Position y [m], "
        << "User Position z [m], "
        << "User Velocity x [m], "
        << "User Velocity y [m], "
        << "User Velocity z [m], "
        << "Desired Phi [rad], "
        << "Desired Theta [rad], "
        << "Desired Psi [rad], "
        << "Deisred Phi rate [rad/s], "
        << "Deisred Theta rate [rad/s], "
        << "Deisred Psi rate [rad/s], "
        << "Desired Omega_x [rad/s], "
        << "Desired Omega_y [rad/s], "
        << "Desired Omega_z [rad/s], "
        << "Desired Alpha_x [rad/s^2], "
        << "Desired Alpha_y [rad/s^2], "
        << "Desired Alpha_z [rad/s^2], "
        << "Error in x [m], "
        << "Error in y [m], "
        << "Error in z [m], "
        << "Integral Error in x [m], "
        << "Integral Error in y [m], "
        << "Integral Error in z [m], "
        << "Error in vx [m/s], "
        << "Error in vy [m/s], "
        << "Error in vz [m/s], "
        << "Error in Phi [rad], "
        << "Error in Theta [rad], "
        << "Error in Psi [rad], "
        << "Integral Error in Phi [rad], "
        << "Integral Error in Theta [rad], "
        << "Integral Error in Psi [rad], "
        << "Error in omega_x [rad/s], "
        << "Error in omega_y [rad/s], "
        << "Error in omega_z [rad/s], "
        << "Virtual Mu x I [N], "
        << "Virtual Mu y I [N], "
        << "Virtual Mu z I [N], "
        << "Virtual Mu x J [N], "
        << "Virtual Mu y J [N], "
        << "Virtual Mu z J [N], "
        << "u1 [N], "
        << "u2 [Nm], "
        << "u3 [Nm], "
        << "u4 [Nm], "
        << "Thrust Motor 1 [N], "
        << "Thrust Motor 2 [N], "
        << "Thrust Motor 3 [N], "
        << "Thrust Motor 4 [N], "
        << "Normalized Thrust 1 [-], "
        << "Normalized Thrust 2 [-], "
        << "Normalized Thrust 3 [-], "
        << "Normalized Thrust 4 [-], "
        ;    

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();

        _message_::SIMULATOR_INFO("[SIMCTL]: WROTE PID OMEGA LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID OMEGA LOG HEADER DATA", e.what());
    }

}

void pid_omega::LogData()
{
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << cim.t << ", "
        << cim.alg_duration << ", "
        << cim.x_tran_pos(0) << ", "
        << cim.x_tran_pos(1) << ", "
        << cim.x_tran_pos(2) << ", "
        << cim.x_tran_vel(0) << ", "
        << cim.x_tran_vel(1) << ", "
        << cim.x_tran_vel(2) << ", "
        << cim.eta_rot(0) << ", "
        << cim.eta_rot(1) << ", "
        << cim.eta_rot(2) << ", "
        << cim.omega_rot(0) << ", "
        << cim.omega_rot(1) << ", "
        << cim.omega_rot(2) << ", "
        << cim.r_user(0) << ", "
        << cim.r_user(1) << ", "
        << cim.r_user(2) << ", "
        << cim.r_dot_user(0) << ", "
        << cim.r_dot_user(1) << ", "
        << cim.r_dot_user(2) << ", "
        << cim.eta_rot_d(0) << ", "
        << cim.eta_rot_d(1) << ", "
        << cim.eta_rot_d(2) << ", "
        << cim.eta_rot_rate_d(0) << ", "
        << cim.eta_rot_rate_d(1) << ", "
        << cim.eta_rot_rate_d(2) << ", "
        << cim.omega_rot_d(0) << ", "
        << cim.omega_rot_d(1) << ", "
        << cim.omega_rot_d(2) << ", "
        << cim.alpha_rot_d(0) << ", "
        << cim.alpha_rot_d(1) << ", "
        << cim.alpha_rot_d(2) << ", "
        << cim.e_tran_pos(0) << ", "
        << cim.e_tran_pos(1) << ", "
        << cim.e_tran_pos(2) << ", "
        << csm.e_tran_pos_I(0) << ", "
        << csm.e_tran_pos_I(1) << ", "
        << csm.e_tran_pos_I(2) << ", "
        << cim.e_tran_vel(0) << ", "
        << cim.e_tran_vel(1) << ", "
        << cim.e_tran_vel(2) << ", "
        << cim.e_rot_eta(0) << ", "
        << cim.e_rot_eta(1) << ", "
        << cim.e_rot_eta(2) << ", "
        << csm.e_rot_eta_I(0) << ", "
        << csm.e_rot_eta_I(1) << ", "
        << csm.e_rot_eta_I(2) << ", "
        << cim.e_rot_omega(0) << ", "
        << cim.e_rot_omega(1) << ", "
        << cim.e_rot_omega(2) << ", "
        << cim.mu_tran_I(0) << ", "
        << cim.mu_tran_I(1) << ", "
        << cim.mu_tran_I(2) << ", "
        << cim.mu_tran_J(0) << ", "
        << cim.mu_tran_J(1) << ", "
        << cim.mu_tran_J(2) << ", "
        << cim.u(0) << ", "
        << cim.u(1) << ", "
        << cim.u(2) << ", "
        << cim.u(3) << ", "
        << cim.Thrust(0) << ", "
        << cim.Thrust(1) << ", "
        << cim.Thrust(2) << ", "
        << cim.Thrust(3) << ", "
        << control_input(0) << ", "
        << control_input(1) << ", "
        << control_input(2) << ", "
        << control_input(3) << ", "
        ;

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID OMEGA LOG HEADER DATA", e.what());
    }
}


}   // namespace _pid_omega_

}   // namespace _qrbp_
    
}   // namespace _acsl_
