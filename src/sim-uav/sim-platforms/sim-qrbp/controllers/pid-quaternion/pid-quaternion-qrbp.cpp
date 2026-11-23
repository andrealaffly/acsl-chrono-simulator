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
 * File:        pid-quaternion-qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        March 09, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID  with quaternion and angular velocities for the QRBP. Inherts the class controller_base for the basic 
 *              functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "pid-quaternion-qrbp.hpp"

namespace _acsl_
{

namespace _qrbp_
{

namespace _pid_quaternion_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
pid_quaternion::pid_quaternion(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                    : ::_acsl_::_control_::controller_base(logger, trajectory)
{
    // Initial Conditions
    init();   
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void pid_quaternion::read_params(const std::string& jsonFile)
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
    cip.Kp_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_rotational"], 3, 3);
    cip.Kd_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_rotational"], 3, 3);

    // Differentiator matrices
    cip.A_filter_Qd = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["A_filter_Qd"], 2, 2);
    cip.B_filter_Qd = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["B_filter_Qd"], 2, 1);
    cip.C_filter_Qd = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["C_filter_Qd"], 1, 2);
    cip.A_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["A_filter_omega_d"], 2, 2);
    cip.B_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["B_filter_omega_d"], 2, 1);
    cip.C_filter_omega_d = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["C_filter_omega_d"], 1, 2);

}

// Implementing virutal functions from controller_base
void pid_quaternion::init(){
    // Reading in the parameters
    read_params("../chrono-assets/parameters/qrbp/PID_QUATERNION/gains_PID_QUATERNION.json");

    // Set the inital conditions
    y.fill(0.0);
    dy.fill(0.0);

    // Setup the logging
    InitiateLogging();
    ConfigureHeaders();
}

// Update function for the controller
void pid_quaternion::update(double time, 
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
    cim.Rji = cim.q.toRotationMatrix();
    cim.Rij = cim.Rji.transpose();


    // Asign the angular velocity
    cim.omega << w_x, w_y, w_z;

    // 2. Get the reference trajectory ---------------------------------------------
    m_traj.UpdateModule(time);
    cim.r_user = m_traj.GetPosition();
    cim.r_dot_user = m_traj.GetVelocity();
    cim.r_ddot_user = m_traj.GetAcceleration();   
    cim.psi_user = m_traj.GetEulerAngle()(2);
    cim.psi_user_unwrapped = ::_shared_::_compute_::unwrapPsiSimple(cim.psi_user, this->psiState);

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Assign the values from the integrator ------------------------------------
    assign_from_rk4();
}

// Function to assign elements from the rk4 integrator
void pid_quaternion::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_q_d0_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_q_d1_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_q_d2_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_q_d3_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_z_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_I, y, index);
}

// Model function for integration
void pid_quaternion::model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t)
{
    //------------ Fill up the dy for integration  ------------//
    int index = 0;
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_q_d0_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_q_d1_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_q_d2_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_q_d3_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos, dy, index);
}


// Function to compute the outerloop control in I
void pid_quaternion::compute_translational_control_in_I()
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
void pid_quaternion::compute_u1_q_d()
{
	// Compute the desired thrust setpoint
    cim.f_d = cim.mu_tran_I.norm();
    
    // Assign the thrust setpoint to the control vector
    cim.u(0) = cim.f_d;

    // Compute the desired control action in the body frame
    cim.mu_tran_J << cim.Rij * cim.mu_tran_I;

    // Compute the desired axis for the thrust in a computation safe way
    // In problems of practical interest \min{\| f_d \|} = MASS * GRAVITY.
    if (cim.f_d > 1e-6) {
        cim.f_d_hat_I = cim.mu_tran_J / cim.f_d;
    }
    else {
        cim.f_d = 0.0;
        cim.f_d_hat_I << 0.0, 0.0, 1.0;
    }

    cim.q_align = ::_shared_::_compute_::qFromTwoVectors(e3_basis, -cim.f_d_hat_I);

    // Geodesic sign choice for the shortest rotation
    // Dot product in ℝ⁴ (w, x, y, z)
    double dot_q = cim.q.vec().dot(cim.q_align.vec()) + cim.q.w() * cim.q_align.w();

    // If dot < 0, flip q_align; otherwise keep as is
    cim.q_align_star = (dot_q < 0.0)
        ? Eigen::Quaterniond(-cim.q_align.w(),
                             -cim.q_align.x(),
                             -cim.q_align.y(),
                             -cim.q_align.z())
        : cim.q_align;

    // Fill up the filter dynamics for integration
    cim.internal_state_q_d0_filter << cip.A_filter_Qd * csm.state_q_d0_filter
                                    + cip.B_filter_Qd * cim.q_align.w();

    cim.internal_state_q_d1_filter << cip.A_filter_Qd * csm.state_q_d1_filter
                                    + cip.B_filter_Qd * cim.q_align.x();
                                    
    cim.internal_state_q_d2_filter << cip.A_filter_Qd * csm.state_q_d2_filter
                                    + cip.B_filter_Qd * cim.q_align.y();                                    

    cim.internal_state_q_d3_filter << cip.A_filter_Qd * csm.state_q_d3_filter
                                    + cip.B_filter_Qd * cim.q_align.z();  
                                    
    // Reconstruct the raw differentiated signal
    cim.q_signal_dot = Eigen::Quaterniond(cip.C_filter_Qd * csm.state_q_d0_filter,
                                          cip.C_filter_Qd * csm.state_q_d1_filter,
                                          cip.C_filter_Qd * csm.state_q_d2_filter,
                                          cip.C_filter_Qd * csm.state_q_d3_filter);

    // Enforce the orthogonality condition
    // Cache the projection q_align^T q_signal_dot
    double proj = cim.q_align.w() * cim.q_signal_dot.w()
                + cim.q_align.vec().dot(cim.q_signal_dot.vec());

    // q_d_dot = q_signal_dot - proj * q_align;
    cim.q_align_dot.w() = cim.q_signal_dot.w() - proj * cim.q_align.w();
    cim.q_align_dot.vec() = cim.q_signal_dot.vec() - proj * cim.q_align.vec();

    // Compute desired angular velocity ω_d from q and q_d_dot
    // q_conj = [ q0; -qv ] - Split into scalar and vector parts 
    Eigen::Quaterniond q_conj = cim.q.conjugate();

    const double a0           = q_conj.w();
    const Eigen::Vector3d av  = q_conj.vec();
    const double b0           = cim.q_align_dot.w();
    const Eigen::Vector3d bv  = cim.q_align_dot.vec();

    Eigen::Vector3d v = a0 * bv
                      + b0 * av
                      + av.cross(bv);

    cim.omega_d = 2.0 * v;

    // Fill up the filter dynamics for integration
    cim.internal_state_omega_x_d_filter << cip.A_filter_omega_d * csm.state_omega_x_d_filter
                                         + cip.B_filter_omega_d * cim.omega_d(0);

    cim.internal_state_omega_y_d_filter << cip.A_filter_omega_d * csm.state_omega_y_d_filter
                                         + cip.B_filter_omega_d * cim.omega_d(1);
                                         
    cim.internal_state_omega_z_d_filter << cip.A_filter_omega_d * csm.state_omega_z_d_filter
                                         + cip.B_filter_omega_d * cim.omega_d(2);

    // Reconstruct the differetiated desired angular acceleration
    cim.alpha_d = Eigen::Matrix<double, 3, 1>(cip.C_filter_omega_d * csm.state_omega_x_d_filter,
                                              cip.C_filter_omega_d * csm.state_omega_y_d_filter,
                                              cip.C_filter_omega_d * csm.state_omega_z_d_filter);

    // TEST -----------------------------------------------------------------

    // Construct the yaw quaternion - NEEDS FIXING IN THEORY
    // Eigen::AngleAxisd yaw_aa( 0.0 , Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_aa(cim.psi_user_unwrapped, -cim.f_d_hat_I.normalized());
    cim.q_yaw = Eigen::Quaterniond(yaw_aa);
    
    // Finally construct the desired orientaiton quaternion
    // cim.q_d = cim.q_yaw * cim.q_align_star;
    cim.q_d = cim.q_yaw * cim.q_align_star;

    // TEST -----------------------------------------------------------------

    // Normalize to guard against numerical drift
    cim.q_d.normalize();                                              

}


// Compute the rotational control
void pid_quaternion::compute_rotational_control()
{
    // // Quaternion error: q_e = q_d ⊗ q*
    // Eigen::Quaterniond q_conj = cim.q.conjugate();   // q*
    // Eigen::Quaterniond q_e    = cim.q_d * q_conj;    // q_d ⊗ q*

    // // Unwinding:
    // // q_e = [1; 0; 0; 0] - [sign(q_e(1))*q_e(1); q_e(2:4)]
    // double s = (q_e.w() >= 0.0) ? 1.0 : -1.0;        // sign(q_e(1))

    // double q0_mod          = s * q_e.w();
    // Eigen::Vector3d qv_mod = q_e.vec();

    // // Final unwound quaternion error stored in cim.q_e
    // cim.q_e = Eigen::Quaterniond(
    //     1.0 - q0_mod,        // scalar part
    //     -qv_mod.x(),
    //     -qv_mod.y(),
    //     -qv_mod.z());

    // // Normalize to guard against numerical drift
    // cim.q_e.normalize();        

    // TEST -----------------------------------------------------------------
    Eigen::Quaterniond q_conj = cim.q.conjugate();   // q*
    Eigen::Quaterniond q_e_raw = cim.q_d * q_conj;   // q_d ⊗ q*

    // Align
    Eigen::Quaterniond q_e;
    double s_align = (q_e_raw.w() >= 0.0) ? 1.0 : -1.0;
    q_e.w()   = s_align * q_e_raw.w();
    q_e.vec() = s_align * q_e_raw.vec();

    // "Unwinding" (actually just Δ = [1;0;0;0] - q_e)
    double s = (q_e.w() >= 0.0) ? 1.0 : -1.0;   // sign(q_e(0))

    double q0_mod          = s * q_e.w();
    Eigen::Vector3d qv_mod = q_e.vec();

    cim.q_e = Eigen::Quaterniond(
        1.0 - q0_mod,
        -qv_mod.x(),
        -qv_mod.y(),
        -qv_mod.z());

    cim.q_e.normalize();

    // TEST -----------------------------------------------------------------

    // Assign the vector parts of q_e
    cim.q_e_vec << cim.q_e.x(), cim.q_e.y(), cim.q_e.z();

    // Compute the angular velocity error
    cim.omega_e << cim.omega - cim.omega_d;

    // Compute the baseline control input
    cim.tau_rot_baseline << inertia_matrix_q * ( - cip.Kp_att * cim.q_e_vec 
                                                 - cip.Kd_att * cim.omega_e
                                                 + cim.alpha_d);

    // Compute with dynamic inversion without aerodynamics
    cim.tau_rot << cim.tau_rot_baseline 
                   + cim.omega.cross(inertia_matrix_q * cim.omega);

    // Assign the control inputs
    cim.u(1) = cim.tau_rot(0);
    cim.u(2) = cim.tau_rot(1);
    cim.u(3) = cim.tau_rot(2); 
                                               
}

// Function to compute the normalized thrusts
void pid_quaternion::compute_normalized_thrusts()
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
void pid_quaternion::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Compute the aerodynamics

    // 2. Compute the translational control input
    compute_translational_control_in_I();

    // 3. Compute the thrust needed and the desired angles
    compute_u1_q_d();

    // 4. Compute the rotational control input
    compute_rotational_control();

    // 5. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 6. Do the integration
    rk4.do_step(boost::bind(&pid_quaternion::model, this, bph::_1, bph::_2, bph::_3),
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
bool pid_quaternion::InitiateLogging()
{
    auto status = _logger_::_filesystem_::setupControllerLogging(this->m_logger, "qrbp" ,"PID_QUATERNION");
    return status;
}

// Funciton that setups up the headers for the log file
void pid_quaternion::ConfigureHeaders()
{

    // Create the oss object
    std::ostringstream oss;

    oss << ", "
        << "Controller Time [s], "
        << "Algorithm Execution Time [us], "
        << "x user [m], "
        << "y user [m], "
        << "z user [m], "
        << "vx user [m/s], "
        << "vy user [m/s], "
        << "vz user [m/s], "
        << "ax user [m/s2], "
        << "ay user [m/s2], "
        << "az user [m/s2], "
        << "psi user [rad], "
        << "psi user unwrapped [rad], "
        << "x [m], "
        << "y [m], "
        << "z [m], "
        << "vx [m/s], "
        << "vy [m/s], "
        << "vz [m/s], "
        << "Error in x [m], "
        << "Error in y [m], "
        << "Error in z [m], "
        << "Error in vx [m/s], "
        << "Error in vy [m/s], "
        << "Error in vz [m/s], "
        << "ex int [-], "
        << "ey int [-], "
        << "ez int [-], "
        << "mu_x I [N], "
        << "mu_y I [N], "
        << "mu_z I [N], "
        << "mu_x J [N], "
        << "mu_y J [N], "
        << "mu_z J [N], "
        << "f_d_hat x [-], "
        << "f_d_hat y [-], "
        << "f_d_hat z [-], "
        << "q_align w [-], "
        << "q_align x [-], "
        << "q_align y [-], "
        << "q_align z [-], "
        << "q_align_star w [-], "
        << "q_align_star x [-], "
        << "q_align_star y [-], "
        << "q_align_star z [-], "
        << "q_yaw w [-], "
        << "q_yaw x [-], "
        << "q_yaw y [-], "
        << "q_yaw z [-], "
        << "q_d w [-], "
        << "q_d x [-], "
        << "q_d y [-], "
        << "q_d z [-], "
        << "q_signal_dot w [-], "
        << "q_signal_dot x [-], "
        << "q_signal_dot y [-], "
        << "q_signal_dot z [-], "
        << "q_align_dot w [-], "
        << "q_align_dot x [-], "
        << "q_align_dot y [-], "
        << "q_align_dot z [-], "
        << "omega_d x [rad/s], "
        << "omega_d y [rad/s], "
        << "omega_d z [rad/s], "
        << "alpha_d x [rad/s2], "
        << "alpha_d y [rad/s2], "
        << "alpha_d z [rad/s2], "
        << "q w [-], "
        << "q x [-], "
        << "q y [-], "
        << "q z [-], "
        << "omega x [rad/s], "
        << "omega y [rad/s], "
        << "omega z [rad/s], "
        << "q_e w [-], "
        << "q_e x [-], "
        << "q_e y [-], "
        << "q_e z [-], "
        << "omega_e x [rad/s], "
        << "omega_e y [rad/s], "
        << "omega_e z [rad/s], "
        << "tau_x [Nm], "
        << "tau_y [Nm], "
        << "tau_z [Nm], "
        << "Motor 1 Thrust [N], "
        << "Motor 2 Thrust [N], "
        << "Motor 3 Thrust [N], "
        << "Motor 4 Thrust [N], "
        << "Normalized Thrust 1 [-], "
        << "Normalized Thrust 2 [-], "
        << "Normalized Thrust 3 [-], "
        << "Normalized Thrust 4 [-], "
        ;    

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();

        _message_::SIMULATOR_INFO("[SIMCTL]: WROTE PID QUATERNION LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID QUATERNION LOG HEADER DATA", e.what());
    }

}

void pid_quaternion::LogData()
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
        << cim.psi_user_unwrapped << ", "
        << cim.x_tran_pos(0) << ", "
        << cim.x_tran_pos(1) << ", "
        << cim.x_tran_pos(2) << ", "
        << cim.x_tran_vel(0) << ", "
        << cim.x_tran_vel(1) << ", "
        << cim.x_tran_vel(2) << ", "
        << cim.e_tran_pos(0) << ", "
        << cim.e_tran_pos(1) << ", "
        << cim.e_tran_pos(2) << ", "
        << cim.e_tran_vel(0) << ", "
        << cim.e_tran_vel(1) << ", "
        << cim.e_tran_vel(2) << ", "
        << csm.e_tran_pos_I(0) << ", "
        << csm.e_tran_pos_I(1) << ", "
        << csm.e_tran_pos_I(2) << ", "
        << cim.mu_tran_I(0) << ", "
        << cim.mu_tran_I(1) << ", "
        << cim.mu_tran_I(2) << ", "
        << cim.mu_tran_J(0) << ", "
        << cim.mu_tran_J(1) << ", "
        << cim.mu_tran_J(2) << ", "
        << cim.f_d_hat_I(0) << ", "
        << cim.f_d_hat_I(1) << ", "
        << cim.f_d_hat_I(2) << ", "
        << cim.q_align.w() << ", "
        << cim.q_align.x() << ", "
        << cim.q_align.y() << ", "
        << cim.q_align.z() << ", "
        << cim.q_align_star.w() << ", "
        << cim.q_align_star.x() << ", "
        << cim.q_align_star.y() << ", "
        << cim.q_align_star.z() << ", "
        << cim.q_yaw.w() << ", "
        << cim.q_yaw.x() << ", "
        << cim.q_yaw.y() << ", "
        << cim.q_yaw.z() << ", "
        << cim.q_d.w() << ", "
        << cim.q_d.x() << ", "
        << cim.q_d.y() << ", "
        << cim.q_d.z() << ", "
        << cim.q_signal_dot.w() << ", "
        << cim.q_signal_dot.x() << ", "
        << cim.q_signal_dot.y() << ", "
        << cim.q_signal_dot.z() << ", "
        << cim.q_align_dot.w() << ", "
        << cim.q_align_dot.x() << ", "
        << cim.q_align_dot.y() << ", "
        << cim.q_align_dot.z() << ", "
        << cim.omega_d(0) << ", "
        << cim.omega_d(1) << ", "
        << cim.omega_d(2) << ", "
        << cim.alpha_d(0) << ", "
        << cim.alpha_d(1) << ", "
        << cim.alpha_d(2) << ", "
        << cim.q.w() << ", "
        << cim.q.x() << ", "
        << cim.q.y() << ", "
        << cim.q.z() << ", "
        << cim.omega(0) << ", "
        << cim.omega(1) << ", "
        << cim.omega(2) << ", "
        << cim.q_e.w() << ", "
        << cim.q_e.x() << ", "
        << cim.q_e.y() << ", "
        << cim.q_e.z() << ", "
        << cim.omega_e(0) << ", "
        << cim.omega_e(1) << ", "
        << cim.omega_e(2) << ", "
        << cim.tau_rot(0) << ", "
        << cim.tau_rot(1) << ", "
        << cim.tau_rot(2) << ", "
        << cim.Thrust(0) << ", "
        << cim.Thrust(1) << ", "
        << cim.Thrust(2) << ", "
        << cim.Thrust(3) << ", "
        << this->control_input(0) << ", "
        << this->control_input(1) << ", "
        << this->control_input(2) << ", "
        << this->control_input(3) << ", "
        ;

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID QUATERNION LOG HEADER DATA", e.what());
    }
}


}   // namespace _pid_quaternion_

}   // namespace _qrbp_
    
}   // namespace _acsl_
