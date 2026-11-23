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
 * File:        x8-parameter-file.hpp
 * Author:      Xavier Casanova
 * Date:        August 04, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for common hardcoded medium quadcopter parameters that are common for all controllers.
 * 
 * GitHub:    https://github.com/xavier1cas/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#ifndef X8_PARAMETER_FILE_HPP_
#define X8_PARAMETER_FILE_HPP_

#include "Eigen/Dense"

namespace _acsl_
{

namespace _x8_
{

// ##################################################################################
// HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED
// ##################################################################################

inline constexpr double MASS = 1.54952076;  // mass [Kg]

// Matrix of inertia of the quadcopter frame 
// [kg*m^2] inertia matrix of the vehicle system expressed in
// Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
// const Eigen::Matrix3d inertia_matrix_q = (Eigen::Matrix3d() << 
//                                                 0.01589049,	0.00001170,-0.00071388,
//                                                 0.00001170,	0.01390128,-0.00001036,
//                                                -0.00071388,-0.00001036,	0.01399882).finished();
const Eigen::Matrix3d inertia_matrix_q = (Eigen::Matrix3d() <<
                                                0.01589049, -0.00001170,  0.00071388,
                                               -0.00001170,  0.01390128,  0.00001036,
                                                0.00071388,  0.00001036,  0.01399882).finished();

// Constants -------------------------------------------------------------------------------------------------------
inline constexpr double G = 9.81;
inline constexpr double RHO_HAT = 1.225;
inline constexpr double PI = 3.1415;
inline constexpr double PIHALF = (PI/2);
inline constexpr double DEG2RAD = (PI/180);
inline constexpr double RAD2DEG = (180/PI);
inline constexpr double SQRT2_OVER_2 = 0.7071067;

// Vehicle and Environment Defines
inline constexpr double LX = 0.0881269;     // dist to motor along x^J              [m]
inline constexpr double LY = 0.1083450;     // dist to motor along y^J              [m]

// Weight vector of the x8
static inline const Eigen::Vector3d e3_basis = Eigen::Vector3d(0.0, 0.0, 1.0);

// Thrust Normalizeing polynomials are hardcoded!
// Take care coding this from matlab. check the documentation for the polyval function.
// The polynomails are reversed!.
// inline constexpr double CT_MOTOR =  0.01;       // motor torque coeff              [-]
// inline constexpr double MAX_THRUST = 10.625;    // max allowed thrust per motor    [N]
inline constexpr double CT_MOTOR =  0.1;        // motor torque coeff              [-]
// Kept below 9.98 N, where the thrust -> normalized polynomial above reaches 1.0: past that the simulator's
// normalized -> thrust polynomial diverges (10.625 N mapped to a 667 N command).
inline constexpr double MAX_THRUST = 9.9;       // max allowed thrust per motor    [N]
inline constexpr double MIN_THRUST = 0.6275;    // min allowed thrust per motor    [N]

// Roll Rate Filter ------------------------------------------------------------------------------------------------
// A matrix of the roll_ref filter
const Eigen::Matrix2d A_filter_roll_ref = (Eigen::Matrix2d() << 
                                                -72.0, -1600.0, 
                                                  1.0,    0.0 
                                            ).finished(); 

// B matrix of the roll_ref filter
const Eigen::Vector2d B_filter_roll_ref = Eigen::Vector2d(1.0, 0); 

// C matrix of the roll_ref filter
const Eigen::RowVector2d C_filter_roll_ref = Eigen::RowVector2d(1600.0, 0); 

// D matrix of the roll_ref filter
const double D_filter_roll_ref = 0.0; 

// Pitch Rate Filter -----------------------------------------------------------------------------------------------
// A matrix of the pitch_ref filter
const Eigen::Matrix2d A_filter_pitch_ref = (Eigen::Matrix2d() << 
                                                -72.0, -1600.0, 
                                                  1.0,    0.0
                                            ).finished(); 

// B matrix of the pitch_ref filter
const Eigen::Vector2d B_filter_pitch_ref = Eigen::Vector2d(1.0, 0); 

// C matrix of the pitch_ref filter
const Eigen::RowVector2d C_filter_pitch_ref = Eigen::RowVector2d(1600.0, 0); 

// D matrix of the pitch_ref filter
const double D_filter_pitch_ref = 0.0; 

// Yaw Rate Filter -------------------------------------------------------------------------------------------------
// A matrix of the yaw_ref filter
const Eigen::Matrix2d A_filter_yaw_ref = (Eigen::Matrix2d() << 
                                                -72.0, -1600.0, 
                                                  1.0,    0.0
                                            ).finished(); 

// B matrix of the yaw_ref filter
const Eigen::Vector2d B_filter_yaw_ref = Eigen::Vector2d(1.0, 0); 

// C matrix of the yaw_ref filter
const Eigen::RowVector2d C_filter_yaw_ref = Eigen::RowVector2d(1600.0, 0); 

// D matrix of the yaw_ref filter
const double D_filter_yaw_ref = 0.0; 

// Roll Acceleration Filter ----------------------------------------------------------------------------------------
// A matrix of the roll_dot_ref filter
const Eigen::Matrix2d A_filter_roll_dot_ref = (Eigen::Matrix2d() << 
                                                -50.0, -2500.0, 
                                                  1.0,    0.0 
                                            ).finished(); 

// B matrix of the roll_dot_ref filter
const Eigen::Vector2d B_filter_roll_dot_ref = Eigen::Vector2d(1.0, 0); 

// C matrix of the roll_dot_ref filter
const Eigen::RowVector2d C_filter_roll_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

// D matrix of the roll_dot_ref filter
const double D_filter_roll_dot_ref = 0.0; 

// Pitch Acceleration Filter ---------------------------------------------------------------------------------------
// A matrix of the pitch_ref filter
const Eigen::Matrix2d A_filter_pitch_dot_ref = (Eigen::Matrix2d() << 
                                                -50.0, -2500.0, 
                                                  1.0,    0.0
                                            ).finished(); 

// B matrix of the pitch_ref filter
const Eigen::Vector2d B_filter_pitch_dot_ref = Eigen::Vector2d(1.0, 0); 

// C matrix of the pitch_ref filter
const Eigen::RowVector2d C_filter_pitch_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

// D matrix of the pitch_ref filter
const double D_filter_pitch_dot_ref = 0.0; 

// Yaw Acceleration Filter -----------------------------------------------------------------------------------------
// A matrix of the yaw_ref filter
const Eigen::Matrix2d A_filter_yaw_dot_ref = (Eigen::Matrix2d() << 
                                                -50.0, -2500.0, 
                                                  1.0,    0.0
                                            ).finished(); 

// B matrix of the yaw_ref filter
const Eigen::Vector2d B_filter_yaw_dot_ref = Eigen::Vector2d(1.0, 0); 

// C matrix of the yaw_ref filter
const Eigen::RowVector2d C_filter_yaw_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

// D matrix of the yaw_ref filter
const double D_filter_yaw_dot_ref = 0.0; 

// Thrust Polynomial -----------------------------------------------------------------------------------------------
// Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
// TMotor P23069 V3.0 - P23069 V3.0 2550 Kv
const Eigen::VectorXd thrust_polynomial_coeff_x8 = (Eigen::VectorXd(8) << 
                                                        1.1397548468561201E-5,
                                                        -0.00038436172518955446,
                                                        0.0051947365478567029,
                                                        -0.036003519831726248,
                                                        0.13652913911293302,
                                                        -0.2857385586886379,
                                                        0.40625173135017634,
                                                        -0.0586576952028257
                                                    ).finished();

// Mixer Matrix ----------------------------------------------------------------------------------------------------
// This captures the necessary constants within the lambda function and initializes the mixer_matrix_x8 with the
// resulting matrix.
//
// Motor layout (PX4 "Octorotor Coaxial", body frame FRD, top layer = 1-4, bottom layer = 5-8):
//   1: front-right top    CCW        5: front-left  bottom   CCW
//   2: front-left  top    CW         6: front-right bottom   CW
//   3: rear-left   top    CCW        7: rear-right  bottom   CCW
//   4: rear-right  top    CW         8: rear-left   bottom   CW
//
// Allocation matrix B (u = B * T), with u = [thrust, roll, pitch, yaw] and T the 8 motor thrusts:
//   roll  = -y_i * T_i   (motors on the left, y < 0, roll right)
//   pitch =  x_i * T_i   (motors in front, x > 0, pitch up)
//   yaw   = +c_t * T_i for CCW props, -c_t * T_i for CW props (a CCW prop's reaction torque is +z NED, see
//           sim-uav.tpp)
// [   1,    1,    1,    1,    1,    1,    1,    1]
// [-l_y,  l_y,  l_y, -l_y,  l_y, -l_y, -l_y,  l_y]
// [ l_x,  l_x, -l_x, -l_x,  l_x,  l_x, -l_x, -l_x]
// [ c_t, -c_t,  c_t, -c_t,  c_t, -c_t,  c_t, -c_t]
//
// The rows of B are orthogonal, so the mixer (its pseudo-inverse) is B transposed with each columndivided by
// 8 * (that row's entry)^2, i.e. every entry below is sign / (8 * l).
const Eigen::Matrix<double,8,4> mixer_matrix_x8 = []() {
    Eigen::Matrix<double,8,4> mat; // 8 rows, 4 columns

    // Assign values element-by-element
    mat(0, 0) =  1.0 / 8.0;
    mat(0, 1) = -1.0 / (8.0 * LY);
    mat(0, 2) =  1.0 / (8.0 * LX);
    mat(0, 3) =  1.0 / (8.0 * CT_MOTOR);

    mat(1, 0) =  1.0 / 8.0;
    mat(1, 1) =  1.0 / (8.0 * LY);
    mat(1, 2) =  1.0 / (8.0 * LX);
    mat(1, 3) = -1.0 / (8.0 * CT_MOTOR);

    mat(2, 0) =  1.0 / 8.0;
    mat(2, 1) =  1.0 / (8.0 * LY);
    mat(2, 2) = -1.0 / (8.0 * LX);
    mat(2, 3) =  1.0 / (8.0 * CT_MOTOR);

    mat(3, 0) =  1.0 / 8.0;
    mat(3, 1) = -1.0 / (8.0 * LY);
    mat(3, 2) = -1.0 / (8.0 * LX);
    mat(3, 3) = -1.0 / (8.0 * CT_MOTOR);

    mat(4, 0) =  1.0 / 8.0;
    mat(4, 1) =  1.0 / (8.0 * LY);
    mat(4, 2) =  1.0 / (8.0 * LX);
    mat(4, 3) =  1.0 / (8.0 * CT_MOTOR);

    mat(5, 0) =  1.0 / 8.0;
    mat(5, 1) = -1.0 / (8.0 * LY);
    mat(5, 2) =  1.0 / (8.0 * LX);
    mat(5, 3) = -1.0 / (8.0 * CT_MOTOR);

    mat(6, 0) =  1.0 / 8.0;
    mat(6, 1) = -1.0 / (8.0 * LY);
    mat(6, 2) = -1.0 / (8.0 * LX);
    mat(6, 3) =  1.0 / (8.0 * CT_MOTOR);

    mat(7, 0) =  1.0 / 8.0;
    mat(7, 1) =  1.0 / (8.0 * LY);
    mat(7, 2) = -1.0 / (8.0 * LX);
    mat(7, 3) = -1.0 / (8.0 * CT_MOTOR);

    return mat;
}();

// ##################################################################################
// HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED
// ##################################################################################

}   // namespace _x8_

}   // namespace _acsl_

#endif  // X8_PARAMETER_FILE_HPP_