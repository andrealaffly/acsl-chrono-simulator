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
 * File:        tailsitter-parameter-file.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 23, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for common hardcoded tailsitter parameters that are common for all controllers.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef TAILSITTER_PARAMETER_FILE_HPP_
#define TAILSITTER_PARAMETER_FILE_HPP_

#include "Eigen/Dense"

namespace _acsl_
{

namespace _tailsitter_
{

// ##################################################################################
// HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED
// ##################################################################################

inline constexpr double MASS = 1.63252801; 

// Matrix of inertia of the quadcopter frame
// [kg*m^2] inertia matrix of the vehicle system expressed in
// Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
const Eigen::Matrix3d inertia_matrix_q = (Eigen::Matrix3d() << 
                                            0.15226711,	-0.00048479, -0.00000929,
                                            -0.00048479, 0.05253416,  0.00110234,
                                            -0.00000929, 0.00110234,  0.18998386).finished();

// Matrix of inertia of the biplane frame
// [kg*m^2] inertia matrix of the vehicle system expressed in
// Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
const Eigen::Matrix3d inertia_matrix_b = (Eigen::Matrix3d() << 
                                            0.18998386,	-0.00110233,  0.00000929,
                                            -0.00110233, 0.05253416, -0.00048479,
                                            0.00000929,	-0.00048479,  0.15226711).finished();  

// Constants -------------------------------------------------------------------------------------------------------
inline constexpr double G = 9.81;
inline constexpr double RHO_HAT = 1.225;
inline constexpr double PI = 3.1415;
inline constexpr double PIHALF = (PI/2);
inline constexpr double DEG2RAD = (PI/180);
inline constexpr double RAD2DEG = (180/PI);
inline constexpr double SQRT2_OVER_2 = 0.7071067;

// Vehicle and Environment Defines
inline constexpr double LX = 0.2230099724;                                    // dist to motor along x^J              [m] 
inline constexpr double LY = 0.2589711696;                                    // dist to motor along y^J              [m] 
inline constexpr double LY_S = 0.108;                                         // dist to the aero center of the stabs [m]
inline constexpr double LZ = 0.001801629;                                     // dist to the aero center of the wings [m] 
inline constexpr double LZ_S = 0.010234962;                                   // dist to aero center of stabs         [m] 
inline constexpr double SPAN_W = 1.50;                                        // span of wings                        [m]
inline constexpr double CHORD_W = 0.20;                                       // chord of wings                       [m]
inline constexpr double SPAN_S = 0.420;                                       // span of stabs                        [m] 
inline constexpr double CHORD_S = 0.05;                                       // chord of stabs                       [m] 
inline constexpr double PLANFORM_AREA_W = (2.0 * SPAN_W * CHORD_S);           // planform area of wings             [m^2]
inline constexpr double PLANFORM_AREA_S = (2.0 * SPAN_S * CHORD_S);           // planform area of stabs             [m^2]

// Aerodynamic Dynamic Coefficient Defines
inline constexpr double DYN_PRESS_COEFF_W = (0.5 * RHO_HAT * PLANFORM_AREA_W);     
inline constexpr double DYN_PRESS_COEFF_S = (0.5 * RHO_HAT * PLANFORM_AREA_S);

// Location of the aerodynamic centers of the wings and stabilizers
const Eigen::Vector3d R_UP{ -LX,   0.0,    LZ};
const Eigen::Vector3d R_LW{  LX,   0.0,    LZ};
const Eigen::Vector3d R_RT{ 0.0,  LY_S,  LZ_S};
const Eigen::Vector3d R_LT{ 0.0, -LY_S,  LZ_S};

// Weight vector of the tailsitter
static inline const Eigen::Vector3d e3_basis = Eigen::Vector3d(0.0, 0.0, 1.0);

// Thrust Normalizeing polynomials are hardcoded!
// Take care coding this from matlab. check the documentation for the polyval function.
// The polynomails are reversed!.
inline constexpr double CT_MOTOR =  0.542418958461701;                        // motor torque coeff              [-]
inline constexpr double MAX_THRUST = 34;                                      // max allowed thrust per motor    [N] 
inline constexpr double MIN_THRUST = 0.7;                                     // min allowed thrust per motor    [N] 

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
// TMotor F35A - Velox V2808 Kv1300
const Eigen::VectorXd thrust_polynomial_coeff_tailsitter = (Eigen::VectorXd(8) << 
                                                                2.4591731397119028E-10,
                                                                -2.9764216922011841E-8,
                                                                1.5256975606726995E-6,
                                                                -4.4210707058374142E-5,
                                                                0.00081199079192906959,
                                                                -0.0098303333146295951,
                                                                0.094801218437836871,
                                                                -0.012240927327923512
                                                            ).finished();

// Mixer Matrix ----------------------------------------------------------------------------------------------------
// This captures the necessary constants within the lambda function and initializes the mixer_matrix_tailsitter with 
// the resulting matrix.
// [1/4, -1/(4*l_y),  1/(4*l_x),  1/(4*c_t)]
// [1/4,  1/(4*l_y), -1/(4*l_x),  1/(4*c_t)]
// [1/4,  1/(4*l_y),  1/(4*l_x), -1/(4*c_t)]
// [1/4, -1/(4*l_y), -1/(4*l_x), -1/(4*c_t)]
const Eigen::Matrix4d mixer_matrix_tailsitter = []() {
    Eigen::Matrix4d mat; // 4 rows, 4 columns
        
    // Assign values element-by-element
    mat(0, 0) =  1.0 / 4.0;
    mat(0, 1) = -1.0 / (4.0 * LY);
    mat(0, 2) =  1.0 / (4.0 * LX);
    mat(0, 3) =  1.0 / (4.0 * CT_MOTOR); 

    mat(1, 0) =  1.0 / 4.0;
    mat(1, 1) =  1.0 / (4.0 * LY);
    mat(1, 2) = -1.0 / (4.0 * LX);
    mat(1, 3) =  1.0 / (4.0 * CT_MOTOR); 

    mat(2, 0) =  1.0 / 4.0;
    mat(2, 1) =  1.0 / (4.0 * LY);
    mat(2, 2) =  1.0 / (4.0 * LX);
    mat(2, 3) = -1.0 / (4.0 * CT_MOTOR); 

    mat(3, 0) =  1.0 / 4.0;
    mat(3, 1) = -1.0 / (4.0 * LY);
    mat(3, 2) = -1.0 / (4.0 * LX);
    mat(3, 3) = -1.0 / (4.0 * CT_MOTOR); 

    return mat; 
}();

// ##################################################################################
// HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED - HARDCODED
// ##################################################################################

}   // namespace _tailsitter_

}   // namespace _acsl_

#endif  // TAILSITTER_PARAMETER_FILE_HPP_