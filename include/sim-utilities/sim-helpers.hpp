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
 * File:        sim-helpers.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the common helper functions that are needed by the simulator
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_HELPERS_HPP_
#define SIM_HELPERS_HPP_

// ===============================
// System includes
// ===============================
#include <cmath>
#include <vector>
#include <filesystem>
#include <string>
#include <stdexcept>

// ===============================
// Chrono includes
// ===============================
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChRotation.h"
#include "chrono/geometry/ChLineNurbs.h"
#include "chrono/geometry/ChSurfaceNurbs.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChBodyAuxRef.h"

// ===============================
// Boost includes
// ===============================
#include <boost/make_shared.hpp>
#include <boost/log/core.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/attributes/constant.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>


// ===============================
// RapidCSV includes
// ===============================
#include "rapidcsv.h"

// ===============================
// JSON includes
// ===============================
#include <nlohmann/json.hpp>            

// ===============================
// Eigen includes
// ===============================
#include <Eigen/Dense>  

// ===============================
// Linterp inlcudes
// ===============================
#include "linterp.h"

// ===============================
// ACSL inlcudes
// ===============================
#include "sim-messages.hpp"


// ============================================================================================================
// Helper Function Declaration: _acsl_ helper namespaces
// Define the functions for sim-helpers.hpp --> Do not include any templated functions. 
//                                          --> Avoid data type ambiguity as much as possible. 
//                                          --> Use doubles religiously.
// ============================================================================================================
namespace _shared_
{

// ============================================================================================================
// Namespace: _transformations_
// Description: Functions for coordinate and orientation conversions 
//              between NED (North-East-Down) and Chrono's global frame
// ============================================================================================================
namespace _transformations_
{

/**
 * @brief Converts NED position vector to Chrono global frame vector.
 * @param nedpos Position in North-East-Down frame (chrono::ChVector3<>)
 * @return Equivalent position in Chrono's global frame (chrono::ChVector3<>)
 */
chrono::ChVector3<> GetChronoPosFromNED(const chrono::ChVector3<>& nedpos);           

/**
 * @brief Converts NED quaternion to Chrono orientation.
 * @param q_ned Quaternion in NED frame (chrono::ChQuaternion<>)
 * @return Corresponding orientation in Chrono global frame (chrono::ChQuaternion<>)
 */
chrono::ChQuaternion<> GetChronoOrientaitonFromNED(const chrono::ChQuaternion<>& q_ned = chrono::ChQuaternion<>(1, 0, 0, 0));

/**
 * @brief Converts position from Chrono global frame to NED position.
 * @param pos_chrono Position in Chrono's global/world frame (chrono::ChVector3<>)
 * @return Equivalent position in the NED frame (chrono::ChVector3<>)
 */
chrono::ChVector3<> GetNEDPosFromChrono(const chrono::ChVector3<>& pos_chrono);   

/**
 * @brief Converts orientation from Chrono global world quaternion to NED quaternion.
 * @param q_chrono Quaternion in Chrono world/global frame (chrono::ChQuaternion<>)
 * @return Corresponding orientation in NED frame (chrono::ChQuaternion<>)
 */
chrono::ChQuaternion<> GetNEDOrientationFromChrono(const chrono::ChQuaternion<>& q_chrono = chrono::ChQuaternion<>(1, 0, 0, 0));

/**
 * @brief Computes the 3-2-1 rotation matrix that transforms a vector from global to local coordinates.
 * 
 * The returned matrix is: R1ᵀ * R2ᵀ * R3ᵀ (roll, pitch, yaw).
 * 
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw   Yaw angle in radians.
 * @return Eigen::Matrix3d The global-to-local rotation matrix.
 */
Eigen::Matrix3d rotationMatrix321GlobalToLocal(double roll, double pitch, double yaw);

/**
 * @brief Computes the 3-2-1 rotation matrix that transforms a vector from local to global coordinates.
 * 
 * The returned matrix is: R3 * R2 * R1 (yaw, pitch, roll).
 * 
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw   Yaw angle in radians.
 * @return Eigen::Matrix3d The local-to-global rotation matrix.
 */
Eigen::Matrix3d rotationMatrix321LocalToGlobal(double roll, double pitch, double yaw);

/**
 * @brief Returns a 3x3 Jacobian matrix inverse for given roll and pitch angles.
 *
 * Exception is thrown if the pitch angle results in a singular configuration.
 * 
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @return Eigen::Matrix3d Inverse of the Euler angle rates Jacobian.
 * @throws std::invalid_argument if pitch near ±pi/2.
 */
Eigen::Matrix3d jacobianMatrixInverse(double roll, double pitch);

/**
 * @brief Computes the 3x3 Euler angle rates Jacobian matrix for given roll and pitch.
 * 
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @return Eigen::Matrix3d Euler angle rates Jacobian.
 */
Eigen::Matrix3d jacobianMatrix(double roll, double pitch);

/**
 * @brief Computes the time derivative of the Jacobian matrix for given roll, pitch, and their time derivatives.
 * 
 * @param roll      Roll angle in radians.
 * @param pitch     Pitch angle in radians.
 * @param roll_dot  Time derivative of roll (rad/s).
 * @param pitch_dot Time derivative of pitch (rad/s).
 * @return Eigen::Matrix3d Time derivative of the Jacobian matrix.
 */
Eigen::Matrix3d jacobianMatrixDerivative(
    double roll, double pitch, double roll_dot, double pitch_dot
);

/**
 * @brief Computes the skew symmetric matrix of a 3d vector. 
 * 
 * @param v The 3d vector v
 * @return Eigen::Matrix3d The skew symmetrix matrix of v.
 */
Eigen::Matrix3d skewSymmetric(Eigen::Matrix<double, 3, 1> v);

/**
 * @brief Extracts the 3d vector from a skew symmetric matrix (inverse of skewSymmetric).
 *
 * Only the lower triangular entries are read, so the input is assumed to be skew symmetric.
 *
 * @param m The 3x3 skew symmetric matrix.
 * @return Eigen::Vector3d The vector v such that skewSymmetric(v) = m.
 */
Eigen::Vector3d veeMap(const Eigen::Matrix3d& m);

/**
 * @brief Computes the rotation matrix from a rotation vector (exponential map of SO(3)).
 *
 * Uses the Rodrigues formula, where the rotation vector is phi = angle * axis.
 * A zero rotation (|phi| < 1e-12) has no defined axis, so the identity is returned.
 *
 * @param phi Rotation vector (axis * angle) in radians.
 * @return Eigen::Matrix3d The rotation matrix Exp(phi).
 */
Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi);

/**
 * @brief Computes the rotation vector from a rotation matrix (logarithmic map of SO(3)).
 *
 * Inverse of expSO3. The returned rotation vector has its angle in [0, pi].
 *
 * @param R Rotation matrix (must be a valid element of SO(3)).
 * @return Eigen::Vector3d The rotation vector Log(R) = angle * axis, in radians.
 */
Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);

} // namespace _transformations_

// ============================================================================================================
// Namespace: _conversions_
// Description: Utilities for physical unit conversion.
// ============================================================================================================
namespace _conversions_
{

/**
 * @brief Converts radians to degrees.
 * @param rad Angle in radians
 * @return Equivalent angle in degrees
 */
double rad2deg(double rad);

/**
 * @brief Converts degrees to radians.
 * @param rad Angle in degrees
 * @return Equivalent angle in radians
 */
double deg2rad(double deg);


/**
 * @brief Convert a boolean value to "true" or "false".
 *
 * @param value Boolean value to convert.
 * @return std::string "true" if value is true, otherwise "false".
 */
std::string bool2string(bool value);

/**
 * @brief Compute roll–pitch–yaw Euler angles from a unit quaternion
 *        using a 3–2–1 (Z–Y–X) rotation sequence.
 *
 * The returned vector is ordered as:
 *   x = roll  (about X)
 *   y = pitch (about Y)
 *   z = yaw   (about Z)
 *
 * @param q Unit quaternion (Chrono convention: e0=w, e1=x, e2=y, e3=z)
 * @return chrono::ChVector3d containing [roll, pitch, yaw] in radians.
 */
chrono::ChVector3d QuaternionToEulerAnglesRPY_321(const chrono::ChQuaterniond& q);

/**
 * @brief Compute roll–pitch–yaw-like Euler angles from a unit quaternion
 *        using an internal 2–3–1 (Y–Z–X) rotation sequence.
 *
 * The quaternion is first converted to 2–3–1 Euler angles
 * (rotation about Y, then Z, then X). These angles are then
 * mapped into a 3–2–1-style roll–pitch–yaw triplet for
 * plotting and logging:
 *
 *   x = roll_321_like  (about body X; from 2–3–1 gamma)
 *   y = pitch_321_like (about body Y; from 2–3–1 beta)
 *   z = yaw_321_like   (about body Z; from 2–3–1 alpha)
 *
 * This representation is intended to remain well-behaved
 * near pitch ≈ 90 deg where a pure 3–2–1 parametrization
 * suffers from gimbal lock.
 *
 * @param q Unit quaternion (Chrono convention: e0 = w, e1 = x, e2 = y, e3 = z).
 * @return chrono::ChVector3d containing [roll_321_like, pitch_321_like,
 *         yaw_321_like] in radians.
 */
chrono::ChVector3d QuaternionToEulerAnglesRPY_321_from231(
    const chrono::ChQuaterniond& q);


} // namespace _conversions_

// ============================================================================================================
// Namespace: _compute_
// Description: General-purpose computation and math utilities
// ============================================================================================================
namespace _compute_
{

/**
 * @brief Factory function for 1D linear interpolators. Given a grid (e.g., time) and its value
 *        vector (e.g., position), returns a fully constructed NDInterpolator_1_ML object ready
 *        for queries.
 * @param grid vector of double, the grid points (must be monotonically increasing)
 * @param values vector of double, values at each grid point (same size as grid)
 * @return NDInterpolator_1_ML object pointer configured for interpolation over the provided data.
 * @note Both grid and value vectors must remain valid for the lifetime of returned interpolator.
 *       Use with care if passing temporary/local vectors.       
 */
NDInterpolator_1_ML* create1DInterpolator(const std::vector<double>& grid, const std::vector<double>& values);

/**
 * @brief Finds the maximum value in a vector of doubles.
 * @param v Input vector of doubles.
 * @return Maximum value in the vector.
 * @note Behavior is undefined for empty vectors; add a check as needed.
 */
double vecmax(const std::vector<double>& v);

/**
 * @brief Wraps a double angle `alpha` to the interval [-pi, pi].
 *
 * @param[in] alpha Angle (in radians) to wrap.
 * @returns Wrapped angle in [-pi, pi] (in radians).
 */
double wrapAngleToMinusPiAndPi(double alpha);

/**
 * @brief Computes a continuous yaw angular error considering the discontinuity at +/- pi.
 *
 * Example usage: 
 * @code
 * double result = makeYawAngularErrorContinuous(yaw, user_defined_yaw);
 * @endcode
 *
 * @param[in] yaw Current yaw angle (in radians).
 * @param[in] user_defined_yaw Reference yaw angle (in radians).
 * @returns Continuous angular error (in radians), accounting for discontinuity at +/- pi.
 */
double makeYawAngularErrorContinuous(double yaw, double user_defined_yaw);

/**
 * @brief State for simple incremental yaw unwrapping.
 *
 * Maintains the last unwrapped yaw and an initialization flag.
 */
struct SimplePsiUnwrapState {
    double prevUnwrapped;
    bool   initialized;

    SimplePsiUnwrapState()
        : prevUnwrapped(0.0),
          initialized(false)
    {}
};

/**
 * @brief Incrementally unwrap yaw by removing ±2*pi jumps in the difference.
 *
 * Standard rule: if dpsi > pi, subtract 2*pi; if dpsi < -pi, add 2*pi.
 *
 * @param[in]  psi_wrapped Wrapped yaw at current time step [rad].
 * @param[in,out] state    Persistent unwrapping state (updated internally).
 * @return Unwrapped (continuous) yaw [rad].
 */
double unwrapPsiSimple(double psi_wrapped, SimplePsiUnwrapState &state);

/**
 * @brief Regularize XYZ Cardan angles to avoid numerical issues near +90 deg pitch.
 *
 * If theta >= theta_max_deg, clamp theta to theta_max_deg and set phi, psi to zero.
 *
 * @param[in] eul_b    Biplane raw XYZ Cardan angles [rad] as (phi, theta, psi)
 * @param[in] eul_q    Quadcopter raw XYZ Cardan angles [rad] as (phi, theta, psi)
 * @return Regularized XYZ Cardan angles [rad].
 */
chrono::ChVector3d RegularizeCardanXYZ(const chrono::ChVector3d& eul_b,
                                       const chrono::ChVector3d& eul_q);


/**
 * @brief Computes midpoints of n equal segments along a 3D line segment.
 * @param p1 Start point of the line segment (chrono::ChVector3d)
 * @param p2 End point of the line segment (chrono::ChVector3d)
 * @param n  Number of equal segments to divide the line into (must be > 0)
 * @return Vector of midpoints for each segment, ordered from p1 to p2
 */
std::vector<chrono::ChVector3d> ComputeSegmentMidpoints(const chrono::ChVector3d& p1,
                                                        const chrono::ChVector3d& p2,
                                                        int n);

/**
 * @brief Construct a quaternion that rotates one vector into another.
 *
 * This matches the MATLAB function:
 *     function q = qFromTwoVectors(u, v)
 *         u = u / norm(u);
 *         v = v / norm(v);
 *         c = dot(u,v);
 *         if c < -1 + 1e-10
 *             axis = null(u.');          % pick any axis ⟂ u
 *             axis = axis(:,1) / norm(axis(:,1));
 *             q = qFromAxisAngle(axis, pi);
 *             return;
 *         end
 *         w = cross(u,v);
 *         q0 = 1 + c;
 *         qv = w;
 *         q = [q0; qv];
 *         % Normalize q safely
 *         nq = norm(q);
 *         if nq < 1e-8
 *             q = [1;0;0;0];
 *         else
 *             q = q / nq;
 *         end
 *     end
 *
 * The implementation:
 * - Normalizes u and v.
 * - Detects the nearly-opposite case (u·v ≈ -1) and explicitly builds a
 *   180-degree rotation around any axis orthogonal to u using Eigen::AngleAxisd.
 * - Otherwise uses the cross/dot formula q = [1 + u·v; u × v].
 * - Safely normalizes the quaternion and falls back to identity if the norm
 *   is extremely small.
 *
 * @param u_in  3D source vector.
 * @param v_in  3D target vector.
 * @return Eigen::Quaterniond  Unit quaternion that rotates u_in into v_in.
 */
Eigen::Quaterniond qFromTwoVectors(const Eigen::Vector3d& u_in,
                                   const Eigen::Vector3d& v_in);
                                                   

} // namespace _compute_

// ============================================================================================================
// Namespace: _serialize_
// Description: Serialization helpers for writing data to a file from the simulator
// ============================================================================================================
namespace _serialize_
{

/**
 * @brief Combines three parallel double vectors into a vector of ChVector3d for 3D point serialization.
 * @param xs Vector of X coordinates.
 * @param ys Vector of Y coordinates.
 * @param zs Vector of Z coordinates.
 * @return std::vector<chrono::ChVector3d> Combined 3D points.
 */
std::vector<chrono::ChVector3d> serialize2ChVector3d(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    const std::vector<double>& zs);

} // namespace _serialize_

// ============================================================================================================
// Namespace: _deserialize_
// Description: Deserialization helpers for reading data from a file into the simulator environment 
// ============================================================================================================
namespace _deserialize_
{

/**
 * @brief Extracts a column of doubles from a RapidCSV document using a header label.
 * @param doc  rapidcsv::Document object
 * @param label String header of column to extract
 * @return std::vector<double> containing the extracted column data
 */
std::vector<double> GetCSVColumn(const rapidcsv::Document& doc, const std::string& label);

/**
 * @brief Checks that the file exists, throws an error if not.
 * @param filepath Path or filename to check.
 * @throws std::runtime_error if the file does not exist.
 */
void FileExists(const std::string& filepath);

/**
 * @brief Extract Eigen::MatrixXd from JSON array of arrays
 * @param jsonMatrix Matrix input that is in the json format
 * @param rows no of rows in the array
 * @param cols no of cols in the array
 */
Eigen::MatrixXd jsonToMatrixXd(const nlohmann::json& jsonMatrix, int rows, int cols);

/**
 * @brief Extract Eigen::MatrixXd from JSON with scaling
 * @param jsonMatrix Matrix input that is in the json format
 * @param rows no of rows in the array
 * @param cols no of cols in the array
 */
Eigen::MatrixXd jsonToScaledMatrixXd(const nlohmann::json& jsonMatrix, int rows, int cols);

} // namespace _deserialize_

// ============================================================================================================
// Namespace: _visualize_
// Description: Helper functions for visualization in the irrlicht environment
// ============================================================================================================
namespace _visualize_
{

/**
 * @brief Creates a NURBS (Non-Uniform Rational B-Spline) visual line asset from a set of 3D control points.
 * 
 * This function generates a NURBS curve using the supplied control points and polynomial order,
 * encapsulates it as a Project Chrono ChVisualShapeLine asset, and sets the display color.
 *
 * @param controlpoints Vector of ChVector3d points defining the NURBS control polygon.
 * @param order         Degree of the NURBS curve (default: cubic, 3; must satisfy order < controlpoints.size()).
 * @param color         Color of the rendered line (default: black RGB).
 * @return std::shared_ptr<chrono::ChVisualShapeLine> Smart pointer to the created NURBS visual asset.
 */
std::shared_ptr<chrono::ChVisualShapeLine> createNurbsVisual(
    std::vector<chrono::ChVector3d>& controlpoints,
    int order = 3,
    const chrono::ChColor& color = chrono::ChColor(0.0f, 0.0f, 0.0f));
    
/**
 * @brief Creates a visual frame representation for a ChMarker using line segments.
 * 
 * Generates three perpendicular line segments (X, Y, Z axes) in red, green, and blue
 * respectively to visualize the marker's coordinate frame in Irrlicht.
 * 
 * @param marker The ChMarker object to visualize. Must have an associated body.
 * @param len    Length of each axis line segment (default: 0.5).
 * @param thk    Thickness of the frame lines (default: 3.0).  
 */
void createMarkerFrameVisual(chrono::ChMarker& marker, double len = 0.5, double thk = 3.0);

} // namespace _visualize_

} // namespace _shared_

#endif // SIM_HELPERS_HPP_
