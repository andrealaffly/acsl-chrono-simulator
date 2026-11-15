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
 * File:        sim-helpers.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: cpp file for the common helper functions that are needed by the simulator
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-helpers.hpp"

// ============================================================================================================
// Helper Function Implementation: _acsl_ helper namespaces
// Place this block in sim_helpers.cpp --> Do not include any templated functions. 
//                                     --> Avoid data type ambiguity as much as possible. 
//                                     --> Use doubles religiously.
// ============================================================================================================

namespace _shared_
{

// ============================================================================================================
// _transformations_ : Coordinate and orientation conversions between NED and Chrono frames
// ============================================================================================================
namespace _transformations_
{

// Converts vector from NED to Chrono global frame using a 90-degree rotation about X.
chrono::ChVector3<> GetChronoPosFromNED(const chrono::ChVector3<>& nedpos)
{
    // Chrono uses 90° rot about X to match NED <-> ENU
    chrono::ChQuaternion<> RotChABSFrametoNEDFrame = chrono::QuatFromAngleX(chrono::CH_PI_2);
    // Rotate the input vector to global ENU frame
    return RotChABSFrametoNEDFrame.Rotate(nedpos);
}

// Converts quaternion from NED to Chrono global frame.
chrono::ChQuaternion<> GetChronoOrientaitonFromNED(const chrono::ChQuaternion<>& q_ned)
{
    // 90° rotation about X to convert orientation (ENU <-> NED)
    chrono::ChQuaternion<> q_rot = chrono::QuatFromAngleX(chrono::CH_PI_2);
    chrono::ChQuaternion<> q_result;
    q_result.Cross(q_ned, q_rot);  // Quaternion multiplication: q_ned * q_rot
    return q_result;
}

// Converts vector from Chrono global frame to NED frame using inverse rotation.
chrono::ChVector3<> GetNEDPosFromChrono(const chrono::ChVector3<>& pos_chrono)
{
    // -90° rotation about X for inverse conversion
    chrono::ChQuaternion<> RotChABSFrametoNEDFrame = chrono::QuatFromAngleX(-chrono::CH_PI_2);
    return RotChABSFrametoNEDFrame.Rotate(pos_chrono);
}

// Converts quaternion from Chrono global frame to NED orientation.
chrono::ChQuaternion<> GetNEDOrientationFromChrono(const chrono::ChQuaternion<>& q_chrono)
{
    // Inverse 90° rotation about X for ENU->NED conversion
    chrono::ChQuaternion<> q_rot = chrono::QuatFromAngleX(-chrono::CH_PI_2);
    chrono::ChQuaternion<> q_result;
    q_result.Cross(q_chrono, q_rot);  // Quaternion multiplication: q_chrono * q_rot
    return q_result;
}


// Computes the 3-2-1 rotation matrix for global-to-local transformation.
Eigen::Matrix3d rotationMatrix321GlobalToLocal(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d R1t, R2t, R3t;
    R1t <<  1.0, 0.0, 0.0,
            0.0, std::cos(roll), std::sin(roll),
            0.0, -std::sin(roll), std::cos(roll);
    R2t <<  std::cos(pitch), 0.0, -std::sin(pitch),
            0.0, 1.0, 0.0,
            std::sin(pitch), 0.0, std::cos(pitch);
    R3t <<  std::cos(yaw), std::sin(yaw), 0.0,
            -std::sin(yaw), std::cos(yaw), 0.0,
            0.0, 0.0, 1.0;
    return R1t * R2t * R3t;
}

// Computes the 3-2-1 rotation matrix for local-to-global transformation.
Eigen::Matrix3d rotationMatrix321LocalToGlobal(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d R1, R2, R3;
    R1 <<  1.0, 0.0, 0.0,
           0.0, std::cos(roll), -std::sin(roll),
           0.0, std::sin(roll),  std::cos(roll);
    R2 <<  std::cos(pitch), 0.0, std::sin(pitch),
           0.0, 1.0, 0.0,
           -std::sin(pitch), 0.0, std::cos(pitch);
    R3 <<  std::cos(yaw), -std::sin(yaw), 0.0,
           std::sin(yaw),  std::cos(yaw), 0.0,
           0.0, 0.0, 1.0;
    return R3 * R2 * R1;
}

// Returns the inverse of the Euler angle rates Jacobian matrix.
Eigen::Matrix3d jacobianMatrixInverse(double roll, double pitch)
{
    if (std::abs(std::cos(pitch)) < 1e-6) {
        throw std::invalid_argument("Pitch value results in division by zero.");
    }
    Eigen::Matrix3d Jinv;
    Jinv(0, 0) = 1.0;
    Jinv(0, 1) = (std::sin(roll) * std::sin(pitch)) / std::cos(pitch);
    Jinv(0, 2) = (std::cos(roll) * std::sin(pitch)) / std::cos(pitch);

    Jinv(1, 0) = 0.0;
    Jinv(1, 1) = std::cos(roll);
    Jinv(1, 2) = -std::sin(roll);

    Jinv(2, 0) = 0.0;
    Jinv(2, 1) = std::sin(roll) / std::cos(pitch);
    Jinv(2, 2) = std::cos(roll) / std::cos(pitch);

    return Jinv;
}

// Computes the Euler angle rates Jacobian matrix for roll and pitch.
Eigen::Matrix3d jacobianMatrix(double roll, double pitch)
{
    Eigen::Matrix3d J;
    J(0, 0) = 1.0;
    J(0, 1) = 0.0;
    J(0, 2) = -std::sin(pitch);

    J(1, 0) = 0.0;
    J(1, 1) = std::cos(roll);
    J(1, 2) = std::sin(roll) * std::cos(pitch);

    J(2, 0) = 0.0;
    J(2, 1) = -std::sin(roll);
    J(2, 2) = std::cos(roll) * std::cos(pitch);

    return J;
}

// Computes the time derivative of the Euler angle rates Jacobian matrix.
Eigen::Matrix3d jacobianMatrixDerivative(double roll, double pitch, double roll_dot, double pitch_dot)
{
    Eigen::Matrix3d Jdot;
    Jdot(0, 0) = 0.0;
    Jdot(0, 1) = 0.0;
    Jdot(0, 2) = -std::cos(pitch) * pitch_dot;

    Jdot(1, 0) = 0.0;
    Jdot(1, 1) = -std::sin(roll) * roll_dot;
    Jdot(1, 2) =  std::cos(roll) * std::cos(pitch) * roll_dot
                 - std::sin(roll) * std::sin(pitch) * pitch_dot;

    Jdot(2, 0) = 0.0;
    Jdot(2, 1) = -std::cos(roll) * roll_dot;
    Jdot(2, 2) = -std::sin(roll) * std::cos(pitch) * roll_dot
                 - std::cos(roll) * std::sin(pitch) * pitch_dot;

    return Jdot;
}

// Computes the skew symmetric matrix of a vector v
Eigen::Matrix3d skewSymmetric(Eigen::Matrix<double, 3, 1> v) {
    
    // Skew-Symmetric matrix structure
    // v = [v1 v2 v3]^T
    // 
    // m = [  0 -v3  v2]
    //     [ v3   0 -v1]
    //     [-v2  v1   0]
    Eigen::Matrix3d m;

    m <<  0.0, -v(2),  v(1),
          v(2),  0.0, -v(0),
         -v(1), v(0),   0.0;

    return m;
}

// Extracts the vector v from a skew symmetric matrix m (inverse of skewSymmetric)
Eigen::Vector3d veeMap(const Eigen::Matrix3d& m)
{
    // Skew-Symmetric matrix structure
    // m = [  0 -v3  v2]
    //     [ v3   0 -v1]
    //     [-v2  v1   0]
    //
    // v = [v1 v2 v3]^T = [m(2,1) m(0,2) m(1,0)]^T
    Eigen::Vector3d v;

    v << m(2, 1), m(0, 2), m(1, 0);

    return v;
}

// Computes the rotation matrix from a rotation vector (exponential map of SO(3))
Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi)
{
    // Rodrigues formula through Eigen's angle-axis representation
    // phi = angle * axis, where angle = |phi| and axis = phi / |phi|
    const double angle = phi.norm();

    // A zero rotation has no defined axis, so return the identity directly
    if (angle < 1e-12) {
        return Eigen::Matrix3d::Identity();
    }

    return Eigen::AngleAxisd(angle, phi / angle).toRotationMatrix();
}

// Computes the rotation vector from a rotation matrix (logarithmic map of SO(3))
Eigen::Vector3d logSO3(const Eigen::Matrix3d& R)
{
    // Inverse of expSO3: R -> angle * axis, with the angle in [0, pi]
    const Eigen::AngleAxisd aa(R);

    return aa.angle() * aa.axis();
}

} // namespace _transformations_


// ============================================================================================================
// _conversions_ : Unit conversion utilities
// ============================================================================================================
namespace _conversions_
{

// Converts radians to degrees, works with any arithmetic double.
double rad2deg(double rad)
{
    // Use Chrono's value for pi for consistency with the simulation environment
    return rad * 180.0 / chrono::CH_PI;
}

// Converts degrees to radians, works with any arithmetic double.
double deg2rad(double degrees)
{
    // Use Chrono's value for pi for consistency with the simulation environment
    return degrees * chrono::CH_PI / 180.0;
}

// Converts boolean values to "true" or "false" strings for logging purposes.
std::string bool2string(bool value) {
    return value ? "true" : "false";
}

// Converts quaternion to euler via 321 rotation
chrono::ChVector3d QuaternionToEulerAnglesRPY_321(const chrono::ChQuaterniond& q)
{
    chrono::ChVector3d euler;

    // Chrono stores quaternion as (e0, e1, e2, e3) = (w, x, y, z)
    double a = q.e0();
    double b = q.e1();
    double c = q.e2();
    double d = q.e3();

    // Precompute products
    double aa = a * a;
    double ab = a * b;
    double ac = a * c;
    double ad = a * d;
    double bb = b * b;
    double bc = b * c;
    double bd = b * d;
    double cc = c * c;
    double cd = c * d;
    double dd = d * d;

    // DCM elements for 3–2–1 extraction
    double dcm00 = aa + bb - cc - dd;
    double dcm02 = 2.0 * (ac + bd);
    double dcm10 = 2.0 * (bc + ad);
    double dcm12 = 2.0 * (cd - ab);
    double dcm20 = 2.0 * (bd - ac);
    double dcm21 = 2.0 * (ab + cd);
    double dcm22 = aa - bb - cc + dd;

    // Clamp dcm20 to [-1, 1] to avoid domain errors in asin
    if (dcm20 < -1.0)
        dcm20 = -1.0;
    if (dcm20 > 1.0)
        dcm20 = 1.0;

    // Pitch
    double pitch = std::asin(-dcm20);

    double roll;
    double yaw;

    // Gimbal lock handling
    if (std::fabs(pitch - M_PI / 2.0) < 1.0e-3) {
        roll = 0.0;
        yaw  = std::atan2(dcm12, dcm02);
    } else if (std::fabs(pitch + M_PI / 2.0) < 1.0e-3) {
        roll = 0.0;
        yaw  = std::atan2(-dcm12, -dcm02);
    } else {
        roll = std::atan2(dcm21, dcm22);
        yaw  = std::atan2(dcm10, dcm00);
    }

    euler.x() = roll;
    euler.y() = pitch;
    euler.z() = yaw;

    return euler;
}

// Quaternion -> Euler via 2-3-1 (Y-Z-X) sequence,
// then mapped into roll/pitch/yaw slots as if 3-2-1.
chrono::ChVector3d QuaternionToEulerAnglesRPY_321_from231(const chrono::ChQuaterniond& q)
{
    // Output will be: x=roll_321_like, y=pitch_321_like, z=yaw_321_like
    chrono::ChVector3d euler_rpy;

    // Chrono quaternion: (e0,e1,e2,e3) = (w,x,y,z)
    double a = q.e0();
    double b = q.e1();
    double c = q.e2();
    double d = q.e3();

    // Precompute products
    double aa = a * a;
    double ab = a * b;
    double ac = a * c;
    double ad = a * d;
    double bb = b * b;
    double bc = b * c;
    double bd = b * d;
    double cc = c * c;
    double cd = c * d;
    double dd = d * d;

    // Full DCM (body -> inertial) from quaternion
    double dcm00 = aa + bb - cc - dd;
    double dcm01 = 2.0 * (bc - ad);
    double dcm02 = 2.0 * (ac + bd);

    double dcm10 = 2.0 * (bc + ad);
    double dcm11 = aa - bb + cc - dd;
    double dcm12 = 2.0 * (cd - ab);

    double dcm20 = 2.0 * (bd - ac);
    double dcm21 = 2.0 * (ab + cd);
    double dcm22 = aa - bb - cc + dd;

    // --- 2-3-1 (Y-Z-X) extraction ---
    // Angles: alpha about 2, beta about 3, gamma about 1
    // Using a consistent set: beta = asin(-C21)
    double s = -dcm21;
    if (s < -1.0) s = -1.0;
    if (s >  1.0) s =  1.0;

    double beta  = std::asin(s);    // middle (3-axis) angle
    double alpha, gamma;

    // Gimbal lock around |beta| = 90 deg
    if (std::fabs(beta - M_PI / 2.0) < 1.0e-3) {
        gamma = 0.0;
        alpha = std::atan2(dcm20, dcm22);
    } else if (std::fabs(beta + M_PI / 2.0) < 1.0e-3) {
        gamma = 0.0;
        alpha = std::atan2(-dcm20, -dcm22);
    } else {
        alpha = std::atan2(dcm20, dcm22);   // rotation about axis-2
        gamma = std::atan2(dcm01, dcm11);   // rotation about axis-1
    }

    // Map 2-3-1 angles into 3-2-1-style roll/pitch/yaw slots for plotting:
    //   roll_321_like  <- gamma (about body-1)
    //   pitch_321_like <- beta  (about body-3)
    //   yaw_321_like   <- alpha (about body-2)
    double roll_321_like  = gamma;
    double pitch_321_like = -1.0 * alpha;
    double yaw_321_like   = -1.0 * beta;

    euler_rpy.x() = roll_321_like;
    euler_rpy.y() = pitch_321_like;
    euler_rpy.z() = yaw_321_like;
    return euler_rpy;
}


} // namespace _conversions_


// ============================================================================================================
// _compute_ : General-purpose computation
// ============================================================================================================
namespace _compute_
{

// Creates and returns a 1-dimensional interpolator
NDInterpolator_1_ML* create1DInterpolator(const std::vector<double>& grid, const std::vector<double>& values)
{
    // ---- linterp expects pointers, not references, so we cast away constness ----
    double* grids_begin[1] = { const_cast<double*>(&grid.front()) };
    int grid_len[1] = { static_cast<int>(grid.size()) };

    // ---- Create interpolator over range [begin, end). Use &front() + size() for safety ----
    return new NDInterpolator_1_ML(grids_begin, grid_len,
                                   const_cast<double*>(&values.front()),
                                   const_cast<double*>(&values.front()) + values.size());
}

// Function returns the max value in a vector array
double vecmax(const std::vector<double>& v)
{
    if (v.empty()) {
        ::_acsl_::_message_::SIMULATOR_ERROR("[SIMHLP]: VECMAX: INPUT VECTOR IS EMPTY");
    }
    
    return *std::max_element(v.begin(), v.end());
}

// Wraps a double angle `alpha` to the interval [-pi, pi]
double wrapAngleToMinusPiAndPi(double alpha)
{
    return alpha - 2.0 * M_PI * std::floor((alpha + M_PI) / (2.0 * M_PI));
}

// Makes the yaw angular error continuous considering the discontinuity at +/- pi
double makeYawAngularErrorContinuous(double yaw, double user_defined_yaw)
{
    user_defined_yaw = wrapAngleToMinusPiAndPi(user_defined_yaw);

    double raw_error = yaw - user_defined_yaw;
    double continuous_error;

    if (std::abs(raw_error) >= M_PI && std::abs(raw_error) < 2.0 * M_PI && raw_error < 0.0) {
        continuous_error = std::fmod(raw_error, M_PI) + M_PI;
    }
    else if (std::abs(raw_error) >= M_PI && std::abs(raw_error) < 2.0 * M_PI && raw_error > 0.0) {
        continuous_error = std::fmod(raw_error, M_PI) - M_PI;
    }
    else {
        continuous_error = std::fmod(raw_error, M_PI);
    }

    return continuous_error;
}

// Incrementally unwrap yaw by removing ±2*pi jumps in the difference.
double unwrapPsiSimple(double psi_wrapped, SimplePsiUnwrapState &state)
{
    constexpr double pi    = M_PI;
    constexpr double twoPi = 2.0 * M_PI;

    // First sample: just pass through
    if (!state.initialized) {
        state.prevUnwrapped = psi_wrapped;
        state.initialized   = true;
        return psi_wrapped;
    }

    double dp = psi_wrapped - state.prevUnwrapped;

    // Bring the increment into [-pi, pi]
    if (dp > pi) {
        dp -= twoPi;
    } else if (dp < -pi) {
        dp += twoPi;
    }

    double psi_unwrapped = state.prevUnwrapped + dp;
    state.prevUnwrapped  = psi_unwrapped;
    return psi_unwrapped;
}

// Regularizes biplane Euler angles using quadcopter angles near 90° pitch.
// eul_b: biplane Euler (roll, pitch, yaw) in radians
// eul_q: quadcopter Euler (roll, pitch, yaw) in radians
chrono::ChVector3d RegularizeCardanXYZ(const chrono::ChVector3d& eul_b,
                                       const chrono::ChVector3d& eul_q)
{
    chrono::ChVector3d e = eul_b;  // (phi_b, theta_b, psi_b)

    const double theta_max_deg = 90.0;      // target singular pitch
    const double tol_deg       = 20.0;      // ±10° band
    const double deg2rad       = chrono::CH_PI / 180.0;
    const double rad2deg       = 180.0 / chrono::CH_PI;

    // Handle explicit NaN pitch
    if (std::isnan(e.y())) {
        e.x() = 0.0;
        e.y() = theta_max_deg * deg2rad;
        e.z() = 0.0;
        return e;
    }

    // Convert pitch to degrees for thresholding
    double theta_b_deg = e.y() * rad2deg;

    // If biplane pitch is near +90 deg, override roll/yaw using quadcopter angles
    if (std::abs(theta_b_deg - theta_max_deg) < tol_deg) {
        // biplane_roll  = -quad_yaw
        // biplane_yaw   =  quad_roll
        e.x() = -eul_q.z();  // roll_b
        e.z() =  eul_q.x();  // yaw_b
        return e;
    }

    // Otherwise, leave biplane Euler as computed
    return e;
}


// This function computes the n midpoints for n segements and returns them.
// Primarily used to compute the aerodynamic center points for distributed aerodynamics.
// NOTE: If using for computing the aero points, p1 and p2 are already in NED frame from
//       the solidworks plugin exported file.
std::vector<chrono::ChVector3d> ComputeSegmentMidpoints(
    const chrono::ChVector3d& p1,
    const chrono::ChVector3d& p2,
    int n)
{
    if (n <= 0) {
        throw std::invalid_argument("ComputeSegmentMidpoints: n must be > 0");
    }

    std::vector<chrono::ChVector3d> mids;
    mids.reserve(static_cast<std::size_t>(n));

    for (int k = 0; k < n; ++k) {
        // Parameter value at midpoint of k-th subsegment in [0, 1]
        // Subsegments: [0,1/n], [1/n,2/n], ..., [(n-1)/n,1]
        // Midpoint: λ_k = (2k + 1) / (2n)
        double lambda = (2.0 * k + 1.0) / (2.0 * n);

        // Linear interpolation: p(λ) = p1 + λ (p2 - p1)
        chrono::ChVector3d p = p1 + (p2 - p1) * lambda;
        mids.push_back(p);
    }

    return mids;
}

Eigen::Quaterniond qFromTwoVectors(const Eigen::Vector3d& u_in,
                                   const Eigen::Vector3d& v_in)
{
    // u = u / norm(u);
    Eigen::Vector3d u = u_in.normalized();

    // v = v / norm(v);
    Eigen::Vector3d v = v_in.normalized();

    // c = dot(u, v);
    double c = u.dot(v);

    // If c < -1 + 1e-10: nearly opposite vectors (angle ≈ pi)
    if (c < -1.0 + 1e-10) {
        // MATLAB: axis = null(u.'); pick any axis ⟂ u
        // Here we construct such an axis via cross product with a basis vector.
        Eigen::Vector3d axis = u.cross(Eigen::Vector3d::UnitX());
        if (axis.squaredNorm() < 1e-6) {
            // If u is (almost) parallel to x, use y instead.
            axis = u.cross(Eigen::Vector3d::UnitY());
        }
        axis.normalize();

        // q = qFromAxisAngle(axis, pi);
        // Use Eigen's AngleAxisd to build a 180-degree rotation quaternion.
        Eigen::AngleAxisd aa(M_PI, axis);
        Eigen::Quaterniond q(aa);
        q.normalize();  // should already be unit, but keep for robustness
        return q;
    }

    // w = cross(u, v);
    Eigen::Vector3d w = u.cross(v);

    // q0 = 1 + c;
    double q0 = 1.0 + c;

    // qv = w;  q = [q0; qv];
    Eigen::Quaterniond q(q0, w.x(), w.y(), w.z());

    // Normalize q safely (like the MATLAB code)
    double nq = q.norm();
    if (nq < 1e-8) {
        // q = [1;0;0;0]; identity quaternion
        q = Eigen::Quaterniond::Identity();
    } else {
        q.normalize();
    }

    return q;
}


} // namespace _compute_


// ============================================================================================================
// _serialize_ : Serialize data to output from the simulator
// ============================================================================================================
namespace _serialize_
{

// Function takes in any 3 equal length vectors and returns a ChVector3d vector array with the data arranged 
// in the same order as the input.
std::vector<chrono::ChVector3d> serialize2ChVector3d(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    const std::vector<double>& zs)
{
    size_t N = xs.size();
    std::vector<chrono::ChVector3d> out;
    out.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        out.emplace_back(xs[i], ys[i], zs[i]);
    }
    return out;
}

} // namespace _serialize_


// ============================================================================================================
// _deserialize_ : De-serialize data to read from file
// ============================================================================================================
namespace _deserialize_
{

// Given the column label, it extracts all the data as a vector
std::vector<double> GetCSVColumn(const rapidcsv::Document& doc, const std::string& label)
{
    // Quickly extract a single column from CSV into std::vector<T>
    return doc.GetColumn<double>(label);
}

// Function checks if the given file exists in the path
void FileExists(const std::string& filepath)
{
    if (!std::filesystem::exists(filepath)) {
        ::_acsl_::_message_::SIMULATOR_ERROR("[SIMHLP]: FILE DOES NOT EXIST" + filepath);
    }
}

// Extract Eigen::MatrixXd from JSON array of arrays
Eigen::MatrixXd jsonToMatrixXd(const nlohmann::json& jsonMatrix, int rows, int cols) {
    Eigen::MatrixXd matrix(rows, cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            matrix(i, j) = jsonMatrix[i][j];
    return matrix;
}

// Extract Eigen::MatrixXd from JSON with scaling
// Expects JSON like: { "matrix": [[...],...], "scaling_coef": value }
Eigen::MatrixXd jsonToScaledMatrixXd(const nlohmann::json& jsonMatrix, int rows, int cols) {
    double scaling_coef = jsonMatrix["scaling_coef"];
    const auto& arr = jsonMatrix["matrix"];
    Eigen::MatrixXd matrix(rows, cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            matrix(i, j) = arr[i][j];
    matrix *= scaling_coef;
    return matrix;
}

} // namespace _deserialize_


// ============================================================================================================
// _visualize_ : Helper functions for visualization in the irrlicht environment
// ============================================================================================================
namespace _visualize_
{

// Creates a NURB line for visualization, with conversion from NED to Chrono coordinates
std::shared_ptr<chrono::ChVisualShapeLine> createNurbsVisual(
    std::vector<chrono::ChVector3d>& controlpoints,
    int order,
    const chrono::ChColor& color)
{
    // If only two points are given, insert (order+1) midpoints between them
    std::vector<chrono::ChVector3d> cps;
    cps.reserve(std::max<std::size_t>(controlpoints.size(), 2));

    if (controlpoints.size() == 2) {
        const auto& p0 = controlpoints[0];
        const auto& p1 = controlpoints[1];

        int N = order + 1;  // number of interior points
        cps.push_back(p0);
        for (int k = 1; k <= N; ++k) {
            double s = static_cast<double>(k) / static_cast<double>(N + 1);
            chrono::ChVector3d pk = (1.0 - s) * p0 + s * p1;
            cps.push_back(pk);
        }
        cps.push_back(p1);
    } else {
        cps = controlpoints;  // use as-is for >= 3 points
    }

    // Create and initialize the NURBS curve with processed points
    auto nurbs = chrono_types::make_shared<chrono::ChLineNurbs>();
    nurbs->Setup(order, cps);

    // Wrap in visual asset
    auto nurbsasset = chrono_types::make_shared<chrono::ChVisualShapeLine>();
    nurbsasset->SetLineGeometry(nurbs);
    nurbsasset->SetColor(color);

    return nurbsasset;
}


// Creates a frame using lines for visualization of the markers.
void createMarkerFrameVisual(chrono::ChMarker& marker, double len, double thk) {
    // Get marker position and rotation in absolute coordinates
    chrono::ChVector3d marker_pos = marker.GetPos();
    chrono::ChQuaterniond marker_rot = marker.GetRot();

    // Get the body the marker is attaced to 
    chrono::ChBody* body = marker.GetBody();

    // x-axis (red) ------------------------------------------------
    auto x_line_geom = chrono_types::make_shared<chrono::ChLineSegment>();
    chrono::ChVector3d x_start = marker_pos;
    chrono::ChVector3d x_end = marker_pos + marker_rot.Rotate(
                                            chrono::ChVector3d(len, 0, 0));
    x_line_geom->pA = x_start; x_line_geom->pB = x_end;
    
    auto x_line = chrono_types::make_shared<chrono::ChVisualShapeLine>();
    x_line->SetLineGeometry(x_line_geom);
    x_line->SetColor(chrono::ChColor(1.0f, 0.0f, 0.0f));
    x_line->SetThickness(thk);
    
    // y-axis (green) ----------------------------------------------
    auto y_line_geom = chrono_types::make_shared<chrono::ChLineSegment>();
    chrono::ChVector3d y_start = marker_pos;
    chrono::ChVector3d y_end = marker_pos + marker_rot.Rotate(
                                            chrono::ChVector3d(0, len, 0));
    y_line_geom->pA = y_start; y_line_geom->pB = y_end;
    
    auto y_line = chrono_types::make_shared<chrono::ChVisualShapeLine>();
    y_line->SetLineGeometry(y_line_geom);
    y_line->SetColor(chrono::ChColor(0.0f, 1.0f, 0.0f));
    y_line->SetThickness(thk);
    
    // z-axis (blue) -----------------------------------------------
    auto z_line_geom = chrono_types::make_shared<chrono::ChLineSegment>();
    chrono::ChVector3d z_start = marker_pos;
    chrono::ChVector3d z_end = marker_pos + marker_rot.Rotate(
                                            chrono::ChVector3d(0, 0, len));
    z_line_geom->pA = z_start; z_line_geom->pB = z_end;
    
    auto z_line = chrono_types::make_shared<chrono::ChVisualShapeLine>();
    z_line->SetLineGeometry(z_line_geom);
    z_line->SetColor(chrono::ChColor(0.0f, 0.0f, 1.0f));
    z_line->SetThickness(thk);

    // Add all the shapes to the parent body of the marker
    body->AddVisualShape(x_line);
    body->AddVisualShape(y_line);
    body->AddVisualShape(z_line);
}

} // namespace _visualize_

} // namespace _shared_