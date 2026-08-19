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
 * File:        adaptive-laws.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        January 17, 2025
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of various adaptive laws.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef ADAPTIVE_LAWS_HPP_
#define ADAPTIVE_LAWS_HPP_

namespace _shared_
{

namespace _adaptive_laws_
{

  // Classical MRAC Adaptive Laws.
  // See matlab-scripts/regressor-vector-computation/MRAC_for_DCMs_in_the_Outer_Loop.pdf for the theory
  template <typename Der1, typename Der2, typename Der3>
  inline auto AdaptiveLawClassical(const Der1& Gamma,
                                   const Der2& pi_vec,
                                   const Der3& eTPB)
  {
    // Adaptive law
    auto K_hat_dot = Gamma * pi_vec * eTPB;

    // Return the derivative of the adaptive gain
    return K_hat_dot;
  }

  // Classical MRAC Adaptive Law for DCMs. R is a direction-cosine-matrix
  template <typename Der1, typename Der2, typename Der3, typename Der4>
  inline auto AdaptiveLawDCMClassical(const Der1& Gamma,
                                      const Der2& pi_vec,
                                      const Der3& eTPB,
                                      const Der4& R)
  {
    // Adaptive law
    auto K_hat_dot = Gamma * pi_vec * eTPB * R;

    // Return the derivative of the adaptive gain
    return K_hat_dot;
  }                                      


  // Robust MRAC - Adaptive law formula equipped with the dead-zone modification and e-modificiation
  template <typename Der1, typename Der2, typename Der3, typename Der4>
  inline auto AdaptiveLawDeadZoneEMod(const Der1& Gamma,
                                      const double dead_zone_value,
                                      const Der2& pi_vec,
                                      const Der3& eTPB,
                                      const double sigma_gain,
                                      const Der4& K_hat_state) 
  {
    // Compute the norm of the e'PB
    auto eTPB_norm = eTPB.norm();

    // Compute the adaptive laws
    auto K_hat_dot = Gamma * dead_zone_value * ( pi_vec * eTPB - sigma_gain * eTPB_norm * K_hat_state);

    // Return the derivative of the adaptive gain
    return K_hat_dot;
  }

  // Robust MRAC - Adaptive law formula equipped with only the dead-zone modification (no e-modification)
  // For reference: E. Lavretsky, K. Wise, "Robust and Adaptive Control", Springer 2013, Sec. 11.2.1
  template <typename Der1, typename Der2, typename Der3>
  inline auto AdaptiveLawDeadZone(const Der1& Gamma,
                                  const double dead_zone_value,
                                  const Der2& pi_vec,
                                  const Der3& eTPB) 
  {

    // Compute the adaptive law update with deadzone
    auto K_hat_dot = Gamma * dead_zone_value * (pi_vec * eTPB);

    // Return the derivative of the adaptive gain
    return K_hat_dot;
  }

  // Robust MRAC - Adaptive law formula equipped with only the dead-zone modification (no e-modification) for DCM matrices.
  // See also classical implementation of the law above.
  // For reference: E. Lavretsky, K. Wise, "Robust and Adaptive Control", Springer 2013, Sec. 11.2.1
  template <typename Der1, typename Der2, typename Der3, typename Der4>
  inline auto AdaptiveLawDCMDeadZone(const Der1& Gamma,
                                     const double dead_zone_value,
                                     const Der2& pi_vec,
                                     const Der3& eTPB,
                                     const Der4& R) 
  {

    // Compute the adaptive law update with deadzone
    auto K_hat_dot = Gamma * dead_zone_value * (pi_vec * eTPB * R);

    // Return the derivative of the adaptive gain
    return K_hat_dot;
  }

} // namespace _adaptive_laws_

namespace _adaptive_laws_hybrid_
{

  /*
    Store the previous iteration trajectory tracking error.
  */
  template <typename VectorType>
  inline VectorType computePreviousTrajectoryTrackingError(const VectorType& e)
  {
    static_assert(
      static_cast<int>(VectorType::ColsAtCompileTime) == 1,
      "e must be a column vector"
    );
    VectorType e_previous = e;

    return e_previous;
  }

  /*
    Evaluate the s-th element of the series used for reference resetting events.
  */
  inline double evaluateSeriesElement(double s, double alpha)
  {
    return 1.0 / std::pow(s, alpha);
  }

  /*
    Compute the next index s in the convergent series.
    weighted_e_squared = eᵀ(t_j) P e(t_j)
  */
  inline int findSseries(int s_previous, double weighted_e_squared, double alpha)
  {
    // Compute ceil(weighted_e_squared^(-1/alpha))
    double exponent = -1.0 / alpha;
    int candidate = static_cast<int>(
      std::ceil(std::pow(weighted_e_squared, exponent))
    );
    int s = std::max(candidate, s_previous + 1);
    return s;
  }

  /*
    Compute the quadratic form: vectorᵀ * matrix * vector
  */
  template <typename VectorType, typename MatrixType>
  inline double computeQuadraticForm(const VectorType& vector, const MatrixType& matrix)
  {
    static_assert(static_cast<int>(VectorType::ColsAtCompileTime) == 1, "vector must be a column vector");

    static_assert(static_cast<int>(MatrixType::RowsAtCompileTime) == static_cast<int>(MatrixType::ColsAtCompileTime),
      "matrix must be square"
    );

    static_assert(static_cast<int>(VectorType::RowsAtCompileTime) == static_cast<int>(MatrixType::RowsAtCompileTime),
      "Dimension mismatch between vector and matrix"
    );

    double result = (vector.transpose() * matrix * vector).value();

    return result;
  }

  /*
    Update the hybrid summation term.
  */
  inline double updateSummationHybridP(double ePe, double ePe_previous, double summation)
  {
    if (ePe > ePe_previous)
    {
      summation += (ePe - ePe_previous);
    }
    return summation;
  }

  /*
    Reset the hybrid series index when either:
    - it grows beyond a prescribed threshold, or
    - sufficient time has elapsed since the last trajectory reset.
  */
  inline int resetSeriesIfNeeded(
    int s_hybrid,
    double time_now,
    double time_of_last_trajectory_reset,
    double tolerance_time_reset_series
  )
  {
    constexpr int S_MAX_THRESHOLD = 1000000000;
    const double time_since_last_trajectory_reset = time_now - time_of_last_trajectory_reset;

    if (s_hybrid > S_MAX_THRESHOLD || (s_hybrid > 0 && time_since_last_trajectory_reset > tolerance_time_reset_series))
    {
      s_hybrid = 0;
    }
    return s_hybrid;
  }

  /*
    Check the hybrid reset condition and, if satisfied, update:
    - reset time,
    - series index s,
    - reference trajectory,
    - integral term storage.
  */
  template <typename VectorType, typename MatrixType, typename DerivedRef>
  inline std::pair<int, double> checkAndTriggerTrajectoryReset(
    double summation_hybrid_P,
    double time_now,
    double time_of_last_trajectory_reset,
    double ePe,
    double alpha_hybrid_series,
    int s_hybrid,
    const VectorType& e,
    const VectorType& e_previous,
    const MatrixType& Q,
    Eigen::MatrixBase<DerivedRef>& reference_model_state_map,
    Eigen::Ref<Eigen::Matrix<double,1,1>> integral_eQe_map
  )
  {
    static_assert(VectorType::ColsAtCompileTime == 1, "Vectors must be column vectors");
    static_assert(MatrixType::RowsAtCompileTime == MatrixType::ColsAtCompileTime, "Q must be square");

    double integral_eQe = integral_eQe_map(0);

    // std::cout << "integral_eQe: " << integral_eQe << std::endl;
    // std::cout << "summation_hybrid_P: " << summation_hybrid_P << std::endl;

    if (integral_eQe >= summation_hybrid_P)
    {
      // Update reset time
      time_of_last_trajectory_reset = time_now;

      // Update series index
      int s_hybrid_old = s_hybrid;
      s_hybrid = findSseries(s_hybrid_old, ePe, alpha_hybrid_series);

      // Update reference trajectory (jump)
      const double series_element = evaluateSeriesElement(static_cast<double>(s_hybrid), alpha_hybrid_series);
      const double delta_series = ePe - series_element;
      // const double jump_factor = 1.0 - std::sqrt(delta_series / ePe);
      const double jump_factor = 1.0 - std::sqrt(delta_series / ePe);
      const VectorType jump_reference_trajectory = jump_factor * e;
      reference_model_state_map +=  jump_reference_trajectory;

      // Update integral storage term related to (e^T * Q * e)
      const double eQe = computeQuadraticForm(e, Q);
      const double eQe_previous = computeQuadraticForm(e_previous, Q);
      const double delta_integral_eQe = eQe - eQe_previous;
      integral_eQe_map(0) += delta_integral_eQe;

      std::cout << "[HYB] Reset triggered at t = " << time_now << '\n';
      std::cout << "      integral_eQe = " << integral_eQe
                << ", summation_hybrid_P = " << summation_hybrid_P << '\n';

      std::cout << "      s_hybrid (old -> new) = "
                << s_hybrid_old << " -> " << s_hybrid << '\n';

      std::cout << "      ePe = " << ePe
                << ", series_element = " << series_element
                << ", delta_series = " << delta_series
                << ", jump_factor = " << jump_factor << '\n';

      std::cout << "      ||e|| = " << e.norm()
                << ", ||jump_ref|| = " << jump_reference_trajectory.norm() << '\n';

      std::cout << "      eQe = " << eQe
                << ", eQe_previous = " << eQe_previous
                << ", delta_integral_eQe = " << delta_integral_eQe
                << ", integral_eQe (new) = " << integral_eQe_map(0)
                << std::endl;

      std::cout << "checkAndTriggerTrajectoryReset() cond. verified [integral_eQe >= summation_hybrid_P]" << std::endl;
    }

    return {s_hybrid, time_of_last_trajectory_reset};
  }

  /*
    Execute one hybrid MRAC step:
    - Update quadratic forms and summation term
    - Handle series reset logic
    - Trigger trajectory reset if condition is met
  */
  template <typename VectorType, typename MatrixType, typename DerivedRef>
  inline std::tuple<Eigen::Matrix<double,1,1>, double, int, double> runHybridStep(
    const VectorType& e,
    const VectorType& e_previous,
    const MatrixType& P,
    const MatrixType& Q,
    double summation_hybrid_P,
    int s_hybrid,
    double time_of_last_trajectory_reset,
    double alpha_hybrid_series,
    double time_now,
    double tolerance_time_reset_series_hybrid,
    Eigen::MatrixBase<DerivedRef>& reference_model_state_map,
    Eigen::Ref<Eigen::Matrix<double,1,1>> integral_eQe_map,
    bool& first_controller_loop
  )
  {
    static_assert(VectorType::ColsAtCompileTime == 1, "Vectors must be column vectors");
    static_assert(MatrixType::RowsAtCompileTime == MatrixType::ColsAtCompileTime, "Matrices must be square");

    // Compute quadratic terms
    Eigen::Matrix<double,1,1> eQe;
    eQe(0,0) = computeQuadraticForm(e, Q);
    const double ePe = computeQuadraticForm(e, P);
    const double ePe_previous = computeQuadraticForm(e_previous, P);
    // Update hybrid summation term
    summation_hybrid_P = updateSummationHybridP(ePe, ePe_previous, summation_hybrid_P);

    if (!first_controller_loop)
    {
      // Reset series index if required
      s_hybrid = resetSeriesIfNeeded(
        s_hybrid,
        time_now,
        time_of_last_trajectory_reset,
        tolerance_time_reset_series_hybrid
      );

      // Check reset condition
      std::tie(s_hybrid, time_of_last_trajectory_reset) = checkAndTriggerTrajectoryReset(
        summation_hybrid_P,
        time_now,
        time_of_last_trajectory_reset,
        ePe,
        alpha_hybrid_series,
        s_hybrid,
        e,
        e_previous,
        Q,
        reference_model_state_map,
        integral_eQe_map               
      );
    }

    first_controller_loop = false;

    return {eQe, summation_hybrid_P, s_hybrid, time_of_last_trajectory_reset};
  }

} // namespace _adaptive_laws_hybrid_

} // namespace _shared_

#endif  // ADAPTIVE_LAWS_HPP_