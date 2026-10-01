/***********************************************************************************************************************
 * Copyright (c) 2026 Haoran Wang, Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
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
 * File:        rkhs-utilities.hpp
 * Author:      Haoran Wang
 * Date:        September 1, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of various RKHS Functions.
 * 
 * GitHub:    https://github.com/haoran9vt/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#ifndef RKHS_UTILITIES_HPP_
#define RKHS_UTILITIES_HPP_

namespace _shared_
{
  namespace _rkhs_functions_
  {
    // Function that calculate operator kernel function for the RKHS adaptive law
    template <typename Der1, typename Der2>
    inline Eigen::MatrixXd kernel_function(const Der1& basis_center,
                                          const Der2& state,
                                          const std::string& kernel_type,
                                          const double kernel_l)
    {
        double dist = (basis_center - state).norm();

        if (kernel_type == "GAUSSIAN") {
            return Eigen::MatrixXd::Identity(3, 3)
                * std::exp(-1.0 / (2.0 * std::pow(kernel_l, 2))
                            * std::pow(dist, 2));
        }
        else {
            ::_acsl_::_message_::SIMULATOR_ERROR(
                "[SIMCTL]: KERNEL TYPE NOT SUPPORTED. "
                "SUPPORTED TYPE IS 'GAUSSIAN'.");

            return Eigen::MatrixXd::Identity(3, 3);
        }
    }          

    // Function that evaluate kernel matrix for the RKHS adaptive law
    template <typename Der1, typename Der2>
    inline Eigen::MatrixXd knl_Xi_N_evaluation(const Der1& center_list,
                                              const Der2& state, 
                                              const std::string& kernel_type, 
                                              const double kernel_l)
                                              // Note that state must be in [X_dot,Y_dot,Z_dot] format
    {
        Eigen::MatrixXd knl_Xi_N_Result(81,3);

        for (int i = 0; i < 27; i++) {
            Eigen::VectorXd currnet_center = center_list.col(i);
            knl_Xi_N_Result.block<3,3>(i*3,0) = kernel_function(currnet_center,
                                                                state, 
                                                                kernel_type, 
                                                                kernel_l);
        }

        return knl_Xi_N_Result;
    }

  }
}

#endif  // RKHS_UTILITIES_HPP_