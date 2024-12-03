/***********************************************************************************************************************
 * Copyright (c) 2024 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
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
 * File:        sim_config.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 6, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: File to configure the environments, and platforms.
 * 
 * GitHub:    https://github.com/andrealaffly/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#ifndef SIM_CONFIG_HPP_
#define SIM_CONFIG_HPP_

/// -------------------------------------- SETTING DEFINITIONS -------------------------------------- ///
// Use the platform definitions that you used in the CMakeLists
#define QRBP 0
#define X8 1

// Use the environment definitions that you used in the CMakeLists
#define SIMPLE_ENV 0

/// -------------------------------------- PICKED DEFINITIONS -------------------------------------- ///
// PICKED Platform 
#define SIM_PLATFORM QRBP

// PICKED environmennt
#define SIM_ENVIRONMENT SIMPLE_ENV

/// ------------------------------------------------------------------------------------------------ ///

/// ------ INITIALIZING THE PLATFORM ------ ///
#if SIM_PLATFORM == QRBP
    #include "sim_qrbp.hpp"
    using _sim_platform_ = _platforms_::_uav_::_qrbp_::qrbp;
#endif

/// ------ INITIALIZING THE ENVIRONEMNT ------ ///
#if SIM_ENVIRONMENT == SIMPLE_ENV
    #include "simple_environment.hpp"
    using _sim_environment_ = _environments_::simple_environment;
#endif

#endif // SIM_CONFIG_HPP_