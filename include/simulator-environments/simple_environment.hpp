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
 * File:        simple_environment.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 6, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class declaration for a simple floor environment.
 * 
 * GitHub:    https://github.com/andrealaffly/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#ifndef SIMPLE_ENVIRONMENT_HPP_
#define SIMPLE_ENVIRONMENT_HPP_

inline constexpr double SIMPLE_ENV_X = 20;        // Size of floor in X
inline constexpr double SIMPLE_ENV_Y = 20;        // Size of floor in Y
inline constexpr double SIMPLE_ENV_Z = 1;         // Size of floor in Z
inline constexpr double SIMPLE_ENV_DELTA_X = 0;   // Distance from the origin to initialize the floor
inline constexpr double SIMPLE_ENV_DELTA_Y = 0;   // Distance from the origin to initialize the floor
inline constexpr double SIMPLE_ENV_DELTA_Z = 0.645;   // Distance from the origin to initialize the floor

inline constexpr bool SET_FIXED = true;           // Is the environment fixed
inline constexpr bool ENABLE_COLLISION = true;    // Is the environment collision enabled

inline constexpr float SIMPLE_ENV_R = 50.2f;       // Simple environment color R
inline constexpr float SIMPLE_ENV_G = 50.2f;       // Simple environment color G
inline constexpr float SIMPLE_ENV_B = 50.2f;       // Simple environment color B

// Chrono physics includes
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/assets/ChVisualShapeBox.h"

namespace _environments_
{

class simple_environment
{
    public:
        // Constructor that accepts the chrono system
        simple_environment(chrono::ChSystem& system);

        // Destructor
        ~simple_environment();

        // Getter for system_
        chrono::ChSystem& GetSystem() const { return system_; }

    private:
        // Stored refrences or pointers to function inputs
        chrono::ChSystem& system_;

        // Function that sets up and adds the floor to the system.
        void ImportEnvironment(chrono::ChSystem& system);
};

}   // namespace _environments_

#endif // SIMPLE_ENVIRONMENT_HPP_


