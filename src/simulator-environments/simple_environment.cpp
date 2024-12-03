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
 * File:        simple_environment.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 6, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class definition for a simple floor environment.
 * 
 * GitHub:    https://github.com/andrealaffly/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#include "simple_environment.hpp"

namespace _environments_
{

simple_environment::simple_environment(chrono::ChSystem& system) : system_(system)
{
    ImportEnvironment(this->system_);
}

simple_environment::~simple_environment() {}

void simple_environment::ImportEnvironment(chrono::ChSystem& system)
{
    auto floor = chrono_types::make_shared<chrono::ChBody>();                       // Create a body
    floor->SetFixed(SET_FIXED);                                                     // Fixed body
    auto floor_mat = chrono_types::make_shared<chrono::ChContactMaterialSMC>();     // Create contact material
    
    // floor_mat->SetYoungModulus(2e5f);
    // floor_mat->SetFriction(0.4f);
    // floor_mat->SetRestitution(0.1f);

    auto floor_shape = chrono_types::make_shared<chrono::ChCollisionShapeBox>(floor_mat,
                                                SIMPLE_ENV_X, SIMPLE_ENV_Y, SIMPLE_ENV_Z);

    floor->AddCollisionShape(floor_shape, chrono::ChFrame<>(chrono::ChVector3d(SIMPLE_ENV_DELTA_X,
                                                                               SIMPLE_ENV_DELTA_Y,
                                                                               SIMPLE_ENV_DELTA_Z), 
                                                                               chrono::QUNIT));
    
    floor->EnableCollision(ENABLE_COLLISION);                                       // Enable the collision on the floor

    auto boxfloor = chrono_types::make_shared<chrono::ChVisualShapeBox>(SIMPLE_ENV_X,
                                                                        SIMPLE_ENV_Y,
                                                                        SIMPLE_ENV_Z);

    boxfloor->SetColor(chrono::ChColor(SIMPLE_ENV_R, SIMPLE_ENV_G, SIMPLE_ENV_B));
    floor->AddVisualShape(boxfloor,chrono::ChFrame<>(chrono::ChVector3d(SIMPLE_ENV_DELTA_X,
                                                                        SIMPLE_ENV_DELTA_Y,
                                                                        SIMPLE_ENV_DELTA_Z), 
                                                                        chrono::QUNIT));
    system.Add(floor);                                                                                                                                             
}
    
}   // namespace _environments_