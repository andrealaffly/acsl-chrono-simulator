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

 /**********************************************************************************************************************
 * File:        sim-tailsitter.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 23, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived UAV class for the simulated tailsitter.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_TAILSITTER_HPP_
#define SIM_TAILSITTER_HPP_

#include "sim-uav.hpp"

namespace _acsl_
{

namespace _tailsitter_
{

// Define the number of propellers for this class of UAV
static constexpr int NOP = 4;

// ============================================================================
// simtailsitter class
// Inherits from the templated base UAV simulator class, specifying nop = 4,
// meaning this is a fixed 4‑propeller variant of the UAV.
// ============================================================================
class simtailsitter : public _acsl_::_uav_::simuav<NOP> 
{
public:
    // ------------------------------------------------------------------------
    // Constructor for simtailsitter
    //
    // Parameters:
    //   sys - a reference to the Chrono physical system (ChSystemNSC)
    //         where bodies, joints, and simulation settings live.
    // 
    // This constructor *must* forward 'sys' to the base class constructor
    // for simuav<4>, because that class stores a reference to ChSystemNSC.
    // ------------------------------------------------------------------------
    simtailsitter(chrono::ChSystemNSC& sys) // <-- Call the base class constructor, Set the wing data.
        : _acsl_::_uav_::simuav<NOP>(sys, ::_acsl_::_uav_::_aerofoil_::AirFoilType::NACA0012)  
    {
        // All initialization specific to 'simtailsitter' goes here.
        // NOTE: THE ORDER OF THESE FUNCTIONS MATTER. YOU CAN IGNORE THE UNIQUE FNS
        //       IF YOU ARE TRYING TO REPLICATE THIS CODE FOR ANOTHER PLATFORM.    
        // 
        // These are:
        //   - Setting the name of the platform
        //   - Setup the inertial reference frame
        //   - Setup the shaped directory for the drone
        //   - Call the function to setup the chassis of the uav
        //   - Initiate the entire UAV and add it to the physics system

        SetUAVPlatformName("tailsitter");                                          // <- Baseclass function
        SetupInertialNEDFrame();                                                   // <- Baseclass function
        SetUAVShapesDir("../chrono-assets/models/tailsitter/tailsitter_shapes/");  // <- Baseclass function
        
        ConfigureTAILSITTERChassis();                                 // <- Compulsary derived class function
        ConfigureTAILSITTERProps();                                   // <- Compulsary derived class function
        ConfigureTAILSITTERLinks();                                   // <- Compulsary derived class function
        ConfigureTAILSITTERMotors();                                  // <- Compulsary derived class function
        ConfigureTAILSITTERAerodynamics();                            // <- Unique derived class function
        // ConfigureTAILSITTERPayload();                                 // <- Unique derived class function
        
        AddUAVToSystem();                                             // <- Baseclass function

    }

    // ------------------------------------------------------------------------
    // You need to add additional functions below that are unique to simqrbp.
    // These functions include some compulsary functions and some unique to 
    // the platform itself.
    // NOTE: YOU CAN CHANGE THE NAME OF THE COMPULSARY FUNCTIONS TO THE NAME 
    //       OF YOUR PARTICULAR PLATFORM BUT THEY SHOULD BE CLEARLY DEFINED 
    //       (IF YOU ARE TRYING TO REPLICATE THIS CLASS FOR ANOTHER PLATFORM)
    // 
    // The comulsary functions are:
    //   - Function to setup the chassis
    //   - Function to setup the propellers
    //   - Function to setup all the links for the platform
    //   - Function to setup all the motors for the platform
    //   - Function to setup all the uav aerodynamics properties
    //
    // The unique functions are:
    //   - Function to setup the aerodynamics
    //   - Function to setup the payload for the UAV
    // ------------------------------------------------------------------------
    void ConfigureTAILSITTERChassis();
    void ConfigureTAILSITTERProps();
    void ConfigureTAILSITTERLinks();
    void ConfigureTAILSITTERMotors();
    void ConfigureTAILSITTERAerodynamics();
    void ConfigureTAILSITTERPayload();
};




} // namespace _tailsitter_

} // namespace _acsl_

#endif // SIM_TAILSITTER_HPP_