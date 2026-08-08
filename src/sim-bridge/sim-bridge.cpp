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
 * File:        sim-bridge.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: The brains of the acsl physics simulator. This file is responsible for running the simulation mode
 *              (HIL, SIL, MIL simulations) and loading the appropirate m_uav into the simulation.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-bridge.hpp"

namespace _acsl_
{

namespace _bridge_
{

// =====================================================================================================================
// ConfigureSimulatorFromConfig()
//
// Purpose:
//   Loads the main simulator configuration from sim-config.yaml, dynamically selects and instantiates:
//     - The simulation mode (e.g., SIL/HIL setting)
//     - The UAV platform to fly
//     - The environment locale (scene) to use
//   This function configures the simbridge by parsing the YAML config and relying on registry structs
//   (platforms and locales) to update, validate, and instantiate the appropriate UAV and environment classes.
//
// Workflow:
//   1. Open the simulator configuration file (`sim_config_filename`).
//   2. Validate file existence and handle errors if missing.
//   3. Parse the YAML configuration into memory using fkyaml.
//   4. Extract the simulation mode (software/hardware-in-the-loop) and debugging settings.
//   5. Populate all available environment locales from YAML and registry.
//      - Validates that one and only one locale is set true.
//      - Instantiates the selected locale via factory (createSelectedLocale).
//   6. Populate all available UAV platforms by dynamically iterating the YAML config and registry.
//      - Validates that one and only one platform is set true.
//      - Instantiates the selected UAV via factory (createSelectedUAV).
//   7. Read the trajectory module parameters in and store it for instantiation.
//   8. Read the controller module paramters ina nd store it for instantiation.
//   9. Log success messages about the loaded config, mode, and UAV/locale instantiation.
//
// Notes:
//   - There is no hardcoding of platform or environment names in this function: all lookups and instantiation
//     logic are handled by iterating struct registries and using the YAML keys.
//   - To add a new UAV/platform or environment/locale, only sim-platforms.hpp, sim-locales.hpp, and sim-config.yaml
//     need updating — not this function.
//   - `m_sys` refers to the physics system object; `m_uav` and `m_env` are unique_ptrs owned by simbridge.
//   - Errors (e.g., missing config, invalid selection) are reported via SIMULATOR_ERROR and halt execution.
// =====================================================================================================================
void simbridge::ConfigureSimulatorFromConfig()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Open the simulator configuration YAML file
    // ------------------------------------------------------------------------
    std::ifstream ifs(sim_config_filename);

    // ------------------------------------------------------------------------
    // STEP 2 – Check that the file exists
    // ------------------------------------------------------------------------
    if (!ifs) {
        _message_::SIMULATOR_ERROR(
            "[SIMBRG]: COULD NOT OPEN SIMULATOR CONFIG FILE: " + sim_config_filename
        );
    }

    // ------------------------------------------------------------------------
    // STEP 3 – Deserialize the YAML configuration into memory
    // ------------------------------------------------------------------------
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);

    // ------------------------------------------------------------------------
    // STEP 4 – Extract simulation mode and debuging settings
    // ------------------------------------------------------------------------
    this->efsl = config_file["mode"]["enable_flightstack_loop"].as_bool();
    this->enable_biplane_frame_data = config_file["mode"]["enable_biplane_frame_data"].as_bool();
    this->log2file = config_file["debug"]["log_physics"].as_bool();
    this->log2terminal = config_file["debug"]["terminal"].as_bool();
    this->sim_debug_stop = config_file["debug"]["sim_debug_stop"].as_bool();
    this->sim_stop_time = static_cast<double>(config_file["debug"]["sim_stop_time"].as_float()); 
    this->developer_mode = config_file["debug"]["developer_mode"].as_bool();
    this->developer_mode_steps = config_file["debug"]["developer_mode_steps"].as_bool();
    this->treat_as_rigid_body = config_file["debug"]["treat_as_rigid_body"].as_bool();

    // ------------------------------------------------------------------------
    // STEP 4.1 – Extract the aerodynamic mode and debugging settings
    // ------------------------------------------------------------------------
    this->enable_chassis_drag = config_file["aerodynamics"]["chassis_drag"].as_bool();
    this->enable_wing_aerodynamics = config_file["aerodynamics"]["wing_aero"].as_bool();
    this->enable_wing_aerodynamics_dbg = config_file["aerodynamics"]["aero_terminal"].as_bool();

    // ------------------------------------------------------------------------
    // STEP 4.2 – Extract the motor disturbance properties
    // ------------------------------------------------------------------------
    this->motor_disturbance.reduce_efficiency = config_file["motor_disturbances"]["reduce_efficiency"].as_bool();
    this->motor_disturbance.affected_motor = config_file["motor_disturbances"]["affected_motor"].as_int();
    this->motor_disturbance.time_of_disturbance = static_cast<double>(config_file["motor_disturbances"]["time_of_disturbance"].as_float());
    this->motor_disturbance.total_efficiency = static_cast<double>(config_file["motor_disturbances"]["total_efficiency"].as_float());

    // ------------------------------------------------------------------------
    // STEP 5 – Populate available_locales dynamically from YAML
    //   asVectorRef() iterates all platform bools by name without hardcoding.
    // ------------------------------------------------------------------------
    for (auto& [name, ref] : available_locals.asVectorRef()) {
        ref = config_file["environment"][name].as_bool();
    }

    // ------------------------------------------------------------------------
    // STEP 5.1 – Validate and get the active locale name
    // ------------------------------------------------------------------------
    this->active_locale = available_locals.validateExclusiveSelection();

    // ------------------------------------------------------------------------
    // STEP 5.2 – Instantiate the selected locale using the factory in locales
    //   - Pass in the Chrono physics system from m_sys for ENV construction.
    //   - Store the ENV in simbridge::locale (std::unique_ptr<simenvbase>).
    // ------------------------------------------------------------------------
    this->m_env = available_locals.createSelectedLocale(this->m_sys.GetPhysicsSystem());

    // ------------------------------------------------------------------------
    // STEP 6 – Populate available_uavs dynamically from YAML
    //   asVectorRef() iterates all platform bools by name without hardcoding.
    // ------------------------------------------------------------------------
    for (auto& [name, ref] : available_uavs.asVectorRef()) {
        ref = config_file["platform"][name].as_bool();
    }

    // ------------------------------------------------------------------------
    // STEP 6.1 – Validate and get the active platform name
    // ------------------------------------------------------------------------
    this->active_platform = available_uavs.validateExclusiveSelection();

    // ------------------------------------------------------------------------
    // STEP 6.2 – Instantiate the selected UAV using the factory in platforms
    //   - Pass in the Chrono physics system from m_sys for UAV construction.
    //   - Store the UAV in simbridge::m_uav (std::unique_ptr<simuavbase>).
    // ------------------------------------------------------------------------
    this->m_uav = available_uavs.createSelectedUAV(this->m_sys.GetPhysicsSystem());

    // ------------------------------------------------------------------------
    // STEP 7 - Load in the trajectory module
    // ------------------------------------------------------------------------
    this->available_trajectories.read(config_file);

    // ------------------------------------------------------------------------
    // STEP 7.1 – Validate and get the active trajectory module name
    // ------------------------------------------------------------------------
    this->active_trajectory = available_trajectories.GetActiveModule();

    // ------------------------------------------------------------------------
    // STEP 7.2 – Instantiate the selected traj using the factory in structure
    //   - Store the module in simbridge::m_trj (std::unique_ptr<trajectorybase>).
    //   - Set the trajectory file from sim-config.yaml
    //   - Initiate the trajectory module
    // ------------------------------------------------------------------------
    this->m_trj = available_trajectories.createSelectedModule();
    this->m_trj->SetFileName(available_trajectories.GetTrajectoryFile());
    this->m_trj->InitiateModule();

    // ------------------------------------------------------------------------
    // STEP 7.3 – If the trajectory is to be visualized, attach it to the NED 
    //            auxilliary refrence body for visualization.
    //            - This means you will always run this logic after the m_uav obj
    //            has been created and make sure the SetupInertialNEDFrame()
    //            was executed prior to this logic.
    // ------------------------------------------------------------------------
    if (this->m_sys.GetVisConfig().render_trajectory) {
        this->m_uav->GetInertialNEDFrameAuxBody()->AddVisualShape(this->m_trj->GetVisualShape());
    }

    // ------------------------------------------------------------------------
    // STEP 8 – Populate available_controllers dynamically from YAML
    //   asVectorRef() iterates all platform bools by name without hardcoding.
    // ------------------------------------------------------------------------
    for (auto& [platform_name, controller_name, ref] : available_controllers.asVectorRef()) {
        ref = config_file["controller"][platform_name][controller_name].as_bool();
    }

    // ------------------------------------------------------------------------
    // STEP 8.1 – Validate and get the active platform & controller name
    // ------------------------------------------------------------------------
    this->active_controller = available_controllers.validateExclusiveSelection(this->active_platform);

    // ------------------------------------------------------------------------
    //  STEP 8.2 – Instantiate the selected controller using the factory
    //    - Pass in the platform for validation 
    //    - Pass in the trajectory object for attaching to the controller
    //    - Pass in the logger for logging.
    // ------------------------------------------------------------------------
    this->m_ctrl = available_controllers.createSelectedController(this->active_platform, this->m_logger, *this->m_trj);

    // ------------------------------------------------------------------------
    // STEP 8.3 – Checks for aerodynamics
    // ------------------------------------------------------------------------
    // If the chassis drag is enabled, check that the necessary parameters are set in the UAV object. If not, throw an error.
    if (this->enable_chassis_drag) {
        if (std::isnan(this->m_uav->GetUAVChassisAerodynamics().chassis_drag_coefficient)) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: CHASSIS DRAG ENABLED BUT CHASSIS DRAG COEFFICIENT IS NOT SET FOR THIS UAV");
        }
        if (std::isnan(this->m_uav->GetUAVChassisAerodynamics().air_density)) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: CHASSIS DRAG ENABLED BUT AIR DENSITY IS NOT SET FOR THIS UAV");
        }
        if (std::isnan(this->m_uav->GetUAVChassisAerodynamics().chassis_body_surface_aera)) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: CHASSIS DRAG ENABLED BUT CHASSIS BODY SURFACE AREA IS NOT SET FOR THIS UAV");
        }
    }

    // If the wing aerodynamics is enabled, check that the necessary parameters are set in the UAV object. If not, throw an error.
    if (this->enable_wing_aerodynamics) {

        const auto& surfaces = this->m_uav->GetUAVAerodynamicSurfaces();

        for (const auto& s: surfaces) 
        {
            if (std::isnan(s.air_density)) {
                _message_::SIMULATOR_ERROR( std::string("[SIMBRG]: WING AERODYNAMICS ENABLED BUT ")
                                                        + "AIR DENSITY IS NOT SET FOR THIS SURFACE: "
                                                        + s.name);
            }
            if (std::isnan(s.aerofoil_span)) {
                _message_::SIMULATOR_ERROR(std::string("[SIMBRG]: WING AERODYNAMICS ENABLED BUT ")
                                           + "WING SPAN IS NOT SET FOR THIS SURFACE: " 
                                           + s.name);
            }
            if (std::isnan(s.aerofoil_chord)) {
                _message_::SIMULATOR_ERROR(std::string("[SIMBRG]: WING AERODYNAMICS ENABLED BUT ")
                                           + "WING CHORD LENGTH IS NOT SET FOR THIS SURFACE: " 
                                           + s.name);
            }
            if (s.aerodynamic_center_frames.empty()) {
                _message_::SIMULATOR_ERROR(std::string("[SIMBRG]: WING AERODYNAMICS ENABLED BUT ")
                                           + "AERODYNAMIC CENTER FRAMES ARE NOT SET FOR THIS SURFACE: " 
                                           + s.name);
            }
            
        }
    }

    // ------------------------------------------------------------------------
    // STEP 9 – Log the loaded config and UAV instantiation
    // ------------------------------------------------------------------------
    _message_::SIMULATOR_INFO("[SIMBRG]: SIMULATOR CONFIG LOADED SUCCESSFULLY");
    _message_::SIMULATOR_INFO("[SIMBRG]:  - DEVELOPER MODE: "               + ::_shared_::_conversions_::bool2string(developer_mode));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - DEVELOPER MODE STEPS: "         + ::_shared_::_conversions_::bool2string(developer_mode_steps));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - HIL / SIL MODE : "              + ::_shared_::_conversions_::bool2string(efsl));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - CHASSIS DRAG ENABLED : "        + ::_shared_::_conversions_::bool2string(enable_chassis_drag));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - AERODYNAMICS ENABLED : "        + ::_shared_::_conversions_::bool2string(enable_wing_aerodynamics));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - ACTIVE PLATFORM: "              + active_platform);
    _message_::SIMULATOR_INFO("[SIMBRG]:  - ACTIVE LOCALE: "                + active_locale);
    _message_::SIMULATOR_INFO("[SIMBRG]:  - ACTIVE TRAJECTORY MODULE: "     + active_trajectory);
    _message_::SIMULATOR_INFO("[SIMBRG]:  - ACTIVE TRAJECTORY FILE: "       + available_trajectories.GetTrajectoryFile());
    _message_::SIMULATOR_INFO("[SIMBRG]:  - ACTIVE CONTROLLER: "            + active_controller.second);
    _message_::SIMULATOR_INFO("[SIMBRG]:  - DISPLAY STATES TO TERMINAL: "   + ::_shared_::_conversions_::bool2string(log2terminal));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - LOG PHYSICS STATES TO FILE: "   + ::_shared_::_conversions_::bool2string(log2file));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - SIMULATION HARD DEBUG STOP: "   + ::_shared_::_conversions_::bool2string(sim_debug_stop));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - SIMULATION STOP TIME (s): "     + std::to_string(sim_stop_time));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - TREAT UAV AS RIGID BODY: "      + ::_shared_::_conversions_::bool2string(treat_as_rigid_body));

}

// =====================================================================================================================
// UpdateVisualizationSystem()
//
// Purpose:
//   Updates and renders the visualization for the simulator according to visualization config settings.
//   Handles scene setup, frame and propeller rendering, COG visualization, shadows, and camera movement.
//   Ensures every requested visual component is displayed and synchronized for the current simulation step.
//
// Workflow:
//   1. Check if visualization is enabled in the simsystem configuration.
//   2. Begin the visualization scene for this simulation step.
//   3. Render the full scene (all core visuals).
//   4. If enabled, render the inertial NED frame visualization.
//   5. If enabled, render the UAV body frame visualization.
//   5.1 If enabled, render the biplane frame visualization.
//   5.2 If enabled, render the chassis drag frame visualization.
//   5.3 If enabled, render the wing aerodynamic frames visualization.
//   6. If enabled, render frames for all UAV propellers.
//   7. If enabled, render all center-of-gravity (COG) frames.
//   8. If enabled, activate shadow rendering for all physics bodies.
//   9. If moving camera is enabled, update camera position and target for live-following the vehicle.
//  10. End the visualization scene, finalizing render output for the step.
//
// Notes:
//   - All rendering options are enabled or disabled via the visualization config (vis-config.yaml).
//   - All visuals are triggered in sequence within a single scene per time step.
//   - Camera and shadow updates provide enhanced realism and real-time following in visuals.
//   - Visualization may be completely skipped if not enabled, preserving compute resources.
// =====================================================================================================================
void simbridge::UpdateVisualizationSystemIrrlicht()
{
    // ------------------------------------------------------------------------
    // STEP 0 – Check if visualization is enabled for this simulator run.
    // ------------------------------------------------------------------------
    if (this->m_sys.GetVisConfig().enable_vis && this->m_sys.GetVisionSystemIrr().Run())
    {
        // ------------------------------------------------------------------------
        // STEP 1 – Throttle rendering to user defined FPS, independent of the 200Hz physics rate.
        // ------------------------------------------------------------------------
        this->m_render_period = 1.0 / this->m_sys.GetVisConfig().target_frame_rate;
        double current_time = this->m_sys.GetPhysicsSystem().GetChTime();
        if (current_time - this->m_last_render_time < this->m_render_period)
        {
            return;  // Not time to render yet — skip this call, physics still steps normally elsewhere
        }
        this->m_last_render_time = current_time;

        // --------------------------------------------------------------------
        // STEP 2 – Begin a new scene to set up rendering for this step.
        // --------------------------------------------------------------------
        this->m_sys.GetVisionSystemIrr().BeginScene();

        // --------------------------------------------------------------------
        // STEP 3 – Render the main simulator scene visuals.
        // --------------------------------------------------------------------
        this->m_sys.GetVisionSystemIrr().Render();

        // --------------------------------------------------------------------
        // STEP 4 – If enabled, render the inertial NED frame in the visualization.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_ned_frame) {
            this->m_sys.GetVisionSystemIrr().RenderFrame(m_uav->GetInertialNEDFrame(), 1);
        }

        // --------------------------------------------------------------------
        // STEP 5 – If enabled, render the NED body frame of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_body_frame) {
            this->m_sys.GetVisionSystemIrr().RenderFrame(m_uav->GetUAVChassis().body->GetFrameRefToAbs(), 0.6);
        }

        // --------------------------------------------------------------------
        // STEP 5.1 – If enabled, render the biplane body frame of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_biplane_frame) {
            this->m_sys.GetVisionSystemIrr().RenderFrame(m_uav->GetUAVChassis().biplane_frame->GetAbsFrame(), 0.6);
        }

        // --------------------------------------------------------------------
        // STEP 5.2 – If enabled, render the chassis drag frame of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_chassis_drag_frame) {
            this->m_sys.GetVisionSystemIrr().RenderFrame(m_uav->GetUAVChassisAerodynamics().chassis_drag_frame->GetAbsFrame(), 0.1);
        }

        // --------------------------------------------------------------------
        // STEP 5.3 – If enabled, render the wing aerodynamic frames of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_wing_aero_frames) {
            for (const auto& aerofoil : this->m_uav->GetUAVAerodynamicSurfaces()) {
                for (const auto& aeroframe : aerofoil.aerodynamic_center_frames) {
                    this->m_sys.GetVisionSystemIrr().RenderFrame(aeroframe->GetAbsFrame(), 0.1);
                }
            }
        }

        // --------------------------------------------------------------------
        // STEP 6 – If enabled, render frames for all UAV propellers.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_prop_frames) {
            for (int idx = 1; idx <= this->m_uav->GetPropCount(); ++idx) {
                this->m_sys.GetVisionSystemIrr().RenderFrame(m_uav->GetUAVProp(idx).body->GetFrameRefToAbs(), 0.3);
            }
        }

        // --------------------------------------------------------------------
        // STEP 7 – If enabled, render all center-of-gravity (COG) frames.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_all_COG_frames) {
            this->m_sys.GetVisionSystemIrr().RenderCOGFrames(1);
        }

        // --------------------------------------------------------------------
        // STEP 8 – If enabled, activate shadow rendering for all physics bodies.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_shadows) {
            // Get all bodies from the simulator and activate shadow rendering.
            auto bodies = this->m_sys.GetPhysicsSystem().GetBodies(); 
            for (auto& body : bodies) {
                this->m_sys.GetVisionSystemIrr().EnableShadows(body);
            }
        }

        // --------------------------------------------------------------------
        // STEP 9 – If the moving camera is enabled, update its position and target for real-time following.
        // --------------------------------------------------------------------
        if (!this->m_sys.GetVisConfig().enable_static_cam)
        {
            this->m_sys.GetCamera()->Update(this->m_sys.GetPhyConfig().StepSize);
            this->m_sys.GetVisionSystemIrr().GetActiveCamera()->setPosition(
                irr::core::vector3dfCH(this->m_sys.GetCamera()->GetCameraPos()));
            this->m_sys.GetVisionSystemIrr().GetActiveCamera()->setTarget(
                irr::core::vector3dfCH(this->m_sys.GetCamera()->GetTargetPos()));
        }

        // --------------------------------------------------------------------
        // STEP 10 – End the scene; finalize rendering for this simulation step.
        // --------------------------------------------------------------------
        this->m_sys.GetVisionSystemIrr().EndScene();
    }
}

void simbridge::UpdateVisualizationSystemVulkan()
{
    // ------------------------------------------------------------------------
    // STEP 0 – Check if visualization is enabled for this simulator run.
    // ------------------------------------------------------------------------
    if (this->m_sys.GetVisConfig().enable_vis && this->m_sys.GetVisionSystemVsg().Run())
    {
        // ------------------------------------------------------------------------
        // STEP 1 – Throttle rendering to user defined FPS, independent of the 200Hz physics rate.
        // ------------------------------------------------------------------------
        this->m_render_period = 1.0 / this->m_sys.GetVisConfig().target_frame_rate;
        double current_time = this->m_sys.GetPhysicsSystem().GetChTime();
        if (current_time - this->m_last_render_time < this->m_render_period)
        {
            return;  // Not time to render yet — skip this call, physics still steps normally elsewhere
        }
        this->m_last_render_time = current_time;

        // --------------------------------------------------------------------
        // STEP 2 – If enabled, render the inertial NED frame in the visualization.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_ned_frame) {
            this->m_sys.GetVisionSystemVsg().RenderFrame(m_uav->GetInertialNEDFrame(), 1);
        }

        // --------------------------------------------------------------------
        // STEP 3 – If enabled, render the NED body frame of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_body_frame) {
            this->m_sys.GetVisionSystemVsg().RenderFrame(m_uav->GetUAVChassis().body->GetFrameRefToAbs(), 0.6);
        }

        // --------------------------------------------------------------------
        // STEP 3.1 – If enabled, render the biplane body frame of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_biplane_frame) {
            this->m_sys.GetVisionSystemVsg().RenderFrame(m_uav->GetUAVChassis().biplane_frame->GetAbsFrame(), 0.6);
        }

        // --------------------------------------------------------------------
        // STEP 3.2 – If enabled, render the chassis drag frame of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_chassis_drag_frame) {
            this->m_sys.GetVisionSystemVsg().RenderFrame(m_uav->GetUAVChassisAerodynamics().chassis_drag_frame->GetAbsFrame(), 0.1);
        }

        // --------------------------------------------------------------------
        // STEP 3.3 – If enabled, render the wing aerodynamic frames of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_wing_aero_frames) {
            for (const auto& aerofoil : this->m_uav->GetUAVAerodynamicSurfaces()) {
                for (const auto& aeroframe : aerofoil.aerodynamic_center_frames) {
                    this->m_sys.GetVisionSystemVsg().RenderFrame(aeroframe->GetAbsFrame(), 0.1);
                }
            }
        }

        // --------------------------------------------------------------------
        // STEP 4 – If enabled, render frames for all UAV propellers.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_prop_frames) {
            for (int idx = 1; idx <= this->m_uav->GetPropCount(); ++idx) {
                this->m_sys.GetVisionSystemVsg().RenderFrame(m_uav->GetUAVProp(idx).body->GetFrameRefToAbs(), 0.3);
            }
        }

        // --------------------------------------------------------------------
        // STEP 5 – If enabled, render all center-of-gravity (COG) frames.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_all_COG_frames) {
            this->m_sys.GetVisionSystemVsg().RenderCOGFrames(1);
        }

        // --------------------------------------------------------------------
        // STEP 6 – If enabled, activate shadow rendering for all physics bodies.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_shadows) {
            // Get all bodies from the simulator and activate shadow rendering.
            this->m_sys.GetVisionSystemVsg().SetShadows(true);
        }

        // --------------------------------------------------------------------
        // STEP 7 – If the moving camera is enabled, update its position and target for real-time following.
        // --------------------------------------------------------------------
        if (!this->m_sys.GetVisConfig().enable_static_cam)
        {
            this->m_sys.GetCamera()->Update(this->m_sys.GetPhyConfig().StepSize);
            this->m_sys.GetVisionSystemVsg().SetCameraPosition(this->m_sys.GetCamera()->GetCameraPos());
            this->m_sys.GetVisionSystemVsg().SetCameraTarget(this->m_sys.GetCamera()->GetTargetPos());
        }

        // --------------------------------------------------------------------
        // STEP 8 – Render the scene; for this simulation step.
        // --------------------------------------------------------------------
        this->m_sys.GetVisionSystemVsg().Render();
    }
}

// =====================================================================================================================
// Purpose:
//   Updates and renders the visualization using Irrlicht or Vulkan depending on flags used in vsd-config.yaml.
//   Maps to the fucions UpdateVisualizationSystemVulkan or UpdateVisualizationSystemIrrlicht using function pointers.
//
// Workflow:
//   Define function to point to the right Update function according to the flag m_active_backend marked during setup. 
//
// =====================================================================================================================
void simbridge::UpdateVisualizationSystem()
{
    using Fn = void (simbridge::*)();

    Fn update = (m_sys.GetActiveBackend() == _acsl_::_system_::RenderBackend::VULKAN)
        ? &simbridge::UpdateVisualizationSystemVulkan
        : &simbridge::UpdateVisualizationSystemIrrlicht;

    (this->*update)();
}

// =====================================================================================================================
// UpdateControlAction()
//
// Purpose:
//   Dispatches the control law’s thrust commands to the simulated UAV by mapping
//   controller outputs to individual propulsors based on the current propulsion layout.
//   If configured, applies a motor efficiency disturbance to the selected motor
//   after the specified disturbance time.
//
// Workflow:
//   1. If the body is to be treated as a rigid body, send the control inputs
//      directly at the vehicle center of mass.
//   2. Otherwise, query the UAV for the number of propulsors currently configured.
//   3. Read controller thrust commands t1...t8 once into local variables.
//   4. If motor-efficiency disturbance is enabled, check whether the disturbance
//      activation time has been reached.
//   5. If the disturbance is active, reduce the thrust of the affected motor by
//      the configured total efficiency factor.
//   6. Dispatch the final thrust commands to the UAV using the existing actuator
//      mapping logic for 1–8 propulsors.
//   7. If the requested number of propulsors is outside the supported range,
//      raise a simulator error with a descriptive message.
//
// Notes:
//   - Supports single-rotor, multi-rotor, and unconventional 1–8 actuator layouts.
//   - Controller interface is assumed to expose get_t1()…get_t8() accessors.
//   - total_efficiency is treated as a multiplier, e.g. 0.5 means the affected
//     motor will produce 50% of the commanded thrust after disturbance activation.
//   - If disturbance is disabled, the function behaves exactly as before.
// =====================================================================================================================
void simbridge::UpdateControlAction() {

    // ------------------------------------------------------------------------
    // STEP 1 – If the vehicle is treated as a rigid body, send the control
    //          inputs directly to the center of mass and exit early.
    // ------------------------------------------------------------------------
    if (this->treat_as_rigid_body) {
        m_uav->SetControlInputAtCOM(m_ctrl->get_u1(),
                                    m_ctrl->get_u2(),
                                    m_ctrl->get_u3(),
                                    m_ctrl->get_u4());
        return;
    }

    // ------------------------------------------------------------------------
    // STEP 2 – Query the number of propulsors currently configured on the UAV.
    // ------------------------------------------------------------------------
    int propCount = m_uav->GetPropCount();

    // ------------------------------------------------------------------------
    // STEP 3 – Read controller thrust commands once into local variables.
    //          These values may be modified below if a disturbance is active.
    // ------------------------------------------------------------------------
    double t1 = m_ctrl->get_t1();
    double t2 = m_ctrl->get_t2();
    double t3 = m_ctrl->get_t3();
    double t4 = m_ctrl->get_t4();
    double t5 = m_ctrl->get_t5();
    double t6 = m_ctrl->get_t6();
    double t7 = m_ctrl->get_t7();
    double t8 = m_ctrl->get_t8();

    // ------------------------------------------------------------------------
    // STEP 4 – If motor disturbance logic is enabled, check whether the
    //          disturbance activation time has been reached.
    // ------------------------------------------------------------------------
    if (this->motor_disturbance.reduce_efficiency) {

        // --------------------------------------------------------------------
        // STEP 5 – If the disturbance time has been reached, reduce the thrust
        //          command of the selected motor by the configured efficiency.
        // --------------------------------------------------------------------
        if (this->m_sys.GetPhysicsSystem().GetChTime() >= this->motor_disturbance.time_of_disturbance) {

            int motor = this->motor_disturbance.affected_motor;
            if      (motor == 1) { t1 *= this->motor_disturbance.total_efficiency; }
            else if (motor == 2) { t2 *= this->motor_disturbance.total_efficiency; }
            else if (motor == 3) { t3 *= this->motor_disturbance.total_efficiency; }
            else if (motor == 4) { t4 *= this->motor_disturbance.total_efficiency; }
            else if (motor == 5) { t5 *= this->motor_disturbance.total_efficiency; }
            else if (motor == 6) { t6 *= this->motor_disturbance.total_efficiency; }
            else if (motor == 7) { t7 *= this->motor_disturbance.total_efficiency; }
            else if (motor == 8) { t8 *= this->motor_disturbance.total_efficiency; }
        }
    }

    // ------------------------------------------------------------------------
    // STEP 6 – Dispatch the final thrust commands using the existing actuator
    //          mapping logic based on the current propulsion layout.
    // ------------------------------------------------------------------------
    switch (propCount) {
        case 1: // Single-prop
            m_uav->SetThrustSetPoint(1, t1);
            break;

        case 2: // Twin-prop
            m_uav->SetThrustSetPoint(1, t1);
            m_uav->SetThrustSetPoint(2, t2);
            break;

        case 3: // Tricopters
            m_uav->SetThrustSetPoint(1, t1);
            m_uav->SetThrustSetPoint(2, t2);
            m_uav->SetThrustSetPoint(3, t3);
            break;

        case 4: // Quadcopters
            m_uav->SetThrustSetPoint(1, t1);
            m_uav->SetThrustSetPoint(2, t2);
            m_uav->SetThrustSetPoint(3, t3);
            m_uav->SetThrustSetPoint(4, t4);
            break;

        case 5: // Weird-stuff-I-guess
            m_uav->SetThrustSetPoint(1, t1);
            m_uav->SetThrustSetPoint(2, t2);
            m_uav->SetThrustSetPoint(3, t3);
            m_uav->SetThrustSetPoint(4, t4);
            m_uav->SetThrustSetPoint(5, t5);
            break;

        case 6: // Hexacopters
            m_uav->SetThrustSetPoint(1, t1);
            m_uav->SetThrustSetPoint(2, t2);
            m_uav->SetThrustSetPoint(3, t3);
            m_uav->SetThrustSetPoint(4, t4);
            m_uav->SetThrustSetPoint(5, t5);
            m_uav->SetThrustSetPoint(6, t6);
            break;

        case 7: // Weirder-stuff-I-think
            m_uav->SetThrustSetPoint(1, t1);
            m_uav->SetThrustSetPoint(2, t2);
            m_uav->SetThrustSetPoint(3, t3);
            m_uav->SetThrustSetPoint(4, t4);
            m_uav->SetThrustSetPoint(5, t5);
            m_uav->SetThrustSetPoint(6, t6);
            m_uav->SetThrustSetPoint(7, t7);
            break;

        case 8: // Octocopters
            m_uav->SetThrustSetPoint(1, t1);
            m_uav->SetThrustSetPoint(2, t2);
            m_uav->SetThrustSetPoint(3, t3);
            m_uav->SetThrustSetPoint(4, t4);
            m_uav->SetThrustSetPoint(5, t5);
            m_uav->SetThrustSetPoint(6, t6);
            m_uav->SetThrustSetPoint(7, t7);
            m_uav->SetThrustSetPoint(8, t8);
            break;

        // --------------------------------------------------------------------
        // STEP 7 – If an unsupported propulsion layout is requested, raise an
        //          error and notify the user.
        // --------------------------------------------------------------------
        default:
            _message_::SIMULATOR_ERROR("[SIMBRG]: ONLY 1-8 ACTUATORS SUPPORTED. YOU HAVE "
                                       "ASKED FOR SOMETHING I CANNOT GIVE YOU - GIRI");
            break;
    }
}

// =====================================================================================================================
// UpdatePhysicsSystem()
//
// Purpose:
//   Advances the physics simulation for the current step, manages real-time synchronization,
//   retrieves the UAV state, and prints color-coded state information to the terminal (if enabled).
//
// Workflow:
//   1. Perform one physics simulation time step using Chrono with the configured step size.
//   2. Maintain soft real-time pacing by spinning until step duration matches target (avoids CPU overrun).
//   3. Query the UAV's current states (position, velocity, rotation, etc.).
//   4. Apply the aerodynamic forces.
//   5. If terminal logging is enabled, format and print the state block in color for operator readability.
//
// Notes:
//   - Real-time spinning helps match simulation wall-clock to desired step size, especially in live/interactive modes.
//   - State block includes simulation time, position, velocity, Euler angles, angular velocity, forces, and torques.
//   - Color-coding improves operational clarity in high-frequency terminal output scenarios.
//   - Updates are performed every simulation tick; rapid terminal output may occur if log2terminal is set true.
// =====================================================================================================================
void simbridge::UpdatePhysicsSystem()
{

    // ------------------------------------------------------------------------
    // STEP 1 – Advance Chrono physics by one step using config step size.
    // ------------------------------------------------------------------------
    this->m_sys.GetPhysicsSystem().DoStepDynamics(this->m_sys.GetPhyConfig().StepSize);

    // ------------------------------------------------------------------------
    // STEP 2 – Spin in place to maintain soft real-time pacing for this 
    //          simulation tick.
    // NOTE: Needs to be handled properly, or else, it forces the simulation to
    //       run much slower as the computation time is already psuedo-realtime. 
    //       recommended to be set off.
    // ------------------------------------------------------------------------
    if(this->m_sys.GetPhyConfig().ThrottleRealTime) {
        this->realtimer.Spin(this->m_sys.GetPhyConfig().StepSize);
    }

    // ------------------------------------------------------------------------
    // STEP 3 – Extract current UAV state variables for reporting and logging.
    // ------------------------------------------------------------------------
    m_state = this->m_uav->GetUAVStateData();

    // ------------------------------------------------------------------------
    // STEP 4 – Apply the aerodynamic forces if enabled in the config.
    // ------------------------------------------------------------------------
    if (this->enable_chassis_drag) {
        this->m_uav->SetChassisDrag();
    }

    if (this->enable_wing_aerodynamics) {
        this->m_uav->SetUAVAeroSurfaceLiftDrag();
    }

    // ------------------------------------------------------------------------
    // STEP 5 – IF EFSL is not on. i.e no software/hardware-in-the-loop
    //          THEN apply model in the loop control.
    //          IF DEVELOPERMODE is not on. i.e we just want the environment
    //          loaded in. THEN apply model in the loop control.
    // ------------------------------------------------------------------------
    if (!this->GetSimMode() && !this->developer_mode) {
        // If using the NED frame of reference (-z body inplane with the prop's axis)
        if (!this->enable_biplane_frame_data) {
            this->m_ctrl->update(m_state.time, 
                                 m_state.pos.x(),
                                 m_state.pos.y(),
                                 m_state.pos.z(),
                                 m_state.vel.x(),
                                 m_state.vel.y(),
                                 m_state.vel.z(),
                                 m_state.quat.e0(),
                                 m_state.quat.e1(),
                                 m_state.quat.e2(),
                                 m_state.quat.e3(),
                                 m_state.eul.x(),
                                 m_state.eul.y(),
                                 m_state.eul.z(),
                                 m_state.ovel.x(),
                                 m_state.ovel.y(),
                                 m_state.ovel.z() );
        }
        else {
            this->m_ctrl->update(m_state.time,
                                 m_state.pos.x(),
                                 m_state.pos.y(),
                                 m_state.pos.z(),
                                 m_state.vel.x(),
                                 m_state.vel.y(),
                                 m_state.vel.z(),
                                 m_state.quat_bp.e0(),
                                 m_state.quat_bp.e1(),
                                 m_state.quat_bp.e2(),
                                 m_state.quat_bp.e3(),
                                 m_state.eul_bp.x(),
                                 m_state.eul_bp.y(),
                                 m_state.eul_bp.z(),
                                 m_state.ovel_bp.x(),
                                 m_state.ovel_bp.y(),
                                 m_state.ovel_bp.z() );
        }

                                        
        this->m_ctrl->run(this->m_sys.GetPhyConfig().StepSize);
        
        this->UpdateControlAction();

    }


    // ------------------------------------------------------------------------
    // STEP 4 – If terminal logging is enabled, print state block in color.
    // ------------------------------------------------------------------------
    if (this->log2terminal)
    {
        std::string color_label = "\033[38;5;214m";  // Bright Orange for labels
        std::string color_value = "\033[1;36m";      // Bright/Bold Cyan for values
        std::string color_reset = "\033[0m";         // Reset to default

        std::ostringstream msg;
        msg << "\n"
            << color_label << "SIMULATION TIME: " << color_value << m_state.time << " s\n" << color_reset
            << color_label << "UAV POSITION IN NED FRAME: " << color_value
            << m_state.pos.x() << ", "
            << m_state.pos.y() << ", "
            << m_state.pos.z() << "\n" << color_reset
            << color_label << "UAV VELOCITY IN NED FRAME: " << color_value
            << m_state.vel.x() << ", "
            << m_state.vel.y() << ", "
            << m_state.vel.z() << "\n" << color_reset
            << color_label << "UAV ROTATION IN NED FRAME: " << color_value
            << _shared_::_conversions_::rad2deg(m_state.eul.x()) << ", "
            << _shared_::_conversions_::rad2deg(m_state.eul.y()) << ", "
            << _shared_::_conversions_::rad2deg(m_state.eul.z()) << "\n" << color_reset
            << color_label << "UAV ROTATIONa IN BIPLANE FRAME: " << color_value
            << _shared_::_conversions_::rad2deg(m_state.eul_bp.x()) << ", "
            << _shared_::_conversions_::rad2deg(m_state.eul_bp.y()) << ", "
            << _shared_::_conversions_::rad2deg(m_state.eul_bp.z()) << "\n" << color_reset
            << color_label << "UAV ANGULAR VELOCITY IN NED FRAME: " << color_value
            << m_state.ovel.x() << ", "
            << m_state.ovel.y() << ", "
            << m_state.ovel.z() << "\n" << color_reset
            << color_label << "UAV ANGULAR VELOCITY IN BIPLANE FRAME: " << color_value
            << m_state.ovel_bp.x() << ", "
            << m_state.ovel_bp.y() << ", "
            << m_state.ovel_bp.z() << "\n" << color_reset
            << color_label << "UAV FORCES IN NED FRAME [I]: " << color_value
            << m_state.muI.x() << ", "
            << m_state.muI.y() << ", "
            << m_state.muI.z() << "\n" << color_reset
            << color_label << "UAV FORCES IN NED FRAME [J]: " << color_value
            << m_state.muJ.x() << ", "
            << m_state.muJ.y() << ", "
            << m_state.muJ.z() << "\n" << color_reset
            << color_label << "UAV TORQUES IN NED FRAME: " << color_value
            << m_state.tauJ.x() << ", "
            << m_state.tauJ.y() << ", "
            << m_state.tauJ.z() << "\n" <<color_reset
            << color_label << "UAV CHASSIS DRAG FORCES IN NED FRAME [J]: " << color_value
            << m_uav->GetUAVChassisAerodynamics().chassis_drag_force.x() << ", "
            << m_uav->GetUAVChassisAerodynamics().chassis_drag_force.y() << ", "
            << m_uav->GetUAVChassisAerodynamics().chassis_drag_force.z() << "\n" <<color_reset;
            
            // --------------------------------------------------------------------
            // Print per-aerodynamic-center debug info (for all aero surfaces)
            // --------------------------------------------------------------------
            if (this->enable_wing_aerodynamics && this->enable_wing_aerodynamics_dbg) {

                msg << color_label << "AERODYNAMIC FORCES:\n" << color_reset;

                for (const auto& aero : this->m_uav->GetUAVAerodynamicSurfaces()) {

                    msg << color_label
                        << " SURFACE: " << color_value << aero.name
                        << color_reset << "\n";

                    const std::size_t n = aero.aerodynamic_center_frames.size();

                    for (std::size_t i = 0; i < n; ++i) {
                        const double alpha_deg = ::_shared_::_conversions_::rad2deg(aero.alpha[i]);

                        msg << color_label << "    "
                        << aero.aerodynamic_center_frames[i]->GetName() << ": "
                        << "alpha [deg]: " << color_value << alpha_deg << color_label
                        << " | CL: "       << color_value << aero.CL[i] << color_label
                        << " | CD: "       << color_value << aero.CD[i] << color_label
                        << " | L [N]: "    << color_value << aero.surf_aero_lift[i] << color_label
                        << " | D [N]: "    << color_value << aero.surf_aero_drag[i]
                        << "\n" << color_reset;
                    }
                }
            }
            else if (this->enable_wing_aerodynamics) {
                msg << color_label << "AERODYNAMIC SURFACES: "
                    << color_value << "SWITCHED ON"
                    << color_reset << "\n";
            }
            else {
                msg << color_label << "AERODYNAMIC SURFACES: "
                    << color_value << "NONE"
                    << color_reset << "\n";
            }
        
        // Print the colorized output to the terminal.
        std::cout << msg.str() << std::endl;
    }

    // ------------------------------------------------------------------------
    // STEP 4 – If file logging is enabled, format and write to file.
    // ------------------------------------------------------------------------
    if (this->log2file) { this->LogData(); }
}



// =====================================================================================================================
// InitiateLogging()
//
// Purpose:
//   Initialize the Physics logger using Boost.Log with a formatted synchronous text sink targeting a file.
//   Sets logging attributes, ensures the log file is created and opened, and applies filtering to only capture the
//   physics data from the chrono system's log data.
//
// Workflow:
//   1. Construct the log file path and verify file can be opened for writing.
//   2. Add a "Tag" attribute to logger for log filtering.
//   3. Define and configure a synchronous text sink pointing to the file.
//   4. Set up formatting to include timestamp, thread/process IDs, tag, line ID, and the message.
//   5. Register the sink with Boost.Log's core.
//   6. Apply a filter to restrict logs by tag value.
//   7. Add Boost common attributes for completeness.
//   8. Return true on success; catch and report exceptions or file errors, returning false.
//
// Notes:
//   - Ensures file is writable before adding as stream.
//   - Reports detailed errors upon any failure.
//   - Ensures log attributes/sinks are not redundantly registered.
//   - Scalable; consider async sinks for high-frequency or multi-threaded logging.
// =====================================================================================================================
bool simbridge::InitiateLogging() {
    try {
        // --------------------------------------------------------------------
        // STEP 1 – Build log file path and check file accessibility.
        // --------------------------------------------------------------------
        std::stringstream log_ss;
        log_ss << this->m_logger.GetLogDirectory().string() << "/physics.log";
        std::string physics_log_filename = log_ss.str();

        // Prepare the log file stream
        auto file_stream = boost::make_shared<std::ofstream>(physics_log_filename);
        if (!file_stream->is_open()) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: FAILED TO OPEN LOG FILE: ", 
                                        physics_log_filename.c_str());
            return false;
        }

        // --------------------------------------------------------------------
        // STEP 2 – Add the "Tag" attribute for Physics log filtering, only if
        //          not already present.
        // --------------------------------------------------------------------
        if (!m_logger.GetPhysicsLogger().get_attributes().count("Tag")) {
            m_logger.GetPhysicsLogger().add_attribute("Tag", 
                        attrs::constant<std::string>("PhysicsTag"));
        }

        // --------------------------------------------------------------------
        // STEP 3 – Define and configure the synchronous sink with text backend.
        // --------------------------------------------------------------------
        typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
        boost::shared_ptr<text_sink> physics_sink = boost::make_shared<text_sink>();
        
        // Unlike the flightstack, we aren't looking for the highest tier of 
        // performance, therfore you can set the logger to flush data to file
        // as soon as it gets it in the cache.
        physics_sink->locked_backend()->auto_flush(true);   

        // --------------------------------------------------------------------
        // STEP 4 – Add file stream to sink backend.
        // --------------------------------------------------------------------
        physics_sink->locked_backend()->add_stream(file_stream);

        // --------------------------------------------------------------------
        // STEP 5 – Set log message format for clarity and detail.
        // --------------------------------------------------------------------
        physics_sink->set_formatter(
            expr::stream
                << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] "
                << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] "
                << "[" << expr::attr<std::string>("Tag") << "] "
                << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] "
                << "[" << expr::attr<unsigned int>("LineID") << "] "
                << expr::smessage
        );

        // --------------------------------------------------------------------
        // STEP 6 – Register the sink with the Boost.Log core.
        // --------------------------------------------------------------------
        logging::core::get()->add_sink(physics_sink);

        // --------------------------------------------------------------------
        // STEP 7 – Set filter to only accept logs with Tag == "PhysicsTag".
        // --------------------------------------------------------------------
        physics_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "PhysicsTag");

        // --------------------------------------------------------------------
        // STEP 8 – Add common Boost.Log attributes for timestamps, IDs, etc.
        // --------------------------------------------------------------------
        logging::add_common_attributes();
        _message_::SIMULATOR_INFO("[SIMBRG]: SIMULATOR PHYSICS LOG FILE OPENED");

        // --------------------------------------------------------------------
        // Return true if all logging setup steps were successful.
        // --------------------------------------------------------------------
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        _message_::SIMULATOR_ERROR("[SIMBRG]: FILESYSTEM ERROR: ", e.what());
    } catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMBRG]: EXCEPTION: ", e.what());
    }

    // ------------------------------------------------------------------------
    // Return false if any error or exception occurred.
    // ------------------------------------------------------------------------
    return false;
}

// =====================================================================================================================
// ConfigureHeaders()
//
// Purpose:
//   Build and write a CSV header row for all chassis and propeller states to the physics log.
//   Ensures that data columns in the log file are clearly labeled and aligned with each timestep's values.
//
// Workflow:
//   1. Define a complete, ordered list of state variable suffixes to describe all measured and derived fields.
//   2. Determine the current UAV's number of propellers (dynamic platform support).
//   3. Assemble header names for both the chassis (e.g., "chassis_x") and each propeller (e.g., "propeller_1_vx").
//   4. Concatenate all headers into a single comma-separated string for log file output.
//   5. Attach the "Tag" logging attribute as required by the Boost.Log sink filter to ensure this line is logged.
//   6. Write the header string using Boost.Log. Report to terminal on success.
//   7. Catch and report any exceptions that occur during log output.
//
// Notes:
//   - Guarantees all physics log columns are properly labeled for later analysis.
//   - Structure and naming align with simulation state output structures and variable names.
//   - Ensures compatibility with log sink filters (must have "PhysicsTag" attribute present).
//   - Fails gracefully, reporting header write errors to the terminal console.
// =====================================================================================================================
void simbridge::ConfigureHeaders()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Create array of suffixes for all UAV and propeller states.
    // ------------------------------------------------------------------------
    std::vector<std::string> state_suffixes = {
        "_t", "_x", "_y", "_z",
        "_vx", "_vy", "_vz",
        "_ax", "_ay", "_az",
        "_phi", "_theta", "_psi",
        "_phi_bp", "_theta_bp", "_psi_bp",
        "_q0", "_q1", "_q2", "_q3",
        "_q0_bp", "_q1_bp", "_q2_bp", "_q3_bp",
        "_wx", "_wy", "_wz",
        "_wx_bp", "_wy_bp", "_wz_bp",
        "_alphx", "_alphy", "_alphz",
        "_alphx_bp", "_alphy_bp", "_alphz_bp",
        "_muIx", "_muIy", "_muIz",
        "_muJx", "_muJy", "_muJz",
        "_tauJx", "_tauJy", "_tau_Jz"
    };

    // ------------------------------------------------------------------------
    // STEP 2 – Query live platform for current number of propellers
    // ------------------------------------------------------------------------
    int num_propellers = this->m_uav->GetPropCount();

    // ------------------------------------------------------------------------
    // STEP 3 – Assemble header names for chassis and all propellers
    // ------------------------------------------------------------------------
    std::vector<std::string> all_headers;
    for (const auto& suffix : state_suffixes) {
        all_headers.push_back("chassis" + suffix);
    }
    for (int idx = 1; idx <= num_propellers; ++idx) {
        for (const auto& suffix : state_suffixes) {
            all_headers.push_back("propeller_" + std::to_string(idx) + suffix);
        }
    }

    // ------------------------------------------------------------------------
    // STEP 4 – Concatenate all header names with a comma separator
    // ------------------------------------------------------------------------
    std::ostringstream oss;
    oss << ", "; // Leading delimiter for tag-data separation (if needed)
    for (const auto& name : all_headers) {
        oss << name << ", ";
    }

    try {
        // ------------------------------------------------------------------------
        // STEP 5 – Attach thread-local "Tag" so the logger filter lets 
        //          this log record through
        // ------------------------------------------------------------------------
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "PhysicsTag");

        // ------------------------------------------------------------------------
        // STEP 6 – Write headers to the physics log using Boost.Log macro
        // ------------------------------------------------------------------------
        BOOST_LOG(m_logger.GetPhysicsLogger()) << oss.str();
        _message_::SIMULATOR_INFO("[SIMBRG] WROTE PHYSICS LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        // ------------------------------------------------------------------------
        // STEP 7 – Report errors to terminal logger if header write fails
        // ------------------------------------------------------------------------
        _message_::SIMULATOR_ERROR("[SIMBRG] FAILED TO WRITE PHYSICS LOG HEADERS", e.what());
    }
}


// =====================================================================================================================
// LogData()
//
// Purpose:
//   Collects state data from the UAV chassis and all propellers and writes a formatted line to the physics log.
//   Ensures each timestep's complete UAV and propeller state is efficiently recorded using Boost.Log.
//
// Workflow:
//   1. Serialize the UAV chassis state variables using the shared serialization helper.
//   2. Iterate over all UAV propellers, serialize each one's state, and append to the output buffer.
//   3. Attach the required thread-local logging tag so this record is accepted by the log filter.
//   4. Write the collected data as a single line to the physics log using Boost.Log.
//   5. Catch and report any exceptions that may occur during log output.
//
// Notes:
//   - Relies on a common serialization helper to maintain consistent field ordering.
//   - Appends in the exact order and format expected by the CSV log headers.
//   - Tag attribute "PhysicsTag" must be present to ensure sink filter matches.
//   - Any logging failure is cleanly reported to the user via the simulator terminal logger.
// =====================================================================================================================
void simbridge::LogData()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Serialize UAV chassis state to buffer
    // ------------------------------------------------------------------------
    std::ostringstream oss;
    oss << ", ";        // Add a delimiter before constructing the message.
    // Use the SerialStateData function of the m_states structure to serialize
    //  the data for output.
    this->m_uav->GetUAVStateData().SerializeStateData(oss);

    // ------------------------------------------------------------------------
    // STEP 2 – Serialize all propeller states, appending them in order
    // ------------------------------------------------------------------------
    int nop = this->m_uav->GetPropCount();
    for (int i = 1; i <= nop; ++i)
    {
        // Use the SerialStateData function of the m_states structure to serialize
        //  the data for output.
        this->m_uav->GetUAVPropStateData(i).SerializeStateData(oss);
    }

    try {
        // ------------------------------------------------------------------------
        // STEP 3 – Attach thread-local "Tag" so the logger filter accepts this record
        // ------------------------------------------------------------------------
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "PhysicsTag");

        // ------------------------------------------------------------------------
        // STEP 4 – Write the serialized data to the physics log using Boost.Log macro
        // ------------------------------------------------------------------------
        BOOST_LOG(m_logger.GetPhysicsLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        // ------------------------------------------------------------------------
        // STEP 5 – On exception, report log failure via terminal logger
        // ------------------------------------------------------------------------
        _message_::SIMULATOR_ERROR("[SIMBRG] FAILED TO WRITE PHYSICS LOG DATA", e.what());
    }
}

// Prototype ever pause function - will house the logic for the ultimate pause. (La Pausa)
void simbridge::EverPause()
{
    if (this->developer_mode_steps) {
        _message_::SIMULATOR_INFO("[SIMBRG] STEP DONE, PRESS **ENTER** KEY TO GO TO THE NEXT STEP");
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();  // waits for Enter
        return;          // Returns
    }
}

// Prototype ever run function -  will house the logic for all the modes.
void simbridge::EverRun()
{
    // If you are in developer mode, you do not update the control input for the system
    if (this->developer_mode)
    {
        while (true)
        {
            // Update the vision system
            this->UpdateVisualizationSystem();

            // Update the physics system
            this->UpdatePhysicsSystem();
            
            // If you need a pause
            EverPause();
        }

    }
    else
    {
        // If the debug stop is used, then you stop at the specified time, else you stop when the drone lands
        if (this->sim_debug_stop)
        {
            while (this->m_sys.GetPhysicsSystem().GetChTime() < this->sim_stop_time)
            {
                // Update the vision system
                this->UpdateVisualizationSystem();

                // Update the physics system
                this->UpdatePhysicsSystem();   
                
                // If you need a pause
                EverPause();
            }
        }
        else
        {
            while (this->m_sys.GetPhysicsSystem().GetChTime() < this->m_trj->GetTmax())
            {
                // Update the vision system
                this->UpdateVisualizationSystem();

                // Update the physics system
                this->UpdatePhysicsSystem();

                // If you need a pause
                EverPause();
            }
        }
    }

    
    
}



}   // namespace _bridge_

}   // namespace _acsl_