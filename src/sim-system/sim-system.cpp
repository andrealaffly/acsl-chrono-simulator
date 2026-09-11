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
 * File:        sim-system.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class file for the chrono system
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/
#include "sim-system.hpp"

namespace _acsl_
{

namespace _system_
{

// =====================================================================================================================
// ReadPhysicsConfigFile()
//
// Purpose:
//   Loads and parses the physics configuration YAML file for the simulation.
//   This configuration defines core physics engine parameters such as:
//     - Gravity enable/disable in NED frame
//     - Solver type and iteration limits
//     - Collision system settings and tuning parameters
//
// Workflow:
//   1. Open the YAML file from the hardcoded path (`phy_config_filename`).
//   2. Validate that the file exists and is accessible.
//   3. Parse (deserialize) the YAML into a `fkyaml::node` object.
//   4. Extract specific physics, solver, and collision parameters
//      into the `phyconfig` struct for later use in SetupPhysicsSystem().
//
// Notes:
//   - This uses the fkyaml library for YAML deserialization.
//   - If the file is missing, a simulator runtime error will be triggered.
// =====================================================================================================================
void simsystem::ReadPhysicsConfigFile()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Open the physics configuration YAML file
    //   Uses phy_config_filename, which is set in the class as a constant path.
    //   `std::ifstream` is used for file input streaming.
    // ------------------------------------------------------------------------
    std::ifstream ifs(this->phy_config_filename);

    // ------------------------------------------------------------------------
    // STEP 2 – Check that the file exists and opened successfully
    //   If not, trigger a simulator error message (likely halts execution).
    // ------------------------------------------------------------------------
    if (!ifs) {
        _message_::SIMULATOR_ERROR(
            "[SIMSYS]: COULD NOT OPEN PHYSICS CONFIG FILE: " + this->phy_config_filename
        );
    }

    // ------------------------------------------------------------------------
    // STEP 3 – Deserialize the YAML configuration into memory
    //   fkyaml::node::deserialize() parses the YAML into a navigable tree.
    // ------------------------------------------------------------------------
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);

    // ------------------------------------------------------------------------
    // STEP 4 – Extract top-level physics parameters
    // ------------------------------------------------------------------------
    phyconfig.gravity = config_file["physics"]["gravity"].as_bool(); // Enable/disable gravity

    // ------------------------------------------------------------------------
    // STEP 5 – Extract solver-related parameters
    // ------------------------------------------------------------------------
    phyconfig.PSOR           = config_file["solver"]["PSOR"].as_bool();                     // Solver type flag
    phyconfig.MaxIterations  = config_file["solver"]["MaxIterations"].as_int();             // Max solver iterations
    phyconfig.EnableWarmStart = config_file["solver"]["EnableWarmStart"].as_bool();         // Warm start toggle
    phyconfig.StepSize = static_cast<double>(config_file["solver"]["StepSize"].as_float()); // Step size 
    phyconfig.ThrottleRealTime = config_file["solver"]["ThrottleRealTime"].as_bool();       // Realtime throrttle

    // ------------------------------------------------------------------------
    // STEP 6 – Extract collision system parameters
    // ------------------------------------------------------------------------
    phyconfig.BULLET                  = config_file["collision"]["BULLET"].as_bool();    // Bullet collision system toggle
    phyconfig.MULTICORE                = config_file["collision"]["MULTICORE"].as_bool();// Multicore collision system toggle
    phyconfig.DefaultSuggestedEnvelope = static_cast<double>(config_file["collision"]["DefaultSuggestedEnvelope"].as_float());
    phyconfig.DefaultSuggestedMargin   = static_cast<double>(config_file["collision"]["DefaultSuggestedMargin"].as_float());
    phyconfig.ContactBreakingThreshold = static_cast<double>(config_file["collision"]["ContactBreakingThreshold"].as_float());
}


// =====================================================================================================================
// ReadVisionConfigFile()
//
// Purpose:
//   Loads and parses the visualization configuration YAML file for the
//   simulation. This configuration defines how the simulator's visualization
//   behaves, including:
//     - Whether visualization is enabled
//     - Camera settings
//     - Frame overlays (NED, body frame)
//     - Rendering options (shadows, collision zones)
//     - Visualization window size and title
//
// Workflow:
//   1. Open the YAML config file from the class constant path
//      (`vis_config_filename`).
//   2. Validate file existence.
//   3. Parse (deserialize) the YAML file into a fkyaml::node object.
//   4. Extract all relevant settings into the `visconfig` struct for later use
//      in SetupVisualizationSystem().
//
// Notes:
//   - Uses the fkyaml library for simple YAML access.
//   - If the file does not exist, an error message is logged and (likely) 
//     simulation will not proceed.
// =====================================================================================================================
void simsystem::ReadVisionConfigFile()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Open the visualization configuration YAML file
    //   Uses vis_config_filename set in the simsystem class.
    // ------------------------------------------------------------------------
    std::ifstream ifs(this->vis_config_filename);

    // ------------------------------------------------------------------------
    // STEP 2 – Ensure the file exists and is readable
    //   If not, trigger a simulator-level error message.
    // ------------------------------------------------------------------------
    if (!ifs) {
        _message_::SIMULATOR_ERROR(
            "[SIMSYS]: COULD NOT OPEN VISUALIZATION CONFIG FILE: " + this->vis_config_filename
        );
    }

    // ------------------------------------------------------------------------
    // STEP 3 – Deserialize the YAML file into a fkyaml node hierarchy
    //   Allows key-based access to configuration entries.
    // ------------------------------------------------------------------------
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);

    // ------------------------------------------------------------------------
    // STEP 4 – Extract visualization "main" options
    //   These control the overall visualization engine, and target framerate.
    // ------------------------------------------------------------------------
    visconfig.enable_vis             = config_file["main"]["enable_vis"].as_bool();                         // Enable 3D window
    visconfig.enable_vulkan          = config_file["main"]["enable_vulkan"].as_bool();                      // Enable Vulkan visualization (default)
    visconfig.enable_irrlicht        = config_file["main"]["enable_irrlicht"].as_bool();                    // Enable Irrlicht 
    visconfig.target_frame_rate = static_cast<double>(config_file["main"]["target_frame_rate"].as_float()); // Target the required framerate.

    // ------------------------------------------------------------------------
    // STEP 5 – Extract visualization "camera" options
    //   These control the overall camera behavior, chase distance and height.
    // ------------------------------------------------------------------------
    visconfig.enable_static_cam      = config_file["camera"]["enable_static_cam"].as_bool();                  // Fixed camera view
    visconfig.mv_cam_chase_ht = static_cast<double>(config_file["camera"]["mv_cam_chase_ht"].as_float());     // Moving camera chase height
    visconfig.mv_cam_chase_dt = static_cast<double>(config_file["camera"]["mv_cam_chase_dt"].as_float());     // Moving camera chase distance

    // ------------------------------------------------------------------------
    // STEP 6 – Extract visualization "render" options
    //   These control the overall visualization overlays.
    // ------------------------------------------------------------------------
    visconfig.render_ned_frame       = config_file["render"]["render_ned_frame"].as_bool();                   // Draw NED axes
    visconfig.render_body_frame      = config_file["render"]["render_body_frame"].as_bool();                  // Draw UAV body axes
    visconfig.render_shadows         = config_file["render"]["render_shadows"].as_bool();                     // Enable shadow rendering
    visconfig.render_collision_zones = config_file["render"]["render_collision_zones"].as_bool();             // Show collision zone meshes
    visconfig.render_all_COG_frames  = config_file["render"]["render_all_COG_frames"].as_bool();              // Draw all COG frames
    visconfig.render_prop_frames     = config_file["render"]["render_prop_frames"].as_bool();                 // Draw all the propeller frames
    visconfig.render_trajectory      = config_file["render"]["render_trajectory"].as_bool();                  // Draw the trajectory in the scene
    visconfig.render_biplane_frame   = config_file["render"]["render_biplane_frame"].as_bool();               // Draw the biplane frame in the scene
    visconfig.render_chassis_drag_frame   = config_file["render"]["render_chassis_drag_frame"].as_bool();     // Draw the chassis drag frame in the scene
    visconfig.render_wing_aero_frames   = config_file["render"]["render_wing_aero_frames"].as_bool();         // Draw the wing aerodynamic frames in the scene

    // ------------------------------------------------------------------------
    // STEP 7 – Extract "window" options
    //   Defines resolution and window title for the visualization.
    // ------------------------------------------------------------------------
    visconfig.width  = static_cast<uint>(config_file["window"]["width"].as_int());   // Window width in pixels
    visconfig.height = static_cast<uint>(config_file["window"]["height"].as_int());  // Window height in pixels
    visconfig.title  = config_file["window"]["title"].as_str();                      // Window title text
}


// =====================================================================================================================
// SetupPhysicsSystem()
//
// Purpose:
//   Initializes and configures the Chrono physics system (`m_physics`) 
//   using parameters loaded from the `phyconfig` struct.
//
//   Key tasks:
//     1. Verify collision system configuration (only one type active).
//     2. Set gravitational acceleration (NED → Chrono conversion).
//     3. Select and configure collision system type.
//     4. Select and configure solver type and parameters.
//     5. Apply collision model tuning parameters.
//     6. Assign solver to the Chrono physics system.
//     7. Output configuration details to the simulator log.
//
// Preconditions:
//   - The physics configuration file (`phyconfig`) must already be loaded 
//     via ReadPhysicsConfigFile().
//   - This must be called before adding simulation bodies/links.
//
// Notes:
//   - Errors here generally stop the simulation because without a properly
//     configured physics core, nothing can run correctly.
// =====================================================================================================================
void simsystem::SetupPhysicsSystem()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Preliminary validation: Check collision system flags
    //   Exactly ONE collision system must be enabled.
    // ------------------------------------------------------------------------
    if (phyconfig.BULLET && phyconfig.MULTICORE) {
        _message_::SIMULATOR_ERROR("[SIMSYS]: PICK ONE COLLISION SYSTEM AT A TIME. ENDING SIMULATION");
    }
    else if (!phyconfig.BULLET && !phyconfig.MULTICORE) {
        _message_::SIMULATOR_ERROR("[SIMSYS]: BOTH COLLISION SYSTEMS ARE NOT ACTIVE. ENDING SIMULATION");
    }

    // ------------------------------------------------------------------------
    // STEP 2 – Assign gravity vector
    //   - Gravity vector provided in NED frame [0, 0, 9.8] m/s² 
    //     or zero if disabled.
    //   - Converted to Chrono coordinates using transformation utilities.
    // ------------------------------------------------------------------------
    if (phyconfig.gravity) {
        m_physics.SetGravitationalAcceleration(
            _shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0, 0, 9.80665)) 
        );
    } else {
        m_physics.SetGravitationalAcceleration(
            _shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0, 0, 0))
        );
    }

    // ------------------------------------------------------------------------
    // STEP 3 – Set collision system type
    //   - BULLET: single-threaded Bullet engine
    //   - MULTICORE: multi-threaded collision
    // ------------------------------------------------------------------------
    if (phyconfig.BULLET) {
        m_physics.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    } 
    else if (phyconfig.MULTICORE) {
        m_physics.SetCollisionSystemType(chrono::ChCollisionSystem::Type::MULTICORE);
    }

    // ------------------------------------------------------------------------
    // STEP 4 – Select and configure solver type
    //   Example: PSOR solver is the active one here.
    //   Additional solver types can be added as `else if` cases.
    // ------------------------------------------------------------------------
    if (phyconfig.PSOR) 
    { 
        // Create a PSOR solver instance
        auto psor_solver = std::make_shared<chrono::ChSolverPSOR>();

        // Configure solver parameters from config file
        psor_solver->SetMaxIterations(phyconfig.MaxIterations);
        psor_solver->EnableWarmStart(phyconfig.EnableWarmStart);
        
        // Store for later reference and pass to the physics system (Step 6)
        m_solver = psor_solver;         
    }
    else {
        _message_::SIMULATOR_ERROR("[SIMSYS]: NO SOLVER PICKED. ENDING SIMULATION");
    }

    // ------------------------------------------------------------------------
    // STEP 5 – Set collision model parameters
    //   These values influence contact surface tolerances and stability.
    // ------------------------------------------------------------------------
    chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(phyconfig.DefaultSuggestedEnvelope);
    chrono::ChCollisionModel::SetDefaultSuggestedMargin(phyconfig.DefaultSuggestedMargin);
    chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(phyconfig.ContactBreakingThreshold);

    // ------------------------------------------------------------------------
    // STEP 6 – Pass the configured solver to the Chrono physics system
    // ------------------------------------------------------------------------
    m_physics.SetSolver(m_solver);

    // ------------------------------------------------------------------------
    // STEP 7 – Log current configuration to the simulator console/log
    // ------------------------------------------------------------------------
    _message_::SIMULATOR_INFO("[SIMSYS]: SIMULATOR PHYSICS SYSTEM ACTIVE");
    _message_::SIMULATOR_INFO("[SIMSYS]:  - STARTING NSC SYSTEM");
    
    // Physics core settings
    _message_::SIMULATOR_INFO("[SIMSYS]:  - SYSTEM [GRAVITY]: ", ::_shared_::_conversions_::bool2string(phyconfig.gravity));
    
    // Solver settings
    _message_::SIMULATOR_INFO("[SIMSYS]:  - SOLVER [PSOR]: ", ::_shared_::_conversions_::bool2string(phyconfig.PSOR));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - SOLVER [MaxIterations]: ", phyconfig.MaxIterations);
    _message_::SIMULATOR_INFO("[SIMSYS]:  - SOLVER [EnableWarmStart]: ", ::_shared_::_conversions_::bool2string(phyconfig.EnableWarmStart));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - SOLVER [StepSize]: ", phyconfig.StepSize); 
    _message_::SIMULATOR_INFO("[SYMSYS]:  - SOLVER [ThrottleRealTime]", ::_shared_::_conversions_::bool2string(phyconfig.ThrottleRealTime));

    // Collision settings
    _message_::SIMULATOR_INFO("[SIMSYS]:  - COLLISION [BULLET]: ", ::_shared_::_conversions_::bool2string(phyconfig.BULLET));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - COLLISION [MULTICORE]: ", ::_shared_::_conversions_::bool2string(phyconfig.MULTICORE));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - COLLISION [DefaultSuggestedEnvelope]: ", phyconfig.DefaultSuggestedEnvelope);
    _message_::SIMULATOR_INFO("[SIMSYS]:  - COLLISION [DefaultSuggestedMargin]: ", phyconfig.DefaultSuggestedMargin);
    _message_::SIMULATOR_INFO("[SIMSYS]:  - COLLISION [ContactBreakingThreshold]: ", phyconfig.ContactBreakingThreshold);
}

// =====================================================================================================================
// SetupVisualizationSystem()
//
// Purpose:
//   Configures and initializes the Irrlicht-based visualization system for the
//   ACSL Physics Simulator, using options loaded from the visconfig struct.
//
//   Features controlled:
//     - Window size and title
//     - Initialization and standard scene setup (logo, sky, lights)
//     - Rendering toggles (collision zone highlighting)
//     - Camera selection (static or dynamic)
//     - Logging of visualization options to the simulator output
//     - Attaches Chrono physics system for real-time rendering
//
// Workflow:
//   1. Check if visualization is enabled; early-return if not.
//   2. Set window size and title, then initialize the visualization engine.
//   3. Add scene elements (logo, skybox, lights).
//   4. Output visualization setup parameters to the log.
//   5. Attach the Chrono system for rendering simulation objects.
//   6. Enable and configure rendering options as chosen in config.
//
// Notes:
//   - If visualization is disabled, prints a message and returns immediately;
//     nothing further is set up or drawn.
//   - Collision zone overlay can have significant GPU impact.
// =====================================================================================================================
void simsystem::SetupVisualizationSystem()
{
    
    // ------------------------------------------------------------------------
    // STEP 1 – Check if visualization is enabled
    //   If not, log status and exit early; simulation will run headless.
    // ------------------------------------------------------------------------
    // Create a RenderBackend object to assign it later. 
    ::_acsl_::_system_::RenderBackend backend;

    if (!visconfig.enable_vis) {
        _message_::SIMULATOR_INFO("[SIMSYS]: VISUALIZATION SYSTEM INACTIVE");
        return;
    }
    else 
    {   // In the case the user wants to visualize things
        // ------------------------------------------------------------------------
        // STEP 1.1 – Check which visualization render engine is picked
        //          - If none are picked-default to irrlicht
        //          - If both are picked-throw an error 
        // ------------------------------------------------------------------------
        if (visconfig.enable_vulkan && visconfig.enable_irrlicht) {
            _message_::SIMULATOR_WARNING("[SYMSYS]: BOTH VULKAN AND IRRLICHT ARE SET TO TRUE, DEFAULTING TO IRRLICHT");
            backend = RenderBackend::IRRLICHT;
        }
        else if (visconfig.enable_vulkan && !visconfig.enable_irrlicht) {
            _message_::SIMULATOR_INFO("[SYMSYS]: VULKAN RENDER ENGINE ACTIVE. ATTACHING TO SYSTEM");
            backend = RenderBackend::VULKAN;
        }
        else if (!visconfig.enable_vulkan && visconfig.enable_irrlicht) {
            _message_::SIMULATOR_INFO("[SYMSYS]: IRRLICHT RENDER ENGINE ACTIVE. ATTACHING TO SYSTEM");
            backend = RenderBackend::IRRLICHT;
        }
        else if (!visconfig.enable_vulkan && !visconfig.enable_irrlicht) {
            _message_::SIMULATOR_WARNING("[SYMSYS]: BOTH VULKAN AND IRRLICHT ARE SET TO FALSE, DEFAULTING TO IRRLICHT");
            backend = RenderBackend::IRRLICHT;
        } 
        else {
            return;
        }

    }
    

    // ------------------------------------------------------------------------
    // STEPS 1.5 - 3 depend on the RenderBackend
    // ------------------------------------------------------------------------
    if (backend == ::_acsl_::_system_::RenderBackend::VULKAN) {
        // ------------------------------------------------------------------------
        // STEP 2 – Main window setup: size, title, and initialization
        // ------------------------------------------------------------------------
        m_vsg.SetWindowSize(visconfig.width, visconfig.height); // Set window size (pixels)
        m_vsg.SetWindowTitle(visconfig.title);                  // Set window title string
        
        // ------------------------------------------------------------------------
        // STEP 3 – Scene elements: add logo, skybox, and lights to environment
        // ------------------------------------------------------------------------
        m_vsg.SetLogoVisible(true);    // Overlay Project Chrono logo (optional branding)
        m_vsg.SetUseSkyBox(true);      // Add skybox textures for realistic environment
        m_vsg.SetLightDirection(chrono::CH_PI / 4.0, chrono::CH_PI / 4.0);  // azimuth/elevation approximating the 4-corner setup
        m_vsg.SetLightIntensity(1.0f);  // Light intensity. You can change it. It's hardcoded with the best setting so far - Xavi  
    }
    else if (backend == ::_acsl_::_system_::RenderBackend::IRRLICHT) {
        // ------------------------------------------------------------------------
        // STEP 1.5 – Force the driver type to use OpenGL.
        //   If not, we receive a benign error at runtime before it defaults to 
        //   OpenGL anyway.
        // ------------------------------------------------------------------------
        m_irrlicht.SetDriverType(irr::video::E_DRIVER_TYPE::EDT_OPENGL);
    
        // ------------------------------------------------------------------------
        // STEP 2 – Main window setup: size, title, and initialization
        // ------------------------------------------------------------------------
        m_irrlicht.SetWindowSize(visconfig.width, visconfig.height); // Set window size (pixels)
        m_irrlicht.SetWindowTitle(visconfig.title);                  // Set window title string
        m_irrlicht.Initialize();                                     // Initialize Irrlicht for drawing
        
        // ------------------------------------------------------------------------
        // STEP 3 – Scene elements: add logo, skybox, and lights to environment
        // ------------------------------------------------------------------------
        m_irrlicht.AddLogo();          // Overlay Project Chrono logo (optional branding)
        m_irrlicht.AddSkyBox();        // Add skybox textures for realistic environment
        
        // ----------------------------------
        // Lighting grid dimensions
        // ----------------------------------

        const double xmin = -1000.0;
        const double xmax =  1000.0;

        const double ymin = -1000.0;
        const double ymax =  1000.0;

        const double zmin = -100.0;
        const double zmax =  100.0;


        // ----------------------------------
        // Top light grid: z = zmax
        // ----------------------------------

        for (double x = xmin; x <= xmax; x += 100) {
            for (double y = ymin; y <= ymax; y += 100) {
                m_irrlicht.AddLight(
                    _shared_::_transformations_::GetChronoPosFromNED(
                        chrono::ChVector3d(x, y, zmax)
                    ),
                    100,
                    chrono::ChColor(0.7f, 0.7f, 0.7f)
                );
            }
        }


        // ----------------------------------
        // Bottom light grid: z = zmin
        // ----------------------------------

        for (double x = xmin; x <= xmax; x += 100) {
            for (double y = ymin; y <= ymax; y += 100) {
                m_irrlicht.AddLight(
                    _shared_::_transformations_::GetChronoPosFromNED(
                        chrono::ChVector3d(x, y, zmin)
                    ),
                    100,
                    chrono::ChColor(0.7f, 0.7f, 0.7f)
                );
            }
        }


        // ----------------------------------
        // Side light grid: x = xmin
        // ----------------------------------

        for (double y = ymin; y <= ymax; y += 100) {
            for (double z = zmin; z <= zmax; z += 100) {
                m_irrlicht.AddLight(
                    _shared_::_transformations_::GetChronoPosFromNED(
                        chrono::ChVector3d(xmin, y, z)
                    ),
                    100,
                    chrono::ChColor(0.7f, 0.7f, 0.7f)
                );
            }
        }


        // ----------------------------------
        // Side light grid: x = xmax
        // ----------------------------------

        for (double y = ymin; y <= ymax; y += 100) {
            for (double z = zmin; z <= zmax; z += 100) {
                m_irrlicht.AddLight(
                    _shared_::_transformations_::GetChronoPosFromNED(
                        chrono::ChVector3d(xmax, y, z)
                    ),
                    100,
                    chrono::ChColor(0.7f, 0.7f, 0.7f)
                );
            }
        }


        // ----------------------------------
        // Side light grid: y = ymin
        // ----------------------------------

        for (double x = xmin; x <= xmax; x += 100) {
            for (double z = zmin; z <= zmax; z += 100) {
                m_irrlicht.AddLight(
                    _shared_::_transformations_::GetChronoPosFromNED(
                        chrono::ChVector3d(x, ymin, z)
                    ),
                    100,
                    chrono::ChColor(0.7f, 0.7f, 0.7f)
                );
            }
        }


        // ----------------------------------
        // Side light grid: y = ymax
        // ----------------------------------

        for (double x = xmin; x <= xmax; x += 100) {
            for (double z = zmin; z <= zmax; z += 100) {
                m_irrlicht.AddLight(
                    _shared_::_transformations_::GetChronoPosFromNED(
                        chrono::ChVector3d(x, ymax, z)
                    ),
                    100,
                    chrono::ChColor(0.7f, 0.7f, 0.7f)
                );
            }
        }


    }
    else {
        return;
    }

    // ------------------------------------------------------------------------
    // STEP 4 – Log visualization configuration to the simulator output/log
    // ------------------------------------------------------------------------
    _message_::SIMULATOR_INFO("[SIMSYS]: VISUALIZATION SYSTEM ACTIVE");
    _message_::SIMULATOR_INFO("[SIMSYS]:  - STARTED WINDOW OF SIZE: " +
                              std::to_string(visconfig.width) + " x " +
                              std::to_string(visconfig.height));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - STARTED WINDOW WITH TITLE: " + visconfig.title);
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER NED FRAME: ", ::_shared_::_conversions_::bool2string(visconfig.render_ned_frame));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER BODY FRAME: ", ::_shared_::_conversions_::bool2string(visconfig.render_body_frame));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER BIPLANE FRAME: ", ::_shared_::_conversions_::bool2string(visconfig.render_biplane_frame));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER CHASSIS DRAG FRAME: ", ::_shared_::_conversions_::bool2string(visconfig.render_chassis_drag_frame));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER WING AERO FRAMES: ", ::_shared_::_conversions_::bool2string(visconfig.render_wing_aero_frames));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER PROP FRAMES: ", ::_shared_::_conversions_::bool2string(visconfig.render_prop_frames));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER ALL COG FRAMES: ", ::_shared_::_conversions_::bool2string(visconfig.render_all_COG_frames));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER TRAJECTORY: ", ::_shared_::_conversions_::bool2string(visconfig.render_trajectory));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER COLLISION ZONES: ", ::_shared_::_conversions_::bool2string(visconfig.render_collision_zones));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - RENDER SHADOWS: ", ::_shared_::_conversions_::bool2string(visconfig.render_shadows));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - ENABLE STATIC CAM: ", ::_shared_::_conversions_::bool2string(visconfig.enable_static_cam));
    _message_::SIMULATOR_INFO("[SIMSYS]:  - CAMERA CHASE HEIGHT: ", visconfig.mv_cam_chase_ht);
    _message_::SIMULATOR_INFO("[SIMSYS]:  - CAMERA CHASE DISTANCE: ", visconfig.mv_cam_chase_dt);

    // ------------------------------------------------------------------------
    // STEPS 5 - 6 depend on the RenderBackend
    // ------------------------------------------------------------------------
    if (backend == ::_acsl_::_system_::RenderBackend::VULKAN) {

        // ------------------------------------------------------------------------
        // STEP 5 – Attach the Chrono physics system for real-time rendering
        //   This connects the visualization window to the simulation world.
        // ------------------------------------------------------------------------
        m_vsg.AttachSystem(&this->m_physics);

        // ------------------------------------------------------------------------
        // STEP 6 – Optional rendering features (camera, collision zones)
        // ------------------------------------------------------------------------

        // Add a static camera if requested
        // Camera is placed at (2, 2, -5) and looks toward (0, 1, 0) in global coordinates
        if (visconfig.enable_static_cam) {
            m_vsg.AddCamera(chrono::ChVector3d(2, 2, -5), chrono::ChVector3d(0, 1, 0));
        }
        // Add a moving camera if requested
        // Camera is placed placed at a specified distance from the yaml file
        else
        {
            // Create the following camera attached to the chassis of the UAV. It is always names chassis
            m_camera = chrono_types::make_shared<chrono::utils::ChChaseCamera>(this->m_physics.SearchBody("chassis"));
            // Set the camera to always follow the chassis
            m_camera->SetState(chrono::utils::ChChaseCamera::State::Follow);
            // Intializet the camera
            m_camera->Initialize(_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0, 0, 0)),    // Attached to the COM of the drone
                                this->m_physics.SearchBody("chassis")->GetCoordsys(),                             // Attached to the COM coordsys of the drone
                                this->GetVisConfig().mv_cam_chase_dt,                                             // Chase distance
                                this->GetVisConfig().mv_cam_chase_ht,                                             // Chase height
                                _shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0 , 0, -1)),  // The up direction for the camera
                                _shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(1 , 0,  0))   // The fws direction for the camera
            );

            // Add the camera to the system
            m_vsg.AddCamera(m_camera->GetCameraPos(), m_camera->GetTargetPos());
        }

        m_vsg.SetCameraVertical(chrono::CameraVerticalDir::Y);  // match Irrlicht's Y-up convention
        
        m_vsg.Initialize();                                     // Initialize the Scene with Vulkan (must be at the end!) 

        // Enable visualization of collision zones/boundaries if requested
        // if (visconfig.render_collision_zones) {
        //     m_vsg.SetCollisionVisibility(true);  // Not available in Chrono 9.0.1's VSG module — feature added in a later Chrono version
        // }
    } 
    else if (backend == ::_acsl_::_system_::RenderBackend::IRRLICHT) {
        
        // ------------------------------------------------------------------------
        // STEP 5 – Attach the Chrono physics system for real-time rendering
        //   This connects the visualization window to the simulation world.
        // ------------------------------------------------------------------------
        m_irrlicht.AttachSystem(&this->m_physics);

        // ------------------------------------------------------------------------
        // STEP 6 – Optional rendering features (camera, collision zones)
        // ------------------------------------------------------------------------

        // Add a static camera if requested
        // Camera is placed at (2, 2, -5) and looks toward (0, 1, 0) in global coordinates
        if (visconfig.enable_static_cam) {
            m_irrlicht.AddCamera(chrono::ChVector3d(2, 2, -5), chrono::ChVector3d(0, 1, 0));
        }
        // Add a moving camera if requested
        // Camera is placed placed at a 
        else
        {
            // Create the following camera attached to the chassis of the UAV. It is always names chassis
            m_camera = chrono_types::make_shared<chrono::utils::ChChaseCamera>(this->m_physics.SearchBody("chassis"));
            // Set the camera to always follow the chassis
            m_camera->SetState(chrono::utils::ChChaseCamera::State::Follow);
            // Intializet the camera
            m_camera->Initialize(_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0, 0, 0)),    // Attached to the COM of the drone
                                this->m_physics.SearchBody("chassis")->GetCoordsys(),                             // Attached to the COM coordsys of the drone
                                this->GetVisConfig().mv_cam_chase_dt,                                             // Chase distance
                                this->GetVisConfig().mv_cam_chase_ht,                                             // Chase height
                                _shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0 , 0, -1)),  // The up direction for the camera
                                _shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(1 , 0,  0))   // The fws direction for the camera
            );

            // Add the camera to the system
            m_irrlicht.AddCamera(m_camera->GetCameraPos(), m_camera->GetTargetPos());
        }

        // Enable visualization of collision zones/boundaries if requested
        if (visconfig.render_collision_zones) {
            m_irrlicht.EnableCollisionShapeDrawing(true);
        }
    }
    else {
        return;
    }

    // Update the active visualization method used
    m_active_backend = backend;

}

}   // namespace _system_

}   // namespace _acsl_