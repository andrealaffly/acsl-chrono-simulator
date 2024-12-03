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
 * File:        uav.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 13, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: uav base class that will be inherited by all the platforms
 * 
 * GitHub:    https://github.com/andrealaffly/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#ifndef UAV_HPP_
#define UAV_HPP_

// System includes
#include <vector>
#include <unordered_map>
#include <string>

// Chorno phsyics includes
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChLinkMotorAll.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtils.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/utils/ChUtilsChaseCamera.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace _platforms_
{

namespace _uav_
{

// Global Defines
inline constexpr double DEFAULT_SUGGESTED_ENVELOPE = 0.003;
inline constexpr double DEFAULT_SUGGESTED_MARGIN = 0.003;
inline constexpr double CONTACT_BREAKING_THRESHOLD = 0.002;

inline constexpr double GRAV_ACC_X = 0.0;
inline constexpr double GRAV_ACC_Y = 0.0;
inline constexpr double GRAV_ACC_Z = 9.8;

inline constexpr double STEP_SIZE = 5e-3;

/// Base class for any uav platform, template parameter nop is the number of properllers.
template <int nop>
class UAV {
    public:
        // Visualization operation mode
        enum VisOptions {
            EnbaleAbsCoord,     ///< Enable Absolute Coordsys Drawing
            EnableColShape      ///< Enable Collision Shape Drawing
        };

        UAV(chrono::ChSystemSMC& sys,                          ///< Containing physical system
            chrono::irrlicht::ChVisualSystemIrrlicht& vis,     ///< Containing visual system
            chrono::ChRealtimeStepTimer& rtimer                ///< Real timer
            );

        void InitFramework();

        /*virtual*/ ~UAV() {}

        /// Add triangular meshes to the chassis and the propellers.
        void AddVisualizationAssets();

        /// Setup Visualization
        void SetupVisualization(chrono::utils::ChChaseCamera::State state,      ///< Camera state
                                double chaseDist,                               ///< Camera Chase Distance
                                double chaseHeight                              ///< Camera Chase Height
        );

        /// Add collision assets to the system
        void AddChassisCollisionShapes(std::vector<std::shared_ptr<chrono::ChCollisionShape>> shapes,
                                       std::vector<chrono::ChFramed> frames
        );

        void AddChassisCollisionShapes(std::array<std::vector<std::shared_ptr<chrono::ChCollisionShape>>, nop> shapes,
                                       std::array<std::vector<chrono::ChFramed>, nop> frames
        );

        /// Add constraints
        void AddConstraints(std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist);

        /// Update the UAV internal physics.
        void Update();

        /// Set the mass of the chassis 
        void SetChassisMass(double chassismass) { cmass = chassismass; }

        // Set chassis initial position
        void SetChassisInitialPosition(chrono::ChVector3d position) { cpos = position; }

        // Set chassis initial rotation
        void SetChassisInitialRotation(chrono::ChQuaterniond rotation) { crot = rotation; }

        /// Set the chassis Inertia
        void SetChassisInertia(chrono::ChVector3d inertiaXX, chrono::ChVector3d inertiaXY) 
        { cinerXX = inertiaXX; cinerXY =  inertiaXY; }

        // Set chassis auxillary frame
        void SetChassisAuxFrame(chrono::ChFramed frame) { cauxframe = frame; }

        // Set the name of the Wavefront file with chassis visualization mesh.
        void SetChassisMeshFileName(const std::string& path) { chassis_mesh_path = path; }

        /// Set the chassis visualization mesh frame.
        void SetChassisMeshFrame(const chrono::ChFramed& frame) { cor_m1 = frame; }

        /// Get the chassis inertia
        std::pair<chrono::ChVector3d, chrono::ChVector3d> GetChassisInertia() const 
        { return std::make_pair(cinerXX, cinerXY); }

        /// Get the mass of the chassis
        double GetChassisMass() const { return cmass; }

        /// Get the chassis visualization mesh frame.
        /// Returns the frame as a constant reference.
        const chrono::ChFramed& GetChassisMeshFrame() const { return cor_m1; }

        /// Get the chassis body
        std::shared_ptr<chrono::ChBodyAuxRef> GetChassis() const { return chassis; }

        /// Get the name of the Wavefront file with chassis visualization mesh.
        /// An empty string is returned if no mesh was specified.
        const std::string& GetChassisMeshFilename() const { return chassis_mesh_path; }

        /// Set the mass of the propellers
        void SetPropellerMass(double propellermass) { pmass = propellermass; }

        /// Set propeller initial poisitions
        void SetPropellerInitialPosition(std::vector<chrono::ChVector3d> position) { ppos = position; }

        /// Set propeller initial rotations
        void SetPropellerInitialRotation(std::vector<chrono::ChQuaterniond> rotation) { prot = rotation; }

        /// Set the propeller inertia
        void SetPropellerInertia(chrono::ChVector3d inertiaXX, chrono::ChVector3d inertiaXY)
        { pinerXX = inertiaXX; pinerXY = inertiaXY; }

        /// Set the rotation direction of each propeller.
        /// The input vector should contain the rotation directions for all propellers.
        void SetPropellerDirections(const std::vector<bool>& directions) {
            if (directions.size() != nop) {
                throw std::invalid_argument("The size of the direction vector must match the number of propellers.");
            }
            pdir = directions;
        }

        /// Set the names of the Wavefront files for the propeller visualization meshes.
        /// The input vector should contain paths to the mesh files for all propellers.
        void SetPropellerMeshFilenames(const std::vector<std::string>& paths) {
            if (paths.size() != nop) {
                throw std::invalid_argument("The size of the path vector must mach the number of propellers.");
            }    
            propeller_mesh_paths = paths; 
        }

        /// Set the propeller mesh frames.
        /// The input vector should contain the frames for all propellers.
        void SetPropellerMeshFrames(const std::vector<chrono::ChFramed>& frames) {
            if (frames.size() != nop) {
                throw std::invalid_argument("The size of the frames vector must match the number of propellers.");
            }
            cor_m2 = frames;
        }

        /// Get the propeller mesh frames.
        /// Returns the vector of frames as a constant reference.
        const std::vector<chrono::ChFramed>& GetPropellerMeshFrames() const { return cor_m2; }

        /// Get propepeller mass
        double GetPropellerMass() const { return pmass; }

        /// Get the propeller inertia
        std::pair<chrono::ChVector3d, chrono::ChVector3d> GetPropellerInertia() const
        { return std::make_pair(pinerXX, pinerXY); }    

        /// Get the rotation direction of each propeller.
        /// Returns the vector of directions as a constant reference.
        const std::vector<bool>& GetPropellerDirections() const { return pdir; }

        /// Get the number of propellers.
        int GetNumProps() const { return nop; }

        /// Get the propellers bodies.
        std::vector<std::shared_ptr<chrono::ChBody>> GetProps() const { return props; }        


        /// Get the name of the Wavefront file with propeller visualization mesh.
        /// An empty string is returned if no mesh was specified.
        const std::vector<std::string>& GetPropellerMeshFilename() const { return propeller_mesh_paths; }

        /// Get the time step for simulation
        double GetStepSize() const { return timestep; }
        
        // Getter for linklist_
        std::vector<std::shared_ptr<chrono::ChLinkBase>> GetLinkList() const { return linklist_; }

    private:
        chrono::ChSystemSMC& sys_;                                                 ///< System
        chrono::irrlicht::ChVisualSystemIrrlicht& vis_;                            ///< Visual System
        chrono::ChRealtimeStepTimer& rtimer_;                                      ///< Real Time Stepper

        double cmass;                                                              ///< Mass of chassis
        chrono::ChVector3d cinerXX;                                                ///< Chassis inertia XX
        chrono::ChVector3d cinerXY;                                                ///< Chassis inertia XY
        chrono::ChVector3d cpos;                                                   ///< Chassis position
        chrono::ChQuaterniond crot;                                                ///< Chassis rotation
        chrono::ChFramed cauxframe;                                                ///< Chassis auxilliary frame
        std::shared_ptr<chrono::ChBodyAuxRef> chassis;                             ///< Auxiliary body for chassis
        std::string chassis_mesh_path;                                             ///< Visualization mesh for chassis
        chrono::ChFramed cor_m1;                                                   ///< Visualization mesh frame

        double pmass;                                                              ///< Propeller mass
        chrono::ChVector3d pinerXX;                                                ///< Propeller inertia XX
        chrono::ChVector3d pinerXY;                                                ///< Propeller inertia XY
        std::vector<chrono::ChVector3d> ppos;                                      ///< Propeller relative positions
        std::vector<chrono::ChQuaterniond> prot;                                   ///< Propeller relative rotations
        std::vector<bool> pdir;                                                    ///< Direction of each propeller
        std::vector<std::shared_ptr<chrono::ChBody>> props;                        ///< Auxiliary bodies for propellers
        std::vector<std::string> propeller_mesh_paths;                             ///< Visualization meshes for props
        std::vector<chrono::ChFramed> cor_m2;                                      ///< Visualization mesh frames - props

        std::vector<std::shared_ptr<chrono::ChForce>> thrusts;                     ///< Thrust forces
        std::vector<std::shared_ptr<chrono::ChForce>> backtorques;                 ///< Propeller resistance torques
        std::vector<std::shared_ptr<chrono::ChLinkMotorRotationSpeed>> actuators;  ///< Propeller motors
        std::vector<std::shared_ptr<chrono::ChFunctionConst>> speeds;              ///< Propeller motor speed functions

        std::shared_ptr<chrono::utils::ChChaseCamera> camera;                      ///< Chase camera object
        double timestep = STEP_SIZE;                                               ///< timestep for simulation
        std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist_;                ///< list of links
};

template <int nop>
UAV<nop>::UAV(chrono::ChSystemSMC& sys, chrono::irrlicht::ChVisualSystemIrrlicht& vis, chrono::ChRealtimeStepTimer& rtimer) 
          : sys_(sys), vis_(vis), rtimer_{rtimer} {} 

template <int nop>
void UAV<nop>::InitFramework()
{
    // Set the collision types - Make sure the chrono physical systems is ChSystemSMC
    sys_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    sys_.SetContactForceModel(chrono::ChSystemSMC::ContactForceModel::Hertz);
    sys_.SetAdhesionForceModel(chrono::ChSystemSMC::AdhesionForceModel::Constant);    

    // Set the gravitational acceleration the right orientation
    sys_.SetGravitationalAcceleration(chrono::ChVector3d(GRAV_ACC_X, GRAV_ACC_Y, GRAV_ACC_Z));

    // Some global settings for collisions
    chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(DEFAULT_SUGGESTED_ENVELOPE);
    chrono::ChCollisionModel::SetDefaultSuggestedMargin(DEFAULT_SUGGESTED_MARGIN);
    chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(CONTACT_BREAKING_THRESHOLD);

    // Create and assign the chassis body data
    chassis = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    chassis->SetPos(cpos);
    chassis->SetRot(crot);
    chassis->SetMass(cmass);
    chassis->SetInertiaXX(cinerXX);
    chassis->SetInertiaXY(cinerXY);
    chassis->SetFixed(false);
    chassis->SetFrameCOMToRef(cauxframe);
    sys_.AddBody(chassis);    

    // Create and assign the propeller body data and the motors
    for (int p = 0; p < nop; p++)
    {
        auto prop = chrono_types::make_shared<chrono::ChBody>();
        prop->SetPos(ppos[p]);
        prop->SetRot(prot[p]);
        prop->SetMass(pmass);
        prop->SetInertiaXX(pinerXX);
        prop->SetInertiaXY(pinerXY);
        prop->SetFixed(false);
        props.push_back(prop);
        sys_.AddBody(prop);

        auto motor = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
        chrono::ChQuaternion<> motor_rot = chrono::QUNIT;
        if (pdir[p]) { motor_rot = - chrono::Q_FLIP_AROUND_X * motor_rot; }
        motor->Initialize(prop, chassis, chrono::ChFrame<>(ppos[p], motor_rot));
        
        auto speed = chrono_types::make_shared<chrono::ChFunctionConst>(300);
        motor->SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::REVOLUTE);
        motor->SetSpeedFunction(speed);
        sys_.AddLink(motor);

        actuators.push_back(motor);
        speeds.push_back(speed);

        // Add the thrust forces
        auto thrust = chrono_types::make_shared<chrono::ChForce>();
        thrust->SetBody(prop.get());
        prop->AddForce(thrust);
        thrust->SetMode(chrono::ChForce::ForceType::FORCE);
        thrust->SetMforce(0);
        thrust->SetRelDir(chrono::ChVector3d(0,0,-1));
        thrusts.push_back(thrust);

        // Add the back torque
        auto backtorque = chrono_types::make_shared<chrono::ChForce>();
        backtorque->SetBody(prop.get());
        prop->AddForce(backtorque);
        backtorque->SetMode(chrono::ChForce::ForceType::TORQUE);
        backtorque->SetMforce(0);
        // Resistance Torque direction opposed to omega
        chrono::ChVector3d tdir = (pdir[p]) ? chrono::ChVector3d(0,0,-1) : chrono::ChVector3d(0,0,1);
        backtorque->SetRelDir(tdir);
        backtorques.push_back(backtorque);
                
    }

    // Add the visualization Assets
    AddVisualizationAssets();
}

template<int nop>
void UAV<nop>::AddVisualizationAssets()
{
    // chassis visual mesh
    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(chassis_mesh_path, true, true);
    auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetMutable(false);
    chassis->AddVisualShape(trimesh_shape, cor_m1);

    // propeller trimesh
    for (std::size_t i = 0; i < props.size(); ++i)
    {
        auto propeller = props[i];
        auto prop_trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(propeller_mesh_paths[i], true, true);
        auto prop_trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        prop_trimesh_shape->SetMesh(prop_trimesh);
        prop_trimesh_shape->SetMutable(false);
        propeller->AddVisualShape(prop_trimesh_shape, cor_m2[i]);
    }
}

template<int nop>
void UAV<nop>::AddChassisCollisionShapes(std::vector<std::shared_ptr<chrono::ChCollisionShape>> shapes,
                                         std::vector<chrono::ChFramed> frames)
{
    // Ensure that both vectors have the same length
    if (shapes.size() != frames.size()) {
        throw std::invalid_argument("Each Collision Shape must have a corresponding frame.");
    }

    chassis->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());
    chassis->EnableCollision(true);

    // Iterate through both vectors and add each collision shape with its corresponding frame
    for (size_t i = 0; i < shapes.size(); ++i) {
        auto& shape = shapes[i];
        auto& frame = frames[i];
        
        // Add the shape to the chassis collision model
        chassis->GetCollisionModel()->AddShape(shape, frame);
    }
}

template <int nop>
void UAV<nop>::AddChassisCollisionShapes(std::array<std::vector<std::shared_ptr<chrono::ChCollisionShape>>, nop> shapes,
                                         std::array<std::vector<chrono::ChFramed>, nop> frames)
{
    for (int i = 0; i < nop; ++i)
    {
        // Ensure that both vectors have the same length
        if (shapes[i].size() != frames[i].size()) {
            throw std::invalid_argument("Each Collision Shape must have a corresponding frame.");
        }

        props[i]->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());
        props[i]->EnableCollision(true);
        
        // Iterate through both vectors and add each collision shape with its corresponding frame
        for (std::size_t j = 0; j < shapes[i].size(); ++j)
        {
            const auto& shape = shapes[i][j];
            const auto& frame = frames[i][j];

            // Add the shape to the propeller collision model
            props[i]->GetCollisionModel()->AddShape(shape, frame);
        }
    }
}

template<int nop>
void UAV<nop>::AddConstraints(std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist)
{
    for (auto& link : linklist) { linklist_.push_back(link);  sys_.Add(link); }
}

template<int nop>
void UAV<nop>::SetupVisualization(chrono::utils::ChChaseCamera::State state, double chaseDist, double chaseHeight)
{   
    // Create the camera
    camera = chrono_types::make_shared<chrono::utils::ChChaseCamera>(this->GetChassis());
    camera->SetState(state);
    camera->Initialize(chrono::ChVector3d(0, 0, 0), this->GetChassis()->GetCoordsys(), chaseDist,  chaseHeight,
                       chrono::ChVector3d(0 , 0, -1), chrono::ChVector3d(0, 0, 0));

    // Setup the irrlicht visualization
    vis_.SetWindowSize(800, 600);
    vis_.SetWindowTitle("ACSL CHRONO SIMULATOR");
    vis_.SetCameraVertical(chrono::CameraVerticalDir::Z);
    vis_.Initialize();
    vis_.AddLogo();
    vis_.AddSkyBox();
    vis_.AddLight(chrono::ChVector3d(30, +30, -80), 280, chrono::ChColor(0.7f, 0.7f, 0.7f));
    vis_.AddLight(chrono::ChVector3d(30, -30, -80), 280, chrono::ChColor(0.7f, 0.7f, 0.7f));
    vis_.AddCamera(camera->GetCameraPos(), camera->GetTargetPos());
    vis_.AttachSystem(&this->sys_);
}

template<int nop>
void UAV<nop>::Update()
{
    // Render the scene
    vis_.BeginScene();
    vis_.Render(); 
    vis_.EndScene();
    // vis_.EnableCollisionShapeDrawing(true);
    // vis_.EnableBodyFrameDrawing(true);

    // Update the Irrlicht camera
    camera->Update(timestep);
    vis_.GetActiveCamera()->setPosition(irr::core::vector3dfCH(camera->GetCameraPos()));
    vis_.GetActiveCamera()->setTarget(irr::core::vector3dfCH(camera->GetTargetPos()));

    // Perform the integration step
    sys_.DoStepDynamics(timestep);

    // Spin in place to maintain soft real-time
    rtimer_.Spin(timestep);
    
}


}   // namespace _uav_
}   // namespace _patforms_




#endif  // UAV_HPP_
