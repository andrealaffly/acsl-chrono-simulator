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
 * File:        sim-uav.tpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class file for the uav baseclass
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_UAV_TPP_
#define SIM_UAV_TPP_

#include "sim-uav.hpp"

namespace _acsl_
{

namespace _uav_
{

// =========================================================================================================
// SetupInertialNEDFrame()
//
// Purpose:
//   Set up the Inertial NED (North–East–Down) reference frame in the Chrono
//   global simulation space. This frame acts as a fixed reference, typically
//   for interpreting UAV states with respect to world coordinates.
//
// Workflow:
//   1. Create the reference body for the NED frame.
//   2. Make it fixed (immovable).
//   3. Position it at the world origin.
//   4. Orient it correctly so the Chrono world (ENU) frame aligns as expected
//      with the UAV's NED convention.
//   5. Add it to the simulation system.
//
// Notes:
//   - This should be called before any other UAV setup—other UAV elements may
//     require a valid inertial reference frame.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetupInertialNEDFrame()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Create the Chrono body for the inertial NED frame
    //   - ChBodyAuxRef allows for a reference frame at an arbitrary pose
    // ------------------------------------------------------------------------
    InertialFrameNED = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    InertialFrameNED->SetName("ned-frame");

    // ------------------------------------------------------------------------
    // STEP 2 – Mark this frame as fixed (does not move during simulation)
    // ------------------------------------------------------------------------
    InertialFrameNED->SetFixed(true);

    // ------------------------------------------------------------------------
    // STEP 3 – Place the frame at the world origin (0,0,0)
    // ------------------------------------------------------------------------
    InertialFrameNED->SetPos(chrono::ChVector3d(0, 0, 0));

    // ------------------------------------------------------------------------
    // STEP 4 – Orient the frame to define NED relative to Chrono global frame
    //   - Use conversion utility to get the appropriate quaternion orientation.
    //   - Ensures NED axes map correctly to Chrono frame (usually ENU by default)
    // ------------------------------------------------------------------------
    InertialFrameNED->SetRot(::_shared_::_transformations_::GetChronoOrientaitonFromNED());

    // ------------------------------------------------------------------------
    // STEP 5 – Register this reference frame with the physics system
    //   - Allows other objects (e.g., UAV, sensors) to use it as a stable anchor.
    // ------------------------------------------------------------------------
    m_physics_.Add(GetInertialNEDFrameAuxBody());
}


// =========================================================================================================
// GetInertialNEDFrame()
//
// Purpose:
//   Retrieve the Chrono (ChFrame<>) representation of the Inertial NED (North-East-Down)
//   frame, which is typically used for visualization or further transformation operations.
//
// Safety:
//   - Checks if the Inertial Frame NED body pointer is valid before accessing.
//   - If not set, triggers the simulator error message.
//
// Returns:
//   Chrono::ChFrame<> constructed using the current position and rotation of
//   the Inertial NED frame's ChBodyAuxRef object.
// =========================================================================================================
template <int nop>
chrono::ChFrame<> simuav<nop>::GetInertialNEDFrame()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Safety check: Ensure the Inertial Frame NED has been set up
    // ------------------------------------------------------------------------
    if (!InertialFrameNED) { _message_::SIMULATOR_ERROR("[SIMUAV]: INERTIAL FRAME NED IS NOT FOUND"); }

    // ------------------------------------------------------------------------
    // STEP 2 – Construct a ChFrame based on NED body's current position/rotation
    //   - Get position and orientation (quaternion) from the ChBodyAuxRef.
    //   - This frame can be used directly for visualization or reference transforms.
    // ------------------------------------------------------------------------
    chrono::ChFrame<> NEDFrame(
        InertialFrameNED->GetPos(),   // NED origin in Chrono world coordinates
        InertialFrameNED->GetRot()    // NED orientation as quaternion
    );

    // ------------------------------------------------------------------------
    // STEP 3 – Return the frame to the caller
    // ------------------------------------------------------------------------
    return NEDFrame;
}


// =========================================================================================================
// InitiateUAVChassis()
//
// Purpose:
//   Create and configure the UAV's chassis (main rigid body) and prepare it for
//   simulation by setting its physical properties and attaching its visual model.
//
// Notes:
//   - This function only sets up the chassis object in memory and stores it in
//     `bodylist`. It does NOT add it to the physics system — that is done later 
//     in AddUAVToSystem().
//   - All chassis configuration parameters (mass, inertia, COM, initial pose,
//     visual file name, etc.) must be set before calling this function.
// =========================================================================================================
template <int nop>
void simuav<nop>::InitiateUAVChassis()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Create the Chrono body for the chassis
    //   ChBodyAuxRef supports an auxiliary reference frame (useful for NED frames).
    // ------------------------------------------------------------------------
    chassis.body = chrono_types::make_shared<chrono::ChBodyAuxRef>();

    // ------------------------------------------------------------------------
    // STEP 2 – Apply basic identifying and initial state properties
    // ------------------------------------------------------------------------
    chassis.body->SetName("chassis");             // Set body name - Always constant
    chassis.body->SetPos(chassis.init_pos);       // Initial position (Chrono coords)
    chassis.body->SetRot(chassis.init_rot);       // Initial orientation (Chrono quat)
    chassis.body->SetMass(chassis.mass);          // Mass in kg
    chassis.body->SetInertiaXX(chassis.InertiaXX);// Principal moments of inertia
    chassis.body->SetInertiaXY(chassis.InertiaXY);// Products of inertia (off-diagonal terms)
    chassis.body->SetFrameCOMToRef(chassis.COM);  // Center of mass frame relative to reference

    // ------------------------------------------------------------------------
    // STEP 2.1 – Create and attach biplane_frame marker at COM. Place marker 
    //            so its origin coincides with the body AuxRef origin: chassis.
    //            COM is the frame from AuxRef -> COM, so we need the opposite 
    //            offset (COM -> AuxRef) to bring the marker origin back to the 
    //            AuxRef origin. Additionally, rotate the marker +90 deg about
    //            the chassis Y axis to align it as a biplane frame.
    // ------------------------------------------------------------------------
    chassis.biplane_frame = chrono_types::make_shared<chrono::ChMarker>();
    chassis.biplane_frame->SetName("biplane_frame");

    // Attach the marker to the chassis body at the COM location
    chassis.body->AddMarker(chassis.biplane_frame);
    chrono::ChVector3d rel_pos_COM = -chassis.COM.GetPos(); 
    chrono::ChQuaternion<> rel_rot_COM = chassis.COM.GetRot(); 

    // +90 deg rotation about COM/chassis Y axis
    chrono::ChQuaternion<> q_y90 = chrono::QuatFromAngleY(chrono::CH_PI_2);
    chrono::ChQuaternion<> rel_rot_marker;
    rel_rot_marker.Cross(rel_rot_COM, q_y90);

    // Frame of marker relative to body reference (AuxRef) frame
    chrono::ChFramed marker_frame(rel_pos_COM, rel_rot_marker);
    chassis.biplane_frame->ImposeRelativeTransform(marker_frame);    

    // ------------------------------------------------------------------------
    // STEP 2.2 - Create and attach the chassis_drag_frame to the UAV. Place
    //            marker so its origin coincides with the body AuxRef origin:
    //            chassis. COM is the frame from AuxRef -> COM, so we need the
    //            opposite offset (COM -> AuxRef) to bring the marker origin
    //            back to the AuxRef origin. No additional rotation is needed.
    // ------------------------------------------------------------------------
    this->GetUAVChassisAerodynamics().chassis_drag_frame = chrono_types::make_shared<chrono::ChMarker>();
    this->GetUAVChassisAerodynamics().chassis_drag_frame->SetName("chassis_drag_frame");

    // Attach the marker to the chassis body at the COM location
    chassis.body->AddMarker(this->GetUAVChassisAerodynamics().chassis_drag_frame);
    chrono::ChQuaternion<> q_identity(1, 0, 0, 0); // No rotation for drag frame
    rel_rot_marker.Cross(rel_rot_COM, q_identity);

    // Frame of the marker relative to body reference (AuxRef) frame
    chrono::ChFramed drag_marker_frame(rel_pos_COM, rel_rot_marker);
    this->GetUAVChassisAerodynamics().chassis_drag_frame->ImposeRelativeTransform(drag_marker_frame);

    // Configure the forces needed for the drag
    this->GetUAVChassisAerodynamics().chassis_drag_force_x = chrono_types::make_shared<chrono::ChForce>();
    this->GetUAVChassisAerodynamics().chassis_drag_force_y = chrono_types::make_shared<chrono::ChForce>();
    this->GetUAVChassisAerodynamics().chassis_drag_force_z = chrono_types::make_shared<chrono::ChForce>();

    // Set the name for these drag forces
    this->GetUAVChassisAerodynamics().chassis_drag_force_x->SetName("chassis_drag_force_x");
    this->GetUAVChassisAerodynamics().chassis_drag_force_y->SetName("chassis_drag_force_y");
    this->GetUAVChassisAerodynamics().chassis_drag_force_z->SetName("chassis_drag_force_z");
    
    // Set the body for these drag forces
    this->GetUAVChassisAerodynamics().chassis_drag_force_x->SetBody(this->GetUAVChassisAerodynamics().chassis_drag_frame->GetBody());
    this->GetUAVChassisAerodynamics().chassis_drag_force_y->SetBody(this->GetUAVChassisAerodynamics().chassis_drag_frame->GetBody());
    this->GetUAVChassisAerodynamics().chassis_drag_force_z->SetBody(this->GetUAVChassisAerodynamics().chassis_drag_frame->GetBody());

    // Set the force type
    this->GetUAVChassisAerodynamics().chassis_drag_force_x->SetMode(chrono::ChForce::ForceType::FORCE);
    this->GetUAVChassisAerodynamics().chassis_drag_force_y->SetMode(chrono::ChForce::ForceType::FORCE);
    this->GetUAVChassisAerodynamics().chassis_drag_force_z->SetMode(chrono::ChForce::ForceType::FORCE);

    // Set the frame to apply this force
    this->GetUAVChassisAerodynamics().chassis_drag_force_x->SetFrame(chrono::ChForce::ReferenceFrame::BODY);
    this->GetUAVChassisAerodynamics().chassis_drag_force_y->SetFrame(chrono::ChForce::ReferenceFrame::BODY);
    this->GetUAVChassisAerodynamics().chassis_drag_force_z->SetFrame(chrono::ChForce::ReferenceFrame::BODY);

    // Configure the relative direction for each force
    this->GetUAVChassisAerodynamics().chassis_drag_force_x->SetRelDir(chrono::ChVector3d(1,0,0));
    this->GetUAVChassisAerodynamics().chassis_drag_force_y->SetRelDir(chrono::ChVector3d(0,1,0));
    this->GetUAVChassisAerodynamics().chassis_drag_force_z->SetRelDir(chrono::ChVector3d(0,0,1));

    // Point out the location in the chassis body frame where we want these forces
    // to be applied.
    this->GetUAVChassisAerodynamics().chassis_drag_force_x->SetVrelpoint(rel_pos_COM);
    this->GetUAVChassisAerodynamics().chassis_drag_force_y->SetVrelpoint(rel_pos_COM);
    this->GetUAVChassisAerodynamics().chassis_drag_force_z->SetVrelpoint(rel_pos_COM);

    // Finally add these forces to the body frame
    chassis.body->AddForce(this->GetUAVChassisAerodynamics().chassis_drag_force_x);
    chassis.body->AddForce(this->GetUAVChassisAerodynamics().chassis_drag_force_y);
    chassis.body->AddForce(this->GetUAVChassisAerodynamics().chassis_drag_force_z);

    // ------------------------------------------------------------------------
    // STEP 2.3 - Create and attach the total thrust force, Mx, My, Mz to COM
    // ------------------------------------------------------------------------
    // Thrust force at Center of Mass
    chassis.com_thrust = chrono_types::make_shared<chrono::ChForce>();      // Initiate the thrust force
    chassis.com_thrust->SetName("com_thrust");                              // Set the name as COM thrust
    chassis.com_thrust->SetBody(chassis.body.get());                        // Thrust is applied to the chassis
    chassis.com_thrust->SetMode(chrono::ChForce::ForceType::FORCE);         // Thrust is a force
    chassis.com_thrust->SetFrame(chrono::ChForce::ReferenceFrame::BODY);    // It is applied in the body frame
    chassis.com_thrust->SetRelDir(chrono::ChVector3d(0,0,-1));              // It is applied in the -ve z (NED)

    // thrust is applied at the COM
    chassis.com_thrust->SetVrelpoint(rel_pos_COM);

    // Add the force to the body frame
    chassis.body->AddForce(chassis.com_thrust);

    // Moment about x at the Center of Mass
    // Initiate the torque forces
    chassis.com_Mx = chrono_types::make_shared<chrono::ChForce>();
    chassis.com_My = chrono_types::make_shared<chrono::ChForce>();
    chassis.com_Mz = chrono_types::make_shared<chrono::ChForce>();
    // Set the name as com_m<>
    chassis.com_Mx->SetName("com_mx");
    chassis.com_My->SetName("com_my");
    chassis.com_Mz->SetName("com_mz");
    // Moment is applied to the chassis
    chassis.com_Mx->SetBody(chassis.body.get());
    chassis.com_My->SetBody(chassis.body.get());
    chassis.com_Mz->SetBody(chassis.body.get());
    // Moment is a troque
    chassis.com_Mx->SetMode(chrono::ChForce::ForceType::TORQUE);
    chassis.com_My->SetMode(chrono::ChForce::ForceType::TORQUE);
    chassis.com_Mz->SetMode(chrono::ChForce::ForceType::TORQUE);
    // It is applied in the body frame
    chassis.com_Mx->SetFrame(chrono::ChForce::ReferenceFrame::BODY);
    chassis.com_My->SetFrame(chrono::ChForce::ReferenceFrame::BODY);
    chassis.com_Mz->SetFrame(chrono::ChForce::ReferenceFrame::BODY);
    // Set the direction of the positive moment
    chassis.com_Mx->SetRelDir(chrono::ChVector3d(1,0,0));
    chassis.com_My->SetRelDir(chrono::ChVector3d(0,1,0));
    chassis.com_Mz->SetRelDir(chrono::ChVector3d(0,0,1));
    // Moments are applied at the COM
    chassis.com_Mx->SetVrelpoint(rel_pos_COM);
    chassis.com_My->SetVrelpoint(rel_pos_COM);
    chassis.com_Mz->SetVrelpoint(rel_pos_COM);
    // Add the torques to the body frame
    chassis.body->AddForce(chassis.com_Mx);
    chassis.body->AddForce(chassis.com_My);
    chassis.body->AddForce(chassis.com_Mz);

    // ------------------------------------------------------------------------
    // STEP 3 – Load and attach a visual mesh for the chassis
    // ------------------------------------------------------------------------
    // Load mesh geometry from Wavefront (.obj) file
    //   - First param: filepath = shapes_dir + chassis visual object file name
    //   - Second param: load normals
    //   - Third param: load UV texture coords
    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
        shapes_dir + chassis.vis_obj_name, true, true
    );

    // Create a Chrono visual shape from the mesh
    auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetMutable(false); // Make it immutable for performance

    // Add shape to body at zero offset with identity rotation
    chassis.body->AddVisualShape(
        trimesh_shape,
        chrono::ChFramed(
            chrono::ChVector3d(0, 0, 0),
            chrono::ChQuaternion<>(1, 0, 0, 0) // identity quaternion
        )
    );

    // ------------------------------------------------------------------------
    // STEP 4 - Add all the collision shapes to the chassis (IF present)
    // ------------------------------------------------------------------------
    if (!this->chassis.collision.empty()) {
        // Create the collision model
        chassis.body->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

        // Iterate through the shapes and frames in the collision tuple and add it to the model
        for (const auto& [shape, frame] : this->chassis.collision) {
            chassis.body->GetCollisionModel()->AddShape(shape, frame);
        }

        // Set chassis body as collidable
        chassis.body->EnableCollision(true);
    }


    // ------------------------------------------------------------------------
    // STEP 5 – Register chassis body into the UAV's internal body list
    //   This list will later be added to the physics system in AddUAVToSystem()
    // ------------------------------------------------------------------------
    bodylist.push_back(chassis.body);
}


// =========================================================================================================
// CheckUAVPropRequest(idx)
//
// Purpose: 
//   Checks if the requested propeller is within range of the total number of propellers.
//   Total number of propellers = 1 - NOP
// Parameters:
//   idx - 1-based index of the propeller hub to access (first propeller is idx=1).
//         Must satisfy 1 <= idx <= nop.
// =========================================================================================================
template <int nop>
void simuav<nop>::CheckUAVPropRequest(size_t idx)
{
    // 1-based -> 0-based adjustment and bounds check
    if (idx == 0 || idx > nop) {
        std::ostringstream oss;
        oss << "[SIMUAV]: PROPELLER INDEX OUT OF RANGE. VALID RANGE: 1 TO " << nop << ", REQUESTED: " << idx;
        _message_::SIMULATOR_ERROR(oss.str());
    }
}


// =========================================================================================================
// GetUAVProp(idx)
// 
// Purpose:
//   Provides direct access to the physical and visualization properties of an individual
//   propeller hub of the UAV by returning a reference to its propstruct.
//   This enables external code to read or modify any propeller's data, supporting flexible
//   setup and configuration of UAV components.
//
// Parameters:
//   idx - 1-based index of the propeller hub to access (first propeller is idx=1).
//         Must satisfy 1 <= idx <= nop.
//
// Returns:
//   Reference to the propstruct instance corresponding to the requested propeller hub.
//
// Notes:
//   - The underlying std::array still uses zero-based indexing; input idx is decremented internally.
//   - Throws std::out_of_range if idx is not a valid 1-based propeller index.
//   - Enables modular, type-safe manipulation of individual UAV propellers for advanced
//     configuration, tuning, or real-time updates.
// =========================================================================================================
template <int nop>
propstruct& simuav<nop>::GetUAVProp(size_t idx)
{
    this->CheckUAVPropRequest(idx);         // Check the request
    return props[idx - 1];                  // Grant if within range
}


// =========================================================================================================
// ConfigureUAVPropInitPos(idx, pos)
//
// Purpose:
//   Set the initial position of the specified propeller for simulation.
// 
// Parameters:
//   idx - 1-based index of the propeller hub (1 <= idx <= nop).
//   pos - Initial position of the propeller (Chrono world coordinates).
// 
// Notes:
//   - This must be called before InitiateUAVProps().
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropInitPos(size_t idx, chrono::ChVector3d pos)
{
    this->CheckUAVPropRequest(idx);                                         // Check the id of the propeller
    props[idx - 1].init_pos = ::_shared_::_transformations_::GetChronoPosFromNED(pos);         // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropInitRot(idx, rot)
//
// Purpose:
//   Set the initial orientation (quaternion) of the specified propeller.
// 
// Parameters:
//   idx - 1-based index of the propeller hub (1 <= idx <= nop).
//   rot - Initial orientation as Chrono quaternion.
// 
// Notes:
//   - This must be called before InitiateUAVProps().
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropInitRot(size_t idx, chrono::ChQuaternion<> rot)
{
    this->CheckUAVPropRequest(idx);                                         // Check the id of the propeller
    props[idx - 1].init_rot = ::_shared_::_transformations_::GetChronoOrientaitonFromNED(rot); // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropMass(mass)
//
// Purpose:
//   Set the mass for all propellers in the UAV.
// 
// Parameters:
//   mass - Mass value to assign to each propeller (kg).
// 
// Notes:
//   - Sets mass uniformly for all propellers prior to simulation.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropMass(size_t idx, double mass)
{
    this->CheckUAVPropRequest(idx);                                         // Check the id of the propeller
    props[idx - 1].mass = mass;             // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropInertiaXX(idx, IXX)
//
// Purpose:
//   Set the principal moments of inertia (diagonal) for a specified propeller.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   IXX - ChVector3d of inertia values.
// 
// Notes:
//   - Must be set before propeller is registered for simulation.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropInertiaXX(size_t idx, chrono::ChVector3d IXX)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].InertiaXX = IXX;          // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropInertiaXY(idx, IXY)
//
// Purpose:
//   Set the products of inertia (off-diagonal) for a specified propeller.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   IXY - ChVector3d of off-diagonal inertia values.
// 
// Notes:
//   - Must be set before physics registration.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropInertiaXY(size_t idx, chrono::ChVector3d IXY)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].InertiaXY = IXY;          // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropCOM(idx, COM)
//
// Purpose:
//   Set the center of mass (frame) for a specified propeller.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   COM - Chrono frame representing center of mass orientation and position.
// 
// Notes:
//   - Required before physics object creation.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropCOM(size_t idx, chrono::ChFrame<> COM)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].COM = COM;                // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropOBJName(idx, name)
//
// Purpose:
//   Set the visual mesh filename for a specified propeller.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   name - Wavefront .obj filename for propeller model.
// 
// Notes:
//   - Needed for visual mesh assignment during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropOBJName(size_t idx, std::string name)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].vis_obj_name = name;      // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropCollisionShapes(idx, list)
//
// Purpose:
//   Assign one or more collision shapes to a specified propeller body.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   list - Vector of CollisionShapeFrame objects describing the shapes.
// 
// Notes:
//   - Shapes should be set prior to physics engine registration.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropCollisionShapes(size_t idx, const std::vector<_acsl_::_uav_::CollisionShapeFrame>& list)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx - 1].collision = list;
}


// =========================================================================================================
// ConfigureUAVPropColor(idx, color)
//
// Purpose:
//   Set the prop color based on the predefined values in _acsl_::_uav_::_prop_color_
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   color - Color value for the prop
// 
// Notes:
//   - Needed for visual mesh assignment during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropColor(size_t idx, chrono::ChColor color)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].color = color;            // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropOpacity(idx, val)
//
// Purpose:
//   Set the prop opacity based on the float value input.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   val - Opacity value from 0 - 1. [ 0 - fully transparent | 1 - fully opaque ]
// 
// Notes:
//   - Needed for visual mesh assignment during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropOpacity(size_t idx, float val)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].opacity = val;            // Assign if within range
}


// =========================================================================================================
// InitiateUAVProps()
//
// Purpose:
//   Create and configure all UAV propeller bodies, preparing them for simulation.
// 
// Notes:
//   - This function registers all propellers in memory and stores them internally.
//   - Physical properties, geometry, and COM must be set before calling.
// =========================================================================================================
template <int nop>
void simuav<nop>::InitiateUAVProps()
{
    // Iterate through the total number of propellers
    for (int idx =0; idx < nop; ++idx)
    {
        // ------------------------------------------------------------------------
        // STEP 1 – Create the Chrono body for the propeller
        // ------------------------------------------------------------------------
        props[idx].body = chrono_types::make_shared<chrono::ChBodyAuxRef>();

        // ------------------------------------------------------------------------
        // STEP 2 – Apply identifying and initial state properties
        // ------------------------------------------------------------------------
        props[idx].body->SetName("propeller_" + std::to_string(idx +1));  // 1-based name
        props[idx].body->SetPos(props[idx].init_pos);
        props[idx].body->SetRot(props[idx].init_rot);
        props[idx].body->SetMass(props[idx].mass);
        props[idx].body->SetInertiaXX(props[idx].InertiaXX);
        props[idx].body->SetInertiaXY(props[idx].InertiaXY);
        props[idx].body->SetFrameCOMToRef(props[idx].COM);

        // ------------------------------------------------------------------------
        // STEP 3 – Load and attach a visual mesh for the propeller
        // ------------------------------------------------------------------------
        // Load mesh geometry from Wavefront (.obj) file
        //   - First param: filepath = shapes_dir + prop[idx] visual object file name
        //   - Second param: load normals
        //   - Third param: load UV texture coords
        auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
            shapes_dir + props[idx].vis_obj_name, true, true
        );

        // Create a Chrono visual shape from the mesh
        auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetMutable(false); // Make it immutable for performance
        trimesh_shape->SetColor(props[idx].color);
        trimesh_shape->SetOpacity(props[idx].opacity);

        // Add shape to body at zero offset with identity rotation
        props[idx].body->AddVisualShape(
            trimesh_shape,
            chrono::ChFramed(
                chrono::ChVector3d(0, 0, 0),
                chrono::ChQuaternion<>(1, 0, 0, 0)  // identity quaternion
            )
        );

        // ------------------------------------------------------------------------
        // STEP 4 - Add all the collision shapes to the propeller (IF present)
        // ------------------------------------------------------------------------
        if (!this->props[idx].collision.empty()) {
            // Create the collision model
            props[idx].body->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

            // Iterate through the shapes and frames in the collision tuple and add it to the model
            for (const auto& [shape, frame] : this->props[idx].collision) {
                props[idx].body->GetCollisionModel()->AddShape(shape, frame);
            }

            // Set propeller body as collidable
            props[idx].body->EnableCollision(true);
        }

        // ------------------------------------------------------------------------
        // STEP 5 – Register propeller body into the UAV's internal body list
        //   This list will later be added to the physics system in AddUAVToSystem()
        // ------------------------------------------------------------------------
        bodylist.push_back(props[idx].body);
    }
}


// =========================================================================================================
// LinkUAVBodies(link_data_vec)
//
// Purpose:
//   Constructs all Chrono link constraints between UAV bodies (e.g., chassis, propellers),
//   using the input vector of link property variants. Each link is created and stored
//   in the class's internal `linklist` for registration with the Chrono system later.
//
// Parameters:
//   link_data_vec - Vector of LinkData (variant property structs) specifying all
//                   constraint types and parameters between UAV bodies.
//
// Notes:
//   - Should be called after all bodies are constructed and stored in bodylist, 
//     but before AddUAVToSystem().
//   - The function does NOT add links to the Chrono system yet; it only prepares them.
//   - The input data is stored in the class member `links` for persistent access and future use.
// =========================================================================================================
template <int nop>
void simuav<nop>::LinkUAVBodies(const std::vector<LinkData>& link_data_vec)
{
    // Store a copy of the input link data in the class member for future/reference operations
    links = link_data_vec;

    // Iterate over all stored link property variants in the member vector
    for (const auto& link_data : links) {
        std::visit([&](const auto& prop) {
            std::shared_ptr<chrono::ChLinkBase> link;
            // Parallel
            if constexpr (std::is_same_v<std::decay_t<decltype(prop)>, LinkProperty<LinkType::Parallel>>) {
                link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
                std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(prop.flipped);
                std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(prop.bodyA, prop.bodyB, false, prop.cA, prop.cB, prop.dA, prop.dB);
                std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName(prop.name);
            }
            // Generic
            else if constexpr (std::is_same_v<std::decay_t<decltype(prop)>, LinkProperty<LinkType::Generic>>) {
                link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
                std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(
                    prop.cx, prop.cy, prop.cz, prop.rx, prop.ry, prop.rz);
                std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(
                    prop.bodyA, prop.bodyB, false, prop.cA, prop.cB, prop.dA, prop.dB);
                std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName(prop.name);
            }
            // DistanceZ
            else if constexpr (std::is_same_v<std::decay_t<decltype(prop)>, LinkProperty<LinkType::DistanceZ>>) {
                link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
                std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(
                    prop.bodyA, prop.bodyB, false, prop.cA, prop.cB, prop.dB);
                std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(prop.distance);
                std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName(prop.name);
            }
            // Orthogonal
            else if constexpr (std::is_same_v<std::decay_t<decltype(prop)>, LinkProperty<LinkType::Orthogonal>>) {
                link = chrono_types::make_shared<chrono::ChLinkMateOrthogonal>();
                std::dynamic_pointer_cast<chrono::ChLinkMateOrthogonal>(link)->Initialize(
                    prop.bodyA, prop.bodyB, false, prop.cA, prop.cB, prop.dA, prop.dB);
                std::dynamic_pointer_cast<chrono::ChLinkMateOrthogonal>(link)->SetName(prop.name);
            }
            linklist.push_back(link);
        }, link_data);
    }
}

// =========================================================================================================
// GetUAVMotor(idx)
// 
// Purpose:
//   Provides direct access to the of an individual motor associated with it's respective
//   propeller hub of the UAV by returning a reference to its motorstruct.
//   This enables external code to read or modify any motors's data, supporting flexible
//   setup and configuration of UAV components.
//
// Parameters:
//   idx - 1-based index of the propeller hub to access (first motor is idx=1).
//         Must satisfy 1 <= idx <= nop.
//
// Returns:
//   Reference to the motorstruct instance corresponding to the requested propeller hub.
//
// Notes:
//   - The underlying std::array still uses zero-based indexing; input idx is decremented internally.
//   - Throws std::out_of_range if idx is not a valid 1-based propeller index.
//   - Enables modular, type-safe manipulation of individual UAV propellers for advanced
//     configuration, tuning, or real-time updates.
// =========================================================================================================
template <int nop>
motorstruct& simuav<nop>::GetUAVMotor(size_t idx)
{
    this->CheckUAVPropRequest(idx);         // Check the request
    return motors[idx - 1];                 // Grant if within range
}


// =========================================================================================================
// ConfigureUAVMotorSpinDir(idx, spin)
//
// Purpose:
//   Set the motor spin based on the int value input.
// 
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   spin - Spin direction value -1 or 1. [ -1 - CW | 1 - CCW ]
// 
// Notes:
//   - Needed motor creation during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorSpinDir(size_t idx, int spin)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    motors[idx -1].spin_dir = spin;         // Assign if within range
}


// =========================================================================================================
// ConfigureUAVMotorFrame(idx, frame)
//
// Purpose:
//   Set the motor frame based on the frame value input.
// 
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   frame - Frame where the motor is located.
// 
// Notes:
//   - Needed motor creation during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorFrame(size_t idx, chrono::ChFrame<> frame)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    motors[idx -1].frame = frame;           // Assign if within range
}


// =========================================================================================================
// ConfigureUAVMotorNorm2Newt(idx, vec)
//
// Purpose:
//   Set the motor polynomial vector for converting normalized thrust values [0-1] to Newtons.
//
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   poly - Eigen::VectorXd containing polynomial coefficients.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorNorm2Newt(size_t idx, Eigen::VectorXd& poly)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].norm2newt = poly;
}


// =========================================================================================================
// ConfigureUAVMotorNewt2Norm(idx, vec)
//
// Purpose:
//   Set the motor polynomial vector for converting Newtons to normalized thrust values [0-1].
//
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   poly - Eigen::VectorXd containing polynomial coefficients.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorNewt2Norm(size_t idx, Eigen::VectorXd& poly)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].newt2norm = poly;
}


// =========================================================================================================
// ConfigureUAVMotorNorm2RPS(idx, vec)
//
// Purpose:
//   Set the motor polynomial vector for converting normalized thrust values [0-1] to rotor speed in rad/s.
//
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   poly - Eigen::VectorXd containing polynomial coefficients.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorNorm2RPS(size_t idx, Eigen::VectorXd& poly)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].norm2rps = poly;
}


// =========================================================================================================
// ConfigureUAVMotorCt(idx, ct)
//
// Purpose:
//   Set the motor torque constant that defines a linear realtionship between thrust and backtorque.
//
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   ct - double value for the ct
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorCt(size_t idx, double ct)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].ct = ct;
}

// =========================================================================================================
// ConfigureUAVMotorRPSScaler(idx, rps_scale)
//
// Purpose:
//   Set the constant that pre-multiplies the rps function for visualization. WILL ONLY AFFECT VIS.
//
// Parameters:
//   idx       - 1-based index (1 <= idx <= nop).
//   rps_scale - double value for the rps_scale
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorRPSScaler(size_t idx, double rps_scale)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].rps_scaler = rps_scale;
}


// =========================================================================================================
// InitiateUAVMotors()
//
// Purpose:
//   Create and configure all UAV motor bodies, preparing them for simulation.
// 
// Notes:
//   - This function registers all motors in memory and stores them internally.
//   - Spin direction and frame must be configured before the initiation
// =========================================================================================================
template <int nop>
void simuav<nop>::InitiateUAVMotors()
{
    // Iterate through the total number of propellers
    for (int idx = 0; idx < nop; ++idx)
    {
        // ------------------------------------------------------------------------
        // STEP 1 – Create the Chrono body for the motor
        // ------------------------------------------------------------------------
        motors[idx].motor = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
        motors[idx].motor->SetName("motor_" + std::to_string(idx + 1));  // 1-based name

        // ------------------------------------------------------------------------
        // STEP 2 – Prepare transformed position and orientation for the motor frame
        // ------------------------------------------------------------------------
        // Convert NED frame to Chrono frame for both position and orientation
        chrono::ChVector3<> motor_pos = ::_shared_::_transformations_::GetChronoPosFromNED(motors[idx].frame.GetPos());
        chrono::ChQuaternion<> base_rot = ::_shared_::_transformations_::GetChronoOrientaitonFromNED(motors[idx].frame.GetRot());

        // 180-degree rotation about the X axis for CCW case
        chrono::ChQuaternion<> rot_180_x;
        rot_180_x = chrono::QuatFromAngleX(chrono::CH_PI);

        chrono::ChQuaternion<> motor_rot;
        if (motors[idx].spin_dir == _motor_dir_::CCW)
        {
            motor_rot = rot_180_x * base_rot;   // Flip for CCW
        }    
        else
        {
            motor_rot = base_rot;               // Use the base orientation for CW
        }

        motors[idx].frame.SetPos(motor_pos);
        motors[idx].frame.SetRot(motor_rot);

        // ------------------------------------------------------------------------
        // STEP 3 – Initialize the Chrono motor link
        // ------------------------------------------------------------------------
        motors[idx].motor->Initialize(
            this->GetUAVProp(idx + 1).body,    // 1-based indexing for the motors 
            this->GetUAVChassis().body,
            motors[idx].frame);
        
        // Set the spindle constraint to revolute
        motors[idx].motor->SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::REVOLUTE);

        // ------------------------------------------------------------------------
        // STEP 4 – Set up the speed function and initial speed
        // ------------------------------------------------------------------------
        motors[idx].speed = chrono_types::make_shared<chrono::ChFunctionConst>();
        motors[idx].speed->SetConstant(0.0);   // Set the initial speed to 0 rpm
        motors[idx].motor->SetSpeedFunction(motors[idx].speed);

        // ------------------------------------------------------------------------
        // STEP 5 – Set up the thrust function and the initial thrust
        // ------------------------------------------------------------------------
        motors[idx].thrust = chrono_types::make_shared<chrono::ChForce>();      // Initiate the thrust force
        motors[idx].thrust->SetName("thrust_" + std::to_string(idx + 1));       // 1-based name
        motors[idx].thrust->SetBody(this->GetUAVChassis().body.get());          // Thrust is applied to the chassis
        motors[idx].thrust->SetMode(chrono::ChForce::ForceType::FORCE);         // It is of the type force
        motors[idx].thrust->SetFrame(chrono::ChForce::ReferenceFrame::BODY);    // It is applied in the body frame
        motors[idx].thrust->SetRelDir(chrono::ChVector3d(0,0,-1));              // It is applied in the -ve z (NED)
        
        // Iteratively compute where this thrust is applied wrt to the chassis and rotor hub.
        auto rel_coord = ::_shared_::_transformations_::GetNEDPosFromChrono( this->GetUAVProp(idx + 1).body->GetPos() 
                                                                 - this->GetUAVChassis().body->GetPos() );
        // Point out the location in the chassis body frame (NED)
        motors[idx].thrust->SetVrelpoint(rel_coord);
        motors[idx].thrust->SetMforce(0);                                       // Apply an initial thrust
        GetUAVChassis().body->AddForce(motors[idx].thrust);                     // Add this thrust force to the body

        // ------------------------------------------------------------------------
        // STEP 5 – Set up the torque function and the initial thrust
        // ------------------------------------------------------------------------
        motors[idx].torque = chrono_types::make_shared<chrono::ChForce>();      // Initiate the torque force
        motors[idx].torque->SetName("torque_" + std::to_string(idx + 1));       // 1-basead name
        motors[idx].torque->SetBody(this->GetUAVChassis().body.get());          // Thrust is applied to the chassis
        motors[idx].torque->SetMode(chrono::ChForce::ForceType::TORQUE);        // It is of the type torque
        motors[idx].torque->SetFrame(chrono::ChForce::ReferenceFrame::BODY);    // It is applied in the body frame
        // Compute the axis around which the backtorque is applied.
        // CCW : -Z NED | CW : Z NED
        if (motors[idx].spin_dir == _motor_dir_::CW)
        {
            motors[idx].torque->SetRelDir(chrono::ChVector3d(0,0,-1));
        }    
        else
        {
            motors[idx].torque->SetRelDir(chrono::ChVector3d(0,0,1));
        }

        // Point out the location in the chassis body frame (NED)
        motors[idx].torque->SetVrelpoint(rel_coord);
        motors[idx].torque->SetMforce(0);                                       // Apply an initial thrust
        GetUAVChassis().body->AddForce(motors[idx].torque);                     // Add this torque force to the body

        _message_::SIMULATOR_INFO("[SIMUAV]: ADDING MOTOR AT: ", rel_coord );
        // ------------------------------------------------------------------------
        // STEP 7 – Register the motor link in the UAV's internal link list
        //   This list will later be added to the physics system in AddUAVToSystem()
        // ------------------------------------------------------------------------
        linklist.push_back(motors[idx].motor);
    }
}

// =========================================================================================================
// ConfigureUAVAeroSurfaces()
// 
// Purpose:
//   Configures the aerodynamic surfaces for the UAV tail sitter, setting up the
//   aerodynamic centers for each wing and other relevant aerodynamic surfaces.
//   The aerodynamic centers are placed as midpoints of equally divided segments
//   between two given endpoints.
// Notes:
//   - The line segment from p1 to p2 is provided in the NED frame exported from
//     the SolidWorks plugin (CAD/NED coordinates).
//   - All intermediate computations for the center locations are done in this
//     CAD/NED frame, consistent with how the chassis COM and initial pose were
//     defined.
//   - For each aerodynamic center:
//       * A marker is created and attached to the chassis body (AuxRef frame).
//       * Two ChForce objects are created and attached to the chassis body:
//           - A lift force acting along the negative X-axis of the UAV body
//             frame (−X in body/NED).
//           - A drag force acting along the positive Z-axis of the UAV body
//             frame (+Z in body/NED).
//       * The forces are applied at the aerodynamic center location.
//       * rel_rotation specifies the relative orientation of the aerodynamic center 
//         marker with respect to NED frame, allowing for custom alignment 
//         of the aerodynamic center axes.
//   - Each marker and its lift/drag forces are named using the surface name and
//     center index for easy identification and debugging.
//   - The markers are rotated by the q_reltive quaternion about the chassis NED frame 
//     to align with the desired orientation of the aerodynamic center axes.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVAeroSurfaces(std::vector<aerodynamicsurfacestruct>& aero_surfaces)
{
    // ------------------------------------------------------------------------
    // STEP 0 – Iterate over the user setup aerodynamic surfaces and cache it.
    // ------------------------------------------------------------------------
    for (auto& surface : aero_surfaces) 
    {
        // ------------------------------------------------------------------------
        // STEP 1 – Compute aerodynamic center locations along the wing segment
        //
        //   - p1 and p2 are given in CAD/NED coordinates (from SolidWorks).
        //   - ComputeSegmentMidpoints interpolates directly in this NED frame and
        //     returns the midpoints in the same CAD/NED coordinates.
        // ------------------------------------------------------------------------
        auto aero_center_locations = 
           ::_shared_::_compute_::ComputeSegmentMidpoints(surface.p1, surface.p2, 
                                                          surface.num_of_aero_centers);
           
        // ------------------------------------------------------------------------
        // STEP 2 – Iterate through the centers and create markers and forces
        //
        //   For each aerodynamic center:
        //   - Create and configure a marker attached to the chassis body.
        //   - Compute its position relative to the chassis COM in the CAD/NED frame.
        //   - Impose the appropriate relative transform (position + orientation).
        //   - Create lift and drag forces at this center and register them.
        // ------------------------------------------------------------------------
        for (std::size_t i = 0; i < aero_center_locations.size(); ++i)
        {
            // --------------------------------------------------------------------
            // STEP 2.1 – Cache the aerodynamic center in the NED frame
            //
            //   - center is the aerodynamic center location in CAD/NED coordinates.
            // --------------------------------------------------------------------
            const auto& center = aero_center_locations[i];

            // Debug print for verification of aero center locations in NED
            _message_::SIMULATOR_INFO(
                "[SIMUAV]: AERO CENTER | surface_id: " + surface.name +
                " | center_" + std::to_string( i + 1) + 
                " -> x[" + std::to_string(center.x()) +
                "] y[" + std::to_string(center.y()) +
                "] z[" + std::to_string(center.z()) + "]"
            );

             // --------------------------------------------------------------------
            // STEP 2.2 – Create and name the aerodynamic center marker
            //
            //   - Create a marker for the aerodynamic center.
            //   - Name it using wing_id and center index (1-based).
            //   - Attach it to the chassis body (AuxRef frame).
            // --------------------------------------------------------------------
            auto marker = chrono_types::make_shared<chrono::ChMarker>();

            // Unique name for this aerodynamic center marker -> 1-based indexing
            marker->SetName(surface.name + "_aero_center_" + std::to_string(i + 1));

            // Attach the marker to the chassis body (AuxRef frame)
            this->GetUAVChassis().body->AddMarker(marker);

            // --------------------------------------------------------------------
            // STEP 2.3 – Compute relative position of the aerodynamic center
            //            with respect to the chassis COM, in the CAD/NED frame.
            //
            //   - center is in global CAD/NED coordinates (from CAD).
            //   - COM.GetPos() is the COM position stored in the chassis AuxRef /
            //     Chrono frame; GetNEDPosFromChrono brings it back to CAD/NED.
            //   - rel_pos_COM = center - COM_NED yields a pure relative vector from
            //     COM to the aerodynamic center, expressed in CAD/NED.
            //   - Because the chassis, COM, and aero centers are all specified in
            //     the same CAD/NED frame (and the UAV is symmetric about the NED
            //     x–z plane), this relative vector can be used directly as the
            //     marker position in the AuxRef frame.
            // --------------------------------------------------------------------
            chrono::ChVector3d rel_pos_COM =
            center
            - ::_shared_::_transformations_::GetNEDPosFromChrono(
                  this->GetUAVChassis().COM.GetPos());

            // --------------------------------------------------------------------
            // STEP 2.4 – Compute marker orientation in the body frame
            //
            //   - Start from the COM frame orientation in Chrono coordinates.
            //   - Apply a rotation about the chassis NED axis to align the
            //     marker axes with the desired aerodynamic frame convention.
            // --------------------------------------------------------------------
            chrono::ChQuaternion<> rel_rot_marker;
            rel_rot_marker.Cross(this->GetUAVChassis().COM.GetRot(), surface.q_relative);

            // --------------------------------------------------------------------
            // STEP 2.5 – Apply the relative transform to the marker
            //
            //   - rel_pos_COM is interpreted as the marker position relative to the
            //     chassis AuxRef frame.
            //   - rel_rot_marker is the marker orientation relative to AuxRef.
            //   - Because the chassis geometry and aero centers share the same
            //     CAD/NED basis and the UAV is symmetric about the NED x–z plane,
            //     this placement yields aero centers physically between the front
            //     two motors as expected (for the front wings).
            // --------------------------------------------------------------------
            chrono::ChFramed marker_frame(rel_pos_COM, rel_rot_marker);
            marker->ImposeRelativeTransform(marker_frame);

            // Add the marker to the aerodynamic center markers list for the specified surface
            surface.aerodynamic_center_frames.push_back(marker);

            // ------------------------------------------------------------------------
            // STEP 2.6 – Create and configure lift/drag forces at each aero center
            //
            //   For each aerodynamic center marker, create two ChForce objects:
            //   - A lift force acting along the desired-axis of the UAV body frame.
            //   - A drag force acting along the desired-axis of the UAV body frame.
            //
            //   Both forces:
            //   - Are attached to the chassis body.
            //   - Use the BODY reference frame so RelDir/Vrelpoint are in body coords.
            //   - Are named consistently with the aerodynamic center marker name.
            //   - Are stored in aero_surf_lift_forces and aero_surf_drag_forces so
            //     they can be updated during the simulation.
            // ------------------------------------------------------------------------

            // Create lift and drag forces associated with this aerodynamic center
            auto lift_force = chrono_types::make_shared<chrono::ChForce>();
            auto drag_force = chrono_types::make_shared<chrono::ChForce>();

            // Name tghem consistently with the aero center marker
            lift_force->SetName(surface.name + "_aero_center_" + std::to_string(i + 1)
                                + "_lift");

            drag_force->SetName(surface.name + "_aero_center_" + std::to_string(i + 1)
                                + "_drag");

            // Attach the forces to the chassis body
            lift_force->SetBody(this->GetUAVChassis().body.get());
            drag_force->SetBody(this->GetUAVChassis().body.get());

            // COnfigure as body-frame forces
            lift_force->SetMode(chrono::ChForce::ForceType::FORCE);
            drag_force->SetMode(chrono::ChForce::ForceType::FORCE);

            lift_force->SetFrame(chrono::ChForce::ReferenceFrame::BODY);
            drag_force->SetFrame(chrono::ChForce::ReferenceFrame::BODY);

            // Set the direction of force in each body frame:
            // For example, for a tailsitter
            //   - Lift acts along -X (negative body X axis).
            //   - Drag acts along +Z (positive body Z axis).
            //   - Note that these are set w.r.t to the NED frame as the drone is 
            //     imported in the NED frame, and we are applying the forces in the
            //     chassis' body frame of reference.
            lift_force->SetRelDir(surface.lift_force_relative_vector);
            drag_force->SetRelDir(surface.drag_force_relative_vector);

            // Apply both forces at the aerodynamic center location in the body frame.
            // rel_pos_COM is already the aerodynamic center position relative to the
            // chassis AuxRef frame.
            lift_force->SetVrelpoint(rel_pos_COM);
            drag_force->SetVrelpoint(rel_pos_COM);

            // Initialize magnitudes to zero; they will be updated in the aero model.
            lift_force->SetMforce(0.0);
            drag_force->SetMforce(0.0);
            
            // Register these forces with the chassis body
            this->GetUAVChassis().body->AddForce(lift_force);
            this->GetUAVChassis().body->AddForce(drag_force);

            // Store them in the aerodynamics struct so each center is tied to its 
            // corresponding lift and drag forces.
            surface.aero_surf_lift_forces.push_back(lift_force);
            surface.aero_surf_drag_forces.push_back(drag_force);

        }

        // ------------------------------------------------------------------------
        // STEP 3 – Resize aerodynamic data compuatation vectors to match centers.
        // ------------------------------------------------------------------------
        const std::size_t nC = surface.aerodynamic_center_frames.size();

        if (surface.alpha.size() != nC) { surface.alpha.resize(nC, 0.0); }

        if (surface.CL.size() != nC) { surface.CL.resize(nC, 0.0); }

        if (surface.CD.size() != nC) { surface.CD.resize(nC, 0.0); }

        if (surface.surf_aero_drag.size() != nC) { surface.surf_aero_drag.resize(nC, 0.0); }

        if (surface.surf_aero_lift.size() != nC) { surface.surf_aero_lift.resize(nC, 0.0); }       

        // ------------------------------------------------------------------------
        // STEP 4 – Store the aerodynamic surface information for later.
        // ------------------------------------------------------------------------
        this->aerosurfaces.push_back(surface);
    }
}

// =========================================================================================================
// GetUAVAerodynamicSurface(name)
// 
// Purpose:
//   Provides direct access to the of an individual aerodynamic surface associated with it's respective
//   name by returning a reference to its aerodynamicsurfacestruct.
//   This enables external code to read or modify any aerodynamic surface's data, supporting flexible
//   setup and configuration of UAV components.
//
// Parameters:
//   name - Name of the aerodynamic surface to access.
//         Must correspond to a valid surface name in the UAV's aerodynamic surfaces map.
//
// Returns:
//   Reference to the aerodynamicsurfacestruct instance corresponding to the requested aerodynamic surface.
//
// Notes:
//   - This function performs a linear search over the internal vector of aerodynamic
//     surfaces, comparing each element's `name` field to the requested name.
//   - If a matching aerodynamic surface is found, a non-const reference to its
//     `aerodynamicsurfacestruct` is returned, allowing the caller to modify its data
//     in-place (e.g., lift/drag parameters, marker frames, etc.).
//   - If no surface with the given name exists, the function reports an error via
//     `User_message_::SIMULATOR_ERROR("[SIMUAV]: CANNOT FIND AERODYNAMIC SURFACE <name>")`
//     and then throws a `std::runtime_error`. The caller should ensure that only
//     valid, pre-registered surface names are requested.
//   - Lookup cost is O(N) in the number of aerodynamic surfaces. For a small,
//     fixed number of surfaces (e.g., left/right wings, tail), this is acceptable
//     and keeps the data structure simple.
// =========================================================================================================
template <int nop>
aerodynamicsurfacestruct& simuav<nop>::GetUAVAerodynamicSurface(std::string name)
{
    // Search through the list of aerodynamic surfaces for one matching the requested name.
    for (auto& surface : aerosurfaces) {
        if (surface.name == name) {
            return surface;
        }
    }

    // Not found: notify user 
    _message_::SIMULATOR_ERROR("[SIMUAV]: CANNOT FIND REQUESTED AERODYNAMIC SURFACE " + name);

}


// =========================================================================================================
// AddUAVToSystem()
// 
// Purpose:
//   Finalize the UAV's addition to the Chrono simulation by registering all 
//   the rigid bodies and kinematic/dynamic links that make up the UAV 
//   into the physics system (`m_physics_`).
//
// Notes:
//   - This function should be called *after* the UAV components (chassis, 
//     propellers, sensors, etc.) have been fully created and added to 
//     `bodylist` and `linklist`.
//   - Once registered, Chrono will manage these objects during simulation 
//     (integration, collision detection, constraints).
// =========================================================================================================
template <int nop>
void simuav<nop>::AddUAVToSystem()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Add rigid bodies to the physics system
    //   - `bodylist` contains all the ChBodyAuxRef objects for this UAV.
    //   - Each is added to the ChSystemNSC instance so it participates in 
    //     physics simulation (forces, collisions, integration).
    // ------------------------------------------------------------------------
    for (auto& body : bodylist) {
        if (body) {
            _message_::SIMULATOR_INFO("[SIMUAV]: ADDING " + body->GetName() + " BODY TO SYSTEM"); 
            m_physics_.Add(body);
        }
    }

    // ------------------------------------------------------------------------
    // STEP 2 – Add links (joints/constraints) to the physics system
    //   - `linklist` contains the joints/constraints (ChLinkBase objects) 
    //     that connect bodies (e.g., propeller hubs to chassis, fixed joints).
    //   - Adding these ensures that the kinematic/dynamic constraints are enforced.
    // ------------------------------------------------------------------------
    for (auto& link : linklist) {
        if (link) {
            _message_::SIMULATOR_INFO("[SIMUAV]: ADDING " + link->GetName() + " LINK TO SYSTEM"); 
            m_physics_.Add(link);
        }
    }

    // UAV is now fully part of the simulation world
}


// =========================================================================================================
// GetUAVStateData()
// 
// Purpose:
//   Collect and return the UAV's state (position, velocity, acceleration,
//   orientation, angular velocity/acceleration) in the NED frame.
//
// Returns:
//   m_states struct containing the UAV's full state in NED coordinates.
// =========================================================================================================
template <int nop>
m_states simuav<nop>::GetUAVStateData()
{
    // Object to store and return the state data
    m_states m_state;

    // ------------------------------------------------------------------------
    // STEP 1 - Get frame transformations
    // ------------------------------------------------------------------------

    // NED frame relative to Chrono's absolute world frame
    chrono::ChFrame<> ned_abs = GetInertialNEDFrameAuxBody()->GetFrameRefToAbs();

    // UAV chassis frame relative to Chrono's absolute world frame
    chrono::ChFrame<> uav_abs = GetUAVChassis().body->GetFrameRefToAbs();

    // UAV biplane frame relative to Chrono's absolute world frame
    chrono::ChFrame<> uav_biplane_abs = GetUAVChassis().biplane_frame->GetAbsFrame();

    // UAV chassis frame relative to the NED frame
    //   - Inverse transform of NED_abs gives Abs->NED conversion
    //   - Multiply by uav_abs to express UAV pose w.r.t NED origin
    chrono::ChFrame<> uav_ned = ned_abs.GetInverse() * uav_abs;

    // UAV biplane frame reltive to the NED frame
    //   - Inverse transform of NED_abs gives Abs->NED conversion
    //   - Multiply by uav_biplane_abs to express UAV biplane pose w.r.t NED origin
    chrono::ChFrame<> uav_biplane_ned = ned_abs.GetInverse() * uav_biplane_abs;

    // ------------------------------------------------------------------------
    // STEP 2 - Get linear kinematics in absolute frame
    // ------------------------------------------------------------------------

    // Linear velocity (absolute frame)
    chrono::ChVector3d uav_vel_abs = GetUAVChassis().body->GetPosDt();

    // Linear acceleration (absolute frame)
    chrono::ChVector3d uav_acc_abs = GetUAVChassis().body->GetPosDt2();

    // ------------------------------------------------------------------------
    // STEP 3 - Prepare NED rotation transformation
    // ------------------------------------------------------------------------

    // Rotation matrix of the NED frame relative to absolute frame
    //   - Used to convert vectors from Abs -> NED coordinates and vice-versa.
    chrono::ChMatrix33d R_ned = ned_abs.GetRotMat();

    // Rotation matrix of the drone relative to the NED frame
    //   - Used to convert vectors from I -> J coordinates and vice-versa.
    chrono::ChMatrix33d R_uav = uav_ned.GetRotMat();

    // ------------------------------------------------------------------------
    // STEP 4 - Fill state structure
    // ------------------------------------------------------------------------

    // --- Time Of the Simulation ---
    m_state.time = this->m_physics_.GetChTime();

    // --- Position in NED frame ---
    m_state.pos = uav_ned.GetPos();

    // --- Linear velocity in NED frame ---
    // Apply rotation transpose (i.e., inverse) to convert from Abs to NED
    m_state.vel = R_ned.transpose() * uav_vel_abs;

    // --- Linear acceleration in NED frame ---
    m_state.acc = R_ned.transpose() * uav_acc_abs;

    // --- Orientation in NED frame ---
    // Quaternion (scalar-last in Chrono convention)
    m_state.quat = uav_ned.GetRot();

    // Quaternion (scalar-last in Chrono convention)
    m_state.quat_bp = uav_biplane_ned.GetRot();

    // Euler angles from quaternion (XYZ Cardan convention)
    m_state.eul = m_state.quat.GetCardanAnglesXYZ();
    // m_state.eul = ::_shared_::_conversions_::QuaternionToEulerAnglesRPY_321(m_state.quat);

    // Euler angles from quaternion (XYZ Cardan convention) -  Nasty fix
    // auto eul_bp_raw = m_state.quat_bp.GetCardanAnglesXYZ();
    auto eul_bp_raw = ::_shared_::_conversions_::QuaternionToEulerAnglesRPY_321_from231(m_state.quat_bp);
    // m_state.eul_bp = ::_shared_::_compute_::RegularizeCardanXYZ(eul_bp_raw, m_state.eul);
    m_state.eul_bp = eul_bp_raw;

    // --- Angular velocity (in UAV local frame) ---
    m_state.ovel = GetUAVChassis().body->GetAngVelLocal();

    // --- Angular velocity (in Biplane local frame) ---
    m_state.ovel_bp.x() = -1.0 * m_state.ovel.z();
    m_state.ovel_bp.y() = m_state.ovel.y();
    m_state.ovel_bp.z() = m_state.ovel.x();

    // --- Angular acceleration (in UAV local frame) ---
    m_state.oacc = GetUAVChassis().body->GetAngAccLocal();

    // --- Angular acceleration (in Biplane local frame) ---
    m_state.oacc_bp.x() = -1.0 * m_state.oacc.z();
    m_state.oacc_bp.y() = m_state.oacc.y();
    m_state.oacc_bp.z() = m_state.oacc.x();

    // --- Forces acting on the body in the Inertial frame (NED) ---
    m_state.muI = ::_shared_::_transformations_::GetNEDPosFromChrono(GetUAVChassis().body->GetAppliedForce());

    // --- Forces acting on the body in the Body frame (NED) ---
    m_state.muJ =  R_uav.transpose() * m_state.muI;

    // --- Torque acting on the body in the Body frame (NED) ---
    m_state.tauJ = GetUAVChassis().body->GetAppliedTorque();

    // Return the filled state struct
    return m_state;
}

// =========================================================================================================
// GetUAVPropStateData()
// 
// Purpose:
//   Collect and return the UAV propeller's state (position, velocity, acceleration,
//   orientation, angular velocity/acceleration) in the NED frame.
//
// Returns:
//   m_states struct containing the UAV propeller's full state in NED coordinates.
// =========================================================================================================
template <int nop>
m_states simuav<nop>::GetUAVPropStateData(size_t idx)
{
    // Object to store and return the state data
    m_states m_state;

    // ------------------------------------------------------------------------
    // STEP 1 - Get frame transformations
    // ------------------------------------------------------------------------

    // NED frame relative to Chrono's absolute world frame
    chrono::ChFrame<> ned_abs = GetInertialNEDFrameAuxBody()->GetFrameRefToAbs();

    // UAV chassis frame relative to Chrono's absolute world frame
    chrono::ChFrame<> prop_abs = GetUAVProp(idx).body->GetFrameRefToAbs();

    // UAV chassis frame relative to the NED frame
    //   - Inverse transform of NED_abs gives Abs->NED conversion
    //   - Multiply by uav_abs to express UAV pose w.r.t NED origin
    chrono::ChFrame<> prop_ned = ned_abs.GetInverse() * prop_abs;

    // ------------------------------------------------------------------------
    // STEP 2 - Get linear kinematics in absolute frame
    // ------------------------------------------------------------------------

    // Linear velocity (absolute frame)
    chrono::ChVector3d prop_vel_abs = GetUAVProp(idx).body->GetPosDt();

    // Linear acceleration (absolute frame)
    chrono::ChVector3d prop_acc_abs = GetUAVProp(idx).body->GetPosDt2();

    // ------------------------------------------------------------------------
    // STEP 3 - Prepare NED rotation transformation
    // ------------------------------------------------------------------------

    // Rotation matrix of the NED frame relative to absolute frame
    //   - Used to convert vectors from Abs -> NED coordinates
    chrono::ChMatrix33d R_ned = ned_abs.GetRotMat();

    // Rotation matrix of the prop relative to the NED frame
    //   - Used to convert vectors from I -> J coordinates and vice-versa.
    chrono::ChMatrix33d R_prop = prop_ned.GetRotMat();

    // ------------------------------------------------------------------------
    // STEP 4 - Fill state structure
    // ------------------------------------------------------------------------

    // --- Time Of the Simulation ---
    m_state.time = this->m_physics_.GetChTime();

    // --- Position in NED frame ---
    m_state.pos = prop_ned.GetPos();

    // --- Linear velocity in NED frame ---
    // Apply rotation transpose (i.e., inverse) to convert from Abs to NED
    m_state.vel = R_ned.transpose() * prop_vel_abs;

    // --- Linear acceleration in NED frame ---
    m_state.acc = R_ned.transpose() * prop_acc_abs;

    // --- Orientation in NED frame ---
    // Quaternion (scalar-last in Chrono convention)
    m_state.quat = prop_ned.GetRot();

    // Euler angles from quaternion (XYZ Cardan convention)
    m_state.eul = m_state.quat.GetCardanAnglesXYZ();

    // --- Angular velocity (in UAV local frame) ---
    m_state.ovel = GetUAVProp(idx).body->GetAngVelLocal();

    // --- Angular acceleration (in UAV local frame) ---
    m_state.oacc = GetUAVProp(idx).body->GetAngAccLocal();

    // --- Forces acting on the body in the Inertial frame (NED) ---
    m_state.muI = ::_shared_::_transformations_::GetNEDPosFromChrono(GetUAVProp(idx).body->GetAppliedForce());

    // --- Forces acting on the body in the Body frame (NED) ---
    m_state.muJ =  R_prop.transpose() * m_state.muI;

    // --- Torque acting on the body in the Body frame (NED) ---
    m_state.tauJ = GetUAVProp(idx).body->GetAppliedTorque();

    // Return the filled state struct
    return m_state;
}

// =========================================================================================================
// SetActuator(idx, thrust, torque, rpm)
// 
// Purpose:
//   Sets the motor thrust, backtorque and the rpm setpoints.
//
// Parameters:
//   idx    - 1-based index (1 <= idx <= nop).
//   thrust - thrust value in (N)
//   torque - troque value in (Nm)
//   rpm    - angular velocity setpoint of the rotors in (rad/s)
//   
// 
// Notes:
//   - Needed motor creation during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetActuator(size_t idx, double thrust, double torque, double rpm)
{
    // Set the thrust setpoint
    this->GetUAVMotor(idx).thrust->SetMforce(thrust);

    // Set the torque setpoint
    this->GetUAVMotor(idx).torque->SetMforce(torque);

    // Set the motor rpm
    this->GetUAVMotor(idx).speed->SetConstant(rpm);
}


// =========================================================================================================
// SetThrustSetPoint(idx, thrustSP)
// 
// Purpose:
//   Sets the motor thrust, backtorque and the rpm setpoints based on the normalized thrust setpoint input
//
// Parameters:
//   idx      - 1-based index (1 <= idx <= nop).
//   thrustSP - thrust value in normalized terms [0-1] (-)
// 
// Notes:
//   - Needed motor creation during simulation setup.
//   - Needed motor polynomial coefficient vectors to be configured during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetThrustSetPoint(size_t idx, double thrustSP)
{
    // Check if the vectors are not empty
    if (motors[idx - 1].norm2newt.size() == 0) 
    { _message_::SIMULATOR_ERROR("[SIMUAV]: THRUST NORMAL VALUE TO NEWTON POLYNOMIAL NOT CONFIGURED"); }
    if (motors[idx - 1].norm2rps.size() == 0)
    { _message_::SIMULATOR_ERROR("[SIMUAV]: THRUST NORMAL VALUE TO RPS FOR PROP SPEED NOT CONFIGURED"); }

    // If it passes the checks then >>
    // Cache the values
    auto thrust = ::_shared_::_compute_::evaluatePolynomial(motors[idx - 1].norm2newt, thrustSP);
    auto torque = 0.15 * motors[idx - 1].ct * thrust;       // Cq = 0.15 * ct -> Cq is the backtorque constant
    
    // IRRLICHT SYSTEM IS FINNICKY. HENCE THE MODIFIER. - ONLY AFFECTS VISUALIZATON
    auto rps = motors[idx - 1].rps_scaler * (::_shared_::_compute_::evaluatePolynomial(motors[idx - 1].norm2rps, 
                                                                                       thrustSP));

    // Set the values
    this->SetActuator(idx, thrust, torque, rps);
}

// =========================================================================================================
// SetControlInputAtCOM(u1, u2, u3, u4)
// 
// Purpose:
//   Sets the thrust, torques control inputs at the center of mass
//
// Parameters:
// u1 - total thrust
// u2 - moment about x axis (NED) body frame
// u3 - moment about y axis (NED) body frame
// u4 - moment about z axis (NED) body frame
// 
// Notes:
//   - The forces are automatically setup during the chassis configuration/initiation functions.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetControlInputAtCOM(double u1, double u2, double u3, double u4)
{
    // Set the force and torque setpoints
    this->GetUAVChassis().com_thrust->SetMforce(u1);
    this->GetUAVChassis().com_Mx->SetMforce(u2);
    this->GetUAVChassis().com_My->SetMforce(u3);
    this->GetUAVChassis().com_Mz->SetMforce(u4);
}

// =========================================================================================================
// SetChassisDrag()
// 
// Purpose:
//   Sets the drag force acting on the UAV chassis based on the input drag coefficient and the current 
//   velocity of the UAV.
// 
// Parameters:
//   none
// 
// Notes:
//   - The drag coefficient should be configured during the UAV setup process else will throw an error if 
//     the chassis drag function is set but the coefficient is not configured.
//   - The drag force is calculated using the formula: F_drag = -(0.5 * \rho * C_d * S * \| v \| * v)
//     where \rho is the air density, C_d is the drag coefficient, S is the surface area of the chassis 
//     body alone, and v is the velocity vector of the UAV. The UAVs body is considered a flat plane in 
//     this instance.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetChassisDrag()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Get NED frame and drag frame in absolute coordinates
    //   - ned_abs: NED frame pose w.r.t. Chrono's global (absolute) frame.
    //   - drag_abs: chassis drag frame pose w.r.t. Chrono's global frame.
    // ------------------------------------------------------------------------
    // NED frame relative to Chrono's absolute world frame
    chrono::ChFrame<> ned_abs = this->GetInertialNEDFrameAuxBody()->GetFrameRefToAbs();

    // UAV drag frame relative to Chrono's absolute world frame
    chrono::ChFrameMoving<> drag_abs = this->GetUAVChassisAerodynamics().chassis_drag_frame->GetAbsFrame();


    // ------------------------------------------------------------------------
    // STEP 2 – Express drag frame relative to NED frame
    //   - Use inverse of NED_abs to map from Abs -> NED.
    //   - Composition gives drag frame pose in NED coordinates.
    // ------------------------------------------------------------------------
    // UAV drag frame relative to the NED frame
    //   - Inverse transform of ned_abs gives Abs->NED conversion
    //   - Multiply by drag_abs to express UAV drag frame pose w.r.t NED origin
    chrono::ChFrame<> drag_ned = ned_abs.GetInverse() * drag_abs;


    // ------------------------------------------------------------------------
    // STEP 3 – Get linear velocity of the drag frame in the absolute frame
    // ------------------------------------------------------------------------
    // Get the linear kinematics of the drag frame in the absolute frame
    chrono::ChVector3d drag_vel_abs = drag_abs.GetPosDt();


    // ------------------------------------------------------------------------
    // STEP 4 – Build rotation matrices for frame transformations
    //   - R_ned: rotation of NED frame relative to Abs (Abs -> NED).
    //   - R_drag: rotation of drag frame relative to NED (NED -> body J).
    // ------------------------------------------------------------------------
    // Prepare NED rotation transformation
    // Rotation matrix of the NED frame relative to the absolute frame
    //  - Used to convert vectors from Abs -> NED coordinates and vice-versa.
    chrono::ChMatrix33d R_ned = ned_abs.GetRotMat();

    // Rotation matrix of the drone relative to the NED frame
    //  - Used to convert vectors from I -> J coordinates and vice-versa.
    chrono::ChMatrix33d R_drag = drag_ned.GetRotMat();


    // ------------------------------------------------------------------------
    // STEP 5 – Transform drag frame linear velocity into NED and then body frame
    // ------------------------------------------------------------------------
    // Linear velocity of the drag frame in the NED frame
    // Apply rotation transpose (i.e., inverse) to convert from Abs to NED
    auto drag_vel_ned = R_ned.transpose() * drag_vel_abs;

    // Linear velocity of the drag frame in the body frame J
    // Apply rotation transpose (i.e., inverse) to convert from NED to body frame
    auto drag_vel_body = R_drag.transpose() * drag_vel_ned;


    // ------------------------------------------------------------------------
    // STEP 6 – Compute drag force in the body frame
    //   F_drag = -(0.5 * rho * C_d * S * |v| * v)
    //   - Uses the body-frame velocity at the drag frame.
    // ------------------------------------------------------------------------
    this->GetUAVChassisAerodynamics().chassis_drag_force = -0.5 * this->GetUAVChassisAerodynamics().air_density 
                                                                * this->GetUAVChassisAerodynamics().chassis_drag_coefficient 
                                                                * this->GetUAVChassisAerodynamics().chassis_body_surface_aera 
                                                                * drag_vel_body.Length() * drag_vel_body;    
    
    
    // ------------------------------------------------------------------------
    // STEP 7 – Apply drag force components to the individual body-frame forces
    //   - Each ChForce is aligned with a body axis and scaled by the corresponding
    //     component of the computed drag force vector.
    // ------------------------------------------------------------------------
    // Apply the drag force to the body
    this->GetUAVChassisAerodynamics().chassis_drag_force_x->SetMforce(this->GetUAVChassisAerodynamics().chassis_drag_force.x());
    this->GetUAVChassisAerodynamics().chassis_drag_force_y->SetMforce(this->GetUAVChassisAerodynamics().chassis_drag_force.y());
    this->GetUAVChassisAerodynamics().chassis_drag_force_z->SetMforce(this->GetUAVChassisAerodynamics().chassis_drag_force.z());
}


// =========================================================================================================
// SetUAVAeroSurfaceLiftDrag()
// 
// Purpose:
//   Sets the lift and drag for the uav aerodynamic surfaces according to the sim-aerofoil
// 
// Parameters:
//   none
// 
// Notes:
//   - The wing span and chord should be configured during the UAV setup process else will throw an error.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetUAVAeroSurfaceLiftDrag()
{
    // --------------------------------------------------------------------------------------------
    // STEP 0 – Iterate through the aerosurfaces struct and precache variables for computation
    // --------------------------------------------------------------------------------------------
    for (auto& surface : this->GetUAVAerodynamicSurfaces())
    {
        // --------------------------------------------------------------------------------------------
        // STEP 1 – Compute local aerodynamic surface area per aerodynamic segment
        //   - Treat each aerodynamic center as representing an equal spanwise strip.
        //   - S_seg = (span * chord) / N_segments, where N_segments is the number of markers.
        // --------------------------------------------------------------------------------------------
        const std::size_t num_centers = surface.aerodynamic_center_frames.size();
        const double S_per_seg = (num_centers > 0)
                                 ? (surface.aerofoil_span * surface.aerofoil_chord / static_cast<double>(num_centers))
                                 : 0.0;
                                
        // --------------------------------------------------------------------------------------------
        // STEP 2 – Get NED frame pose and rotation in the absolute (Chrono) frame
        //   - ned_abs: pose of the inertial NED frame with respect to Chrono's global frame.
        //   - R_ned  : rotation matrix that maps vectors from Abs -> NED coordinates.
        // --------------------------------------------------------------------------------------------
        chrono::ChFrame<> ned_abs = this->GetInertialNEDFrameAuxBody()->GetFrameRefToAbs();
        chrono::ChMatrix33d R_ned = ned_abs.GetRotMat();

        // --------------------------------------------------------------------------------------------
        // STEP 3– Loop over all aerodynamic center markers on the surface of the body
        //   For each marker i:
        //     1) Get its pose and velocity in the absolute frame.
        //     2) Express its pose relative to NED.
        //     3) Build rotation matrices for NED and marker frames.
        //     4) Transform its linear velocity into NED and then marker body frame.
        //     5) Compute and apply the lift and drag force
        // --------------------------------------------------------------------------------------------
        for (std::size_t i = 0; i < num_centers; ++i)
        {

            // ----------------------------------------------------------------------------------------
            // STEP 3.1 – Get aerodynamic center frame in absolute coordinates
            //   - marker_abs: pose and velocity of the aerodynamic center marker in Abs frame.
            // ----------------------------------------------------------------------------------------
            auto marker_abs = surface.aerodynamic_center_frames[i]->GetAbsFrame();

            // ----------------------------------------------------------------------------------------
            // STEP 3.2 – Express aerodynamic center frame relative to NED frame
            //   - marker_ned: pose of the marker with respect to the NED frame.
            //   - This uses ned_abs^{-1} to map from Abs -> NED and composes with marker_abs.
            // ----------------------------------------------------------------------------------------
            chrono::ChFrame<> marker_ned = ned_abs.GetInverse() * marker_abs;

            // ----------------------------------------------------------------------------------------
            // STEP 3.3 – Build rotation matrix for marker frame
            //   - R_marker: rotation of the marker frame relative to NED.
            //   - Used to transform vectors between NED and the local marker body frame.
            // ----------------------------------------------------------------------------------------
            chrono::ChMatrix33d R_marker = marker_ned.GetRotMat();

            // ----------------------------------------------------------------------------------------
            // STEP 3.4 – Get linear velocity of the marker in the absolute frame
            //   - vel_abs: linear velocity of the aerodynamic center marker in Abs coordinates.
            // ----------------------------------------------------------------------------------------
            chrono::ChVector3d vel_abs = marker_abs.GetPosDt();

            // ----------------------------------------------------------------------------------------
            // STEP 3.5 – Transform marker velocity from Abs -> NED frame
            //   - Use R_ned^T (inverse of R_ned) to convert velocity from Abs to NED coordinates.
            // ----------------------------------------------------------------------------------------
            chrono::ChVector3d vel_ned = R_ned.transpose() * vel_abs;

            // ----------------------------------------------------------------------------------------
            // STEP 3.6 – Transform marker velocity from NED -> local marker body frame
            //   - Use R_marker^T (inverse of R_marker) to convert velocity from NED to marker frame.
            //   - vel_body now contains the velocity components in the aerodynamic center's body frame.
            // ----------------------------------------------------------------------------------------
            chrono::ChVector3d vel_body = R_marker.transpose() * vel_ned;

            // ----------------------------------------------------------------------------------------
            // NOTE:
            //   At this point, vel_body.x(), vel_body.y(), vel_body.z() give the body-frame velocity
            //   components at the aerodynamic center:
            //     - Use vel_body to compute angle of attack.
            //     - Then evaluate CL/CD/CM and construct lift/drag forces in this local frame.
            //
            // ----------------------------------------------------------------------------------------

            // ----------------------------------------------------------------------------------------
            // STEP 3.7 – Find the angle of attack of each aerodynamic center
            //   - \alpha = atan2(vel_body.z(), vel_body.x())
            // ----------------------------------------------------------------------------------------
            surface.alpha[i] = atan2(vel_body.z(), vel_body.x());

            // ----------------------------------------------------------------------------------------
            // STEP 3.8 – Find the coefficeint of lift of each aerodynamic center
            //   - CL = ComputeCL(alpha)
            // ----------------------------------------------------------------------------------------
            surface.CL[i] = this->ComputeCL(surface.alpha[i]);

            // ----------------------------------------------------------------------------------------
            // STEP 3.9 – Find the coefficeint of drag of each aerodynamic center
            //   - CD = ComputeCD(alpha)
            // ----------------------------------------------------------------------------------------
            surface.CD[i] = this->ComputeCD(surface.alpha[i]);

            // ----------------------------------------------------------------------------------------
            // STEP 3.10 – Compute the lift and drag forces 
            //
            //   Definitions:
            //     - vel_body : local relative wind velocity at this aerodynamic center,
            //                  expressed in the marker's body frame [m/s].
            //     - V_mag    : magnitude of vel_body, i.e. |V| [m/s].
            //     - rho      : air density (surface.air_density) [kg/m^3].
            //     - S_per_seg: reference area represented by this aerodynamic segment
            //                  (span * chord / number_of_segments) [m^2].
            //     - cl       : lift coefficient C_L at the local angle of attack [-].
            //     - cd       : drag coefficient C_D at the local angle of attack [-].
            //
            //   Full formulas:
            //     - Dynamic pressure:
            //         q_dyn = 0.5 * rho * V_mag^2
            //
            //     - Segment drag force magnitude:
            //         D_seg = q_dyn * S_per_seg * cd
            //
            //     - Segment lift force magnitude:
            //         L_seg = q_dyn * S_per_seg * cl
            //
            //   Notes:
            //     - Drag acts along the direction of the relative wind, opposite to vel_body.
            //     - Lift acts perpendicular to the local relative wind in the lift plane;
            //       its exact direction will be constructed later using unit vectors.
            // ----------------------------------------------------------------------------------------
            // Magnitude of local velocity in the marker body frame [m/s]
            const double V_mag_sq = vel_body.Length2();

            // Dynamic pressure at this segment [N/m^2]
            const double q_dyn = 0.5 * surface.air_density * V_mag_sq;

            surface.surf_aero_drag[i] = q_dyn * S_per_seg * surface.CD[i];  // Drag force magnitude
            surface.surf_aero_lift[i] = q_dyn * S_per_seg * surface.CL[i];  // Lift force magnitude

            // ----------------------------------------------------------------------------------------
            // STEP 3.11 – Apply the forces due to aerodynamic interaction
            // ----------------------------------------------------------------------------------------
            // Apply the lift and drag forces to the body.
            surface.aero_surf_drag_forces[i]->SetMforce(surface.surf_aero_drag[i]);
            surface.aero_surf_lift_forces[i]->SetMforce(surface.surf_aero_lift[i]);
        
        }

    }

}

}   // namespace _uav_

}   // namespace _acsl_

#endif  // SIM_UAV_TPP_