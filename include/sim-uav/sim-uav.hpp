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
 * File:        sim-uav.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the uav baseclass
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_UAV_HPP_
#define SIM_UAV_HPP_

// System includes
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <array>
#include <variant>

// ACSL physics sim includes
#include "sim-helpers.hpp"
#include "fkYAML/node.hpp"
#include "Eigen/Dense"
#include "sim-templated-helpers.hpp"
#include "sim-aerofoil.hpp"

// Chrono includes
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChBodyEasy.h"



namespace _acsl_
{
    
namespace _uav_
{

// =====================================================================================================================
// Special type definitions
// =====================================================================================================================
using CollisionShapeFrame = std::tuple<std::shared_ptr<chrono::ChCollisionShape>, chrono::ChFrame<>>;

// ----------------------------------------------------------------------------
// Namespace: _acsl_::_uav_::_prop_color_
//
// Purpose:
//   Defines a collection of compile-time UAV color constants for use in
//   visualization and simulation. Each color is stored as a chrono::ChColor
//   instance with specified RGB values, making assignment readable and robust.
//
// Usage:
//   - Access colors directly (e.g. _acsl_::_uav_::_prop_color_::RED)
//   - Assign to visualization or property structs.
//   - Includes a NULLCOLOR for "unset" or transparent/default cases.
//
// Members:
//   NULLCOLOR - Special "null" color to represent an unset or default state (white RGB).
//   RED       - Red    (RGB: 1.0, 0.0, 0.0)
//   GREEN     - Green  (RGB: 0.0, 1.0, 0.0)
//   BLUE      - Blue   (RGB: 0.0, 0.0, 1.0)
//   YELLOW    - Yellow (RGB: 1.0, 1.0, 0.0)
//   BLACK     - Black  (RGB: 0.0, 0.0, 0.0)
//   ORANGE    - Orange (RGB: 1.0, 0.5, 0.0)
//   PURPLE    - Purple (RGB: 1.0, 0.0, 1.0)
//   GREY      - Grey   (RGB: 0.5, 0.5, 0.5)
// ----------------------------------------------------------------------------
namespace _prop_color_ {
    inline const chrono::ChColor NULLCOLOR {1.0f, 1.0f, 1.0f}; // Unset/null color
    inline const chrono::ChColor RED       {1.0f, 0.0f, 0.0f};
    inline const chrono::ChColor GREEN     {0.0f, 1.0f, 0.0f};
    inline const chrono::ChColor BLUE      {0.0f, 0.0f, 1.0f};
    inline const chrono::ChColor YELLOW    {1.0f, 1.0f, 0.0f};
    inline const chrono::ChColor BLACK     {0.0f, 0.0f, 0.0f};
    inline const chrono::ChColor ORANGE    {1.0f, 0.5f, 0.0f};
    inline const chrono::ChColor PURPLE    {1.0f, 0.0f, 1.0f};
    inline const chrono::ChColor GREY      {0.1f, 0.1f, 0.1f};
} // namespace _prop_color_

// ----------------------------------------------------------------------------
// Namespace: _acsl_::_uav_::_motor_dir_
//
// Purpose:
//   Defines integer constants for motor rotation direction, for use in
//   UAV modeling and simulation. Positive (+1) indicates counterclockwise (CCW),
//   negative (-1) indicates clockwise (CW).
//
// Members:
//   CCW - Counterclockwise rotation (value: +1)
//   CW  - Clockwise rotation (value: -1)
// ----------------------------------------------------------------------------
namespace _motor_dir_ {
    inline const int CCW = +1; // Counterclockwise
    inline const int CW  = -1; // Clockwise
} // namespace _motor_dir_


// =====================================================================================================================
// Shared data structures
// =====================================================================================================================

// ------------------------------------------------------------------------------------------------------------
// Structure: chassisstruct
//
// Purpose:
//   Defines the rigid body representation and physical properties of the UAV's 
//   main chassis.
//
// Members:
//   body                      - Shared pointer to the Chrono body (AuxRef) for the chassis.
//   init_pos                  - Initial position of the chassis in Chrono coordinates.
//   init_rot                  - Initial orientation of the chassis (quaternion).
//   mass                      - Mass of the chassis body.
//   InertiaXX                 - Principal moments of inertia about the chassis axes.
//   InertiaXY                 - Products of inertia of the chassis body.
//   COM                       - Center of mass reference frame (offset from body origin).
//   vis_obj_name              - Filename of the visualization mesh (.obj).
//   collision                 - List of collision shapes with their associated frames.
//   biplane_frame             - Marker frame for the biplane reference (is always on).
//   com_thrust                - Thrust force applied to the center of mass.
//   com_Mx                    - Moment about the NED x axis applied to the center of mass.
//   com_My                    - Moment about the NED y axis applied to the center of mass.
//   com_Mz                    - Moment about the NED z axis applied to the center of mass.
// ------------------------------------------------------------------------------------------------------------
struct chassisstruct {
    std::shared_ptr<chrono::ChBodyAuxRef> body;
    chrono::ChVector3d init_pos;
    chrono::ChQuaternion<> init_rot;
    double mass;
    chrono::ChVector3d InertiaXX;
    chrono::ChVector3d InertiaXY;
    chrono::ChFramed COM;
    std::string vis_obj_name;
    std::vector<_acsl_::_uav_::CollisionShapeFrame> collision;
    std::shared_ptr<chrono::ChMarker> biplane_frame; 
    std::shared_ptr<chrono::ChForce> com_thrust;
    std::shared_ptr<chrono::ChForce> com_Mx;
    std::shared_ptr<chrono::ChForce> com_My;
    std::shared_ptr<chrono::ChForce> com_Mz;
};


// ------------------------------------------------------------------------------------------------------------
// Structure: propstruct
//
// Purpose:
//   Stores the physical, visualization, and collision properties for a single
//   propeller hub of the UAV. Includes a Chrono body (AuxRef), initial pose,
//   mass and inertia properties, reference frame, visualization filename, 
//   collision shapes, rendering color, opacity, and rotation direction.
//
// Members:
//   body          - Shared pointer to Chrono body (AuxRef) for the propeller.
//   init_pos      - Initial position of the propeller (Chrono coordinates).
//   init_rot      - Initial orientation of the propeller (quaternion).
//   mass          - Mass of the propeller body (kg).
//   InertiaXX     - Principal moments of inertia about axes.
//   InertiaXY     - Products of inertia (off-diagonal).
//   COM           - Center of mass reference frame.
//   vis_obj_name  - Filename for visualization mesh (.obj).
//   collision     - List of collision shapes and associated frames.
//   color         - Chrono RGB color value (default: NULLCOLOR for unset).
//   opacity       - Opacity value (default: 1.0 for fully opaque; 
//                                           0.0 is transparent/null).
// ------------------------------------------------------------------------------------------------------------
struct propstruct {
    std::shared_ptr<chrono::ChBodyAuxRef> body;
    chrono::ChVector3d init_pos;
    chrono::ChQuaternion<> init_rot;
    double mass;
    chrono::ChVector3d InertiaXX;
    chrono::ChVector3d InertiaXY;
    chrono::ChFramed COM;
    std::string vis_obj_name;
    std::vector<CollisionShapeFrame> collision;
    chrono::ChColor color = _prop_color_::NULLCOLOR;      // Default: unset/null
    float opacity = 1.0f;                                 // Default: opaque
};


// ------------------------------------------------------------------------------------------------------------
// Structure: motorstruct
//
// Purpose:
//   Stores all the data associated with the actuators for the UAV.
//
// Members:
//   motor         - Shared pointer to Chrono body (ChLinMotorRotationSppeed) 
//                   for the motor.
//   speed         - Shared pointer to Chrono function (ChFunctionConst) 
//                   for the motor.
//   thrust        - Shared pointer to Chrono Thrust (ChForce) for the motor.
//   torque        - Shared pointer to Chrono Torque (ChForce) for the motor.
//   frame         - Frame / location of the motor. Needs the marker location
//                   from the CAD model export.
//   rotation_dir  - Propeller rotation direction integer (+1 = CCW, -1 = CW;
//                   use values from _acsl_::_uav_::_rotation_dir_).
//   norm2newt     - Polynomial vector to compute the thrust in N from a
//                   normalized thrust [0-1] value.
//   newt2norm     - Polynomial vector to compute the normalized thrust [0-1]
//                   from a thrust in N value.
//   norm2rps      - Polynomial vector to compute the rad/s speed of the rotor
//                   from a normalized thrust [0-1] value.
//   ct            - Motor torque coefficient for computing the backtorque.
//   rps_scaler    - scaling coefficient for the rps value, helps with the
//                   irrlicht visualization system. A fast rotation object
//                   does not render well.
// ------------------------------------------------------------------------------------------------------------
struct motorstruct {
    std::shared_ptr<chrono::ChLinkMotorRotationSpeed> motor;
    std::shared_ptr<chrono::ChFunctionConst> speed;
    std::shared_ptr<chrono::ChForce> thrust;
    std::shared_ptr<chrono::ChForce> torque;
    chrono::ChFramed frame;                              
    int spin_dir = _motor_dir_::CW;                      // Default: clockwise
    Eigen::VectorXd norm2newt;
    Eigen::VectorXd newt2norm;
    Eigen::VectorXd norm2rps;
    double ct;
    double rps_scaler = 0.01;                            // Default: 0.01 scaled
};

// ------------------------------------------------------------------------------------------------------------
// Structure: chassisaerodynamicstruct
//
// Purpose:
//   Stores all the data associated with the chassis aerodynamic properties.
//
// Members:
//   air_density               - Density of the air you want to set
//   chassis_drag_frame        - Marker frame for applying drag forces on the chassis. (NED Convention)
//   chassis_drag_coefficients - Coefficients for computing the aerodynamic drag force on the chassis.
//   chassis_drag_force        - Chrono drag force vector computed for the chassis.
// ------------------------------------------------------------------------------------------------------------
struct chassisaerodynamicstruct {

    // Common aerodynamic parameters
    double air_density = std::numeric_limits<double>::quiet_NaN();

    // Parameters for the chassis drag force calculation
    double chassis_drag_coefficient = std::numeric_limits<double>::quiet_NaN();
    double chassis_body_surface_aera = std::numeric_limits<double>::quiet_NaN();

    // Variables to store the chassis drag force
    std::shared_ptr<chrono::ChMarker> chassis_drag_frame;   // For visualization
    chrono::ChVector3d chassis_drag_force;                  // For computation
    std::shared_ptr<chrono::ChForce> chassis_drag_force_x;  // For application in X direction
    std::shared_ptr<chrono::ChForce> chassis_drag_force_y;  // For application in Y direction
    std::shared_ptr<chrono::ChForce> chassis_drag_force_z;  // For application in Z direction
};

// ------------------------------------------------------------------------------------------------------------
// Structure: aerodynamicsurfacestruct
//
// Purpose:
//   Stores all the data associated with the aerodynamic surfaces for the UAV other than the chassis.
//
// Members:
//   air_density                  - Density of the air you want to set
//   name                         - Name of the aerodynamic surface (e.g., "left_wing", "right_wing").   
//   aerofoil_span                - Span of the aerofoil (wing) for lift and drag calculations.
//   aerofoil_chord               - Chord length of the aerofoil (wing) for lift and drag calculations.
//   num_of_aero_centers          - Number of aerodynamic centers per wing (for distributed aerodynamics).
//   p1                           - Point 1 for the quarter-chord line of the wing.
//   p2                           - Point 2 for the quarter-chord line of the wing.
//   aerodynamic_center_frames    - List of marker frames for applying aerodynamic forces (NED Convention).
//   alpha                        - Angle of attack for each aerodynamic center (for lift and drag calculations).
//   CL                           - Lift coefficient for each aerodynamic center.
//   CD                           - Drag coefficient for each aerodynamic center.
//   wing_aero_drag               - Computed drag force for each aerodynamic center.
//   wing_aero_lift               - Computed lift force for each aerodynamic center.
//   wing_aero_drag_forces        - Chrono force objects for applying drag forces.
//   wing_aero_lift_forces        - Chrono force objects for applying lift forces.
//   q_relative                   - Orientation of the wing's aerodynamic center frame relative to the NED frame.
//   lift_force_relative_vector   - Direction vector for lift force relative in the chassis NED frame.
//   drag_force_relative_vector   - Direction vector for drag force relative in the chassis NED frame.
// ------------------------------------------------------------------------------------------------------------
struct aerodynamicsurfacestruct {  

    // Common aerodynamic parameters
    double air_density = std::numeric_limits<double>::quiet_NaN();

    // Name of the aerodynamic surface (e.g., "left_wing", "right_wing").
    std::string name;

    // Parameters for the aerofoil lift and drag force calculation
    double aerofoil_span = std::numeric_limits<double>::quiet_NaN();
    double aerofoil_chord = std::numeric_limits<double>::quiet_NaN();
 
    // Variables to store the wing aerodynamic forces
    int num_of_aero_centers = 0;                                                // Number of aerodynamic centers per wing (for distributed aerodynamics)
    chrono::ChVector3d p1;                                                     // Point 1 for the quarter-chord line
    chrono::ChVector3d p2;                                                     // Point 2 for the quarter-chord line
    
    std::vector<std::shared_ptr<chrono::ChMarker>> aerodynamic_center_frames;  // For visualization
    std::vector<double> alpha;                                                 // For computation
    std::vector<double> CL;                                                    // For computation
    std::vector<double> CD;                                                    // For computation
    std::vector<double> surf_aero_drag;                                        // For computation
    std::vector<double> surf_aero_lift;                                        // For computation
    std::vector<std::shared_ptr<chrono::ChForce>> aero_surf_drag_forces;       // For application of drag in -ve x direction (NED)
    std::vector<std::shared_ptr<chrono::ChForce>> aero_surf_lift_forces;       // For application of lift in -ve z direction (NED)
    chrono::ChQuaterniond q_relative;                                          // For setting up the wing's aerodynamic center frame orientation wrt the NED frame
    chrono::ChVector3d lift_force_relative_vector;                             // For setting up the direction in which the lift force acts wrt to the body NED frame
    chrono::ChVector3d drag_force_relative_vector;                             // For setting up the direction in which the drag force acts wrt to the body NED frame

};  

// ------------------------------------------------------------------------------------------------------------
// Structure: m_states
//
// Purpose:
//   Stores the UAV's runtime state data relative to the NED frame, including
//   translation, rotation, and kinematic properties.
//
// Members:
//   time    - Simulation time stamp.
//   pos     - Position in NED coordinates.
//   vel     - Linear velocity in NED coordinates.
//   acc     - Linear acceleration in NED coordinates.
//   eul     - Euler angles (rotation) in NED coordinates.
//   eul_bp  - Euler angles (rotation) in biplane frame (relative to NED).
//   quat    - Orientation quaternion in NED coordinates.
//   quat_bp - Orientation quaternion in biplane frame (relative to NED).
//   ovel    - Angular velocity in body frame (relative to NED).
//   oacc    - Angular acceleration in body frame (relative to NED).
//   muI     - Forces acting on the body in the inertial frame (NED).
//   muJ     - Forces acting on the body in the body frame (NED).
//   tauJ    - Torque acting on the body in the body frame (NED).
// ------------------------------------------------------------------------------------------------------------
struct m_states {
    double time;
    chrono::ChVector3d pos;
    chrono::ChVector3d vel;
    chrono::ChVector3d acc;
    chrono::ChVector3d eul;
    chrono::ChVector3d eul_bp;
    chrono::ChQuaterniond quat;
    chrono::ChQuaterniond quat_bp;
    chrono::ChVector3d ovel;
    chrono::ChVector3d ovel_bp;
    chrono::ChVector3d oacc;
    chrono::ChVector3d oacc_bp;
    chrono::ChVector3d muI;
    chrono::ChVector3d muJ;
    chrono::ChVector3d tauJ;

    /**
     * @brief Serializes the state data to a CSV-compatible output stream.
     *        Fields are written as a single CSV line in the order defined above.
     * @param oss Output stringstream to write the CSV line to.
     */
    void SerializeStateData(std::ostringstream& oss) const
    {
        oss << time << ", "
            << pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
            << vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
            << acc.x() << ", " << acc.y() << ", " << acc.z() << ", "
            << eul.x() << ", " << eul.y() << ", " << eul.z() << ", "
            << eul_bp.x() << ", " << eul_bp.y() << ", " << eul_bp.z() << ", "
            << quat.e0() << ", " << quat.e1() << ", " << quat.e2() << ", " << quat.e3() << ", "
            << quat_bp.e0() << ", " << quat_bp.e1() << ", " << quat_bp.e2() << ", " << quat_bp.e3() << ", "
            << ovel.x() << ", " << ovel.y() << ", " << ovel.z() << ", "
            << ovel_bp.x() << ", " << ovel_bp.y() << ", " << ovel_bp.z() << ", "
            << oacc.x() << ", " << oacc.y() << ", " << oacc.z() << ", "
            << oacc_bp.x() << ", " << oacc_bp.y() << ", " << oacc_bp.z() << ", "
            << muI.x() << ", " << muI.y() << ", " << muI.z() << ", "
            << muJ.x() << ", " << muJ.y() << ", " << muJ.z() << ", "
            << tauJ.x() << ", " << tauJ.y() << ", " << tauJ.z() << ", ";
    }
};


// ------------------------------------------------------------------------------------------------------------
// Enum: LinkType
//
// Purpose:
//   Enumerates the supported types of mate constraints (Chrono links) between
//   UAV bodies for modular link property specification and initialization.
//
// Values:
//   Parallel   - Represents a ChLinkMateParallel constraint (parallel axis).
//   Generic    - Represents a ChLinkMateGeneric constraint (custom DOF lock).
//   DistanceZ  - Represents a ChLinkMateDistanceZ constraint (Z-axis distance).
//   Orthogonal - Represents a ChLinkMateOrthogonal constriaint (edge to edge).
// ------------------------------------------------------------------------------------------------------------
enum class LinkType { Parallel, Generic, DistanceZ, Orthogonal };


// ------------------------------------------------------------------------------------------------------------
// Template: LinkProperty<LT>
//
// Purpose:
//   Primary template declaration for link property storage structs, specialized
//   for each supported LinkType. This is a forward declaration; use the
//   specializations below for valid property types.
//
// Notes:
//   Each specialization holds only the properties required to fully
//   specify and later create a link of that type.
// ------------------------------------------------------------------------------------------------------------
template<LinkType LT>
struct LinkProperty;


// ------------------------------------------------------------------------------------------------------------
// Structure: LinkProperty<LinkType::Parallel>
//
// Purpose:
//   Stores all properties required for a ChLinkMateParallel constraint between
//   two Chrono bodies (enforcing parallel axes, with optional flipping).
//
// Members:
//   flipped - Whether to flip the direction vector for the parallel constraint.
//   name    - Unique or descriptive link name for identification/debugging.
//   bodyA   - Shared pointer to the first Chrono body (e.g., chassis).
//   bodyB   - Shared pointer to the second Chrono body (e.g., propeller).
//   cA      - Constraint position on bodyA (local frame).
//   cB      - Constraint position on bodyB (local frame).
//   dA      - Direction vector on bodyA for axis alignment.
//   dB      - Direction vector on bodyB for axis alignment.
// ------------------------------------------------------------------------------------------------------------
template<>
struct LinkProperty<LinkType::Parallel> {
    bool flipped;
    std::string name;
    std::shared_ptr<chrono::ChBodyAuxRef> bodyA, bodyB;
    chrono::ChVector3d cA, cB, dA, dB;
};


// ------------------------------------------------------------------------------------------------------------
// Structure: LinkProperty<LinkType::Generic>
//
// Purpose:
//   Stores all properties required for a ChLinkMateGeneric constraint, allowing
//   selective locking of translational and rotational degrees of freedom (DOF).
//
// Members:
//   name    - Unique or descriptive name for the link.
//   bodyA   - Shared pointer to Chrono body A.
//   bodyB   - Shared pointer to Chrono body B.
//   cA      - Constraint point on bodyA (local frame).
//   cB      - Constraint point on bodyB (local frame).
//   dA      - Direction vector on bodyA for orientation.
//   dB      - Direction vector on bodyB for orientation.
//   cx,cy,cz - If true, locks translation along X, Y, and Z, respectively.
//   rx,ry,rz - If true, locks rotation about X, Y, and Z axes, respectively.
// ------------------------------------------------------------------------------------------------------------
template<>
struct LinkProperty<LinkType::Generic> {
    std::string name;
    std::shared_ptr<chrono::ChBodyAuxRef> bodyA, bodyB;
    chrono::ChVector3d cA, cB, dA, dB;
    bool cx, cy, cz, rx, ry, rz;
};


// ------------------------------------------------------------------------------------------------------------
// Structure: LinkProperty<LinkType::DistanceZ>
//
// Purpose:
//   Stores all properties required for a ChLinkMateDistanceZ constraint,
//   which constrains the Z-axis distance between two points on two bodies.
//
// Members:
//   name     - Descriptive or unique name for the link.
//   bodyA    - Shared pointer to body A.
//   bodyB    - Shared pointer to body B.
//   cA       - Constraint point on bodyA (local frame).
//   cB       - Constraint point on bodyB (local frame).
//   dB       - Direction vector on bodyB (should point along constrained axis).
//   distance - The signed Z distance value; 0 for coincident constraints.
// ------------------------------------------------------------------------------------------------------------
template<>
struct LinkProperty<LinkType::DistanceZ> {
    std::string name;
    std::shared_ptr<chrono::ChBodyAuxRef> bodyA, bodyB;
    chrono::ChVector3d cA, cB, dB;
    double distance;
};

// ------------------------------------------------------------------------------------------------------------
// Structure: LinkProperty<LinkType::Orthogonal>
//
// Purpose:
//   Stores all properties required for a ChLinkMateOrthogonal constraint,
//   which constrains the edge to edge mate of two bodies.
//
// Members:
//   name     - Descriptive or unique name for the link.
//   bodyA    - Shared pointer to body A.
//   bodyB    - Shared pointer to body B.
//   cA       - Constraint point on bodyA (local frame).
//   cB       - Constraint point on bodyB (local frame).
//   dA       - Direction vector on bodyA for orientation.
//   dB       - Direction vector on bodyB for orientation. 
// ------------------------------------------------------------------------------------------------------------
template<>
struct LinkProperty<LinkType::Orthogonal> {
    std::string name;
    std::shared_ptr<chrono::ChBodyAuxRef> bodyA, bodyB;
    chrono::ChVector3d cA, cB, dA, dB;
};


// ------------------------------------------------------------------------------------------------------------
// Type Alias: LinkData
//
// Purpose:
//   Variadic property type that can hold any specialization of LinkProperty,
//   letting you store properties for any supported type of UAV link constraint
//   in a single container (such as std::vector<LinkData>).
//
// Usage:
//   - Use LinkData as the type for vectors or lists to store mixed link types.
//   - Retrieve and process the specific type using std::visit, std::get, or std::holds_alternative.
//
// Contains:
//   LinkProperty<LinkType::Parallel>   — Properties for parallel constraints.
//   LinkProperty<LinkType::Generic>    — Properties for generic constraints.
//   LinkProperty<LinkType::DistanceZ>  — Properties for Z distance constraints.
// ------------------------------------------------------------------------------------------------------------
using LinkData = std::variant<
    LinkProperty<LinkType::Parallel>,
    LinkProperty<LinkType::Generic>,
    LinkProperty<LinkType::DistanceZ>,
    LinkProperty<LinkType::Orthogonal>
>;


// =====================================================================================================================
// Non-templated polymorphic base interface
//
// Purpose:
//   Defines the full UAV API contract so that any platform type (concrete UAV
//   class) can be managed through a simuavbase pointer.
//
// Structure:
//   - Public section  : General UAV API accessible externally.
//   - Protected section: Setup/configuration API intended for derived classes.
//
// Notes:
//   - All methods are pure virtual (= 0), forcing implementation in derived types.
//   - This allows simbridge or other managers to store and manage UAVs
//     polymorphically.
// =====================================================================================================================
class simuavbase {
public:
    virtual ~simuavbase() = default;

    // ---------------- Public API ----------------

    // Set the human-readable platform name.
    virtual void SetUAVPlatformName(std::string name) = 0;

    // Retrieve the currently assigned UAV platform name.
    virtual std::string GetUAVPlatformName() const = 0;

    // Retrieve the UAV physics system
    virtual chrono::ChSystemNSC& getPhysicsSystem() = 0;

    // Get the file system path string for the UAV's shapes directory.
    virtual std::string GetUAVShapesDir() const = 0;

    // Get number of UAV propellers.
    virtual int GetPropCount() = 0;

    // Return the inertial NED frame as a Chrono ChFrame object.
    virtual chrono::ChFrame<> GetInertialNEDFrame() = 0;

    // Get the inertial NED frame auxiliary body pointer.
    virtual std::shared_ptr<chrono::ChBodyAuxRef> GetInertialNEDFrameAuxBody() = 0;

    // Access the chassisstruct for modifying chassis parameters directly.
    virtual chassisstruct& GetUAVChassis() = 0;

    // Access the chassisaerodynamicstruct for modifying aerodynamic parameters directly.
    virtual chassisaerodynamicstruct& GetUAVChassisAerodynamics() = 0;

    // Access the aerodynamicsurfacestruct for modifying the aerodynamics parameters directly by name.
    virtual aerodynamicsurfacestruct& GetUAVAerodynamicSurface(std::string name) = 0;

    // Access the aerodynamic surface struct vector for verification purposes and logging only
    virtual std::vector<aerodynamicsurfacestruct>& GetUAVAerodynamicSurfaces() = 0;

    // Access the propstruct for modifying prop parameters directly.
    virtual propstruct& GetUAVProp(size_t idx) = 0;

    // Acess the motorstruct for modifying motor parameters directly.
    virtual motorstruct& GetUAVMotor(size_t idx) = 0;

    // Return all Chrono body pointers belonging to the UAV.
    virtual std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& GetUAVBodyList() = 0;

    // Return all Chrono link/joint pointers belonging to the UAV.
    virtual std::vector<std::shared_ptr<chrono::ChLinkBase>>& GetUAVLinkList() = 0;

    // Finalize and add all UAV components into the Chrono physics system.
    virtual void AddUAVToSystem() = 0;

    // Retrieve the UAV's full state data in the NED frame.
    virtual m_states GetUAVStateData() = 0;

    // Retrieve the Propeller's full state data in the NED frame.
    virtual m_states GetUAVPropStateData(size_t idx) = 0;

    // Function to set the thrust setpoints of the UAV
    virtual void SetActuator(size_t idx, double thrust, double torque, double rpm) = 0;

    // Function to set the normalized thrust setpoint for the UAV
    virtual void SetThrustSetPoint(size_t idx, double thrustSP) = 0;

    // Function to set the thrust, mx, my, mz at the com of the UAV
    virtual void SetControlInputAtCOM(double u1, double u2, double u3, double u4) = 0;

    // Function to compute and apply the aerodynamic chassis drag force
    virtual void SetChassisDrag() = 0;

    // Function to compute and apply the aerodynamic forces for an arbitrary aerodynamic surface
    virtual void SetUAVAeroSurfaceLiftDrag() = 0;

protected:
    // ---------------- Protected API ----------------

    // Set the path to the UAV's shapes directory.
    virtual void SetUAVShapesDir(std::string dir) = 0;

    // Create and set up the inertial NED frame in the simulation.
    virtual void SetupInertialNEDFrame() = 0;

    // Configure the chassis initial position.
    virtual void ConfigureUAVChassisInitPos(chrono::ChVector3d pos) = 0;

    // Configure the chassis initial orientation quaternion.
    virtual void ConfigureUAVChassisInitRot(chrono::ChQuaternion<> rot) = 0;

    // Set the chassis mass (kg).
    virtual void ConfigureUAVChassisMass(double mass) = 0;

    // Set principal moments of inertia (Ixx, Iyy, Izz).
    virtual void ConfigureUAVChassisInertiaXX(chrono::ChVector3d IXX) = 0;

    // Set products of inertia (Ixy, Ixz, Iyz).
    virtual void ConfigureUAVChassisInertiaXY(chrono::ChVector3d IXY) = 0;

    // Set center-of-mass frame relative to chassis reference.
    virtual void ConfigureUAVChassisCOM(chrono::ChFrame<> COM) = 0;

    // Set chassis visualization mesh filename.
    virtual void ConfigureUAVChassisOBJName(std::string name) = 0;

    // Set chassis collision shapes
    virtual void ConfigureUAVChassisCollisionShapes(
    const std::vector<_acsl_::_uav_::CollisionShapeFrame>& list) = 0;

    // Create, initialize, and register chassis body in internal body list.
    virtual void InitiateUAVChassis() = 0;

    // Check the propeller request (starts at 1 and not 0)
    virtual void CheckUAVPropRequest(size_t idx) = 0;

    // Configure the propeller initial position.
    virtual void ConfigureUAVPropInitPos(size_t idx, chrono::ChVector3d pos) = 0;

    // Configure the prop initial orientation quaternion.
    virtual void ConfigureUAVPropInitRot(size_t idx, chrono::ChQuaternion<> rot) = 0;

    // Set the propeller mass (kg).
    virtual void ConfigureUAVPropMass(size_t idx, double mass) = 0;

    // Set principal moments of inertia (Ixx, Iyy, Izz).override
    virtual void ConfigureUAVPropInertiaXX(size_t idx, chrono::ChVector3d IXX) = 0;

    // Set products of inertia (Ixy, Ixz, Iyz).
    virtual void ConfigureUAVPropInertiaXY(size_t idx, chrono::ChVector3d IXY) = 0;

    // Set center-of-mass frame relative to prop reference.
    virtual void ConfigureUAVPropCOM(size_t idx, chrono::ChFrame<> COM) = 0;

    // Set prop visualization mesh filename.
    virtual void ConfigureUAVPropOBJName(size_t idx, std::string name) = 0;

    // Set prop collision shapes
    virtual void ConfigureUAVPropCollisionShapes(
    size_t idx, const std::vector<_acsl_::_uav_::CollisionShapeFrame>& list) = 0;

    // Set prop color based on the predefined enum values
    virtual void ConfigureUAVPropColor(size_t idx, chrono::ChColor color) = 0;

    // Set prop opacity based on the value passed in
    virtual void ConfigureUAVPropOpacity(size_t idx, float val) = 0;

    // Create, intialize, and register prop body in interal body list.
    virtual void InitiateUAVProps() = 0;

    // Set motor spin direction based on the value passed in 
    virtual void ConfigureUAVMotorSpinDir(size_t idx, int spin) = 0;

    // Set motor frame based on the value passed in
    virtual void ConfigureUAVMotorFrame(size_t idx, chrono::ChFrame<> frame) = 0;

    // Set the motor polynomial for converting from Normalized thrust value to Newtons
    virtual void ConfigureUAVMotorNorm2Newt(size_t idx, Eigen::VectorXd& poly) = 0;

    // Set the motor polynomial for converting from Newtons to Normalized thrust value
    virtual void ConfigureUAVMotorNewt2Norm(size_t idx, Eigen::VectorXd& poly) = 0;

    // Set the motor polynomial for converting from Normalized thrust value to RPS
    virtual void ConfigureUAVMotorNorm2RPS(size_t idx, Eigen::VectorXd& poly) = 0;

    // Set the motor torque constant 
    virtual void ConfigureUAVMotorCt(size_t idx, double ct) = 0;

    // Set the motor rps scalar constant
    virtual void ConfigureUAVMotorRPSScaler(size_t idx, double rps_scale) = 0;

    // Create, initialize, and register motor in the internal list
    virtual void InitiateUAVMotors() = 0;

    // Create, inialize, and register all the links present in the drone.
    virtual void LinkUAVBodies(const std::vector<LinkData>& link_data_vec) = 0;

    // Set the chassis drag coefficient
    virtual void ConfigureUAVChassisDragCoefficient(double coeff) = 0;

    // Set the wing aerodynamic centers based on the aerodynamic surfaces data passed in
    virtual void ConfigureUAVAeroSurfaces(std::vector<aerodynamicsurfacestruct>& aero_surfaces) = 0;
};


// =====================================================================================================================
// Templated implementation of full API
//
// Purpose:
//   Provides a complete base UAV implementation that is parameterized by the
//   number of propellers (nop) and implements all functions from simuavbase.
//   Also inherits the simairfoil class with the aerofoil types. The type will
//   need to be set at instantiation of this class.  
// 
//
// Notes:
//   - Implements both high-level control API (e.g., AddUAVToSystem) and
//     lower-level setup/configuration API from the base.
//   - Designed to be extended by platform-specific UAV classes that call
//     protected setup methods in their constructors.
//   - Inherits simairfoil to expose CL/CD/CM(alpha) based on a
//     selected AirFoilType, without fixing the type here.
// =====================================================================================================================
template <int nop>
class simuav : public simuavbase, public ::_acsl_::_uav_::_aerofoil_::simairfoil {
public:
    // Constructor: stores a reference to the Chrono simulation system.
    simuav(chrono::ChSystemNSC& sys, // <-- The base class constructor need system, Set the wing data to none by default.
           ::_acsl_::_uav_::_aerofoil_::AirFoilType type = ::_acsl_::_uav_::_aerofoil_::AirFoilType::NONE) 
           : m_physics_(sys), simairfoil(type) { }

    // ---------------- Public API overrides ----------------

    void SetUAVPlatformName(std::string name) override { this->name_ = name; }
    std::string GetUAVPlatformName() const override { return name_; }
    chrono::ChSystemNSC& getPhysicsSystem() { return this->m_physics_; }
    std::string GetUAVShapesDir() const override { return shapes_dir; }
    int GetPropCount() override { return nop; }

    chrono::ChFrame<> GetInertialNEDFrame() override;
    std::shared_ptr<chrono::ChBodyAuxRef> GetInertialNEDFrameAuxBody() override { return InertialFrameNED; }
    
    chassisstruct& GetUAVChassis() override { return chassis; }
    propstruct& GetUAVProp(size_t idx) override;
    motorstruct& GetUAVMotor(size_t idx) override;
    chassisaerodynamicstruct& GetUAVChassisAerodynamics() override { return chassisaero; };
    aerodynamicsurfacestruct& GetUAVAerodynamicSurface(std::string name) override;
    std::vector<aerodynamicsurfacestruct>& GetUAVAerodynamicSurfaces() override { return aerosurfaces; }


    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& GetUAVBodyList() override { return bodylist; }
    std::vector<std::shared_ptr<chrono::ChLinkBase>>& GetUAVLinkList() override { return linklist; }
    
    void AddUAVToSystem() override;

    m_states GetUAVStateData() override;
    m_states GetUAVPropStateData(size_t idx) override;

    void SetActuator(size_t idx, double thrust, double torque, double rpm) override;

    void SetThrustSetPoint(size_t idx, double thrustSP) override;

    void SetControlInputAtCOM(double u1, double u2, double u3, double u4) override;

    void SetChassisDrag() override;

    void SetUAVAeroSurfaceLiftDrag() override;

protected:
    // ---------------- Protected API overrides ----------------

    void SetUAVShapesDir(std::string dir) override { shapes_dir = dir; }
    void SetupInertialNEDFrame() override;

    void ConfigureUAVChassisInitPos(chrono::ChVector3d pos) override { 
         chassis.init_pos = _shared_::_transformations_::GetChronoPosFromNED(pos); }
    void ConfigureUAVChassisInitRot(chrono::ChQuaternion<> rot) override { 
         chassis.init_rot = _shared_::_transformations_::GetChronoOrientaitonFromNED(rot); }

    void ConfigureUAVChassisMass(double mass) override { chassis.mass = mass; }
    void ConfigureUAVChassisInertiaXX(chrono::ChVector3d IXX) override { chassis.InertiaXX = IXX; }
    void ConfigureUAVChassisInertiaXY(chrono::ChVector3d IXY) override { chassis.InertiaXY = IXY; }
    void ConfigureUAVChassisCOM(chrono::ChFrame<> COM) override { chassis.COM = COM; }
    void ConfigureUAVChassisOBJName(std::string name) override { chassis.vis_obj_name = name; }
    
    void ConfigureUAVChassisCollisionShapes
    (const std::vector<_acsl_::_uav_::CollisionShapeFrame>& list) override
    {   chassis.collision = list;   }

    void InitiateUAVChassis() override;

    void CheckUAVPropRequest(size_t idx) override;
    
	void ConfigureUAVPropInitPos(size_t idx, chrono::ChVector3d pos) override;
    void ConfigureUAVPropInitRot(size_t idx, chrono::ChQuaternion<> rot) override;

    void ConfigureUAVPropMass(size_t idx, double mass) override;
    void ConfigureUAVPropInertiaXX(size_t idx, chrono::ChVector3d IXX) override;
    void ConfigureUAVPropInertiaXY(size_t idx, chrono::ChVector3d IXY) override;
    void ConfigureUAVPropCOM(size_t idx, chrono::ChFrame<> COM) override;
    void ConfigureUAVPropOBJName(size_t idx, std::string name) override;
	
	void ConfigureUAVPropCollisionShapes
	(size_t idx, const std::vector<_acsl_::_uav_::CollisionShapeFrame>& list) override;

    void ConfigureUAVPropColor(size_t idx, chrono::ChColor color) override;
    void ConfigureUAVPropOpacity(size_t idx, float val) override;

    void InitiateUAVProps() override;

    void ConfigureUAVMotorSpinDir(size_t idx, int spin) override;
    void ConfigureUAVMotorFrame(size_t idx, chrono::ChFrame<> frame) override;

    
    void ConfigureUAVMotorNorm2Newt(size_t idx, Eigen::VectorXd& poly) override;
    void ConfigureUAVMotorNewt2Norm(size_t idx, Eigen::VectorXd& poly) override;
    void ConfigureUAVMotorNorm2RPS(size_t idx, Eigen::VectorXd& poly) override;
    void ConfigureUAVMotorCt(size_t idx, double ct) override;
    virtual void ConfigureUAVMotorRPSScaler(size_t idx, double rps_scale) override;

    void InitiateUAVMotors() override;

    void LinkUAVBodies(const std::vector<LinkData>& link_data_vec) override;

    void ConfigureUAVChassisDragCoefficient(double coeff) override { chassisaero.chassis_drag_coefficient = coeff; }

    void ConfigureUAVAeroSurfaces(std::vector<aerodynamicsurfacestruct>& aero_surfaces) override;

private:
    // ---------------- Internal state ----------------

    chrono::ChSystemNSC& m_physics_;                                // Reference to Chrono physics system
    std::string name_;                                              // UAV name
    std::string shapes_dir;                                         // Path to visual shape files (.obj)
    std::shared_ptr<chrono::ChBodyAuxRef> InertialFrameNED;         // Inertial NED frame chrono-body
    chassisstruct chassis;                                          // Chassis data
    chassisaerodynamicstruct chassisaero;                           // Vehicle aerodynamic for the chassis
    std::vector<aerodynamicsurfacestruct> aerosurfaces;             // Vehicle aerodynamic surfaces vector
    std::array<propstruct, nop> props;                              // Propeller data
    std::array<motorstruct, nop> motors;                            // Actuator data
    std::vector<LinkData> links;                                    // All the link data for the UAV
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;    // All body pointers for registration
    std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;      // All link pointers for registration
};

}   // namespace _uav_

}   // namespace _acsl_

// ---------------------------------------------------------------------------
// Implementation of template methods is in the separate .tpp file
// ---------------------------------------------------------------------------
#include "sim-uav.tpp"


#endif  // SIM_UAV_HPP_