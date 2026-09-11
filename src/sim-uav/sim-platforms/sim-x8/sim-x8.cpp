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
 * File:        sim-x8.cpp
 * Author:      Xavier Casanova
 * Date:        August 04, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived UAV class for the simulated x8.
 * 
 * GitHub:    https://github.com/xavier1cas/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#include "sim-x8.hpp"

namespace _acsl_
{

namespace _x8_
{

// Compulsary derived class function that initiates the chassis with all the necessary parameters
void simx8::ConfigureX8Chassis()
{
    // Create the struct for processing the uav chassis properties
    _uav_::chassisstruct chassis_properties;

    // Configure the initial position of the chassis ------------ -----------------------------------------------------
    chassis_properties.init_pos = chrono::ChVector3d(0,0,0);
    
    // Configure the initial rotation of the chasssis -----------------------------------------------------------------
    chassis_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);
    
    // Configure the mass of the chassis ------------------------------------------------------------------------------
    chassis_properties.mass = 1.52039110869982;

    // Configure the Chassis Inertia vectors --------------------------------------------------------------------------
    chassis_properties.InertiaXX = chrono::ChVector3d(0.0154869282101269,0.0136133467493814,0.0134064606478442);
    chassis_properties.InertiaXY = chrono::ChVector3d(-1.21648368653966e-05,0.000721380361655569,1.05235970977448e-05);

    // Configure the chassis COM along with it's auxilliary frame at the COM ------------------------------------------
    // This is the most important as we will consider this to be the pixhawk in our subsequent calculations
    chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(0.0118119639499877,-0.00303486723088082,0.0334178902325843),
                                               chrono::ChQuaternion<>(1,0,0,0)); 

    // Configure the name of the visualization obj file ---------------------------------------------------------------
    chassis_properties.vis_obj_name = "chassis.obj";

    // Configure the chassis collision shapes -------------------------------------------------------------------------
    // Cache some variables for this 
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();   // Material
    std::shared_ptr<chrono::ChCollisionShape> col;                          // Collision shape
    chrono::ChMatrix33<> mr;                                                // Rotation Matrix for the frame definition
    chrono::ChVector3d pos;                                                 // Position Vector for the frame definition
    chrono::ChVector3d dim;                                                 // Dimensions of the collison box 
    chrono::ChFrame<> frame;                                                // Frame of the collision shape

    // 1
    mr(0,0)=-1;                     mr(1,0)=0;                      mr(2,0)=7.02385995171017E-15;
    mr(0,1)=-1.48723864357112E-16;  mr(1,1)=-1;                     mr(2,1)=-1.48723864357112E-16;
    mr(0,2)=7.02385995171017E-15;   mr(1,2)=-1.48723864357113E-16;  mr(2,2)=1;
    pos = chrono::ChVector3d(0.00304557838099178,-0.00322141559125175,0.269749999999996);
    dim = chrono::ChVector3d(0.245,0.373249790618444,0.0385);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

    chassis_properties.collision.emplace_back(col, frame);

    // 2
    mr(0,0)=0;                      mr(1,0)=1;                      mr(2,0)=1.40930265789226E-16;
    mr(0,1)=-1;                     mr(1,1)=0;                      mr(2,1)=7.08961260536019E-15;
    mr(0,2)=7.08961260536019E-15;   mr(1,2)=-1.40930265789226E-16;  mr(2,2)=1;
    pos = chrono::ChVector3d(0.0100647803623046,-0.00351913709958371,-0.00337500000000418);
    dim = chrono::ChVector3d(0.246181821379837,0.231961596037371,0.09175);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos,mr);

    chassis_properties.collision.emplace_back(col, frame);

    // Call all the configuration helper functions --------------------------------------------------------------------
    ConfigureUAVChassisInitPos(chassis_properties.init_pos);
    ConfigureUAVChassisInitRot(chassis_properties.init_rot);
    ConfigureUAVChassisMass(chassis_properties.mass);
    ConfigureUAVChassisInertiaXX(chassis_properties.InertiaXX);
    ConfigureUAVChassisInertiaXY(chassis_properties.InertiaXY);                                               
    ConfigureUAVChassisCOM(chassis_properties.COM);
    ConfigureUAVChassisOBJName(chassis_properties.vis_obj_name);
    ConfigureUAVChassisCollisionShapes(chassis_properties.collision);

    // Initiate the UAV chassis ---------------------------------------------------------------------------------------
    InitiateUAVChassis();
}

// Compulsary derived class function that initiates the propellers with all the necessary parameters
void simx8::ConfigureX8Props()
{
    // Create the struct for processing the uav propeller properties
    _uav_::propstruct prop_properties;

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0251400000000041);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00364120675790278;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "prop.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::RED;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(1, prop_properties.init_pos);
    ConfigureUAVPropInitRot(1, prop_properties.init_rot);
    ConfigureUAVPropMass(1, prop_properties.mass);
    ConfigureUAVPropInertiaXX(1, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(1, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(1, prop_properties.COM);
    ConfigureUAVPropOBJName(1, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(1, prop_properties.color);
    ConfigureUAVPropOpacity(1, prop_properties.opacity);
    
    // ------------------------------------------------ 2 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0251400000000041);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00364120675790278;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "prop.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
        prop_properties.color = _uav_::_prop_color_::ORANGE;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(2, prop_properties.init_pos);
    ConfigureUAVPropInitRot(2, prop_properties.init_rot);
    ConfigureUAVPropMass(2, prop_properties.mass);
    ConfigureUAVPropInertiaXX(2, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(2, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(2, prop_properties.COM);
    ConfigureUAVPropOBJName(2, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(2, prop_properties.color);
    ConfigureUAVPropOpacity(2, prop_properties.opacity);

    // ------------------------------------------------ 3 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0251400000000059);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "prop.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::RED;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(3, prop_properties.init_pos);
    ConfigureUAVPropInitRot(3, prop_properties.init_rot);
    ConfigureUAVPropMass(3, prop_properties.mass);
    ConfigureUAVPropInertiaXX(3, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(3, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(3, prop_properties.COM);
    ConfigureUAVPropOBJName(3, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(3, prop_properties.color);
    ConfigureUAVPropOpacity(3, prop_properties.opacity);

    // ------------------------------------------------ 4 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0251400000000042);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "prop.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::ORANGE;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(4, prop_properties.init_pos);
    ConfigureUAVPropInitRot(4, prop_properties.init_rot);
    ConfigureUAVPropMass(4, prop_properties.mass);
    ConfigureUAVPropInertiaXX(4, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(4, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(4, prop_properties.COM);
    ConfigureUAVPropOBJName(4, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(4, prop_properties.color);
    ConfigureUAVPropOpacity(4, prop_properties.opacity);

    // ------------------------------------------------ 5 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.0321399999999959);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "prop.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::RED;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(5, prop_properties.init_pos);
    ConfigureUAVPropInitRot(5, prop_properties.init_rot);
    ConfigureUAVPropMass(5, prop_properties.mass);
    ConfigureUAVPropInertiaXX(5, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(5, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(5, prop_properties.COM);
    ConfigureUAVPropOBJName(5, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(5, prop_properties.color);
    ConfigureUAVPropOpacity(5, prop_properties.opacity);

    // ------------------------------------------------ 6 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0321399999999959);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "prop.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::ORANGE;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(6, prop_properties.init_pos);
    ConfigureUAVPropInitRot(6, prop_properties.init_rot);
    ConfigureUAVPropMass(6, prop_properties.mass);
    ConfigureUAVPropInertiaXX(6, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(6, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(6, prop_properties.COM);
    ConfigureUAVPropOBJName(6, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(6, prop_properties.color);
    ConfigureUAVPropOpacity(6, prop_properties.opacity);

    // ------------------------------------------------ 7 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0321399999999958);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "prop.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::RED;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(7, prop_properties.init_pos);
    ConfigureUAVPropInitRot(7, prop_properties.init_rot);
    ConfigureUAVPropMass(7, prop_properties.mass);
    ConfigureUAVPropInertiaXX(7, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(7, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(7, prop_properties.COM);
    ConfigureUAVPropOBJName(7, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(7, prop_properties.color);
    ConfigureUAVPropOpacity(7, prop_properties.opacity);

    // ------------------------------------------------ 8 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0321399999999942);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.42060313992044e-06,1.4208475579394e-06,2.81765034379699e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.69794196181289e-12,1.19342661024841e-12,-1.38379431333175e-13);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-4.58230968269229e-08,-2.31695875230463e-07,-1.27309700362432e-08),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "prop.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::ORANGE;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(8, prop_properties.init_pos);
    ConfigureUAVPropInitRot(8, prop_properties.init_rot);
    ConfigureUAVPropMass(8, prop_properties.mass);
    ConfigureUAVPropInertiaXX(8, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(8, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(8, prop_properties.COM);
    ConfigureUAVPropOBJName(8, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(8, prop_properties.color);
    ConfigureUAVPropOpacity(8, prop_properties.opacity);

    // ------------------------------------------------------------------------------ INITIATE THE PROPELLERS
    InitiateUAVProps();
}

// Compulsary derived class function that initiates all the links for the UAV and passes
// that data to the UAV for initialization.
void simx8::ConfigureX8Links()
{
    // Create the link data vector based on the structure that was defined in the sim-uav.hpp
    std::vector<_uav_::LinkData> link_data_vec;
    
    // ---- Concentric1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.090651591965577,0.105370088769343,-0.0330000000000055),
        chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0292800000000041),
        chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.090651591965577,0.105370088769343,-0.0330000000000055),
        chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0292800000000041),
        chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0883883629628807,0.106432065455224,-0.0210000000000055),
        chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0210000000000041),
        chrono::ChVector3d(0,0,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0883883629628807,0.106432065455224,-0.0210000000000055),
        chrono::ChVector3d(0.0906515919655771,0.105370088769343,-0.0210000000000041),
        chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1),
        chrono::ChVector3d(0,0,1)
    });

    // ---- Concentric2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(0.0911724945380793,-0.111566436084732,-0.033000000000004),
        chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0292800000000041),
        chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(0.0911724945380793,-0.111566436084732,-0.033000000000004),
        chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0292800000000041),
        chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(0.0889161874738823,-0.112643041133974,-0.021000000000004),
        chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0210000000000041),
        chrono::ChVector3d(0,0,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(0.0889161874738823,-0.112643041133974,-0.021000000000004),
        chrono::ChVector3d(0.0911724945380796,-0.111566436084732,-0.0210000000000041),
        chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1),
        chrono::ChVector3d(0,0,1)
    });

    // ---- Concentric3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(-0.0845604352035955,-0.111812919951849,-0.0330000000000045),
        chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0292800000000058),
        chrono::ChVector3d(7.06379399417759e-15,6.86049799777155e-15,1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(-0.0845604352035955,-0.111812919951849,-0.0330000000000045),
        chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0292800000000058),
        chrono::ChVector3d(7.06379399417759e-15,6.86049799777155e-15,1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(-0.0868236642062918,-0.110750943265968,-0.0210000000000045),
        chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0210000000000059),
        chrono::ChVector3d(0,0,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(-0.0868236642062918,-0.110750943265968,-0.0210000000000045),
        chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,-0.0210000000000059),
        chrono::ChVector3d(-7.06379399417759e-15,-6.86049799777155e-15,-1),
        chrono::ChVector3d(0,0,1)
    });

    // ---- Concentric4 (chassis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.085081337776099,0.105123604902229,-0.0330000000000043),
        chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0292800000000041),
        chrono::ChVector3d(7.06379399417755e-15,6.86049799777154e-15,1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.085081337776099,0.105123604902229,-0.0330000000000043),
        chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0292800000000041),
        chrono::ChVector3d(7.06379399417755e-15,6.86049799777154e-15,1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coinciden4 (chassis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.087337644840296,0.104046999852986,-0.0210000000000042),
        chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0210000000000042),
        chrono::ChVector3d(0,0,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.087337644840296,0.104046999852986,-0.0210000000000042),
        chrono::ChVector3d(-0.0850813377760989,0.105123604902229,-0.0210000000000042),
        chrono::ChVector3d(-7.06379399417755e-15,-6.86049799777154e-15,-1),
        chrono::ChVector3d(0,0,1)
    });

    // ---- Concentric5 (chassis to propeller_5) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric5", GetUAVChassis().body, GetUAVProp(5).body,
        chrono::ChVector3d(0.0911724945380798,-0.111566436084732,0.0399999999999961),
        chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.0362799999999959),
        chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric5", GetUAVChassis().body, GetUAVProp(5).body,
        chrono::ChVector3d(0.0911724945380798,-0.111566436084732,0.0399999999999961),
        chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.0362799999999959),
        chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident5 (chassis to propeller_5) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident5", GetUAVChassis().body, GetUAVProp(5).body,
        chrono::ChVector3d(0.0900958894888372,-0.109310129020535,0.027999999999996),
        chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.027999999999996),
        chrono::ChVector3d(0,0,-1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident5", GetUAVChassis().body, GetUAVProp(5).body,
        chrono::ChVector3d(0.0900958894888372,-0.109310129020535,0.027999999999996),
        chrono::ChVector3d(0.0911724945380796,-0.111566436084732,0.027999999999996),
        chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1),
        chrono::ChVector3d(0,0,-1)
    });

    // ---- Concentric6 (chassis to propeller_6) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric6", GetUAVChassis().body, GetUAVProp(6).body,
        chrono::ChVector3d(0.0906515919655775,0.105370088769343,0.0399999999999945),
        chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0362799999999959),
        chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric6", GetUAVChassis().body, GetUAVProp(6).body,
        chrono::ChVector3d(0.0906515919655775,0.105370088769343,0.0399999999999945),
        chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0362799999999959),
        chrono::ChVector3d(-7.06379399417756e-15,-6.86049799777153e-15,-1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident6 (chassis to propeller_6) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident6", GetUAVChassis().body, GetUAVProp(6).body,
        chrono::ChVector3d(0.091713568651458,0.10763331777204,0.0279999999999945),
        chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0279999999999959),
        chrono::ChVector3d(0,0,-1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident6", GetUAVChassis().body, GetUAVProp(6).body,
        chrono::ChVector3d(0.091713568651458,0.10763331777204,0.0279999999999945),
        chrono::ChVector3d(0.0906515919655771,0.105370088769343,0.0279999999999959),
        chrono::ChVector3d(7.06379399417756e-15,6.86049799777153e-15,1),
        chrono::ChVector3d(0,0,-1)
    });

    // ---- Concentric7 (chassis to propeller_7) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric7", GetUAVChassis().body, GetUAVProp(7).body,
        chrono::ChVector3d(-0.0850813377760986,0.105123604902229,0.0399999999999957),
        chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0362799999999958),
        chrono::ChVector3d(-7.06379399417755e-15,-6.86049799777154e-15,-1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric7", GetUAVChassis().body, GetUAVProp(7).body,
        chrono::ChVector3d(-0.0850813377760986,0.105123604902229,0.0399999999999957),
        chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0362799999999958),
        chrono::ChVector3d(-7.06379399417755e-15,-6.86049799777154e-15,-1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident7 (chassis to propeller_7) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident7", GetUAVChassis().body, GetUAVProp(7).body,
        chrono::ChVector3d(-0.0861579428253412,0.107379911966426,0.0279999999999957),
        chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0279999999999958),
        chrono::ChVector3d(0,0,-1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident7", GetUAVChassis().body, GetUAVProp(7).body,
        chrono::ChVector3d(-0.0861579428253412,0.107379911966426,0.0279999999999957),
        chrono::ChVector3d(-0.085081337776099,0.105123604902229,0.0279999999999958),
        chrono::ChVector3d(7.06379399417755e-15,6.86049799777154e-15,1),
        chrono::ChVector3d(0,0,-1)
    });

    // ---- Concentric8 (chassis to propeller_8) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric8", GetUAVChassis().body, GetUAVProp(8).body,
        chrono::ChVector3d(-0.084560435203595,-0.111812919951848,0.0399999999999955),
        chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0362799999999942),
        chrono::ChVector3d(-7.06379399417759e-15,-6.86049799777155e-15,-1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric8", GetUAVChassis().body, GetUAVProp(8).body,
        chrono::ChVector3d(-0.084560435203595,-0.111812919951848,0.0399999999999955),
        chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0362799999999942),
        chrono::ChVector3d(-7.06379399417759e-15,-6.86049799777155e-15,-1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coinciden8 (chassis to propeller_8) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident8", GetUAVChassis().body, GetUAVProp(8).body,
        chrono::ChVector3d(-0.0834984585177144,-0.109549690949152,0.0279999999999955),
        chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0279999999999942),
        chrono::ChVector3d(0,0,-1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident8", GetUAVChassis().body, GetUAVProp(8).body,
        chrono::ChVector3d(-0.0834984585177144,-0.109549690949152,0.0279999999999955),
        chrono::ChVector3d(-0.0845604352035951,-0.111812919951849,0.0279999999999942),
        chrono::ChVector3d(7.06379399417759e-15,6.86049799777155e-15,1),
        chrono::ChVector3d(0,0,-1)
    });

    // Finally link all the bodies together
    this->LinkUAVBodies(link_data_vec);
}

// Compulsary derived class function that initiates the motors with all the necessary parameters
void simx8::ConfigureX8Motors()
{
    // Create the struct for processing the uav motor properties
    _uav_::motorstruct motor_properties;

    // Cache the motor torque constant
    motor_properties.ct =  0.19972258741297011;

    // Cache the rps scaler value 
    motor_properties.rps_scaler = 0.015;

    // Cache the motor polynomial coefficients
    // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
    // TMotor P23069 V3.0 - P23069 V3.0 2550 Kv
    motor_properties.newt2norm = (Eigen::VectorXd(8) << 
                                    1.1397548468561201E-5,
                                    -0.00038436172518955446,
                                    0.0051947365478567029,
                                    -0.036003519831726248,
                                    0.13652913911293302,
                                    -0.2857385586886379,
                                    0.40625173135017634,
                                    -0.0586576952028257
                                 ).finished();

    // Polynomial coefficients vector to evaluate the thrust in [N] based on the Commanded Thrust [-]
    // TMotor P23069 V3.0 - P23069 V3.0 2550 Kv
    motor_properties.norm2newt = (Eigen::VectorXd(8) <<
                                    474.38534956372183,
                                    -1496.7377035961833,
                                    1804.2309340905622,
                                    -1061.3133392140433,
                                    316.1068384833909,
                                    -33.102487712309681,
                                    6.2571119597408353,
                                    0.046322637768972989
                                 ).finished();

    // Polynomial coefficients vector to evaluate the rad/s of motor based on the Commanded Thrust [-]
    // TMotor P23069 V3.0 - P23069 V3.0 2550 Kv                                 
    motor_properties.norm2rps = (Eigen::VectorXd(8) <<
                                    -4479.502173784761,
                                    0,
                                    9512.8235882135123,
                                    0,
                                    -6821.6115897712389,
                                    0,
                                    4570.664148941808,
                                    233.24081458152327
                                ).finished();

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.090651591965577,0.105370088769343,-0.0330000000000055),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(1, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(1, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(1, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(1, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(1, motor_properties.norm2rps);
    ConfigureUAVMotorCt(1, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(1, motor_properties.rps_scaler);
    
    // ------------------------------------------------ 2 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.0911724945380793,-0.111566436084732,-0.033000000000004),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(2, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(2, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(2, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(2, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(2, motor_properties.norm2rps);
    ConfigureUAVMotorCt(2, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(2, motor_properties.rps_scaler);

    // ------------------------------------------------ 3 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.0845604352035955,-0.111812919951849,-0.0330000000000045),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(3, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(3, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(3, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(3, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(3, motor_properties.norm2rps);
    ConfigureUAVMotorCt(3, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(3, motor_properties.rps_scaler);

    // ------------------------------------------------ 4 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.085081337776099,0.105123604902229,-0.0330000000000043),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(4, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(4, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(4, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(4, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(4, motor_properties.norm2rps);
    ConfigureUAVMotorCt(4, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(4, motor_properties.rps_scaler);

    // ------------------------------------------------ 5 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.0911724945380798,-0.111566436084732,0.0399999999999961),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(5, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(5, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(5, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(5, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(5, motor_properties.norm2rps);
    ConfigureUAVMotorCt(5, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(5, motor_properties.rps_scaler);
    
    // ------------------------------------------------ 6 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.0906515919655775,0.105370088769343,0.0399999999999945),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(6, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(6, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(6, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(6, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(6, motor_properties.norm2rps);
    ConfigureUAVMotorCt(6, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(6, motor_properties.rps_scaler);

    // ------------------------------------------------ 7 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.0850813377760986,0.105123604902229,0.0399999999999957),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(7, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(7, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(7, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(7, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(7, motor_properties.norm2rps);
    ConfigureUAVMotorCt(7, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(7, motor_properties.rps_scaler);

    // ------------------------------------------------ 8 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.084560435203595,-0.111812919951848,0.0399999999999955),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(8, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(8, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(8, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(8, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(8, motor_properties.norm2rps);
    ConfigureUAVMotorCt(8, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(8, motor_properties.rps_scaler);
    
    // ---------------------------------------------------------------------------------- INITIATE THE MOTORS
    this->InitiateUAVMotors();
    
}

// Optional derived class function that initiates the aerodynamic properties with all the necessary parameters
void simx8::ConfigureX8Aerodynamics()
{
    // Set the chassis drag coefficient - We consider this a flat plane
    GetUAVChassisAerodynamics().chassis_drag_coefficient = 1.28; 

    // Set the chassis drag surface area - We consider this to be the area of the flat plane so only
    // measure the top surface area of the chassis - preferably this should be computed in blender
    // with the 3D print toolbox on the acutal exported model .obj file.
    GetUAVChassisAerodynamics().chassis_body_surface_aera = 0.07;

    // Set the air density for the simulation
    GetUAVChassisAerodynamics().air_density = 1.225;

}


// Unique function that initiates and attaches the payload to the tailsitter
void simx8::ConfigureX8Payload()
{
    // Initialize the box body
    auto box = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    box->SetName("Box");
    box->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996)));
    box->SetRot(::_shared_::_transformations_::GetChronoOrientaitonFromNED(chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,0,0)));
    box->SetMass(0.356294160000001);
    box->SetInertiaXX(chrono::ChVector3d(0.00230557943592285,0.00230557943592285,0.00384243450096002));
    box->SetInertiaXY(chrono::ChVector3d(0,0,0));
    box->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0,0.0343231818337972,0),chrono::ChQuaternion<>(1,0,0,0)));

    // Visualization shape
    auto box_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>(); 
    box_shape->SetFilename(this->GetUAVShapesDir() + "box.obj");
    box_shape->SetMutable(true);
    box_shape->SetColor(chrono::ChColor(0.0f,0.1f,0.1f));
    box_shape->SetOpacity(0.2);
    box->AddVisualShape(box_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaterniond(1,0,0,0)));

    // Collision model
    box->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

    // Collision material
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    mat->SetRollingFriction(0.01);
    mat->SetSpinningFriction(0.01);

    // Create a triangle mesh collision shape 
    auto mesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(this->GetUAVShapesDir() + "box_collision.obj", false, true);
    mesh->RepairDuplicateVertexes(1e-9);

    auto coll_shape = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(mat, mesh, false, false, 0.001);
    box->AddCollisionShape(coll_shape);
    box->EnableCollision(true);

    // Push back the body 
    this->GetUAVBodyList().push_back(box);

    // Create the links to the chassis
    std::vector<_uav_::LinkData> link_data_vec;

    // // ---- Coincident9 (chassis to box) ----
    // link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
    //     "Coincident9", this->GetUAVChassis().body, box,
    //     chrono::ChVector3d(0.0880455783809917,0.176778584408747,0.225499999999995),
    //     chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996),
    //     chrono::ChVector3d(0,0,1),
    //     0.0
    // });

    // link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
    //     true, "Coincident9", this->GetUAVChassis().body, box,
    //     chrono::ChVector3d(0.0880455783809917,0.176778584408747,0.225499999999995),
    //     chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996),
    //     chrono::ChVector3d(0,0,-1),
    //     chrono::ChVector3d(0,0,1)
    // });

    // // ---- Distance2 (chassis to box) ----
    // link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
    //     "Distance2", this->GetUAVChassis().body, box,
    //     chrono::ChVector3d(0,0,0),
    //     chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996),
    //     chrono::ChVector3d(0,1,0),
    //     0.003
    // });

    // link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
    //     false, "Distance2", this->GetUAVChassis().body, box,
    //     chrono::ChVector3d(0,0,0),
    //     chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996),
    //     chrono::ChVector3d(0,1,0),
    //     chrono::ChVector3d(0,1,0)
    // });

    // // ---- Distance1 (chassis to box) ----
    // link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
    //     "Distance1", this->GetUAVChassis().body, box,
    //     chrono::ChVector3d(0,0,0),
    //     chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996),
    //     chrono::ChVector3d(1,0,0),
    //     -0.003
    // });

    // link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
    //     false, "Distance1", this->GetUAVChassis().body, box,
    //     chrono::ChVector3d(0,0,0),
    //     chrono::ChVector3d(0.00300000000000161,-0.00300000000000006,0.225499999999996),
    //     chrono::ChVector3d(1,0,0),
    //     chrono::ChVector3d(1,0,0)
    // });

    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Fix>{
        "BoxWeld", this->GetUAVChassis().body, box,
    });

    this->LinkUAVBodies(link_data_vec);


    // Add the payload
    auto sph_mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    sph_mat->SetFriction(1.0f);
    sph_mat->SetDampingF(0.5f);

    auto sphereBody = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        0.015,
        1.5 * 14147.1,
        sph_mat
    );

    auto sphereBody1 = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        0.015,
        1.5 * 14147.1,
        sph_mat
    );

    auto sphereBody2 = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        0.015,
        0.5 * 14147.1,
        sph_mat
    );

    auto sphereBody3 = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        0.015,
        0.5 * 14147.1,
        sph_mat
    );

    // sphereBody->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(-0.820174613366277,
    //                                                                                           0.138257774369255,
    //                                                                                           0.0490871680500573)));

    sphereBody->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0.00300000000000161,
                                                                                            -0.00300000000000006,
                                                                                             0.205499999999996)));

    sphereBody1->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0.00300000000000161 + 0.02,
                                                                                              -0.00300000000000006 + 0.02,
                                                                                              0.205499999999996)));

    sphereBody2->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0.00300000000000161 - 0.02,
                                                                                              -0.00300000000000006 - 0.02,
                                                                                              0.205499999999996)));

    sphereBody3->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0.00300000000000161 + 0.02,
                                                                                              -0.00300000000000006 - 0.02,
                                                                                              0.205499999999996)));


                                                                                          
    sphereBody->GetVisualShape(0)->SetTexture(
        chrono::GetChronoDataFile("textures/bluewhite.png")
    );
                                                                                              
    sphereBody1->GetVisualShape(0)->SetTexture(
        chrono::GetChronoDataFile("textures/redwhite.png")
    );

    sphereBody2->GetVisualShape(0)->SetTexture(
        chrono::GetChronoDataFile("textures/greenwhite.png")
    );

    sphereBody3->GetVisualShape(0)->SetTexture(
        chrono::GetChronoDataFile("textures/pinkwhite.png")
    );

    this->getPhysicsSystem().Add(sphereBody);
    this->getPhysicsSystem().Add(sphereBody1);
    this->getPhysicsSystem().Add(sphereBody2);
    this->getPhysicsSystem().Add(sphereBody3);


}

}   // namespace _x8_

}   // namespace _acsl_