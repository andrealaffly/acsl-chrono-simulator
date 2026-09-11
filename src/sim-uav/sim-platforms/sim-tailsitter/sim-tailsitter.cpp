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
 * File:        sim-tailsitter.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 23, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived UAV class for the simulated tailsitter.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-tailsitter.hpp"

namespace _acsl_
{

namespace _tailsitter_
{

// Compulsary derived class function that initiates the chassis with all the necessary parameters
void simtailsitter::ConfigureTAILSITTERChassis()
{
    // Create the struct for processing the uav chassis properties
    _uav_::chassisstruct chassis_properties;

    // Configure the initial position of the chassis ------------ -----------------------------------------------------
    chassis_properties.init_pos = chrono::ChVector3d(-0.820174552834782,0.138326409126481,0.0275871680500605);
    
    // Configure the initial rotation of the chasssis -----------------------------------------------------------------
    chassis_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);
    
    // Configure the mass of the chassis ------------------------------------------------------------------------------
    chassis_properties.mass = 1.54332953655354;

    // Configure the Chassis Inertia vectors --------------------------------------------------------------------------
    chassis_properties.InertiaXX = chrono::ChVector3d(0.151016870974707,0.0512672876060956,0.187965807814355);
    chassis_properties.InertiaXY = chrono::ChVector3d(-5.06412826018973e-07,6.54535930503061e-06,1.71490800154151e-06);

    // Configure the chassis COM along with it's auxilliary frame at the COM ------------------------------------------
    // This is the most important as we will consider this to be the pixhawk in our subsequent calculations
    chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(0.000260032975304549,
                                                                  3.98043445694742e-05,
                                                                  0.0115331100824462),
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
    mr(0,0)=1;                     mr(1,0)=1.85037170770859E-15; mr(2,0)=0;
    mr(0,1)=-1.09836294215862E-16; mr(1,1)=1.09836294215862E-14; mr(2,1)=-1;
    mr(0,2)=-1.85037170770859E-15; mr(1,2)=1;                    mr(2,2)=1.09836294215862E-14;
    pos = chrono::ChVector3d(0.222750060533507,-6.86347572278478E-05,0.0529997453814312);
    dim = chrono::ChVector3d(0.06,0.252699490762868,1.5);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

    chassis_properties.collision.emplace_back(col, frame);

    // 2
    mr(0,0)=-1;                    mr(1,0)=-5.55111512312549E-15; mr(2,0)=4.69531820831031E-14;
    mr(0,1)=1.09836294215861E-16;  mr(1,1)=-1.14229745984496E-14; mr(2,1)=1;
    mr(0,2)=-5.55111512312495E-15; mr(1,2)=1;                     mr(2,2)=1.14229745984496E-14;
    pos = chrono::ChVector3d(-0.222749939466497,-6.86347572295132E-05,0.0529997453814354);
    dim = chrono::ChVector3d(0.0600000000000032,0.25269949076287,1.5);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

    chassis_properties.collision.emplace_back(col, frame);

    // 3
    mr(0,0)=-1;                    mr(1,0)=-4.1646502831122E-16;  mr(2,0)=0;
    mr(0,1)=-1.80231010491098E-16; mr(1,1)=-7.20924041964391E-16; mr(2,1)=1;
    mr(0,2)=-4.1646502831122E-16;  mr(1,2)=1;                     mr(2,2)=7.20924041964391E-16;
    pos = chrono::ChVector3d(-6.05329052111353E-08,6.82461176663568E-05,-0.0169999999999997);
    dim = chrono::ChVector3d(0.199936900307179,0.0769999999999996,0.0760000000000003);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

    // FIX - assign collision groups for box and frame
    // don't really need this anyway.
    // chassis_properties.collision.emplace_back(col, frame);

    // 4
    mr(0,0)=-1;                   mr(1,0)=-2.18684361597377E-15; mr(2,0)=-1.1007112867068E-14;
    mr(0,1)=0;                    mr(1,1)=3.26560016492296E-15;  mr(2,1)=-1;
    mr(0,2)=2.18684361597381E-15; mr(1,2)=-1;                    mr(2,2)=-3.26560016492296E-15;
    pos = chrono::ChVector3d(6.05357001302583E-08,-0.459068634757229,0.12935581309071);
    dim = chrono::ChVector3d(0.380762146130002,0.0509962779529998,0.0100000000000001);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

    chassis_properties.collision.emplace_back(col, frame);

    // 5
    mr(0,0)=0;                    mr(1,0)=1.08853338830768E-14; mr(2,0)=-1;
    mr(0,1)=1;                    mr(1,1)=2.18684361597378E-15; mr(2,1)=0;
    mr(0,2)=2.18684361597378E-15; mr(1,2)=-1;                   mr(2,2)=-1.08853338830768E-14;
    pos = chrono::ChVector3d(6.05356991865687E-08,0.458931365242771,0.129355813090654);
    dim = chrono::ChVector3d(0.0509962779529988,0.38076214613,0.0100000000000002);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

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
void simtailsitter::ConfigureTAILSITTERProps()
{
    // Create the struct for processing the uav propeller properties
    _uav_::propstruct prop_properties;

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0472628319499406);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(2.53545399382762e-15,2.53545399382763e-15,-0.707106781186546,0.707106781186549);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.02311480613903;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(0.000243481962218972,2.00887421444941e-06,0.000245256129385561);
    prop_properties.InertiaXY = chrono::ChVector3d(1.27126215242116e-05,1.25226930519482e-14,8.53229755274596e-15);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-3.39798185223223e-09,-0.00151682657384863,1.1268864500665e-09),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "propeller.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::BLACK;
    prop_properties.opacity = 0.4;

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
    prop_properties.init_pos = chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0472628319499344);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(2.53545399382762e-15,2.53545399382763e-15,-0.707106781186546,0.707106781186549);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.02311480613903;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(0.000243481962218972,2.00887421444941e-06,0.000245256129385561);
    prop_properties.InertiaXY = chrono::ChVector3d(1.27126215242116e-05,1.25226930519482e-14,8.53229755274596e-15);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-3.39798185223223e-09,-0.00151682657384863,1.1268864500665e-09),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "propeller.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::BLACK;
    prop_properties.opacity = 0.4;

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
    prop_properties.init_pos = chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0472628319499391);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(2.53545399382762e-15,2.53545399382763e-15,-0.707106781186546,0.707106781186549);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.02311480613903;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(0.000243481962218972,2.00887421444941e-06,0.000245256129385561);
    prop_properties.InertiaXY = chrono::ChVector3d(1.27126215242116e-05,1.25226930519482e-14,8.53229755274596e-15);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-3.39798185223223e-09,-0.00151682657384863,1.1268864500665e-09),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "propeller.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::BLACK;
    prop_properties.opacity = 0.4;

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
    prop_properties.init_pos = chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0472628319499379);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(2.53545399382762e-15,2.53545399382763e-15,-0.707106781186546,0.707106781186549);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.02311480613903;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(0.000243481962218972,2.00887421444941e-06,0.000245256129385561);
    prop_properties.InertiaXY = chrono::ChVector3d(1.27126215242116e-05,1.25226930519482e-14,8.53229755274596e-15);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-3.39798185223223e-09,-0.00151682657384863,1.1268864500665e-09),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "propeller.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::BLACK;
    prop_properties.opacity = 0.4;

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

    // ------------------------------------------------------------------------------ INITIATE THE PROPELLERS
    InitiateUAVProps();
}

// Compulsary derived class function that initiates all the links for the UAV and passes
// that data to the UAV for initialization.
void simtailsitter::ConfigureTAILSITTERLinks()
{
    // Create the link data vector based on the structure that was defined in the sim-uav.hpp
    std::vector<_uav_::LinkData> link_data_vec;
    
    // ---- Concentric1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(-0.597424492301444,0.397257774369086,-0.0507628319499394),
        chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0487628319499406),
        chrono::ChVector3d(-3.50566874250047e-15,5.22275672758959e-15,-1),
        chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(-0.597424492301444,0.397257774369086,-0.0507628319499394),
        chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0487628319499406),
        chrono::ChVector3d(-3.50566874250047e-15,5.22275672758959e-15,-1),
        chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1),
        true, true, false, false, false, false
    });

    // ---- Coincident1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(-0.597424492301444,0.397257774369086,-0.0457628319499394),
        chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0457628319499406),
        chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(-0.597424492301444,0.397257774369086,-0.0457628319499394),
        chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0457628319499406),
        chrono::ChVector3d(-3.50566874250047e-15,5.22275672758959e-15,-1),
        chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1)
    });

    // ---- Concentric2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-1.04292449230144,-0.120742225630915,-0.0507628319499392),
        chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0487628319499344),
        chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1),
        chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-1.04292449230144,-0.120742225630915,-0.0507628319499392),
        chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0487628319499344),
        chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1),
        chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1),
        true, true, false, false, false, false
    });

    // ---- Coincident2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-1.04292449230144,-0.120742225630915,-0.0457628319499392),
        chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0457628319499344),
        chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-1.04292449230144,-0.120742225630915,-0.0457628319499392),
        chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0457628319499344),
        chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1),
        chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1)
    });

    // ---- Concentric3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(-0.597424492301445,-0.120742225630913,-0.0507628319499426),
        chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0487628319499391),
        chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1),
        chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(-0.597424492301445,-0.120742225630913,-0.0507628319499426),
        chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0487628319499391),
        chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1),
        chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1),
        true, true, false, false, false, false
    });

    // ---- Coincident3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(-0.597424492301445,-0.120742225630913,-0.0457628319499426),
        chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0457628319499391),
        chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(-0.597424492301445,-0.120742225630913,-0.0457628319499426),
        chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0457628319499391),
        chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1),
        chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1)
    });

    // ---- Concentric4 (chassis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-1.04292449230144,0.397257774369086,-0.0507628319499354),
        chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0487628319499379),
        chrono::ChVector3d(-4.22524492555908e-15,1.03617656841521e-14,-1),
        chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-1.04292449230144,0.397257774369086,-0.0507628319499354),
        chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0487628319499379),
        chrono::ChVector3d(-4.22524492555908e-15,1.03617656841521e-14,-1),
        chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1),
        true, true, false, false, false, false
    });

    // ---- Coincident4 (chasssis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-1.04292449230144,0.397257774369086,-0.0457628319499354),
        chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0457628319499379),
        chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-1.04292449230144,0.397257774369086,-0.0457628319499354),
        chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0457628319499379),
        chrono::ChVector3d(-4.22524492555908e-15,1.03617656841521e-14,-1),
        chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1)
    });


    // Finally link all the bodies together
    LinkUAVBodies(link_data_vec);
}

// Compulsary derived class function that initiates the motors with all the necessary parameters
void simtailsitter::ConfigureTAILSITTERMotors()
{
    // Create the struct for processing the uav motor properties
    _uav_::motorstruct motor_properties;

    // Cache the motor torque constant
    motor_properties.ct = 0.542418958461701;

    // Cache the rps scaler value 
    motor_properties.rps_scaler = 0.25;

    // Cache the motor polynomial coefficients
    // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
    // TMotor F35A - Velox V2808 Kv1300
    motor_properties.newt2norm = (Eigen::VectorXd(8) << 
                                     2.4591731397119028E-10,
                                    -2.9764216922011841E-8,
                                     1.5256975606726995E-6,
                                    -4.4210707058374142E-5,
                                     0.00081199079192906959,
                                    -0.0098303333146295951,
                                     0.094801218437836871,
                                    -0.012240927327923512
                                 ).finished();

    // Polynomial coefficients vector to evaluate the thrust in [N] based on the Commanded Thrust [-]
    // based on the TMotor F35A - Velox V2808 Kv1300
    motor_properties.norm2newt = (Eigen::VectorXd(8) <<
                                    -1225.1452159827293,
                                     3889.9087791150923,
                                    -4796.6047697858776,
                                     2844.362204073208,
                                    -809.95536988748518,
                                     128.06200953087625,
                                     4.2767571075650777,
                                     0.22158755108870978
                                 ).finished();

    // Polynomial coefficients vector to evaluate the rad/s of motor based on the Commanded Thrust [-]
    // based on the TMotor F35A - Velox V2808 Kv1300                                 
    motor_properties.norm2rps = (Eigen::VectorXd(8) <<
                                    -32359.819235086237,
                                     113883.27012694081,
                                    -158219.57844917616,
                                     108564.74451572458,
                                    -36915.814365961291,
                                     4757.5215597332008,
                                     1069.2500477197036,
                                    -30.701497890560272
                                ).finished();

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.597424492301397,0.397257774369148,-0.0457628319499394),
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
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-1.04292449230149,-0.120742225630677,-0.0457628319499405),
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
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.597424492301483,-0.120742225630798,-0.0457628319499421),
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
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0457628319499378),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(4, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(4, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(4, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(4, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(4, motor_properties.norm2rps);
    ConfigureUAVMotorCt(4, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(4, motor_properties.rps_scaler);

    // ---------------------------------------------------------------------------------- INITIATE THE MOTORS
    InitiateUAVMotors();
    
}

// Optional derived class function that initiates the aerodynamic properties with all the necessary parameters
void simtailsitter::ConfigureTAILSITTERAerodynamics()
{

    // ------------------------------------------------------------------------------- CONFIGURE CHASSIS DRAG
    // Set the chassis air density used
    GetUAVChassisAerodynamics().air_density = 1.228;

    // Set the chassis drag coefficient - We consider this a flat plane
    GetUAVChassisAerodynamics().chassis_drag_coefficient = 1.28; 

    // Set the chassis drag surface area - We consider this to be the area of the flat plane so only
    // measure the top surface area of the chassis - preferably this should be computed in blender
    // with the 3D print toolbox on the acutal exported model .obj file.
    GetUAVChassisAerodynamics().chassis_body_surface_aera = 3.2126;

    // ------------------------------------------------------------------------------------ COMMON PARAMETERS
    // Create a vector for the aerodynamic surfaces
    std::vector<::_acsl_::_uav_::aerodynamicsurfacestruct> aerofoils;

    // Create a single object for the aerodynamic surface
    ::_acsl_::_uav_::aerodynamicsurfacestruct aerofoil;

    // Set the common properties for the aerodynamic surfaces
    // Set the air density
    aerofoil.air_density = 1.228;

    // ---------------------------------------------------------------------------------- CONFIGURE THE WINGS
    // Now, we set the common properties for the wings - the span and chord are the same.
    // q_relative is the relative orientation of the aerodynamic surface with respect to the body NED frame.
    // Set the relative direction vectors for lift and drag forces with respect to the body NED frame
    //   - Lift acts along -X (negative body X axis).
    //   - Drag acts along +Z (positive body Z axis).
    // Set the number of aero centers, it's the same for both wings as we want it to be symmetrical
    aerofoil.aerofoil_span = 1.50;
    aerofoil.aerofoil_chord = 0.2;
    aerofoil.q_relative = chrono::QuatFromAngleY(chrono::CH_PI_2);
    aerofoil.lift_force_relative_vector = chrono::ChVector3d(-1, 0, 0);
    aerofoil.drag_force_relative_vector = chrono::ChVector3d(0, 0, 1);
    aerofoil.num_of_aero_centers = 10;

    // Lower Wing
    aerofoil.name = "lower_wing";
    aerofoil.p1 = chrono::ChVector3d(0.22251642,0.75 + 0.00011867,0.01801629);
    aerofoil.p2 = chrono::ChVector3d(0.22251642,-0.75 + 0.00011867,0.01801629);

    // Add the wing to the vector of aerofoils
    aerofoils.push_back(aerofoil);

    // Upper Wing
    aerofoil.name = "upper_wing";
    aerofoil.p1 = chrono::ChVector3d(-0.22251642,0.75 + 0.00011867,0.01801629);
    aerofoil.p2 = chrono::ChVector3d(-0.22251642,-0.75 + 0.00011867,0.01801629);

    // Add the wing to the vector of aerofoils
    aerofoils.push_back(aerofoil);

    // ---------------------------------------------------------------------------- CONFIGURE THE STABILIZERS
    // Now, we set the common properties for the stablizers - the span and chord are the same.
    // q_relative is the relative orientation of the aerodynamic surface with respect to the body NED frame.
    // Set the relative direction vectors for lift and drag forces with respect to the body NED frame
    //   - Lift acts along -X (negative body X axis).
    //   - Drag acts along +Z (positive body Z axis).
    // Set the number of aero centers, it's the same for both stabilizers as we want it to be symmetrical
    aerofoil.aerofoil_span = 0.10516;
    aerofoil.aerofoil_chord = 0.05;
    aerofoil.num_of_aero_centers = 5;

    // Right Stabilizer
    aerofoil.name = "right_stabilizer";
    aerofoil.p1 = chrono::ChVector3d(0.21012, 0.47, 0.10782);
    aerofoil.p2 = chrono::ChVector3d(-0.21064, 0.47, 0.10782);
    aerofoil.q_relative = chrono::QuatFromAngleY(chrono::CH_PI_2) *  chrono::QuatFromAngleX(chrono::CH_PI_2);
    aerofoil.lift_force_relative_vector = chrono::ChVector3d(0, 1, 0);
    aerofoil.drag_force_relative_vector = chrono::ChVector3d(0, 0, 1);

    // Add the stabilizer to the vector of aerofoils
    aerofoils.push_back(aerofoil);

    // Left Stabilizer
    aerofoil.name = "left_stabilizer";
    aerofoil.p1 = chrono::ChVector3d(0.21012, -0.4475, 0.10782);
    aerofoil.p2 = chrono::ChVector3d(-0.21064, -0.4475, 0.10782);
    aerofoil.q_relative = chrono::QuatFromAngleY(chrono::CH_PI_2) *  chrono::QuatFromAngleX(-chrono::CH_PI_2);
    aerofoil.lift_force_relative_vector = chrono::ChVector3d(0, -1, 0);
    aerofoil.drag_force_relative_vector = chrono::ChVector3d(0, 0, 1);

    // Add the stabilizer to the vector of aerofoils
    aerofoils.push_back(aerofoil);

    // Finally configure the UAV aerodynamics with the wings we have set up
    ConfigureUAVAeroSurfaces(aerofoils);

}

// Unique function that initiates and attaches the payload to the tailsitter
void simtailsitter::ConfigureTAILSITTERPayload()
{
    // Initialize the box body
    auto box = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    box->SetName("Box");
    box->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(-0.820174613366277,0.138257774369255,0.0490871680500573)));
    box->SetRot(::_shared_::_transformations_::GetChronoOrientaitonFromNED(chrono::ChQuaternion<>(1,0,0,0)));
    box->SetMass(0.142211570470112);
    box->SetInertiaXX(chrono::ChVector3d(0.000475676703258402,0.000431812269027828,0.000781017171981662));
    box->SetInertiaXY(chrono::ChVector3d(0,-2.46791768127287e-19,-1.65702232028199e-21));
    box->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-1.67786076288889e-17,0,0.025),chrono::ChQuaternion<>(1,0,0,0)));

    // Visualization shape
    auto box_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>(); 
    box_shape->SetFilename(this->GetUAVShapesDir() + "box.obj");
    box_shape->SetMutable(true);
    box_shape->SetColor(chrono::ChColor(0.0f,0.1f,0.1f));
    box_shape->SetOpacity(0.1);
    box->AddVisualShape(box_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaterniond(1,0,0,0)));

    // Collision model
    box->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

    // Collision material
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    mat->SetRollingFriction(0.01);
    mat->SetSpinningFriction(0.01);

    // Create a triangle mesh collision shape 
    auto mesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(this->GetUAVShapesDir() + "box.obj", false, true);
    mesh->RepairDuplicateVertexes(1e-9);

    auto coll_shape = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(mat, mesh, false, false, 0.005);
    box->AddCollisionShape(coll_shape);
    box->EnableCollision(true);

    // Push back the body 
    this->GetUAVBodyList().push_back(box);

    // Create the links to the chassis
    std::vector<_uav_::LinkData> link_data_vec;

    // ---- Coincident15 (chassis to box) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident15", this->GetUAVChassis().body, box,
        chrono::ChVector3d(-0.776202997969024,0.169865825445263,0.0490871680500605),
        chrono::ChVector3d(-0.820174613366277,0.138257774369255,0.0490871680500573),
        chrono::ChVector3d(0,0,-1),
        0.0
    });

    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident15", this->GetUAVChassis().body, box,
        chrono::ChVector3d(-0.776202997969024,0.169865825445263,0.0490871680500605),
        chrono::ChVector3d(-0.820174613366277,0.138257774369255,0.0490871680500573),
        chrono::ChVector3d(-4.08802930999812e-20,9.30604413408574e-19,1),
        chrono::ChVector3d(0,0,-1)
    });

    // ---- Coincident16 (chassis to box) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident16", this->GetUAVChassis().body, box,
        chrono::ChVector3d(-0.895143063521276,-0.120742225630746,-0.00641283194993837),
        chrono::ChVector3d(-0.895143063521276,0.0582577743692548,0.0990871680500574),
        chrono::ChVector3d(-1,0,0),
        0.0
    });

    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident16", this->GetUAVChassis().body, box,
        chrono::ChVector3d(-0.895143063521276,-0.120742225630746,-0.00641283194993837),
        chrono::ChVector3d(-0.895143063521276,0.0582577743692548,0.0990871680500574),
        chrono::ChVector3d(1,1.38376723274325e-15,-3.53608945387768e-15),
        chrono::ChVector3d(-1,0,0)
    });

    // ---- Distance2 (chassis to box) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Distance2", this->GetUAVChassis().body, box,
        chrono::ChVector3d(-0.895143063521276,0.218257774369255,0.0990871680500574),
        chrono::ChVector3d(-0.813345143747726,0.178257774369254,0.00458716805006078),
        chrono::ChVector3d(-4.41487124636098e-16,1,-9.30604413453649e-19),
        0.04
    });

    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Distance2", this->GetUAVChassis().body, box,
        chrono::ChVector3d(-0.895143063521276,0.218257774369255,0.0990871680500574),
        chrono::ChVector3d(-0.813345143747726,0.178257774369254,0.00458716805006078),
        chrono::ChVector3d(0,1,0),
        chrono::ChVector3d(-4.41487124636098e-16,1,-9.30604413453649e-19)
    });


    this->LinkUAVBodies(link_data_vec);


    // Add the payload
    auto sph_mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    sph_mat->SetFriction(1.0f);
    sph_mat->SetDampingF(0.5f);

    auto sphereBody = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        0.015,
        4.435 * 7073.55,
        sph_mat
    );

    auto sphereBody1 = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        0.015,
        4.425 * 7073.55,
        sph_mat
    );

    auto sphereBody2 = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        0.015,
        4.415 * 7073.55,
        sph_mat
    );

    auto sphereBody3 = chrono_types::make_shared<chrono::ChBodyEasySphere>(
        0.015,
        3.105 * 7073.55,
        sph_mat
    );

    sphereBody->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(-0.820174613366277,
                                                                                              0.138257774369255,
                                                                                              0.0490871680500573)));

    sphereBody1->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(-0.820174613366277 + 0.025,
                                                                                              0.138257774369255 + 0.025,
                                                                                              0.0490871680500573)));

    sphereBody2->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(-0.820174613366277 - 0.025,
                                                                                              0.138257774369255 - 0.025,
                                                                                              0.0490871680500573)));

    sphereBody3->SetPos(::_shared_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(-0.820174613366277 + 0.025,
                                                                                              0.138257774369255 - 0.025,
                                                                                              0.0490871680500573)));


                                                                                          
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

}   // namespace _tailsitter_

}   // namespace _acsl_