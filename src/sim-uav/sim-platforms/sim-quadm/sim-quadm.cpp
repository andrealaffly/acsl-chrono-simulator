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
 * File:        sim-quadm.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 11, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived UAV class for the simulated quadm.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-quadm.hpp"

namespace _acsl_
{

namespace _quadm_
{

// Compulsary derived class function that initiates the chassis with all the necessary parameters
void simquadm::ConfigureQUADMChassis()
{
    // Create the struct for processing the uav chassis properties
    _uav_::chassisstruct chassis_properties;

    // Configure the initial position of the chassis ------------ -----------------------------------------------------
    chassis_properties.init_pos = chrono::ChVector3d(0,0,0);
    
    // Configure the initial rotation of the chasssis -----------------------------------------------------------------
    chassis_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);
    
    // Configure the mass of the chassis ------------------------------------------------------------------------------
    chassis_properties.mass = 1.27359544600205;

    // Configure the Chassis Inertia vectors --------------------------------------------------------------------------
    chassis_properties.InertiaXX = chrono::ChVector3d(0.00654969371759306,0.00621558396568854,0.0102525161842417);
    chassis_properties.InertiaXY = chrono::ChVector3d(-1.21105971588835e-06,-9.27032947877171e-06,-2.16784001716213e-06);

    // Configure the chassis COM along with it's auxilliary frame at the COM ------------------------------------------
    // This is the most important as we will consider this to be the pixhawk in our subsequent calculations
    // chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(-0.0163151142227842,0.0102080920244478,0.017016814810521),
    //                                            chrono::ChQuaternion<>(1,0,0,0)); 

    chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(0.000979643148821178,0.000801422765113241,0.000453896450721421),
                                               chrono::ChQuaternion<>(1,0,0,0)); 

    // Configure the name of the visualization obj file ---------------------------------------------------------------
    chassis_properties.vis_obj_name = "body_1_1.obj";

    // Configure the chassis collision shapes -------------------------------------------------------------------------
    // Cache some variables for this 
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();   // Material
    std::shared_ptr<chrono::ChCollisionShape> col;                          // Collision shape
    chrono::ChMatrix33<> mr;                                                // Rotation Matrix for the frame definition
    chrono::ChVector3d pos;                                                 // Position Vector for the frame definition
    chrono::ChVector3d dim;                                                 // Dimensions of the collison box 
    chrono::ChFrame<> frame;                                                // Frame of the collision shape

    // 1
    mr(0,0)=-1; mr(1,0)=0; mr(2,0)=5.83579624654883E-17;
    mr(0,1)=0; mr(1,1)=-1; mr(2,1)=-1.56852284371505E-16;
    mr(0,2)=5.83579624654883E-17; mr(1,2)=-1.56852284371505E-16; mr(2,2)=1;
    pos = chrono::ChVector3d(0.00120871121755825,0.000836515898205509,-0.00287856405321896);
    dim = chrono::ChVector3d(0.237804529519369,0.26543039261599,0.137);
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
void simquadm::ConfigureQUADMProps()
{
    // Create the struct for processing the uav propeller properties
    _uav_::propstruct prop_properties;

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.035503564053219);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_2_1.obj";

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
    prop_properties.init_pos = chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.035503564053219);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_2_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
        prop_properties.color = _uav_::_prop_color_::RED;
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
    prop_properties.init_pos = chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.035503564053219);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_2_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::ORANGE;
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
    prop_properties.init_pos = chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.035503564053219);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_2_1.obj";

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

    // ------------------------------------------------------------------------------ INITIATE THE PROPELLERS
    InitiateUAVProps();
}

// Compulsary derived class function that initiates all the links for the UAV and passes
// that data to the UAV for initialization.
void simquadm::ConfigureQUADMLinks()
{
    // Create the link data vector based on the structure that was defined in the sim-uav.hpp
    std::vector<_uav_::LinkData> link_data_vec;
    
    // ---- Concentric1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0463785640532189),
        chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.039128564053219),
        chrono::ChVector3d(9.86076131526265e-32,4.93038065763132e-32,-1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0463785640532189),
        chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.039128564053219),
        chrono::ChVector3d(9.86076131526265e-32,4.93038065763132e-32,-1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0971766624903463,0.108491147175327,-0.0318785640532189),
        chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0318785640532189),
        chrono::ChVector3d(0,0,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0971766624903463,0.108491147175327,-0.0318785640532189),
        chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0318785640532189),
        chrono::ChVector3d(1.20533813419052e-17,1.72292970257481e-17,-1),
        chrono::ChVector3d(0,0,1)
    });

    // ---- Concentric2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0463785640532189),
        chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0391285640532189),
        chrono::ChVector3d(2.46519032881566e-32,-7.39557098644699e-32,-1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0463785640532189),
        chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0391285640532189),
        chrono::ChVector3d(2.46519032881566e-32,-7.39557098644699e-32,-1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.0926329885112532,-0.110812993896686,-0.0318785640532189),
        chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.031878564053219),
        chrono::ChVector3d(0,0,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.0926329885112532,-0.110812993896686,-0.0318785640532189),
        chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.031878564053219),
        chrono::ChVector3d(-1.72292970257482e-17,1.20533813419047e-17,-1),
        chrono::ChVector3d(0,0,1)
    });

    // ---- Concentric3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0463785640532189),
        chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0391285640532189),
        chrono::ChVector3d(6.16297582203915e-33,-7.39557098644699e-32,-1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0463785640532189),
        chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0391285640532189),
        chrono::ChVector3d(6.16297582203915e-33,-7.39557098644699e-32,-1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coincident3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.100733027547883,-0.108044328872842,-0.0318785640532189),
        chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.031878564053219),
        chrono::ChVector3d(0,0,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.100733027547883,-0.108044328872842,-0.0318785640532189),
        chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.031878564053219),
        chrono::ChVector3d(-1.20533813418501e-17,1.72292970257864e-17,-1),
        chrono::ChVector3d(0,0,1)
    });

    // ---- Concentric4 (chassis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0463785640532189),
        chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.039128564053219),
        chrono::ChVector3d(6.16297582203915e-33,-7.39557098644699e-32,-1),
        chrono::ChVector3d(0,0,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0463785640532189),
        chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.039128564053219),
        chrono::ChVector3d(6.16297582203915e-33,-7.39557098644699e-32,-1),
        chrono::ChVector3d(0,0,1),
        true, true, false, false, false, false
    });

    // ---- Coinciden4 (chassis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.0931387230643319,0.10962483877249,-0.0318785640532189),
        chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0318785640532189),
        chrono::ChVector3d(0,0,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.0931387230643319,0.10962483877249,-0.0318785640532189),
        chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0318785640532189),
        chrono::ChVector3d(-1.20533813418501e-17,1.72292970257864e-17,-1),
        chrono::ChVector3d(0,0,1)
    });

    // Finally link all the bodies together
    LinkUAVBodies(link_data_vec);
}

// Compulsary derived class function that initiates the motors with all the necessary parameters
void simquadm::ConfigureQUADMMotors()
{
    // Create the struct for processing the uav motor properties
    _uav_::motorstruct motor_properties;

    // Cache the motor torque constant
    motor_properties.ct =  0.11263070755508531;

    // Cache the rps scaler value 
    motor_properties.rps_scaler = 0.015;

    // Cache the motor polynomial coefficients
    // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
    // TMotor F35A - Velox V2808 Kv1300
    motor_properties.newt2norm = (Eigen::VectorXd(8) << 
                                     3.0316084940856985E-8,
                                    -1.8144686579320055E-6,
                                    4.2434512948450148E-5,
                                    -0.00050487431926882675,
                                    0.0035249290605074172,
                                    -0.017552333963266033,
                                    0.1149654034763074,
                                    0.011001286689222546
                                 ).finished();

    // Polynomial coefficients vector to evaluate the thrust in [N] based on the Commanded Thrust [-]
    // based on the TMotor F35A - Velox V2808 Kv1300
    motor_properties.norm2newt = (Eigen::VectorXd(8) <<
                                   -964.19079412220037,
                                    3455.3831563385143,
                                    -4891.1971714992978,
                                    3484.3076361578146,
                                    -1331.2807972002227,
                                    282.15568651345643,
                                    -17.583937014183508,
                                    0.79484684468749423
                                 ).finished();

    // Polynomial coefficients vector to evaluate the rad/s of motor based on the Commanded Thrust [-]
    // based on the TMotor F35A - Velox V2808 Kv1300                                 
    motor_properties.norm2rps = (Eigen::VectorXd(8) <<
                                    -1468.5518114227311,
                                    0,
                                    3053.5665052664854,
                                    0,
                                    -2172.4802760460461,
                                    0,
                                    2722.5525374402337,
                                    307.17670168725192
                                ).finished();

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0463785640532189),
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
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0463785640532189),
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
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0463785640532189),
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
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0463785640532189),
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
void simquadm::ConfigureQUADMAerodynamics()
{
    // Set the chassis drag coefficient - We consider this a flat plane
    GetUAVAerodynamics().chassis_drag_coefficient = 1.28; 

    // Set the chassis drag surface area - We consider this to be the area of the flat plane so only
    // measure the top surface area of the chassis - preferably this should be computed in blender
    // with the 3D print toolbox on the acutal exported model .obj file.
    GetUAVAerodynamics().chassis_body_surface_aera = 0.06;

    // Set the air density for the simulation
    GetUAVAerodynamics().air_density = 1.228;

}



}   // namespace _quadm_

}   // namespace _acsl_