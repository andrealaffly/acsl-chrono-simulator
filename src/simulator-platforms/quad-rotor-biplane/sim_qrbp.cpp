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
 * File:        sim_qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 18, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class definition for the model of the qrbp.
 * 
 * GitHub:    https://github.com/andrealaffly/acsl-chrono-simulator.git
 **********************************************************************************************************************/

#include "sim_qrbp.hpp"

namespace _platforms_
{
namespace _uav_
{
namespace _qrbp_
{

void qrbp::SetupDrone()
{
    // Set chassis porperties
    SetChassisMass(2.09224219084586);

    SetChassisInitialPosition(chrono::ChVector3d(0.0171306785555524,
                                                -0.00832708181360941,
                                                -0.00785621720215227));

    SetChassisInitialRotation(chrono::ChQuaternion<>(-0.499999999999998,
                                                      0.500000000000005,
                                                     -0.500000000000002,
                                                      0.499999999999995));    

    SetChassisInertia(chrono::ChVector3d(0.0221767221436968,0.0378698749614418,0.0324427018402532),
                      chrono::ChVector3d(1.04305458836941e-06,-8.98410015148422e-06,0.00091943506900011));

    SetChassisAuxFrame(chrono::ChFramed(chrono::ChVector3d(-0.0132794988606019,
                                                           -0.00580556028628224,
                                                           -0.0144622284658121),
                                        chrono::ChQuaternion<>(0.7071,-0.7071,0,0)));

    SetChassisMeshFileName(std::string(SHAPES_DIR) + "body_1_1.obj");

    SetChassisMeshFrame(chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));
    
    // Set properller properties
    SetPropellerMass(0.00395402309271749);

    SetPropellerInitialPosition({chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0452097403684691),
                                 chrono::ChVector3d(-0.0876697193697793,-0.105809423167326,-0.0452097403684664),
                                 chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0179597403684691),
                                 chrono::ChVector3d(-0.0876697193697808,0.115620969448612,-0.0179597403684664)});

    SetPropellerInitialRotation(
        {chrono::ChQuaternion<>(0.106343145835906,6.94336572027546e-15,7.42590218399898e-16,0.994329490326885),
         chrono::ChQuaternion<>(-0.00835912077524803,6.98271870651446e-15,-5.83714283918076e-17,0.999965061939598),
         chrono::ChQuaternion<>(0.856221155277496,-3.56222304048412e-15,6.02022298477362e-15,-0.516609459122914),
         chrono::ChQuaternion<>(0.986223667061496,-1.09785465125519e-15,6.90848991757748e-15,-0.165417286060968)}
    );

    SetPropellerInertia(chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06),
                        chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11));

    
    SetPropellerDirections({true, true, false, false});

    SetPropellerMeshFilenames({std::string(SHAPES_DIR) + "body_2_1.obj",
                               std::string(SHAPES_DIR) + "body_3_1.obj",
                               std::string(SHAPES_DIR) + "body_4_1.obj",
                               std::string(SHAPES_DIR) + "body_5_1.obj"});

    SetPropellerMeshFrames({chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)),
                            chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)),
                            chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)),
                            chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0))});

    // Initalize the drone framework
    InitFramework();

    // Add collision Shapes
    SetupCollision();

    // Intialize the visualization
    SetupVisualization(chrono::utils::ChChaseCamera::State::Follow, 3.0, 0.75);

}

void qrbp::SetupCollision()
{
    // Collision material
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialSMC>();

    // Cache some variables
    std::shared_ptr<chrono::ChCollisionShape> colshape;
    double height;
    chrono::ChFramed frame;
    chrono::ChVector3d p1;
    chrono::ChVector3d p2;
    double radius;
    chrono::ChLineSegment seg;
    chrono::ChMatrix33<> mr;
    double lx, ly, lz;

    // cache chassis collision shape
    std::vector<std::shared_ptr<chrono::ChCollisionShape>> chassis_shapes;
    std::vector<chrono::ChFramed> chassis_frames;

    // Add chassis collision shapes - Cylinders in chassis
    p1 = chrono::ChVector3d(-0.0207166379929781,0.0873285231663156,-0.124502371636842);
    p2 = chrono::ChVector3d(-0.0207166379929781,-0.00267147683368442,-0.124502371636842);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.00477500133882154;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    p1 = chrono::ChVector3d(0.0974823413537155,0.0142285231663156,0.0890041315941103);
    p2 = chrono::ChVector3d(0.0974823413537155,0.0332285231663156,0.0890041315941103);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.01675;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    p1 = chrono::ChVector3d(0.0974823413537159,0.0142285231663156,-0.104800397925332);
    p2 = chrono::ChVector3d(0.0974823413537159,0.0332285231663156,-0.104800397925332);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.01675;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    p1 = chrono::ChVector3d(-0.123948051262223,0.0142285231663156,-0.104800397925332);
    p2 = chrono::ChVector3d(-0.123948051262223,0.0332285231663156,-0.104800397925332);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.01675;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    p1 = chrono::ChVector3d(-0.123948051262223,0.0142285231663156,0.0890041315941103);
    p2 = chrono::ChVector3d(-0.123948051262223,0.0332285231663156,0.0890041315941103);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.01675;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    // Add chassis collision shapes - Boxes in chassis
    mr(0,0)=1.84296331487336E-14; mr(1,0)=-1.02386850826298E-15; mr(2,0)=-1;
    mr(0,1)=0;                    mr(1,1)=1;                     mr(2,1)=0;
    mr(0,2)=1;                    mr(1,2)=0;                     mr(2,2)=1.84296331487336E-14;
    lx = 0.0271085353164314; ly = 0.133753023527107; lz = 0.5;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat, lx, ly, lz);
    frame = chrono::ChFramed(chrono::ChVector3d(-0.0132433308801697,-0.060647988597238,-0.104800397925318), mr);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    mr(0,0)=1;                   mr(1,0)=0;  mr(2,0)=0;
    mr(0,1)=1.2772170934726E-16; mr(1,1)=0;  mr(2,1)=1;
    mr(0,2)=0;                   mr(1,2)=-1; mr(2,2)=0;
    lx = 0.05; ly = 0.162984678314347; lz = 0.145863655604698;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat, lx, ly, lz);
    frame = chrono::ChFramed(chrono::ChVector3d(-0.0132433308801698,0.106660350968665,-0.0235768233650008), mr);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    mr(0,0)=2.43464271261192E-15; mr(1,0)=0;  mr(2,0)=-1;
    mr(0,1)=1;                    mr(1,1)=0;  mr(2,1)=2.26353909385586E-16;
    mr(0,2)=0;                    mr(1,2)=-1; mr(2,2)=0;
    lx = 0.193804529519442; ly = 0.183930392615939; lz = 0.0195;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat, lx, ly, lz);
    frame = chrono::ChFramed(chrono::ChVector3d(-0.0132328549542535,0.0234785231663156,-0.00789813316561091), mr);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    mr(0,0)=-1;                   mr(1,0)=0; mr(2,0)=-1.78428700386186E-14;
    mr(0,1)=0;                    mr(1,1)=1; mr(2,1)=0;
    mr(0,2)=1.78428700386186E-14; mr(1,2)=0; mr(2,2)=-1;
    lx = 0.014; ly = 0.03; lz = 0.166695994202991;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat, lx, ly, lz);
    frame = chrono::ChFramed(chrono::ChVector3d(-0.123958527380171,-0.0520745003610669,-0.00789813316561051), mr);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    mr(0,0)=1;                     mr(1,0)=0;                     mr(2,0)=1.58603289232159E-14;
    mr(0,1)=0;                     mr(1,1)=1;                     mr(2,1)=3.70074341541719E-15;
    mr(0,2)=-1.58603289232159E-14; mr(1,2)=-3.70074341541719E-15; mr(2,2)=1;
    lx = 0.0140000000000006; ly = 0.03; lz = 0.166695994202991;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat, lx, ly, lz);
    frame = chrono::ChFramed(chrono::ChVector3d(0.0974718656198289,-0.0520745003610672,-0.0078981331656067), mr);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    mr(0,0)=-4.15027262692179E-16; mr(1,0)=-1;                    mr(2,0)=0;
    mr(0,1)=1.63818961322091E-14;  mr(1,1)=0;                     mr(2,1)=-1;
    mr(0,2)=1;                     mr(1,2)=-4.15027262692179E-16; mr(2,2)=1.63818961322091E-14;
    lx = 0.133753023527107; ly = 0.0271085353164291; lz = 0.5;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat, lx, ly, lz);
    frame = chrono::ChFramed(chrono::ChVector3d(-0.0132433308801731,-0.060647988597238,0.0890041315941014), mr);

    chassis_shapes.push_back(colshape); chassis_frames.push_back(frame);

    // Add the chassis collsion shapes to the framework
    AddChassisCollisionShapes(chassis_shapes, chassis_frames);

    // cache chassis collision shape
    std::array<std::vector<std::shared_ptr<chrono::ChCollisionShape>>, 4> propeller_shapes;
    std::array<std::vector<chrono::ChFramed>, 4> propeller_frames;

    // First propeller collision shape
    p1 = chrono::ChVector3d(0,0,0.00362499999999999);
    p2 = chrono::ChVector3d(0,0,-0.00362499999999999);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.085;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    propeller_shapes[0].push_back(colshape); propeller_frames[0].push_back(frame);

    // Second propeller collision shape
    p1 = chrono::ChVector3d(0,0,0.00362499999999999);
    p2 = chrono::ChVector3d(0,0,-0.00362499999999999);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.085;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    propeller_shapes[1].push_back(colshape); propeller_frames[1].push_back(frame);

    // Third propeller collision shape
    p1 = chrono::ChVector3d(0,0,-0.030875);
    p2 = chrono::ChVector3d(0,0,-0.023625);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.085;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    propeller_shapes[2].push_back(colshape); propeller_frames[2].push_back(frame);

    // Fourth propeller collision shape
    p1 = chrono::ChVector3d(0,0,-0.030875);
    p2 = chrono::ChVector3d(0,0,-0.023625);
    seg.pA = p1; seg.pB = p2; height = seg.GetLength(); frame = seg.GetFrame(); radius = 0.085;
    colshape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(mat, radius, height);

    propeller_shapes[3].push_back(colshape); propeller_frames[3].push_back(frame);

    // Add the chassis collsion shapes to the framework
    AddChassisCollisionShapes(propeller_shapes, propeller_frames);

}


}   // namespace _qrbp_
}   // namespace _uav_
}   // namespace _platforms_