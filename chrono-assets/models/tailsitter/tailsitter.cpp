// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: D:\Dropbox\Projects\QRBP\SolidWorks_Files\BQRBP\BQRBP_V1.1\chrono_assembly.SLDASM


#include <string>
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "tailsitter.h"


/// Function to import Solidworks assembly directly into Chrono ChSystem.
void ImportSolidworksSystemCpp(chrono::ChSystem& system, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;
std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;
ImportSolidworksSystemCpp(bodylist, linklist, motfun_map);
for (auto& body : bodylist)
    system.Add(body);
for (auto& link : linklist)
    system.Add(link);
}


/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& bodylist, std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {

// Some global settings
double sphereswept_r = 0.001;
chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.003);
chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(0.002);

std::string shapes_dir = "tailsitter_shapes/";

// Prepare some data for later use
std::shared_ptr<chrono::ChVisualShapeModelFile> body_shape;
chrono::ChMatrix33<> mr;
std::shared_ptr<chrono::ChLinkBase> link;
chrono::ChVector3d cA;
chrono::ChVector3d cB;
chrono::ChVector3d dA;
chrono::ChVector3d dB;

// Assembly ground body
auto body_0 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_0->SetName("SLDW_GROUND");
body_0->SetFixed(true);
bodylist.push_back(body_0);

// Rigid body part
auto body_1 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_1->SetName("16x5.4-1");
body_1->SetPos(chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0472628319499406));
body_1->SetRot(chrono::ChQuaternion<>(2.53545399382762e-15,2.53545399382763e-15,-0.707106781186546,0.707106781186549));
body_1->SetMass(0.023114806244527);
body_1->SetInertiaXX(chrono::ChVector3d(0.000243481962218972,2.00887421444941e-06,0.000245256129385561));
body_1->SetInertiaXY(chrono::ChVector3d(1.27126215242116e-05,1.25226930519482e-14,8.53229755274596e-15));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.39798185223223e-09,-0.00151682657384863,1.1268864500665e-09),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("16x5.4-4");
body_2->SetPos(chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0472628319499379));
body_2->SetRot(chrono::ChQuaternion<>(2.53545399382762e-15,2.53545399382763e-15,-0.707106781186546,0.707106781186549));
body_2->SetMass(0.023114806244527);
body_2->SetInertiaXX(chrono::ChVector3d(0.000243481962218972,2.00887421444941e-06,0.000245256129385561));
body_2->SetInertiaXY(chrono::ChVector3d(1.27126215242116e-05,1.25226930519482e-14,8.53229755274596e-15));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.39798185223223e-09,-0.00151682657384863,1.1268864500665e-09),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("16x5.4-2");
body_3->SetPos(chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0472628319499344));
body_3->SetRot(chrono::ChQuaternion<>(2.53545399382762e-15,2.53545399382763e-15,-0.707106781186546,0.707106781186549));
body_3->SetMass(0.023114806244527);
body_3->SetInertiaXX(chrono::ChVector3d(0.000243481962218972,2.00887421444941e-06,0.000245256129385561));
body_3->SetInertiaXY(chrono::ChVector3d(1.27126215242116e-05,1.25226930519482e-14,8.53229755274596e-15));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.39798185223223e-09,-0.00151682657384863,1.1268864500665e-09),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("16x5.4-3");
body_4->SetPos(chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0472628319499391));
body_4->SetRot(chrono::ChQuaternion<>(2.53545399382762e-15,2.53545399382763e-15,-0.707106781186546,0.707106781186549));
body_4->SetMass(0.023114806244527);
body_4->SetInertiaXX(chrono::ChVector3d(0.000243481962218972,2.00887421444941e-06,0.000245256129385561));
body_4->SetInertiaXY(chrono::ChVector3d(1.27126215242116e-05,1.25226930519482e-14,8.53229755274596e-15));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.39798185223223e-09,-0.00151682657384863,1.1268864500665e-09),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("frame_assembly_as_part_combined_w_collision_zones-1");
body_5->SetPos(chrono::ChVector3d(-0.820174552834782,0.138326409126481,0.0275871680500605));
body_5->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_5->SetMass(1.54332953655354);
body_5->SetInertiaXX(chrono::ChVector3d(0.151016870974707,0.0512672876060956,0.187965807814355));
body_5->SetInertiaXY(chrono::ChVector3d(-5.06412826018973e-07,6.54535930503061e-06,1.71490800154151e-06));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.000260032975304549,-3.98043445694742e-05,0.0115331100824462),chrono::ChQuaternion<>(1,0,0,0)));
body_5->SetFixed(true);

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_5_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_5->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_5 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_5;
mr(0,0)=1; mr(1,0)=1.85037170770859E-15; mr(2,0)=0;
mr(0,1)=-1.09836294215862E-16; mr(1,1)=1.09836294215862E-14; mr(2,1)=-1;
mr(0,2)=-1.85037170770859E-15; mr(1,2)=1; mr(2,2)=1.09836294215862E-14;
collshape_5 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_5,0.06,0.252699490762868,1.5);
body_5->GetCollisionModel()->AddShape(collshape_5,chrono::ChFramed(chrono::ChVector3d(0.222750060533507,-6.86347572278478E-05,0.0529997453814312), mr));
mr(0,0)=-1; mr(1,0)=-5.55111512312549E-15; mr(2,0)=4.69531820831031E-14;
mr(0,1)=1.09836294215861E-16; mr(1,1)=-1.14229745984496E-14; mr(2,1)=1;
mr(0,2)=-5.55111512312495E-15; mr(1,2)=1; mr(2,2)=1.14229745984496E-14;
collshape_5 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_5,0.0600000000000032,0.25269949076287,1.5);
body_5->GetCollisionModel()->AddShape(collshape_5,chrono::ChFramed(chrono::ChVector3d(-0.222749939466497,-6.86347572295132E-05,0.0529997453814354), mr));
mr(0,0)=-1; mr(1,0)=-4.1646502831122E-16; mr(2,0)=0;
mr(0,1)=-1.80231010491098E-16; mr(1,1)=-7.20924041964391E-16; mr(2,1)=1;
mr(0,2)=-4.1646502831122E-16; mr(1,2)=1; mr(2,2)=7.20924041964391E-16;
collshape_5 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_5,0.199936900307179,0.0769999999999996,0.0760000000000003);
body_5->GetCollisionModel()->AddShape(collshape_5,chrono::ChFramed(chrono::ChVector3d(-6.05329052111353E-08,6.82461176663568E-05,-0.0169999999999997), mr));
mr(0,0)=-1; mr(1,0)=-2.18684361597377E-15; mr(2,0)=-1.1007112867068E-14;
mr(0,1)=0; mr(1,1)=3.26560016492296E-15; mr(2,1)=-1;
mr(0,2)=2.18684361597381E-15; mr(1,2)=-1; mr(2,2)=-3.26560016492296E-15;
collshape_5 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_5,0.380762146130002,0.0509962779529998,0.0100000000000001);
body_5->GetCollisionModel()->AddShape(collshape_5,chrono::ChFramed(chrono::ChVector3d(6.05357001302583E-08,-0.459068634757229,0.12935581309071), mr));
mr(0,0)=0; mr(1,0)=1.08853338830768E-14; mr(2,0)=-1;
mr(0,1)=1; mr(1,1)=2.18684361597378E-15; mr(2,1)=0;
mr(0,2)=2.18684361597378E-15; mr(1,2)=-1; mr(2,2)=-1.08853338830768E-14;
collshape_5 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_5,0.0509962779529988,0.38076214613,0.0100000000000002);
body_5->GetCollisionModel()->AddShape(collshape_5,chrono::ChFramed(chrono::ChVector3d(6.05356991865687E-08,0.458931365242771,0.129355813090654), mr));
body_5->EnableCollision(true);

bodylist.push_back(body_5);




// Mate constraint: Concentric1 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: frame_assembly_as_part_combined_w_collision_zones-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_1 , SW name: 16x5.4-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.597424492301444,0.397257774369086,-0.0507628319499394);
dA = chrono::ChVector3d(-3.50566874250047e-15,5.22275672758959e-15,-1);
cB = chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0487628319499406);
dB = chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.597424492301444,0.397257774369086,-0.0507628319499394);
cB = chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0487628319499406);
dA = chrono::ChVector3d(-3.50566874250047e-15,5.22275672758959e-15,-1);
dB = chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_5,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric1");
linklist.push_back(link);


// Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_5 , SW name: frame_assembly_as_part_combined_w_collision_zones-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_1 , SW name: 16x5.4-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.597424492301444,0.397257774369086,-0.0457628319499394);
cB = chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0457628319499406);
dA = chrono::ChVector3d(-3.50566874250047e-15,5.22275672758959e-15,-1);
dB = chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_1,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.597424492301444,0.397257774369086,-0.0457628319499394);
dA = chrono::ChVector3d(-3.50566874250047e-15,5.22275672758959e-15,-1);
cB = chrono::ChVector3d(-0.597424492301443,0.397257774369087,-0.0457628319499406);
dB = chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident1");
linklist.push_back(link);


// Mate constraint: Concentric2 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: frame_assembly_as_part_combined_w_collision_zones-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_3 , SW name: 16x5.4-2 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-1.04292449230144,-0.120742225630915,-0.0507628319499392);
dA = chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1);
cB = chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0487628319499344);
dB = chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-1.04292449230144,-0.120742225630915,-0.0507628319499392);
cB = chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0487628319499344);
dA = chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1);
dB = chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_5,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric2");
linklist.push_back(link);


// Mate constraint: Coincident2 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_5 , SW name: frame_assembly_as_part_combined_w_collision_zones-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: 16x5.4-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-1.04292449230144,-0.120742225630915,-0.0457628319499392);
cB = chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0457628319499344);
dA = chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1);
dB = chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-1.04292449230144,-0.120742225630915,-0.0457628319499392);
dA = chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1);
cB = chrono::ChVector3d(-1.04292449230144,-0.120742225630912,-0.0457628319499344);
dB = chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident2");
linklist.push_back(link);


// Mate constraint: Concentric3 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: frame_assembly_as_part_combined_w_collision_zones-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_4 , SW name: 16x5.4-3 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.597424492301445,-0.120742225630913,-0.0507628319499426);
dA = chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1);
cB = chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0487628319499391);
dB = chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.597424492301445,-0.120742225630913,-0.0507628319499426);
cB = chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0487628319499391);
dA = chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1);
dB = chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_5,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
linklist.push_back(link);


// Mate constraint: Coincident3 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_5 , SW name: frame_assembly_as_part_combined_w_collision_zones-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: 16x5.4-3 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.597424492301445,-0.120742225630913,-0.0457628319499426);
cB = chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0457628319499391);
dA = chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1);
dB = chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.597424492301445,-0.120742225630913,-0.0457628319499426);
dA = chrono::ChVector3d(-4.22391390242588e-15,1.08859376815227e-14,-1);
cB = chrono::ChVector3d(-0.597424492301445,-0.120742225630912,-0.0457628319499391);
dB = chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident3");
linklist.push_back(link);


// Mate constraint: Concentric4 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: frame_assembly_as_part_combined_w_collision_zones-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_2 , SW name: 16x5.4-4 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-1.04292449230144,0.397257774369086,-0.0507628319499354);
dA = chrono::ChVector3d(-4.22524492555908e-15,1.03617656841521e-14,-1);
cB = chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0487628319499379);
dB = chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-1.04292449230144,0.397257774369086,-0.0507628319499354);
cB = chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0487628319499379);
dA = chrono::ChVector3d(-4.22524492555908e-15,1.03617656841521e-14,-1);
dB = chrono::ChVector3d(-7.17134684968812e-15,-3.49148133884311e-15,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_5,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
linklist.push_back(link);


// Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_5 , SW name: frame_assembly_as_part_combined_w_collision_zones-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: 16x5.4-4 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-1.04292449230144,0.397257774369086,-0.0457628319499354);
cB = chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0457628319499379);
dA = chrono::ChVector3d(-4.22524492555908e-15,1.03617656841521e-14,-1);
dB = chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-1.04292449230144,0.397257774369086,-0.0457628319499354);
dA = chrono::ChVector3d(-4.22524492555908e-15,1.03617656841521e-14,-1);
cB = chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0457628319499379);
dB = chrono::ChVector3d(7.17134684968812e-15,3.49148133884311e-15,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident4");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("motor_1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.597424492301397,0.397257774369148,-0.0457628319499394),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("motor_2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-1.04292449230149,-0.120742225630677,-0.0457628319499405),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("motor_3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.597424492301483,-0.120742225630798,-0.0457628319499421),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("motor_4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-1.04292449230144,0.397257774369089,-0.0457628319499378),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_5 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_5->SetName("lower_wing_p1");
body_0->AddMarker(marker_0_5);
marker_0_5->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.597424490855427,0.888257774369254,0.0559371680500684),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_6 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_6->SetName("lower_wing_p2");
body_0->AddMarker(marker_0_6);
marker_0_6->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.597424490855419,-0.611742225630746,0.0559371680500521),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_7 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_7->SetName("upper_wing_p1");
body_0->AddMarker(marker_0_7);
marker_0_7->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-1.04292449230128,0.888257774369251,0.0559371680500718),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_8 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_8->SetName("upper_wing_p2");
body_0->AddMarker(marker_0_8);
marker_0_8->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-1.04292449230127,-0.611742225630749,0.0559371680500555),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_9 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_9->SetName("right_stab_p1");
body_0->AddMarker(marker_0_9);
marker_0_9->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.609793419234083,0.597257774369246,0.143944877372945),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_10 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_10->SetName("right_stab_p2");
body_0->AddMarker(marker_0_10);
marker_0_10->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-1.03055556536408,0.597257774369245,0.143944877372947),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_11 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_11->SetName("left_stab_p1");
body_0->AddMarker(marker_0_11);
marker_0_11->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.60979341923408,-0.320742225630725,0.143944877373001),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_12 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_12->SetName("left_stab_p2");
body_0->AddMarker(marker_0_12);
marker_0_12->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-1.03055556536408,-0.320742225630726,0.143944877373002),chrono::ChQuaternion<>(1,0,0,0)));


} // end function
