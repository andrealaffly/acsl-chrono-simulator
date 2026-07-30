// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: D:\Virginia-Tech-PhD\PHD_research\PyChrono\UAV_CAD_Models\SQRBP_V1.8\SQRBP_V1.8\Subassemblies\Chrono_Assembly.SLDASM


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
#include "medianoQuad.h"


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

std::string shapes_dir = "medianoQuad_shapes/";

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
body_1->SetName("Frame_Combined-1");
body_1->SetPos(chrono::ChVector3d(0,0,0));
body_1->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_1->SetMass(1.27359544600205);
body_1->SetInertiaXX(chrono::ChVector3d(0.00654969371759306,0.00621558396568854,0.0102525161842417));
body_1->SetInertiaXY(chrono::ChVector3d(-1.21105971588835e-06,-9.27032947877171e-06,-2.16784001716213e-06));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.000979643148821178,0.000801422765113241,0.000453896450721421),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_1->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_1 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_1;
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=5.83579624654883E-17;
mr(0,1)=0; mr(1,1)=-1; mr(2,1)=-1.56852284371505E-16;
mr(0,2)=5.83579624654883E-17; mr(1,2)=-1.56852284371505E-16; mr(2,2)=1;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.237804529519369,0.26543039261599,0.137);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0.00120871121755825,0.000836515898205509,-0.00287856405321896), mr));
body_1->EnableCollision(true);

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("dp7x4x3-4");
body_2->SetPos(chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.035503564053219));
body_2->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_2->SetMass(0.00395402309271749);
body_2->SetInertiaXX(chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06));
body_2->SetInertiaXY(chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("dp7x4x3-2");
body_3->SetPos(chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.035503564053219));
body_3->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_3->SetMass(0.00395402309271749);
body_3->SetInertiaXX(chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06));
body_3->SetInertiaXY(chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("dp7x4x3-3");
body_4->SetPos(chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.035503564053219));
body_4->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_4->SetMass(0.00395402309271749);
body_4->SetInertiaXX(chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06));
body_4->SetInertiaXY(chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("dp7x4x3-1");
body_5->SetPos(chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.035503564053219));
body_5->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_5->SetMass(0.00395402309271749);
body_5->SetInertiaXX(chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06));
body_5->SetInertiaXY(chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_5);


// Mate constraint: Coincident1 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Frame_Combined-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: dp7x4x3-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.0971766624903463,0.108491147175327,-0.0318785640532189);
cB = chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0318785640532189);
dA = chrono::ChVector3d(1.20533813419052e-17,1.72292970257481e-17,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_5,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0971766624903463,0.108491147175327,-0.0318785640532189);
dA = chrono::ChVector3d(1.20533813419052e-17,1.72292970257481e-17,-1);
cB = chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0318785640532189);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident1");
linklist.push_back(link);


// Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Frame_Combined-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_5 , SW name: dp7x4x3-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0463785640532189);
dA = chrono::ChVector3d(9.86076131526265e-32,4.93038065763132e-32,-1);
cB = chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.039128564053219);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0463785640532189);
cB = chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.039128564053219);
dA = chrono::ChVector3d(9.86076131526265e-32,4.93038065763132e-32,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric1");
linklist.push_back(link);


// Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Frame_Combined-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_3 , SW name: dp7x4x3-2 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0463785640532189);
dA = chrono::ChVector3d(2.46519032881566e-32,-7.39557098644699e-32,-1);
cB = chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0391285640532189);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0463785640532189);
cB = chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0391285640532189);
dA = chrono::ChVector3d(2.46519032881566e-32,-7.39557098644699e-32,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric2");
linklist.push_back(link);


// Mate constraint: Coincident2 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Frame_Combined-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: dp7x4x3-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0926329885112532,-0.110812993896686,-0.0318785640532189);
cB = chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.031878564053219);
dA = chrono::ChVector3d(-1.72292970257482e-17,1.20533813419047e-17,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident2");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0926329885112532,-0.110812993896686,-0.0318785640532189);
dA = chrono::ChVector3d(-1.72292970257482e-17,1.20533813419047e-17,-1);
cB = chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.031878564053219);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident2");
linklist.push_back(link);


// Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Frame_Combined-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_4 , SW name: dp7x4x3-3 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0463785640532189);
dA = chrono::ChVector3d(6.16297582203915e-33,-7.39557098644699e-32,-1);
cB = chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0391285640532189);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0463785640532189);
cB = chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0391285640532189);
dA = chrono::ChVector3d(6.16297582203915e-33,-7.39557098644699e-32,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
linklist.push_back(link);


// Mate constraint: Coincident3 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Frame_Combined-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: dp7x4x3-3 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.100733027547883,-0.108044328872842,-0.0318785640532189);
cB = chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.031878564053219);
dA = chrono::ChVector3d(-1.20533813418501e-17,1.72292970257864e-17,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.100733027547883,-0.108044328872842,-0.0318785640532189);
dA = chrono::ChVector3d(-1.20533813418501e-17,1.72292970257864e-17,-1);
cB = chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.031878564053219);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident3");
linklist.push_back(link);


// Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Frame_Combined-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_2 , SW name: dp7x4x3-4 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0463785640532189);
dA = chrono::ChVector3d(6.16297582203915e-33,-7.39557098644699e-32,-1);
cB = chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.039128564053219);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0463785640532189);
cB = chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.039128564053219);
dA = chrono::ChVector3d(6.16297582203915e-33,-7.39557098644699e-32,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
linklist.push_back(link);


// Mate constraint: Coincident4 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Frame_Combined-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: dp7x4x3-4 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0931387230643319,0.10962483877249,-0.0318785640532189);
cB = chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0318785640532189);
dA = chrono::ChVector3d(-1.20533813418501e-17,1.72292970257864e-17,-1);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0931387230643319,0.10962483877249,-0.0318785640532189);
dA = chrono::ChVector3d(-1.20533813418501e-17,1.72292970257864e-17,-1);
cB = chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0318785640532189);
dB = chrono::ChVector3d(0,0,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident4");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("motor_1");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0981109759772428,0.111551712206201,-0.0463785640532189),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("motor_2");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0956935535421263,-0.10987868040979,-0.0463785640532189),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("motor_3");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.0981109759772452,-0.109878680409738,-0.0463785640532189),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("motor_4");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.095693553542128,0.111551712206248,-0.0463785640532189),chrono::ChQuaternion<>(1,0,0,0)));


} // end function
