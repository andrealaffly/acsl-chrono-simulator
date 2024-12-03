// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in



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
#include "sqrbp_model.h"


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

std::string shapes_dir = "../assets/sqrbp-chrono-assets/export/";

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
body_1->SetName("Final_Assembly-1");
body_1->SetPos(chrono::ChVector3d(0.0171306785555524,-0.00832708181360941,-0.00785621720215227));
body_1->SetRot(chrono::ChQuaternion<>(-0.499999999999998,0.500000000000005,-0.500000000000002,0.499999999999995));
body_1->SetMass(2.09224219084586);
body_1->SetInertiaXX(chrono::ChVector3d(0.0221767221436968,0.0378698749614418,0.0324427018402532));
body_1->SetInertiaXY(chrono::ChVector3d(1.04305458836941e-06,-8.98410015148422e-06,0.00091943506900011));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-0.0132794988606019,-0.00580556028628224,-0.0144622284658121),chrono::ChQuaternion<>(1,0,0,0)));

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
chrono::ChVector3d p1_body_1(-0.0207166379929781,0.0873285231663156,-0.124502371636842);
chrono::ChVector3d p2_body_1(-0.0207166379929781,-0.00267147683368442,-0.124502371636842);
body_1->GetCollisionModel()->AddCylinder(mat_1,0.00477500133882154,p1_body_1,p2_body_1);
chrono::ChVector3d p1_body_2(0.0974823413537155,0.0142285231663156,0.0890041315941103);
chrono::ChVector3d p2_body_2(0.0974823413537155,0.0332285231663156,0.0890041315941103);
body_1->GetCollisionModel()->AddCylinder(mat_1,0.01675,p1_body_2,p2_body_2);
chrono::ChVector3d p1_body_3(0.0974823413537159,0.0142285231663156,-0.104800397925332);
chrono::ChVector3d p2_body_200(0.0974823413537159,0.0332285231663156,-0.104800397925332);
body_1->GetCollisionModel()->AddCylinder(mat_1,0.01675,p1_body_3,p2_body_200);
mr(0,0)=1.84296331487336E-14; mr(1,0)=-1.02386850826298E-15; mr(2,0)=-1;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=0;
mr(0,2)=1; mr(1,2)=0; mr(2,2)=1.84296331487336E-14;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0271085353164314,0.133753023527107,0.5);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(-0.0132433308801697,-0.060647988597238,-0.104800397925318), mr));
chrono::ChVector3d p1_body_4(-0.123948051262223,0.0142285231663156,-0.104800397925332);
chrono::ChVector3d p2_body_3(-0.123948051262223,0.0332285231663156,-0.104800397925332);
body_1->GetCollisionModel()->AddCylinder(mat_1,0.01675,p1_body_4,p2_body_3);
mr(0,0)=1; mr(1,0)=0; mr(2,0)=0;
mr(0,1)=1.2772170934726E-16; mr(1,1)=0; mr(2,1)=1;
mr(0,2)=0; mr(1,2)=-1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.05,0.162984678314347,0.145863655604698);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(-0.0132433308801698,0.106660350968665,-0.0235768233650008), mr));
mr(0,0)=2.43464271261192E-15; mr(1,0)=0; mr(2,0)=-1;
mr(0,1)=1; mr(1,1)=0; mr(2,1)=2.26353909385586E-16;
mr(0,2)=0; mr(1,2)=-1; mr(2,2)=0;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.193804529519442,0.183930392615939,0.0195);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(-0.0132328549542535,0.0234785231663156,-0.00789813316561091), mr));
mr(0,0)=-1; mr(1,0)=0; mr(2,0)=-1.78428700386186E-14;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=0;
mr(0,2)=1.78428700386186E-14; mr(1,2)=0; mr(2,2)=-1;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.014,0.03,0.166695994202991);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(-0.123958527380171,-0.0520745003610669,-0.00789813316561051), mr));
mr(0,0)=1; mr(1,0)=0; mr(2,0)=1.58603289232159E-14;
mr(0,1)=0; mr(1,1)=1; mr(2,1)=3.70074341541719E-15;
mr(0,2)=-1.58603289232159E-14; mr(1,2)=-3.70074341541719E-15; mr(2,2)=1;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.0140000000000006,0.03,0.166695994202991);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(0.0974718656198289,-0.0520745003610672,-0.0078981331656067), mr));
mr(0,0)=-4.15027262692179E-16; mr(1,0)=-1; mr(2,0)=0;
mr(0,1)=1.63818961322091E-14; mr(1,1)=0; mr(2,1)=-1;
mr(0,2)=1; mr(1,2)=-4.15027262692179E-16; mr(2,2)=1.63818961322091E-14;
collshape_1 = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat_1,0.133753023527107,0.0271085353164291,0.5);
body_1->GetCollisionModel()->AddShape(collshape_1,chrono::ChFramed(chrono::ChVector3d(-0.0132433308801731,-0.060647988597238,0.0890041315941014), mr));
chrono::ChVector3d p1_body_5(-0.123948051262223,0.0142285231663156,0.0890041315941103);
chrono::ChVector3d p2_body_4(-0.123948051262223,0.0332285231663156,0.0890041315941103);
body_1->GetCollisionModel()->AddCylinder(mat_1,0.01675,p1_body_5,p2_body_4);
body_1->EnableCollision(true);

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("blade_1-1");
body_2->SetPos(chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0452097403684691));
body_2->SetRot(chrono::ChQuaternion<>(0.106343145835906,6.94336572027546e-15,7.42590218399898e-16,0.994329490326885));
body_2->SetMass(0.00395402309271749);
body_2->SetInertiaXX(chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06));
body_2->SetInertiaXY(chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_2->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_2 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_2;
chrono::ChVector3d p1_body_25(0,0,0.00362499999999999);
chrono::ChVector3d p2_body_21(0,0,-0.00362499999999999);
body_2->GetCollisionModel()->AddCylinder(mat_2,0.085,p1_body_25,p2_body_21);
body_2->EnableCollision(true);

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("blade_2-1");
body_3->SetPos(chrono::ChVector3d(-0.0876697193697793,-0.105809423167326,-0.0452097403684664));
body_3->SetRot(chrono::ChQuaternion<>(-0.00835912077524803,6.98271870651446e-15,-5.83714283918076e-17,0.999965061939598));
body_3->SetMass(0.00395402309271749);
body_3->SetInertiaXX(chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06));
body_3->SetInertiaXY(chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_3->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_3 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_3;
chrono::ChVector3d p1_body_11(0,0,0.00362499999999999);
chrono::ChVector3d p2_body_31(0,0,-0.00362499999999999);
body_3->GetCollisionModel()->AddCylinder(mat_3,0.085,p1_body_11,p2_body_31);
body_3->EnableCollision(true);

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("blade_3-1");
body_4->SetPos(chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0179597403684691));
body_4->SetRot(chrono::ChQuaternion<>(0.856221155277496,-3.56222304048412e-15,6.02022298477362e-15,-0.516609459122914));
body_4->SetMass(0.00395402309271749);
body_4->SetInertiaXX(chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06));
body_4->SetInertiaXY(chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_4_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

// Collision Model
body_4->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

// Collision material
auto mat_4 = chrono_types::make_shared<chrono::ChContactMaterialNSC>();

// Collision shape
std::shared_ptr<chrono::ChCollisionShape> collshape_4;
chrono::ChVector3d p1_body_41(0,0,-0.030875);
chrono::ChVector3d p2_body_41(0,0,-0.023625);
body_4->GetCollisionModel()->AddCylinder(mat_4,0.085,p1_body_41,p2_body_41);
body_4->EnableCollision(true);

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("blade_4-1");
body_5->SetPos(chrono::ChVector3d(-0.0876697193697808,0.115620969448612,-0.0179597403684664));
body_5->SetRot(chrono::ChQuaternion<>(0.986223667061496,-1.09785465125519e-15,6.90848991757748e-15,-0.165417286060968));
body_5->SetMass(0.00395402309271749);
body_5->SetInertiaXX(chrono::ChVector3d(2.83955219476793e-06,2.84026926028925e-06,5.66580058802151e-06));
body_5->SetInertiaXY(chrono::ChVector3d(7.53921616260059e-10,-1.55407912210321e-11,-5.50513541652329e-11));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),chrono::ChQuaternion<>(1,0,0,0)));

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
chrono::ChVector3d p1_body_51(0,0,-0.030875);
chrono::ChVector3d p2_body_51(0,0,-0.023625);
body_5->GetCollisionModel()->AddCylinder(mat_5,0.085,p1_body_51,p2_body_51);
body_5->EnableCollision(true);

bodylist.push_back(body_5);




// Mate constraint: motor_1_conc [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: Final_Assembly-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_2 , SW name: blade_1-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0477424753953881);
dA = chrono::ChVector3d(1.39659253553725e-14,3.66822593832662e-29,1);
cB = chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0488347403684691);
dB = chrono::ChVector3d(1.39659253553726e-14,3.43647531836903e-29,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("motor_1_conc");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0477424753953881);
cB = chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0488347403684691);
dA = chrono::ChVector3d(1.39659253553725e-14,3.66822593832662e-29,1);
dB = chrono::ChVector3d(1.39659253553726e-14,3.43647531836903e-29,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("motor_1_conc");
linklist.push_back(link);


// Mate constraint: motor_1_coin [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Final_Assembly-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: blade_1-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.108756861720293,0.113786617911709,-0.0415847403684691);
cB = chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0415847403684691);
dA = chrono::ChVector3d(-1.39538719740306e-14,1.72292970257113e-17,-1);
dB = chrono::ChVector3d(1.39659253553726e-14,3.43647531836903e-29,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("motor_1_coin");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.108756861720293,0.113786617911709,-0.0415847403684691);
dA = chrono::ChVector3d(-1.39538719740306e-14,1.72292970257113e-17,-1);
cB = chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0415847403684691);
dB = chrono::ChVector3d(1.39659253553726e-14,3.43647531836903e-29,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("motor_1_coin");
linklist.push_back(link);


// Mate constraint: motor_2_conc [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_1 , SW name: Final_Assembly-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_3 , SW name: blade_2-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0876697193697794,-0.105809423167326,-0.0477424753953854);
dA = chrono::ChVector3d(1.39659253553725e-14,3.66822593832662e-29,1);
cB = chrono::ChVector3d(-0.0876697193697794,-0.105809423167326,-0.0488347403684664);
dB = chrono::ChVector3d(1.39659253553726e-14,4.2647792688511e-29,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("motor_2_conc");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.0876697193697794,-0.105809423167326,-0.0477424753953854);
cB = chrono::ChVector3d(-0.0876697193697794,-0.105809423167326,-0.0488347403684664);
dA = chrono::ChVector3d(1.39659253553725e-14,3.66822593832662e-29,1);
dB = chrono::ChVector3d(1.39659253553726e-14,4.2647792688511e-29,1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("motor_2_conc");
linklist.push_back(link);


// Mate constraint: motor_2_coin [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Final_Assembly-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: blade_2-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0850476677991472,-0.107643774704231,-0.0415847403684664);
cB = chrono::ChVector3d(-0.0876697193697793,-0.105809423167326,-0.0415847403684664);
dA = chrono::ChVector3d(-1.39538719740306e-14,1.72292970257108e-17,-1);
dB = chrono::ChVector3d(1.39659253553726e-14,4.2647792688511e-29,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("motor_2_coin");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0850476677991472,-0.107643774704231,-0.0415847403684664);
dA = chrono::ChVector3d(-1.39538719740306e-14,1.72292970257108e-17,-1);
cB = chrono::ChVector3d(-0.0876697193697793,-0.105809423167326,-0.0415847403684664);
dB = chrono::ChVector3d(1.39659253553726e-14,4.2647792688511e-29,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("motor_2_coin");
linklist.push_back(link);


// Mate constraint: motor_3_conc [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Final_Assembly-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_4 , SW name: blade_3-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0477424753953881);
dA = chrono::ChVector3d(1.39659253553725e-14,3.66822593832662e-29,1);
cB = chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0488347403684691);
dB = chrono::ChVector3d(-1.39898407945414e-14,1.20106825767623e-16,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("motor_3_conc");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0477424753953881);
cB = chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0488347403684691);
dA = chrono::ChVector3d(1.39659253553725e-14,3.66822593832662e-29,1);
dB = chrono::ChVector3d(-1.39898407945414e-14,1.20106825767623e-16,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("motor_3_conc");
linklist.push_back(link);


// Mate constraint: motor_3_coin [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Final_Assembly-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: blade_3-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(0.107969161686559,-0.108431474737962,-0.0415847403684691);
cB = chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0415847403684691);
dA = chrono::ChVector3d(-1.39486960583467e-14,1.20533813418135e-17,-1);
dB = chrono::ChVector3d(1.39898407945414e-14,-1.20106825767623e-16,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_4,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("motor_3_coin");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(0.107969161686559,-0.108431474737962,-0.0415847403684691);
dA = chrono::ChVector3d(-1.39486960583467e-14,1.20533813418135e-17,-1);
cB = chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0415847403684691);
dB = chrono::ChVector3d(1.39898407945414e-14,-1.20106825767623e-16,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("motor_3_coin");
linklist.push_back(link);


// Mate constraint: motor_4_conc [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Final_Assembly-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_5 , SW name: blade_4-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0876697193697813,0.115620969448612,-0.0477424753953854);
dA = chrono::ChVector3d(1.39659253553725e-14,3.66822593832662e-29,1);
cB = chrono::ChVector3d(-0.0876697193697813,0.115620969448612,-0.0488347403684664);
dB = chrono::ChVector3d(-1.39898407945414e-14,1.20106825767625e-16,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("motor_4_conc");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-0.0876697193697813,0.115620969448612,-0.0477424753953854);
cB = chrono::ChVector3d(-0.0876697193697813,0.115620969448612,-0.0488347403684664);
dA = chrono::ChVector3d(1.39659253553725e-14,3.66822593832662e-29,1);
dB = chrono::ChVector3d(-1.39898407945414e-14,1.20106825767625e-16,-1);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("motor_4_conc");
linklist.push_back(link);


// Mate constraint: motor_4_coin [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_1 , SW name: Final_Assembly-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: blade_4-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-0.0886331560866604,0.114343554209714,-0.0415847403684664);
cB = chrono::ChVector3d(-0.0876697193697812,0.115620969448612,-0.0415847403684664);
dA = chrono::ChVector3d(-1.39486960583467e-14,1.20533813418134e-17,-1);
dB = chrono::ChVector3d(1.39898407945414e-14,-1.20106825767625e-16,1);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_1,body_5,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("motor_4_coin");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-0.0886331560866604,0.114343554209714,-0.0415847403684664);
dA = chrono::ChVector3d(-1.39486960583467e-14,1.20533813418134e-17,-1);
cB = chrono::ChVector3d(-0.0876697193697812,0.115620969448612,-0.0415847403684664);
dB = chrono::ChVector3d(1.39898407945414e-14,-1.20106825767625e-16,1);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_1,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("motor_4_coin");
linklist.push_back(link);


// Auxiliary marker (coordinate system feature)
auto marker_0_1 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_1->SetName("motor_1_axis");
body_0->AddMarker(marker_0_1);
marker_0_1->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.106134810149661,0.115620969448614,-0.0477424753953881),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_2 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_2->SetName("motor_2_axis");
body_0->AddMarker(marker_0_2);
marker_0_2->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0876697193697794,-0.105809423167326,-0.0477424753953854),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_3 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_3->SetName("motor_3_axis");
body_0->AddMarker(marker_0_3);
marker_0_3->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(0.106134810149663,-0.105809423167324,-0.0477424753953881),chrono::ChQuaternion<>(1,0,0,0)));

// Auxiliary marker (coordinate system feature)
auto marker_0_4 = chrono_types::make_shared<chrono::ChMarker>();
marker_0_4->SetName("motor_4_axis");
body_0->AddMarker(marker_0_4);
marker_0_4->ImposeAbsoluteTransform(chrono::ChFramed(chrono::ChVector3d(-0.0876697193697813,0.115620969448612,-0.0477424753953854),chrono::ChQuaternion<>(1,0,0,0)));


} // end function
