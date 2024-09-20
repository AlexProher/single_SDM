
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/core/ChRandom.h"
#include "MySystem.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"


using namespace chrono;
using namespace rapidjson;

MySystem::MySystem() {

};

void MySystem::BuildConfig(Document& configFile) {
	std::cout << "1-DOF Sprung Mass with Damper Simulation\n";
	std::cout << "\n";

	xPos = configFile["Position"]["x"].GetDouble();
	yPos = configFile["Position"]["y"].GetDouble();
	zPos = configFile["Position"]["z"].GetDouble();

	wcGainMode = configFile["General"]["WorstGainCaseMode"].GetBool();
	if (wcGainMode) {
		std::cout << "WORST CASE GAIN MODE" << "\n" << "\n";
	}
	else {
		std::cout << "NOMINAL MODEL MODE" << "\n" << "\n";
	}

	if (configFile.HasMember("Wheel")) {
		rWheelDim = configFile["Wheel"]["rWheel"].GetDouble();
		std::cout << "r Wheel size - " << rWheelDim << " m" << "\n";

		hWheelDim = configFile["Wheel"]["hWheel"].GetDouble();
		std::cout << "h Wheel size - " << hWheelDim << " m" << "\n";
		wheelDensity = configFile["Wheel"]["density"].GetDouble();
	}

	if (configFile.HasMember("Motor")) {
		motorRotation = configFile["Motor"]["Speed"].GetDouble();
		std::cout << "Motor speed - " << motorRotation << " rad/s" << "\n" << "\n";
	}
	motorFunction = chrono_types::make_shared<ChFunctionConst>(motorRotation);

	if (configFile.HasMember("Floor")) {
		xFloorDim = configFile["Floor"]["x"].GetDouble();
		std::cout << "x Floor size - " << xFloorDim << " m" << "\n";

		yFloorDim = configFile["Floor"]["z"].GetDouble();
		std::cout << "y Floor size - " << xFloorDim << " m" << "\n";

		zFloorDim = configFile["Floor"]["z"].GetDouble();
		std::cout << "z Floor size - " << xFloorDim << " m" << "\n" << "\n";
	}

	if (configFile.HasMember("Body")) {
		xDim = configFile["Body"]["xSize"].GetDouble();
		std::cout << "x body size - " << xDim << " m" << "\n";

		yDim = configFile["Body"]["ySize"].GetDouble();
		std::cout << "y body size - " << yDim << " m" << "\n";

		zDim = configFile["Body"]["zSize"].GetDouble();
		std::cout << "z body size - " << zDim << " m" << "\n";
		if (wcGainMode) {
			bodyDensity = configFile["Body"]["density_wc"].GetDouble();
		}
		else {
			bodyDensity = configFile["Body"]["density_nominal"].GetDouble();
		}
		std::cout << "body density - " << bodyDensity << " kg/m3" << "\n" << "\n";
	}

	if (configFile.HasMember("SD")) {
		if (wcGainMode) {
			spring = configFile["SD"]["spring_wc"].GetDouble();
		}
		else {
			spring = configFile["SD"]["spring_nominal"].GetDouble();
		}
		std::cout << "spring - " << spring << " N/m" << "\n";
		if (wcGainMode) {
			damping = configFile["SD"]["damping_wc"].GetDouble();
		}
		else {
			damping = configFile["SD"]["damping_nominal"].GetDouble();
		}
		std::cout << "damping - " << damping << " Ns/m" << "\n";

		suspBase = configFile["SD"]["base"].GetDouble();
		std::cout << "base - " << suspBase << " m" << "\n" << "\n";
	}
}

void MySystem::CreateFloor() {
	auto floor_mat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto floor_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
	floor = chrono_types::make_shared<ChBodyEasyBox>(xFloorDim, yFloorDim, zFloorDim, 1, true, true, floor_mat);
	floor->SetPos(ChVector3d(0, -yFloorDim/2, 0));
	floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"), 100, 100);
	floor->SetFixed(true);
}

void MySystem::CreateWheel() {

	auto wheel_mat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto wheel_vis_mat = chrono_types::make_shared<ChVisualMaterial>();

	wheel = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis(2), rWheelDim, hWheelDim, wheelDensity, wheel_mat);
	wheel->SetPos(ChVector3d(xPos, yPos, zPos));
	wheel->EnableCollision(true);
	wheel->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/redwhite.png"));

	//wheel->SetFixed(true);
	axis = chrono_types::make_shared<ChBody>();
	axis->SetPos(ChVector3d(xPos, yPos, zPos));
}

void MySystem::CreateBody() {
	auto body_mat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto body_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
	body = chrono_types::make_shared<ChBodyEasyBox>(xDim, yDim, zDim, bodyDensity, true, true, body_mat);
	body->SetPos(ChVector3d(xPos, yPos + suspBase - rWheelDim, zPos));
	body->GetVisualShape(0)->SetColor(ChColor(0.8, 0.7, 0.7));
	body->AddForce(frcX);

}

void MySystem::LinkBodies() {

	wheelAxisLink = chrono_types::make_shared<ChLinkMateSpherical>();
	wheelAxisLink->Initialize(wheel, axis, false, wheel->GetPos(), axis->GetPos());
	wheelAxisLink->SetConstrainedCoords(true, true, true, true, true, false);

	holdBodyRotationLink = chrono_types::make_shared<ChLinkMateSpherical>();
	holdBodyRotationLink->Initialize(body, floor, false, body->GetPos(), floor->GetPos());
	holdBodyRotationLink->SetConstrainedCoords(false, false, false, true, true, true);

	holdBodyTranslationLink = chrono_types::make_shared<ChLinkMateSpherical>();
	holdBodyTranslationLink->Initialize(body, axis, false, body->GetPos(), axis->GetPos());
	holdBodyTranslationLink->SetConstrainedCoords(true, false, true, true, true, true);

}

void MySystem::LinkSuspention() {
	suspentionLink = chrono_types::make_shared<ChLinkTSDA>();
	suspentionLink->Initialize(body, axis, false, body->GetPos(), axis->GetPos());
	suspentionLink->SetSpringCoefficient(spring);
	suspentionLink->SetRestLength(suspBase-rWheelDim);
	suspentionLink->SetDampingCoefficient(damping);
	suspentionLink->SetActuatorForce(0);
}


void MySystem::CreateBrick(ChSystemNSC& sys, ChVector3d pos, double dimX, double dimY, double dimZ) {

	auto brick_mat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto brick_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
	auto brick = chrono_types::make_shared<ChBodyEasyBox>(dimX, dimY, dimZ, 1, true, true, brick_mat);
	brick->SetPos(pos);
	brick->SetFixed(true);
	brick->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/redwhite.png"));
	sys.AddBody(brick);
}

void MySystem::CreateBumper(ChSystemNSC& sys, ChVector3d pos, double r, double h) {
	auto bumper_mat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto bumper_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
	auto bumper = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis(2), r, h, 1, true, true, bumper_mat);
	bumper->SetPos(pos);
	bumper->SetFixed(true);
	bumper->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/redwhite.png"));
	sys.AddBody(bumper);
}

void MySystem::AddRandomCylinders(ChSystemNSC& sys, double posX, double dimMax, double N, double distFactor) {
	auto box_mat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto cyl_mat = chrono_types::make_shared<ChContactMaterialNSC>();

	// Create falling rigid bodies (spheres and boxes etc.)
	for (int bi = 0; bi < N; bi++) {
		double r = dimMax * ChRandom::Get();
		auto cylBody = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis(2),  //
			r,
			2,          // radius, height
			100,        // density
			cyl_mat     // contact material
		);
		cylBody->SetPos(ChVector3d(posX + double(bi) / distFactor + r, 0.2, 0));
		cylBody->SetFixed(true);
		cylBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/redwhite.png"));
		sys.Add(cylBody);
	}
}

ChVector3d MySystem::GetBodyPos() {
	return body->GetPos();
}

ChVector3d MySystem::GetBodyPosRel() {
	return suspentionLink->GetPoint2Rel();
}

ChVector3d MySystem::GetWheelPos() {
	return wheel->GetPos();
}

ChVector3d MySystem::GetWheelVel() {
	return wheel->GetLinVel();
}

ChVector3d MySystem::GetWheelAcc() {
	return wheel->GetLinAcc();
}

void MySystem::SetWheelPos(ChVector3d pos) {
	return wheel->SetPos(pos);
}

void MySystem::SetWheelVel(double xVel) {
	wheel->SetLinVel(ChVector3d(xVel, 0, 0));
}

void MySystem::UpdateActForce(double controlForce) {
	suspentionLink->SetActuatorForce(actForce * body->GetMass() + controlForce);
}

void MySystem::CreateMotor() {

	auto linkPos = wheel->GetPos();
	motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
	motor->Initialize(wheel, axis, ChFrame<>(linkPos, QuatFromAngleZ(CH_PI_2)));
	motor->SetSpeedFunction(motorFunction);
}

ChVector3d MySystem::getWheelContactForce() {
	return wheel->GetContactForce();
}

void MySystem::AddSystem(ChSystemNSC& sys) {
	CreateFloor();

	sys.AddBody(floor);

	CreateWheel();
	sys.AddBody(wheel);

	CreateBody();
	sys.AddBody(body);
	sys.AddBody(axis);

	LinkBodies();

	CreateMotor();
	sys.Add(motor);

	LinkSuspention();

	sys.AddLink(wheelAxisLink);
	sys.AddLink(holdBodyRotationLink);
	sys.AddLink(holdBodyTranslationLink);
	sys.AddLink(suspentionLink);

}