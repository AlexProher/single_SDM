
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "MySystem.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"


using namespace chrono;
using namespace rapidjson;

MySystem::MySystem(Document& config) {

	std::cout << "Create MySystem\n";

	xPos = config["Position"]["x"].GetDouble();
	yPos = config["Position"]["y"].GetDouble();
	zPos = config["Position"]["z"].GetDouble();
	
	if (config.HasMember("Wheel")) {
		rWheelDim = config["Wheel"]["rWheel"].GetDouble();
		std::cout << "r Wheel size - " << rWheelDim << "\n";

		hWheelDim = config["Wheel"]["hWheel"].GetDouble();
		std::cout << "h Wheel size - " << hWheelDim << "\n";

		wheelDensity = config["Wheel"]["density"].GetDouble();
	}

	if (config.HasMember("Floor")) {
		xFloorDim = config["Floor"]["x"].GetDouble();
		std::cout << "x Floor size - " << xFloorDim << "\n";

		zFloorDim = config["Floor"]["z"].GetDouble();
		std::cout << "h Floor size - " << xFloorDim << "\n";
	}

	if (config.HasMember("Body")) {
		xDim = config["Body"]["xSize"].GetDouble();
		std::cout << "x body size - " << xDim << "\n";

		yDim = config["Body"]["ySize"].GetDouble();
		std::cout << "y body size - " << yDim << "\n";

		zDim = config["Body"]["zSize"].GetDouble();
		std::cout << "z body size - " << zDim << "\n";

		bodyDensity = config["Body"]["density"].GetDouble();
	}

	if (config.HasMember("SD")) {
		spring = config["SD"]["spring"].GetDouble();
		std::cout << "spring - " << spring << "\n";

		damping = config["SD"]["damping"].GetDouble();
		std::cout << "damping - " << damping << "\n";

		suspBase = config["SD"]["base"].GetDouble();
		std::cout << "base - " << suspBase << "\n";
	}

};

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


	axis = chrono_types::make_shared<ChBody>();
	axis->SetPos(ChVector3d(xPos, yPos, zPos));
}

void MySystem::CreateBody() {
	auto body_mat = chrono_types::make_shared<ChContactMaterialNSC>();
	auto body_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
	body = chrono_types::make_shared<ChBodyEasyBox>(xDim, yDim, zDim, bodyDensity, true, true, body_mat);
	body->SetPos(ChVector3d(xPos, yPos + suspBase, zPos));
	body->GetVisualShape(0)->SetColor(ChColor(0.8, 0.7, 0.7));

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
	suspentionLink->SetRestLength(suspBase);
	suspentionLink->SetDampingCoefficient(damping);
	suspentionLink->SetActuatorForce(actForce * body->GetMass());
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

void MySystem::SetWheelPos(ChVector3d pos) {
	return wheel->SetPos(pos);
}

void MySystem::SetWheelVel(double xVel) {
	wheel->SetLinVel(ChVector3d(xVel, 0, 0));
}

void MySystem::UpdateActForce(double controlForce) {
	suspentionLink->SetActuatorForce(actForce * body->GetMass() + controlForce);
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

	LinkSuspention();

	sys.AddLink(wheelAxisLink);
	sys.AddLink(holdBodyRotationLink);
	sys.AddLink(holdBodyTranslationLink);
	sys.AddLink(suspentionLink);

}