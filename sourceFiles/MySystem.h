#pragma once

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


using namespace chrono;
using namespace rapidjson;

class MySystem {

private:

	double xPos;
	double yPos;
	double zPos;

	double wcGainMode = false;

	double actForce = 9.8f;

	ChSystemSMC system;

	double damping;
	double spring;

	std::shared_ptr<ChLinkTSDA> suspentionLink;
	std::shared_ptr<ChLinkMateSpherical> wheelAxisLink;
	std::shared_ptr<ChLinkMateSpherical> holdBodyRotationLink;
	std::shared_ptr<ChLinkMateSpherical> holdBodyTranslationLink;

	std::shared_ptr<ChLinkMotorRotationSpeed> motor;
	std::shared_ptr<ChFunctionConst> motorFunction;
	double motorRotation = -2;
	double motorAcc = 2;


	std::shared_ptr<ChForce> frcY = chrono_types::make_shared<ChForce>();
	std::shared_ptr<ChForce> frcX = chrono_types::make_shared<ChForce>();


	double bodyDensity = 100;
	double xDim = 1;
	double yDim = 1;
	double zDim = 1;

	double wheelDensity = 1000;
	double wheelYoungMod = 1e6;
	double wheelDamping = 1e3;
	double hWheelDim = 1;
	double rWheelDim = 1;
	bool isFixed = false;

	double xFloorDim = 10;
	double yFloorDim = 10;
	double zFloorDim = 10;
	double floorYoungMod = 1e8;
	double floorDensity = 1e4;

	double suspBase = 3;

	std::shared_ptr<ChBody> body;
	std::shared_ptr<ChBody> wheel;
	std::shared_ptr<ChBody> axis;
	std::shared_ptr<ChBody> floor;

public:

	MySystem();

	void BuildConfig(Document&);
	void AddSystem(ChSystemSMC&);

	void CreateFloor();

	void CreateWheel();

	void CreateBody();

	void CreateBrick(ChSystemSMC&, ChVector3d, double, double, double, double, double);
	void CreateBumper(ChSystemSMC&, ChVector3d, double, double, double, double);
	void AddRandomCylinders(ChSystemSMC&, double, double, double, double, double, double, double);

	ChVector3d GetBodyPos();

	ChVector3d GetBodyPosRel();

	ChVector3d GetWheelPos();

	ChVector3d GetWheelVel();

	ChVector3d GetWheelAcc();

	ChVector3d getWheelContactForce();

	void SetWheelPos(ChVector3d);

	void SetWheelVel(double xVel);

	void UpdateActForce(double);

	void CreateMotor();

	void LinkBodies();

	void LinkSuspention();
	
};