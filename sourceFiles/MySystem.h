#pragma once

#include "chrono/physics/ChSystemNSC.h"
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

	double actForce = 9.8f;

	ChSystemNSC system;

	double damping;
	double spring;

	std::shared_ptr<ChLinkTSDA> suspentionLink;
	std::shared_ptr<ChLinkMateSpherical> wheelAxisLink;
	std::shared_ptr<ChLinkMateSpherical> holdBodyRotationLink;
	std::shared_ptr<ChLinkMateSpherical> holdBodyTranslationLink;

	std::shared_ptr<ChForce> frc = chrono_types::make_shared<ChForce>();


	double bodyDensity = 100;
	double xDim = 1;
	double yDim = 1;
	double zDim = 1;

	double wheelDensity = 1000;
	double hWheelDim = 1;
	double rWheelDim = 1;

	double xFloorDim = 10;
	double yFloorDim = 0.1f;
	double zFloorDim = 10;

	double suspBase = 3;

	std::shared_ptr<ChBody> body;
	std::shared_ptr<ChBody> wheel;
	std::shared_ptr<ChBody> axis;
	std::shared_ptr<ChBody> floor;

public:

	MySystem(Document&);
	void AddSystem(ChSystemNSC&);

	void CreateFloor();

	void CreateWheel();

	void CreateBody();

	ChVector3d GetBodyPos();

	ChVector3d GetBodyPosRel();

	ChVector3d GetWheelPos();

	void SetWheelPos(ChVector3d);

	void SetWheelVel(double xVel);

	void UpdateActForce(double);

	void LinkBodies();

	void LinkSuspention();
	
};