#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;


// Create an empty model
OpenSim::Model createDynamicWalkerModel(SimTK::String modelname)
{
	
	//*********************
	// CREATE EMPTY MODEL
	
	// Define key model variables
	double pelvisWidth = 0.20, thighLength = 0.40, shakLength = 0.435;

	// Create OpenSim Model
	Model osimModel = Model();
	osimModel.setName(modelname);

	// Get a reference to the ground object
	OpenSim::Body& ground = osimModel.getGroundBody();

	// Define gravitational acceleration
	osimModel.setGravity(Vec3(0, -0.98665, 0));


	//*********************
	// ADD PLATFORM

	// mass and inertia properties
	double mass = 1;
	Vec3 comLocInBody(0.0, 0.0, 0.0);
	Inertia bodyInertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);

	// create platform
	OpenSim::Body* platform = new OpenSim::Body("Platform", mass, comLocInBody, bodyInertia);

	// connect platform to ground with a pin joint
	Vec3 locationInParent(0.0, 0.0, 0.0), locationInChild(0.0, 0.0, 0.0);
	Vec3 orientationInParent(0.0, 0.0, 0.0), orientationInChild(0.0, 0.0, 0.0);
	PinJoint* plaftormToGround = new PinJoint("PlatformToGround", ground, locationInParent, orientationInParent, *platform, locationInChild, orientationInChild);

	// set joint properties, rotate about z-axis
	CoordinateSet& platformCoords = plaftormToGround->upd_CoordinateSet();
	platformCoords[0].setName("platform_rz");
	double rotRangePlatform[2] = { -Pi / 2.0, 0 };
	platformCoords[0].setRange(rotRangePlatform);
	platformCoords[0].setDefaultValue(convertDegreesToRadians(-10.0));
	platformCoords[0].setDefaultLocked(true);

	// add display geometry to platform
	platform->addDisplayGeometry("box.vtp");
	platform->updDisplayer()->setScaleFactors(Vec3(1, 0.05, 1));

	// add platform to model
	osimModel.addBody(platform);


	//*********************
	// CREATE THE PELVIS

	// mass and inertia properties
	mass = 1;
	comLocInBody = Vec3(0.0, 0.0, 0.0);
	bodyInertia = Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);

	// create pelvis body
	OpenSim::Body* pelvis = new OpenSim::Body("Pelvis", mass, comLocInBody, bodyInertia);

	// connect pelvis to platform with a free joint
	locationInParent = Vec3(0.0, 0.0, 0.0);
	locationInChild = Vec3(0.0, 0.0, 0.0);
	orientationInParent = Vec3(0.0, 0.0, 0.0);
	orientationInChild = Vec3(0.0, 0.0, 0.0);
	FreeJoint* pelvisToPlatform = new FreeJoint("pelvisToPlatform", *platform, locationInParent, orientationInParent, *pelvis, locationInChild, orientationInChild);

	// set joint properties, rotate about z-axis
	CoordinateSet& pelvisJointCoords = pelvisToPlatform->upd_CoordinateSet();
	String coordnames[6] = { "pelvis_rx", "pelvis_ry", "pelvis_rz", "pelvis_tx", "pelvis_tx", "pelvis_tx" };
	double rangelower[6] = { -Pi, -Pi, -Pi, -10.0, -1.0, -1.0 };
	double rangeupper[6] = { Pi, Pi, Pi, 10.0, 2.0, 1.0 };
	double defvals[6] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
	bool deflocked[6] = {true,true,true,false,false,true};
	double rotRange[2] = { 0.0, 0.0 };
	for (int i = 0; i < 6; i++) {
		pelvisJointCoords[i].setName(coordnames[i]);
		rotRange[0] = rangelower[i];
		rotRange[1] = rangeupper[i];
		pelvisJointCoords[i].setRange(rotRange);
		pelvisJointCoords[i].setDefaultValue(defvals[i]);
		pelvisJointCoords[i].setDefaultLocked(deflocked[i]);
	}

	// add display geometry to pelvis
	pelvis->addDisplayGeometry("sphere.vtp");
	pelvis->updDisplayer()->setScaleFactors(Vec3(pelvisWidth/2.0,pelvisWidth/2.0,pelvisWidth));

	// add pelvis to model
	osimModel.addBody(pelvis);


	//*********************
	// CREATETHIGHS AND SHANKS





	// Save model to a file
	osimModel.print(modelname + ".osim");

	return osimModel;
}