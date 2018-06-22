#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;


// Create an empty model
OpenSim::Model createDynamicWalkerModel(SimTK::String modelname)
{
	
	//*********************
	// CREATE EMPTY MODEL
	
	
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

	// length, mass and inertia properties
	double pelvisWidth = 0.20;
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
	// CREATE THIGHS AND SHANKS\


	// initialise parent segment
	OpenSim::Body* parentsegment = pelvis;

	// create and add segments
	double seglength;
	CoordinateSet segpinCoords;
	OpenSim::Body* legsegment[4];
	PinJoint* segPinJoint;
	String bodynames[4] = { "LeftThigh", "RightThigh", "LeftShank", "RightShank" };
	String jointnames[4] = {"LeftThighToPelvis","RightThighToPelvis","LeftShankToThigh","RightShankToThigh"};
	String pinnames[4] = { "LHip_rz", "RHip_rz", "LKnee_rz", "RKnee_rz" };
	double locparenty[4] = { 0.0, 0.0, -0.40 / 2, -0.40 / 2 };
	double locchildy[4] = { 0.40 / 2, 0.40 / 2, 0.435 / 2, 0.435 / 2 };
	double seglengths[4] = { 0.40, 0.40, 0.435, 0.435 };	// left thigh, right thigh, left shank, right shank
	double pinrangelow[4] = { -100, -100, 0.0, 0.0 };
	double pinrangeupp[4] = { 100, 100, 0.0, 0.0 };
	double pindefaults[4] = { -10.0, 30.0, -30.0, -30.0 };
	for (int j = 0; j < 4; j++) {

		// update parent segment
		if (j >= 2) parentsegment = legsegment[j - 2];

		// length, mass and inertia (mass and inertia common to all thigh and shank segments)
		seglength = seglengths[j];
		mass = 1;
		comLocInBody = Vec3(0.0, 0.0, 0.0);
		bodyInertia = Inertia(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);

		// create pelvis body
		legsegment[j] = new OpenSim::Body(bodynames[j], mass, comLocInBody, bodyInertia);		

		// connect pelvis to platform with a free joint
		locationInParent = Vec3(0.0, locparenty[j], 0.0);
		locationInChild = Vec3(0.0, locchildy[j], 0.0);
		orientationInParent = Vec3(0.0, 0.0, 0.0);
		orientationInChild = Vec3(0.0, 0.0, 0.0);
		segPinJoint = new PinJoint(pinnames[j], *parentsegment, locationInParent, orientationInParent, *legsegment[j], locationInChild, orientationInChild);

		// set joint properties, rotate about z-axis
		segpinCoords = segPinJoint->upd_CoordinateSet();
		segpinCoords[0].setName(pinnames[j]);
		rotRangePlatform[0] = convertDegreesToRadians(pinrangelow[j]);
		rotRangePlatform[1] = convertDegreesToRadians(pinrangeupp[j]);
		segpinCoords[0].setRange(rotRangePlatform);
		segpinCoords[0].setDefaultValue(convertDegreesToRadians(pindefaults[j]));
		segpinCoords[0].setDefaultLocked(true);

		// add display geometry to pelvis
		legsegment[j]->addDisplayGeometry("sphere.vtp");
		legsegment[j]->updDisplayer()->setScaleFactors(Vec3(seglength / 10.0, seglength, seglength / 10.0));

		// add pelvis to model
		osimModel.addBody(legsegment[j]);

	}


	// Save model to a file
	osimModel.print(modelname + ".osim");

	return osimModel;
}