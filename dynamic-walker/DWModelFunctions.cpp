// Function definitions for dynamics walker tutorial
// Prasanna Sritharan, July 2018

#include <OpenSim/OpenSim.h>
#include "DWModelFunctions.h"

using namespace OpenSim;
using namespace SimTK;


// Create the dynamic walker model
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
	osimModel.setGravity(Vec3(0, -0.980665, 0));


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
	String coordnames[6] = { "pelvis_rx", "pelvis_ry", "pelvis_rz", "pelvis_tx", "pelvis_ty", "pelvis_tz" };
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
	// CREATE THIGHS AND SHANKS

	// initialise parent segment
	OpenSim::Body* parentsegment = pelvis;

	// create and add segments
	double seglength;
	OpenSim::Body* legsegment[4];
	PinJoint* segPinJoint[4];
	String bodynames[4] = { "LeftThigh", "RightThigh", "LeftShank", "RightShank" };
	String jointnames[4] = {"LeftThighToPelvis","RightThighToPelvis","LeftShankToThigh","RightShankToThigh"};
	String pinnames[4] = { "LHip_rz", "RHip_rz", "LKnee_rz", "RKnee_rz" };
	double locparenty[6] = { 0.0, 0.0, -0.40 / 2, -0.40 / 2, -0.435 / 2,-0.435 / 2 };
	double locparentz[6] = { -pelvisWidth / 2, pelvisWidth / 2, 0.0, 0.0, 0.0, 0.0 };
	double locchildy[4] = { 0.40 / 2, 0.40 / 2, 0.435 / 2, 0.435 / 2 };
	double seglengths[4] = { 0.40, 0.40, 0.435, 0.435 };	// left thigh, right thigh, left shank, right shank
	double pinrangelow[4] = { -100.0, -100.0, -100.0, -100.0 };
	double pinrangeupp[4] = { 100.0, 100.0, 0.0, 0.0 };
	double pindefaults[4] = { -10.0, 30.0, -30.0, -30.0 };
	bool pindeflocked[4] = { false,false,true,true };
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
		locationInParent = Vec3(0.0, locparenty[j], locparentz[j]);
		locationInChild = Vec3(0.0, locchildy[j], 0.0);
		orientationInParent = Vec3(0.0, 0.0, 0.0);
		orientationInChild = Vec3(0.0, 0.0, 0.0);
		segPinJoint[j] = new PinJoint(pinnames[j], *parentsegment, locationInParent, orientationInParent, *legsegment[j], locationInChild, orientationInChild);

		// set joint properties, rotate about z-axis
		CoordinateSet& segpinCoords = segPinJoint[j]->upd_CoordinateSet();
		segpinCoords[0].setName(pinnames[j]);
		rotRangePlatform[0] = convertDegreesToRadians(pinrangelow[j]);
		rotRangePlatform[1] = convertDegreesToRadians(pinrangeupp[j]);
		segpinCoords[0].setRange(rotRangePlatform);
		segpinCoords[0].setDefaultValue(convertDegreesToRadians(pindefaults[j]));
		segpinCoords[0].setDefaultLocked(pindeflocked[j]);

		// add display geometry to pelvis
		legsegment[j]->addDisplayGeometry("sphere.vtp");
		legsegment[j]->updDisplayer()->setScaleFactors(Vec3(seglength / 10.0, seglength, seglength / 10.0));

		// add pelvis to model
		osimModel.addBody(legsegment[j]);

	}


	//*********************
	// ADD CONTACT GEOMETRY

	// create platform contact geometry
	ContactHalfSpace* platformContact = new ContactHalfSpace(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -Pi / 2), *platform, "PlatformContact");
	osimModel.addContactGeometry(platformContact);

	// joint contact spheres: initialise parent segment
	parentsegment = pelvis;

	// model contact spheres (hips and knees)
	double contactSphereRadius = 0.05;
	Vec3 jointlocinparent(0.0, 0.0, 0.0);
	ContactSphere* jointcontactspheres[6];
	String contactnames[6] = { "LHipContact", "RHipContact", "LKneeContact", "RKneeContact","LFootContact","RFootContact" };
	for (int j = 0; j < 6; j++) {
		if (j >= 2) parentsegment = legsegment[j - 2];
		jointlocinparent = Vec3(0.0, locparenty[j], locparentz[j]);
		jointcontactspheres[j] = new ContactSphere(contactSphereRadius, jointlocinparent, *parentsegment, contactnames[j]);
		osimModel.addContactGeometry(jointcontactspheres[j]);
	}


	//*********************
	// ADD HUNT-CROSSLEY FORCES

	// create contact model parameters
	double stiffness = 1E7, dissipation = 0.1;
	double staticFriction = 0.6, dynamicFriction = 0.4, viscosity = 0.01;
	
	// create and apply contact force models
	OpenSim::HuntCrossleyForce* jointContactForce[6];
	OpenSim::HuntCrossleyForce::ContactParameters* jointContactParameters[6];
	for (int j = 0; j < 6; j++) {
		
		// create contact parameters object
		jointContactParameters[j] = new OpenSim::HuntCrossleyForce::ContactParameters(stiffness, dissipation, staticFriction, dynamicFriction, viscosity);

		// select which contact geometries between which contact model is to be applied
		jointContactParameters[j]->addGeometry(contactnames[j]);
		jointContactParameters[j]->addGeometry("PlatformContact");

		// create the force model
		jointContactForce[j] = new OpenSim::HuntCrossleyForce(jointContactParameters[j]);

		// add force model to the OpenSim model
		osimModel.addForce(jointContactForce[j]);
	}
	

	//*********************'
	// ADD COORDINATE LIMITING FORCES

	// limiting force parameters
	double lstiffness = 1E6, ldamping = 1E5;
	double transition = 5; // degrees

	// create limiting force
	OpenSim::CoordinateLimitForce* jointLimitForce[4];
	for (int j = 0; j < 4; j++) {
		jointLimitForce[j] = new OpenSim::CoordinateLimitForce(pinnames[j], pinrangeupp[j], lstiffness, pinrangelow[j], lstiffness, ldamping, transition);
		osimModel.addForce(jointLimitForce[j]);
	}
	


	//*********************'
	// PRINT OSIM FILE AND RETURN

	// add a force reporter analysis to the model
	ForceReporter* reporter = new OpenSim::ForceReporter(&osimModel);
	osimModel.addAnalysis(reporter);

	// Save model to a file
	osimModel.print(modelname + ".osim");

	return osimModel;
}


// simulate the dynamic walker model
int simulateDynamicWalkerModel(OpenSim::Model osimModel, bool *isdeg, double *qs, double *dqs, double inittime, double finaltime) {

	// build and initialise the multibody system
	SimTK::State& si = osimModel.initSystem();

	// set the initial values for the system
	CoordinateSet& modelCoordinateSet = osimModel.updCoordinateSet();
	for (int i = 0; i < 11; i++) {
		if (isdeg[i] == true) {
			qs[i] = convertDegreesToRadians(qs[i]);
			dqs[i] = convertDegreesToRadians(dqs[i]);
		}
		modelCoordinateSet[i].setValue(si, qs[i]);
		modelCoordinateSet[i].setSpeedValue(si, dqs[i]);
	}

	// create the integrator
	SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
	integrator.setAccuracy(1.0E-8);

	// create the manager for the simulation
	Manager manager(osimModel, integrator);
	manager.setInitialTime(inittime);
	manager.setFinalTime(finaltime);
	
	// perform the simulation
	std::cout << "\n\nIntegrating from " << inittime << "to" << finaltime << std::endl;
	manager.integrate(si);

	// print states
	Storage statesOutput(manager.getStateStorage());
	statesOutput.print("DynamicWalker_states.sto");
	osimModel.updSimbodyEngine().convertRadiansToDegrees(statesOutput);
	statesOutput.setWriteSIMMHeader(true);
	statesOutput.print("DynamicWalker_states_degrees.sto");

	// print forces
	OpenSim::AnalysisSet analyses = osimModel.getAnalysisSet();
	analyses[0].printResults("DynamicWalker");


	return 0;
}