// Cost function definitions for arm optimisation example
// Prasanna Sritharan, July 2018


#include <OpenSim/OpenSim.h>
#include "ArmOptimiser.h"


using namespace OpenSim;
using namespace SimTK;
using namespace std;


// constructor class, parameters accessed in objectiveFunc()
ExampleOptimizationSystem::ExampleOptimizationSystem(int numParameters, State& s, Model& aModel, int stepCount, double bestSoFar, double initialTime, double finalTime) :
numControls(numParameters), OptimizerSystem(numParameters), si(s), osimModel(aModel), steps(stepCount), bestf(bestSoFar), itime(initialTime), ftime(finalTime) {}



// objective function
int ExampleOptimizationSystem::objectiveFunc(const Vector &newControls, const bool new_coefficients, Real& f) const {

	// make a copy of the initial states
	State s = si;

	// update the controls
	osimModel.updDefaultControls() = newControls;

	// create an integrator for the simulation
	RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
	integrator.setAccuracy(1.0E-6);

	// create a manager to run the simulation
	Manager manager(osimModel, integrator);

	// integrate from start time to finish time
	manager.setInitialTime(itime);
	manager.setFinalTime(ftime);
	osimModel.getMultibodySystem().realize(s, Stage::Acceleration);
	manager.integrate(s);

	// calculate value of cost function (arm COM forward velocity to be maximised)
	Vec3 massCenter;
	Vec3 velocity;
	osimModel.getBodySet().get("r_ulna_radius_hand").getMassCenter(massCenter);
	osimModel.getMultibodySystem().realize(s, Stage::Velocity);
	osimModel.getSimbodyEngine().getVelocity(s, osimModel.getBodySet().get("r_ulna_radius_hand"), massCenter, velocity);
	f = -velocity[0];	// maximise by multiplying by -1
	steps++;

	// store and print intermediate results of an arbitrary time step
	if (steps == 23) {
		Storage statesDegrees(manager.getStateStorage());
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.print("Arm26_randomSample_states_degrees.sto");
	}
	// store results of first step (i.e. states at initial activiation)
	else if (steps == 1) {
		Storage statesDegrees(manager.getStateStorage());
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.print("Arm26_noActivation_states_degrees.sto");
	}

	// store new best result if found, and update best results
	if (f < bestf) {

		// update value
		bestf = f;

		// write best value and print to file
		Storage statesDegrees(manager.getStateStorage());
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.print("bestSoFar_states_degrees.sto");
		cout << "\nOptimization Step #: " << steps << " controls = " << newControls << " bestSoFar = " << f << endl;
	}

	
	

	return 0;
	
}
