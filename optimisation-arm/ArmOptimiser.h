#ifndef _armopt_
#define _armopt_

#include <OpenSim/OpenSim.h>

// the cost function for optimisation (OptimizerSystem is the class that defines an objective function)
class ExampleOptimizationSystem : public SimTK::OptimizerSystem {

// parameters
private:
	int numControls;
	SimTK::State& si;
	OpenSim::Model& osimModel;
	double itime;
	double ftime;
	mutable int steps;
	mutable SimTK::Real bestf;	

// methods
public:
	ExampleOptimizationSystem(int numParameters, SimTK::State& s, OpenSim::Model& aModel, int stepCount, double bestSoFar, double initialTime, double finalTime);
	int objectiveFunc(const SimTK::Vector &newControls, const bool new_coefficients, SimTK::Real& f) const;
	
	// Not used for this tutorial (see API documentation):
	// int gradientFunc(...);
	// int constraintFunc(...);
	// int constraintJacobian(...);


};


#endif 