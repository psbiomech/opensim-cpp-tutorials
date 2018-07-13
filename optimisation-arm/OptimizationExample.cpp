/* -------------------------------------------------------------------------- *
 *                     OpenSim:  OptimizationExample.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Samuel R. Hamner, Ajay Seth                                                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Optimisation example, implemented with cost function defined externally
 * rather than within the same file as the main() function.
 *
 * Prasanna Sritharan, July 2018 */


//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include "ArmOptimiser.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;


//______________________________________________________________________________
/**
 * Define an optimization problem that finds a set of muscle controls to maximize 
 * the forward velocity of the forearm/hand segment mass center. 
 */
int main()
{
	try {

		// optimiser parameters
		int stepCount = 0;
		double initialTime = 0.0;
		double finalTime = 0.25;
		double bestSoFar = Infinity;


		//********************
		// INITIALISE THE MODEL

		// create OpenSim model object from XML file
		Model osimModel("Arm26_Optimize.osim");

		// define initial states of muscles
		const Set<Muscle> &muscleSet = osimModel.getMuscles();
		for (int i = 0; i < muscleSet.getSize(); i++) {

			// get pointer to muscle fibre properties
			ActivationFiberLengthMuscle* mus = dynamic_cast<ActivationFiberLengthMuscle*> (&muscleSet[i]);

			//  set the state
			if (mus) {
				mus->setDefaultActivation(0.5);
				mus->setDefaultFiberLength(0.1);
			}

		}

		// initialise the system and get the current state
		State& si = osimModel.initSystem();

		// make sure the muscle states are in equilibrium
		osimModel.equilibrateMuscles(si);

		

		//********************
		// CREATE AND RUN THE OPTIMISER


		// get the number of controls
		int numControls = osimModel.getNumControls();

		// initialise our custom objective function
		ExampleOptimizationSystem sys(numControls, si, osimModel, stepCount, bestSoFar, initialTime, finalTime);
		
		// optimisation bounds
		Vector controls(numControls, 0.01);
		Vector lbounds(numControls, 0.01);
		Vector ubounds(numControls, 0.99);
		sys.setParameterLimits(lbounds, ubounds);

		// create the optimiser (LBFGSB: limited-memory Broyden-Fletcher-Goldfarb-Shanno algorithm, gradient descent, with simple bounds).
		Optimizer opt(sys, SimTK::LBFGSB);

		// specify settings for the optimiser
		opt.setConvergenceTolerance(0.05);
		opt.useNumericalGradient(true);
		opt.setMaxIterations(1000);
		opt.setLimitedMemoryHistory(500);

		// run the optimiser
		Real f = NaN;
		f = opt.optimize(controls);



		//********************
		// OUTPUT RESULTS


		// controls
		const Set<Actuator>& actuators = osimModel.getActuators();
		for (int i = 0; i < actuators.getSize(); ++i) {
			cout << actuators[i].getName() << " control value = " << controls[i] << endl;
		}

		// maximum forward velocity
		cout << "\nMaximum hand velocity = " << -f << "m/s" << endl;
		cout << "OpenSim example completed successfully.\n";

		// Dump out optimization results to a text file for testing
		ofstream ofile;
		ofile.open("Arm26_optimization_result.txt");
		for (int i = 0; i<actuators.getSize(); ++i){
			ofile << controls[i] << endl;
		}
		ofile << -f << endl;
		ofile.close();


	}
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
	
	// End of main() routine.
	return 0;
}
