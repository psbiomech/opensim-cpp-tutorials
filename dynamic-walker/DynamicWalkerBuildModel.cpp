/* OpenSim Dynamic walker tutorial
 * Implemented using additional functions defined externally, rather than
 * everything defined in the main() function as per the online solutions.
 *
 * Prasanna Sritharan, July 2018 */



//= == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == =
#include <OpenSim/OpenSim.h>
#include "DWModelFunctions.h"

// Set the namespace to shorten the declarations
// Note: Several classes appear in both namespaces and require using the full name
using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
*
*/

int main()
{
	try {

		// Model setup
		String modelname = "DynamicWalkerModel";
		Model osimModel = createDynamicWalkerModel(modelname);

		// Simulate model
		double inittime = 0.0, finaltime = 1.0;
		bool isdeg[11] = { true, true, true, true, false, false, false, true, true, true, true };
		double qs[11] = { -10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -10.0, 30.0, -30.0, -30.0 };
		double dqs[11] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		simulateDynamicWalkerModel(osimModel,isdeg,qs,dqs,inittime,finaltime);

		return 0;
	}
	catch (OpenSim::Exception ex)
	{
		std::cout << ex.getMessage() << std::endl;
		return 1;
	}
	catch (SimTK::Exception::Base ex)
	{
		std::cout << ex.getMessage() << std::endl;
		return 1;
	}
	catch (std::exception ex)
	{
		std::cout << ex.what() << std::endl;
		return 1;
	}
	catch (...)
	{
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		return 1;
	}
	std::cout << "OpenSim example completed successfully" << std::endl;
	std::cout << "Press return to continue" << std::endl;
	std::cin.get();
	return 0;
}