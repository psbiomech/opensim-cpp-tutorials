#include <OpenSim/OpenSim.h>
#include <COMKinematicsPlugin.h>
#include <RegisterTypes_osimPlugin.h> // not required when creating an instance of COMKinematicsPlugin

using namespace OpenSim;
using namespace SimTK;
using namespace std;


int main()
{
	try {
		
		// register type (not required when creating an instance of COMKinematicsPlugin)		
		RegisterTypes_osimPlugin();	

		// create the analyser tool to manage the analysis
		AnalyzeTool analyser("C:\\Users\\psritharan\\Documents\\03 Projects\\opensim-cpp-tutorials\\com-kinematics-analysis\\main\\test_data\\subject01_Setup_AnalysisPluginTemplate.xml",true);
		
		// run the analysis
		analyser.run();		

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