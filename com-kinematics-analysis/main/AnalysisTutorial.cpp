#include <OpenSim/OpenSim.h>
#include <COMKinematicsPlugin.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


int main()
{
	try {

		// load model
		//Model osimModel("subject01_simbody_adjusted.osim");
		

		// create analysis object
		//int stepint = 1;
		//double starttime = 0.5, endtime = 4.0;
		//bool indegs = true, ison = true;
		//COMKinematicsPlugin* comReporter = new COMKinematicsPlugin();
		//comReporter->setOn(ison);
		//comReporter->setStartTime(starttime);
		//comReporter->setEndTime(endtime);
		//comReporter->setStepInterval(stepint);
		//comReporter->setInDegrees(indegs);
		
		// add to model
		//osimModel.addAnalysis(comReporter);		

		// initialise the model
		//SimTK::State& si = osimModel.initSystem();

		// create the analyser tool to manage the analysis
		//int outprec = 20;
		//double lpfreq = 6.0;
		AnalyzeTool* analyser = new AnalyzeTool("C:\\Users\\psritharan\\Documents\\03 Projects\\opensim-cpp-tutorials\\com-kinematics-analysis\\main\\build\\RelWithDebInfo\\subject01_Setup_AnalysisPluginTemplate.xml");
		
		// run the analysis
		analyser->run();


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