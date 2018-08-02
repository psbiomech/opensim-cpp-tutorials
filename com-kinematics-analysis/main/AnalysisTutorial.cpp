/* OPENSIM ANALYSIS TUTORIAL - MAIN PROGRAM
 * Prasanna Sritharan, July 2018 
 */



#include <OpenSim/OpenSim.h>
#include <COMKinematicsPlugin.h>
//#include <RegisterTypes_osimPlugin.h> // not required when creating an instance of COMKinematicsPlugin

using namespace OpenSim;
using namespace SimTK;
using namespace std;


int main()
{
	try {
		

		//*********************
		// OPTION 1: BUILD ANALYSIS (WITH MODEL AND KINEMATICS FROM XML FILE)

		// load model from file
		Model osimModel("C:\\Users\\psritharan\\Documents\\03 Projects\\opensim-cpp-tutorials\\com-kinematics-analysis\\main\\test_data\\subject01_simbody_adjusted.osim");

		// load kinematic data
		Storage jangles("C:\\Users\\psritharan\\Documents\\03 Projects\\opensim-cpp-tutorials\\com-kinematics-analysis\\main\\test_data\\subject01_walk1_ik.mot");

		
		// analysis parameters
		double stime = 0.5, ftime = 4.0;
		bool  onflag = TRUE, indeg = TRUE;
		int stepint = 1;
		String aname = "COMKinematicsPlugin", bodnames = "all";
						
		// create analysis
		COMKinematicsPlugin comkinal;
		comkinal.setName(aname);
		comkinal.setOn(onflag);
		comkinal.setStartTime(stime);
		comkinal.setEndTime(stime);
		comkinal.setStepInterval(stepint);
		comkinal.append_body_names(bodnames);
		osimModel.addAnalysis(&comkinal);

		// analysis tool parameters
		String toolname = "subject01";
		String resdir = "Results";
		int outprec = 20;
		double lpass = 6.0;

		// create analysis tool
		AnalyzeTool analyser;
		analyser.setName(toolname);
		analyser.setModel(osimModel);
		analyser.setResultsDir(resdir);
		analyser.setStartTime(stime);
		analyser.setFinalTime(ftime);
		analyser.setStatesStorage(jangles);

		// initialise the model
		SimTK::State& si = osimModel.initSystem();
		
		// set initial model state from motion
		analyser.setStatesFromMotion(si, jangles, TRUE);

		// run the analysis
		analyser.run();




		//*********************
		// OPTION 2: LOAD ANALYSIS FROM XML FILE

		// register type (not required when creating an instance of COMKinematicsPlugin)		
		//RegisterTypes_osimPlugin();	

		// create the analyser tool to manage the analysis
		//AnalyzeTool analyser("C:\\Users\\psritharan\\Documents\\03 Projects\\opensim-cpp-tutorials\\com-kinematics-analysis\\main\\test_data\\subject01_Setup_COMKinematicsPlugin.xml",true);
		
		// run the analysis
		//analyser.run();		




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