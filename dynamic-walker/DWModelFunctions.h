// Function declarations for dynamics walker tutorial
// Prasanna Sritharan, July 2018

#ifndef _createDWModel_h_
#define _createDWModel_h_

#include <OpenSim/OpenSim.h>

// dynamic walker model functions
OpenSim::Model createDynamicWalkerModel(SimTK::String);
int simulateDynamicWalkerModel(OpenSim::Model, bool* ,double*, double*, double, double);

#endif