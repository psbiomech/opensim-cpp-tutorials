#------------------------------------------------------------------------
# CMake file for building an OpenSim project
# Author: The OpenSim Team
#------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.6)
project(DynamicWalker)


# Detect the folder where OpenSim is installed.
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME} CACHE PATH "Path to OpenSim Install.")

# Use the FindOpenSim.cmake script to get OpenSim's include directories
# and libraries.
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
find_package(OpenSim MODULE)

# We override the default CMAKE_CONFIGURATION_TYPES to avoid
# building in the Debug configuration, which does not work with the
# OpenSim API distributed via the GUI.
set(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release" CACHE STRING "List of supported configuration types." FORCE)
		
# Setup Targets
set(TARGET DynamicWalkerBuild CACHE STRING "Name of the executable.")
set(SOURCE DynamicWalkerBuildModel.cpp CACHE STRING "Name of the source files.")
# Include OpenSim's and Simbody's headers.
include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS})
# Define an executable to be created from the provided C++ source files.
add_executable(${TARGET} ${SOURCE})

# Link to OpenSim and Simbody libraries.
target_link_libraries(${TARGET} ${OPENSIMSIMBODY_LIBRARIES})
