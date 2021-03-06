#------------------------------------------------------------------------
# CMake file for building an OpenSim project
# Author: The OpenSim Team
#------------------------------------------------------------------------

# Modified by Prasanna Sritharan, July 2018



cmake_minimum_required(VERSION 2.6)
project(AnalysisTutorial)


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
set(TARGET AnalysisTutorial CACHE STRING "Name of the executable.")
set(SOURCE AnalysisTutorial.cpp CACHE STRING "Name of the source files.")

# plugin paths
set(PLUGIN_HEADER_PATH "${CMAKE_SOURCE_DIR}/plugin/include/" CACHE STRING "Folder containing plugin header files")
set(PLUGIN_LIBRARY_PATH "${CMAKE_SOURCE_DIR}/plugin/libraries/" CACHE STRING "Folder containing plugin .lib file")

# Include headers.
include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS} ${PLUGIN_HEADER_PATH})

# Define an executable to be created from the provided C++ source files.
add_executable(${TARGET} ${SOURCE})

# Link to libraries.
target_link_libraries(${TARGET} ${OPENSIMSIMBODY_LIBRARIES}  "${PLUGIN_LIBRARY_PATH}/COMKinematicsPlugin.lib")

# set Visual Studio start-up project.
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${TARGET})

# post-build copy DLLs
add_custom_command(
	TARGET ${TARGET} POST_BUILD

	# make bin folders
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/RelWithDebInfo/"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/Release/"
	
	# DLLs
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plugin/libraries/COMKinematicsPlugin.dll" "${CMAKE_BINARY_DIR}/RelWithDebInfo/"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/plugin/libraries/COMKinematicsPlugin.dll" "${CMAKE_BINARY_DIR}/Release/"
)


