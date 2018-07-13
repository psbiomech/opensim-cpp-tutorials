###########################################
## For building with the rest of OpenSim ##
###########################################

# Modified by Prasanna Sritharan, July 2018


cmake_minimum_required(VERSION 2.6)

# Define project
PROJECT (OptimizationExample)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors)

SET(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME} 
		CACHE PATH "Top-level directory of OpenSim install")

# Change name of build target
SET(TARGET optimizationExample CACHE TYPE STRING)
FILE(GLOB SOURCE_FILES *.h *.cpp)
SET(SOURCE ${SOURCE_FILES})

# To add Debug feature add ";Debug" after Release on the line below
SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release" 
      CACHE STRING "Semicolon separated list of supported configuration types, only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything else will be ignored." FORCE )
	
# Location of headers
SET(SIMTK_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
INCLUDE_DIRECTORIES(${SIMTK_HEADERS_DIR} ${OPENSIM_HEADERS_DIR})
# Libraries and dlls
SET(OPENSIM_LIBS_DIR ${OPENSIM_INSTALL_DIR}/sdk/lib ${OPENSIM_INSTALL_DIR}/lib)
SET(OPENSIM_DLLS_DIR ${OPENSIM_INSTALL_DIR}/bin)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR} ${OPENSIM_DLLS_DIR})

# Namespace
SET(NameSpace "OpenSim_" CACHE STRING "Prefix for simtk lib names, includes trailing '_'. Leave empty to use stock SimTK libraries.")
MARK_AS_ADVANCED(NameSpace)

# add other tutorial source files
set(TUTORIAL_FILES_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Folder containing additional source tutorial files.")
file(GLOB TUTORIAL_SOURCE_FILES ${TUTORIAL_FILES_FOLDER}/*.h ${TUTORIAL_FILES_FOLDER}/*.cpp)

# executable
ADD_EXECUTABLE(${TARGET} ${SOURCE} ${TUTORIAL_SOURCE_FILES})

TARGET_LINK_LIBRARIES(${TARGET}
	debug osimSimulation_d	optimized osimSimulation
	debug osimActuators_d	optimized osimActuators
	debug osimCommon_d		optimized osimCommon
	debug osimAnalyses_d	optimized osimAnalyses
	debug osimTools_d		optimized osimTools
	debug ${NameSpace}SimTKcommon_d optimized ${NameSpace}SimTKcommon
	debug ${NameSpace}SimTKmath_d optimized  ${NameSpace}SimTKmath
	debug ${NameSpace}SimTKsimbody_d optimized ${NameSpace}SimTKsimbody
)

# This block copies the additional files into the running directory
# For example vtp, obj files. Add to the end for more extentions
FILE(GLOB DATA_FILES *.vtp *.obj *.osim)
FOREACH (dataFile ${DATA_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET ${TARGET}
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OptimizationExample_BINARY_DIR})
 ENDFOREACH (dataFile) 

# set Visual Studio start-up project.
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${TARGET})
 
MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)
