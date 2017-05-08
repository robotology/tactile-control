# Copyright: 2015 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Massimo Regoli
# email:   massimo.regoli@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 


set(MODULENAME stableGrasp)

# Search for source code.
set(INC_HEADERS

    include/tactileControlWrapper/HandController.h
    include/tactileControlWrapper/HandControllerWrapper.h
    include/tactileControlWrapper/Enums.h
    include/tactileControlWrapper/RPCData.h
    include/tactileControlWrapper/RPCUtil.h
	
)

set(INC_SOURCES

    main.cpp
    HandControllerWrapper.cpp
    RPCData.cpp
    RPCUtil.cpp

)

source_group("Source Files" FILES ${INC_SOURCES})
source_group("Header Files" FILES ${INC_HEADERS})

# Include directories
include_Directories(${ctrlLib_INCLUDE_DIRS} ${skinDynLib_INCLUDE_DIRS})
include_directories(include ${iKin_INCLUDE_DIRS})

add_executable(${MODULENAME} ${INC_SOURCES} ${INC_HEADERS})
target_link_libraries(${MODULENAME} ${YARP_LIBRARIES} ctrlLib skinDynLib iKin TactileControl)

if(WIN32)
    install(TARGETS ${MODULENAME} DESTINATION bin/${CMAKE_BUILD_TYPE})
else(WIN32)
    install(TARGETS ${MODULENAME} DESTINATION bin)
endif(WIN32)
