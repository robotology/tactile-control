# This is the "physical" (i.e. real) name of the library.
# The actual file name of the library built is constructed based on conventions
# of the native platform, such as lib<LIBRARY_TARGET_NAME>.a,
# lib<LIBRARY_TARGET_NAME>.dylib or <LIBRARY_TARGET_NAME>.lib.
set(LIBRARY_TARGET_NAME TactileControl)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(Gurls)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

add_definitions(${YARP_DEFINES})

#include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Gurls_INCLUDE_DIRS})

# List of CPP (source) library files
set(${LIBRARY_TARGET_NAME}_SRC
        src/HandController.cpp
		src/ObjectRecognitionManager.cpp
		
		src/data/TaskData.cpp
		src/data/GMMData.cpp
		src/data/MLData.cpp

		src/util/ControllerUtil.cpp
		src/util/PortUtil.cpp
		src/util/ICubUtil.cpp
		src/util/CommonUtil.cpp
		src/util/MLUtil.cpp
		
		src/thread/DataCollectionThread.cpp
		src/thread/TaskThread.cpp
		
		src/task/Task.cpp
		src/task/StepTask.cpp
		src/task/ApproachTask.cpp
		src/task/ControlTask.cpp
		
)

# List of HPP (header) library files
set(${LIBRARY_TARGET_NAME}_HDR
        include/TactileControl/HandController.h
		include/TactileControl/ObjectRecognitionManager.h
		
		include/TactileControl/data/TaskData.h
		include/TactileControl/data/GMMData.h
		include/TactileControl/data/Parameters.h
		include/TactileControl/data/Enums.h
		include/TactileControl/data/MLData.h
		
		include/TactileControl/util/ControllerUtil.h
		include/TactileControl/util/PortUtil.h
		include/TactileControl/util/ICubUtil.h
		include/TactileControl/util/CommonUtil.h
		include/TactileControl/util/MLUtil.h
		
		include/TactileControl/task/Task.h
		include/TactileControl/task/StepTask.h
		include/TactileControl/task/ApproachTask.h
		include/TactileControl/task/ControlTask.h
		
		include/TactileControl/thread/DataCollectionThread.h
		include/TactileControl/thread/TaskThread.h
		
)

# Adds a library target called ${LIBRARY_TARGET_NAME} to be built from the
# source and header files listed in the command invocation.
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

# Set two minimum target properties for the library.
# See https://cmake.org/cmake/help/latest/command/set_target_properties.html
# Properties are: 1) Library version number 2) list of corresponding public headers
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION})

														
target_link_libraries(${LIBRARY_TARGET_NAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Gurls_LIBRARIES})
# ctrlLib skinDynLib iKin ${Gurls_LIBRARIES})
														
														
# Specify include directories for both compilation and installation process.
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
							 ${YARP_INCLUDE_DIRS}
							 ${ICUB_INCLUDE_DIRS}
							 ${Gurls_INCLUDE_DIRS})

# Specify installation targets, typology and destination folders.
install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin)

install(DIRECTORY include/${LIBRARY_TARGET_NAME} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${LIBRARY_TARGET_NAME})
