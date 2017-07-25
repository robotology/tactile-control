#include "TactileControl/HandController.h"

#include "TactileControl/data/Parameters.h"

#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>

using tactileControl::HandController;

HandController::HandController(){

    controllerInitialized = false;
    taskRunning = false;

    dbgTag = "HandController: ";
}

bool HandController::open(){

    // initialize data if not already initialized
    if (!settingsLoaded){
        yarp::os::Property options;
        if (!set(options)){
            return false;
        }
    }

    // initialize motor interfaces
    controllerUtil = new ControllerUtil();
    if (!controllerUtil->init(taskData)){
        yError() << dbgTag << "failed to initialize motor interfaces";
        return false;
    }

    // initialize yarp ports
    portUtil = new PortUtil();
    if (!portUtil->init(taskData)){
        yError() << dbgTag << "failed to initialize yarp ports";
        return false;
    }

    // start task thread
    taskThread = new TaskThread(taskData->getInt(PAR_COMMON_TASK_THREAD_PERIOD), taskData, controllerUtil, portUtil);
    if (!taskThread->start()) {
        yError() << dbgTag << "could not start the task thread";
        return false;
    }
    taskThread->suspend();

    // start data collection thread
    dataCollectionThread = new DataCollectionThread(taskData->getInt(PAR_COMMON_DATA_COLLECTION_THREAD_PERIOD), taskData, controllerUtil, portUtil);
    if (!dataCollectionThread->start()) {
        yError() << dbgTag << "could not start data collection thread";
        return false;
    }

    controllerInitialized = true;

    return true;

}

bool HandController::set(const yarp::os::Property &options){

    // initialize data
    taskData = new TaskData();
    if (!taskData->init(options)){
        yError() << dbgTag << "failed to initialize data";
        return false;
    }

    settingsLoaded = true;

    return true;
}

bool HandController::set(std::string context, std::string configFile){
    using yarp::os::ResourceFinder;
    using yarp::os::Bottle;

    // load main configuration file
    ResourceFinder mainConfigFileRF;
    mainConfigFileRF.setDefaultContext(context.c_str());
    mainConfigFileRF.setDefaultConfigFile((configFile + ".ini").c_str());
    char **fakeArgV;
    mainConfigFileRF.configure(0, fakeArgV, false);

    std::string icub = mainConfigFileRF.find(PAR_COMMON_ICUB).asString();
    std::string hand = mainConfigFileRF.find(PAR_COMMON_HAND).asString();

    // load icub-specific configuration file
    ResourceFinder iCubConfigFileRF;
    iCubConfigFileRF.setDefaultContext(context.c_str());
    iCubConfigFileRF.setDefaultConfigFile((configFile + "_" + icub + ".ini").c_str());
    iCubConfigFileRF.configure(0, fakeArgV, false);

    // get hand-specific settings
    Bottle &iCubHandSpecificData = iCubConfigFileRF.findGroup(hand + "Hand");

    // put all settings into Property object
    yarp::os::Property options;
    options.fromString(mainConfigFileRF.toString() + " " + iCubHandSpecificData.toString());

    return set(options);
}



bool HandController::set(const yarp::os::ConstString &key,const yarp::os::Value &value){

    if (!settingsLoaded) return false;

    return taskData->set(key,value);
}

bool HandController::get(const yarp::os::ConstString &key,yarp::os::Value &value){

    if (!settingsLoaded) return false;

    return taskData->get(key, value);
 }

bool HandController::closeHand(bool wait){

    if (!controllerInitialized) return false;

    if (taskRunning == true){

        return false;

    } else {

        taskThread->addApproachTask();
        taskThread->addControlTask();
        bool success = startTask();

        if (wait == true){
            success = waitForGraspStabilization(15,1);
        }

        return success;
    }
}

bool HandController::isHandClose(){

    if (!controllerInitialized) return false;

    return taskData->graspIsStable;
}

bool HandController::openHand(bool fullyOpen, bool wait){

    if (!controllerInitialized) return false;

    if (taskRunning == true){

        taskRunning = false;
        taskThread->suspend();
        return taskThread->afterRun(fullyOpen,wait);

    } else {

        return controllerUtil->openHand(fullyOpen,wait);
    }

}

bool HandController::isHandOpen(){

    if (!controllerInitialized) return false;

    return controllerUtil->isMotionDone();
}

bool HandController::setMinForce(double minForce){

    if (!controllerInitialized) return false;

    taskData->set(PAR_CTRL_MIN_FORCE_ENABLED, yarp::os::Value("true"));
    taskData->set(PAR_CTRL_MIN_FORCE,minForce);

    return true;
}

bool HandController::disableMinForce(){

    if (!controllerInitialized) return false;

    taskData->set(PAR_CTRL_MIN_FORCE_ENABLED, yarp::os::Value("false"));

    return true;
}

bool HandController::setGripStrength(double gripStrength){

    if (!controllerInitialized) return false;

    taskData->set(PAR_CTRL_GRIP_STRENGTH, gripStrength);

    return true;
}

bool HandController::close(){

    if (!controllerInitialized) return false;

    if (taskThread->isRunning()){
        taskThread->suspend();
    }
    if (dataCollectionThread->isRunning()){
        dataCollectionThread->suspend();
    }

    // stop threads
    taskThread->stop();
    dataCollectionThread->stop();

    taskData->release();

    // close controllers and ports
    controllerUtil->release();
    portUtil->release();

    delete(taskData);
    delete(controllerUtil);
    delete(portUtil);

    yInfo() << dbgTag << "hand controller succesfully closed";

    return true;
}

bool HandController::startTask(){

    if (!controllerInitialized) return false;

    if (taskRunning){

        return false;

    } else {

        if (!taskThread->initializeGrasping()) return false;
        taskRunning = true;
        taskThread->resume();
        return true;
    }
}


bool HandController::waitForGraspStabilization(double timeout, double delay){
    using yarp::os::Time;

    double startTime = Time::now();

    while (!taskData->graspIsStable && (Time::now() - startTime < timeout)) {
        Time::delay(delay);
    }

    return taskData->graspIsStable;
}


bool HandController::addStepTask(const std::vector<double> &targets){

    if (!controllerInitialized) return false;

    return taskThread->addStepTask(targets);
}

bool HandController::addApproachTask(){

    if (!controllerInitialized) return false;

    return taskThread->addApproachTask();
}

bool HandController::addControlTask(){

    if (!controllerInitialized) return false;

    return taskThread->addControlTask();
}
bool HandController::addControlTask(const std::vector<double> &targets){

    if (!controllerInitialized) return false;

    return taskThread->addControlTask(targets);
}

bool HandController::clearTaskList(){

    if (!controllerInitialized) return false;

    return taskThread->clearTaskList();
}

std::string HandController::getDataDescription(){

    if (!controllerInitialized) return "";

    return taskData->getDataDescription();
}

std::string HandController::getTaskListDescription(){

    if (!controllerInitialized) return "";

    return taskThread->getTaskListDescription();
}
